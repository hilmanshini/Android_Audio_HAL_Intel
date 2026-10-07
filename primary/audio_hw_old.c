/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * ===========================================================================
 *  DEBUG BUILD - extremely verbose logging under logcat tag "AudioDBG"
 * ===========================================================================
 *  Behaviour is intentionally IDENTICAL to the stock HAL (known bugs are NOT
 *  fixed) so the logs show what really happens.
 *
 *  Collect:   adb logcat -G 16M
 *             adb logcat -b all -v threadtime -s AudioDBG:V > audiodbg.log
 *
 *  Log line format:   [ secs.usecs] #seq tTID function: message
 *  The timestamp is CLOCK_MONOTONIC, the same clock dmesg uses, so lines can
 *  be correlated with kernel messages.
 *
 *  Runtime knobs (setprop, no rebuild needed, re-read every heartbeat):
 *    debug.audiodbg.tick_ms       fast watcher period, default 100
 *    debug.audiodbg.hb_ms         heartbeat period, default 1000
 *    debug.audiodbg.full_s        full mixer dump period (s), default 30
 *    debug.audiodbg.poll          1 = watcher reads mixer + /proc (default)
 *                                 0 = watcher only prints HAL-internal state
 *    debug.audiodbg.io            1 = log every write/read (default)
 *                                 0 = only anomalies + 1 in 50
 *    debug.audiodbg.codec_events  1 = dump key codec nodes on events (def 1)
 *    debug.audiodbg.codec_hb      1 = dump key codec nodes every heartbeat
 *                                 (default 0)
 *
 *  OBSERVER EFFECT: reading mixer controls, /proc/asound/.../codec#0 and
 *  friends can wake the codec / DSP from runtime suspend. If you suspect an
 *  idle power-management problem, set debug.audiodbg.poll=0 and
 *  debug.audiodbg.codec_hb=0 and compare. The heartbeat always logs the PCI
 *  runtime_status first so you can see whether polling keeps it awake.
 * ===========================================================================
 */

#define LOG_TAG "AudioDBG"
#define LOG_NDEBUG 0
#include <sys/sysmacros.h> // for major(), minor()
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <log/log.h>
#include <cutils/properties.h>
#include <cutils/str_parms.h>

#include <hardware/audio.h>
#include <hardware/hardware.h>

#include <system/audio.h>

#include <linux/ioctl.h>
#include <sound/asound.h>
#include <tinyalsa/asoundlib.h>

#include <audio_utils/channels.h>
#include <audio_utils/resampler.h>
#include <audio_route.h>
#include <sys/stat.h>

#define PCM_CARD 0
#define PCM_CARD_DEFAULT 0
#define PCM_DEVICE 0

#define OUT_PERIOD_SIZE 1024
#define OUT_PERIOD_COUNT 4
#define OUT_SAMPLING_RATE 48000

#define IN_PERIOD_SIZE 1024 //default period size
#define IN_PERIOD_MS 10
#define IN_PERIOD_COUNT 4
#define IN_SAMPLING_RATE 48000

#define AUDIO_PARAMETER_HFP_ENABLE   "hfp_enable"
#define AUDIO_PARAMETER_BT_SCO       "BT_SCO"
#define AUDIO_BT_DRIVER_NAME         "btaudiosource"
#define SAMPLE_SIZE_IN_BYTES          2
#define SAMPLE_SIZE_IN_BYTES_STEREO   4

#define DBG_MAX_STREAMS 8
#define ULL(x) ((unsigned long long)(x))

//#define DEBUG_PCM_DUMP

#ifdef DEBUG_PCM_DUMP
// To enable dumps, explicitly create "/vendor/dump/" folder and reboot device
FILE *sco_call_write = NULL;
FILE *sco_call_write_remapped = NULL;
FILE *sco_call_write_bt = NULL;
FILE *sco_call_read = NULL;
FILE *sco_call_read_remapped = NULL;
FILE *sco_call_read_bt = NULL;
FILE *out_write_dump = NULL;
FILE *in_read_dump = NULL;
#endif

struct pcm_config pcm_config_out = {
        .channels = 2,
        .rate = OUT_SAMPLING_RATE,
        .period_size = OUT_PERIOD_SIZE,
        .period_count = OUT_PERIOD_COUNT,
        .format = PCM_FORMAT_S16_LE,
        .start_threshold = OUT_PERIOD_SIZE * OUT_PERIOD_COUNT,
};

struct pcm_config pcm_config_in = {
        .channels = 2,
        .rate = IN_SAMPLING_RATE,
        .period_size = IN_PERIOD_SIZE,
        .period_count = IN_PERIOD_COUNT,
        .format = PCM_FORMAT_S16_LE,
        .start_threshold = 1,
        .stop_threshold = (IN_PERIOD_SIZE * IN_PERIOD_COUNT),
};

//[ BT ALSA Card config
struct pcm_config bt_out_config = {
        .channels = 1,
        .rate = 8000,
        .period_size = 240,
        .period_count = 5,
        .start_threshold = 0,
        .stop_threshold = 0,
        .silence_threshold = 0,
        .silence_size = 0,
        .avail_min = 0
};

struct pcm_config bt_in_config = {
        .channels = 1,
        .rate = 8000,
        .period_size = 240,
        .period_count = 5,
        .start_threshold = 0,
        .stop_threshold = 0,
        .silence_threshold = 0,
        .silence_size = 0,
        .avail_min = 0
};

struct audio_device {
    struct audio_hw_device hw_device;

    pthread_mutex_t lock; /* see note below on mutex acquisition order */
    unsigned int out_device;
    unsigned int in_device;
    bool standby;
    bool mic_mute;
    struct audio_route *ar;

    int card;
    int cardc;
    struct stream_out *active_out;
    struct stream_in *active_in;

//[BT-HFP Voice Call
    bool is_hfp_call_active;
//BT-HFP Voice Call]

    bool in_needs_standby;
    bool out_needs_standby;

//[BT SCO VoIP Call
    bool in_sco_voip_call;
    int bt_card;
    struct resampler_itfe *voip_in_resampler;
    struct resampler_itfe *voip_out_resampler;
//BT SCO VoIP Call]

    /* ---------------- debug instrumentation ---------------- */
    volatile long lock_owner;            /* tid currently holding ->lock */
    volatile uint64_t lock_since_us;
    const char *volatile lock_fn;
    pthread_mutex_t dbg_lock;            /* protects outs[] / ins[] */
    struct stream_out *outs[DBG_MAX_STREAMS];
    struct stream_in *ins[DBG_MAX_STREAMS];
    uint32_t next_stream_id;
    int card_dbg;                        /* card used by the watcher */
    struct mixer *dbg_mixer;             /* our own read-only mixer handle */
    pthread_t dbg_thread;
    bool dbg_thread_started;
    volatile bool dbg_stop;
    uint32_t sel_count;
};

struct stream_out {
    struct audio_stream_out stream;

    pthread_mutex_t lock; /* see note below on mutex acquisition order */
    struct pcm *pcm;
    struct pcm_config *pcm_config;
    struct audio_config req_config;
    bool unavailable;
    bool standby;
    uint64_t written;
    struct audio_device *dev;

    /* ---------------- debug instrumentation ---------------- */
    uint32_t id;
    volatile long lock_owner;
    volatile uint64_t lock_since_us;
    const char *volatile lock_fn;
    uint32_t writes;                /* total out_write calls */
    uint32_t writes_since_start;
    uint32_t underruns;             /* total -EPIPE */
    uint32_t epipe_run;             /* consecutive -EPIPE */
    uint32_t errors;                /* total non-zero pcm_write / start errors */
    uint32_t starts;                /* successful start_output_stream */
    uint64_t last_write_enter_us;
    uint64_t last_write_done_us;
    int last_ret;
    uint64_t created_us;
    /* content / tone instrumentation */
    int last_peak;                  /* peak |sample| of the last write      */
    int content_state;              /* -1 unknown, 0 silent, 1 audible      */
    uint32_t content_run;           /* writes since content_state changed   */
    uint32_t tone_pos;              /* sample counter for the test tone     */
};

struct stream_in {
    struct audio_stream_in stream;

    pthread_mutex_t lock; /* see note below on mutex acquisition order */
    struct pcm *pcm;
    struct pcm_config *pcm_config;
    struct audio_config req_config;
    bool unavailable;
    bool standby;

    struct audio_device *dev;

    /* ---------------- debug instrumentation ---------------- */
    uint32_t id;
    uint32_t reads;
    uint32_t errors;
    uint32_t starts;
    uint64_t last_read_enter_us;
    uint64_t last_read_done_us;
    int last_ret;
    uint64_t created_us;
};

/* ======================================================================== */
/*  Debug core: timestamps, logger, rate limiter                            */
/* ======================================================================== */

static volatile uint32_t g_seq = 0;
static volatile int g_log_io = 1;         /* log every write/read            */
static volatile int g_poll = 1;           /* watcher reads mixer + /proc     */
static volatile int g_codec_events = 1;   /* codec dump on events            */
static volatile int g_codec_hb = 0;       /* codec dump each heartbeat       */
static volatile int g_tone = 0;           /* 1 = replace audio with a 440 Hz test tone */

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static uint64_t now_ms(void)
{
    return now_us() / 1000ULL;
}

static void dbg_log(int prio, const char *func, const char *fmt, ...)
__attribute__((format(printf, 3, 4)));

static void dbg_log(int prio, const char *func, const char *fmt, ...)
{
    char msg[1500];
    va_list ap;
    uint64_t us = now_us();

    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    __android_log_print(prio, LOG_TAG, "[%5llu.%06llu] #%06u t%ld %s: %s",
                        ULL(us / 1000000ULL), ULL(us % 1000000ULL),
                        (unsigned int)__sync_add_and_fetch(&g_seq, 1),
                        (long)gettid(), func, msg);
}

#define DI(fmt, ...) dbg_log(ANDROID_LOG_INFO,  __func__, fmt, ##__VA_ARGS__)
#define DW(fmt, ...) dbg_log(ANDROID_LOG_WARN,  __func__, fmt, ##__VA_ARGS__)
#define DE(fmt, ...) dbg_log(ANDROID_LOG_ERROR, __func__, fmt, ##__VA_ARGS__)

struct ratelimit {
    uint64_t next_ms;
    uint32_t suppressed;
};

static bool rl_allow(struct ratelimit *r, uint32_t interval_ms, uint32_t *suppressed)
{
    uint64_t t = now_ms();
    if (t < r->next_ms) {
        r->suppressed++;
        return false;
    }
    *suppressed = r->suppressed;
    r->suppressed = 0;
    r->next_ms = t + interval_ms;
    return true;
}

/* Rate limited info log: at most one line per call site per interval. */
#define DI_RL(ms, fmt, ...) do {                                              \
        static struct ratelimit _rl;                                          \
        uint32_t _s = 0;                                                      \
        if (rl_allow(&_rl, (ms), &_s))                                        \
            dbg_log(ANDROID_LOG_INFO, __func__, "(+%u suppressed) " fmt,      \
                    _s, ##__VA_ARGS__);                                       \
    } while (0)

/* ======================================================================== */
/*  Decoders                                                                */
/* ======================================================================== */

struct name_bit {
    uint32_t bit;
    const char *name;
};

static const struct name_bit k_out_dev_names[] = {
        { AUDIO_DEVICE_OUT_EARPIECE, "EARPIECE" },
        { AUDIO_DEVICE_OUT_SPEAKER, "SPEAKER" },
        { AUDIO_DEVICE_OUT_WIRED_HEADSET, "WIRED_HEADSET" },
        { AUDIO_DEVICE_OUT_WIRED_HEADPHONE, "WIRED_HEADPHONE" },
        { AUDIO_DEVICE_OUT_BLUETOOTH_SCO, "BT_SCO" },
        { AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET, "BT_SCO_HEADSET" },
        { AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT, "BT_SCO_CARKIT" },
        { AUDIO_DEVICE_OUT_BLUETOOTH_A2DP, "BT_A2DP" },
        { AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_HEADPHONES, "BT_A2DP_HP" },
        { AUDIO_DEVICE_OUT_BLUETOOTH_A2DP_SPEAKER, "BT_A2DP_SPK" },
        { AUDIO_DEVICE_OUT_AUX_DIGITAL, "HDMI" },
        { AUDIO_DEVICE_OUT_USB_ACCESSORY, "USB_ACCESSORY" },
        { AUDIO_DEVICE_OUT_USB_DEVICE, "USB_DEVICE" },
        { AUDIO_DEVICE_OUT_REMOTE_SUBMIX, "REMOTE_SUBMIX" },
        { AUDIO_DEVICE_OUT_LINE, "LINE" },
        { AUDIO_DEVICE_OUT_SPEAKER_SAFE, "SPEAKER_SAFE" },
        { AUDIO_DEVICE_OUT_BUS, "BUS" },
        { AUDIO_DEVICE_OUT_USB_HEADSET, "USB_HEADSET" },
};

#define IN_BIT(x) ((uint32_t)((x) & ~AUDIO_DEVICE_BIT_IN))
static const struct name_bit k_in_dev_names[] = {
        { IN_BIT(AUDIO_DEVICE_IN_COMMUNICATION), "COMMUNICATION" },
        { IN_BIT(AUDIO_DEVICE_IN_AMBIENT), "AMBIENT" },
        { IN_BIT(AUDIO_DEVICE_IN_BUILTIN_MIC), "BUILTIN_MIC" },
        { IN_BIT(AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET), "BT_SCO_HEADSET" },
        { IN_BIT(AUDIO_DEVICE_IN_WIRED_HEADSET), "WIRED_HEADSET" },
        { IN_BIT(AUDIO_DEVICE_IN_BACK_MIC), "BACK_MIC" },
        { IN_BIT(AUDIO_DEVICE_IN_REMOTE_SUBMIX), "REMOTE_SUBMIX" },
        { IN_BIT(AUDIO_DEVICE_IN_USB_DEVICE), "USB_DEVICE" },
        { IN_BIT(AUDIO_DEVICE_IN_LINE), "LINE" },
        { IN_BIT(AUDIO_DEVICE_IN_BLUETOOTH_A2DP), "BT_A2DP" },
        { IN_BIT(AUDIO_DEVICE_IN_IP), "IP" },
        { IN_BIT(AUDIO_DEVICE_IN_BUS), "BUS" },
};

static void decode_mask(uint32_t mask, const struct name_bit *tbl, size_t n,
                        char *buf, size_t sz)
{
    size_t off = 0;
    uint32_t known = 0;

    buf[0] = '\0';
    if (mask == 0) {
        snprintf(buf, sz, "NONE");
        return;
    }
    for (size_t i = 0; i < n; i++) {
        if ((mask & tbl[i].bit) && off < sz) {
            off += (size_t)snprintf(buf + off, sz - off, "%s%s",
                                    off ? "|" : "", tbl[i].name);
            known |= tbl[i].bit;
        }
    }
    if ((mask & ~known) && off < sz)
        snprintf(buf + off, sz - off, "%sUNKNOWN(0x%x)", off ? "|" : "", mask & ~known);
}

static void decode_out_dev(uint32_t mask, char *buf, size_t sz)
{
    decode_mask(mask, k_out_dev_names, sizeof(k_out_dev_names) / sizeof(k_out_dev_names[0]), buf, sz);
}

static void decode_in_dev(uint32_t mask, char *buf, size_t sz)
{
    decode_mask(mask, k_in_dev_names, sizeof(k_in_dev_names) / sizeof(k_in_dev_names[0]), buf, sz);
}

static const char *mode_name(audio_mode_t mode)
{
    switch ((int)mode) {
        case 0: return "NORMAL";
        case 1: return "RINGTONE";
        case 2: return "IN_CALL";
        case 3: return "IN_COMMUNICATION";
        case 4: return "CALL_SCREEN";
        default: return "UNKNOWN";
    }
}

/* ======================================================================== */
/*  Instrumented locks (record owner + hold time, warn on contention)       */
/* ======================================================================== */

static uint64_t dev_lock_acquire(struct audio_device *adev, const char *fn)
{
    uint64_t t0 = now_us();

    if (pthread_mutex_trylock(&adev->lock) != 0) {
        long owner = adev->lock_owner;
        uint64_t since = adev->lock_since_us;
        DI("adev->lock CONTENDED: %s waiting (owner tid=%ld in %s, held %llu us so far)",
           fn, owner, adev->lock_fn ? adev->lock_fn : "?",
           ULL(since ? now_us() - since : 0));
        pthread_mutex_lock(&adev->lock);
    }
    adev->lock_owner = (long)gettid();
    adev->lock_since_us = now_us();
    adev->lock_fn = fn;
    return now_us() - t0;
}

static void dev_lock_release(struct audio_device *adev, const char *fn)
{
    uint64_t held = now_us() - adev->lock_since_us;

    adev->lock_owner = 0;
    pthread_mutex_unlock(&adev->lock);
    if (held > 20000)
        DW("adev->lock was HELD %llu us by %s", ULL(held), fn);
}

static uint64_t out_lock_acquire(struct stream_out *out, const char *fn)
{
    uint64_t t0 = now_us();

    if (pthread_mutex_trylock(&out->lock) != 0) {
        long owner = out->lock_owner;
        uint64_t since = out->lock_since_us;
        DI("out#%u->lock CONTENDED: %s waiting (owner tid=%ld in %s, held %llu us so far)",
           out->id, fn, owner, out->lock_fn ? out->lock_fn : "?",
           ULL(since ? now_us() - since : 0));
        pthread_mutex_lock(&out->lock);
    }
    out->lock_owner = (long)gettid();
    out->lock_since_us = now_us();
    out->lock_fn = fn;
    return now_us() - t0;
}

static void out_lock_release(struct stream_out *out, const char *fn)
{
    uint64_t held = now_us() - out->lock_since_us;

    out->lock_owner = 0;
    pthread_mutex_unlock(&out->lock);
    if (held > 20000)
        DW("out#%u->lock was HELD %llu us by %s", out->id, ULL(held), fn);
}

/* ======================================================================== */
/*  File / proc / sysfs helpers                                             */
/* ======================================================================== */

static int read_file(const char *path, char *buf, size_t sz)
{
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    ssize_t r;

    if (fd < 0) {
        snprintf(buf, sz, "<open failed: %s>", strerror(errno));
        return -1;
    }
    r = read(fd, buf, sz - 1);
    close(fd);
    if (r < 0) {
        snprintf(buf, sz, "<read failed: %s>", strerror(errno));
        return -1;
    }
    buf[r] = '\0';
    return (int)r;
}

/* Collapse whitespace: newlines become '|', runs of spaces become one. */
static void compact(char *s)
{
    char *w = s;
    bool sp = false;

    for (char *r = s; *r; r++) {
        char c = *r;
        if (c == '\n')
            c = '|';
        else if (c == '\r' || c == '\t')
            c = ' ';
        if (c == ' ') {
            if (sp)
                continue;
            sp = true;
        } else {
            sp = false;
        }
        *w++ = c;
    }
    *w = '\0';
    while (w > s && (w[-1] == '|' || w[-1] == ' '))
        *--w = '\0';
}

static void read_line(const char *path, char *buf, size_t sz)
{
    read_file(path, buf, sz);
    for (char *p = buf; *p; p++) {
        if (*p == '\n') {
            *p = '\0';
            break;
        }
    }
}

static int prop_int(const char *key, int def)
{
    return property_get_int32(key, def);
}

/* One-line snapshot of /proc/asound/cardN/pcmDx/sub0/status. dir = 'p' or 'c'. */
static void pcm_status_compact(int card, char dir, char *buf, size_t sz)
{
    char path[128];

    snprintf(path, sizeof(path), "/proc/asound/card%d/pcm%d%c/sub0/status", card, PCM_DEVICE, dir);
    read_file(path, buf, sz);
    compact(buf);
}

static void pcm_state_word(int card, char dir, char *state, size_t sz)
{
    char buf[512];
    char path[128];

    snprintf(path, sizeof(path), "/proc/asound/card%d/pcm%d%c/sub0/status", card, PCM_DEVICE, dir);
    read_file(path, buf, sizeof(buf));
    if (strncmp(buf, "closed", 6) == 0) {
        snprintf(state, sz, "closed");
    } else if (sscanf(buf, "state: %31s", state) != 1) {
        snprintf(state, sz, "?");
    }
}

static void dump_proc_file(const char *label, const char *filepath)
{
    FILE *fp = fopen(filepath, "r");
    if (!fp) {
        DW("[TRACE_PCM] Unable to open %s: %s (errno %d)",
           filepath, strerror(errno), errno);
        return;
    }

    DI("[TRACE_PCM] --- START %s (%s) ---", label, filepath);

    char line[256];
    while (fgets(line, sizeof(line), fp) != NULL) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (len > 0) {
            DI("[TRACE_PCM]   %s", line);
        }
    }

    DI("[TRACE_PCM] --- END %s ---", label);
    fclose(fp);
}

static void trace_pcm_proc_info(int card, int device, int subdevice)
{
    char proc_path[128];

    snprintf(proc_path, sizeof(proc_path), "/proc/asound/card%d/pcm%dp/info", card, device);
    dump_proc_file("PCM Device Info", proc_path);

    snprintf(proc_path, sizeof(proc_path), "/proc/asound/card%d/pcm%dp/sub%d/info", card, device, subdevice);
    dump_proc_file("Subdevice Info", proc_path);

    snprintf(proc_path, sizeof(proc_path), "/proc/asound/card%d/pcm%dp/sub%d/hw_params", card, device, subdevice);
    dump_proc_file("HW Params", proc_path);

    snprintf(proc_path, sizeof(proc_path), "/proc/asound/card%d/pcm%dp/sub%d/sw_params", card, device, subdevice);
    dump_proc_file("SW Params", proc_path);

    snprintf(proc_path, sizeof(proc_path), "/proc/asound/card%d/pcm%dp/sub%d/status", card, device, subdevice);
    dump_proc_file("PCM Runtime Status", proc_path);
}

static void trace_pcm_node_access(int card, int device)
{
    char pcm_path[128];
    snprintf(pcm_path, sizeof(pcm_path), "/dev/snd/pcmC%dD%dp", card, device);

    struct stat st;
    if (stat(pcm_path, &st) != 0) {
        DE("[TRACE_PCM] Failed to stat %s: %s (errno %d)",
           pcm_path, strerror(errno), errno);
        return;
    }

    char perm[11] = "----------";
    if (S_ISCHR(st.st_mode))      perm[0] = 'c';
    else if (S_ISBLK(st.st_mode)) perm[0] = 'b';
    else if (S_ISDIR(st.st_mode)) perm[0] = 'd';

    if (st.st_mode & S_IRUSR) perm[1] = 'r';
    if (st.st_mode & S_IWUSR) perm[2] = 'w';
    if (st.st_mode & S_IXUSR) perm[3] = 'x';
    if (st.st_mode & S_IRGRP) perm[4] = 'r';
    if (st.st_mode & S_IWGRP) perm[5] = 'w';
    if (st.st_mode & S_IXGRP) perm[6] = 'x';
    if (st.st_mode & S_IROTH) perm[7] = 'r';
    if (st.st_mode & S_IWOTH) perm[8] = 'w';
    if (st.st_mode & S_IXOTH) perm[9] = 'x';

    int can_read  = (access(pcm_path, R_OK) == 0);
    int can_write = (access(pcm_path, W_OK) == 0);

    DI("[TRACE_PCM] Path: %s | %s UID:%d GID:%d Major:%d,Minor:%d | R_OK:%d W_OK:%d",
       pcm_path, perm, st.st_uid, st.st_gid,
       major(st.st_rdev), minor(st.st_rdev), can_read, can_write);

    trace_pcm_proc_info(card, device, 0);
}

/* Runtime PM, load average: does not wake anything. */
static void log_pm_and_load(const char *why)
{
    char rs[64], ctl[64], act[64], sus[64], crs[64], load[128];

    read_line("/sys/bus/pci/devices/0000:00:1f.3/power/runtime_status", rs, sizeof(rs));
    read_line("/sys/bus/pci/devices/0000:00:1f.3/power/control", ctl, sizeof(ctl));
    read_line("/sys/bus/pci/devices/0000:00:1f.3/power/runtime_active_time", act, sizeof(act));
    read_line("/sys/bus/pci/devices/0000:00:1f.3/power/runtime_suspended_time", sus, sizeof(sus));
    read_line("/sys/bus/hdaudio/devices/ehdaudio0D0/power/runtime_status", crs, sizeof(crs));
    read_line("/proc/loadavg", load, sizeof(load));
    DI("%s PM: pci-1f.3 runtime_status=%s control=%s active_ms=%s suspended_ms=%s | codec runtime_status=%s | loadavg=%s",
       why, rs, ctl, act, sus, crs, load);
}

/* Selected nodes of the HDA codec proc file.  NOTE: reading this file powers
 * the codec up, see OBSERVER EFFECT in the header comment. */
static void codec_dump_key_nodes(int card, const char *why)
{
    char path[96];
    char *buf;
    char *save = NULL;
    bool interesting = true;   /* AFG header (power, GPIO) is interesting */
    bool print_next = false;

    snprintf(path, sizeof(path), "/proc/asound/card%d/codec#0", card);
    buf = malloc(32768);
    if (!buf)
        return;
    if (read_file(path, buf, 32768) < 0) {
        DW("CODEC (%s): %s", why, buf);
        free(buf);
        return;
    }

    DI("--- CODEC KEY NODES (%s) ---", why);
    for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        if (strncmp(line, "Node 0x", 7) == 0) {
            unsigned int node = (unsigned int)strtoul(line + 5, NULL, 16);
            interesting = (node == 0x02 || node == 0x03 || node == 0x14 ||
                           node == 0x1b || node == 0x21);
            print_next = false;
            if (interesting)
                DI("  %s", line);
            continue;
        }
        if (!interesting)
            continue;
        if (print_next) {
            DI("    %s", line);
            print_next = false;
            continue;
        }
        if (strstr(line, "Amp-Out vals") || strstr(line, "Converter:") ||
            strstr(line, "EAPD") || strstr(line, "Pin-ctls") ||
            strstr(line, "Power:") || strstr(line, "GPIO") ||
            strstr(line, "IO[")) {
            DI("  %s", line);
        } else if (strstr(line, "Connection:")) {
            DI("  %s", line);
            print_next = true;
        }
    }
    DI("--- END CODEC KEY NODES ---");
    free(buf);
}

/* ======================================================================== */
/*  Mixer inspection (own read-only handle, independent of audio_route)     */
/* ======================================================================== */

static const char *const k_key_ctls[] = {
        "Master Playback Switch", "Master Playback Volume",
        "Speaker Playback Switch", "Speaker Playback Volume",
        "Headphone Playback Switch", "Headphone Playback Volume",
        "Auto-Mute Mode",
        "PGA1.0 1 Master Playback Volume", "PGA7.0 7 Master Playback Volume",
        "PGA8.0 8 Master Playback Volume", "PGA9.0 9 Master Playback Volume",
        "IEC958 Playback Switch",
        "Capture Switch", "Capture Volume",
        "Dmic0 Capture Switch", "Dmic0 Capture Volume", "Dmic1 2nd Capture Volume",
        "PGA2.0 2 Master Capture Volume",
};

static void fmt_ctl_values(struct mixer_ctl *ctl, char *buf, size_t sz)
{
    enum mixer_ctl_type type = mixer_ctl_get_type(ctl);
    unsigned int n = mixer_ctl_get_num_values(ctl);
    size_t off = 0;

    buf[0] = '\0';
    if (type != MIXER_CTL_TYPE_BOOL && type != MIXER_CTL_TYPE_INT &&
        type != MIXER_CTL_TYPE_ENUM) {
        snprintf(buf, sz, "<type %s count=%u>", mixer_ctl_get_type_string(ctl), n);
        return;
    }
    off += (size_t)snprintf(buf + off, sz - off, "[");
    for (unsigned int i = 0; i < n && i < 8 && off < sz; i++)
        off += (size_t)snprintf(buf + off, sz - off, "%s%d", i ? " " : "",
                                mixer_ctl_get_value(ctl, i));
    if (off < sz)
        off += (size_t)snprintf(buf + off, sz - off, "]");
    if (type == MIXER_CTL_TYPE_ENUM && n > 0 && off < sz) {
        int v = mixer_ctl_get_value(ctl, 0);
        const char *s = mixer_ctl_get_enum_string(ctl, (unsigned int)v);
        snprintf(buf + off, sz - off, "(%s)", s ? s : "?");
    }
}

static void mixer_snapshot(struct audio_device *adev, char *buf, size_t sz)
{
    size_t off = 0;

    buf[0] = '\0';
    if (!adev->dbg_mixer) {
        snprintf(buf, sz, "(no debug mixer handle)");
        return;
    }
    for (size_t i = 0; i < sizeof(k_key_ctls) / sizeof(k_key_ctls[0]); i++) {
        struct mixer_ctl *ctl = mixer_get_ctl_by_name(adev->dbg_mixer, k_key_ctls[i]);
        char v[96];

        if (!ctl)
            continue;
        fmt_ctl_values(ctl, v, sizeof(v));
        if (off < sz)
            off += (size_t)snprintf(buf + off, sz - off, "%s%s=%s", off ? "; " : "",
                                    k_key_ctls[i], v);
    }
}

static int mixer_get_first(struct audio_device *adev, const char *name, int *val)
{
    struct mixer_ctl *ctl;

    if (!adev->dbg_mixer)
        return -1;
    ctl = mixer_get_ctl_by_name(adev->dbg_mixer, name);
    if (!ctl)
        return -1;
    *val = mixer_ctl_get_value(ctl, 0);
    return 0;
}

static void mixer_full_dump(struct audio_device *adev, const char *why)
{
    unsigned int n;

    if (!adev->dbg_mixer)
        return;
    n = mixer_get_num_ctls(adev->dbg_mixer);
    DI("--- MIXER FULL DUMP (%s): %u controls ---", why, n);
    for (unsigned int i = 0; i < n; i++) {
        struct mixer_ctl *ctl = mixer_get_ctl(adev->dbg_mixer, i);
        char v[128];

        if (!ctl)
            continue;
        if (mixer_ctl_get_type(ctl) == MIXER_CTL_TYPE_BYTE ||
            mixer_ctl_get_type(ctl) == MIXER_CTL_TYPE_IEC958) {
            DI("  [%2u] %-40s <%s, skipped>", i, mixer_ctl_get_name(ctl),
               mixer_ctl_get_type_string(ctl));
            continue;
        }
        fmt_ctl_values(ctl, v, sizeof(v));
        DI("  [%2u] %-40s %s", i, mixer_ctl_get_name(ctl), v);
    }
    DI("--- END MIXER FULL DUMP ---");
}

/* Everything we can cheaply learn about the current state, for error paths. */
static void log_failure_context(struct audio_device *adev, const char *why)
{
    char st[600], snap[1200];

    DW("=== FAILURE CONTEXT: %s ===", why);
    pcm_status_compact(adev->card_dbg, 'p', st, sizeof(st));
    DW("pcm0p status: %s", st);
    pcm_status_compact(adev->card_dbg, 'c', st, sizeof(st));
    DW("pcm0c status: %s", st);
    mixer_snapshot(adev, snap, sizeof(snap));
    DW("mixer: %s", snap);
    log_pm_and_load("failure");
    if (g_codec_events)
        codec_dump_key_nodes(adev->card_dbg, why);
    DW("=== END FAILURE CONTEXT ===");
}

/* ======================================================================== */
/*  Forward declarations                                                    */
/* ======================================================================== */

static uint32_t out_get_sample_rate(const struct audio_stream *stream);
static size_t out_get_buffer_size(const struct audio_stream *stream);
static audio_format_t out_get_format(const struct audio_stream *stream);
static uint32_t in_get_sample_rate(const struct audio_stream *stream);
static size_t in_get_buffer_size(const struct audio_stream *stream);
static audio_format_t in_get_format(const struct audio_stream *stream);

/* ======================================================================== */
/*  Routing                                                                 */
/* ======================================================================== */

/* called with adev->lock held (or single threaded from adev_open) */
static void select_devices(struct audio_device *adev, const char *why)
{
    int headphone_on;
    int speaker_on;
    int main_mic_on;
    int headset_mic_on;
    char outn[200], inn[200];
    char snap_before[1200], snap_after[1200];
    uint64_t t0 = now_us();
    uint32_t n = ++adev->sel_count;

    decode_out_dev(adev->out_device, outn, sizeof(outn));
    decode_in_dev(adev->in_device, inn, sizeof(inn));
    DI("#%u ENTER why=%s out_device=0x%x[%s] in_device=0x%x[%s]",
       n, why, adev->out_device, outn, adev->in_device, inn);

    mixer_snapshot(adev, snap_before, sizeof(snap_before));
    DI("#%u mixer BEFORE: %s", n, snap_before);

    headphone_on = adev->out_device & (AUDIO_DEVICE_OUT_WIRED_HEADSET |
                                       AUDIO_DEVICE_OUT_WIRED_HEADPHONE);
    speaker_on = adev->out_device & AUDIO_DEVICE_OUT_SPEAKER;
    main_mic_on = adev->in_device & AUDIO_DEVICE_IN_BUILTIN_MIC;
    headset_mic_on = adev->in_device & AUDIO_DEVICE_IN_WIRED_HEADSET;

    DI("#%u computed flags: headphone_on=%d speaker_on=%d main_mic_on=%d headset_mic_on=%d",
       n, headphone_on, speaker_on, main_mic_on, headset_mic_on);

    if (!speaker_on && !headphone_on) {
        DW("#%u NO OUTPUT PATH SELECTED (out_device=0x%x[%s]): audio_route_reset() restores "
           "the saved reset state (Master Playback OFF / volume 0 at adev_open time) and "
           "audio_route_update_mixer() WRITES IT TO HARDWARE => output goes SILENT while "
           "the PCM keeps RUNNING", n, adev->out_device, outn);
    }

    DI("#%u calling audio_route_reset()", n);
    audio_route_reset(adev->ar);
    DI("#%u audio_route_reset() done", n);

    if (speaker_on) {
        DI("#%u applying path \"speaker\"", n);
        audio_route_apply_path(adev->ar, "speaker");
    } else {
        DI("#%u skipping path \"speaker\"", n);
    }

    if (headphone_on) {
        DI("#%u applying path \"headphone\"", n);
        audio_route_apply_path(adev->ar, "headphone");
    } else {
        DI("#%u skipping path \"headphone\"", n);
    }

    if (main_mic_on) {
        DI("#%u applying path \"main-mic\"", n);
        audio_route_apply_path(adev->ar, "main-mic");
    } else {
        DI("#%u skipping path \"main-mic\"", n);
    }

    if (headset_mic_on) {
        DI("#%u applying path \"headset-mic\"", n);
        audio_route_apply_path(adev->ar, "headset-mic");
    } else {
        DI("#%u skipping path \"headset-mic\"", n);
    }

    DI("#%u calling audio_route_update_mixer() (writes to hardware)", n);
    audio_route_update_mixer(adev->ar);
    DI("#%u audio_route_update_mixer() done", n);

    mixer_snapshot(adev, snap_after, sizeof(snap_after));
    DI("#%u mixer AFTER : %s", n, snap_after);
    if (strcmp(snap_before, snap_after) != 0)
        DI("#%u mixer state CHANGED by this call", n);
    else
        DI("#%u mixer state UNCHANGED by this call", n);

    if (speaker_on) {
        int v;
        if (mixer_get_first(adev, "Master Playback Switch", &v) == 0 && v == 0)
            DW("#%u speaker path applied but Master Playback Switch == 0 (MUTED)", n);
        if (mixer_get_first(adev, "Master Playback Volume", &v) == 0 && v == 0)
            DW("#%u speaker path applied but Master Playback Volume == 0", n);
        if (mixer_get_first(adev, "Speaker Playback Switch", &v) == 0 && v == 0)
            DW("#%u speaker path applied but Speaker Playback Switch == 0 (MUTED)", n);
        if (mixer_get_first(adev, "Speaker Playback Volume", &v) == 0 && v == 0)
            DW("#%u speaker path applied but Speaker Playback Volume == 0", n);
    }

    DI("#%u SUMMARY hp=%c speaker=%c main-mic=%c headset-mic=%c",
       n, headphone_on ? 'y' : 'n', speaker_on ? 'y' : 'n',
       main_mic_on ? 'y' : 'n', headset_mic_on ? 'y' : 'n');
    DI("#%u EXIT took %llu us", n, ULL(now_us() - t0));
}

/* ======================================================================== */
/*  Standby                                                                 */
/* ======================================================================== */

/* must be called with hw device and output stream mutexes locked */
static void do_out_standby(struct stream_out *out, const char *reason)
{
    struct audio_device *adev = out->dev;

    DI("out#%u standby requested (%s): standby=%d pcm=%p written=%llu writes=%u underruns=%u",
       out->id, reason, out->standby, (void *)out->pcm, ULL(out->written),
       out->writes, out->underruns);
    if (!out->standby) {
        char st[600];
        uint64_t t0;

        pcm_status_compact(adev->card_dbg, 'p', st, sizeof(st));
        DW("out#%u CLOSING pcm=%p reason=%s | pcm0p before close: %s",
           out->id, (void *)out->pcm, reason, st);
        t0 = now_us();
        pcm_close(out->pcm);
        DI("out#%u pcm_close took %llu us", out->id, ULL(now_us() - t0));
        out->pcm = NULL;
        adev->active_out = NULL;
        out->standby = true;
        out->writes_since_start = 0;
        pcm_status_compact(adev->card_dbg, 'p', st, sizeof(st));
        DI("out#%u pcm0p after close: %s", out->id, st);
    } else {
        DI("out#%u already in standby, nothing to close", out->id);
    }
    out->unavailable = false;
}

/* must be called with hw device and input stream mutexes locked */
static void do_in_standby(struct stream_in *in, const char *reason)
{
    struct audio_device *adev = in->dev;

    DI("in#%u standby requested (%s): standby=%d pcm=%p reads=%u",
       in->id, reason, in->standby, (void *)in->pcm, in->reads);
    if (!in->standby) {
        uint64_t t0 = now_us();

        DW("in#%u CLOSING pcm=%p reason=%s", in->id, (void *)in->pcm, reason);
        pcm_close(in->pcm);
        DI("in#%u pcm_close took %llu us", in->id, ULL(now_us() - t0));
        in->pcm = NULL;
        adev->active_in = NULL;
        in->standby = true;
    } else {
        DI("in#%u already in standby, nothing to close", in->id);
    }
}

static int get_pcm_card(const char* name)
{
    char id_filepath[PATH_MAX] = {0};
    char number_filepath[PATH_MAX] = {0};
    ssize_t written;

    snprintf(id_filepath, sizeof(id_filepath), "/proc/asound/%s", name);

    written = readlink(id_filepath, number_filepath, sizeof(number_filepath));
    if (written < 0) {
        DW("Sound card %s does not exist (%s)", name, strerror(errno));
        return -1;
    } else if (written >= (ssize_t)sizeof(id_filepath)) {
        DE("Sound card %s name is too long - setting default", name);
        return -1;
    }
    DI("Sound card %s exists -> link \"%s\"", name, number_filepath);
    return atoi(number_filepath + 4);
}

static void update_bt_card(struct audio_device *adev){
    adev->bt_card = get_pcm_card(AUDIO_BT_DRIVER_NAME); //update driver name if changed from BT side.
}

static unsigned int round_to_16_mult(unsigned int size)
{
    return (size + 15) & ~15;   /* 0xFFFFFFF0; */
}

static void log_pcm_params(const char *what, struct pcm_params *p, const struct pcm_config *cfg)
{
    unsigned int rmin, rmax, cmin, cmax, smin, smax, nmin, nmax;

    if (!p) {
        DW("%s: pcm_params_get returned NULL", what);
        return;
    }
    rmin = pcm_params_get_min(p, PCM_PARAM_RATE);
    rmax = pcm_params_get_max(p, PCM_PARAM_RATE);
    cmin = pcm_params_get_min(p, PCM_PARAM_CHANNELS);
    cmax = pcm_params_get_max(p, PCM_PARAM_CHANNELS);
    smin = pcm_params_get_min(p, PCM_PARAM_PERIOD_SIZE);
    smax = pcm_params_get_max(p, PCM_PARAM_PERIOD_SIZE);
    nmin = pcm_params_get_min(p, PCM_PARAM_PERIODS);
    nmax = pcm_params_get_max(p, PCM_PARAM_PERIODS);
    DI("%s: hardware accepts rate[%u..%u] channels[%u..%u] period_size[%u..%u] periods[%u..%u]",
       what, rmin, rmax, cmin, cmax, smin, smax, nmin, nmax);
    DI("%s: HAL config asks  rate=%u channels=%u period_size=%u periods=%u",
       what, cfg->rate, cfg->channels, cfg->period_size, cfg->period_count);
    if (cfg->rate < rmin || cfg->rate > rmax)
        DW("%s: CONFIG OUT OF RANGE: rate %u not in [%u..%u]", what, cfg->rate, rmin, rmax);
    if (cfg->channels < cmin || cfg->channels > cmax)
        DW("%s: CONFIG OUT OF RANGE: channels %u not in [%u..%u]", what, cfg->channels, cmin, cmax);
    if (cfg->period_size < smin || cfg->period_size > smax)
        DW("%s: CONFIG OUT OF RANGE: period_size %u not in [%u..%u]", what, cfg->period_size, smin, smax);
    if (cfg->period_count < nmin || cfg->period_count > nmax)
        DW("%s: CONFIG OUT OF RANGE: period_count %u not in [%u..%u]", what, cfg->period_count, nmin, nmax);
}

/* ======================================================================== */
/*  Start streams                                                           */
/* ======================================================================== */

/* must be called with hw device and output stream mutexes locked */
static int start_output_stream(struct stream_out *out)
{
    struct audio_device *adev = out->dev;
    int retry;
    uint64_t t_start = now_us();

    DI("out#%u ENTER card=%d device=%d cfg[rate=%u ch=%u fmt=%d period=%u count=%u start_thr=%u] "
       "unavailable=%d voip=%d bt_card=%d",
       out->id, adev->card, PCM_DEVICE, out->pcm_config->rate, out->pcm_config->channels,
       (int)out->pcm_config->format, out->pcm_config->period_size,
       out->pcm_config->period_count, out->pcm_config->start_threshold,
       out->unavailable, adev->in_sco_voip_call, adev->bt_card);

    trace_pcm_node_access(out->dev->card, PCM_DEVICE);

    if (out->unavailable) {
        DW("out#%u output not available -> -ENODEV", out->id);
        return -ENODEV;
    }

//[BT SCO VoIP Call
    for (retry = 0; retry < 3; retry++) {
        uint64_t t0 = now_us();

        errno = 0;
        if (adev->in_sco_voip_call) {
            out->pcm = pcm_open(adev->bt_card, PCM_DEVICE, PCM_OUT, &bt_out_config);
        } else {
            out->pcm = pcm_open(adev->card, PCM_DEVICE,
                                PCM_OUT | PCM_NORESTART | PCM_MONOTONIC,
                                out->pcm_config);
        }
        DI("out#%u pcm_open attempt %d took %llu us: pcm=%p ready=%d errno=%d(%s) tinyalsa_err=\"%s\"",
           out->id, retry, ULL(now_us() - t0), (void *)out->pcm,
           out->pcm ? pcm_is_ready(out->pcm) : 0, errno, strerror(errno),
           out->pcm ? pcm_get_error(out->pcm) : "pcm==NULL");

        if (out->pcm && pcm_is_ready(out->pcm))
            break; /* success */

        DW("out#%u start_output_stream: attempt %d FAILED: %s", out->id, retry,
           out->pcm ? pcm_get_error(out->pcm) : "pcm_open returned NULL");

        if (out->pcm) {
            pcm_close(out->pcm);
            out->pcm = NULL;      /* never leave a dangling closed pointer */
        }
        usleep(20 * 1000);        /* give the DSP teardown time to finish */
    }
//BT SCO VoIP Call]

    if (!out->pcm) {
        DE("out#%u pcm_open(out) failed after %d attempts: device not found -> -ENODEV (%llu us)",
           out->id, retry, ULL(now_us() - t_start));
        out->errors++;
        log_failure_context(adev, "pcm_open(out) failed");
        return -ENODEV;
    } else if (!pcm_is_ready(out->pcm)) {
        DE("out#%u pcm_open(out) failed: %s", out->id, pcm_get_error(out->pcm));
        pcm_close(out->pcm);
        out->unavailable = true;
        out->errors++;
        return -ENOMEM;
    }

    out->starts++;
    out->epipe_run = 0;
    out->writes_since_start = 0;
    out->content_state = -1;
    adev->active_out = out;
    DI("out#%u PCM OPEN OK after %d failed attempt(s): pcm=%p buffer_size=%u frames "
       "(start #%u, %llu us since ENTER)",
       out->id, retry, (void *)out->pcm, pcm_get_buffer_size(out->pcm),
       out->starts, ULL(now_us() - t_start));

    /* now the PCM is open: hw_params / sw_params / status are populated */
    trace_pcm_proc_info(adev->card, PCM_DEVICE, 0);

    /* force mixer updates */
    select_devices(adev, "start_output_stream");

    if (g_codec_events)
        codec_dump_key_nodes(adev->card_dbg, "after start_output_stream");

    DI("out#%u EXIT ok, total %llu us", out->id, ULL(now_us() - t_start));
    return 0;
}

/* must be called with hw device and input stream mutexes locked */
static int start_input_stream(struct stream_in *in)
{
    struct audio_device *adev = in->dev;
    uint64_t t_start = now_us();

    DI("in#%u ENTER cardc=%d voip=%d", in->id, adev->cardc, adev->in_sco_voip_call);

//[BT SCO VoIP Call
    if(adev->in_sco_voip_call) {
        DI("in#%u sco voip call active, opening bt card %d", in->id, adev->bt_card);
        errno = 0;
        in->pcm = pcm_open(adev->bt_card, PCM_DEVICE, PCM_IN, &bt_in_config);
//BT SCO VoIP Call]
    } else {
        DI("in#%u PCM record card selected = %d cfg[rate=%u ch=%u period=%u count=%u]",
           in->id, adev->cardc, in->pcm_config->rate, in->pcm_config->channels,
           in->pcm_config->period_size, in->pcm_config->period_count);
        errno = 0;
        in->pcm = pcm_open(adev->cardc, PCM_DEVICE, PCM_IN, in->pcm_config);
    }
    DI("in#%u pcm_open took %llu us: pcm=%p ready=%d errno=%d(%s) tinyalsa_err=\"%s\"",
       in->id, ULL(now_us() - t_start), (void *)in->pcm,
       in->pcm ? pcm_is_ready(in->pcm) : 0, errno, strerror(errno),
       in->pcm ? pcm_get_error(in->pcm) : "pcm==NULL");

    if (!in->pcm) {
        DE("in#%u pcm_open(in) returned NULL -> -ENODEV", in->id);
        in->errors++;
        return -ENODEV;
    } else if (!pcm_is_ready(in->pcm)) {
        DE("in#%u pcm_open(in) failed: %s", in->id, pcm_get_error(in->pcm));
        pcm_close(in->pcm);
        in->errors++;
        return -ENOMEM;
    }

    in->starts++;
    adev->active_in = in;
    DI("in#%u PCM OPEN OK (start #%u)", in->id, in->starts);

    /* force mixer updates */
    select_devices(adev, "start_input_stream");

    DI("in#%u EXIT ok, total %llu us", in->id, ULL(now_us() - t_start));
    return 0;
}

/* ======================================================================== */
/*  audio_stream_out                                                        */
/* ======================================================================== */

static uint32_t out_get_sample_rate(const struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    DI_RL(2000, "out#%u rate %d", out->id, out->req_config.sample_rate);
    return out->req_config.sample_rate;
}

static int out_set_sample_rate(struct audio_stream *stream __unused, uint32_t rate)
{
    DI("out_set_sample_rate(%u) -> -ENOSYS", rate);
    return -ENOSYS;
}

static size_t out_get_buffer_size(const struct audio_stream *stream)
{
    size_t sz = pcm_config_out.period_size *
                audio_stream_out_frame_size((struct audio_stream_out *)stream);
    DI_RL(2000, "out_get_buffer_size -> %zu bytes", sz);
    return sz;
}

static uint32_t out_get_channels(const struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    DI_RL(2000, "out#%u channels %d (mask 0x%x)", out->id,
          popcount(out->req_config.channel_mask), out->req_config.channel_mask);
    return out->req_config.channel_mask;
}

static audio_format_t out_get_format(const struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    DI_RL(2000, "out#%u format 0x%x", out->id, out->req_config.format);
    return out->req_config.format;
}

static int out_set_format(struct audio_stream *stream __unused, audio_format_t format)
{
    DI("out_set_format(0x%x) -> -ENOSYS", format);
    return -ENOSYS;
}

static int out_standby(struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;

    DI("out#%u AudioFlinger requested standby", out->id);
    dev_lock_acquire(out->dev, __func__);
    out_lock_acquire(out, __func__);
    do_out_standby(out, "out_standby() from AudioFlinger");
    out_lock_release(out, __func__);
    dev_lock_release(out->dev, __func__);

    return 0;
}

static int out_dump(const struct audio_stream *stream __unused, int fd __unused)
{
    DI("out_dump");
    return 0;
}

static int out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    DI("ENTER kvpairs: %s", kvpairs);
    struct stream_out *out = (struct stream_out *)stream;
    struct audio_device *adev = out->dev;
    struct str_parms *parms;
    char value[32];
    char names[200];
    int ret;
    int status = 0;
    unsigned int val;

    parms = str_parms_create_str(kvpairs);
    DI("out#%u str_parms_create_str done, parms=%p", out->id, (void *)parms);

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING,
                            value, sizeof(value));
    DI("out#%u str_parms_get_str(ROUTING) ret=%d value=\"%s\"", out->id, ret,
       ret >= 0 ? value : "(none)");

    dev_lock_acquire(adev, __func__);

    if (ret >= 0) {
        val = atoi(value);
        decode_out_dev(val, names, sizeof(names));
        DI("out#%u parsed routing val=0x%x[%s], current adev->out_device=0x%x",
           out->id, val, names, adev->out_device);

        if ((adev->out_device != val) && (val != 0)) {
            char oldn[200];
            decode_out_dev(adev->out_device, oldn, sizeof(oldn));
            DW("out#%u routing CHANGED 0x%x[%s] -> 0x%x[%s]%s", out->id,
               adev->out_device, oldn, val, names,
               (val & AUDIO_DEVICE_OUT_SPEAKER) ? "" :
               "  <-- NEW ROUTE HAS NO SPEAKER BIT: HAL will apply no speaker path!");
            adev->out_device = val;
            select_devices(adev, "out_set_parameters(routing)");
        } else {
            DI("out#%u routing UNCHANGED or val==0 -> skipping select_devices() (out_device=0x%x val=0x%x)",
               out->id, adev->out_device, val);
        }
    } else {
        DI("out#%u no ROUTING key in kvpairs, nothing to do", out->id);
    }
    dev_lock_release(adev, __func__);

    str_parms_destroy(parms);
    DI("EXIT status=%d", status);
    return status;
}

static char *out_get_parameters(const struct audio_stream *stream, const char *keys)
{
    DI_RL(2000, "keys : %s", keys);
    struct stream_out *out = (struct stream_out *)stream;
    struct str_parms *query = str_parms_create_str(keys);
    char *str_parm = NULL;
    char value[256];
    struct str_parms *reply = str_parms_create();
    int ret;

    if(reply == NULL || query == NULL) {
        if(reply != NULL) str_parms_destroy(reply);
        if(query != NULL) str_parms_destroy(query);
        return NULL;
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_FORMATS, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_FORMATS, "AUDIO_FORMAT_PCM_16_BIT");
        str_parm = str_parms_to_str(reply);
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_int(reply, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES, out->req_config.sample_rate);

        if(str_parm != NULL)
            str_parms_destroy((struct str_parms *)str_parm);

        str_parm = str_parms_to_str(reply);
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_CHANNELS, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_CHANNELS,
                          (out->req_config.channel_mask == AUDIO_CHANNEL_OUT_MONO ? "AUDIO_CHANNEL_OUT_MONO" : "AUDIO_CHANNEL_OUT_STEREO"));

        if(str_parm != NULL)
            str_parms_destroy((struct str_parms *)str_parm);

        str_parm = str_parms_to_str(reply);
    }

    str_parms_destroy(query);
    str_parms_destroy(reply);

    DI_RL(2000, "out#%u returning keyValuePair %s", out->id, str_parm ? str_parm : "(null)");
    return str_parm;
}

static uint32_t out_get_latency(const struct audio_stream_out *stream __unused)
{
    uint32_t lat = (pcm_config_out.period_size * OUT_PERIOD_COUNT * 1000) / pcm_config_out.rate;
    DI_RL(2000, "out_get_latency -> %u ms", lat);
    return lat;
}

static int out_set_volume(struct audio_stream_out *stream __unused, float left,
                          float right)
{
    DI("out_set_volume: Left:%f Right:%f -> -ENOSYS (HAL does no software volume)", left, right);
    return -ENOSYS;
}

static void out_log_write_summary(struct stream_out *out, size_t bytes,
                                  unsigned int frames, uint64_t gap_us,
                                  uint64_t lock_wait_us, uint64_t write_us,
                                  int ret, bool did_start, uint64_t start_us,
                                  bool did_standby, int peak, bool tone)
{
    const uint64_t period_us =
            (uint64_t)out->pcm_config->period_size * 1000000ULL / out->pcm_config->rate;
    uint32_t n = ++out->writes;
    bool gap_bad, wait_bad, write_bad, anomaly;

    out->writes_since_start++;
    out->last_peak = peak;
    {
        int state = (peak > 0) ? 1 : 0;
        if (state != out->content_state) {
            dbg_log(ANDROID_LOG_WARN, "out_write",
                    "out#%u CONTENT CHANGED: %s -> %s (peak=%d of 32767, previous state lasted %u writes = %.2f s)%s",
                    out->id,
                    out->content_state < 0 ? "start" : (out->content_state ? "AUDIBLE" : "SILENT"),
                    state ? "AUDIBLE (real samples)" : "SILENT (all-zero samples from AudioFlinger)",
                    peak, out->content_run,
                    (double)out->content_run * (double)out->pcm_config->period_size /
                    (double)out->pcm_config->rate,
                    tone ? " [TEST TONE ACTIVE]" : "");
            out->content_state = state;
            out->content_run = 0;
        }
        out->content_run++;
    }
    gap_bad = !did_start && !did_standby && n > 1 && gap_us > period_us * 2;
    wait_bad = lock_wait_us > 5000;
    write_bad = write_us > period_us * 3;
    anomaly = gap_bad || wait_bad || write_bad || ret != 0 || did_start || did_standby;

    if (g_log_io || anomaly || (n % 50) == 0) {
        dbg_log(anomaly ? ANDROID_LOG_WARN : ANDROID_LOG_INFO, "out_write",
                "out#%u WRITE n=%u bytes=%zu frames=%u gap=%.3f ms lockwait=%llu us "
                "start=%llu us pcm_write=%.3f ms ret=%d written=%llu since_start=%u peak=%d%s%s%s%s%s%s%s",
                out->id, n, bytes, frames, (double)gap_us / 1000.0, ULL(lock_wait_us),
                ULL(start_us), (double)write_us / 1000.0, ret, ULL(out->written),
                out->writes_since_start, peak,
                tone ? " [TONE]" : "",
                did_start ? " [STARTED-STREAM]" : "",
                did_standby ? " [WENT-STANDBY-FIRST]" : "",
                gap_bad ? " [GAP: writer was late -> underrun risk]" : "",
                wait_bad ? " [LOCK-WAIT]" : "",
                write_bad ? " [SLOW-PCM_WRITE]" : "",
                ret != 0 ? " [ERROR]" : "");
    }
    if (out->writes_since_start == 50 && ret == 0 && g_codec_events)
        codec_dump_key_nodes(out->dev->card_dbg, "50 writes after stream start");
}

static ssize_t out_write(struct audio_stream_out *stream, const void* buffer,
                         size_t bytes)
{
    int ret = 0;
    struct stream_out *out = (struct stream_out *)stream;
    struct audio_device *adev = out->dev;
    size_t frame_size = audio_stream_out_frame_size(stream);
    int16_t *out_buffer = (int16_t *)buffer;
    unsigned int out_frames = bytes / frame_size;

    uint64_t t_enter = now_us();
    uint64_t gap_us = out->last_write_enter_us ? (t_enter - out->last_write_enter_us) : 0;
    uint64_t lock_wait_us = 0;
    uint64_t write_us = 0;
    uint64_t start_us = 0;
    bool did_standby = false;
    bool did_start = false;
    char pcm_err[160] = "";
    int peak = 0;
    bool tone = false;
    int16_t *tone_buf = NULL;

    out->last_write_enter_us = t_enter;

    /*
     * acquiring hw device mutex systematically is useful if a low
     * priority thread is waiting on the output stream mutex - e.g.
     * executing out_set_parameters() while holding the hw device
     * mutex
     */
    lock_wait_us += dev_lock_acquire(adev, __func__);
    lock_wait_us += out_lock_acquire(out, __func__);

    if(adev->out_needs_standby) {
        DW("out#%u out_needs_standby is set -> forcing standby before this write", out->id);
        do_out_standby(out, "out_needs_standby flag (set by set_mode / BT SCO / HFP)");
        adev->out_needs_standby = false;
        did_standby = true;
    }

    if (out->standby) {
        uint64_t t0 = now_us();

        DI("out#%u stream is in standby, (re)starting it (hfp_active=%d)", out->id, adev->is_hfp_call_active);
        if(!adev->is_hfp_call_active) {
            ret = start_output_stream(out);
        } else {
            DW("out#%u HFP call active -> refusing to start stream", out->id);
            ret = -1;
        }
        start_us = now_us() - t0;
        did_start = true;
        if (ret != 0) {
            DE("out#%u start_output_stream FAILED ret=%d (%s) after %llu us", out->id, ret,
               strerror(ret < 0 ? -ret : ret), ULL(start_us));
            out->errors++;
            dev_lock_release(adev, __func__);
            goto exit;
        }
        out->standby = false;
    }
    dev_lock_release(adev, __func__);

//[BT SCO VoIP Call
    if(adev->in_sco_voip_call) {
        /* VoIP pcm write in celadon devices goes to bt alsa card */
        size_t frames_in = round_to_16_mult(out->pcm_config->period_size);
        size_t frames_out = round_to_16_mult(bt_out_config.period_size);
        size_t buf_size_out = bt_out_config.channels * frames_out * SAMPLE_SIZE_IN_BYTES;
        size_t buf_size_in = out->pcm_config->channels * frames_in * SAMPLE_SIZE_IN_BYTES;
        size_t buf_size_remapped = bt_out_config.channels * frames_in * SAMPLE_SIZE_IN_BYTES;
        int16_t *buf_out = (int16_t *) malloc (buf_size_out);
        int16_t *buf_in = (int16_t *) malloc (buf_size_in);
        int16_t *buf_remapped = (int16_t *) malloc (buf_size_remapped);

        DI_RL(1000, "out#%u writing through the BT SCO VoIP path", out->id);

        if(adev->voip_out_resampler == NULL) {
            int rret = create_resampler(out->pcm_config->rate /*src rate*/, bt_out_config.rate /*dst rate*/, bt_out_config.channels/*dst channels*/,
                                        RESAMPLER_QUALITY_DEFAULT, NULL, &(adev->voip_out_resampler));
            DI("frames_in %zu frames_out %zu, to write bytes: %zu", frames_in, frames_out, bytes);
            DI("size_in %zu size_out %zu size_remapped %zu", buf_size_in, buf_size_out, buf_size_remapped);

            if (rret != 0) {
                adev->voip_out_resampler = NULL;
                DE("Failure to create resampler %d", rret);

                free(buf_in);
                free(buf_out);
                free(buf_remapped);
                ret = rret;
                goto exit;
            } else {
                DI("voip_out_resampler created rate : [%d -> %d]", out->pcm_config->rate, bt_out_config.rate);
            }
        }

        memset(buf_in, 0, buf_size_in);
        memset(buf_remapped, 0, buf_size_remapped);
        memset(buf_out, 0, buf_size_out);

        memcpy(buf_in, buffer, buf_size_in);

#ifdef DEBUG_PCM_DUMP
        if(sco_call_write != NULL) {
            fwrite(buf_in, 1, buf_size_in, sco_call_write);
        }
#endif

        adjust_channels(buf_in, out->pcm_config->channels, buf_remapped, bt_out_config.channels,
                        SAMPLE_SIZE_IN_BYTES, buf_size_in);

#ifdef DEBUG_PCM_DUMP
        if(sco_call_write_remapped != NULL) {
            fwrite(buf_remapped, 1, buf_size_remapped, sco_call_write_remapped);
        }
#endif

        if(adev->voip_out_resampler != NULL) {
            adev->voip_out_resampler->resample_from_input(adev->voip_out_resampler, (int16_t *)buf_remapped, (size_t *)&frames_in, (int16_t *) buf_out, (size_t *)&frames_out);
        }

        buf_size_out = bt_out_config.channels * frames_out * SAMPLE_SIZE_IN_BYTES;
        bytes = out->pcm_config->channels * frames_in * SAMPLE_SIZE_IN_BYTES;

#ifdef DEBUG_PCM_DUMP
        if(sco_call_write_bt != NULL) {
            fwrite(buf_out, 1, buf_size_out, sco_call_write_bt);
        }
#endif

        {
            uint64_t t0 = now_us();
            ret = pcm_write(out->pcm, buf_out, buf_size_out);
            write_us = now_us() - t0;
        }

        free(buf_in);
        free(buf_out);
        free(buf_remapped);
//BT SCO VoIP Call]
    } else {
        /* Normal pcm out to primary card */
        uint64_t t0;
        unsigned int nsamp = out_frames * out->pcm_config->channels;

        for (unsigned int i = 0; i < nsamp; i++) {
            int v = out_buffer[i];
            if (v < 0)
                v = -v;
            if (v > peak)
                peak = v;
        }

        if (g_tone) {
            /* DEBUG: replace whatever AudioFlinger sent with a 440 Hz sine so we can
             * tell "Android sends silence" apart from "hardware path is silent". */
            tone_buf = malloc(out_frames * frame_size);
            if (tone_buf) {
                for (unsigned int f = 0; f < out_frames; f++) {
                    /* 440 Hz at 48 kHz repeats exactly every 48000 samples, so wrapping is seamless */
                    uint32_t n = out->tone_pos++ % 48000u;
                    int16_t s = (int16_t)(sinf(2.0f * 3.14159265f * 440.0f * (float)n / 48000.0f) * 12000.0f);
                    for (unsigned int c = 0; c < out->pcm_config->channels; c++)
                        tone_buf[f * out->pcm_config->channels + c] = s;
                }
                out_buffer = tone_buf;
                tone = true;
                peak = 12000;
            }
        }

        t0 = now_us();
        ret = pcm_write(out->pcm, out_buffer, out_frames * frame_size);
        write_us = now_us() - t0;
        free(tone_buf);
        tone_buf = NULL;

#ifdef DEBUG_PCM_DUMP
        if(out_write_dump != NULL) {
            fwrite(out_buffer, 1, out_frames * frame_size, out_write_dump);
        }
#endif

        if (ret == -EPIPE) {
            /* In case of underrun, don't sleep since we want to catch up asap */
            out->underruns++;
            out->epipe_run++;
            out->errors++;
            out->last_ret = ret;
            snprintf(pcm_err, sizeof(pcm_err), "%s", out->pcm ? pcm_get_error(out->pcm) : "pcm==NULL");
            out_lock_release(out, __func__);
            out_log_write_summary(out, bytes, out_frames, gap_us, lock_wait_us, write_us,
                                  ret, did_start, start_us, did_standby, peak, tone);
            DE("out#%u pcm_write returned -EPIPE (XRUN): underruns=%u consecutive=%u "
               "tinyalsa_err=\"%s\" -- with PCM_NORESTART the HAL never re-prepares, "
               "so every following write will fail the same way",
               out->id, out->underruns, out->epipe_run, pcm_err);
            if (out->epipe_run == 1 || (out->epipe_run % 200) == 0)
                log_failure_context(adev, "pcm_write -EPIPE");
            out->last_write_done_us = now_us();
            return ret;
        }
    }

    if (ret == 0) {
        out->written += out_frames;
        out->epipe_run = 0;
    }

    exit:
    out->last_ret = ret;
    if (ret != 0)
        snprintf(pcm_err, sizeof(pcm_err), "%s", out->pcm ? pcm_get_error(out->pcm) : "pcm==NULL");
    out_lock_release(out, __func__);
    out_log_write_summary(out, bytes, out_frames, gap_us, lock_wait_us, write_us,
                          ret, did_start, start_us, did_standby, peak, tone);

    if (ret != 0) {
        out->errors++;
        DE("out#%u out_write error: %d (%s) pcm=%p tinyalsa_err=\"%s\" errors=%u, sleeping %llu us",
           out->id, ret, strerror(ret < 0 ? -ret : ret), (void *)out->pcm,
           pcm_err, out->errors,
           ULL(bytes * 1000000ULL / audio_stream_out_frame_size(stream) /
               out_get_sample_rate(&stream->common)));
        if (out->errors == 1 || (out->errors % 200) == 0)
            log_failure_context(adev, "out_write error");
        usleep(bytes * 1000000 / audio_stream_out_frame_size(stream) /
               out_get_sample_rate(&stream->common));
    }

    out->last_write_done_us = now_us();
    return bytes;
}

static int out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    struct stream_out *out = (struct stream_out *)stream;
    *dsp_frames = out->written;
    DI_RL(1000, "out#%u dsp_frames: %u", out->id, *dsp_frames);
    return 0;
}

static int out_get_presentation_position(const struct audio_stream_out *stream,
                                         uint64_t *frames, struct timespec *timestamp)
{
    struct stream_out *out = (struct stream_out *)stream;
    int ret = -1;

    if (!out->pcm) {
        DI_RL(1000, "out#%u out->pcm is NULL (stream not started / in standby)", out->id);
        return ret;
    }

    unsigned int avail = 0;
    int hts_ret = pcm_get_htimestamp(out->pcm, &avail, timestamp);
    DI_RL(1000, "out#%u pcm_get_htimestamp returned %d, avail=%u", out->id, hts_ret, avail);

    if (hts_ret == 0) {
        unsigned int kernel_buffer_size = out->pcm_config->period_size * out->pcm_config->period_count;
        int64_t signed_frames = out->written - kernel_buffer_size + avail;

        DI_RL(1000, "out#%u written=%llu kernel_buffer_size=%u avail=%u signed_frames=%lld",
              out->id, ULL(out->written), kernel_buffer_size, avail, (long long)signed_frames);

        if (signed_frames >= 0) {
            *frames = signed_frames;
            ret = 0;
        } else {
            DI_RL(1000, "out#%u signed_frames negative, not returning position", out->id);
        }
    } else {
        DW("out#%u pcm_get_htimestamp FAILED (%d), position unavailable", out->id, hts_ret);
    }

    return ret;
}

static int out_add_audio_effect(const struct audio_stream *stream __unused, effect_handle_t effect)
{
    DI("out_add_audio_effect: %p", (void *)effect);
    return 0;
}

static int out_remove_audio_effect(const struct audio_stream *stream __unused, effect_handle_t effect)
{
    DI("out_remove_audio_effect: %p", (void *)effect);
    return 0;
}

static int out_get_next_write_timestamp(const struct audio_stream_out *stream __unused,
                                        int64_t *timestamp __unused)
{
    DI_RL(2000, "out_get_next_write_timestamp -> -ENOSYS");
    return -ENOSYS;
}

/* ======================================================================== */
/*  audio_stream_in                                                         */
/* ======================================================================== */

static uint32_t in_get_sample_rate(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;
    DI_RL(2000, "in#%u req_config rate %d", in->id, in->req_config.sample_rate);
    return in->req_config.sample_rate;
}

static int in_set_sample_rate(struct audio_stream *stream __unused, uint32_t rate)
{
    DI("in_set_sample_rate(%u) -> -ENOSYS", rate);
    return -ENOSYS;
}

static size_t in_get_buffer_size(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;
    size_t size;

    /*
     * take resampling into account and return the closest majoring
     * multiple of 16 frames, as audioflinger expects audio buffers to
     * be a multiple of 16 frames
     */
    size = (in->pcm_config->period_size * in_get_sample_rate(stream)) /
           in->pcm_config->rate;
    size = ((size + 15) / 16) * 16;

    size *= audio_stream_in_frame_size(&in->stream);
    DI_RL(2000, "in#%u buffer_size : %zu", in->id, size);
    return size;
}

static uint32_t in_get_channels(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;

    DI_RL(2000, "in#%u channels %d", in->id, popcount(in->req_config.channel_mask));
    return in->req_config.channel_mask;
}

static audio_format_t in_get_format(const struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;
    DI_RL(2000, "in#%u req_config format %d", in->id, in->req_config.format);
    return in->req_config.format;
}

static int in_set_format(struct audio_stream *stream __unused, audio_format_t format)
{
    DI("in_set_format(0x%x) -> -ENOSYS", format);
    return -ENOSYS;
}

static int in_standby(struct audio_stream *stream)
{
    struct stream_in *in = (struct stream_in *)stream;

    DI("in#%u AudioFlinger requested standby", in->id);
    dev_lock_acquire(in->dev, __func__);
    pthread_mutex_lock(&in->lock);
    do_in_standby(in, "in_standby() from AudioFlinger");
    pthread_mutex_unlock(&in->lock);
    dev_lock_release(in->dev, __func__);

    return 0;
}

static int in_dump(const struct audio_stream *stream __unused, int fd __unused)
{
    DI("in_dump");
    return 0;
}

static int in_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct stream_in *in = (struct stream_in *)stream;
    struct audio_device *adev = in->dev;
    struct str_parms *parms;
    char value[32];
    char names[200];
    int ret;
    int status = 0;
    unsigned int val;

    DI("ENTER in#%u kvpairs: %s", in->id, kvpairs);
    parms = str_parms_create_str(kvpairs);

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING,
                            value, sizeof(value));
    dev_lock_acquire(adev, __func__);
    if (ret >= 0) {
        val = atoi(value) & ~AUDIO_DEVICE_BIT_IN;
        decode_in_dev(val, names, sizeof(names));
        DI("in#%u parsed routing val=0x%x[%s], current adev->in_device=0x%x",
           in->id, val, names, adev->in_device);
        if ((adev->in_device != val) && (val != 0)) {
            DW("in#%u input routing CHANGED 0x%x -> 0x%x[%s]", in->id, adev->in_device, val, names);
            adev->in_device = val;
            select_devices(adev, "in_set_parameters(routing)");
        } else {
            DI("in#%u input routing UNCHANGED or val==0 -> skipping select_devices()", in->id);
        }
    } else {
        DI("in#%u no ROUTING key in kvpairs", in->id);
    }
    dev_lock_release(adev, __func__);

    str_parms_destroy(parms);
    DI("EXIT status=%d", status);
    return status;
}

static char * in_get_parameters(const struct audio_stream *stream,
                                const char *keys)
{
    DI_RL(2000, "keys : %s", keys);
    struct stream_in *in = (struct stream_in *)stream;
    struct str_parms *query = str_parms_create_str(keys);
    char *str_parm = NULL;
    char value[256];
    struct str_parms *reply = str_parms_create();
    int ret;

    if(reply == NULL || query == NULL) {
        if(reply != NULL) str_parms_destroy(reply);
        if(query != NULL) str_parms_destroy(query);
        return NULL;
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_FORMATS, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_FORMATS, "AUDIO_FORMAT_PCM_16_BIT");
        str_parm = str_parms_to_str(reply);
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_int(reply, AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES, in->req_config.sample_rate);

        if(str_parm != NULL)
            str_parms_destroy((struct str_parms *)str_parm);

        str_parm = str_parms_to_str(reply);
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_SUP_CHANNELS, value, sizeof(value));
    if (ret >= 0) {
        str_parms_add_str(reply, AUDIO_PARAMETER_STREAM_SUP_CHANNELS,
                          (in->req_config.channel_mask == AUDIO_CHANNEL_IN_MONO ? "AUDIO_CHANNEL_IN_MONO" : "AUDIO_CHANNEL_IN_STEREO"));

        if(str_parm != NULL)
            str_parms_destroy((struct str_parms *)str_parm);

        str_parm = str_parms_to_str(reply);
    }

    str_parms_destroy(query);
    str_parms_destroy(reply);

    DI_RL(2000, "in#%u returning keyValuePair %s", in->id, str_parm ? str_parm : "(null)");
    return str_parm;
}

static int in_set_gain(struct audio_stream_in *stream __unused, float gain)
{
    DI("in_set_gain(%f) -> 0 (ignored)", gain);
    return 0;
}

static ssize_t in_read(struct audio_stream_in *stream, void* buffer,
                       size_t bytes)
{
    int ret = 0;
    struct stream_in *in = (struct stream_in *)stream;
    struct audio_device *adev = in->dev;
    uint64_t t_enter = now_us();
    uint64_t gap_us = in->last_read_enter_us ? (t_enter - in->last_read_enter_us) : 0;
    uint64_t lock_wait_us = 0;
    uint64_t read_us = 0;
    bool did_start = false;

    in->last_read_enter_us = t_enter;

    /*
     * acquiring hw device mutex systematically is useful if a low
     * priority thread is waiting on the input stream mutex - e.g.
     * executing in_set_parameters() while holding the hw device
     * mutex
     */
    lock_wait_us += dev_lock_acquire(adev, __func__);
    pthread_mutex_lock(&in->lock);

    if(adev->in_needs_standby) {
        DW("in#%u in_needs_standby is set -> forcing standby before this read", in->id);
        do_in_standby(in, "in_needs_standby flag (set by set_mode / BT SCO / HFP)");
        adev->in_needs_standby = false;
    }

    if (in->standby) {
        DI("in#%u stream is in standby, (re)starting it (hfp_active=%d)", in->id, adev->is_hfp_call_active);
        if(!adev->is_hfp_call_active) {
            ret = start_input_stream(in);
        } else {
            DW("in#%u HFP call active -> refusing to start stream", in->id);
            ret = -1;
        }
        did_start = true;
        if (ret == 0)
            in->standby = 0;
        else
            DE("in#%u start_input_stream FAILED ret=%d", in->id, ret);
    }
    dev_lock_release(adev, __func__);

    if (ret < 0)
        goto exit;

//[BT SCO VoIP Call
    if(adev->in_sco_voip_call) {
        /* VoIP pcm read from bt alsa card */
        size_t frames_out = round_to_16_mult(in->pcm_config->period_size);
        size_t frames_in = round_to_16_mult(bt_in_config.period_size);
        size_t buf_size_out = in->pcm_config->channels * frames_out * SAMPLE_SIZE_IN_BYTES;
        size_t buf_size_in = bt_in_config.channels * frames_in * SAMPLE_SIZE_IN_BYTES;
        size_t buf_size_remapped = in->pcm_config->channels * frames_in * SAMPLE_SIZE_IN_BYTES;
        int16_t *buf_out = (int16_t *) malloc (buf_size_out);
        int16_t *buf_in = (int16_t *) malloc (buf_size_in);
        int16_t *buf_remapped = (int16_t *) malloc (buf_size_remapped);

        DI_RL(1000, "in#%u reading through the BT SCO VoIP path", in->id);

        if(adev->voip_in_resampler == NULL) {
            int rret = create_resampler(bt_in_config.rate /*src rate*/, in->pcm_config->rate /*dst rate*/, in->pcm_config->channels/*dst channels*/,
                                        RESAMPLER_QUALITY_DEFAULT, NULL, &(adev->voip_in_resampler));
            DI("bytes_requested : %zu frames_in %zu frames_out %zu", bytes, frames_in, frames_out);
            if (rret != 0) {
                adev->voip_in_resampler = NULL;
                DE("Failure to create resampler %d", rret);

                free(buf_in);
                free(buf_out);
                free(buf_remapped);
                goto exit;
            } else {
                DI("voip_in_resampler created rate : [%d -> %d]", bt_in_config.rate, in->pcm_config->rate);
            }
        }

        memset(buf_in, 0, buf_size_in);
        memset(buf_remapped, 0, buf_size_remapped);
        memset(buf_out, 0, buf_size_out);

        {
            uint64_t t0 = now_us();
            ret = pcm_read(in->pcm, buf_in, buf_size_in);
            read_us = now_us() - t0;
        }

#ifdef DEBUG_PCM_DUMP
        if(sco_call_read != NULL) {
            fwrite(buf_in, 1, buf_size_in, sco_call_read);
        }
#endif
        adjust_channels(buf_in, bt_in_config.channels, buf_remapped, in->pcm_config->channels,
                        SAMPLE_SIZE_IN_BYTES, buf_size_in);

#ifdef DEBUG_PCM_DUMP
        if(sco_call_read_remapped != NULL) {
            fwrite(buf_remapped, 1, buf_size_remapped, sco_call_read_remapped);
        }
#endif

        if(adev->voip_in_resampler != NULL) {
            adev->voip_in_resampler->resample_from_input(adev->voip_in_resampler, (int16_t *)buf_remapped, (size_t *)&frames_in, (int16_t *) buf_out, (size_t *)&frames_out);
        }

        buf_size_out = in->pcm_config->channels * frames_out * SAMPLE_SIZE_IN_BYTES;
        bytes = buf_size_out;

#ifdef DEBUG_PCM_DUMP
        if(sco_call_read_bt != NULL) {
            fwrite(buf_out, 1, buf_size_out, sco_call_read_bt);
        }
#endif

        memcpy(buffer, buf_out, buf_size_out);

        free(buf_in);
        free(buf_out);
        free(buf_remapped);
//BT SCO VoIP Call]
    } else {
        /* pcm read for primary card */
        uint64_t t0 = now_us();

        ret = pcm_read(in->pcm, buffer, bytes);
        read_us = now_us() - t0;

#ifdef DEBUG_PCM_DUMP
        if(in_read_dump != NULL) {
            fwrite(buffer, 1, bytes, in_read_dump);
        }
#endif
    }
    if (ret > 0)
        ret = 0;

    /*
     * Instead of writing zeroes here, we could trust the hardware
     * to always provide zeroes when muted.
     */
    if (ret == 0 && adev->mic_mute)
        memset(buffer, 0, bytes);

    exit:
    in->last_ret = ret;
    pthread_mutex_unlock(&in->lock);

    {
        uint32_t n = ++in->reads;
        bool anomaly = ret != 0 || did_start || lock_wait_us > 5000;

        if (ret != 0)
            in->errors++;
        if (g_log_io || anomaly || (n % 50) == 0) {
            dbg_log(anomaly ? ANDROID_LOG_WARN : ANDROID_LOG_INFO, "in_read",
                    "in#%u READ n=%u bytes=%zu gap=%.3f ms lockwait=%llu us pcm_read=%.3f ms "
                    "ret=%d mic_mute=%d%s%s",
                    in->id, n, bytes, (double)gap_us / 1000.0, ULL(lock_wait_us),
                    (double)read_us / 1000.0, ret, adev->mic_mute,
                    did_start ? " [STARTED-STREAM]" : "", ret != 0 ? " [ERROR]" : "");
        }
    }

    if (ret < 0) {
        DE("in#%u in_read error %d (%s) pcm=%p, sleeping", in->id, ret,
           strerror(-ret), (void *)in->pcm);
        usleep(bytes * 1000000 / audio_stream_in_frame_size(stream) /
               in_get_sample_rate(&stream->common));
    }

    in->last_read_done_us = now_us();
    return bytes;
}

static uint32_t in_get_input_frames_lost(struct audio_stream_in *stream __unused)
{
    DI_RL(2000, "in_get_input_frames_lost -> 0");
    return 0;
}

static int in_add_audio_effect(const struct audio_stream *stream __unused,
                               effect_handle_t effect)
{
    DI("in_add_audio_effect: %p", (void *)effect);
    return 0;
}

static int in_remove_audio_effect(const struct audio_stream *stream __unused,
                                  effect_handle_t effect)
{
    DI("in_remove_audio_effect: %p", (void *)effect);
    return 0;
}

/* ======================================================================== */
/*  Stream registry (lets the watcher see every stream, also when idle)     */
/* ======================================================================== */

static void dbg_register_out(struct audio_device *adev, struct stream_out *out)
{
    pthread_mutex_lock(&adev->dbg_lock);
    out->id = ++adev->next_stream_id;
    out->created_us = now_us();
    for (int i = 0; i < DBG_MAX_STREAMS; i++) {
        if (!adev->outs[i]) {
            adev->outs[i] = out;
            break;
        }
    }
    pthread_mutex_unlock(&adev->dbg_lock);
}

static void dbg_unregister_out(struct audio_device *adev, struct stream_out *out)
{
    pthread_mutex_lock(&adev->dbg_lock);
    for (int i = 0; i < DBG_MAX_STREAMS; i++) {
        if (adev->outs[i] == out)
            adev->outs[i] = NULL;
    }
    pthread_mutex_unlock(&adev->dbg_lock);
}

static void dbg_register_in(struct audio_device *adev, struct stream_in *in)
{
    pthread_mutex_lock(&adev->dbg_lock);
    in->id = ++adev->next_stream_id;
    in->created_us = now_us();
    for (int i = 0; i < DBG_MAX_STREAMS; i++) {
        if (!adev->ins[i]) {
            adev->ins[i] = in;
            break;
        }
    }
    pthread_mutex_unlock(&adev->dbg_lock);
}

static void dbg_unregister_in(struct audio_device *adev, struct stream_in *in)
{
    pthread_mutex_lock(&adev->dbg_lock);
    for (int i = 0; i < DBG_MAX_STREAMS; i++) {
        if (adev->ins[i] == in)
            adev->ins[i] = NULL;
    }
    pthread_mutex_unlock(&adev->dbg_lock);
}

/* ======================================================================== */
/*  Watcher thread: heartbeat + change detection, runs also when idle       */
/* ======================================================================== */

static void dbg_refresh_props(int *tick_ms, int *hb_ms, int *full_s)
{
    if (tick_ms)
        *tick_ms = prop_int("debug.audiodbg.tick_ms", 100);
    if (hb_ms)
        *hb_ms = prop_int("debug.audiodbg.hb_ms", 1000);
    if (full_s)
        *full_s = prop_int("debug.audiodbg.full_s", 30);
    g_poll = prop_int("debug.audiodbg.poll", 1);
    g_log_io = prop_int("debug.audiodbg.io", 1);
    g_codec_events = prop_int("debug.audiodbg.codec_events", 1);
    g_codec_hb = prop_int("debug.audiodbg.codec_hb", 0);
    g_tone = prop_int("debug.audiodbg.tone", 0);
}

static void dbg_heartbeat(struct audio_device *adev, uint32_t hb_no)
{
    char outn[200], inn[200], st[600], snap[1200];
    uint64_t t = now_us();

    log_pm_and_load("HB");   /* first: does not wake anything */

    decode_out_dev(adev->out_device, outn, sizeof(outn));
    decode_in_dev(adev->in_device, inn, sizeof(inn));
    DI("HB#%u adev: out_device=0x%x[%s] in_device=0x%x[%s] standby=%d mic_mute=%d hfp=%d sco=%d "
       "out_needs_standby=%d in_needs_standby=%d active_out=%p active_in=%p card=%d cardc=%d "
       "select_devices_calls=%u adev->lock_owner=%ld(%s)",
       hb_no, adev->out_device, outn, adev->in_device, inn, adev->standby, adev->mic_mute,
       adev->is_hfp_call_active, adev->in_sco_voip_call, adev->out_needs_standby,
       adev->in_needs_standby, (void *)adev->active_out, (void *)adev->active_in,
       adev->card, adev->cardc, adev->sel_count, adev->lock_owner,
       adev->lock_owner && adev->lock_fn ? adev->lock_fn : "-");

    pthread_mutex_lock(&adev->dbg_lock);
    {
        int n_out = 0, n_in = 0;

        for (int i = 0; i < DBG_MAX_STREAMS; i++) {
            struct stream_out *o = adev->outs[i];
            if (!o)
                continue;
            n_out++;
            DI("HB#%u out#%u %p: standby=%d unavailable=%d pcm=%p written=%llu writes=%u starts=%u "
               "underruns=%u epipe_run=%u errors=%u last_ret=%d last_write_age=%lld ms lock_owner=%ld",
               hb_no, o->id, (void *)o, o->standby, o->unavailable, (void *)o->pcm,
               ULL(o->written), o->writes, o->starts, o->underruns, o->epipe_run,
               o->errors, o->last_ret,
               o->last_write_enter_us ? (long long)((t - o->last_write_enter_us) / 1000) : -1LL,
               o->lock_owner);
        }
        for (int i = 0; i < DBG_MAX_STREAMS; i++) {
            struct stream_in *s = adev->ins[i];
            if (!s)
                continue;
            n_in++;
            DI("HB#%u in#%u %p: standby=%d unavailable=%d pcm=%p reads=%u starts=%u errors=%u "
               "last_ret=%d last_read_age=%lld ms",
               hb_no, s->id, (void *)s, s->standby, s->unavailable, (void *)s->pcm,
               s->reads, s->starts, s->errors, s->last_ret,
               s->last_read_enter_us ? (long long)((t - s->last_read_enter_us) / 1000) : -1LL);
        }
        DI("HB#%u streams: %d output, %d input registered", hb_no, n_out, n_in);
    }
    pthread_mutex_unlock(&adev->dbg_lock);

    if (g_poll) {
        pcm_status_compact(adev->card_dbg, 'p', st, sizeof(st));
        DI("HB#%u pcm0p: %s", hb_no, st);
        pcm_status_compact(adev->card_dbg, 'c', st, sizeof(st));
        DI("HB#%u pcm0c: %s", hb_no, st);
        mixer_snapshot(adev, snap, sizeof(snap));
        DI("HB#%u mixer: %s", hb_no, snap);
        if (g_codec_hb)
            codec_dump_key_nodes(adev->card_dbg, "heartbeat");
    } else {
        DI("HB#%u (debug.audiodbg.poll=0: not touching mixer or /proc/asound)", hb_no);
    }
    DI("HB#%u done in %llu us", hb_no, ULL(now_us() - t));
}

static void *dbg_watcher(void *arg)
{
    struct audio_device *adev = arg;
    char last_p[32] = "", last_c[32] = "", cur_p[32], cur_c[32];
    char last_snap[1200] = "", snap[1200];
    uint64_t last_hb = 0, last_full = 0;
    uint32_t hb_no = 0;
    int tick_ms = 100, hb_ms = 1000, full_s = 30;

    dbg_refresh_props(&tick_ms, &hb_ms, &full_s);
    DI("watcher thread started: tick=%d ms heartbeat=%d ms full_mixer_dump=%d s poll=%d io=%d "
       "codec_events=%d codec_hb=%d card=%d",
       tick_ms, hb_ms, full_s, g_poll, g_log_io, g_codec_events, g_codec_hb, adev->card_dbg);

    while (!adev->dbg_stop) {
        uint64_t now = now_ms();
        bool hb_due = (now - last_hb) >= (uint64_t)hb_ms;

        if (hb_due)
            dbg_refresh_props(&tick_ms, &hb_ms, &full_s);

        if (g_poll) {
            pcm_state_word(adev->card_dbg, 'p', cur_p, sizeof(cur_p));
            if (strcmp(cur_p, last_p) != 0) {
                char full[600];
                pcm_status_compact(adev->card_dbg, 'p', full, sizeof(full));
                DW("PCM0p STATE CHANGE: %s -> %s | %s", last_p[0] ? last_p : "(init)", cur_p, full);
                snprintf(last_p, sizeof(last_p), "%s", cur_p);
            }
            pcm_state_word(adev->card_dbg, 'c', cur_c, sizeof(cur_c));
            if (strcmp(cur_c, last_c) != 0) {
                DW("PCM0c STATE CHANGE: %s -> %s", last_c[0] ? last_c : "(init)", cur_c);
                snprintf(last_c, sizeof(last_c), "%s", cur_c);
            }
            mixer_snapshot(adev, snap, sizeof(snap));
            if (strcmp(snap, last_snap) != 0) {
                DW("MIXER CHANGED (if no select_devices ENTER line is just above, something "
                   "OTHER than this HAL changed it)");
                DW("  old: %s", last_snap[0] ? last_snap : "(init)");
                DW("  new: %s", snap);
                snprintf(last_snap, sizeof(last_snap), "%s", snap);
            }
        }

        {
            long owner = adev->lock_owner;
            uint64_t since = adev->lock_since_us;
            if (owner && since && now_us() - since > 200000)
                DI_RL(1000, "STALL: adev->lock held %llu ms by tid=%ld in %s",
                      ULL((now_us() - since) / 1000), owner,
                      adev->lock_fn ? adev->lock_fn : "?");
        }

        if (hb_due) {
            last_hb = now;
            dbg_heartbeat(adev, ++hb_no);
            if (g_poll && (now - last_full) / 1000 >= (uint64_t)full_s) {
                last_full = now;
                mixer_full_dump(adev, "periodic");
            }
        }

        usleep((useconds_t)(tick_ms > 10 ? tick_ms : 10) * 1000);
    }
    DI("watcher thread stopping");
    return NULL;
}

/* ======================================================================== */
/*  audio_hw_device                                                         */
/* ======================================================================== */

static int adev_open_output_stream(struct audio_hw_device *dev,
                                   audio_io_handle_t handle,
                                   audio_devices_t devices,
                                   audio_output_flags_t flags,
                                   struct audio_config *config,
                                   struct audio_stream_out **stream_out,
                                   const char *address)
{
    char names[200];
    struct audio_device *adev = (struct audio_device *)dev;
    struct stream_out *out;
    struct pcm_params *params = NULL;

    decode_out_dev(devices, names, sizeof(names));
    DI("ENTER handle=%d devices=0x%x[%s] flags=0x%x address=\"%s\" requested config: "
       "[rate %d format 0x%x channels %d mask 0x%x]",
       (int)handle, (unsigned int)devices, names, (unsigned int)flags,
       address ? address : "", config->sample_rate, config->format,
       popcount(config->channel_mask), config->channel_mask);

    adev->card = get_pcm_card("PCH");
    if (adev->card != -1)
        params = pcm_params_get(adev->card, PCM_DEVICE, PCM_OUT);
    else {
        adev->card = get_pcm_card("Intel");
        if (adev->card != -1)
            params = pcm_params_get(adev->card, PCM_DEVICE, PCM_OUT);
        else {
            adev->card = get_pcm_card("sofhdadsp");
            if (adev->card != -1)
                params = pcm_params_get(adev->card, PCM_DEVICE, PCM_OUT);
            else {
                adev->card = get_pcm_card("Dummy");
                params = pcm_params_get(adev->card, PCM_DEVICE, PCM_OUT);
            }
        }
    }

    if (!params) {
        DW("no pcm params for card %d, falling back to Dummy card", adev->card);
        adev->card = get_pcm_card("Dummy");
        params = pcm_params_get(adev->card, PCM_DEVICE, PCM_OUT);
        if (!params) {
            DE("no usable playback card -> -ENOSYS");
            return -ENOSYS;
        }
    }

    DI("PCM playback card selected = %d", adev->card);
    if (adev->card >= 0)
        adev->card_dbg = adev->card;
    log_pcm_params("playback", params, &pcm_config_out);

    trace_pcm_node_access(adev->card, PCM_DEVICE);
    out = (struct stream_out *)calloc(1, sizeof(struct stream_out));
    if (!out) {
        free(params);
        return -ENOMEM;
    }

    out->stream.common.get_sample_rate = out_get_sample_rate;
    out->stream.common.set_sample_rate = out_set_sample_rate;
    out->stream.common.get_buffer_size = out_get_buffer_size;
    out->stream.common.get_channels = out_get_channels;
    out->stream.common.get_format = out_get_format;
    out->stream.common.set_format = out_set_format;
    out->stream.common.standby = out_standby;
    out->stream.common.dump = out_dump;
    out->stream.common.set_parameters = out_set_parameters;
    out->stream.common.get_parameters = out_get_parameters;
    out->stream.common.add_audio_effect = out_add_audio_effect;
    out->stream.common.remove_audio_effect = out_remove_audio_effect;
    out->stream.get_latency = out_get_latency;
    out->stream.set_volume = out_set_volume;
    out->stream.write = out_write;
    out->stream.get_render_position = out_get_render_position;
    out->stream.get_next_write_timestamp = out_get_next_write_timestamp;
    out->stream.get_presentation_position = out_get_presentation_position;

    out->pcm_config = &pcm_config_out;

    out->written = 0;

// VTS : Device doesn't support mono channel or sample_rate other than 48000
//       make a copy of requested config to feed it back if requested.
    memcpy(&out->req_config, config, sizeof(struct audio_config));

    out->dev = adev;
    out->standby = true;
    out->unavailable = false;

    out->content_state = -1;
    dbg_register_out(adev, out);

    config->format = out_get_format(&out->stream.common);
    config->channel_mask = out_get_channels(&out->stream.common);
    config->sample_rate = out_get_sample_rate(&out->stream.common);

    *stream_out = &out->stream;

    free(params);

    DI("EXIT created out#%u %p (standby, PCM not opened until first write); "
       "HAL reports config back: [rate %d format 0x%x mask 0x%x]",
       out->id, (void *)out, config->sample_rate, config->format, config->channel_mask);
    return 0;
}

static void adev_close_output_stream(struct audio_hw_device *dev,
                                     struct audio_stream_out *stream)
{
    struct audio_device *adev = (struct audio_device *)dev;
    struct stream_out *out = (struct stream_out *)stream;

    DW("out#%u CLOSING stream: writes=%u underruns=%u errors=%u starts=%u written=%llu frames",
       out->id, out->writes, out->underruns, out->errors, out->starts, ULL(out->written));
    out_standby(&stream->common);
    dbg_unregister_out(adev, out);
    free(stream);
}

//called with adev lock
static void stop_existing_output_input(struct audio_device *adev, const char *why){
    DW("forcing out/in standby (%s)", why);
    adev->in_needs_standby = true;
    adev->out_needs_standby = true;
}

static int adev_set_parameters(struct audio_hw_device *dev, const char *kvpairs)
{
    DI("ENTER kvpairs: %s", kvpairs);

    struct audio_device * adev = (struct audio_device *)dev;
    char value[32];
    int ret;
    struct str_parms *parms;

    parms = str_parms_create_str(kvpairs);

    if(parms == NULL) {
        DW("str_parms_create_str failed");
        return 0;
    }

    ret = str_parms_get_str(parms, AUDIO_PARAMETER_HFP_ENABLE, value, sizeof(value));
    if (ret >= 0) {
        DI("hfp_enable=%s", value);
        dev_lock_acquire(adev, __func__);
        if (strcmp(value, "true") == 0){
            stop_existing_output_input(adev, "hfp_enable=true");
            adev->is_hfp_call_active = true;
        } else {
            adev->is_hfp_call_active = false;
        }
        dev_lock_release(adev, __func__);
    }

//[BT SCO VoIP Call
    ret = str_parms_get_str(parms, AUDIO_PARAMETER_BT_SCO, value, sizeof(value));
    if (ret >= 0) {
        DI("BT_SCO=%s", value);
        dev_lock_acquire(adev, __func__);
        if (strcmp(value, "on") == 0){
            adev->in_sco_voip_call = true;
            stop_existing_output_input(adev, "BT_SCO=on");
        } else {
            adev->in_sco_voip_call = false;
            stop_existing_output_input(adev, "BT_SCO=off");

            release_resampler(adev->voip_in_resampler);
            adev->voip_in_resampler = NULL;
            release_resampler(adev->voip_out_resampler);
            adev->voip_out_resampler = NULL;
        }
        dev_lock_release(adev, __func__);
    }
//BT SCO VoIP Call]

    str_parms_destroy(parms);
    DI("EXIT");
    return 0;
}

static char * adev_get_parameters(const struct audio_hw_device *dev __unused,
                                  const char *keys)
{
    DI_RL(2000, "keys : %s", keys);
    struct str_parms *query = str_parms_create_str(keys);
    char value[256];
    int ret;

    if(query == NULL) {
        return NULL;
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_STREAM_HW_AV_SYNC, value, sizeof(value));
    if (ret >= 0) {
        str_parms_destroy(query);
        return NULL;
    }

    ret = str_parms_get_str(query, AUDIO_PARAMETER_KEY_TTY_MODE, value, sizeof(value));
    if(ret >= 0) {
        DE("no support of TTY");
        str_parms_destroy(query);
        return NULL;
    }

    str_parms_destroy(query);
    return strdup(keys);
}

static int adev_init_check(const struct audio_hw_device *dev __unused)
{
    DI_RL(2000, "adev_init_check -> 0");
    return 0;
}

//Supported vol range [0:1], return OK for inrange volume request
static int adev_set_voice_volume(struct audio_hw_device *dev __unused, float volume)
{
    int32_t ret = 0;

    if(volume < 0.0f){
        ret = -EINVAL;
    }
    DI("adev_set_voice_volume: %f -> %d (platform has no such handling)", volume, ret);

    return ret;
}

static int adev_set_master_volume(struct audio_hw_device *dev __unused, float volume)
{
    DI("adev_set_master_volume: %f -> -ENOSYS", volume);
    return -ENOSYS;
}

static int adev_get_master_volume(struct audio_hw_device *dev __unused, float *volume __unused)
{
    DI_RL(2000, "adev_get_master_volume -> -ENOSYS");
    return -ENOSYS;
}

static int adev_set_master_mute(struct audio_hw_device *dev __unused, bool muted)
{
    DI("adev_set_master_mute: %d -> -ENOSYS", muted);
    return -ENOSYS;
}

static int adev_get_master_mute(struct audio_hw_device *dev __unused, bool *muted __unused)
{
    DI_RL(2000, "adev_get_master_mute -> -ENOSYS");
    return -ENOSYS;
}

static int adev_set_mode(struct audio_hw_device *dev, audio_mode_t mode)
{
    struct audio_device *adev = (struct audio_device *)dev;

    DW("mode=%d(%s)", (int)mode, mode_name(mode));

    dev_lock_acquire(adev, __func__);
    stop_existing_output_input(adev, "adev_set_mode");
    dev_lock_release(adev, __func__);

    return 0;
}

static int adev_set_mic_mute(struct audio_hw_device *dev, bool state)
{
    struct audio_device *adev = (struct audio_device *)dev;
    DI("adev_set_mic_mute: %d", state);
    adev->mic_mute = state;
    return 0;
}

static int adev_get_mic_mute(const struct audio_hw_device *dev, bool *state)
{
    struct audio_device *adev = (struct audio_device *)dev;
    *state = adev->mic_mute;
    DI_RL(2000, "adev_get_mic_mute -> %d", *state);
    return 0;
}

static size_t adev_get_input_buffer_size(const struct audio_hw_device *dev __unused,
                                         const struct audio_config *config)
{
    size_t size;

    /*
     * take resampling into account and return the closest majoring
     * multiple of 16 frames, as audioflinger expects audio buffers to
     * be a multiple of 16 frames
     */
    size = (pcm_config_in.period_size * config->sample_rate) / pcm_config_in.rate;
    size = ((size + 15) / 16) * 16;

    size = (size * popcount(config->channel_mask) *
            audio_bytes_per_sample(config->format));
    DI_RL(2000, "adev_get_input_buffer_size(rate=%d ch=%d fmt=0x%x) -> %zu",
          config->sample_rate, popcount(config->channel_mask), config->format, size);
    return size;
}

static int adev_open_input_stream(struct audio_hw_device *dev,
                                  audio_io_handle_t handle,
                                  audio_devices_t devices,
                                  struct audio_config *config,
                                  struct audio_stream_in **stream_in,
                                  audio_input_flags_t flags,
                                  const char *address,
                                  audio_source_t source)

{
    char names[200];
    struct audio_device *adev = (struct audio_device *)dev;
    struct stream_in *in;
    struct pcm_params *params = NULL;

    decode_in_dev((uint32_t)devices & ~AUDIO_DEVICE_BIT_IN, names, sizeof(names));
    DI("ENTER handle=%d devices=0x%x[%s] flags=0x%x source=%d address=\"%s\" requested config: "
       "[rate %d format 0x%x channels %d mask 0x%x]",
       (int)handle, (unsigned int)devices, names, (unsigned int)flags, (int)source,
       address ? address : "", config->sample_rate, config->format,
       popcount(config->channel_mask), config->channel_mask);

    *stream_in = NULL;

    adev->cardc = get_pcm_card("PCH");
    if (adev->cardc != -1)
        params = pcm_params_get(adev->cardc, PCM_DEVICE, PCM_IN);
    else {
        adev->cardc = get_pcm_card("Intel");
        if (adev->cardc != -1)
            params = pcm_params_get(adev->cardc, PCM_DEVICE, PCM_IN);
        else {
            adev->cardc = get_pcm_card("sofhdadsp");
            if (adev->cardc != -1)
                params = pcm_params_get(adev->cardc, PCM_DEVICE, PCM_IN);
            else {
                adev->cardc = get_pcm_card("Dummy");
                params = pcm_params_get(adev->cardc, PCM_DEVICE, PCM_IN);
            }
        }
    }
    if(!params) {
        DW("no capture pcm params for card %d, falling back to Dummy card", adev->cardc);
        adev->cardc = get_pcm_card("Dummy");
        params = pcm_params_get(adev->cardc, PCM_DEVICE, PCM_IN);
        if (!params) {
            DE("no usable capture card -> -ENOSYS");
            return -ENOSYS;
        }
    }
    DI("PCM capture card selected = %d", adev->cardc);
    log_pcm_params("capture", params, &pcm_config_in);

    in = (struct stream_in *)calloc(1, sizeof(struct stream_in));
    if (!in) {
        free(params);
        return -ENOMEM;
    }

    in->stream.common.get_sample_rate = in_get_sample_rate;
    in->stream.common.set_sample_rate = in_set_sample_rate;
    in->stream.common.get_buffer_size = in_get_buffer_size;
    in->stream.common.get_channels = in_get_channels;
    in->stream.common.get_format = in_get_format;
    in->stream.common.set_format = in_set_format;
    in->stream.common.standby = in_standby;
    in->stream.common.dump = in_dump;
    in->stream.common.set_parameters = in_set_parameters;
    in->stream.common.get_parameters = in_get_parameters;
    in->stream.common.add_audio_effect = in_add_audio_effect;
    in->stream.common.remove_audio_effect = in_remove_audio_effect;
    in->stream.set_gain = in_set_gain;
    in->stream.read = in_read;
    in->stream.get_input_frames_lost = in_get_input_frames_lost;

    in->dev = adev;
    in->standby = true;

    in->pcm_config = &pcm_config_in; /* default PCM config */

// VTS : Device doesn't support mono channel or sample_rate other than 48000
//       make a copy of requested config to feed it back if requested.
    memcpy(&in->req_config, config, sizeof(struct audio_config));

    dbg_register_in(adev, in);

    *stream_in = &in->stream;

    free(params);
    DI("EXIT created in#%u %p (standby, PCM not opened until first read)", in->id, (void *)in);
    return 0;
}

static void adev_close_input_stream(struct audio_hw_device *dev,
                                    struct audio_stream_in *stream)
{
    struct audio_device *adev = (struct audio_device *)dev;
    struct stream_in *in = (struct stream_in *)stream;

    DW("in#%u CLOSING stream: reads=%u errors=%u starts=%u", in->id, in->reads, in->errors, in->starts);
    in_standby(&stream->common);
    dbg_unregister_in(adev, in);
    free(stream);
}

static int adev_dump(const audio_hw_device_t *device __unused, int fd __unused)
{
    DI("adev_dump");
    return 0;
}

static int adev_get_microphones(const audio_hw_device_t *device __unused, struct audio_microphone_characteristic_t *mic_array, size_t *actual_mics)
{
    int32_t ret = 0;
    *actual_mics = 1;
    memset(&mic_array[0], 0, sizeof(mic_array[0]));
    DI("adev_get_microphones -> 1 (zeroed) mic");

    return ret;
}

static int adev_close(hw_device_t *device)
{
    struct audio_device *adev = (struct audio_device *)device;

    DW("adev_close: HAL device closing (audioserver restart or shutdown)");

    if (adev->dbg_thread_started) {
        adev->dbg_stop = true;
        pthread_join(adev->dbg_thread, NULL);
        adev->dbg_thread_started = false;
    }
    if (adev->dbg_mixer) {
        mixer_close(adev->dbg_mixer);
        adev->dbg_mixer = NULL;
    }

    audio_route_free(adev->ar);

#ifdef DEBUG_PCM_DUMP
    if(sco_call_write != NULL) {
        fclose(sco_call_write);
    }
    if(sco_call_write_remapped != NULL) {
        fclose(sco_call_write_remapped);
    }
    if(sco_call_write_bt != NULL) {
        fclose(sco_call_write_bt);
    }
    if(sco_call_read != NULL) {
        fclose(sco_call_read);
    }
    if(sco_call_read_remapped != NULL) {
        fclose(sco_call_read_remapped);
    }
    if(sco_call_read_bt != NULL) {
        fclose(sco_call_read_bt);
    }
    if(out_write_dump != NULL) {
        fclose(out_write_dump);
    }
    if(in_read_dump != NULL) {
        fclose(in_read_dump);
    }
#endif

    free(device);
    return 0;
}

static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device)
{
    struct audio_device *adev;
    int card = 0;
    char mixer_path[PATH_MAX];

    dbg_refresh_props(NULL, NULL, NULL);
    DI("=================== AudioDBG HAL loaded ===================");
    DI("pid=%d tid=%ld name=\"%s\" (verbose AudioDBG build)", (int)getpid(),
       (long)gettid(), name);
    DI("knobs: poll=%d io=%d codec_events=%d codec_hb=%d (see header comment for setprop names)",
       g_poll, g_log_io, g_codec_events, g_codec_hb);

    if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    adev = calloc(1, sizeof(struct audio_device));
    if (!adev)
        return -ENOMEM;

    adev->hw_device.common.tag = HARDWARE_DEVICE_TAG;
    adev->hw_device.common.version = AUDIO_DEVICE_API_VERSION_2_0;
    adev->hw_device.common.module = (struct hw_module_t *) module;
    adev->hw_device.common.close = adev_close;
    adev->hw_device.init_check = adev_init_check;
    adev->hw_device.set_voice_volume = adev_set_voice_volume;
    adev->hw_device.set_master_volume = adev_set_master_volume;
    adev->hw_device.get_master_volume = adev_get_master_volume;
    adev->hw_device.set_master_mute = adev_set_master_mute;
    adev->hw_device.get_master_mute = adev_get_master_mute;
    adev->hw_device.set_mode = adev_set_mode;
    adev->hw_device.set_mic_mute = adev_set_mic_mute;
    adev->hw_device.get_mic_mute = adev_get_mic_mute;
    adev->hw_device.set_parameters = adev_set_parameters;
    adev->hw_device.get_parameters = adev_get_parameters;
    adev->hw_device.get_input_buffer_size = adev_get_input_buffer_size;
    adev->hw_device.open_output_stream = adev_open_output_stream;
    adev->hw_device.close_output_stream = adev_close_output_stream;
    adev->hw_device.open_input_stream = adev_open_input_stream;
    adev->hw_device.close_input_stream = adev_close_input_stream;
    adev->hw_device.dump = adev_dump;
    adev->hw_device.get_microphones = adev_get_microphones;

    card = get_pcm_card("PCH");
    if (card == -1 )
        card = get_pcm_card("Intel");
    if (card == -1 )
        card = get_pcm_card("sofhdadsp");
    if (card == -1 )
        card = get_pcm_card("Dummy");
    adev->card_dbg = card >= 0 ? card : 0;
    DI("using card %d for mixer (card_dbg=%d)", card, adev->card_dbg);

    dump_proc_file("ALSA cards", "/proc/asound/cards");
    dump_proc_file("ALSA pcm list", "/proc/asound/pcm");
    dump_proc_file("ALSA devices", "/proc/asound/devices");

    snprintf(mixer_path,PATH_MAX,"/vendor/etc/mixer_paths_0.xml");
    DI("audio_route_init(card=%d, \"%s\")", card, mixer_path);
    adev->ar = audio_route_init(card, mixer_path);
    if (!adev->ar) {
        DE("Failed to init audio route controls for card %d, aborting.", card);
        goto error;
    }

    if (card >= 0)
        adev->dbg_mixer = mixer_open((unsigned int)card);
    DI("debug mixer handle: %p (%s)", (void *)adev->dbg_mixer,
       adev->dbg_mixer ? "ok" : "mixer_open failed, mixer snapshots disabled");
    mixer_full_dump(adev, "adev_open (state saved by audio_route as the RESET state)");

    adev->out_device = AUDIO_DEVICE_OUT_SPEAKER;
    adev->in_device = AUDIO_DEVICE_IN_BUILTIN_MIC & ~AUDIO_DEVICE_BIT_IN;
    select_devices(adev, "adev_open");   /* apply initial routing now, don't wait for a "change" */;

    *device = &adev->hw_device.common;

// CLK target codec only works with sample_rate 48000, identify the target and update default pcm_config.rate if needed.
    char product[PROPERTY_VALUE_MAX] = "cel_kbl";
    if(property_get("ro.hardware", product, NULL) <= 0) {
        DE("failed to read ro.hardware");
    } else {
        if(strcmp(product, "clk") == 0) {
            pcm_config_in.rate = 48000;
        }
    }

//Update period_size based on sample rate and period_ms
    size_t size = (pcm_config_in.rate * IN_PERIOD_MS * SAMPLE_SIZE_IN_BYTES_STEREO) / 1000;
    pcm_config_in.period_size = size;

    DI("will use input [rate : period] as [%d : %u] for %s variants", pcm_config_in.rate, pcm_config_in.period_size, product);

//[BT SCO VoIP Call
    update_bt_card(adev);

    adev->in_sco_voip_call = false;
    adev->is_hfp_call_active = false;
    adev->voip_in_resampler = NULL;
    adev->voip_out_resampler = NULL;
//BT SCO VoIP Call]

    adev->in_needs_standby = false;
    adev->out_needs_standby = false;

#ifdef DEBUG_PCM_DUMP
    sco_call_write = fopen("/vendor/dump/sco_call_write.pcm", "a");
    sco_call_write_remapped = fopen("/vendor/dump/sco_call_write_remapped.pcm", "a");
    sco_call_write_bt = fopen("/vendor/dump/sco_call_write_bt.pcm", "a");
    sco_call_read = fopen("/vendor/dump/sco_call_read.pcm", "a");
    sco_call_read_remapped = fopen("/vendor/dump/sco_call_read_remapped.pcm", "a");
    sco_call_read_bt = fopen("/vendor/dump/sco_call_read_bt.pcm", "a");
    out_write_dump = fopen("/vendor/dump/out_write_dump.pcm", "a");
    in_read_dump = fopen("/vendor/dump/in_read_dump.pcm", "a");

    if(sco_call_write == NULL || sco_call_write_remapped == NULL || sco_call_write_bt == NULL || sco_call_read == NULL ||
            sco_call_read_bt == NULL || sco_call_read_remapped == NULL || out_write_dump == NULL || in_read_dump == NULL)
        DI("failed to open dump files");
    else
        DI("success in opening dump files");
#endif

    if (pthread_create(&adev->dbg_thread, NULL, dbg_watcher, adev) == 0) {
        adev->dbg_thread_started = true;
    } else {
        DE("failed to start watcher thread: %s", strerror(errno));
    }

    DI("adev_open done, HAL ready");
    return 0;

    error:
    free(adev);
    return -ENODEV;
}

static struct hw_module_methods_t hal_module_methods = {
        .open = adev_open,
};

struct audio_module HAL_MODULE_INFO_SYM = {
        .common = {
                .tag = HARDWARE_MODULE_TAG,
                .module_api_version = AUDIO_MODULE_API_VERSION_0_1,
                .hal_api_version = HARDWARE_HAL_API_VERSION,
                .id = AUDIO_HARDWARE_MODULE_ID,
                .name = "Android IA minimal HW HAL (AudioDBG verbose build)",
                .author = "The Android Open Source Project",
                .methods = &hal_module_methods,
        },
};