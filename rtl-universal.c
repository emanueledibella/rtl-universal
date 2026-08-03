// rtl-universal.c
// Main entry-point: selects the protocol module, asks the module which
// demodulator it needs, then forwards raw RTL-SDR IQ samples to that
// demodulator. All protocol-specific DSP lives outside this file.

#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <rtl-sdr.h>

#include "modules/header/ais_decoder.h"
#include "modules/header/adsb_decoder.h"
#include "modules/header/adsb_protocol.h"
#include "modules/header/voice_module.h"
#include "modules/header/sonde_module.h"
#include "modules/header/sstv_module.h"
#include "modules/header/meteor_module.h"
#include "demod/header/demodulator.h"
#include "utility/dashboard.h"
#include "utility/output.h"

static volatile int g_stop = 0;
static rtlsdr_dev_t *g_dev = NULL;
static demodulator_t g_demod;

static voice_module_t g_voice;
static ais_receiver_t g_ais;
static adsb_ctx_t g_adsb;
static sonde_module_t g_sonde;
static sstv_module_t g_sstv;
static meteor_module_t g_meteor;

static int streq_icase(const char *a, const char *b);

typedef enum {
    INPUT_FORMAT_AUTO = 0,
    INPUT_FORMAT_NMEA,
    INPUT_FORMAT_AVR,
    INPUT_FORMAT_BEAST,
    INPUT_FORMAT_IQ_U8
} input_format_t;

typedef struct {
    const char *module_name;
    double stats_interval;
    struct timespec started;
    struct timespec last_report;
    uint64_t iq_pairs;
    uint64_t clipped_components;
    double power_sum;
    uint64_t previous_valid;
    uint64_t previous_candidates;
} live_stats_t;

static live_stats_t g_live_stats;

static double timespec_delta(const struct timespec *newer,
                             const struct timespec *older) {
    return (double)(newer->tv_sec - older->tv_sec)
           + (double)(newer->tv_nsec - older->tv_nsec) / 1e9;
}

static void live_stats_maybe_report(live_stats_t *stats, int force) {
    struct timespec now;
    double elapsed;
    double signal_dbfs = -INFINITY;
    double clipping = 0.0;
    uint64_t valid = 0u;
    uint64_t rejected = 0u;
    uint64_t crc_errors = 0u;
    uint64_t candidates = 0u;
    uint64_t non_adsb = 0u;
    uint64_t quality_rejected = 0u;

    if (!stats || stats->stats_interval <= 0.0) return;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return;
    elapsed = timespec_delta(&now, &stats->last_report);
    if (!force && elapsed < stats->stats_interval) return;
    if (elapsed <= 0.0) elapsed = stats->stats_interval;
    if (stats->iq_pairs > 0u) {
        double normalized = stats->power_sum
                            / ((double)stats->iq_pairs * 2.0 * 127.5 * 127.5);
        if (normalized > 0.0) signal_dbfs = 10.0 * log10(normalized);
        clipping = 100.0 * (double)stats->clipped_components
                   / (double)(stats->iq_pairs * 2u);
    }
    if (streq_icase(stats->module_name, "adsb")) {
        valid = g_adsb.valid_frame_count;
        rejected = g_adsb.rejected_frame_count;
        crc_errors = g_adsb.crc_error_count;
        candidates = g_adsb.detected_preamble_count;
        non_adsb = g_adsb.non_adsb_candidate_count;
        quality_rejected = g_adsb.quality_rejected_count;
    } else if (streq_icase(stats->module_name, "ais")) {
        for (unsigned int i = 0; i < g_ais.channel_count; i++) {
            valid += g_ais.channel[i].frames_valid;
            rejected += g_ais.channel[i].crc_errors + g_ais.channel[i].aborted_frames
                        + g_ais.channel[i].overflow_frames;
            crc_errors += g_ais.channel[i].crc_errors;
        }
    } else if (streq_icase(stats->module_name, "sonde")) {
        valid = g_sonde.frames_valid;
        rejected = g_sonde.rejected_frames;
        crc_errors = g_sonde.crc_errors;
        candidates = g_sonde.frame_candidates;
    } else if (streq_icase(stats->module_name, "sstv")) {
        valid = g_sstv.images_saved;
        rejected = g_sstv.rejected_headers;
        candidates = g_sstv.vis_headers;
    }
    if (output_is_dashboard()) {
        dashboard_update_stats(
            signal_dbfs, clipping,
            (double)(valid - stats->previous_valid) / elapsed,
            (double)(candidates - stats->previous_candidates) / elapsed,
            valid, crc_errors, non_adsb, quality_rejected,
            rejected >= crc_errors ? rejected - crc_errors : 0u);
    } else if (streq_icase(stats->module_name, "adsb")) {
        fprintf(stderr,
                "[RX] state=receiving signal=%.1f dBFS clipping=%.2f%% frame_rate=%.2f/s candidate_rate=%.2f/s valid=%llu crc_errors=%llu non_adsb_candidates=%llu quality_rejected=%llu\n",
                signal_dbfs, clipping,
                (double)(valid - stats->previous_valid) / elapsed,
                (double)(candidates - stats->previous_candidates) / elapsed,
                (unsigned long long)valid, (unsigned long long)crc_errors,
                (unsigned long long)non_adsb,
                (unsigned long long)quality_rejected);
    } else {
        fprintf(stderr,
                "[RX] state=receiving signal=%.1f dBFS clipping=%.2f%% frame_rate=%.2f/s valid=%llu crc_errors=%llu demod_or_other_rejected=%llu\n",
                signal_dbfs, clipping,
                (double)(valid - stats->previous_valid) / elapsed,
                (unsigned long long)valid, (unsigned long long)crc_errors,
                (unsigned long long)(rejected >= crc_errors ? rejected - crc_errors : 0u));
    }
    stats->last_report = now;
    stats->previous_valid = valid;
    stats->previous_candidates = candidates;
    stats->iq_pairs = 0u;
    stats->clipped_components = 0u;
    stats->power_sum = 0.0;
}

typedef struct {
    const char *name;
    void *ctx;
    int (*init)(void *ctx, const demod_config_t *cfg);
    void (*fill_demod_config)(void *ctx, demod_config_t *cfg);
    demod_output_t (*get_demod_output)(void *ctx);
    void (*flush)(void *ctx);
    int (*run_test)(void *ctx);
} module_ops_t;

static void on_sigint(int sig) {
    (void)sig;
    g_stop = 1;
    if (g_dev) rtlsdr_cancel_async(g_dev);
    meteor_module_stop();
}

static int streq_icase(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static const char *demod_kind_name(demod_kind_t kind) {
    switch (kind) {
    case DEMOD_KIND_AM:
        return "am";
    case DEMOD_KIND_FM:
        return "fm";
    case DEMOD_KIND_GMSK:
        return "gmsk";
    case DEMOD_KIND_NONE:
    default:
        return "none";
    }
}

static void rtlsdr_cb(unsigned char *buf, uint32_t len, void *ctx) {
    live_stats_t *stats = (live_stats_t *)ctx;
    if (g_stop) return;
    if (stats) {
        for (uint32_t p = 0; p + 1u < len; p += 2u) {
            double i = (double)buf[p] - 127.5;
            double q = (double)buf[p + 1u] - 127.5;
            stats->power_sum += i * i + q * q;
            stats->iq_pairs++;
            if (buf[p] <= 1u || buf[p] >= 254u) stats->clipped_components++;
            if (buf[p + 1u] <= 1u || buf[p + 1u] >= 254u) stats->clipped_components++;
        }
    }
    demodulator_process_raw_iq_u8(&g_demod, buf, len);
    live_stats_maybe_report(stats, 0);
}

static int voice_init_module(void *ctx, const demod_config_t *cfg) {
    return voice_module_init((voice_module_t *)ctx, cfg);
}

static void voice_fill_demod_config_module(void *ctx, demod_config_t *cfg) {
    voice_module_get_demod_config((voice_module_t *)ctx, cfg);
}

static demod_output_t voice_get_demod_output_module(void *ctx) {
    return voice_module_get_demod_output((voice_module_t *)ctx);
}

static void voice_flush_module(void *ctx) {
    voice_module_flush((voice_module_t *)ctx);
}

static int ais_init_module(void *ctx, const demod_config_t *cfg) {
    (void)cfg;
    ais_receiver_init((ais_receiver_t *)ctx);
    return 1;
}

static void ais_fill_demod_config_module(void *ctx, demod_config_t *cfg) {
    ais_receiver_get_demod_config((ais_receiver_t *)ctx, cfg);
}

static demod_output_t ais_get_demod_output_module(void *ctx) {
    return ais_receiver_get_demod_output((ais_receiver_t *)ctx);
}

static void ais_flush_module(void *ctx) {
    ais_receiver_flush((ais_receiver_t *)ctx);
}

static int ais_run_test_module(void *ctx) {
    return ais_receiver_test((ais_receiver_t *)ctx);
}

static int adsb_init_module(void *ctx, const demod_config_t *cfg) {
    init((adsb_ctx_t *)ctx, cfg ? cfg->output_fs : 0);
    return 1;
}

static void adsb_fill_demod_config_module(void *ctx, demod_config_t *cfg) {
    (void)ctx;
    get_demod_config(cfg);
}

static demod_output_t adsb_get_demod_output_module(void *ctx) {
    return get_demod_output((adsb_ctx_t *)ctx);
}

static void adsb_flush_module(void *ctx) {
    flush((adsb_ctx_t *)ctx);
}

static int adsb_run_test_module(void *ctx) {
    return adsb_test_emit_examples((adsb_ctx_t *)ctx);
}

static int sonde_init_module(void *ctx, const demod_config_t *cfg) {
    return sonde_module_init((sonde_module_t *)ctx, cfg);
}

static void sonde_fill_demod_config_module(void *ctx, demod_config_t *cfg) {
    sonde_module_get_demod_config((sonde_module_t *)ctx, cfg);
}

static demod_output_t sonde_get_demod_output_module(void *ctx) {
    return sonde_module_get_demod_output((sonde_module_t *)ctx);
}

static void sonde_flush_module(void *ctx) {
    sonde_module_flush((sonde_module_t *)ctx);
}

static int sonde_run_test_module(void *ctx) {
    return sonde_module_run_test((sonde_module_t *)ctx);
}

static int sstv_init_module(void *ctx, const demod_config_t *cfg) {
    return sstv_module_init((sstv_module_t *)ctx, cfg);
}

static void sstv_fill_demod_config_module(void *ctx, demod_config_t *cfg) {
    sstv_module_get_demod_config((sstv_module_t *)ctx, cfg);
}

static demod_output_t sstv_get_demod_output_module(void *ctx) {
    return sstv_module_get_demod_output((sstv_module_t *)ctx);
}

static void sstv_flush_module(void *ctx) {
    sstv_module_flush((sstv_module_t *)ctx);
}

static int sstv_run_test_module(void *ctx) {
    return sstv_module_run_test((sstv_module_t *)ctx);
}

static int meteor_run_test_module(void *ctx) {
    return meteor_module_run_test((meteor_module_t *)ctx);
}

static const module_ops_t g_modules[] = {
    { "voice", &g_voice, voice_init_module, voice_fill_demod_config_module, voice_get_demod_output_module, voice_flush_module, NULL                 },
    { "ais",   &g_ais,   ais_init_module,   ais_fill_demod_config_module,   ais_get_demod_output_module,   ais_flush_module,   ais_run_test_module  },
    { "adsb",  &g_adsb,  adsb_init_module,  adsb_fill_demod_config_module,  adsb_get_demod_output_module,  adsb_flush_module,  adsb_run_test_module },
    { "sonde", &g_sonde, sonde_init_module, sonde_fill_demod_config_module, sonde_get_demod_output_module, sonde_flush_module, sonde_run_test_module },
    { "sstv",  &g_sstv,  sstv_init_module,  sstv_fill_demod_config_module,  sstv_get_demod_output_module,  sstv_flush_module,  sstv_run_test_module },
    { "meteor", &g_meteor, NULL, NULL, NULL, NULL, meteor_run_test_module },
    { NULL,    NULL,     NULL,              NULL,                            NULL,                           NULL,               NULL                 }
};

static const module_ops_t *find_module(const char *name) {
    if (!name) return NULL;
    if (streq_icase(name, "adsb") || streq_icase(name, "ads-b")
        || streq_icase(name, "adb-s")) name = "adsb";
    if (streq_icase(name, "ship") || streq_icase(name, "ships")
        || streq_icase(name, "navi") || streq_icase(name, "maritime")) name = "ais";
    if (streq_icase(name, "voce")) name = "voice";
    if (streq_icase(name, "rs41") || streq_icase(name, "radiosonde")
        || streq_icase(name, "weather-sonde")) name = "sonde";
    if (streq_icase(name, "slow-scan-tv") || streq_icase(name, "slow-scan")) {
        name = "sstv";
    }
    if (streq_icase(name, "meteor-lrpt") || streq_icase(name, "lrpt")) name = "meteor";
    for (int i = 0; g_modules[i].name; i++) {
        if (streq_icase(name, g_modules[i].name)) return &g_modules[i];
    }
    return NULL;
}

static void usage(const char *prog) {
    fprintf(stderr, "Usage: %s [freq_mhz] [gain_db] --mode <voice|ais|adsb|sonde|sstv|meteor> [options]\n", prog);
    fprintf(stderr, "       %s --mode ais --ais-test\n", prog);
    fprintf(stderr, "       %s --mode adsb --adsb-test\n", prog);
    fprintf(stderr, "       %s --mode ais --ais-payload <hex>\n", prog);
    fprintf(stderr, "       %s --ais-nmea '<!AIVDM/...>'\n", prog);
    fprintf(stderr, "       %s --mode adsb --adsb-frame <hex>\n", prog);
    fprintf(stderr, "       %s --mode sonde --sonde-frame <hex>\n", prog);
    fprintf(stderr, "       %s --input <file|-> --input-format <auto|nmea|avr|beast|iq-u8>\n", prog);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  --freq <mhz>        center frequency (defaults: AIS 162.000, ADS-B 1090, RS41 403)\n");
    fprintf(stderr, "  --gain <db>         manual tuner gain; omitted means automatic gain\n");
    fprintf(stderr, "  --ppm <int>         frequency correction (e.g. -20, +35)\n");
    fprintf(stderr, "  --bw <hz>           tuner bandwidth in Hz, 0=automatic\n");
    fprintf(stderr, "  --device <index|serial> select an RTL-SDR (default: index 0)\n");
    fprintf(stderr, "  --list-devices      list available RTL-SDR receivers and exit\n");
    fprintf(stderr, "  --reconnect <sec>   retry after a receiver/disconnect error\n");
    fprintf(stderr, "  --stats <sec>       receiver status interval; 0 disables (default: 5)\n");
    fprintf(stderr, "  --demod <fm|am>     voice demodulator (default: fm)\n");
    fprintf(stderr, "  --lat/--lon <deg>   receiver reference for ADS-B local/surface CPR\n");
    fprintf(stderr, "  --output <format>   dashboard(default)|log|json|csv|avr|beast|quiet\n");
    fprintf(stderr, "  --input <file|->    decode protocol data or replay an IQ recording\n");
    fprintf(stderr, "  --input-format <f>  auto|nmea|avr|beast|iq-u8\n");
    fprintf(stderr, "  --ais-channel <c>   both|A|B (default: both, centered at 162.000 MHz)\n");
    fprintf(stderr, "  --test              run the selected protocol without RTL-SDR hardware\n");
    fprintf(stderr, "  --ais-test          run AIS NRZI/HDLC/CRC protocol examples\n");
    fprintf(stderr, "  --adsb-test         run valid ADS-B identification/position/velocity examples\n");
    fprintf(stderr, "  --ais-payload <hex> decode one packed AIS information payload offline\n");
    fprintf(stderr, "  --ais-nmea <line>   validate and decode one !AIVDM/!AIVDO sentence\n");
    fprintf(stderr, "  --adsb-frame <hex>  validate/decode one 56/112-bit Mode S frame offline\n");
    fprintf(stderr, "  --sonde-frame <hex> decode one de-whitened 320/518-byte RS41 frame\n");
    fprintf(stderr, "  --sstv-mode <mode>  auto|pd120|martin-m1 (default: auto/VIS)\n");
    fprintf(stderr, "  --save-dir <path>   output directory for SSTV/Meteor images\n");
    fprintf(stderr, "  --meteor-pipeline <p> m2|m2-x|m2-x-80k (default: m2-x)\n");
    fprintf(stderr, "  --satellite <name>  Auto|M2|M2-2|M2-3|M2-4\n");
    fprintf(stderr, "  --satdump <path>    SatDump CLI or .app path (default: satdump in PATH)\n");
    fprintf(stderr, "  --duration <sec>    stop a live Meteor capture after this duration\n");
    fprintf(stderr, "Examples:\n");
    fprintf(stderr, "  %s 145.500 --mode voice --demod fm\n", prog);
    fprintf(stderr, "  %s --mode ais --ppm -20 --bw 100000\n", prog);
    fprintf(stderr, "  %s --mode adsb --lat 41.9 --lon 12.5\n", prog);
    fprintf(stderr, "  %s --mode sonde --freq 403.000\n", prog);
    fprintf(stderr, "  %s --mode sstv --freq 145.800 --save-dir images\n", prog);
    fprintf(stderr, "  %s --mode meteor --freq 137.900 --save-dir meteor-output\n", prog);
}

static int parse_int_arg(const char *s, int *out) {
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (s == end || *end != '\0') return 0;
    *out = (int)v;
    return 1;
}

static int parse_double_arg(const char *s, double *out) {
    char *end = NULL;
    if (!s || !out) return 0;
    double value = strtod(s, &end);
    if (s == end || *end != '\0' || !isfinite(value)) return 0;
    *out = value;
    return 1;
}

static int parse_input_format(const char *name, input_format_t *format) {
    if (!name || !format) return 0;
    if (streq_icase(name, "auto")) *format = INPUT_FORMAT_AUTO;
    else if (streq_icase(name, "nmea") || streq_icase(name, "aivdm")) {
        *format = INPUT_FORMAT_NMEA;
    } else if (streq_icase(name, "avr") || streq_icase(name, "raw")) {
        *format = INPUT_FORMAT_AVR;
    } else if (streq_icase(name, "beast")) {
        *format = INPUT_FORMAT_BEAST;
    } else if (streq_icase(name, "iq") || streq_icase(name, "iq-u8")
               || streq_icase(name, "cu8")) {
        *format = INPUT_FORMAT_IQ_U8;
    } else {
        return 0;
    }
    return 1;
}

static int list_rtlsdr_devices(void) {
    uint32_t count = rtlsdr_get_device_count();
    printf("RTL-SDR devices: %u\n", count);
    for (uint32_t i = 0; i < count; i++) {
        char manufacturer[256] = "";
        char product[256] = "";
        char serial[256] = "";
        int strings_ok = rtlsdr_get_device_usb_strings(i, manufacturer, product, serial) == 0;
        printf("  [%u] %s", i, rtlsdr_get_device_name(i));
        if (strings_ok) {
            printf(" | manufacturer=%s | product=%s | serial=%s",
                   manufacturer, product, serial);
        }
        printf("\n");
    }
    return count > 0u ? 0 : 1;
}

static int resolve_device_index(const char *selector) {
    int parsed = 0;
    uint32_t count = rtlsdr_get_device_count();
    if (!selector) return count > 0u ? 0 : -1;
    if (parse_int_arg(selector, &parsed)) {
        return parsed >= 0 && (uint32_t)parsed < count ? parsed : -1;
    }
    parsed = rtlsdr_get_index_by_serial(selector);
    return parsed >= 0 && (uint32_t)parsed < count ? parsed : -1;
}

static input_format_t sniff_input_format(const char *path) {
    FILE *stream;
    int c;
    if (!path || strcmp(path, "-") == 0) return INPUT_FORMAT_AUTO;
    stream = fopen(path, "rb");
    if (!stream) return INPUT_FORMAT_AUTO;
    do {
        c = fgetc(stream);
    } while (c != EOF && isspace((unsigned char)c));
    fclose(stream);
    if (c == 0x1A) return INPUT_FORMAT_BEAST;
    if (c == '!') return INPUT_FORMAT_NMEA;
    if (c == '*' || c == '@' || c == '+') return INPUT_FORMAT_AVR;
    return INPUT_FORMAT_AUTO;
}

static int process_offline_input(const char *path, input_format_t format) {
    FILE *stream = strcmp(path, "-") == 0 ? stdin : fopen(path, "rb");
    int successes = 0;
    int failures = 0;

    if (!stream) {
        perror(path);
        return 0;
    }
    if (format == INPUT_FORMAT_AUTO) {
        int c;
        do {
            c = fgetc(stream);
        } while (c != EOF && isspace((unsigned char)c));
        if (c != EOF) ungetc(c, stream);
        if (c == 0x1A) format = INPUT_FORMAT_BEAST;
        else if (c == '!') format = INPUT_FORMAT_NMEA;
        else if (c == '*' || c == '@' || c == '+') format = INPUT_FORMAT_AVR;
    }
    if (format == INPUT_FORMAT_BEAST) {
        adsb_beast_parser_t parser;
        uint8_t buffer[8192];
        size_t bytes;
        adsb_beast_parser_init(&parser);
        while ((bytes = fread(buffer, 1u, sizeof(buffer), stream)) > 0u) {
            adsb_beast_parser_feed(&parser, buffer, bytes);
        }
        successes = (int)parser.frames_valid;
        failures = (int)parser.frames_rejected;
        if (parser.in_frame || parser.after_escape) failures++;
    } else if (format == INPUT_FORMAT_NMEA || format == INPUT_FORMAT_AVR) {
        char line[4096];
        while (fgets(line, sizeof(line), stream)) {
            const char *p = line;
            int ok;
            while (isspace((unsigned char)*p)) p++;
            if (*p == '\0' || *p == '#' || *p == ';') continue;
            ok = format == INPUT_FORMAT_NMEA
                     ? ais_decode_nmea_sentence(p)
                     : adsb_protocol_handle_avr_line(p);
            if (ok) successes++;
            else failures++;
        }
        if (format == INPUT_FORMAT_NMEA && ais_nmea_has_pending_fragments()) {
            fprintf(stderr, "[AIS][NMEA] input ended before all fragments arrived\n");
            failures++;
        }
    } else {
        fprintf(stderr, "Cannot detect input format; use --input-format nmea|avr|beast\n");
        failures++;
    }
    if (stream != stdin) fclose(stream);
    fprintf(output_diagnostics(), "[INPUT] accepted=%d rejected=%d\n", successes, failures);
    return successes > 0 && failures == 0;
}

static int process_iq_recording(const char *path) {
    FILE *stream = strcmp(path, "-") == 0 ? stdin : fopen(path, "rb");
    unsigned char buffer[65536];
    uint64_t total_bytes = 0u;
    size_t bytes;
    if (!stream) {
        perror(path);
        return 0;
    }
    while ((bytes = fread(buffer, 1u, sizeof(buffer), stream)) > 0u) {
        if ((bytes & 1u) != 0u) {
            fprintf(stderr, "[IQ] recording contains an incomplete I/Q pair\n");
            if (stream != stdin) fclose(stream);
            return 0;
        }
        demodulator_process_raw_iq_u8(&g_demod, buffer, (uint32_t)bytes);
        total_bytes += bytes;
    }
    if (ferror(stream)) {
        fprintf(stderr, "[IQ] read error\n");
        if (stream != stdin) fclose(stream);
        return 0;
    }
    if (stream != stdin) fclose(stream);
    fprintf(output_diagnostics(), "[IQ] replayed_bytes=%llu iq_pairs=%llu\n",
            (unsigned long long)total_bytes,
            (unsigned long long)(total_bytes / 2u));
    return total_bytes > 0u;
}

static int open_configured_rtlsdr(rtlsdr_dev_t **out, const char *selector,
                                  const demod_config_t *cfg, uint32_t freq_hz,
                                  int have_ppm, int ppm, uint32_t tuner_bw,
                                  int use_manual_gain, int gain) {
    rtlsdr_dev_t *dev = NULL;
    int device_index = resolve_device_index(selector);
    int r;
    if (!out || !cfg || device_index < 0) {
        fprintf(stderr, "RTL-SDR device not found: %s\n", selector ? selector : "index 0");
        return 0;
    }
    r = rtlsdr_open(&dev, (uint32_t)device_index);
    if (r < 0) {
        fprintf(stderr, "rtlsdr_open(%d) failed (%d)\n", device_index, r);
        return 0;
    }
    if (have_ppm && (r = rtlsdr_set_freq_correction(dev, ppm)) < 0) {
        fprintf(stderr, "Warning: frequency correction failed (%d)\n", r);
    }
    if ((r = rtlsdr_set_sample_rate(dev, (uint32_t)cfg->input_fs)) < 0) {
        fprintf(stderr, "rtlsdr_set_sample_rate failed (%d)\n", r);
        rtlsdr_close(dev);
        return 0;
    }
    if (tuner_bw > 0u && (r = rtlsdr_set_tuner_bandwidth(dev, tuner_bw)) < 0) {
        fprintf(stderr, "Warning: tuner bandwidth failed (%d)\n", r);
    }
    if ((r = rtlsdr_set_center_freq(dev, freq_hz)) < 0) {
        fprintf(stderr, "rtlsdr_set_center_freq failed (%d)\n", r);
        rtlsdr_close(dev);
        return 0;
    }
    if (use_manual_gain) {
        (void)rtlsdr_set_tuner_gain_mode(dev, 1);
        if ((r = rtlsdr_set_tuner_gain(dev, gain * 10)) < 0) {
            fprintf(stderr, "Warning: tuner gain failed (%d)\n", r);
        }
    } else {
        (void)rtlsdr_set_tuner_gain_mode(dev, 0);
    }
    if ((r = rtlsdr_reset_buffer(dev)) < 0) {
        fprintf(stderr, "Warning: RTL-SDR buffer reset failed (%d)\n", r);
    }
    if (!output_is_dashboard()) {
        fprintf(output_diagnostics(), "[RX] device=%d name=\"%s\" state=connected\n",
                device_index, rtlsdr_get_device_name((uint32_t)device_index));
    }
    *out = dev;
    return 1;
}

int main(int argc, char **argv) {
    signal(SIGINT, on_sigint);
    voice_module_reset(&g_voice);
    sstv_module_reset(&g_sstv);
    meteor_module_reset(&g_meteor);
    adsb_protocol_reset();

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    const char *freq_opt = NULL;
    const char *module_name = NULL;
    int gain = 0;
    int use_manual_gain = 0;
    int ais_test = 0;
    int adsb_test = 0;
    int generic_test = 0;
    int ppm = 0;
    int have_ppm = 0;
    uint32_t tuner_bw = 0;
    const char *voice_demod = NULL;
    const char *adsb_frame = NULL;
    const char *sonde_frame = NULL;
    const char *sstv_mode = NULL;
    const char *save_dir = NULL;
    const char *meteor_pipeline = NULL;
    const char *meteor_satellite = NULL;
    const char *satdump_path = NULL;
    int meteor_duration = 0;
    const char *ais_payload = NULL;
    const char *ais_nmea = NULL;
    const char *ais_channels = "both";
    const char *input_path = NULL;
    const char *device_selector = NULL;
    input_format_t input_format = INPUT_FORMAT_AUTO;
    int input_format_explicit = 0;
    int output_format_explicit = 0;
    int list_devices = 0;
    int reconnect_seconds = 0;
    double stats_interval = 5.0;
    double receiver_lat = 0.0;
    double receiver_lon = 0.0;
    int have_receiver_lat = 0;
    int have_receiver_lon = 0;

    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
            usage(argv[0]);
            return 0;
        }
        if (strcmp(arg, "--list-devices") == 0) {
            list_devices = 1;
            continue;
        }
        if (strcmp(arg, "--device") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --device\n");
                return 1;
            }
            device_selector = argv[++i];
            continue;
        }
        if (strcmp(arg, "--reconnect") == 0) {
            if (i + 1 >= argc || !parse_int_arg(argv[++i], &reconnect_seconds)
                || reconnect_seconds < 0) {
                fprintf(stderr, "Invalid or missing value for --reconnect\n");
                return 1;
            }
            continue;
        }
        if (strcmp(arg, "--stats") == 0) {
            if (i + 1 >= argc || !parse_double_arg(argv[++i], &stats_interval)
                || stats_interval < 0.0) {
                fprintf(stderr, "Invalid or missing value for --stats\n");
                return 1;
            }
            continue;
        }
        if (strcmp(arg, "--output") == 0) {
            if (i + 1 >= argc || !output_set_format_name(argv[++i])) {
                fprintf(stderr, "Invalid --output: use dashboard|log|json|csv|avr|beast|quiet\n");
                return 1;
            }
            output_format_explicit = 1;
            continue;
        }
        if (strcmp(arg, "--input") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --input\n");
                return 1;
            }
            input_path = argv[++i];
            continue;
        }
        if (strcmp(arg, "--input-format") == 0) {
            if (i + 1 >= argc || !parse_input_format(argv[++i], &input_format)) {
                fprintf(stderr, "Invalid --input-format: use auto|nmea|avr|beast|iq-u8\n");
                return 1;
            }
            input_format_explicit = 1;
            continue;
        }
        if (strcmp(arg, "--ais-channel") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --ais-channel\n");
                return 1;
            }
            ais_channels = argv[++i];
            continue;
        }
        if (strcmp(arg, "--mode") == 0 || strcmp(arg, "--decoder") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for %s\n", arg);
                usage(argv[0]);
                return 1;
            }
            module_name = argv[++i];
            continue;
        }
        if (strcmp(arg, "--ais-test") == 0) {
            ais_test = 1;
            continue;
        }
        if (strcmp(arg, "--adsb-test") == 0) {
            adsb_test = 1;
            continue;
        }
        if (strcmp(arg, "--test") == 0) {
            generic_test = 1;
            continue;
        }
        if (strcmp(arg, "--freq") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --freq\n");
                return 1;
            }
            freq_opt = argv[++i];
            continue;
        }
        if (strcmp(arg, "--gain") == 0) {
            if (i + 1 >= argc || !parse_int_arg(argv[++i], &gain)) {
                fprintf(stderr, "Invalid or missing value for --gain\n");
                return 1;
            }
            use_manual_gain = 1;
            continue;
        }
        if (strcmp(arg, "--adsb-frame") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --adsb-frame\n");
                return 1;
            }
            adsb_frame = argv[++i];
            continue;
        }
        if (strcmp(arg, "--sonde-frame") == 0 || strcmp(arg, "--rs41-frame") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for %s\n", arg);
                return 1;
            }
            sonde_frame = argv[++i];
            continue;
        }
        if (strcmp(arg, "--sstv-mode") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --sstv-mode\n");
                return 1;
            }
            sstv_mode = argv[++i];
            continue;
        }
        if (strcmp(arg, "--save-dir") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --save-dir\n");
                return 1;
            }
            save_dir = argv[++i];
            continue;
        }
        if (strcmp(arg, "--meteor-pipeline") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --meteor-pipeline\n");
                return 1;
            }
            meteor_pipeline = argv[++i];
            continue;
        }
        if (strcmp(arg, "--satellite") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --satellite\n");
                return 1;
            }
            meteor_satellite = argv[++i];
            continue;
        }
        if (strcmp(arg, "--satdump") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --satdump\n");
                return 1;
            }
            satdump_path = argv[++i];
            continue;
        }
        if (strcmp(arg, "--duration") == 0) {
            if (i + 1 >= argc || !parse_int_arg(argv[++i], &meteor_duration)
                || meteor_duration < 0) {
                fprintf(stderr, "Invalid or missing value for --duration\n");
                return 1;
            }
            continue;
        }
        if (strcmp(arg, "--ais-payload") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --ais-payload\n");
                return 1;
            }
            ais_payload = argv[++i];
            continue;
        }
        if (strcmp(arg, "--ais-nmea") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for --ais-nmea\n");
                return 1;
            }
            ais_nmea = argv[++i];
            continue;
        }
        if (strcmp(arg, "--lat") == 0 || strcmp(arg, "--lon") == 0) {
            double value;
            int is_lat = strcmp(arg, "--lat") == 0;
            if (i + 1 >= argc || !parse_double_arg(argv[++i], &value)) {
                fprintf(stderr, "Invalid or missing value for %s\n", arg);
                return 1;
            }
            if (is_lat) {
                receiver_lat = value;
                have_receiver_lat = 1;
            } else {
                receiver_lon = value;
                have_receiver_lon = 1;
            }
            continue;
        }
        if (strcmp(arg, "--ppm") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for %s\n", arg);
                usage(argv[0]);
                return 1;
            }
            if (!parse_int_arg(argv[++i], &ppm)) {
                fprintf(stderr, "Invalid value for --ppm\n");
                return 1;
            }
            have_ppm = 1;
            continue;
        }
        if (strcmp(arg, "--bw") == 0) {
            int parsed_bw = 0;
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for %s\n", arg);
                usage(argv[0]);
                return 1;
            }
            if (!parse_int_arg(argv[++i], &parsed_bw) || parsed_bw < 0) {
                fprintf(stderr, "Invalid value for --bw\n");
                return 1;
            }
            tuner_bw = (uint32_t)parsed_bw;
            continue;
        }
        if (strcmp(arg, "--demod") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for %s\n", arg);
                usage(argv[0]);
                return 1;
            }
            voice_demod = argv[++i];
            continue;
        }
        if (find_module(arg)) {
            module_name = arg;
            continue;
        }
        if (!freq_opt) {
            double parsed_frequency;
            if (parse_double_arg(arg, &parsed_frequency)) {
                freq_opt = arg;
                continue;
            }
        }
        if (!use_manual_gain) {
            int parsed_gain = 0;
            if (parse_int_arg(arg, &parsed_gain)) {
                gain = parsed_gain;
                use_manual_gain = 1;
                continue;
            }
        }
        fprintf(stderr, "Unknown argument: %s\n", arg);
        usage(argv[0]);
        return 1;
    }

    if (list_devices) return list_rtlsdr_devices();

    if (!input_path && input_format_explicit) {
        fprintf(stderr, "--input-format requires --input\n");
        return 1;
    }

    if (input_path && input_format == INPUT_FORMAT_AUTO
        && strcmp(input_path, "-") != 0) {
        input_format = sniff_input_format(input_path);
    }

    if (!module_name) {
        if (adsb_test || adsb_frame) module_name = "adsb";
        else if (sonde_frame) module_name = "sonde";
        else if (ais_test || ais_payload || ais_nmea) module_name = "ais";
        else if (input_format == INPUT_FORMAT_NMEA) module_name = "ais";
        else if (input_format == INPUT_FORMAT_AVR || input_format == INPUT_FORMAT_BEAST) {
            module_name = "adsb";
        }
        else if (input_path) {
            fprintf(stderr, "Cannot infer protocol from input; add --mode ais|adsb or --input-format\n");
            return 1;
        }
        else module_name = "ais"; /* Backward-compatible live default. */
    }

    if (streq_icase(module_name, "voice")) {
        if (!voice_module_set_demod(&g_voice, voice_demod)) {
            fprintf(stderr, "Unknown voice demodulation: %s\n", voice_demod);
            usage(argv[0]);
            return 1;
        }
    } else if (voice_demod) {
        fprintf(stderr, "--demod works only with mode=voice\n");
        usage(argv[0]);
        return 1;
    }

    const module_ops_t *module = find_module(module_name);
    if (!module) {
        fprintf(stderr, "Unknown mode/module: %s\n", module_name);
        usage(argv[0]);
        return 1;
    }

    if (streq_icase(module->name, "sstv")) {
        if (!sstv_module_set_mode(&g_sstv, sstv_mode)) {
            fprintf(stderr, "Invalid --sstv-mode: use auto|pd120|martin-m1\n");
            return 1;
        }
        if (save_dir && !sstv_module_set_save_dir(&g_sstv, save_dir)) {
            fprintf(stderr, "Invalid --save-dir\n");
            return 1;
        }
    } else if (sstv_mode) {
        fprintf(stderr, "--sstv-mode works only with mode=sstv\n");
        return 1;
    }
    if (streq_icase(module->name, "meteor")) {
        if (meteor_pipeline && !meteor_module_set_pipeline(&g_meteor, meteor_pipeline)) {
            fprintf(stderr, "Invalid --meteor-pipeline: use m2|m2-x|m2-x-80k\n");
            return 1;
        }
        if (meteor_satellite
            && !meteor_module_set_satellite(&g_meteor, meteor_satellite)) {
            fprintf(stderr, "Invalid --satellite: use Auto|M2|M2-2|M2-3|M2-4\n");
            return 1;
        }
        if (satdump_path && !meteor_module_set_executable(&g_meteor, satdump_path)) {
            fprintf(stderr, "Invalid --satdump path\n");
            return 1;
        }
        if (save_dir && !meteor_module_set_output_dir(&g_meteor, save_dir)) {
            fprintf(stderr, "Invalid --save-dir\n");
            return 1;
        }
        meteor_module_set_timeout(&g_meteor, (unsigned int)meteor_duration);
    } else if (meteor_pipeline || meteor_satellite || satdump_path || meteor_duration) {
        fprintf(stderr, "Meteor options work only with mode=meteor\n");
        return 1;
    } else if (save_dir && !streq_icase(module->name, "sstv")) {
        fprintf(stderr, "--save-dir works only with mode=sstv or mode=meteor\n");
        return 1;
    }

    if ((streq_icase(module->name, "voice") || streq_icase(module->name, "sstv")
         || streq_icase(module->name, "meteor"))
        && output_is_dashboard()) {
        if (output_format_explicit) {
            fprintf(stderr, "Dashboard output is available for ADS-B, AIS and sondes; use --output log here\n");
            return 1;
        }
        (void)output_set_format_name("log");
    } else if (output_is_dashboard()) {
        dashboard_set_mode(module->name);
    }

    if (ais_test && !streq_icase(module->name, "ais")) {
        fprintf(stderr, "--ais-test works only with mode=ais\n");
        return 1;
    }
    if (adsb_test && !streq_icase(module->name, "adsb")) {
        fprintf(stderr, "--adsb-test works only with mode=adsb\n");
        return 1;
    }
    if (ais_payload && !streq_icase(module->name, "ais")) {
        fprintf(stderr, "--ais-payload works only with mode=ais\n");
        return 1;
    }
    if (ais_nmea && !streq_icase(module->name, "ais")) {
        fprintf(stderr, "--ais-nmea works only with mode=ais\n");
        return 1;
    }
    if (adsb_frame && !streq_icase(module->name, "adsb")) {
        fprintf(stderr, "--adsb-frame works only with mode=adsb\n");
        return 1;
    }
    if (sonde_frame && !streq_icase(module->name, "sonde")) {
        fprintf(stderr, "--sonde-frame works only with mode=sonde\n");
        return 1;
    }
    if (input_path && input_format == INPUT_FORMAT_NMEA
        && !streq_icase(module->name, "ais")) {
        fprintf(stderr, "NMEA input requires mode=ais\n");
        return 1;
    }
    if (input_path && (input_format == INPUT_FORMAT_AVR
                       || input_format == INPUT_FORMAT_BEAST)
        && !streq_icase(module->name, "adsb")) {
        fprintf(stderr, "AVR/Beast input requires mode=adsb\n");
        return 1;
    }
    if (output_is_adsb_raw() && !streq_icase(module->name, "adsb")) {
        fprintf(stderr, "AVR/Beast output is available only in mode=adsb\n");
        return 1;
    }
    if (streq_icase(module->name, "ais")) {
        if (!ais_receiver_set_channels(&g_ais, ais_channels)) {
            fprintf(stderr, "Invalid --ais-channel: use both, A/1, or B/2\n");
            return 1;
        }
    } else if (!streq_icase(ais_channels, "both")) {
        fprintf(stderr, "--ais-channel works only with mode=ais\n");
        return 1;
    }
    if (generic_test && !module->run_test) {
        fprintf(stderr, "--test is not available for mode=%s\n", module->name);
        return 1;
    }
    if (have_receiver_lat != have_receiver_lon) {
        fprintf(stderr, "--lat and --lon must be supplied together\n");
        return 1;
    }
    if (have_receiver_lat) {
        if (!streq_icase(module->name, "adsb")) {
            fprintf(stderr, "--lat/--lon currently apply to ADS-B CPR decoding only\n");
            return 1;
        }
        if (!adsb_protocol_set_reference(receiver_lat, receiver_lon)) {
            fprintf(stderr, "Invalid receiver coordinates: latitude -90..90, longitude -180..180\n");
            return 1;
        }
    }

    if (streq_icase(module->name, "meteor")) {
        double meteor_frequency = 137.900;
        char satdump_device[256];
        const char *satdump_device_id = NULL;
        int ok;
        if (freq_opt && (!parse_double_arg(freq_opt, &meteor_frequency)
                         || meteor_frequency < 10.0 || meteor_frequency > 4294.0)) {
            fprintf(stderr, "Invalid Meteor frequency.\n");
            return 1;
        }
        if (device_selector) {
            int index = resolve_device_index(device_selector);
            char manufacturer[256];
            char product[256];
            char serial[256];
            if (index >= 0
                && rtlsdr_get_device_usb_strings((uint32_t)index, manufacturer,
                                                 product, serial) == 0
                && serial[0] != '\0') {
                snprintf(satdump_device, sizeof(satdump_device), "%s", serial);
            } else {
                snprintf(satdump_device, sizeof(satdump_device), "%s", device_selector);
            }
            satdump_device_id = satdump_device;
        }
        if (generic_test) return module->run_test(module->ctx) ? 0 : 1;
        if (input_path) {
            if (input_format != INPUT_FORMAT_AUTO && input_format != INPUT_FORMAT_IQ_U8) {
                fprintf(stderr, "Meteor offline input must use --input-format iq-u8\n");
                return 1;
            }
            ok = meteor_module_run_offline(&g_meteor, input_path);
        } else {
            ok = meteor_module_run_live(&g_meteor, meteor_frequency,
                                        satdump_device_id, use_manual_gain, gain,
                                        have_ppm, ppm);
        }
        return ok ? 0 : 1;
    }

    demod_config_t demod_cfg;
    memset(&demod_cfg, 0, sizeof(demod_cfg));
    module->fill_demod_config(module->ctx, &demod_cfg);
    if (demod_cfg.kind == DEMOD_KIND_NONE || demod_cfg.input_fs <= 0 || demod_cfg.output_fs <= 0) {
        fprintf(stderr, "Invalid demodulator config for mode=%s\n", module->name);
        return 1;
    }

    if (!module->init(module->ctx, &demod_cfg)) {
        fprintf(stderr, "Failed to initialize mode=%s\n", module->name);
        module->flush(module->ctx);
        return 1;
    }

    if (ais_test || adsb_test || generic_test || ais_payload || ais_nmea
        || adsb_frame || sonde_frame || (input_path && input_format != INPUT_FORMAT_IQ_U8)) {
        int ok = 1;
        if (ais_test || adsb_test || generic_test) {
            ok = module->run_test && module->run_test(module->ctx) && ok;
        }
        if (ais_payload) ok = ais_decode_payload_hex(ais_payload) && ok;
        if (ais_nmea) ok = (ais_decode_nmea_sentence(ais_nmea) == 1) && ok;
        if (adsb_frame) ok = adsb_protocol_handle_frame_hex(adsb_frame) && ok;
        if (sonde_frame) ok = sonde_module_decode_hex(&g_sonde, sonde_frame) && ok;
        if (input_path) ok = process_offline_input(input_path, input_format) && ok;
        module->flush(module->ctx);
        return ok ? 0 : 1;
    }

    demod_output_t demod_output = module->get_demod_output(module->ctx);
    if (!demodulator_init(&g_demod, &demod_cfg, &demod_output)) {
        fprintf(stderr, "Failed to initialize demodulator=%s for mode=%s\n",
                demod_kind_name(demod_cfg.kind), module->name);
        module->flush(module->ctx);
        return 1;
    }

    if (input_path && input_format == INPUT_FORMAT_IQ_U8) {
        int ok = process_iq_recording(input_path);
        demodulator_flush(&g_demod);
        module->flush(module->ctx);
        return ok ? 0 : 1;
    }

    if (!freq_opt) {
        if (streq_icase(module->name, "adsb")) freq_opt = "1090.0";
        else if (streq_icase(module->name, "ais")) {
            if (g_ais.selected_channel == 'A') freq_opt = "161.975";
            else if (g_ais.selected_channel == 'B') freq_opt = "162.025";
            else freq_opt = "162.000";
        }
        else if (streq_icase(module->name, "sonde")) freq_opt = "403.000";
        else if (streq_icase(module->name, "sstv")) freq_opt = "145.800";
        else {
            fprintf(stderr, "Voice mode requires a frequency\n");
            demodulator_flush(&g_demod);
            module->flush(module->ctx);
            return 1;
        }
    }

    double freq_mhz;
    if (!parse_double_arg(freq_opt, &freq_mhz) || freq_mhz < 10.0 || freq_mhz > 4294.0) {
        fprintf(stderr, "Invalid frequency.\n");
        demodulator_flush(&g_demod);
        module->flush(module->ctx);
        return 1;
    }
    uint32_t freq_hz = (uint32_t)llround(freq_mhz * 1e6);

    char gain_desc[32];
    if (use_manual_gain) snprintf(gain_desc, sizeof(gain_desc), "%d dB", gain);
    else snprintf(gain_desc, sizeof(gain_desc), "auto");

    memset(&g_live_stats, 0, sizeof(g_live_stats));
    g_live_stats.module_name = module->name;
    g_live_stats.stats_interval = stats_interval;
    (void)clock_gettime(CLOCK_MONOTONIC, &g_live_stats.started);
    g_live_stats.last_report = g_live_stats.started;

    if (output_is_dashboard()) {
        dashboard_configure_receiver(module->name, freq_mhz, gain_desc,
                                     device_selector ? device_selector : "0");
        dashboard_set_connection("connessione...");
    } else {
        FILE *diag = output_diagnostics();
        fprintf(diag, "START RX (async)\n");
        fprintf(diag,
                "  freq=%.3f MHz | raw_fs=%d | demod=%s | demod_fs=%d | gain=%s | mode=%s | output=%s",
                freq_mhz, demod_cfg.input_fs, demod_kind_name(demod_cfg.kind),
                demod_cfg.output_fs, gain_desc, module->name,
                output_format_name(output_get_format()));
        if (have_ppm) fprintf(diag, " | ppm=%d", ppm);
        if (tuner_bw) fprintf(diag, " | bw=%u", tuner_bw);
        fprintf(diag, "\n  Ctrl+C to stop\n");
    }

    int final_read_result = 0;
    while (!g_stop) {
        rtlsdr_dev_t *dev = NULL;
        int r;
        if (!open_configured_rtlsdr(&dev, device_selector, &demod_cfg, freq_hz,
                                    have_ppm, ppm, tuner_bw, use_manual_gain, gain)) {
            final_read_result = -1;
            if (output_is_dashboard()) dashboard_set_connection("device non trovato");
        } else {
            if (output_is_dashboard()) dashboard_set_connection("connesso");
            g_dev = dev;
            r = rtlsdr_read_async(dev, rtlsdr_cb, &g_live_stats, 0, 0);
            final_read_result = r;
            g_dev = NULL;
            rtlsdr_close(dev);
            live_stats_maybe_report(&g_live_stats, 1);
            if (!g_stop) {
                if (output_is_dashboard()) dashboard_set_connection("disconnesso");
                else fprintf(stderr, "[RX] state=disconnected read_error=%d\n", r);
            }
        }
        if (g_stop || reconnect_seconds <= 0) break;
        if (output_is_dashboard()) dashboard_set_connection("riconnessione...");
        else fprintf(stderr, "[RX] state=reconnecting retry_in=%ds\n", reconnect_seconds);
        {
            struct timespec delay;
            delay.tv_sec = reconnect_seconds;
            delay.tv_nsec = 0;
            while (!g_stop && nanosleep(&delay, &delay) != 0) {
                /* Resume the remaining delay when interrupted by a non-stop signal. */
            }
        }
    }

    demodulator_flush(&g_demod);
    module->flush(module->ctx);

    if (output_is_dashboard()) {
        dashboard_set_connection(g_stop ? "arrestato" : "disconnesso");
        dashboard_shutdown();
    }

    if (!g_stop && final_read_result < 0) {
        fprintf(stderr, "rtlsdr_read_async exited with error %d\n", final_read_result);
        return 1;
    }
    return 0;
}
