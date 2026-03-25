// rtl-universal.c
// Main entry-point: selects the protocol module, asks the module which
// demodulator it needs, then forwards raw RTL-SDR IQ samples to that
// demodulator. All protocol-specific DSP lives outside this file.

#include <ctype.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rtl-sdr.h>

#include "modules/header/ais_decoder.h"
#include "modules/header/adsb_decoder.h"
#include "modules/header/voice_module.h"
#include "demod/header/demodulator.h"

static volatile int g_stop = 0;
static rtlsdr_dev_t *g_dev = NULL;
static demodulator_t g_demod;

static voice_module_t g_voice;
static ais_ctx_t g_ais;
static adsb_ctx_t g_adsb;

typedef struct {
    const char *name;
    void *ctx;
    int (*init)(void *ctx, const demod_config_t *cfg);
    void (*fill_demod_config)(void *ctx, demod_config_t *cfg);
    demod_output_t (*get_demod_output)(void *ctx);
    void (*flush)(void *ctx);
    void (*run_test)(void *ctx);
} module_ops_t;

static void on_sigint(int sig) {
    (void)sig;
    g_stop = 1;
    if (g_dev) rtlsdr_cancel_async(g_dev);
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
    (void)ctx;
    if (g_stop) return;
    demodulator_process_raw_iq_u8(&g_demod, buf, len);
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
    ais_init((ais_ctx_t *)ctx);
    return 1;
}

static void ais_fill_demod_config_module(void *ctx, demod_config_t *cfg) {
    (void)ctx;
    ais_get_demod_config(cfg);
}

static demod_output_t ais_get_demod_output_module(void *ctx) {
    return ais_get_demod_output((ais_ctx_t *)ctx);
}

static void ais_flush_module(void *ctx) {
    ais_flush((ais_ctx_t *)ctx);
}

static void ais_run_test_module(void *ctx) {
    (void)ctx;
    ais_test_emit_example();
}

static int adsb_init_module(void *ctx, const demod_config_t *cfg) {
    adsb_init((adsb_ctx_t *)ctx, cfg ? cfg->output_fs : 0);
    return 1;
}

static void adsb_fill_demod_config_module(void *ctx, demod_config_t *cfg) {
    (void)ctx;
    adsb_get_demod_config(cfg);
}

static demod_output_t adsb_get_demod_output_module(void *ctx) {
    return adsb_get_demod_output((adsb_ctx_t *)ctx);
}

static void adsb_flush_module(void *ctx) {
    adsb_flush((adsb_ctx_t *)ctx);
}

static void adsb_run_test_module(void *ctx) {
    adsb_test_emit_example((adsb_ctx_t *)ctx);
}

static const module_ops_t g_modules[] = {
    { "voice", &g_voice, voice_init_module, voice_fill_demod_config_module, voice_get_demod_output_module, voice_flush_module, NULL                 },
    { "ais",   &g_ais,   ais_init_module,   ais_fill_demod_config_module,   ais_get_demod_output_module,   ais_flush_module,   ais_run_test_module  },
    { "adsb",  &g_adsb,  adsb_init_module,  adsb_fill_demod_config_module,  adsb_get_demod_output_module,  adsb_flush_module,  adsb_run_test_module },
    { NULL,    NULL,     NULL,              NULL,                            NULL,                           NULL,               NULL                 }
};

static const module_ops_t *find_module(const char *name) {
    if (!name) return NULL;
    if (streq_icase(name, "adsb") || streq_icase(name, "adb-s")) name = "adsb";
    if (streq_icase(name, "voce")) name = "voice";
    for (int i = 0; g_modules[i].name; i++) {
        if (streq_icase(name, g_modules[i].name)) return &g_modules[i];
    }
    return NULL;
}

static void usage(const char *prog) {
    fprintf(stderr, "Usage: %s <freq_mhz> [gain_db] [mode]\n", prog);
    fprintf(stderr, "       %s <freq_mhz> [gain_db] --mode <voice|ais|adsb>\n", prog);
    fprintf(stderr, "       %s <freq_mhz> --mode voice [--demod <fm|am>]\n", prog);
    fprintf(stderr, "       %s <freq_mhz> --mode ais --ais-test\n", prog);
    fprintf(stderr, "       %s <freq_mhz> --mode adsb --adsb-test\n", prog);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  --ppm <int>   frequency correction (e.g. -20, +35)\n");
    fprintf(stderr, "  --bw  <hz>    tuner bandwidth (Hz), 0=auto\n");
    fprintf(stderr, "  --demod <x>   voice demodulator: fm (default) or am\n");
    fprintf(stderr, "Example: %s 145.500 --mode voice --demod fm\n", prog);
    fprintf(stderr, "         %s 118.300 --mode voice --demod am\n", prog);
    fprintf(stderr, "Example: %s 162.025 --mode ais --ppm -20 --bw 25000\n", prog);
    fprintf(stderr, "         %s 1090.0 --mode adsb\n", prog);
}

static int parse_int_arg(const char *s, int *out) {
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (s == end || *end != '\0') return 0;
    *out = (int)v;
    return 1;
}

int main(int argc, char **argv) {
    signal(SIGINT, on_sigint);
    voice_module_reset(&g_voice);

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    const char *freq_opt = argv[1];
    const char *module_name = "ais";
    int gain = 0;
    int use_manual_gain = 0;
    int ais_test = 0;
    int adsb_test = 0;
    int ppm = 0;
    int have_ppm = 0;
    uint32_t tuner_bw = 0;
    const char *voice_demod = NULL;

    for (int i = 2; i < argc; i++) {
        const char *arg = argv[i];
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
        if (streq_icase(arg, "voice") || streq_icase(arg, "voce")
            || streq_icase(arg, "ais") || streq_icase(arg, "adsb")
            || streq_icase(arg, "adb-s")) {
            module_name = arg;
            continue;
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

    if (ais_test) {
        if (!streq_icase(module->name, "ais")) {
            fprintf(stderr, "--ais-test works only with mode=ais\n");
            module->flush(module->ctx);
            return 1;
        }
        module->run_test(module->ctx);
        module->flush(module->ctx);
        return 0;
    }

    if (adsb_test) {
        if (!streq_icase(module->name, "adsb")) {
            fprintf(stderr, "--adsb-test works only with mode=adsb\n");
            module->flush(module->ctx);
            return 1;
        }
        module->run_test(module->ctx);
        module->flush(module->ctx);
        return 0;
    }

    demod_output_t demod_output = module->get_demod_output(module->ctx);
    if (!demodulator_init(&g_demod, &demod_cfg, &demod_output)) {
        fprintf(stderr, "Failed to initialize demodulator=%s for mode=%s\n",
                demod_kind_name(demod_cfg.kind), module->name);
        module->flush(module->ctx);
        return 1;
    }

    double freq_mhz = atof(freq_opt);
    if (freq_mhz < 10.0) {
        fprintf(stderr, "Invalid frequency.\n");
        demodulator_flush(&g_demod);
        module->flush(module->ctx);
        return 1;
    }
    uint32_t freq_hz = (uint32_t)(freq_mhz * 1e6);

    rtlsdr_dev_t *dev = NULL;
    int r = rtlsdr_open(&dev, 0);
    if (r < 0) {
        fprintf(stderr, "rtlsdr_open failed\n");
        demodulator_flush(&g_demod);
        module->flush(module->ctx);
        return 1;
    }
    g_dev = dev;

    if (have_ppm) {
        r = rtlsdr_set_freq_correction(dev, ppm);
        if (r < 0) {
            fprintf(stderr, "Warning: rtlsdr_set_freq_correction failed (%d)\n", r);
        }
    }

    r = rtlsdr_set_sample_rate(dev, (uint32_t)demod_cfg.input_fs);
    if (r < 0) {
        fprintf(stderr, "rtlsdr_set_sample_rate failed (%d)\n", r);
        g_dev = NULL;
        rtlsdr_close(dev);
        demodulator_flush(&g_demod);
        module->flush(module->ctx);
        return 1;
    }

    if (tuner_bw > 0) {
        r = rtlsdr_set_tuner_bandwidth(dev, tuner_bw);
        if (r < 0) {
            fprintf(stderr, "Warning: rtlsdr_set_tuner_bandwidth failed (%d)\n", r);
        }
    }

    r = rtlsdr_set_center_freq(dev, freq_hz);
    if (r < 0) {
        fprintf(stderr, "rtlsdr_set_center_freq failed (%d)\n", r);
        g_dev = NULL;
        rtlsdr_close(dev);
        demodulator_flush(&g_demod);
        module->flush(module->ctx);
        return 1;
    }

    if (use_manual_gain) {
        rtlsdr_set_tuner_gain_mode(dev, 1);
        rtlsdr_set_tuner_gain(dev, gain * 10);
    } else {
        rtlsdr_set_tuner_gain_mode(dev, 0);
    }

    rtlsdr_reset_buffer(dev);

    char gain_desc[32];
    if (use_manual_gain) snprintf(gain_desc, sizeof(gain_desc), "%d dB", gain);
    else snprintf(gain_desc, sizeof(gain_desc), "auto");

    printf("START RX C (async)\n");
    printf("  freq=%.3f MHz | raw_fs=%d | demod=%s | demod_fs=%d | gain=%s | mode=%s",
           freq_mhz, demod_cfg.input_fs, demod_kind_name(demod_cfg.kind),
           demod_cfg.output_fs, gain_desc, module->name);
    if (have_ppm) printf(" | ppm=%d", ppm);
    if (tuner_bw) printf(" | bw=%u", tuner_bw);
    printf("\n");
    printf("  Ctrl+C to stop\n");

    r = rtlsdr_read_async(dev, rtlsdr_cb, NULL, 0, 0);

    g_stop = 1;
    demodulator_flush(&g_demod);
    module->flush(module->ctx);

    g_dev = NULL;
    rtlsdr_close(dev);

    if (r < 0) {
        fprintf(stderr, "rtlsdr_read_async exited with error %d\n", r);
        return 1;
    }
    return 0;
}
