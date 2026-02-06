// wfm_live.c
// Minimal WFM (FM broadcast) mono receiver: RTL-SDR -> FM demod -> PortAudio
// Build (Homebrew):
//   cc -O2 -std=c11 wfm_live.c modules/ais_decoder.c modules/voice_decoder.c -o wfm_live $(pkg-config --cflags --libs librtlsdr portaudio-2.0) -lm
//
// Run:
//   ./wfm_live 100.0   (MHz)             [voice default]
// Optional gain (dB) and mode:
//   ./wfm_live 100.0 20 voice
//   ./wfm_live 162.0 --mode ais
//   ./wfm_live 162.0 --mode ais --ais-test
//
// Notes:
// - Uses async read (stable like rtl_fm).
// - Downsamples 2.4 MS/s -> 240 kS/s (decim=10) using simple boxcar averaging.
// - Then 240 kS/s -> 48 kS/s (decim=5) also boxcar.
// - FM demod via phase difference angle(IQ[n]*conj(IQ[n-1])).
// - De-emphasis 50us (EU) 1-pole IIR.

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>

#include <rtl-sdr.h>
#include <portaudio.h>
#include "modules/header/ais_decoder.h"
#include "modules/header/voice_decoder.h"

#define SDR_FS        2400000   // 2.4 MS/s
#define AUDIO_FS      240000     // after decim2=5
#define DECIM1        5
#define DECIM2        2

// ring buffer seconds (audio)
#define RING_SECONDS  4
#define RING_SIZE     (AUDIO_FS * RING_SECONDS)

// ------------ globals / state ------------
static volatile int g_stop = 0;
static rtlsdr_dev_t* g_dev = NULL;

// AIS
static ais_ctx_t g_ais;

// Voice
static voice_ctx_t g_voice;


static void on_sigint(int sig) {
    (void)sig;
    g_stop = 1;
    if (g_dev) rtlsdr_cancel_async(g_dev);
}

static int streq_icase(const char* a, const char* b) {
    if (!a || !b) return 0;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

// Simple ring buffer for float mono audio
typedef struct {
    float buf[RING_SIZE];
    uint32_t w; // write index
    uint32_t r; // read index
} ring_t;

static ring_t g_ring;

// Push N samples into ring (drops oldest if overflow)
static inline void ring_push(const float* x, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        g_ring.buf[g_ring.w] = x[i];
        g_ring.w = (g_ring.w + 1) % RING_SIZE;
        // overflow: move read forward
        if (g_ring.w == g_ring.r) {
            g_ring.r = (g_ring.r + 1) % RING_SIZE;
        }
    }
}

// Pop N samples; if not enough, pad with zeros
static inline void ring_pop(float* out, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        if (g_ring.r == g_ring.w) {
            out[i] = 0.0f;
        } else {
            out[i] = g_ring.buf[g_ring.r];
            g_ring.r = (g_ring.r + 1) % RING_SIZE;
        }
    }
}

// FM demod helper: angle of complex product
static inline float fm_discriminator(float i, float q, float i_prev, float q_prev) {
    // prod = (i + jq) * conj(i_prev + jq_prev) = (i + jq) * (i_prev - jq_prev)
    float re = i * i_prev + q * q_prev;
    float im = q * i_prev - i * q_prev;
    return atan2f(im, re);
}

// --------- PortAudio callback ----------
static int pa_cb(const void* input, void* output,
                 unsigned long frameCount,
                 const PaStreamCallbackTimeInfo* timeInfo,
                 PaStreamCallbackFlags statusFlags,
                 void* userData) {
    (void)input; (void)timeInfo; (void)statusFlags; (void)userData;
    float* out = (float*)output;
    ring_pop(out, (uint32_t)frameCount);
    return g_stop ? paComplete : paContinue;
}

// --------- RTL-SDR async callback (producer) ----------
typedef struct {
    // Decim1 accumulators
    int acc1_count;
    int32_t acc1_i;
    int32_t acc1_q;

    // Decim2 accumulators (audio stage)
    int acc2_count;
    float acc2;

    // FM demod previous IQ (at FS1)
    float i_prev;
    float q_prev;
    int have_prev;

    // Small temp AIS buffer (raw demod @ 48k)
    float ais_tmp[4096];
    uint32_t ais_tmp_len;
    
} dsp_state_t;

static dsp_state_t g_dsp;

// --------- Decoder interface ----------
typedef struct {
    const char* name;
    int needs_audio; // 1 = uses PortAudio output
    void (*init)(int fs_demod);
    void (*process_sample)(float sample);
    void (*flush)(void);
} decoder_ops_t;

static void voice_init_decoder(int fs_demod);
static void voice_process_sample_decoder(float sample);
static void voice_flush_decoder(void);
static void ais_init_decoder(int fs_demod);
static void ais_process_sample(float sample);
static void ais_flush(void);

// Add new decoders here.
static const decoder_ops_t g_decoders[] = {
    { "voice", 1, voice_init_decoder, voice_process_sample_decoder, voice_flush_decoder },
    { "ais",   0, ais_init_decoder, ais_process_sample, ais_flush },
    { NULL,    0, NULL, NULL, NULL }
};

static const decoder_ops_t* g_decoder = NULL;

static const decoder_ops_t* find_decoder(const char* name) {
    if (!name) return NULL;
    if (streq_icase(name, "voce")) name = "voice";
    for (int i = 0; g_decoders[i].name; i++) {
        if (streq_icase(name, g_decoders[i].name)) return &g_decoders[i];
    }
    return NULL;
}

static void rtlsdr_cb(unsigned char* buf, uint32_t len, void* ctx) {
    (void)ctx;
    if (g_stop) return;

    // buf contains interleaved unsigned I/Q bytes: I0 Q0 I1 Q1 ...
    // Convert to signed centered around 0: (byte - 127.5)
    // We do boxcar decimations to reduce rate and CPU.

    for (uint32_t p = 0; p + 1 < len; p += 2) {
        int32_t i = (int32_t)buf[p]   - 127;
        int32_t q = (int32_t)buf[p+1] - 127;

        // --- Decim1: 2.4M -> 240k by averaging 10 samples ---
        g_dsp.acc1_i += i;
        g_dsp.acc1_q += q;
        g_dsp.acc1_count++;

        if (g_dsp.acc1_count == DECIM1) {
            float I1 = (float)g_dsp.acc1_i / (float)DECIM1;
            float Q1 = (float)g_dsp.acc1_q / (float)DECIM1;

            g_dsp.acc1_i = 0;
            g_dsp.acc1_q = 0;
            g_dsp.acc1_count = 0;

            // --- FM demod at 240k ---
            float dem;
            if (!g_dsp.have_prev) {
                g_dsp.i_prev = I1;
                g_dsp.q_prev = Q1;
                g_dsp.have_prev = 1;
                continue;
            }
            dem = fm_discriminator(I1, Q1, g_dsp.i_prev, g_dsp.q_prev);
            g_dsp.i_prev = I1;
            g_dsp.q_prev = Q1;

            // --- Decim2: 240k -> 48k (average 5 demod samples) ---
            g_dsp.acc2 += dem;
            g_dsp.acc2_count++;

            if (g_dsp.acc2_count == DECIM2) {
                float a = g_dsp.acc2 / (float)DECIM2;
                g_dsp.acc2 = 0.0f;
                g_dsp.acc2_count = 0;
                if (g_decoder && g_decoder->process_sample) {
                    g_decoder->process_sample(a);
                }
            }
        }
    }
}

// --------- Decoder implementations ----------
static void voice_ring_output(void* user, const float* samples, uint32_t n) {
    (void)user;
    ring_push(samples, n);
}

static void voice_init_decoder(int fs_demod) {
    voice_decoder_init(&g_voice, fs_demod, voice_ring_output, NULL);
}

static void voice_process_sample_decoder(float sample) {
    voice_decoder_process_sample(&g_voice, sample);
}

static void voice_flush_decoder(void) {
    voice_decoder_flush(&g_voice);
}

static void ais_init_decoder(int fs_demod) {
    ais_init(&g_ais, fs_demod);
    g_dsp.ais_tmp_len = 0;
}

static void ais_process_sample(float sample) {
    g_dsp.ais_tmp[g_dsp.ais_tmp_len++] = sample;
    if (g_dsp.ais_tmp_len >= 4096) {
        ais_feed_samples(&g_ais, g_dsp.ais_tmp, g_dsp.ais_tmp_len);
        g_dsp.ais_tmp_len = 0;
    }
}

static void ais_flush(void) {
    if (g_dsp.ais_tmp_len == 0) return;
    ais_feed_samples(&g_ais, g_dsp.ais_tmp, g_dsp.ais_tmp_len);
    g_dsp.ais_tmp_len = 0;
}

static void usage(const char* prog) {
    fprintf(stderr, "Usage: %s <freq_mhz> [gain_db] [mode]\n", prog);
    fprintf(stderr, "       %s <freq_mhz> [gain_db] --mode <voice|ais>\n", prog);
    fprintf(stderr, "       %s <freq_mhz> --mode ais --ais-test\n", prog);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  --ppm <int>   frequency correction (e.g. -20, +35)\n");
    fprintf(stderr, "  --bw  <hz>    tuner bandwidth (Hz), 0=auto\n");
    fprintf(stderr, "Example: %s 100.0 20 voice\n", prog);
    fprintf(stderr, "         %s 162.0 --mode ais\n", prog);
}

static int parse_int_arg(const char* s, int* out) {
    char* end = NULL;
    long v = strtol(s, &end, 10);
    if (s == end || *end != '\0') return 0;
    *out = (int)v;
    return 1;
}

int main(int argc, char** argv) {
    signal(SIGINT, on_sigint);
    char* freq_opt = argv[1];

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    const char* decoder_name = "voice";
    int gain = 0;
    int use_manual_gain = 0;
    int ais_test = 0;
    int ppm = 0;
    int have_ppm = 0;
    uint32_t tuner_bw = 0;

    for (int i = 2; i < argc; i++) {
        const char* arg = argv[i];
        if (strcmp(arg, "--mode") == 0 || strcmp(arg, "--decoder") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for %s\n", arg);
                usage(argv[0]);
                return 1;
            }
            decoder_name = argv[++i];
            continue;
        }
        if (strcmp(arg, "--ais-test") == 0) {
            ais_test = 1;
            continue;
        }
        if (strcmp(arg, "--ppm") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for %s\n", arg);
                usage(argv[0]);
                return 1;
            }
            int parsed_ppm = 0;
            if (!parse_int_arg(argv[++i], &parsed_ppm)) {
                fprintf(stderr, "Invalid value for --ppm\n");
                return 1;
            }
            ppm = parsed_ppm;
            have_ppm = 1;
            continue;
        }
        if (strcmp(arg, "--bw") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for %s\n", arg);
                usage(argv[0]);
                return 1;
            }
            int parsed_bw = 0;
            if (!parse_int_arg(argv[++i], &parsed_bw) || parsed_bw < 0) {
                fprintf(stderr, "Invalid value for --bw\n");
                return 1;
            }
            tuner_bw = (uint32_t)parsed_bw;
            continue;
        }
      
        if (streq_icase(arg, "voice") || streq_icase(arg, "voce") || streq_icase(arg, "ais")) {
            decoder_name = arg;
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

    g_decoder = find_decoder(decoder_name);
    if (!g_decoder) {
        fprintf(stderr, "Unknown mode/decoder: %s\n", decoder_name);
        usage(argv[0]);
        return 1;
    }

    double freq_mhz = atof(freq_opt);
    if (freq_mhz < 10.0) {
        fprintf(stderr, "Invalid frequency.\n");
        return 1;
    }
    uint32_t freq_hz = (uint32_t)(freq_mhz * 1e6);

    memset(&g_ring, 0, sizeof(g_ring));
    memset(&g_dsp, 0, sizeof(g_dsp));
    if (g_decoder->init) g_decoder->init(AUDIO_FS);

    if (ais_test) {
        if (streq_icase(g_decoder->name, "ais")) {
            ais_test_emit_example();
            return 0;
        }
        fprintf(stderr, "--ais-test works only with mode=ais\n");
        return 1;
    }

    // ---- init RTL-SDR ----
    rtlsdr_dev_t* dev = NULL;
    int r = rtlsdr_open(&dev, 0);
    if (r < 0) {
        fprintf(stderr, "rtlsdr_open failed\n");
        return 1;
    }
    g_dev = dev;

    if (have_ppm) {
        r = rtlsdr_set_freq_correction(dev, ppm);
        if (r < 0) {
            fprintf(stderr, "Warning: rtlsdr_set_freq_correction failed (%d)\n", r);
        }
    }

    // Set sample rate + center frequency
    r = rtlsdr_set_sample_rate(dev, SDR_FS);
    if (r < 0) {
        fprintf(stderr, "rtlsdr_set_sample_rate failed (%d)\n", r);
        g_dev = NULL;
        rtlsdr_close(dev);
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
        return 1;
    }

    // Gain
    if (use_manual_gain) {
        // RTL-SDR expects tenths of dB
        rtlsdr_set_tuner_gain_mode(dev, 1);
        rtlsdr_set_tuner_gain(dev, gain * 10);
    } else {
        rtlsdr_set_tuner_gain_mode(dev, 0); // auto
    }

    // Reset buffer
    rtlsdr_reset_buffer(dev);

    char gain_desc[32];
    if (use_manual_gain) {
        snprintf(gain_desc, sizeof(gain_desc), "%d dB", gain);
    } else {
        snprintf(gain_desc, sizeof(gain_desc), "auto");
    }

    printf("START WFM C (async)\n");
    printf("  freq=%.3f MHz | sdr_fs=%d | audio_fs=%d | gain=%s | mode=%s",
           freq_mhz, SDR_FS, AUDIO_FS, gain_desc, g_decoder->name);
    if (have_ppm) printf(" | ppm=%d", ppm);
    if (tuner_bw) printf(" | bw=%u", tuner_bw);
    printf("\n");
    printf("  Ctrl+C to stop\n");

    PaStream* stream = NULL;
    PaError pe = paNoError;
    if (g_decoder->needs_audio) {
        // ---- init PortAudio ----
        pe = Pa_Initialize();
        if (pe != paNoError) {
            fprintf(stderr, "Pa_Initialize error: %s\n", Pa_GetErrorText(pe));
            g_dev = NULL;
            rtlsdr_close(dev);
            return 1;
        }

        pe = Pa_OpenDefaultStream(&stream,
                                  0,          // no input
                                  1,          // mono output
                                  paFloat32,  // float32 output
                                  AUDIO_FS,
                                  1024,       // frames per buffer
                                  pa_cb,
                                  NULL);
        if (pe != paNoError) {
            fprintf(stderr, "Pa_OpenDefaultStream error: %s\n", Pa_GetErrorText(pe));
            Pa_Terminate();
            g_dev = NULL;
            rtlsdr_close(dev);
            return 1;
        }

        pe = Pa_StartStream(stream);
        if (pe != paNoError) {
            fprintf(stderr, "Pa_StartStream error: %s\n", Pa_GetErrorText(pe));
            Pa_CloseStream(stream);
            Pa_Terminate();
            g_dev = NULL;
            rtlsdr_close(dev);
            return 1;
        }
    }

    // ---- start async RTL read (blocking call) ----
    // Buffer length chosen moderate; librtlsdr will manage streaming.
    // This will return when rtlsdr_cancel_async is called or error occurs.
    r = rtlsdr_read_async(dev, rtlsdr_cb, NULL, 0, 0);

    // Cleanup
    g_stop = 1;
    if (g_decoder && g_decoder->flush) g_decoder->flush();
    if (g_decoder->needs_audio) {
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        Pa_Terminate();
    }

    g_dev = NULL;
    rtlsdr_close(dev);

    if (r < 0) {
        fprintf(stderr, "rtlsdr_read_async exited with error %d\n", r);
        return 1;
    }
    return 0;
}
