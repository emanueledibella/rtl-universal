// wfm_live.c
// Minimal WFM (FM broadcast) mono receiver: RTL-SDR -> FM demod -> PortAudio
// Build (Homebrew):
//   cc -O2 -std=c11 wfm_live.c -o wfm_live $(pkg-config --cflags --libs librtlsdr portaudio-2.0) -lm
//
// Run:
//   ./wfm_live 100.0   (MHz)
// Optional gain (dB) as second arg:
//   ./wfm_live 100.0 20
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

#include <rtl-sdr.h>
#include <portaudio.h>

#define SDR_FS        2400000   // 2.4 MS/s
#define FS1           240000    // after decim1=10
#define AUDIO_FS      48000     // after decim2=5
#define DECIM1        10
#define DECIM2        5

// ring buffer seconds (audio)
#define RING_SECONDS  4
#define RING_SIZE     (AUDIO_FS * RING_SECONDS)

// ------------ globals / state ------------
static volatile int g_stop = 0;
static rtlsdr_dev_t* g_dev = NULL;

static void on_sigint(int sig) {
    (void)sig;
    g_stop = 1;
    if (g_dev) rtlsdr_cancel_async(g_dev);
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

// 1-pole de-emphasis filter y[n] = (1-a)*x[n] + a*y[n-1]
typedef struct {
    float a;
    float y1;
} deemph_t;

static deemph_t g_deemph;

static inline float deemph_process(deemph_t* d, float x) {
    float y = (1.0f - d->a) * x + d->a * d->y1;
    d->y1 = y;
    return y;
}

// Normalize block softly
static void normalize_block(float* x, uint32_t n, float target) {
    float m = 1e-9f;
    for (uint32_t i = 0; i < n; i++) {
        float a = fabsf(x[i]);
        if (a > m) m = a;
    }
    float k = target / m;
    for (uint32_t i = 0; i < n; i++) x[i] *= k;
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

    // Small temp audio buffer
    float audio_tmp[4096];
    uint32_t audio_tmp_len;
} dsp_state_t;

static dsp_state_t g_dsp;

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

                // De-emphasis (EU 50us)
                a = deemph_process(&g_deemph, a);

                // Append to temp buffer
                g_dsp.audio_tmp[g_dsp.audio_tmp_len++] = a;

                // Flush periodically
                if (g_dsp.audio_tmp_len >= 2048) {
                    // Remove DC and normalize lightly
                    float mean = 0.0f;
                    for (uint32_t k = 0; k < g_dsp.audio_tmp_len; k++) mean += g_dsp.audio_tmp[k];
                    mean /= (float)g_dsp.audio_tmp_len;
                    for (uint32_t k = 0; k < g_dsp.audio_tmp_len; k++) g_dsp.audio_tmp[k] -= mean;

                    normalize_block(g_dsp.audio_tmp, g_dsp.audio_tmp_len, 0.5f);
                    ring_push(g_dsp.audio_tmp, g_dsp.audio_tmp_len);
                    g_dsp.audio_tmp_len = 0;
                }
            }
        }
    }
}

static void usage(const char* prog) {
    fprintf(stderr, "Usage: %s <freq_mhz> [gain]\n", prog);
    fprintf(stderr, "Example: %s 100.0 20\n", prog);
}

int main(int argc, char** argv) {
    signal(SIGINT, on_sigint);

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    double freq_mhz = atof(argv[1]);
    if (freq_mhz < 10.0) {
        fprintf(stderr, "Invalid frequency.\n");
        return 1;
    }
    uint32_t freq_hz = (uint32_t)(freq_mhz * 1e6);

    int gain = 0; // 0 means auto in librtlsdr API? We'll set manual if provided
    int use_manual_gain = 0;
    if (argc >= 3) {
        gain = atoi(argv[2]);
        use_manual_gain = 1;
    }

    // De-emphasis alpha
    // a = exp(-1/(fs*tau))
    const float tau = 50e-6f;
    g_deemph.a = expf(-1.0f / ((float)AUDIO_FS * tau));
    g_deemph.y1 = 0.0f;

    memset(&g_ring, 0, sizeof(g_ring));
    memset(&g_dsp, 0, sizeof(g_dsp));

    // ---- init RTL-SDR ----
    rtlsdr_dev_t* dev = NULL;
    int r = rtlsdr_open(&dev, 0);
    if (r < 0) {
        fprintf(stderr, "rtlsdr_open failed\n");
        return 1;
    }
    g_dev = dev;

    // Set sample rate + center frequency
    rtlsdr_set_sample_rate(dev, SDR_FS);
    rtlsdr_set_center_freq(dev, freq_hz);

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

    printf("START WFM C (async)\n");
    printf("  freq=%.3f MHz | sdr_fs=%d | audio_fs=%d | gain=%s\n",
           freq_mhz, SDR_FS, AUDIO_FS, use_manual_gain ? argv[2] : "auto");
    printf("  Ctrl+C to stop\n");

    // ---- init PortAudio ----
    PaError pe = Pa_Initialize();
    if (pe != paNoError) {
        fprintf(stderr, "Pa_Initialize error: %s\n", Pa_GetErrorText(pe));
        g_dev = NULL;
        rtlsdr_close(dev);
        return 1;
    }

    PaStream* stream = NULL;
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

    // ---- start async RTL read (blocking call) ----
    // Buffer length chosen moderate; librtlsdr will manage streaming.
    // This will return when rtlsdr_cancel_async is called or error occurs.
    r = rtlsdr_read_async(dev, rtlsdr_cb, NULL, 0, 0);

    // Cleanup
    g_stop = 1;
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();

    g_dev = NULL;
    rtlsdr_close(dev);

    if (r < 0) {
        fprintf(stderr, "rtlsdr_read_async exited with error %d\n", r);
        return 1;
    }
    return 0;
}