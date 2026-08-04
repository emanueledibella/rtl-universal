#include "spectrum.h"

#include <complex.h>
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <liquid/liquid.h>

#define SPECTRUM_MIN_DBFS (-140.0f)
#define SPECTRUM_MAX_DBFS (20.0f)

struct spectrum_analyzer {
    int sample_rate;
    uint32_t center_frequency_hz;
    unsigned int fft_size;
    uint64_t skip_samples;
    uint64_t skip_remaining;
    unsigned int capture_count;
    double frames_per_second;
    FILE *output;

    unsigned char *capture;
    unsigned char *pending;
    int pending_ready;
    int stop;

    pthread_mutex_t lock;
    pthread_cond_t condition;
    pthread_t worker;
    int lock_ready;
    int condition_ready;
    int worker_ready;
};

static int spectrum_valid_fft_size(unsigned int fft_size) {
    return fft_size >= 256u && fft_size <= 4096u
           && (fft_size & (fft_size - 1u)) == 0u;
}

static float clamp_dbfs(float value) {
    if (!isfinite(value) || value < SPECTRUM_MIN_DBFS) return SPECTRUM_MIN_DBFS;
    if (value > SPECTRUM_MAX_DBFS) return SPECTRUM_MAX_DBFS;
    return value;
}

int spectrum_compute_dbfs(const unsigned char *samples,
                          unsigned int fft_size,
                          float *bins_dbfs) {
    liquid_float_complex *input = NULL;
    liquid_float_complex *output = NULL;
    fftplan plan = NULL;
    double window_sum = 0.0;
    int ok = 0;

    if (!samples || !bins_dbfs || !spectrum_valid_fft_size(fft_size)) return 0;

    input = (liquid_float_complex *)fft_malloc(
        fft_size * (unsigned int)sizeof(*input));
    output = (liquid_float_complex *)fft_malloc(
        fft_size * (unsigned int)sizeof(*output));
    if (!input || !output) goto cleanup;

    for (unsigned int i = 0u; i < fft_size; i++) {
        float phase = (float)(2.0 * M_PI * (double)i / (double)(fft_size - 1u));
        float window = 0.5f - 0.5f * cosf(phase);
        float in_phase = ((float)samples[i * 2u] - 127.5f) / 127.5f;
        float quadrature = ((float)samples[i * 2u + 1u] - 127.5f) / 127.5f;
        input[i] = window * (in_phase + _Complex_I * quadrature);
        window_sum += window;
    }

    plan = fft_create_plan(fft_size, input, output, LIQUID_FFT_FORWARD, 0);
    if (!plan || window_sum <= 0.0) goto cleanup;
    (void)fft_execute(plan);

    for (unsigned int i = 0u; i < fft_size; i++) {
        unsigned int shifted = (i + fft_size / 2u) % fft_size;
        float magnitude = cabsf(output[shifted]) / (float)window_sum;
        bins_dbfs[i] = clamp_dbfs(20.0f * log10f(fmaxf(magnitude, 1e-7f)));
    }
    ok = 1;

cleanup:
    if (plan) (void)fft_destroy_plan(plan);
    if (input) fft_free(input);
    if (output) fft_free(output);
    return ok;
}

static void emit_spectrum_frame(spectrum_analyzer_t *ctx,
                                const unsigned char *samples,
                                float *bins) {
    float minimum = SPECTRUM_MAX_DBFS;
    float maximum = SPECTRUM_MIN_DBFS;

    if (!spectrum_compute_dbfs(samples, ctx->fft_size, bins)) return;
    for (unsigned int i = 0u; i < ctx->fft_size; i++) {
        if (bins[i] < minimum) minimum = bins[i];
        if (bins[i] > maximum) maximum = bins[i];
    }

    (void)fprintf(ctx->output,
                  "{\"type\":\"spectrum\",\"center_hz\":%u,"
                  "\"sample_rate\":%d,\"fft_size\":%u,"
                  "\"min_dbfs\":%.1f,\"max_dbfs\":%.1f,\"bins\":[",
                  ctx->center_frequency_hz, ctx->sample_rate, ctx->fft_size,
                  minimum, maximum);
    for (unsigned int i = 0u; i < ctx->fft_size; i++) {
        if (i > 0u) (void)fputc(',', ctx->output);
        (void)fprintf(ctx->output, "%.1f", bins[i]);
    }
    (void)fputs("]}\n", ctx->output);
    (void)fflush(ctx->output);
}

static void *spectrum_worker(void *user) {
    spectrum_analyzer_t *ctx = (spectrum_analyzer_t *)user;
    float *bins = NULL;
    unsigned char *local = NULL;

    if (!ctx) return NULL;
    bins = (float *)malloc(ctx->fft_size * sizeof(*bins));
    local = (unsigned char *)malloc(ctx->fft_size * 2u);
    if (!bins || !local) goto cleanup;

    for (;;) {
        (void)pthread_mutex_lock(&ctx->lock);
        while (!ctx->pending_ready && !ctx->stop) {
            (void)pthread_cond_wait(&ctx->condition, &ctx->lock);
        }
        if (ctx->stop && !ctx->pending_ready) {
            (void)pthread_mutex_unlock(&ctx->lock);
            break;
        }
        memcpy(local, ctx->pending, ctx->fft_size * 2u);
        ctx->pending_ready = 0;
        (void)pthread_mutex_unlock(&ctx->lock);

        emit_spectrum_frame(ctx, local, bins);
    }

cleanup:
    free(local);
    free(bins);
    return NULL;
}

spectrum_analyzer_t *spectrum_analyzer_create(int sample_rate,
                                              uint32_t center_frequency_hz,
                                              unsigned int fft_size,
                                              double frames_per_second,
                                              FILE *output) {
    spectrum_analyzer_t *ctx;
    double requested_stride;

    if (sample_rate <= 0 || center_frequency_hz == 0u || !output
        || !spectrum_valid_fft_size(fft_size)
        || !isfinite(frames_per_second)
        || frames_per_second < 1.0 || frames_per_second > 60.0) {
        return NULL;
    }

    ctx = (spectrum_analyzer_t *)calloc(1u, sizeof(*ctx));
    if (!ctx) return NULL;
    ctx->sample_rate = sample_rate;
    ctx->center_frequency_hz = center_frequency_hz;
    ctx->fft_size = fft_size;
    ctx->frames_per_second = frames_per_second;
    ctx->output = output;
    ctx->capture = (unsigned char *)malloc(fft_size * 2u);
    ctx->pending = (unsigned char *)malloc(fft_size * 2u);
    if (!ctx->capture || !ctx->pending) goto fail;

    requested_stride = (double)sample_rate / frames_per_second;
    ctx->skip_samples = requested_stride > (double)fft_size
                            ? (uint64_t)llround(requested_stride) - fft_size
                            : 0u;

    if (pthread_mutex_init(&ctx->lock, NULL) != 0) goto fail;
    ctx->lock_ready = 1;
    if (pthread_cond_init(&ctx->condition, NULL) != 0) goto fail;
    ctx->condition_ready = 1;
    if (pthread_create(&ctx->worker, NULL, spectrum_worker, ctx) != 0) goto fail;
    ctx->worker_ready = 1;
    return ctx;

fail:
    spectrum_analyzer_destroy(ctx);
    return NULL;
}

void spectrum_analyzer_feed_u8(spectrum_analyzer_t *ctx,
                               const unsigned char *samples,
                               uint32_t length) {
    if (!ctx || !samples) return;

    for (uint32_t p = 0u; p + 1u < length; p += 2u) {
        if (ctx->skip_remaining > 0u) {
            ctx->skip_remaining--;
            continue;
        }
        ctx->capture[ctx->capture_count * 2u] = samples[p];
        ctx->capture[ctx->capture_count * 2u + 1u] = samples[p + 1u];
        ctx->capture_count++;
        if (ctx->capture_count < ctx->fft_size) continue;

        (void)pthread_mutex_lock(&ctx->lock);
        if (!ctx->pending_ready) {
            memcpy(ctx->pending, ctx->capture, ctx->fft_size * 2u);
            ctx->pending_ready = 1;
            (void)pthread_cond_signal(&ctx->condition);
        }
        (void)pthread_mutex_unlock(&ctx->lock);
        ctx->capture_count = 0u;
        ctx->skip_remaining = ctx->skip_samples;
    }
}

void spectrum_analyzer_destroy(spectrum_analyzer_t *ctx) {
    if (!ctx) return;
    if (ctx->lock_ready) {
        (void)pthread_mutex_lock(&ctx->lock);
        ctx->stop = 1;
        if (ctx->condition_ready) (void)pthread_cond_signal(&ctx->condition);
        (void)pthread_mutex_unlock(&ctx->lock);
    }
    if (ctx->worker_ready) (void)pthread_join(ctx->worker, NULL);
    if (ctx->condition_ready) (void)pthread_cond_destroy(&ctx->condition);
    if (ctx->lock_ready) (void)pthread_mutex_destroy(&ctx->lock);
    free(ctx->capture);
    free(ctx->pending);
    free(ctx);
}
