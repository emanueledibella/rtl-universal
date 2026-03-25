#include "header/voice_module.h"

#include <ctype.h>
#include <portaudio.h>
#include <stdio.h>
#include <string.h>

static int voice_module_streq_icase(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static void voice_module_ring_push(voice_module_t *ctx, const float *samples, uint32_t n) {
    if (!ctx || !samples || !ctx->ring_lock_ready) return;

    pthread_mutex_lock(&ctx->ring_lock);
    for (uint32_t i = 0; i < n; i++) {
        ctx->ring[ctx->ring_w] = samples[i];
        ctx->ring_w = (ctx->ring_w + 1u) % voice_MODULE_RING_SIZE;
        if (ctx->ring_w == ctx->ring_r) {
            ctx->ring_r = (ctx->ring_r + 1u) % voice_MODULE_RING_SIZE;
        }
    }
    pthread_mutex_unlock(&ctx->ring_lock);
}

static void voice_module_ring_pop(voice_module_t *ctx, float *out, uint32_t n) {
    if (!ctx || !out || !ctx->ring_lock_ready) return;

    pthread_mutex_lock(&ctx->ring_lock);
    for (uint32_t i = 0; i < n; i++) {
        if (ctx->ring_r == ctx->ring_w) {
            out[i] = 0.0f;
        } else {
            out[i] = ctx->ring[ctx->ring_r];
            ctx->ring_r = (ctx->ring_r + 1u) % voice_MODULE_RING_SIZE;
        }
    }
    pthread_mutex_unlock(&ctx->ring_lock);
}

static int voice_module_pa_cb(const void *input, void *output,
                              unsigned long frame_count,
                              const PaStreamCallbackTimeInfo *time_info,
                              PaStreamCallbackFlags status_flags,
                              void *user_data) {
    voice_module_t *ctx = (voice_module_t *)user_data;
    float *out = (float *)output;

    (void)input;
    (void)time_info;
    (void)status_flags;

    voice_module_ring_pop(ctx, out, (uint32_t)frame_count);
    return paContinue;
}

static void voice_module_audio_out_cb(void *user, const float *samples, uint32_t n) {
    voice_module_ring_push((voice_module_t *)user, samples, n);
}

static void voice_module_on_demod_sample_cb(void *user, float sample) {
    voice_module_t *ctx = (voice_module_t *)user;
    if (!ctx) return;
    voice_decoder_process_sample(&ctx->decoder, sample);
}

void voice_module_reset(voice_module_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->demod_kind = VOICE_DEMOD_KIND_FM;
}

int voice_module_set_demod(voice_module_t *ctx, const char *name) {
    if (!ctx) return 0;
    if (!name || name[0] == '\0'
        || voice_module_streq_icase(name, "fm")
        || voice_module_streq_icase(name, "nfm")
        || voice_module_streq_icase(name, "wfm")) {
        ctx->demod_kind = VOICE_DEMOD_KIND_FM;
        return 1;
    }
    if (voice_module_streq_icase(name, "am")) {
        ctx->demod_kind = VOICE_DEMOD_KIND_AM;
        return 1;
    }
    return 0;
}

const char *voice_module_demod_name(const voice_module_t *ctx) {
    if (!ctx) return "fm";
    switch (ctx->demod_kind) {
    case VOICE_DEMOD_KIND_AM:
        return "am";
    case VOICE_DEMOD_KIND_FM:
    default:
        return "fm";
    }
}

void voice_module_get_demod_config(voice_module_t *ctx, demod_config_t *cfg) {
    if (!cfg) return;

    memset(cfg, 0, sizeof(*cfg));
    cfg->input_fs = voice_MODULE_INPUT_FS;
    cfg->output_fs = voice_MODULE_OUTPUT_FS;

    if (ctx && ctx->demod_kind == VOICE_DEMOD_KIND_AM) {
        cfg->kind = DEMOD_KIND_AM;
        cfg->u.am.dc_alpha = 0.0005f;
        return;
    }

    cfg->kind = DEMOD_KIND_FM;
    cfg->u.fm.dc_alpha = 0.0005f;
}

int voice_module_init(voice_module_t *ctx, const demod_config_t *cfg) {
    PaDeviceIndex device_index;
    PaError pa_err;
    float deemph_tau_s = 0.0f;

    if (!ctx || !cfg || cfg->output_fs <= 0) {
        fprintf(stderr, "[voice] invalid init config\n");
        return 0;
    }

    ctx->audio_fs = cfg->output_fs;
    ctx->ring_w = 0u;
    ctx->ring_r = 0u;

    if (pthread_mutex_init(&ctx->ring_lock, NULL) != 0) {
        fprintf(stderr, "[voice] pthread_mutex_init failed\n");
        return 0;
    }
    ctx->ring_lock_ready = 1;

    if (ctx->demod_kind == VOICE_DEMOD_KIND_FM) {
        deemph_tau_s = 50e-6f;
    }
    voice_decoder_init(&ctx->decoder, cfg->output_fs, deemph_tau_s,
                       voice_module_audio_out_cb, ctx);

    pa_err = Pa_Initialize();
    if (pa_err != paNoError) {
        fprintf(stderr, "[voice] Pa_Initialize error: %s\n", Pa_GetErrorText(pa_err));
        return 0;
    }
    ctx->pa_initialized = 1;

    device_index = Pa_GetDefaultOutputDevice();
    if (device_index == paNoDevice) {
        fprintf(stderr, "[voice] No default audio output device available\n");
        return 0;
    }

    pa_err = Pa_OpenDefaultStream((PaStream **)&ctx->stream,
                                  0,
                                  1,
                                  paFloat32,
                                  (double)ctx->audio_fs,
                                  1024,
                                  voice_module_pa_cb,
                                  ctx);
    if (pa_err != paNoError) {
        fprintf(stderr, "[voice] Pa_OpenDefaultStream error: %s\n", Pa_GetErrorText(pa_err));
        return 0;
    }

    pa_err = Pa_StartStream((PaStream *)ctx->stream);
    if (pa_err != paNoError) {
        fprintf(stderr, "[voice] Pa_StartStream error: %s\n", Pa_GetErrorText(pa_err));
        return 0;
    }

    return 1;
}

demod_output_t voice_module_get_demod_output(voice_module_t *ctx) {
    demod_output_t out;

    memset(&out, 0, sizeof(out));
    out.on_float = voice_module_on_demod_sample_cb;
    out.user = ctx;
    return out;
}

void voice_module_flush(voice_module_t *ctx) {
    if (!ctx) return;

    voice_decoder_flush(&ctx->decoder);

    if (ctx->stream) {
        Pa_StopStream((PaStream *)ctx->stream);
        Pa_CloseStream((PaStream *)ctx->stream);
        ctx->stream = NULL;
    }
    if (ctx->pa_initialized) {
        Pa_Terminate();
        ctx->pa_initialized = 0;
    }
    if (ctx->ring_lock_ready) {
        pthread_mutex_destroy(&ctx->ring_lock);
        ctx->ring_lock_ready = 0;
    }
}
