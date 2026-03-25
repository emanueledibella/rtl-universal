#include "header/voice_decoder.h"
#include <string.h>
#include <math.h>

static inline float deemph_process(voice_ctx_t* ctx, float x) {
    float y = (1.0f - ctx->deemph_a) * x + ctx->deemph_a * ctx->deemph_y1;
    ctx->deemph_y1 = y;
    return y;
}

static void normalize_block(float* x, uint32_t n, float target) {
    float m = 1e-9f;
    for (uint32_t i = 0; i < n; i++) {
        float a = fabsf(x[i]);
        if (a > m) m = a;
    }
    float k = target / m;
    for (uint32_t i = 0; i < n; i++) x[i] *= k;
}

static void voice_flush_block(voice_ctx_t* ctx) {
    if (ctx->tmp_len == 0) return;

    // Remove DC and normalize lightly
    float mean = 0.0f;
    for (uint32_t k = 0; k < ctx->tmp_len; k++) mean += ctx->tmp[k];
    mean /= (float)ctx->tmp_len;
    for (uint32_t k = 0; k < ctx->tmp_len; k++) ctx->tmp[k] -= mean;

    normalize_block(ctx->tmp, ctx->tmp_len, ctx->norm_target);
    if (ctx->out_cb) ctx->out_cb(ctx->out_user, ctx->tmp, ctx->tmp_len);
    ctx->tmp_len = 0;
}

void voice_decoder_init(voice_ctx_t* ctx, int fs_demod, float deemph_tau_s,
                        voice_output_cb_t out_cb, void* out_user) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->out_cb = out_cb;
    ctx->out_user = out_user;

    if (deemph_tau_s > 0.0f && fs_demod > 0) {
        ctx->deemph_a = expf(-1.0f / ((float)fs_demod * deemph_tau_s));
    } else {
        ctx->deemph_a = 0.0f;
    }
    ctx->deemph_y1 = 0.0f;

    ctx->flush_threshold = 2048;
    ctx->norm_target = 0.5f;
}

void voice_decoder_process_sample(voice_ctx_t* ctx, float sample) {
    float y = deemph_process(ctx, sample);

    // Avoid overflow in case flush_threshold is misconfigured.
    if (ctx->tmp_len >= (uint32_t)(sizeof(ctx->tmp) / sizeof(ctx->tmp[0]))) {
        voice_flush_block(ctx);
    }

    ctx->tmp[ctx->tmp_len++] = y;
    if (ctx->tmp_len >= ctx->flush_threshold) {
        voice_flush_block(ctx);
    }
}

void voice_decoder_flush(voice_ctx_t* ctx) {
    voice_flush_block(ctx);
}
