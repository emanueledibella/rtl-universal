#include "header/fm_demod.h"

#include <math.h>
#include <string.h>

static inline float fm_demod_discriminator(float i, float q, float i_prev, float q_prev) {
    float re = i * i_prev + q * q_prev;
    float im = q * i_prev - i * q_prev;
    return atan2f(im, re);
}

int fm_demod_init(fm_demod_ctx_t *ctx, const fm_demod_config_t *cfg,
                  fm_demod_output_cb_t out_cb, void *out_user) {
    int decim = 0;

    if (!ctx || !cfg || !out_cb) return 0;
    if (cfg->input_fs <= 0 || cfg->output_fs <= 0) return 0;
    if ((cfg->input_fs % cfg->output_fs) != 0) return 0;

    decim = cfg->input_fs / cfg->output_fs;
    if (decim <= 0) return 0;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->out_cb = out_cb;
    ctx->out_user = out_user;
    ctx->decim = decim;
    return 1;
}

void fm_demod_process_raw_iq_u8(fm_demod_ctx_t *ctx, const unsigned char *buf, uint32_t len) {
    if (!ctx || !ctx->out_cb || !buf) return;

    for (uint32_t p = 0; p + 1 < len; p += 2) {
        float i = (float)((int32_t)buf[p] - 127);
        float q = (float)((int32_t)buf[p + 1] - 127);
        float demod = 0.0f;

        if (!ctx->have_prev) {
            ctx->prev_i = i;
            ctx->prev_q = q;
            ctx->have_prev = 1;
            continue;
        }

        demod = fm_demod_discriminator(i, q, ctx->prev_i, ctx->prev_q);
        ctx->prev_i = i;
        ctx->prev_q = q;

        if (ctx->cfg.dc_alpha > 0.0f && ctx->cfg.dc_alpha < 1.0f) {
            ctx->dc_estimate += ctx->cfg.dc_alpha * (demod - ctx->dc_estimate);
            demod -= ctx->dc_estimate;
        }

        ctx->decim_acc += demod;
        ctx->decim_count++;
        if (ctx->decim_count >= ctx->decim) {
            ctx->out_cb(ctx->out_user, ctx->decim_acc / (float)ctx->decim_count);
            ctx->decim_acc = 0.0f;
            ctx->decim_count = 0;
        }
    }
}

void fm_demod_flush(fm_demod_ctx_t *ctx) {
    if (!ctx || !ctx->out_cb) return;
    if (ctx->decim_count <= 0) return;

    ctx->out_cb(ctx->out_user, ctx->decim_acc / (float)ctx->decim_count);
    ctx->decim_acc = 0.0f;
    ctx->decim_count = 0;
}
