#include "header/am_demod.h"

#include <math.h>
#include <string.h>

int am_demod_init(am_demod_ctx_t *ctx, const am_demod_config_t *cfg,
                  am_demod_output_cb_t out_cb, void *out_user) {
    if (!ctx || !cfg || !out_cb) return 0;
    if (cfg->input_fs <= 0 || cfg->output_fs <= 0) return 0;
    if (cfg->input_fs != cfg->output_fs) return 0;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->out_cb = out_cb;
    ctx->out_user = out_user;
    return 1;
}

void am_demod_process_raw_iq_u8(am_demod_ctx_t *ctx, const unsigned char *buf, uint32_t len) {
    if (!ctx || !ctx->out_cb || !buf) return;

    for (uint32_t p = 0; p + 1 < len; p += 2) {
        float i = (float)((int32_t)buf[p] - 127);
        float q = (float)((int32_t)buf[p + 1] - 127);
        float mag = sqrtf(i * i + q * q);

        if (ctx->cfg.dc_alpha > 0.0f && ctx->cfg.dc_alpha < 1.0f) {
            ctx->dc_estimate += ctx->cfg.dc_alpha * (mag - ctx->dc_estimate);
            mag -= ctx->dc_estimate;
        }

        ctx->out_cb(ctx->out_user, mag);
    }
}

void am_demod_flush(am_demod_ctx_t *ctx) {
    (void)ctx;
}
