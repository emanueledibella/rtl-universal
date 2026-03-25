#include "header/gmsk_demod.h"

#include <stdlib.h>
#include <string.h>

int gmsk_demod_init(gmsk_demod_ctx_t *ctx, const gmsk_demod_config_t *cfg,
                    gmsk_demod_output_cb_t out_cb, void *out_user) {
    if (!ctx || !cfg || !out_cb) return 0;
    if (cfg->input_fs <= 0 || cfg->output_fs <= 0 || cfg->symbol_rate == 0) return 0;
    if (cfg->input_fs % cfg->output_fs != 0) return 0;
    if (cfg->output_fs % (int)cfg->symbol_rate != 0) return 0;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->out_cb = out_cb;
    ctx->out_user = out_user;
    ctx->decim = (unsigned int)(cfg->input_fs / cfg->output_fs);
    ctx->k = (unsigned int)(cfg->output_fs / (int)cfg->symbol_rate);
    if (ctx->k < 2) return 0;

    ctx->demod = gmskdem_create(ctx->k, cfg->m, cfg->bt);
    if (!ctx->demod) return 0;

    ctx->sym_buf = (liquid_float_complex *)calloc(ctx->k, sizeof(liquid_float_complex));
    if (!ctx->sym_buf) {
        gmskdem_destroy(ctx->demod);
        ctx->demod = NULL;
        return 0;
    }
    return 1;
}

void gmsk_demod_process_raw_iq_u8(gmsk_demod_ctx_t *ctx, const unsigned char *buf, uint32_t len) {
    if (!ctx || !ctx->demod || !ctx->sym_buf || !ctx->out_cb || !buf) return;

    for (uint32_t p = 0; p + 1 < len; p += 2) {
        ctx->acc_i += (int32_t)buf[p] - 127;
        ctx->acc_q += (int32_t)buf[p + 1] - 127;
        ctx->decim_count++;

        if (ctx->decim_count != ctx->decim) continue;

        float i = (float)ctx->acc_i / (float)ctx->decim;
        float q = (float)ctx->acc_q / (float)ctx->decim;
        ctx->acc_i = 0;
        ctx->acc_q = 0;
        ctx->decim_count = 0;

        ctx->sym_buf[ctx->sym_idx++] = i + _Complex_I * q;
        if (ctx->sym_idx >= ctx->k) {
            unsigned int sym = 0;
            gmskdem_demodulate(ctx->demod, ctx->sym_buf, &sym);
            ctx->out_cb(ctx->out_user, (uint8_t)(sym & 1u));
            ctx->sym_idx = 0;
        }
    }
}

void gmsk_demod_flush(gmsk_demod_ctx_t *ctx) {
    if (!ctx) return;
    if (ctx->demod) {
        gmskdem_destroy(ctx->demod);
        ctx->demod = NULL;
    }
    if (ctx->sym_buf) {
        free(ctx->sym_buf);
        ctx->sym_buf = NULL;
    }
    ctx->sym_idx = 0;
}
