#include "header/demodulator.h"

#include <string.h>

int demodulator_init(demodulator_t *ctx, const demod_config_t *config, const demod_output_t *output) {
    if (!ctx || !config || !output) return 0;

    memset(ctx, 0, sizeof(*ctx));
    ctx->kind = config->kind;

    switch (config->kind) {
    case DEMOD_KIND_AM: {
        am_demod_config_t am_cfg;
        memset(&am_cfg, 0, sizeof(am_cfg));
        am_cfg.input_fs = config->input_fs;
        am_cfg.output_fs = config->output_fs;
        am_cfg.dc_alpha = config->u.am.dc_alpha;
        return am_demod_init(&ctx->u.am, &am_cfg, output->on_float, output->user);
    }
    case DEMOD_KIND_FM: {
        fm_demod_config_t fm_cfg;
        memset(&fm_cfg, 0, sizeof(fm_cfg));
        fm_cfg.input_fs = config->input_fs;
        fm_cfg.output_fs = config->output_fs;
        fm_cfg.dc_alpha = config->u.fm.dc_alpha;
        return fm_demod_init(&ctx->u.fm, &fm_cfg, output->on_float, output->user);
    }
    case DEMOD_KIND_GMSK: {
        gmsk_demod_config_t gmsk_cfg;
        memset(&gmsk_cfg, 0, sizeof(gmsk_cfg));
        gmsk_cfg.input_fs = config->input_fs;
        gmsk_cfg.output_fs = config->output_fs;
        gmsk_cfg.symbol_rate = config->u.gmsk.symbol_rate;
        gmsk_cfg.m = config->u.gmsk.m;
        gmsk_cfg.bt = config->u.gmsk.bt;
        return gmsk_demod_init(&ctx->u.gmsk, &gmsk_cfg, output->on_bit, output->user);
    }
    case DEMOD_KIND_NONE:
    default:
        return 0;
    }
}

void demodulator_process_raw_iq_u8(demodulator_t *ctx, const unsigned char *buf, uint32_t len) {
    if (!ctx || !buf) return;

    switch (ctx->kind) {
    case DEMOD_KIND_AM:
        am_demod_process_raw_iq_u8(&ctx->u.am, buf, len);
        break;
    case DEMOD_KIND_FM:
        fm_demod_process_raw_iq_u8(&ctx->u.fm, buf, len);
        break;
    case DEMOD_KIND_GMSK:
        gmsk_demod_process_raw_iq_u8(&ctx->u.gmsk, buf, len);
        break;
    case DEMOD_KIND_NONE:
    default:
        break;
    }
}

void demodulator_flush(demodulator_t *ctx) {
    if (!ctx) return;

    switch (ctx->kind) {
    case DEMOD_KIND_AM:
        am_demod_flush(&ctx->u.am);
        break;
    case DEMOD_KIND_FM:
        fm_demod_flush(&ctx->u.fm);
        break;
    case DEMOD_KIND_GMSK:
        gmsk_demod_flush(&ctx->u.gmsk);
        break;
    case DEMOD_KIND_NONE:
    default:
        break;
    }
    ctx->kind = DEMOD_KIND_NONE;
}
