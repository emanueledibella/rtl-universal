#include "header/demodulator.h"

#include <math.h>
#include <string.h>

int demodulator_init(demodulator_t *ctx, const demod_config_t *config, const demod_output_t *output) {
    if (!ctx || !config || !output) return 0;

    memset(ctx, 0, sizeof(*ctx));
    ctx->kind = config->kind;
    ctx->input_fs = config->input_fs;
    atomic_init(&ctx->squelch_update_pending, 0);
    atomic_init(&ctx->requested_squelch_enabled, 0);
    atomic_init(&ctx->requested_squelch_dbfs, 0.0f);
    atomic_init(&ctx->frequency_offset_update_pending, 0);
    atomic_init(&ctx->requested_frequency_offset_hz, 0.0f);
    atomic_init(&ctx->filter_update_pending, 0);
    atomic_init(&ctx->requested_filter_type, ANALOG_FILTER_NONE);
    atomic_init(&ctx->requested_filter_width_hz, 0);

    switch (config->kind) {
    case DEMOD_KIND_AM: {
        am_demod_config_t am_cfg;
        memset(&am_cfg, 0, sizeof(am_cfg));
        am_cfg.input_fs = config->input_fs;
        am_cfg.output_fs = config->output_fs;
        am_cfg.dc_alpha = config->u.am.dc_alpha;
        am_cfg.frontend = config->analog;
        return am_demod_init(&ctx->u.am, &am_cfg, output->on_float, output->user);
    }
    case DEMOD_KIND_FM: {
        fm_demod_config_t fm_cfg;
        memset(&fm_cfg, 0, sizeof(fm_cfg));
        fm_cfg.input_fs = config->input_fs;
        fm_cfg.output_fs = config->output_fs;
        fm_cfg.dc_alpha = config->u.fm.dc_alpha;
        fm_cfg.frontend = config->analog;
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
        gmsk_cfg.channel_count = config->u.gmsk.channel_count;
        gmsk_cfg.channel_offset_hz[0] = config->u.gmsk.channel_offset_hz[0];
        gmsk_cfg.channel_offset_hz[1] = config->u.gmsk.channel_offset_hz[1];
        return gmsk_demod_init(&ctx->u.gmsk, &gmsk_cfg, output->on_bit,
                               output->on_channel_bit, output->user);
    }
    case DEMOD_KIND_NONE:
    default:
        return 0;
    }
}

void demodulator_process_raw_iq_u8(demodulator_t *ctx, const unsigned char *buf, uint32_t len) {
    if (!ctx || !buf) return;

    if (atomic_exchange_explicit(&ctx->squelch_update_pending, 0,
                                 memory_order_acquire)) {
        int enabled = atomic_load_explicit(&ctx->requested_squelch_enabled,
                                           memory_order_relaxed);
        float threshold = atomic_load_explicit(&ctx->requested_squelch_dbfs,
                                               memory_order_relaxed);
        if (ctx->kind == DEMOD_KIND_AM) {
            (void)analog_frontend_set_squelch(&ctx->u.am.frontend, enabled,
                                              threshold);
        } else if (ctx->kind == DEMOD_KIND_FM) {
            (void)analog_frontend_set_squelch(&ctx->u.fm.frontend, enabled,
                                              threshold);
        }
    }
    if (atomic_exchange_explicit(&ctx->frequency_offset_update_pending, 0,
                                 memory_order_acquire)) {
        float offset = atomic_load_explicit(
            &ctx->requested_frequency_offset_hz, memory_order_relaxed);
        if (ctx->kind == DEMOD_KIND_AM) {
            (void)analog_frontend_set_frequency_offset(&ctx->u.am.frontend,
                                                       offset);
        } else if (ctx->kind == DEMOD_KIND_FM) {
            (void)analog_frontend_set_frequency_offset(&ctx->u.fm.frontend,
                                                       offset);
        } else if (ctx->kind == DEMOD_KIND_GMSK) {
            (void)gmsk_demod_set_frequency_offset(&ctx->u.gmsk, offset);
        }
    }
    if (atomic_exchange_explicit(&ctx->filter_update_pending, 0,
                                 memory_order_acquire)) {
        analog_filter_type_t filter_type = (analog_filter_type_t)
            atomic_load_explicit(&ctx->requested_filter_type,
                                 memory_order_relaxed);
        int filter_width_hz = atomic_load_explicit(
            &ctx->requested_filter_width_hz, memory_order_relaxed);
        if (ctx->kind == DEMOD_KIND_AM) {
            (void)analog_frontend_set_filter(&ctx->u.am.frontend, filter_type,
                                             filter_width_hz);
        } else if (ctx->kind == DEMOD_KIND_FM) {
            (void)analog_frontend_set_filter(&ctx->u.fm.frontend, filter_type,
                                             filter_width_hz);
        }
    }

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

int demodulator_set_squelch(demodulator_t *ctx, int enabled,
                            float threshold_dbfs) {
    if (!ctx || (ctx->kind != DEMOD_KIND_AM && ctx->kind != DEMOD_KIND_FM)) {
        return 0;
    }
    if (enabled && (!isfinite(threshold_dbfs)
                    || threshold_dbfs < -120.0f || threshold_dbfs > 0.0f)) {
        return 0;
    }
    atomic_store_explicit(&ctx->requested_squelch_enabled, enabled ? 1 : 0,
                          memory_order_relaxed);
    atomic_store_explicit(&ctx->requested_squelch_dbfs, threshold_dbfs,
                          memory_order_relaxed);
    atomic_store_explicit(&ctx->squelch_update_pending, 1,
                          memory_order_release);
    return 1;
}

int demodulator_set_frequency_offset(demodulator_t *ctx, float offset_hz) {
    if (!ctx || ctx->kind == DEMOD_KIND_NONE || ctx->input_fs <= 0
        || !isfinite(offset_hz)
        || fabsf(offset_hz) > 0.5f * (float)ctx->input_fs) {
        return 0;
    }
    if (ctx->kind == DEMOD_KIND_GMSK) {
        for (unsigned int i = 0u; i < ctx->u.gmsk.cfg.channel_count; i++) {
            float combined = ctx->u.gmsk.cfg.channel_offset_hz[i] + offset_hz;
            if (fabsf(combined) > 0.5f * (float)ctx->input_fs) return 0;
        }
    }
    atomic_store_explicit(&ctx->requested_frequency_offset_hz, offset_hz,
                          memory_order_relaxed);
    atomic_store_explicit(&ctx->frequency_offset_update_pending, 1,
                          memory_order_release);
    return 1;
}

int demodulator_set_filter(demodulator_t *ctx,
                           analog_filter_type_t filter_type,
                           int filter_width_hz) {
    if (!ctx || (ctx->kind != DEMOD_KIND_AM && ctx->kind != DEMOD_KIND_FM)
        || ctx->input_fs <= 0
        || filter_type < ANALOG_FILTER_NONE
        || filter_type > ANALOG_FILTER_IIR
        || filter_width_hz < 0 || filter_width_hz >= ctx->input_fs
        || (filter_type == ANALOG_FILTER_NONE && filter_width_hz != 0)
        || (filter_type != ANALOG_FILTER_NONE && filter_width_hz == 0)) {
        return 0;
    }
    atomic_store_explicit(&ctx->requested_filter_type, (int)filter_type,
                          memory_order_relaxed);
    atomic_store_explicit(&ctx->requested_filter_width_hz, filter_width_hz,
                          memory_order_relaxed);
    atomic_store_explicit(&ctx->filter_update_pending, 1,
                          memory_order_release);
    return 1;
}

int demodulator_get_squelch_status(const demodulator_t *ctx,
                                   float *level_dbfs,
                                   float *threshold_dbfs,
                                   int *open) {
    if (!ctx) return 0;
    switch (ctx->kind) {
    case DEMOD_KIND_AM:
        return analog_frontend_get_squelch_status(&ctx->u.am.frontend,
                                                  level_dbfs,
                                                  threshold_dbfs, open);
    case DEMOD_KIND_FM:
        return analog_frontend_get_squelch_status(&ctx->u.fm.frontend,
                                                  level_dbfs,
                                                  threshold_dbfs, open);
    case DEMOD_KIND_GMSK:
    case DEMOD_KIND_NONE:
    default:
        return 0;
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
