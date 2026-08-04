#include "header/analog_frontend.h"

#include <complex.h>
#include <math.h>
#include <string.h>

#define ANALOG_FIR_LENGTH 129u
#define ANALOG_IIR_ORDER 6u
#define ANALOG_SQUELCH_SMOOTHING_SECONDS 0.010f
#define ANALOG_IQ_FULL_SCALE_POWER (2.0f * 127.5f * 127.5f)
#define ANALOG_TWO_PI_F 6.28318530717958647692f

int analog_frontend_init(analog_frontend_t *ctx,
                         const analog_frontend_config_t *cfg,
                         int sample_rate) {
    float cutoff;

    if (!ctx || !cfg || sample_rate <= 0) return 0;
    if (cfg->filter_type < ANALOG_FILTER_NONE
        || cfg->filter_type > ANALOG_FILTER_IIR) return 0;
    if (cfg->filter_width_hz < 0 || cfg->filter_width_hz >= sample_rate) return 0;
    if (cfg->filter_type == ANALOG_FILTER_NONE && cfg->filter_width_hz != 0) return 0;
    if (cfg->filter_type != ANALOG_FILTER_NONE && cfg->filter_width_hz == 0) return 0;
    if (cfg->squelch_enabled
        && (!isfinite(cfg->squelch_dbfs)
            || cfg->squelch_dbfs < -120.0f || cfg->squelch_dbfs > 0.0f)) return 0;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->sample_rate = sample_rate;
    ctx->mixer = nco_crcf_create(LIQUID_NCO);
    if (!ctx->mixer) return 0;
    cutoff = (float)cfg->filter_width_hz / (2.0f * (float)sample_rate);

    if (cfg->filter_type == ANALOG_FILTER_FIR) {
        const float *coefficients;
        unsigned int coefficient_count;
        float coefficient_sum = 0.0f;

        ctx->fir = firfilt_crcf_create_kaiser(ANALOG_FIR_LENGTH, cutoff,
                                              60.0f, 0.0f);
        if (!ctx->fir) {
            nco_crcf_destroy(ctx->mixer);
            ctx->mixer = NULL;
            return 0;
        }
        coefficients = firfilt_crcf_get_coefficients(ctx->fir);
        coefficient_count = firfilt_crcf_get_length(ctx->fir);
        for (unsigned int i = 0; i < coefficient_count; i++) {
            coefficient_sum += coefficients[i];
        }
        if (fabsf(coefficient_sum) < 1e-6f
            || firfilt_crcf_set_scale(ctx->fir, 1.0f / coefficient_sum)
                   != LIQUID_OK) {
            (void)firfilt_crcf_destroy(ctx->fir);
            ctx->fir = NULL;
            nco_crcf_destroy(ctx->mixer);
            ctx->mixer = NULL;
            return 0;
        }
    } else if (cfg->filter_type == ANALOG_FILTER_IIR) {
        ctx->iir = iirfilt_crcf_create_lowpass(ANALOG_IIR_ORDER, cutoff);
        if (!ctx->iir) {
            nco_crcf_destroy(ctx->mixer);
            ctx->mixer = NULL;
            return 0;
        }
    }

    ctx->power_alpha = 1.0f
                       - expf(-1.0f / ((float)sample_rate
                                       * ANALOG_SQUELCH_SMOOTHING_SECONDS));
    if (cfg->squelch_enabled) {
        ctx->squelch_threshold = powf(10.0f, cfg->squelch_dbfs / 10.0f);
    } else {
        ctx->squelch_open = 1;
    }
    return 1;
}

int analog_frontend_process(analog_frontend_t *ctx, float i, float q,
                            float *filtered_i, float *filtered_q) {
    liquid_float_complex input;
    liquid_float_complex output;

    if (!ctx || !filtered_i || !filtered_q) return 0;

    input = i + _Complex_I * q;
    output = input;
    if (ctx->mixer && ctx->frequency_offset_hz != 0.0f) {
        (void)nco_crcf_mix_down(ctx->mixer, input, &output);
        (void)nco_crcf_step(ctx->mixer);
    }
    if (ctx->fir) {
        (void)firfilt_crcf_execute_one(ctx->fir, output, &output);
    } else if (ctx->iir) {
        (void)iirfilt_crcf_execute(ctx->iir, output, &output);
    }
    *filtered_i = crealf(output);
    *filtered_q = cimagf(output);

    {
        float power = (*filtered_i * *filtered_i + *filtered_q * *filtered_q)
                      / ANALOG_IQ_FULL_SCALE_POWER;
        ctx->power_estimate += ctx->power_alpha * (power - ctx->power_estimate);
    }
    if (ctx->cfg.squelch_enabled) {
        ctx->squelch_open = ctx->power_estimate >= ctx->squelch_threshold;
    } else {
        ctx->squelch_open = 1;
    }
    return ctx->squelch_open;
}

int analog_frontend_set_squelch(analog_frontend_t *ctx, int enabled,
                                float threshold_dbfs) {
    if (!ctx || (enabled && (!isfinite(threshold_dbfs)
                             || threshold_dbfs < -120.0f
                             || threshold_dbfs > 0.0f))) {
        return 0;
    }
    ctx->cfg.squelch_enabled = enabled ? 1 : 0;
    if (enabled) {
        ctx->cfg.squelch_dbfs = threshold_dbfs;
        ctx->squelch_threshold = powf(10.0f, threshold_dbfs / 10.0f);
        ctx->squelch_open = ctx->power_estimate >= ctx->squelch_threshold;
    } else {
        ctx->squelch_open = 1;
    }
    return 1;
}

int analog_frontend_set_filter(analog_frontend_t *ctx,
                               analog_filter_type_t filter_type,
                               int filter_width_hz) {
    firfilt_crcf next_fir = NULL;
    iirfilt_crcf next_iir = NULL;
    float cutoff;

    if (!ctx || ctx->sample_rate <= 0
        || filter_type < ANALOG_FILTER_NONE
        || filter_type > ANALOG_FILTER_IIR
        || filter_width_hz < 0 || filter_width_hz >= ctx->sample_rate
        || (filter_type == ANALOG_FILTER_NONE && filter_width_hz != 0)
        || (filter_type != ANALOG_FILTER_NONE && filter_width_hz == 0)) {
        return 0;
    }

    cutoff = (float)filter_width_hz / (2.0f * (float)ctx->sample_rate);
    if (filter_type == ANALOG_FILTER_FIR) {
        const float *coefficients;
        unsigned int coefficient_count;
        float coefficient_sum = 0.0f;

        next_fir = firfilt_crcf_create_kaiser(ANALOG_FIR_LENGTH, cutoff,
                                               60.0f, 0.0f);
        if (!next_fir) return 0;
        coefficients = firfilt_crcf_get_coefficients(next_fir);
        coefficient_count = firfilt_crcf_get_length(next_fir);
        for (unsigned int i = 0; i < coefficient_count; i++) {
            coefficient_sum += coefficients[i];
        }
        if (fabsf(coefficient_sum) < 1e-6f
            || firfilt_crcf_set_scale(next_fir, 1.0f / coefficient_sum)
                   != LIQUID_OK) {
            (void)firfilt_crcf_destroy(next_fir);
            return 0;
        }
    } else if (filter_type == ANALOG_FILTER_IIR) {
        next_iir = iirfilt_crcf_create_lowpass(ANALOG_IIR_ORDER, cutoff);
        if (!next_iir) return 0;
    }

    if (ctx->fir) (void)firfilt_crcf_destroy(ctx->fir);
    if (ctx->iir) (void)iirfilt_crcf_destroy(ctx->iir);
    ctx->fir = next_fir;
    ctx->iir = next_iir;
    ctx->cfg.filter_type = filter_type;
    ctx->cfg.filter_width_hz = filter_width_hz;
    return 1;
}

int analog_frontend_set_frequency_offset(analog_frontend_t *ctx,
                                         float offset_hz) {
    if (!ctx || !ctx->mixer || ctx->sample_rate <= 0 || !isfinite(offset_hz)
        || fabsf(offset_hz) > 0.5f * (float)ctx->sample_rate) {
        return 0;
    }
    ctx->frequency_offset_hz = offset_hz;
    (void)nco_crcf_set_frequency(
        ctx->mixer, ANALOG_TWO_PI_F * offset_hz / (float)ctx->sample_rate);
    (void)nco_crcf_set_phase(ctx->mixer, 0.0f);
    return 1;
}

int analog_frontend_get_squelch_status(const analog_frontend_t *ctx,
                                       float *level_dbfs,
                                       float *threshold_dbfs,
                                       int *open) {
    if (!ctx || !ctx->cfg.squelch_enabled) return 0;
    if (level_dbfs) {
        *level_dbfs = ctx->power_estimate > 0.0f
                          ? 10.0f * log10f(ctx->power_estimate)
                          : -INFINITY;
    }
    if (threshold_dbfs) *threshold_dbfs = ctx->cfg.squelch_dbfs;
    if (open) *open = ctx->squelch_open;
    return 1;
}

void analog_frontend_destroy(analog_frontend_t *ctx) {
    if (!ctx) return;
    if (ctx->mixer) (void)nco_crcf_destroy(ctx->mixer);
    if (ctx->fir) (void)firfilt_crcf_destroy(ctx->fir);
    if (ctx->iir) (void)iirfilt_crcf_destroy(ctx->iir);
    memset(ctx, 0, sizeof(*ctx));
}
