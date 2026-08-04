#pragma once

#include <liquid/liquid.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ANALOG_FILTER_NONE = 0,
    ANALOG_FILTER_FIR,
    ANALOG_FILTER_IIR,
} analog_filter_type_t;

typedef struct {
    analog_filter_type_t filter_type;
    int filter_width_hz;
    int squelch_enabled;
    float squelch_dbfs;
} analog_frontend_config_t;

typedef struct {
    analog_frontend_config_t cfg;
    firfilt_crcf fir;
    iirfilt_crcf iir;
    float power_estimate;
    float power_alpha;
    float squelch_threshold;
    int squelch_open;
} analog_frontend_t;

int analog_frontend_init(analog_frontend_t *ctx,
                         const analog_frontend_config_t *cfg,
                         int sample_rate);
int analog_frontend_process(analog_frontend_t *ctx, float i, float q,
                            float *filtered_i, float *filtered_q);
int analog_frontend_set_squelch(analog_frontend_t *ctx, int enabled,
                                float threshold_dbfs);
int analog_frontend_get_squelch_status(const analog_frontend_t *ctx,
                                       float *level_dbfs,
                                       float *threshold_dbfs,
                                       int *open);
void analog_frontend_destroy(analog_frontend_t *ctx);

#ifdef __cplusplus
}
#endif
