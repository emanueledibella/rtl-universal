#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*fm_demod_output_cb_t)(void *user, float sample);

typedef struct {
    int input_fs;
    int output_fs;
    float dc_alpha;
} fm_demod_config_t;

typedef struct {
    fm_demod_config_t cfg;
    fm_demod_output_cb_t out_cb;
    void *out_user;
    float prev_i;
    float prev_q;
    int have_prev;
    float dc_estimate;
    int decim;
    int decim_count;
    float decim_acc;
} fm_demod_ctx_t;

int fm_demod_init(fm_demod_ctx_t *ctx, const fm_demod_config_t *cfg,
                  fm_demod_output_cb_t out_cb, void *out_user);
void fm_demod_process_raw_iq_u8(fm_demod_ctx_t *ctx, const unsigned char *buf, uint32_t len);
void fm_demod_flush(fm_demod_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
