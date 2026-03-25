#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*am_demod_output_cb_t)(void *user, float sample);

typedef struct {
    int input_fs;
    int output_fs;
    float dc_alpha;
} am_demod_config_t;

typedef struct {
    am_demod_config_t cfg;
    am_demod_output_cb_t out_cb;
    void *out_user;
    float dc_estimate;
    int decim;
    int decim_count;
    float decim_acc;
} am_demod_ctx_t;

int am_demod_init(am_demod_ctx_t *ctx, const am_demod_config_t *cfg,
                  am_demod_output_cb_t out_cb, void *out_user);
void am_demod_process_raw_iq_u8(am_demod_ctx_t *ctx, const unsigned char *buf, uint32_t len);
void am_demod_flush(am_demod_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
