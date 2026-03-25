#pragma once

#include <stdint.h>
#include <liquid/liquid.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*gmsk_demod_output_cb_t)(void *user, uint8_t bit);

typedef struct {
    int input_fs;
    int output_fs;
    unsigned int symbol_rate;
    unsigned int m;
    float bt;
} gmsk_demod_config_t;

typedef struct {
    gmsk_demod_config_t cfg;
    gmsk_demod_output_cb_t out_cb;
    void *out_user;

    unsigned int decim;
    unsigned int decim_count;
    int32_t acc_i;
    int32_t acc_q;

    gmskdem demod;
    unsigned int k;
    unsigned int sym_idx;
    liquid_float_complex *sym_buf;
} gmsk_demod_ctx_t;

int gmsk_demod_init(gmsk_demod_ctx_t *ctx, const gmsk_demod_config_t *cfg,
                    gmsk_demod_output_cb_t out_cb, void *out_user);
void gmsk_demod_process_raw_iq_u8(gmsk_demod_ctx_t *ctx, const unsigned char *buf, uint32_t len);
void gmsk_demod_flush(gmsk_demod_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
