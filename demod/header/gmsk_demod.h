#pragma once

#include <stdint.h>
#include <liquid/liquid.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*gmsk_demod_output_cb_t)(void *user, uint8_t bit);
typedef void (*gmsk_demod_channel_output_cb_t)(void *user,
                                               unsigned int channel,
                                               uint8_t bit);

#define GMSK_DEMOD_MAX_CHANNELS 2u

typedef struct {
    int input_fs;
    int output_fs;
    unsigned int symbol_rate;
    unsigned int m;
    float bt;
    unsigned int channel_count;
    float channel_offset_hz[GMSK_DEMOD_MAX_CHANNELS];
} gmsk_demod_config_t;

typedef struct {
    nco_crcf mixer;
    firdecim_crcf decimator;
    symsync_crcf timing;
    gmskdem demod;
    liquid_float_complex *decim_buf;
    unsigned int decim_count;
    liquid_float_complex sym_buf[2];
    unsigned int sym_idx;
} gmsk_demod_channel_t;

typedef struct {
    gmsk_demod_config_t cfg;
    gmsk_demod_output_cb_t out_cb;
    gmsk_demod_channel_output_cb_t out_channel_cb;
    void *out_user;
    unsigned int decim;
    gmsk_demod_channel_t channel[GMSK_DEMOD_MAX_CHANNELS];
} gmsk_demod_ctx_t;

int gmsk_demod_init(gmsk_demod_ctx_t *ctx, const gmsk_demod_config_t *cfg,
                    gmsk_demod_output_cb_t out_cb,
                    gmsk_demod_channel_output_cb_t out_channel_cb,
                    void *out_user);
void gmsk_demod_process_raw_iq_u8(gmsk_demod_ctx_t *ctx, const unsigned char *buf, uint32_t len);
void gmsk_demod_flush(gmsk_demod_ctx_t *ctx);
int gmsk_demod_self_test(void);

#ifdef __cplusplus
}
#endif
