#pragma once

#include <stdatomic.h>
#include <stdint.h>

#include "am_demod.h"
#include "fm_demod.h"
#include "gmsk_demod.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DEMOD_KIND_NONE = 0,
    DEMOD_KIND_AM,
    DEMOD_KIND_FM,
    DEMOD_KIND_GMSK,
} demod_kind_t;

typedef struct {
    void (*on_bit)(void *user, uint8_t bit);
    void (*on_channel_bit)(void *user, unsigned int channel, uint8_t bit);
    void (*on_float)(void *user, float sample);
    void *user;
} demod_output_t;

typedef struct {
    demod_kind_t kind;
    int input_fs;
    int output_fs;
    analog_frontend_config_t analog;
    union {
        struct {
            unsigned int symbol_rate;
            unsigned int m;
            float bt;
            unsigned int channel_count;
            float channel_offset_hz[2];
        } gmsk;
        struct {
            float dc_alpha;
        } am;
        struct {
            float dc_alpha;
        } fm;
    } u;
} demod_config_t;

typedef struct {
    demod_kind_t kind;
    int input_fs;
    _Atomic int squelch_update_pending;
    _Atomic int requested_squelch_enabled;
    _Atomic float requested_squelch_dbfs;
    _Atomic int frequency_offset_update_pending;
    _Atomic float requested_frequency_offset_hz;
    _Atomic int filter_update_pending;
    _Atomic int requested_filter_type;
    _Atomic int requested_filter_width_hz;
    union {
        am_demod_ctx_t am;
        fm_demod_ctx_t fm;
        gmsk_demod_ctx_t gmsk;
    } u;
} demodulator_t;

int demodulator_init(demodulator_t *ctx, const demod_config_t *config, const demod_output_t *output);
void demodulator_process_raw_iq_u8(demodulator_t *ctx, const unsigned char *buf, uint32_t len);
int demodulator_get_squelch_status(const demodulator_t *ctx,
                                   float *level_dbfs,
                                   float *threshold_dbfs,
                                   int *open);
int demodulator_set_squelch(demodulator_t *ctx, int enabled,
                            float threshold_dbfs);
int demodulator_set_filter(demodulator_t *ctx,
                           analog_filter_type_t filter_type,
                           int filter_width_hz);
int demodulator_set_frequency_offset(demodulator_t *ctx, float offset_hz);
void demodulator_flush(demodulator_t *ctx);

#ifdef __cplusplus
}
#endif
