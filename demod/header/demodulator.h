#pragma once

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
    void (*on_float)(void *user, float sample);
    void *user;
} demod_output_t;

typedef struct {
    demod_kind_t kind;
    int input_fs;
    int output_fs;
    union {
        struct {
            unsigned int symbol_rate;
            unsigned int m;
            float bt;
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
    union {
        am_demod_ctx_t am;
        fm_demod_ctx_t fm;
        gmsk_demod_ctx_t gmsk;
    } u;
} demodulator_t;

int demodulator_init(demodulator_t *ctx, const demod_config_t *config, const demod_output_t *output);
void demodulator_process_raw_iq_u8(demodulator_t *ctx, const unsigned char *buf, uint32_t len);
void demodulator_flush(demodulator_t *ctx);

#ifdef __cplusplus
}
#endif
