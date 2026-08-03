#pragma once

#include <stddef.h>
#include <stdint.h>

#include "demodulator.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SONDE_RS41_FRAME_LEN 518u

typedef struct {
    uint64_t sync_window;
    uint64_t sync_pattern;
    uint8_t raw_frame[SONDE_RS41_FRAME_LEN];
    size_t raw_index;
    unsigned int raw_bit;
    int capturing;
    int inverted;

    uint64_t demod_bits;
    uint64_t frame_candidates;
    uint64_t frames_valid;
    uint64_t crc_errors;
    uint64_t rejected_frames;
    uint64_t fec_corrected_symbols;
    uint64_t fec_failures;
} sonde_module_t;

void sonde_module_reset(sonde_module_t *ctx);
int sonde_module_init(sonde_module_t *ctx, const demod_config_t *cfg);
void sonde_module_get_demod_config(sonde_module_t *ctx, demod_config_t *cfg);
demod_output_t sonde_module_get_demod_output(sonde_module_t *ctx);
void sonde_module_flush(sonde_module_t *ctx);
int sonde_module_run_test(sonde_module_t *ctx);

/* Decode an already de-whitened RS41 frame represented as hexadecimal. */
int sonde_module_decode_hex(sonde_module_t *ctx, const char *hex);

#ifdef __cplusplus
}
#endif
