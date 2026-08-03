#pragma once

#include <stdint.h>

#include "demodulator.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SSTV_MAX_WIDTH 640u
#define SSTV_MAX_HEIGHT 496u

typedef enum {
    SSTV_MODE_AUTO = 0,
    SSTV_MODE_PD120,
    SSTV_MODE_MARTIN_M1
} sstv_mode_t;

typedef struct {
    sstv_mode_t requested_mode;
    sstv_mode_t active_mode;
    char save_dir[512];
    int sample_rate;

    float previous_sample;
    uint64_t samples_since_crossing;
    double tone_hz;
    double tone_sum;
    unsigned int tone_samples;
    unsigned int tone_block_samples;

    unsigned int vis_state;
    unsigned int vis_tone_blocks;
    unsigned int vis_bit_blocks;
    unsigned int vis_ones;
    unsigned int vis_zeros;
    unsigned int vis_bit_index;
    uint8_t vis_value;
    uint8_t vis_parity;

    uint64_t sync_run_samples;
    int line_active;
    uint64_t line_sample;
    unsigned int scan_line;
    double channel_sum[4][SSTV_MAX_WIDTH];
    unsigned int channel_count[4][SSTV_MAX_WIDTH];
    uint8_t image[SSTV_MAX_WIDTH * SSTV_MAX_HEIGHT * 3u];

    uint64_t audio_samples;
    uint64_t vis_headers;
    uint64_t lines_decoded;
    uint64_t images_saved;
    uint64_t rejected_headers;
} sstv_module_t;

void sstv_module_reset(sstv_module_t *ctx);
int sstv_module_set_mode(sstv_module_t *ctx, const char *name);
int sstv_module_set_save_dir(sstv_module_t *ctx, const char *path);
int sstv_module_init(sstv_module_t *ctx, const demod_config_t *cfg);
void sstv_module_get_demod_config(sstv_module_t *ctx, demod_config_t *cfg);
demod_output_t sstv_module_get_demod_output(sstv_module_t *ctx);
void sstv_module_flush(sstv_module_t *ctx);
int sstv_module_run_test(sstv_module_t *ctx);

#ifdef __cplusplus
}
#endif
