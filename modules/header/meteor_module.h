#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char executable[512];
    char pipeline[64];
    char output_dir[512];
    char satellite[32];
    unsigned int sample_rate;
    unsigned int timeout_seconds;
} meteor_module_t;

void meteor_module_reset(meteor_module_t *ctx);
int meteor_module_set_executable(meteor_module_t *ctx, const char *path);
int meteor_module_set_pipeline(meteor_module_t *ctx, const char *name);
int meteor_module_set_output_dir(meteor_module_t *ctx, const char *path);
int meteor_module_set_satellite(meteor_module_t *ctx, const char *name);
void meteor_module_set_timeout(meteor_module_t *ctx, unsigned int seconds);
int meteor_module_run_live(meteor_module_t *ctx, double frequency_mhz,
                           const char *device_id, int manual_gain, int gain_db,
                           int have_ppm, int ppm);
int meteor_module_run_offline(meteor_module_t *ctx, const char *input_path);
int meteor_module_run_test(meteor_module_t *ctx);
void meteor_module_stop(void);

#ifdef __cplusplus
}
#endif
