#pragma once

#include <pthread.h>
#include <stdint.h>

#include "demodulator.h"
#include "voice_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VOICE_DEMOD_KIND_FM = 0,
    VOICE_DEMOD_KIND_AM,
} voice_demod_kind_t;

#define voice_MODULE_OUTPUT_FS 48000
#define voice_MODULE_INPUT_FS 240000
#define voice_MODULE_RING_SECONDS 4
#define voice_MODULE_RING_SIZE (voice_MODULE_OUTPUT_FS * voice_MODULE_RING_SECONDS)

typedef struct {
    voice_demod_kind_t demod_kind;
    voice_ctx_t decoder;
    float ring[voice_MODULE_RING_SIZE];
    uint32_t ring_w;
    uint32_t ring_r;
    pthread_mutex_t ring_lock;
    int ring_lock_ready;
    void *stream;
    int pa_initialized;
    int audio_fs;
} voice_module_t;

void voice_module_reset(voice_module_t *ctx);
int voice_module_set_demod(voice_module_t *ctx, const char *name);
const char *voice_module_demod_name(const voice_module_t *ctx);
void voice_module_get_demod_config(voice_module_t *ctx, demod_config_t *cfg);
int voice_module_init(voice_module_t *ctx, const demod_config_t *cfg);
demod_output_t voice_module_get_demod_output(voice_module_t *ctx);
void voice_module_flush(voice_module_t *ctx);

#ifdef __cplusplus
}
#endif
