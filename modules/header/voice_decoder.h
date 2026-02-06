#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*voice_output_cb_t)(void* user, const float* samples, uint32_t n);

typedef struct {
    // de-emphasis filter
    float deemph_a;
    float deemph_y1;

    // block processing before output
    float tmp[4096];
    uint32_t tmp_len;
    uint32_t flush_threshold;
    float norm_target;

    voice_output_cb_t out_cb;
    void* out_user;
} voice_ctx_t;

// fs_demod: sample rate of demodulated samples (e.g. 48000)
void voice_decoder_init(voice_ctx_t* ctx, int fs_demod, voice_output_cb_t out_cb, void* out_user);
void voice_decoder_process_sample(voice_ctx_t* ctx, float sample);
void voice_decoder_flush(voice_ctx_t* ctx);

#ifdef __cplusplus
}
#endif

