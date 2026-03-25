#pragma once

#include <stdint.h>
#include <stddef.h>

#include "demodulator.h"

#ifdef __cplusplus
extern "C" {
#endif

#define adsb_MAG_HISTORY_LEN 512

typedef void (*adsb_on_bit_cb_t)(void *user, uint8_t bit, uint64_t bit_index);
typedef void (*adsb_on_frame_cb_t)(void *user, const uint8_t *frame, size_t frame_bits);

typedef struct {
    int fs_demod;
    uint64_t demod_sample_count;
    float ema_abs;
    float threshold;
    uint32_t pulse_count;
    uint64_t detected_preamble_count;
    uint64_t rejected_frame_count;
    float mag_history[adsb_MAG_HISTORY_LEN];
    uint64_t capture_start_sample;
    uint64_t next_search_sample;
    int capture_active;

    // Clean-bit path (for protocol work independent of RF demod)
    int frame_bits_target;        // usually 112 (long) or 56 (short)
    uint8_t frame_bits_buf[112];  // one frame max (long Mode S)
    int frame_bits_len;
    uint64_t clean_bit_count;
    uint64_t clean_frame_count;

    adsb_on_bit_cb_t on_bit_cb;
    void *on_bit_user;
    adsb_on_frame_cb_t on_frame_cb;
    void *on_frame_user;
} adsb_ctx_t;

void adsb_init(adsb_ctx_t *ctx, int fs_demod);
void adsb_get_demod_config(demod_config_t *cfg);
demod_output_t adsb_get_demod_output(adsb_ctx_t *ctx);
void adsb_process_am_sample(adsb_ctx_t *ctx, float sample);

// Configure callbacks for clean-bit pipeline.
void adsb_set_callbacks(adsb_ctx_t *ctx,
                        adsb_on_bit_cb_t on_bit_cb, void *on_bit_user,
                        adsb_on_frame_cb_t on_frame_cb, void *on_frame_user);

// Set expected frame length for clean-bit assembly (typically 112 or 56).
// Values outside 1..112 are ignored.
void adsb_set_frame_bits_target(adsb_ctx_t *ctx, int frame_bits_target);

// Feed one already-demodulated bit (0/1). When frame_bits_target bits are collected,
// on_frame_cb is called with packed frame bytes.
void adsb_feed_clean_bit(adsb_ctx_t *ctx, uint8_t bit);

// Feed a vector of already-demodulated bits (each element 0/1).
void adsb_feed_clean_bits(adsb_ctx_t *ctx, const uint8_t *bits, size_t nbits);

// Default conversion hook called for each assembled clean frame.
// Edit/replace this logic to map bits to your protocol structures.
void adsb_on_clean_frame(const uint8_t *frame, size_t frame_bits);

// cleanup/final stats
void adsb_flush(adsb_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
