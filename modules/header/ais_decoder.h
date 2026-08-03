#pragma once
#include <stdint.h>
#include <stddef.h>

#include "demodulator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // --- HDLC/AIS state ---
    uint32_t shift_reg;   // per cercare 0x7E
    int in_frame;

    // destuff
    int ones_count;
    int skip_next_zero;

    // NRZI decode
    int last_nrzi;
    int have_last_nrzi;

    // frame buffer (byte-wise, LSB-first bits)
    uint8_t buf[1024];
    size_t  buf_len;
    uint8_t cur_byte;
    int     cur_bitpos;   // 0..7

    uint64_t demod_bits;
    uint64_t frames_seen;
    uint64_t frames_valid;
    uint64_t crc_errors;
    uint64_t aborted_frames;
    uint64_t overflow_frames;
    int overflow;
    char channel_name;

} ais_ctx_t;

typedef struct {
    ais_ctx_t channel[2];
    unsigned int channel_count;
    char selected_channel; /* '*' for both, otherwise 'A' or 'B'. */
} ais_receiver_t;

void ais_init(ais_ctx_t *ctx);
void ais_get_demod_config(demod_config_t *cfg);
demod_output_t ais_get_demod_output(ais_ctx_t *ctx);
void ais_process_demod_bit(ais_ctx_t *ctx, uint8_t bit);

// flush/cleanup
void ais_flush(ais_ctx_t *ctx);

// Callback invoked for a valid frame (CRC OK).
void ais_on_frame(const uint8_t *frame, size_t len);

// Decode and print one AIS information payload (without HDLC flags/FCS).
// Returns 1 when the payload contains a supported AIS message type.
int ais_decode_payload(const uint8_t *payload, size_t len);

// Decode a hexadecimal AIS information payload supplied at startup.
int ais_decode_payload_hex(const char *hex);

// Validate and decode one !AIVDM/!AIVDO sentence. Returns 1 when decoded,
// 2 when a multipart message is valid but incomplete, or 0 on error.
int ais_decode_nmea_sentence(const char *sentence);
int ais_nmea_has_pending_fragments(void);

/* Dual-channel AIS receiver helpers (AIS 1 and AIS 2 from one IQ stream). */
int ais_receiver_set_channels(ais_receiver_t *receiver, const char *selection);
void ais_receiver_init(ais_receiver_t *receiver);
void ais_receiver_get_demod_config(ais_receiver_t *receiver, demod_config_t *cfg);
demod_output_t ais_receiver_get_demod_output(ais_receiver_t *receiver);
void ais_receiver_flush(ais_receiver_t *receiver);
int ais_receiver_test(ais_receiver_t *receiver);

// Test helper: feeds synthetic frames through NRZI, HDLC, bit de-stuffing and CRC.
int ais_test_emit_examples(ais_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
