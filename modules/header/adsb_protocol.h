#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reset the in-memory aircraft table. */
void adsb_protocol_reset(void);

/*
 * Set the receiver position used to resolve a single CPR position message.
 * This is required for surface positions and is also used as a fallback while
 * waiting for an airborne even/odd pair. Returns zero for invalid coordinates.
 */
int adsb_protocol_set_reference(double latitude, double longitude);

/* Return the Mode S polynomial remainder. A valid DF17/DF18 frame returns 0. */
uint32_t adsb_frame_crc(const uint8_t *frame, size_t frame_bits);

/* Validate, decode and print one packed (MSB-first) 56- or 112-bit frame. */
int adsb_protocol_handle_frame(const uint8_t *frame, size_t frame_bits);

/* Hexadecimal convenience wrapper (14 or 28 hex digits). */
int adsb_protocol_handle_frame_hex(const char *hex);

/* Decode one AVR text record: *HEX; or @TIMESTAMPHEX;. */
int adsb_protocol_handle_avr_line(const char *line);

typedef struct {
    uint8_t payload[32];
    size_t payload_len;
    size_t expected_len;
    uint64_t frames_valid;
    uint64_t frames_rejected;
    uint8_t type;
    int after_escape;
    int in_frame;
} adsb_beast_parser_t;

/* Incremental Mode-S Beast input parser (type 1 skipped, types 2/3 decoded). */
void adsb_beast_parser_init(adsb_beast_parser_t *parser);
void adsb_beast_parser_feed(adsb_beast_parser_t *parser,
                            const uint8_t *data, size_t len);

/* Decode an already separated extended-squitter payload. */
void protocol_handle_message(uint8_t df, uint8_t ca, uint32_t icao,
                             uint64_t me, uint32_t pi);

/* Offline deterministic examples used by --adsb-test. */
int adsb_protocol_emit_test_examples(void);

#ifdef __cplusplus
}
#endif
