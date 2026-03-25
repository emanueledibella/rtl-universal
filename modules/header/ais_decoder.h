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

} ais_ctx_t;

void ais_init(ais_ctx_t *ctx);
void ais_get_demod_config(demod_config_t *cfg);
demod_output_t ais_get_demod_output(ais_ctx_t *ctx);
void ais_process_demod_bit(ais_ctx_t *ctx, uint8_t bit);

// flush/cleanup
void ais_flush(ais_ctx_t *ctx);

// callback chiamata quando decodifichiamo un frame valido (CRC OK)
// puoi sostituirla con la tua logica (stampa, JSON, ecc.)
void ais_on_frame(const uint8_t *frame, size_t len);

// test helper: emette un frame AIS sintetico (bypassa la parte RF/HDLC)
void ais_test_emit_example(void);

#ifdef __cplusplus
}
#endif
