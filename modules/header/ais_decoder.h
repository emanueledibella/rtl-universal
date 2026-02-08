#pragma once
#include <stdint.h>
#include <stddef.h>
#include <liquid/liquid.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // --- GMSK demod ---
    gmskdem demod;
    unsigned int k;       // samples per symbol
    unsigned int m;       // filter delay (symbols)
    float bt;             // Gaussian BT
    unsigned int sym_idx;
    liquid_float_complex* sym_buf;

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

// init con sample rate baseband complesso (es. 96000)
void ais_init(ais_ctx_t *ctx, int fs_demod);

// feed: campioni IQ baseband (float)
void ais_process_sample_iq(ais_ctx_t *ctx, float i, float q);

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
