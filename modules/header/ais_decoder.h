#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // --- bit slicer (48k -> 9600) ---
    int samp_per_bit;     // 5 se fs=48000
    int phase;            // 0..samp_per_bit-1
    float lp;             // piccolo filtro IIR
    float lp_alpha;       // 0..1

    // --- HDLC/AIS state ---
    uint32_t shift_reg;   // per cercare 0x7E
    int in_frame;

    // destuff
    int ones_count;
    int skip_next_zero;

    // NRZI decode
    int last_nrzi;

    // frame buffer (byte-wise, LSB-first bits)
    uint8_t buf[1024];
    size_t  buf_len;
    uint8_t cur_byte;
    int     cur_bitpos;   // 0..7

} ais_ctx_t;

// init con sample rate output del demod (es. 48000)
void ais_init(ais_ctx_t *ctx, int fs_demod);

// feed: campioni demod (float, +/-)
void ais_feed_samples(ais_ctx_t *ctx, const float *samples, size_t n);

// callback chiamata quando decodifichiamo un frame valido (CRC OK)
// puoi sostituirla con la tua logica (stampa, JSON, ecc.)
void ais_on_frame(const uint8_t *frame, size_t len);

// test helper: emette un frame AIS sintetico (bypassa la parte RF/HDLC)
void ais_test_emit_example(void);

#ifdef __cplusplus
}
#endif
