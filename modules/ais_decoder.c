#include "header/ais_decoder.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <complex.h>

// ---- CRC-16 (HDLC/PPP style, reflected 0x1021 -> 0x8408) ----
static uint16_t crc16_hdlc(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0x8408;
            else         crc >>= 1;
        }
    }
    return (uint16_t)~crc; // ones complement
}

static void buf_reset(ais_ctx_t *ctx) {
    ctx->buf_len = 0;
    ctx->cur_byte = 0;
    ctx->cur_bitpos = 0;
}

static void buf_push_bit_lsb(ais_ctx_t *ctx, int bit) {
    if (ctx->buf_len >= sizeof(ctx->buf)) return; // drop
    if (bit) ctx->cur_byte |= (uint8_t)(1u << ctx->cur_bitpos);
    ctx->cur_bitpos++;
    if (ctx->cur_bitpos == 8) {
        ctx->buf[ctx->buf_len++] = ctx->cur_byte;
        ctx->cur_byte = 0;
        ctx->cur_bitpos = 0;
    }
}

// Flag HDLC 0x7E = 0b01111110 (LSB-first in stream -> pattern detection via shift reg)
static int is_flag_0x7E(uint8_t last8) {
    return last8 == 0x7E;
}

// ---- AIS payload helpers ----
static uint32_t get_bits(const uint8_t *bytes, int start_bit, int bit_len) {
    // AIS fields sono big-endian bit-order in letteratura, ma qui arrivano spesso LSB-first per byte.
    // Per una prima versione semplice, estraiamo bit-by-bit in ordine "bit stream" (LSB-first).
    // Start_bit: 0 = primo bit del payload (come entra nel decoder).
    uint32_t v = 0;
    for (int i = 0; i < bit_len; i++) {
        int bit_index = start_bit + i;
        int byte_i = bit_index / 8;
        int bit_i  = bit_index % 8;
        int bit = (bytes[byte_i] >> bit_i) & 1;
        v |= (uint32_t)bit << (bit_len - 1 - i); // accumulo MSB-first nel valore
    }
    return v;
}

// Stampa base di alcuni tipi (1/2/3 e 5)
static void ais_parse_and_print(const uint8_t *info, size_t info_len) {
    // info = bytes "Information field" (senza FCS), contenenti il payload AIS (bitstream)
    // In AIS, i primi 6 bit sono message type, poi 2 repeat, poi 30 MMSI, ecc.
    // Qui assumiamo bitstream allineato dall'inizio.
    if (info_len < 8) return;

    uint32_t msg_type = get_bits(info, 0, 6);
    uint32_t mmsi     = get_bits(info, 8, 30); // 6+2=8

    printf("[AIS] type=%u mmsi=%u\n", msg_type, mmsi);

    if (msg_type == 1 || msg_type == 2 || msg_type == 3) {
        // Position report class A (semplificato)
        int32_t lon_raw = (int32_t)get_bits(info, 61, 28);
        int32_t lat_raw = (int32_t)get_bits(info, 89, 27);

        // convert signed two's complement
        if (lon_raw & (1 << 27)) lon_raw |= ~((1 << 28) - 1);
        if (lat_raw & (1 << 26)) lat_raw |= ~((1 << 27) - 1);

        double lon = lon_raw / 600000.0;
        double lat = lat_raw / 600000.0;

        uint32_t sog = get_bits(info, 50, 10); // speed over ground *10
        double sog_kn = sog / 10.0;

        printf("      pos: lat=%.5f lon=%.5f sog=%.1f kn\n", lat, lon, sog_kn);
    } else if (msg_type == 5) {
        // Static and voyage related data (qui stampiamo solo MMSI già sopra)
        printf("      (type 5 static/voyage data)\n");
    }
}

static void set_bits_stream(uint8_t *bytes, int start_bit, int bit_len, uint32_t value) {
    // write MSB-first value into LSB-first bitstream
    for (int i = 0; i < bit_len; i++) {
        int bit_index = start_bit + i;
        int byte_i = bit_index / 8;
        int bit_i  = bit_index % 8;
        int bit = (value >> (bit_len - 1 - i)) & 1;
        if (bit) {
            bytes[byte_i] |= (uint8_t)(1u << bit_i);
        } else {
            bytes[byte_i] &= (uint8_t)~(1u << bit_i);
        }
    }
}

static uint32_t encode_signed(int32_t v, int bits) {
    if (v >= 0) return (uint32_t)v;
    return (uint32_t)((1u << bits) + v);
}

// ---- Frame handling ----
static void hdlc_end_frame(ais_ctx_t *ctx) {
    // flush last partial byte? (in HDLC normalmente byte allineati, ma per sicurezza)
    // se cur_bitpos != 0, potresti decidere di ignorare; qui ignoriamo il partial.

    if (ctx->buf_len < 3) { // almeno 1 byte info + 2 byte FCS
        buf_reset(ctx);
        return;
    }

    // ultimi 2 byte sono FCS (LSB-first)
    size_t info_len = ctx->buf_len - 2;
    const uint8_t *info = ctx->buf;

    uint16_t rx_fcs = (uint16_t)(ctx->buf[info_len] | ((uint16_t)ctx->buf[info_len + 1] << 8));
    uint16_t calc   = crc16_hdlc(info, info_len);

    if (calc == rx_fcs) {
        // frame valido
        ais_on_frame(info, info_len);
        // parse base (opzionale)
        ais_parse_and_print(info, info_len);
    }

    buf_reset(ctx);
}

static void hdlc_push_bit(ais_ctx_t *ctx, int bit) {
    // AIS usa NRZI: raw level 0/1 dal demod (slicer). Prima facciamo NRZI decode,
    // poi HDLC (flag detection + destuffing) sul bitstream decodificato.
    int raw = bit & 1;
    if (!ctx->have_last_nrzi) {
        ctx->last_nrzi = raw;
        ctx->have_last_nrzi = 1;
        return;
    }

    int decoded = (raw == ctx->last_nrzi) ? 1 : 0;
    ctx->last_nrzi = raw;

    // shift register per flag detection (LSB-first)
    ctx->shift_reg = ((ctx->shift_reg << 1) | (uint32_t)(decoded & 1)) & 0xFF;

    if (is_flag_0x7E((uint8_t)ctx->shift_reg)) {
        if (ctx->in_frame) {
            // fine frame
            hdlc_end_frame(ctx);
        }
        // start new frame
        ctx->in_frame = 1;
        ctx->ones_count = 0;
        ctx->skip_next_zero = 0;
        buf_reset(ctx);
        return;
    }

    if (!ctx->in_frame) return;

    // --- bit destuffing (HDLC): dopo 5 '1' consecutivi, lo zero successivo è stuffing ---
    if (ctx->skip_next_zero) {
        ctx->skip_next_zero = 0;
        if (decoded == 0) return; // stuffed, discard
        // se non è zero, frame corrotto: reset
        ctx->in_frame = 0;
        buf_reset(ctx);
        return;
    }

    if (decoded == 1) {
        ctx->ones_count++;
        if (ctx->ones_count == 5) {
            ctx->skip_next_zero = 1;
        }
    } else {
        ctx->ones_count = 0;
    }

    // aggiungi bit al buffer
    buf_push_bit_lsb(ctx, decoded);
}

void ais_init(ais_ctx_t *ctx, int fs_demod) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->k = (unsigned int)(fs_demod / 9600);
    if (ctx->k < 2) ctx->k = 2;
    if ((fs_demod % 9600) != 0) {
        fprintf(stderr, "[AIS] warning: fs_demod=%d not multiple of 9600, k=%u\n",
                fs_demod, ctx->k);
    }

    ctx->m = 3;
    ctx->bt = 0.4f;
    ctx->demod = gmskdem_create(ctx->k, ctx->m, ctx->bt);
    if (!ctx->demod) {
        fprintf(stderr, "[AIS] gmskdem_create failed\n");
        return;
    }

    ctx->sym_buf = (liquid_float_complex*)calloc(ctx->k, sizeof(liquid_float_complex));
    if (!ctx->sym_buf) {
        fprintf(stderr, "[AIS] sym_buf alloc failed\n");
        gmskdem_destroy(ctx->demod);
        ctx->demod = NULL;
        return;
    }
    ctx->sym_idx = 0;
}

void ais_process_sample_iq(ais_ctx_t *ctx, float i, float q) {
    if (!ctx || !ctx->demod || !ctx->sym_buf) return;
    ctx->sym_buf[ctx->sym_idx++] = i + _Complex_I * q;
    if (ctx->sym_idx >= ctx->k) {
        unsigned int sym = 0;
        gmskdem_demodulate(ctx->demod, ctx->sym_buf, &sym);
        hdlc_push_bit(ctx, (int)(sym & 1u));
        ctx->sym_idx = 0;
    }
}

void ais_flush(ais_ctx_t *ctx) {
    if (!ctx) return;
    if (ctx->demod) {
        gmskdem_destroy(ctx->demod);
        ctx->demod = NULL;
    }
    if (ctx->sym_buf) {
        free(ctx->sym_buf);
        ctx->sym_buf = NULL;
    }
    ctx->sym_idx = 0;
}

// Default callback: stampa solo lunghezza frame.
// Tu la puoi sostituire in un altro file, oppure cambiare questa.
void ais_on_frame(const uint8_t *frame, size_t len) {
    printf("[AIS] frame len=%zu bytes\n", len);
}

void ais_test_emit_example(void) {
    // Example AIS type 1 (position report) payload, minimal fields set.
    uint8_t info[21];
    memset(info, 0, sizeof(info));

    // type=1, repeat=0, mmsi=247320162
    set_bits_stream(info, 0, 6, 1);
    set_bits_stream(info, 6, 2, 0);
    set_bits_stream(info, 8, 30, 247320162u);

    // SOG = 12.0 kn (value *10)
    set_bits_stream(info, 50, 10, 120u);

    // Lon/Lat (1/10000 minute = 1/600000 degree), two's complement
    double lon = -122.4194;
    double lat = 37.7749;
    int32_t lon_raw = (int32_t)lrint(lon * 600000.0);
    int32_t lat_raw = (int32_t)lrint(lat * 600000.0);
    set_bits_stream(info, 61, 28, encode_signed(lon_raw, 28));
    set_bits_stream(info, 89, 27, encode_signed(lat_raw, 27));

    printf("[AIS] test frame (synthetic)\n");
    ais_on_frame(info, sizeof(info));
    ais_parse_and_print(info, sizeof(info));
}
