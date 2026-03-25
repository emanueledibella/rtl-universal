#include "header/ais_decoder.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>

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

static void buf_drop_tail_bits(ais_ctx_t *ctx, size_t nbits) {
    size_t total_bits = (ctx->buf_len * 8u) + (size_t)ctx->cur_bitpos;
    if (nbits >= total_bits) {
        buf_reset(ctx);
        return;
    }

    size_t new_total = total_bits - nbits;
    size_t new_buf_len = new_total / 8u;
    int new_cur_bitpos = (int)(new_total % 8u);

    if (new_cur_bitpos == 0) {
        ctx->cur_byte = 0;
        ctx->cur_bitpos = 0;
    } else {
        uint8_t mask = (uint8_t)((1u << new_cur_bitpos) - 1u);
        if (new_buf_len < ctx->buf_len) {
            ctx->cur_byte = (uint8_t)(ctx->buf[new_buf_len] & mask);
        } else {
            ctx->cur_byte = (uint8_t)(ctx->cur_byte & mask);
        }
        ctx->cur_bitpos = new_cur_bitpos;
    }

    ctx->buf_len = new_buf_len;
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

static int32_t get_sbits(const uint8_t *bytes, int start_bit, int bit_len) {
    uint32_t v = get_bits(bytes, start_bit, bit_len);
    if (bit_len <= 0 || bit_len >= 32) return (int32_t)v;
    uint32_t sign = 1u << (bit_len - 1);
    if (v & sign) {
        v |= ~((1u << bit_len) - 1u);
    }
    return (int32_t)v;
}

static char ais_sixbit_to_char(uint8_t v) {
    v &= 0x3F;
    if (v < 32) {
        static const char t0[32] = "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_";
        return t0[v];
    }
    static const char t1[32] = " !\"#$%&'()*+,-./0123456789:;<=>?";
    return t1[v - 32];
}

static void ais_get_text(const uint8_t *info, int start_bit, int n_chars, char *out, size_t out_sz) {
    if (!out || out_sz == 0) return;
    int n = n_chars;
    if (n < 0) n = 0;
    if ((size_t)n + 1 > out_sz) n = (int)out_sz - 1;

    for (int i = 0; i < n; i++) {
        uint8_t c6 = (uint8_t)get_bits(info, start_bit + i * 6, 6);
        out[i] = ais_sixbit_to_char(c6);
    }
    out[n] = '\0';

    // trim trailing spaces and '@' padding
    for (int i = n - 1; i >= 0; i--) {
        if (out[i] == ' ' || out[i] == '@') out[i] = '\0';
        else break;
    }

    // replace non-printable with '.'
    for (int i = 0; out[i] != '\0'; i++) {
        if (!isprint((unsigned char)out[i])) out[i] = '.';
    }
}

static int ais_has_bits(size_t info_len, int bits_required) {
    return (int)(info_len * 8u) >= bits_required;
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
        if (!ais_has_bits(info_len, 168)) {
            printf("      warning: payload too short for type %u\n", msg_type);
            return;
        }

        // Position report class A
        uint32_t nav_status = get_bits(info, 38, 4);
        uint32_t sog = get_bits(info, 50, 10); // speed over ground *10
        int32_t lon_raw = get_sbits(info, 61, 28);
        int32_t lat_raw = get_sbits(info, 89, 27);
        uint32_t cog = get_bits(info, 116, 12);      // *10 deg
        uint32_t hdg = get_bits(info, 128, 9);       // deg
        uint32_t ts  = get_bits(info, 137, 6);       // sec

        double lon = lon_raw / 600000.0;
        double lat = lat_raw / 600000.0;
        double sog_kn = sog / 10.0;
        double cog_deg = cog / 10.0;

        printf("      classA: nav=%u lat=%.5f lon=%.5f sog=%.1f kn cog=%.1f hdg=%u ts=%u\n",
               nav_status, lat, lon, sog_kn, cog_deg, hdg, ts);
    } else if (msg_type == 5) {
        if (!ais_has_bits(info_len, 424)) {
            printf("      warning: payload too short for type 5\n");
            return;
        }

        uint32_t imo = get_bits(info, 40, 30);
        uint32_t ship_type = get_bits(info, 232, 8);
        uint32_t to_bow = get_bits(info, 240, 9);
        uint32_t to_stern = get_bits(info, 249, 9);
        uint32_t to_port = get_bits(info, 258, 6);
        uint32_t to_starboard = get_bits(info, 264, 6);
        uint32_t eta_month = get_bits(info, 274, 4);
        uint32_t eta_day = get_bits(info, 278, 5);
        uint32_t eta_hour = get_bits(info, 283, 5);
        uint32_t eta_min = get_bits(info, 288, 6);
        uint32_t draught_dm = get_bits(info, 294, 8); // 0.1m

        char callsign[8];
        char name[21];
        char dest[21];
        ais_get_text(info, 70, 7, callsign, sizeof(callsign));
        ais_get_text(info, 112, 20, name, sizeof(name));
        ais_get_text(info, 302, 20, dest, sizeof(dest));

        printf("      type5: imo=%u callsign=%s name=%s ship_type=%u dim=%u/%u/%u/%u eta=%02u-%02u %02u:%02u draught=%.1fm dest=%s\n",
               imo, callsign, name, ship_type, to_bow, to_stern, to_port, to_starboard,
               eta_month, eta_day, eta_hour, eta_min, draught_dm / 10.0, dest);
    } else if (msg_type == 18) {
        if (!ais_has_bits(info_len, 168)) {
            printf("      warning: payload too short for type 18\n");
            return;
        }

        uint32_t sog = get_bits(info, 46, 10);
        int32_t lon_raw = get_sbits(info, 57, 28);
        int32_t lat_raw = get_sbits(info, 85, 27);
        uint32_t cog = get_bits(info, 112, 12);
        uint32_t hdg = get_bits(info, 124, 9);
        uint32_t ts  = get_bits(info, 133, 6);

        printf("      classB: lat=%.5f lon=%.5f sog=%.1f kn cog=%.1f hdg=%u ts=%u\n",
               lat_raw / 600000.0, lon_raw / 600000.0, sog / 10.0, cog / 10.0, hdg, ts);
    } else if (msg_type == 19) {
        if (!ais_has_bits(info_len, 312)) {
            printf("      warning: payload too short for type 19\n");
            return;
        }

        uint32_t sog = get_bits(info, 46, 10);
        int32_t lon_raw = get_sbits(info, 57, 28);
        int32_t lat_raw = get_sbits(info, 85, 27);
        uint32_t cog = get_bits(info, 112, 12);
        uint32_t hdg = get_bits(info, 124, 9);
        uint32_t ship_type = get_bits(info, 263, 8);
        uint32_t to_bow = get_bits(info, 271, 9);
        uint32_t to_stern = get_bits(info, 280, 9);
        uint32_t to_port = get_bits(info, 289, 6);
        uint32_t to_starboard = get_bits(info, 295, 6);
        char name[21];
        ais_get_text(info, 143, 20, name, sizeof(name));

        printf("      classB-ext: name=%s ship_type=%u lat=%.5f lon=%.5f sog=%.1f kn cog=%.1f hdg=%u dim=%u/%u/%u/%u\n",
               name, ship_type, lat_raw / 600000.0, lon_raw / 600000.0, sog / 10.0, cog / 10.0, hdg,
               to_bow, to_stern, to_port, to_starboard);
    } else if (msg_type == 24) {
        if (!ais_has_bits(info_len, 160)) {
            printf("      warning: payload too short for type 24\n");
            return;
        }

        uint32_t part_no = get_bits(info, 38, 2);
        if (part_no == 0) {
            char name[21];
            ais_get_text(info, 40, 20, name, sizeof(name));
            printf("      type24A: name=%s\n", name);
        } else if (part_no == 1) {
            if (!ais_has_bits(info_len, 168)) {
                printf("      warning: payload too short for type 24B\n");
                return;
            }
            uint32_t ship_type = get_bits(info, 40, 8);
            uint32_t to_bow = get_bits(info, 132, 9);
            uint32_t to_stern = get_bits(info, 141, 9);
            uint32_t to_port = get_bits(info, 150, 6);
            uint32_t to_starboard = get_bits(info, 156, 6);
            char vendor[8];
            char callsign[8];
            ais_get_text(info, 48, 7, vendor, sizeof(vendor));
            ais_get_text(info, 90, 7, callsign, sizeof(callsign));

            printf("      type24B: ship_type=%u vendor=%s callsign=%s dim=%u/%u/%u/%u\n",
                   ship_type, vendor, callsign, to_bow, to_stern, to_port, to_starboard);
        } else {
            printf("      type24: invalid part=%u\n", part_no);
        }
    } else {
        printf("      payload bits=%zu (parser base)\n", info_len * 8u);
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
            // I 7 bit precedenti sono i primi 7 bit del flag di chiusura.
            // Rimuoviamoli dal payload prima della validazione CRC.
            buf_drop_tail_bits(ctx, 7);
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
        if (decoded == 0) {
            // stuffed zero: termina la sequenza di 1.
            ctx->ones_count = 0;
            return;
        }
        // Se è 1, non è stuffed (può essere parte di flag/abort): continua.
    }

    if (decoded == 1) {
        ctx->ones_count++;
        if (ctx->ones_count == 5) {
            ctx->skip_next_zero = 1;
        } else if (ctx->ones_count >= 7) {
            // HDLC abort / rumore: reset del frame corrente.
            ctx->in_frame = 0;
            ctx->ones_count = 0;
            ctx->skip_next_zero = 0;
            buf_reset(ctx);
            return;
        }
    } else {
        ctx->ones_count = 0;
    }

    // aggiungi bit al buffer
    buf_push_bit_lsb(ctx, decoded);
}

static void ais_on_demod_bit_cb(void *user, uint8_t bit) {
    if (!user) return;
    ais_process_demod_bit((ais_ctx_t *)user, bit);
}

void ais_init(ais_ctx_t *ctx) {
    memset(ctx, 0, sizeof(*ctx));
}

void ais_get_demod_config(demod_config_t *cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->kind = DEMOD_KIND_GMSK;
    cfg->input_fs = 2400000;
    cfg->output_fs = 96000;
    cfg->u.gmsk.symbol_rate = 9600;
    cfg->u.gmsk.m = 3;
    cfg->u.gmsk.bt = 0.4f;
}

demod_output_t ais_get_demod_output(ais_ctx_t *ctx) {
    demod_output_t out;
    memset(&out, 0, sizeof(out));
    out.on_bit = ais_on_demod_bit_cb;
    out.user = ctx;
    return out;
}

void ais_process_demod_bit(ais_ctx_t *ctx, uint8_t bit) {
    if (!ctx) return;
    hdlc_push_bit(ctx, (int)(bit & 1u));
}

void ais_flush(ais_ctx_t *ctx) {
    (void)ctx;
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
