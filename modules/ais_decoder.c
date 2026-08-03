#include "header/ais_decoder.h"
#include "dashboard.h"
#include "output.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>

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
    ctx->overflow = 0;
}

static void buf_push_bit_lsb(ais_ctx_t *ctx, int bit) {
    if (ctx->buf_len >= sizeof(ctx->buf)) {
        ctx->overflow = 1;
        return;
    }
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
    // AIS fields are MSB-first values. HDLC puts successive on-air bits in
    // increasing bit positions of each internal byte, so extract by stream order.
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

static int ais_require_bits(size_t info_len, int bits, uint32_t type) {
    if (ais_has_bits(info_len, bits)) return 1;
    printf("[AIS] type=%u invalid=payload-too-short have=%zu required=%d bits\n",
           type, info_len * 8u, bits);
    return 0;
}

static void ais_print_position(int32_t lon_raw, int32_t lat_raw, double scale) {
    double lon = (double)lon_raw / scale;
    double lat = (double)lat_raw / scale;
    if (fabs(lon) > 180.0 || fabs(lat) > 90.0) {
        printf(" position=unavailable");
    } else {
        printf(" lat=%.6f lon=%.6f", lat, lon);
    }
}

static const char *ais_type_name(uint32_t type) {
    static const char *const names[29] = {
        "invalid", "class-a-position", "class-a-position", "class-a-position",
        "base-station-report", "static-voyage-data", "addressed-binary",
        "binary-acknowledge", "binary-broadcast", "sar-aircraft-position",
        "utc-inquiry", "utc-response", "addressed-safety", "safety-acknowledge",
        "safety-broadcast", "interrogation", "assigned-mode-command",
        "dgnss-broadcast", "class-b-position", "class-b-extended-position",
        "data-link-management", "aid-to-navigation", "channel-management",
        "group-assignment", "static-data-report", "single-slot-binary",
        "multi-slot-binary", "long-range-broadcast", "single-slot-aid-to-navigation"
    };
    return type < 29u ? names[type] : "future-or-unknown";
}

static void ais_json_string(const char *text) {
    putchar('"');
    if (text) {
        for (const unsigned char *p = (const unsigned char *)text; *p; p++) {
            if (*p == '"' || *p == '\\') printf("\\%c", *p);
            else if (*p >= 0x20u) putchar(*p);
        }
    }
    putchar('"');
}

static void ais_json_position(int32_t lon_raw, int32_t lat_raw, double scale) {
    double lon = (double)lon_raw / scale;
    double lat = (double)lat_raw / scale;
    if (fabs(lon) <= 180.0 && fabs(lat) <= 90.0) {
        printf(",\"latitude\":%.6f,\"longitude\":%.6f", lat, lon);
    }
}

static int ais_minimum_bits(uint32_t type, const uint8_t *info, size_t info_len) {
    (void)info_len;
    switch (type) {
    case 1: case 2: case 3: case 4: case 9: case 11: case 18: case 22:
    case 28: return 168;
    case 5: return 424;
    case 6: case 15: return 88;
    case 7: case 10: case 12: case 13: case 20: return 72;
    case 8: return 56;
    case 14: case 25: return 40;
    case 16: case 27: return 96;
    case 17: return 80;
    case 19: return 312;
    case 21: return 272;
    case 23: return 160;
    case 24:
        return info && get_bits(info, 38, 2) == 1u ? 168 : 160;
    case 26: return 60;
    default: return 38;
    }
}

static void ais_dashboard_position(dashboard_ais_update_t *update,
                                   int32_t longitude_raw, int32_t latitude_raw,
                                   double scale) {
    double longitude = (double)longitude_raw / scale;
    double latitude = (double)latitude_raw / scale;
    if (!update || fabs(longitude) > 180.0 || fabs(latitude) > 90.0) return;
    update->longitude = longitude;
    update->latitude = latitude;
    update->has_position = 1;
}

static void ais_dashboard_payload(const uint8_t *info, size_t info_len,
                                  uint32_t type, uint32_t mmsi, char channel) {
    dashboard_ais_update_t update;
    char name[35] = "";
    char callsign[8] = "";
    uint32_t sog;
    uint32_t cog;
    uint32_t heading;
    (void)info_len;
    memset(&update, 0, sizeof(update));
    update.mmsi = mmsi;
    update.message_type = (uint8_t)type;
    update.channel = channel;

    switch (type) {
    case 1: case 2: case 3:
        sog = get_bits(info, 50, 10);
        cog = get_bits(info, 116, 12);
        heading = get_bits(info, 128, 9);
        ais_dashboard_position(&update, get_sbits(info, 61, 28),
                               get_sbits(info, 89, 27), 600000.0);
        if (sog < 1023u) { update.speed_kt = (double)sog / 10.0; update.has_speed = 1; }
        if (cog < 3600u) { update.course_deg = (double)cog / 10.0; update.has_course = 1; }
        if (heading < 360u) { update.heading_deg = heading; update.has_heading = 1; }
        update.navigation_status = get_bits(info, 38, 4);
        update.has_navigation_status = 1;
        break;
    case 4: case 11:
        ais_dashboard_position(&update, get_sbits(info, 79, 28),
                               get_sbits(info, 107, 27), 600000.0);
        break;
    case 5:
        ais_get_text(info, 70, 7, callsign, sizeof(callsign));
        ais_get_text(info, 112, 20, name, sizeof(name));
        update.callsign = callsign;
        update.name = name;
        update.ship_type = (uint8_t)get_bits(info, 232, 8);
        update.has_ship_type = 1;
        break;
    case 9:
        sog = get_bits(info, 50, 10);
        cog = get_bits(info, 116, 12);
        ais_dashboard_position(&update, get_sbits(info, 61, 28),
                               get_sbits(info, 89, 27), 600000.0);
        if (sog < 1023u) { update.speed_kt = (double)sog / 10.0; update.has_speed = 1; }
        if (cog < 3600u) { update.course_deg = (double)cog / 10.0; update.has_course = 1; }
        break;
    case 18: case 19:
        sog = get_bits(info, 46, 10);
        cog = get_bits(info, 112, 12);
        heading = get_bits(info, 124, 9);
        ais_dashboard_position(&update, get_sbits(info, 57, 28),
                               get_sbits(info, 85, 27), 600000.0);
        if (sog < 1023u) { update.speed_kt = (double)sog / 10.0; update.has_speed = 1; }
        if (cog < 3600u) { update.course_deg = (double)cog / 10.0; update.has_course = 1; }
        if (heading < 360u) { update.heading_deg = heading; update.has_heading = 1; }
        if (type == 19u) {
            ais_get_text(info, 143, 20, name, sizeof(name));
            update.name = name;
            update.ship_type = (uint8_t)get_bits(info, 263, 8);
            update.has_ship_type = 1;
        }
        break;
    case 21:
        ais_get_text(info, 43, 20, name, sizeof(name));
        update.name = name;
        ais_dashboard_position(&update, get_sbits(info, 164, 28),
                               get_sbits(info, 192, 27), 600000.0);
        break;
    case 24:
        if (get_bits(info, 38, 2) == 0u) {
            ais_get_text(info, 40, 20, name, sizeof(name));
            update.name = name;
        } else if (get_bits(info, 38, 2) == 1u) {
            ais_get_text(info, 90, 7, callsign, sizeof(callsign));
            update.callsign = callsign;
            update.ship_type = (uint8_t)get_bits(info, 40, 8);
            update.has_ship_type = 1;
        }
        break;
    case 27:
        ais_dashboard_position(&update, get_sbits(info, 44, 18),
                               get_sbits(info, 62, 17), 600.0);
        sog = get_bits(info, 79, 6);
        cog = get_bits(info, 85, 9);
        if (sog < 63u) { update.speed_kt = (double)sog; update.has_speed = 1; }
        if (cog < 511u) { update.course_deg = (double)cog / 2.0; update.has_course = 1; }
        update.navigation_status = get_bits(info, 40, 4);
        update.has_navigation_status = 1;
        break;
    case 28:
        ais_dashboard_position(&update, get_sbits(info, 44, 28),
                               get_sbits(info, 72, 27), 600000.0);
        break;
    default:
        break;
    }
    dashboard_update_ais(&update);
}

static int ais_decode_payload_channel(const uint8_t *info, size_t info_len,
                                      char channel) {
    uint32_t type;
    uint32_t mmsi;

    if (!info || !ais_has_bits(info_len, 38)) {
        fprintf(stderr, "[AIS] payload must contain at least 38 bits\n");
        return 0;
    }
    type = get_bits(info, 0, 6);
    mmsi = get_bits(info, 8, 30);
    {
        int required = ais_minimum_bits(type, info, info_len);
        if (!ais_has_bits(info_len, required)) {
            fprintf(stderr,
                    "[AIS] type=%u invalid=payload-too-short have=%zu required=%d bits\n",
                    type, info_len * 8u, required);
            return 0;
        }
    }
    if (output_is_dashboard()) {
        ais_dashboard_payload(info, info_len, type, mmsi, channel);
        return type >= 1u && type <= 28u;
    }
    if (output_get_format() == OUTPUT_FORMAT_JSON) {
        char text[32];
        printf("{\"protocol\":\"ais\"");
        if (channel == 'A' || channel == 'B') printf(",\"channel\":\"%c\"", channel);
        printf(",\"message_type\":%u,\"message_name\":\"%s\",\"repeat\":%u,\"mmsi\":\"%09u\"",
               type, ais_type_name(type), get_bits(info, 6, 2), mmsi);
        if ((type >= 1u && type <= 3u) && ais_has_bits(info_len, 168)) {
            uint32_t sog = get_bits(info, 50, 10);
            uint32_t cog = get_bits(info, 116, 12);
            ais_json_position(get_sbits(info, 61, 28), get_sbits(info, 89, 27), 600000.0);
            if (sog < 1023u) printf(",\"speed_over_ground_kt\":%.1f", (double)sog / 10.0);
            if (cog < 3600u) printf(",\"course_over_ground_deg\":%.1f", (double)cog / 10.0);
        } else if ((type == 4u || type == 11u) && ais_has_bits(info_len, 168)) {
            ais_json_position(get_sbits(info, 79, 28), get_sbits(info, 107, 27), 600000.0);
        } else if (type == 5u && ais_has_bits(info_len, 424)) {
            ais_get_text(info, 70, 7, text, sizeof(text));
            printf(",\"callsign\":");
            ais_json_string(text);
            ais_get_text(info, 112, 20, text, sizeof(text));
            printf(",\"vessel_name\":");
            ais_json_string(text);
            printf(",\"ship_type\":%u", get_bits(info, 232, 8));
        } else if (type == 9u && ais_has_bits(info_len, 168)) {
            ais_json_position(get_sbits(info, 61, 28), get_sbits(info, 89, 27), 600000.0);
        } else if ((type == 18u || type == 19u) && ais_has_bits(info_len, 168)) {
            ais_json_position(get_sbits(info, 57, 28), get_sbits(info, 85, 27), 600000.0);
        } else if (type == 21u && ais_has_bits(info_len, 272)) {
            ais_get_text(info, 43, 20, text, sizeof(text));
            printf(",\"name\":");
            ais_json_string(text);
            ais_json_position(get_sbits(info, 164, 28), get_sbits(info, 192, 27), 600000.0);
        } else if (type == 27u && ais_has_bits(info_len, 96)) {
            ais_json_position(get_sbits(info, 44, 18), get_sbits(info, 62, 17), 600.0);
        } else if (type == 28u && ais_has_bits(info_len, 168)) {
            ais_json_position(get_sbits(info, 44, 28), get_sbits(info, 72, 27), 600000.0);
        }
        printf(",\"payload_hex_lsb_packed\":\"");
        for (size_t i = 0; i < info_len; i++) printf("%02X", info[i]);
        printf("\"}\n");
        return type >= 1u && type <= 28u;
    }
    if (output_get_format() == OUTPUT_FORMAT_CSV) {
        static int header_printed = 0;
        if (!header_printed) {
            printf("protocol,channel,message_type,message_name,repeat,mmsi,payload_hex\n");
            header_printed = 1;
        }
        printf("ais,%c,%u,%s,%u,%09u,",
               channel == 'A' || channel == 'B' ? channel : '-',
               type, ais_type_name(type),
               get_bits(info, 6, 2), mmsi);
        for (size_t i = 0; i < info_len; i++) printf("%02X", info[i]);
        printf("\n");
        return type >= 1u && type <= 28u;
    }
    if (output_get_format() == OUTPUT_FORMAT_QUIET) return type >= 1u && type <= 28u;
    printf("[AIS]");
    if (channel == 'A' || channel == 'B') printf(" channel=%c", channel);
    printf(" type=%u name=%s repeat=%u mmsi=%09u",
           type, ais_type_name(type), get_bits(info, 6, 2), mmsi);

    switch (type) {
    case 1:
    case 2:
    case 3: {
        uint32_t sog;
        uint32_t cog;
        uint32_t heading;
        if (!ais_require_bits(info_len, 168, type)) return 0;
        sog = get_bits(info, 50, 10);
        cog = get_bits(info, 116, 12);
        heading = get_bits(info, 128, 9);
        printf(" nav_status=%u rot_raw=%d", get_bits(info, 38, 4),
               get_sbits(info, 42, 8));
        ais_print_position(get_sbits(info, 61, 28), get_sbits(info, 89, 27), 600000.0);
        if (sog < 1023u) printf(" sog=%.1fkt", (double)sog / 10.0);
        if (cog < 3600u) printf(" cog=%.1fdeg", (double)cog / 10.0);
        if (heading < 360u) printf(" heading=%udeg", heading);
        printf(" timestamp=%u\n", get_bits(info, 137, 6));
        return 1;
    }
    case 4:
    case 11:
        if (!ais_require_bits(info_len, 168, type)) return 0;
        printf(" utc=%04u-%02u-%02uT%02u:%02u:%02uZ accuracy=%u",
               get_bits(info, 38, 14), get_bits(info, 52, 4),
               get_bits(info, 56, 5), get_bits(info, 61, 5),
               get_bits(info, 66, 6), get_bits(info, 72, 6),
               get_bits(info, 78, 1));
        ais_print_position(get_sbits(info, 79, 28), get_sbits(info, 107, 27), 600000.0);
        printf(" epfd=%u raim=%u\n", get_bits(info, 134, 4), get_bits(info, 148, 1));
        return 1;
    case 5: {
        char callsign[8];
        char name[21];
        char destination[21];
        if (!ais_require_bits(info_len, 424, type)) return 0;
        ais_get_text(info, 70, 7, callsign, sizeof(callsign));
        ais_get_text(info, 112, 20, name, sizeof(name));
        ais_get_text(info, 302, 20, destination, sizeof(destination));
        printf(" ais_version=%u imo=%u callsign=\"%s\" vessel=\"%s\" ship_type=%u",
               get_bits(info, 38, 2), get_bits(info, 40, 30), callsign, name,
               get_bits(info, 232, 8));
        printf(" dimensions=%u/%u/%u/%u epfd=%u eta=%02u-%02uT%02u:%02u draught=%.1fm destination=\"%s\" dte=%u\n",
               get_bits(info, 240, 9), get_bits(info, 249, 9),
               get_bits(info, 258, 6), get_bits(info, 264, 6),
               get_bits(info, 270, 4), get_bits(info, 274, 4),
               get_bits(info, 278, 5), get_bits(info, 283, 5),
               get_bits(info, 288, 6), (double)get_bits(info, 294, 8) / 10.0,
               destination, get_bits(info, 422, 1));
        return 1;
    }
    case 6:
        if (!ais_require_bits(info_len, 88, type)) return 0;
        printf(" sequence=%u destination=%09u retransmit=%u dac=%u fid=%u data_bits=%zu\n",
               get_bits(info, 38, 2), get_bits(info, 40, 30), get_bits(info, 70, 1),
               get_bits(info, 72, 10), get_bits(info, 82, 6), info_len * 8u - 88u);
        return 1;
    case 7:
    case 13: {
        size_t bit = 40u;
        if (!ais_require_bits(info_len, 72, type)) return 0;
        while (bit + 32u <= info_len * 8u && bit < 168u) {
            printf(" ack=%09u/%u", get_bits(info, (int)bit, 30),
                   get_bits(info, (int)bit + 30, 2));
            bit += 32u;
        }
        printf("\n");
        return 1;
    }
    case 8:
        if (!ais_require_bits(info_len, 56, type)) return 0;
        printf(" dac=%u fid=%u data_bits=%zu\n", get_bits(info, 40, 10),
               get_bits(info, 50, 6), info_len * 8u - 56u);
        return 1;
    case 9: {
        uint32_t altitude;
        uint32_t sog;
        uint32_t cog;
        if (!ais_require_bits(info_len, 168, type)) return 0;
        altitude = get_bits(info, 38, 12);
        sog = get_bits(info, 50, 10);
        cog = get_bits(info, 116, 12);
        if (altitude < 4095u) printf(" altitude=%um", altitude);
        if (sog < 1023u) printf(" sog=%.1fkt", (double)sog / 10.0);
        ais_print_position(get_sbits(info, 61, 28), get_sbits(info, 89, 27), 600000.0);
        if (cog < 3600u) printf(" cog=%.1fdeg", (double)cog / 10.0);
        printf(" timestamp=%u altitude_sensor=%u dte=%u assigned=%u raim=%u\n",
               get_bits(info, 128, 6), get_bits(info, 134, 1),
               get_bits(info, 142, 1), get_bits(info, 146, 1), get_bits(info, 147, 1));
        return 1;
    }
    case 10:
        if (!ais_require_bits(info_len, 72, type)) return 0;
        printf(" destination=%09u\n", get_bits(info, 40, 30));
        return 1;
    case 12:
    case 14: {
        int text_start = type == 12u ? 72 : 40;
        int chars;
        char text[162];
        if (!ais_require_bits(info_len, text_start, type)) return 0;
        if (type == 12u) {
            printf(" sequence=%u destination=%09u retransmit=%u",
                   get_bits(info, 38, 2), get_bits(info, 40, 30), get_bits(info, 70, 1));
        }
        chars = (int)((info_len * 8u - (size_t)text_start) / 6u);
        ais_get_text(info, text_start, chars, text, sizeof(text));
        printf(" text=\"%s\"\n", text);
        return 1;
    }
    case 15:
        if (!ais_require_bits(info_len, 88, type)) return 0;
        printf(" interrogated_mmsi=%09u message_type=%u slot_offset=%u",
               get_bits(info, 40, 30), get_bits(info, 70, 6), get_bits(info, 76, 12));
        if (ais_has_bits(info_len, 110)) {
            printf(" second_type=%u second_slot=%u", get_bits(info, 90, 6),
                   get_bits(info, 96, 12));
        }
        printf("\n");
        return 1;
    case 16:
        if (!ais_require_bits(info_len, 96, type)) return 0;
        printf(" destination=%09u offset=%u increment=%u",
               get_bits(info, 40, 30), get_bits(info, 70, 12), get_bits(info, 82, 10));
        if (ais_has_bits(info_len, 144)) {
            printf(" destination2=%09u offset2=%u increment2=%u",
                   get_bits(info, 92, 30), get_bits(info, 122, 12), get_bits(info, 134, 10));
        }
        printf("\n");
        return 1;
    case 17:
        if (!ais_require_bits(info_len, 80, type)) return 0;
        ais_print_position(get_sbits(info, 40, 18), get_sbits(info, 58, 17), 600.0);
        printf(" correction_bits=%zu\n", info_len * 8u - 80u);
        return 1;
    case 18: {
        uint32_t sog;
        uint32_t cog;
        uint32_t heading;
        if (!ais_require_bits(info_len, 168, type)) return 0;
        sog = get_bits(info, 46, 10);
        cog = get_bits(info, 112, 12);
        heading = get_bits(info, 124, 9);
        ais_print_position(get_sbits(info, 57, 28), get_sbits(info, 85, 27), 600000.0);
        if (sog < 1023u) printf(" sog=%.1fkt", (double)sog / 10.0);
        if (cog < 3600u) printf(" cog=%.1fdeg", (double)cog / 10.0);
        if (heading < 360u) printf(" heading=%udeg", heading);
        printf(" timestamp=%u cs=%u display=%u dsc=%u band=%u msg22=%u assigned=%u raim=%u\n",
               get_bits(info, 133, 6), get_bits(info, 141, 1), get_bits(info, 142, 1),
               get_bits(info, 143, 1), get_bits(info, 144, 1), get_bits(info, 145, 1),
               get_bits(info, 146, 1), get_bits(info, 147, 1));
        return 1;
    }
    case 19: {
        char name[21];
        if (!ais_require_bits(info_len, 312, type)) return 0;
        ais_get_text(info, 143, 20, name, sizeof(name));
        ais_print_position(get_sbits(info, 57, 28), get_sbits(info, 85, 27), 600000.0);
        printf(" sog=%.1fkt cog=%.1fdeg heading=%u vessel=\"%s\" ship_type=%u dimensions=%u/%u/%u/%u epfd=%u raim=%u dte=%u assigned=%u\n",
               (double)get_bits(info, 46, 10) / 10.0,
               (double)get_bits(info, 112, 12) / 10.0,
               get_bits(info, 124, 9), name, get_bits(info, 263, 8),
               get_bits(info, 271, 9), get_bits(info, 280, 9),
               get_bits(info, 289, 6), get_bits(info, 295, 6),
               get_bits(info, 301, 4), get_bits(info, 305, 1),
               get_bits(info, 306, 1), get_bits(info, 307, 1));
        return 1;
    }
    case 20: {
        size_t bit = 40u;
        if (!ais_require_bits(info_len, 72, type)) return 0;
        while (bit + 30u <= info_len * 8u && bit < 160u) {
            printf(" reservation=%u/%u/%u/%u", get_bits(info, (int)bit, 12),
                   get_bits(info, (int)bit + 12, 4), get_bits(info, (int)bit + 16, 3),
                   get_bits(info, (int)bit + 19, 11));
            bit += 30u;
        }
        printf("\n");
        return 1;
    }
    case 21: {
        char name[35];
        int chars;
        if (!ais_require_bits(info_len, 272, type)) return 0;
        chars = 20;
        if (info_len * 8u > 272u) chars += (int)((info_len * 8u - 272u) / 6u);
        if (chars > 34) chars = 34;
        ais_get_text(info, 43, 20, name, sizeof(name));
        if (chars > 20) {
            char extension[15];
            size_t base_len = strlen(name);
            ais_get_text(info, 272, chars - 20, extension, sizeof(extension));
            if (base_len + strlen(extension) < sizeof(name)) strcat(name, extension);
        }
        printf(" aid_type=%u name=\"%s\" accuracy=%u", get_bits(info, 38, 5),
               name, get_bits(info, 163, 1));
        ais_print_position(get_sbits(info, 164, 28), get_sbits(info, 192, 27), 600000.0);
        printf(" dimensions=%u/%u/%u/%u epfd=%u timestamp=%u off_position=%u regional=%u raim=%u virtual=%u assigned=%u\n",
               get_bits(info, 219, 9), get_bits(info, 228, 9),
               get_bits(info, 237, 6), get_bits(info, 243, 6),
               get_bits(info, 249, 4), get_bits(info, 253, 6),
               get_bits(info, 259, 1), get_bits(info, 260, 8),
               get_bits(info, 268, 1), get_bits(info, 269, 1), get_bits(info, 270, 1));
        return 1;
    }
    case 22:
        if (!ais_require_bits(info_len, 168, type)) return 0;
        printf(" channel_a=%u channel_b=%u txrx_mode=%u low_power=%u addressed=%u",
               get_bits(info, 40, 12), get_bits(info, 52, 12), get_bits(info, 64, 4),
               get_bits(info, 68, 1), get_bits(info, 139, 1));
        if (get_bits(info, 139, 1)) {
            printf(" destination1=%09u destination2=%09u",
                   get_bits(info, 69, 30), get_bits(info, 104, 30));
        } else {
            printf(" ne_lon=%.3f ne_lat=%.3f sw_lon=%.3f sw_lat=%.3f",
                   (double)get_sbits(info, 69, 18) / 600.0,
                   (double)get_sbits(info, 87, 17) / 600.0,
                   (double)get_sbits(info, 104, 18) / 600.0,
                   (double)get_sbits(info, 122, 17) / 600.0);
        }
        printf(" bandwidth_a=%u bandwidth_b=%u zone=%u\n",
               get_bits(info, 140, 1), get_bits(info, 141, 1), get_bits(info, 142, 3));
        return 1;
    case 23:
        if (!ais_require_bits(info_len, 160, type)) return 0;
        printf(" ne_lon=%.3f ne_lat=%.3f sw_lon=%.3f sw_lat=%.3f station_type=%u ship_type=%u interval=%u quiet=%u\n",
               (double)get_sbits(info, 40, 18) / 600.0,
               (double)get_sbits(info, 58, 17) / 600.0,
               (double)get_sbits(info, 75, 18) / 600.0,
               (double)get_sbits(info, 93, 17) / 600.0,
               get_bits(info, 110, 4), get_bits(info, 114, 8),
               get_bits(info, 144, 4), get_bits(info, 148, 4));
        return 1;
    case 24: {
        uint32_t part;
        if (!ais_require_bits(info_len, 160, type)) return 0;
        part = get_bits(info, 38, 2);
        if (part == 0u) {
            char name[21];
            ais_get_text(info, 40, 20, name, sizeof(name));
            printf(" part=A vessel=\"%s\"\n", name);
        } else if (part == 1u) {
            char vendor[4];
            char callsign[8];
            if (!ais_require_bits(info_len, 168, type)) return 0;
            ais_get_text(info, 48, 3, vendor, sizeof(vendor));
            ais_get_text(info, 90, 7, callsign, sizeof(callsign));
            printf(" part=B ship_type=%u vendor=%s model=%u serial=%u callsign=%s dimensions_or_mothership=%08X\n",
                   get_bits(info, 40, 8), vendor, get_bits(info, 66, 4),
                   get_bits(info, 70, 20), callsign, get_bits(info, 132, 30));
        } else {
            printf(" invalid_part=%u\n", part);
            return 0;
        }
        return 1;
    }
    case 25:
    case 26: {
        uint32_t addressed;
        uint32_t structured;
        size_t start = 40u;
        if (!ais_require_bits(info_len, type == 25u ? 40 : 60, type)) return 0;
        addressed = get_bits(info, 38, 1);
        structured = get_bits(info, 39, 1);
        printf(" addressed=%u structured=%u", addressed, structured);
        if (addressed) {
            printf(" destination=%09u", get_bits(info, (int)start, 30));
            start += 30u;
        }
        if (structured) {
            printf(" dac=%u fid=%u", get_bits(info, (int)start, 10),
                   get_bits(info, (int)start + 10, 6));
            start += 16u;
        }
        printf(" data_bits=%zu\n", info_len * 8u > start ? info_len * 8u - start : 0u);
        return 1;
    }
    case 27:
        if (!ais_require_bits(info_len, 96, type)) return 0;
        printf(" accuracy=%u raim=%u nav_status=%u",
               get_bits(info, 38, 1), get_bits(info, 39, 1), get_bits(info, 40, 4));
        ais_print_position(get_sbits(info, 44, 18), get_sbits(info, 62, 17), 600.0);
        if (get_bits(info, 79, 6) < 63u) printf(" sog=%ukt", get_bits(info, 79, 6));
        if (get_bits(info, 85, 9) < 511u) printf(" cog=%.1fdeg", (double)get_bits(info, 85, 9) / 2.0);
        printf(" gnss=%u\n", get_bits(info, 94, 1));
        return 1;
    case 28:
        if (!ais_require_bits(info_len, 168, type)) return 0;
        printf(" timestamp=%u", get_bits(info, 38, 6));
        ais_print_position(get_sbits(info, 44, 28), get_sbits(info, 72, 27), 600000.0);
        printf(" restricted_use=%u station_type=%u aton_type=%u mrn=%u dimension_type=%u dimension_a=%u dimension_b=%u additional_dimensions=%u charted=%u on_station=%u status=%u authenticated=%u\n",
               get_bits(info, 99, 2), get_bits(info, 101, 3),
               get_bits(info, 104, 7), get_bits(info, 111, 17),
               get_bits(info, 128, 4), get_bits(info, 132, 9),
               get_bits(info, 141, 11), get_bits(info, 152, 1),
               get_bits(info, 153, 1), get_bits(info, 154, 4),
               get_bits(info, 158, 8), get_bits(info, 167, 1));
        return 1;
    default:
        printf(" payload_bits=%zu decoder=future-or-unknown\n", info_len * 8u);
        return 0;
    }
}

int ais_decode_payload(const uint8_t *info, size_t info_len) {
    return ais_decode_payload_channel(info, info_len, '-');
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
    return (uint32_t)((int64_t)(1ULL << bits) + (int64_t)v);
}

static uint8_t ais_char_to_sixbit(char c) {
    c = (char)toupper((unsigned char)c);
    if (c >= 'A' && c <= 'Z') return (uint8_t)(c - 'A' + 1);
    if (c >= '0' && c <= '9') return (uint8_t)c;
    if (c >= ' ' && c <= '?') return (uint8_t)c;
    return 32u;
}

static void set_text_stream(uint8_t *bytes, int start_bit, int chars, const char *text) {
    size_t text_len = text ? strlen(text) : 0u;
    for (int i = 0; i < chars; i++) {
        char c = (size_t)i < text_len ? text[i] : '@';
        set_bits_stream(bytes, start_bit + i * 6, 6, ais_char_to_sixbit(c));
    }
}

// ---- Frame handling ----
static void hdlc_end_frame(ais_ctx_t *ctx) {
    ctx->frames_seen++;

    if (ctx->overflow) {
        ctx->overflow_frames++;
        buf_reset(ctx);
        return;
    }

    if (ctx->cur_bitpos != 0 || ctx->buf_len < 3) {
        buf_reset(ctx);
        return;
    }

    // ultimi 2 byte sono FCS (LSB-first)
    size_t info_len = ctx->buf_len - 2;
    const uint8_t *info = ctx->buf;

    uint16_t rx_fcs = (uint16_t)(ctx->buf[info_len] | ((uint16_t)ctx->buf[info_len + 1] << 8));
    uint16_t calc   = crc16_hdlc(info, info_len);

    if (calc == rx_fcs) {
        ctx->frames_valid++;
        ais_on_frame(info, info_len);
        (void)ais_decode_payload_channel(info, info_len, ctx->channel_name);
    } else {
        ctx->crc_errors++;
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
            ctx->aborted_frames++;
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
    ctx->channel_name = '-';
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
    cfg->u.gmsk.channel_count = 1u;
    cfg->u.gmsk.channel_offset_hz[0] = 0.0f;
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
    ctx->demod_bits++;
    hdlc_push_bit(ctx, (int)(bit & 1u));
}

void ais_flush(ais_ctx_t *ctx) {
    if (!ctx) return;
    fprintf(output_diagnostics(),
            "[AIS] flush channel=%c demod_bits=%llu frames_seen=%llu valid=%llu crc_errors=%llu aborted=%llu overflow=%llu\n",
            ctx->channel_name,
            (unsigned long long)ctx->demod_bits,
            (unsigned long long)ctx->frames_seen,
            (unsigned long long)ctx->frames_valid,
            (unsigned long long)ctx->crc_errors,
            (unsigned long long)ctx->aborted_frames,
            (unsigned long long)ctx->overflow_frames);
}

static void ais_on_channel_demod_bit_cb(void *user, unsigned int channel,
                                        uint8_t bit) {
    ais_receiver_t *receiver = (ais_receiver_t *)user;
    if (!receiver || channel >= receiver->channel_count) return;
    ais_process_demod_bit(&receiver->channel[channel], bit);
}

int ais_receiver_set_channels(ais_receiver_t *receiver, const char *selection) {
    char selected;
    if (!receiver) return 0;
    if (!selection || strcmp(selection, "both") == 0 || strcmp(selection, "BOTH") == 0
        || strcmp(selection, "*") == 0) {
        selected = '*';
    } else if ((selection[0] == 'A' || selection[0] == 'a' || selection[0] == '1')
               && selection[1] == '\0') {
        selected = 'A';
    } else if ((selection[0] == 'B' || selection[0] == 'b' || selection[0] == '2')
               && selection[1] == '\0') {
        selected = 'B';
    } else {
        return 0;
    }
    receiver->selected_channel = selected;
    receiver->channel_count = selected == '*' ? 2u : 1u;
    return 1;
}

void ais_receiver_init(ais_receiver_t *receiver) {
    char selected;
    if (!receiver) return;
    selected = receiver->selected_channel;
    if (selected != 'A' && selected != 'B' && selected != '*') selected = '*';
    memset(receiver, 0, sizeof(*receiver));
    receiver->selected_channel = selected;
    receiver->channel_count = selected == '*' ? 2u : 1u;
    for (unsigned int i = 0; i < receiver->channel_count; i++) {
        ais_init(&receiver->channel[i]);
    }
    if (selected == '*') {
        receiver->channel[0].channel_name = 'A';
        receiver->channel[1].channel_name = 'B';
    } else {
        receiver->channel[0].channel_name = selected;
    }
}

void ais_receiver_get_demod_config(ais_receiver_t *receiver, demod_config_t *cfg) {
    ais_get_demod_config(cfg);
    if (!receiver || !cfg) return;
    if (receiver->selected_channel != 'A' && receiver->selected_channel != 'B'
        && receiver->selected_channel != '*') {
        receiver->selected_channel = '*';
        receiver->channel_count = 2u;
    }
    cfg->u.gmsk.channel_count = receiver->selected_channel == '*' ? 2u : 1u;
    if (cfg->u.gmsk.channel_count == 2u) {
        cfg->u.gmsk.channel_offset_hz[0] = -25000.0f;
        cfg->u.gmsk.channel_offset_hz[1] = 25000.0f;
    } else {
        cfg->u.gmsk.channel_offset_hz[0] = 0.0f;
    }
}

demod_output_t ais_receiver_get_demod_output(ais_receiver_t *receiver) {
    demod_output_t out;
    memset(&out, 0, sizeof(out));
    out.on_channel_bit = ais_on_channel_demod_bit_cb;
    out.user = receiver;
    return out;
}

void ais_receiver_flush(ais_receiver_t *receiver) {
    if (!receiver) return;
    for (unsigned int i = 0; i < receiver->channel_count; i++) {
        ais_flush(&receiver->channel[i]);
    }
}

int ais_receiver_test(ais_receiver_t *receiver) {
    int protocol_ok;
    int iq_ok;
    char channel_name;
    if (!receiver) return 0;
    channel_name = receiver->channel[0].channel_name;
    protocol_ok = ais_test_emit_examples(&receiver->channel[0]);
    receiver->channel[0].channel_name = channel_name;
    iq_ok = gmsk_demod_self_test();
    fprintf(output_diagnostics(), "[AIS] dual_channel_iq_test=%s\n",
            iq_ok ? "PASS" : "FAIL");
    return protocol_ok && iq_ok;
}

// Default frame callback. Applications may replace this output integration point.
void ais_on_frame(const uint8_t *frame, size_t len) {
    (void)frame;
    if (output_is_human()) printf("[AIS] frame len=%zu bytes\n", len);
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)tolower((unsigned char)c);
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

int ais_decode_payload_hex(const char *hex) {
    uint8_t payload[1024];
    size_t digits = 0u;
    size_t payload_len = 0u;
    int high = -1;

    if (!hex) return 0;
    for (const char *p = hex; *p; p++) {
        int value;
        if (isspace((unsigned char)*p) || *p == ':' || *p == '-') continue;
        value = hex_value(*p);
        if (value < 0) {
            fprintf(stderr, "[AIS] invalid hexadecimal payload\n");
            return 0;
        }
        digits++;
        if (high < 0) {
            high = value;
        } else {
            if (payload_len >= sizeof(payload)) {
                fprintf(stderr, "[AIS] hexadecimal payload is too large\n");
                return 0;
            }
            payload[payload_len++] = (uint8_t)((high << 4) | value);
            high = -1;
        }
    }
    if (digits == 0u || high >= 0) {
        fprintf(stderr, "[AIS] hexadecimal payload must contain complete bytes\n");
        return 0;
    }
    return ais_decode_payload(payload, payload_len);
}

typedef struct {
    int active;
    int total;
    int next_part;
    char sequence[16];
    char channel;
    char armored[2048];
    size_t armored_len;
} ais_nmea_assembly_t;

static ais_nmea_assembly_t g_nmea_assembly;

int ais_nmea_has_pending_fragments(void) {
    return g_nmea_assembly.active;
}

static int nmea_hex_byte(const char *text, unsigned int *value) {
    int hi;
    int lo;
    if (!text || !value || text[0] == '\0' || text[1] == '\0') return 0;
    hi = hex_value(text[0]);
    lo = hex_value(text[1]);
    if (hi < 0 || lo < 0) return 0;
    *value = (unsigned int)((hi << 4) | lo);
    return 1;
}

static int nmea_parse_int(const char *text, int *value) {
    char *end = NULL;
    long parsed;
    if (!text || !value) return 0;
    parsed = strtol(text, &end, 10);
    if (text == end || *end != '\0') return 0;
    *value = (int)parsed;
    return 1;
}

static int decode_nmea_armored(const char *armored, int fill_bits, char channel) {
    uint8_t payload[1024];
    size_t chars;
    size_t total_bits;

    if (!armored || fill_bits < 0 || fill_bits > 5) return 0;
    chars = strlen(armored);
    if (chars == 0u || chars * 6u < (size_t)fill_bits) return 0;
    total_bits = chars * 6u - (size_t)fill_bits;
    if ((total_bits + 7u) / 8u > sizeof(payload)) return 0;
    memset(payload, 0, sizeof(payload));

    for (size_t i = 0; i < chars; i++) {
        int value = (unsigned char)armored[i] - 48;
        if (value > 40) value -= 8;
        if (value < 0 || value > 63) {
            fprintf(stderr, "[AIS][NMEA] invalid armored character\n");
            return 0;
        }
        for (int bit = 0; bit < 6; bit++) {
            size_t stream_bit = i * 6u + (size_t)bit;
            if (stream_bit >= total_bits) break;
            if ((value >> (5 - bit)) & 1) {
                payload[stream_bit / 8u] |= (uint8_t)(1u << (stream_bit % 8u));
            }
        }
    }
    if (output_is_human()) {
        printf("[AIS][NMEA] channel=%c payload_bits=%zu checksum=valid\n",
               channel ? channel : '-', total_bits);
    }
    return ais_decode_payload_channel(payload, (total_bits + 7u) / 8u, channel);
}

int ais_decode_nmea_sentence(const char *sentence) {
    char copy[2300];
    char *fields[7] = { 0 };
    char *cursor;
    char *checksum_mark;
    size_t sentence_len;
    unsigned int expected_checksum;
    unsigned int checksum = 0u;
    int field_count = 0;
    int total;
    int part;
    int fill;

    if (!sentence) return 0;
    while (isspace((unsigned char)*sentence)) sentence++;
    sentence_len = strcspn(sentence, "\r\n");
    if (sentence_len >= sizeof(copy)) return 0;
    memcpy(copy, sentence, sentence_len);
    copy[sentence_len] = '\0';
    if (strncmp(copy, "!AIVDM,", 7u) != 0 && strncmp(copy, "!AIVDO,", 7u) != 0) {
        fprintf(stderr, "[AIS][NMEA] expected !AIVDM or !AIVDO\n");
        return 0;
    }
    checksum_mark = strrchr(copy, '*');
    if (!checksum_mark || !nmea_hex_byte(checksum_mark + 1, &expected_checksum)) {
        fprintf(stderr, "[AIS][NMEA] missing or invalid checksum\n");
        return 0;
    }
    for (char *p = copy + 1; p < checksum_mark; p++) checksum ^= (unsigned char)*p;
    if (checksum != expected_checksum) {
        fprintf(stderr, "[AIS][NMEA] checksum mismatch expected=%02X calculated=%02X\n",
                expected_checksum, checksum);
        return 0;
    }
    *checksum_mark = '\0';

    cursor = copy;
    while (field_count < 7) {
        fields[field_count++] = cursor;
        cursor = strchr(cursor, ',');
        if (!cursor) break;
        *cursor++ = '\0';
    }
    if (field_count != 7 || !nmea_parse_int(fields[1], &total)
        || !nmea_parse_int(fields[2], &part) || !nmea_parse_int(fields[6], &fill)) {
        fprintf(stderr, "[AIS][NMEA] malformed sentence fields\n");
        return 0;
    }
    if (total < 1 || total > 9 || part < 1 || part > total || fill < 0 || fill > 5) {
        fprintf(stderr, "[AIS][NMEA] invalid fragment counters or fill bits\n");
        return 0;
    }
    if (total == 1) return decode_nmea_armored(fields[5], fill, fields[4][0]);

    if (part == 1) {
        memset(&g_nmea_assembly, 0, sizeof(g_nmea_assembly));
        g_nmea_assembly.active = 1;
        g_nmea_assembly.total = total;
        g_nmea_assembly.next_part = 1;
        g_nmea_assembly.channel = fields[4][0];
        (void)snprintf(g_nmea_assembly.sequence, sizeof(g_nmea_assembly.sequence), "%s", fields[3]);
    }
    if (!g_nmea_assembly.active || total != g_nmea_assembly.total
        || part != g_nmea_assembly.next_part
        || strcmp(fields[3], g_nmea_assembly.sequence) != 0) {
        fprintf(stderr, "[AIS][NMEA] multipart sequence is incomplete or out of order\n");
        memset(&g_nmea_assembly, 0, sizeof(g_nmea_assembly));
        return 0;
    }
    if (g_nmea_assembly.armored_len + strlen(fields[5]) >= sizeof(g_nmea_assembly.armored)) {
        memset(&g_nmea_assembly, 0, sizeof(g_nmea_assembly));
        return 0;
    }
    strcat(g_nmea_assembly.armored, fields[5]);
    g_nmea_assembly.armored_len += strlen(fields[5]);
    g_nmea_assembly.next_part++;
    if (part < total) return 2;

    {
        int result = decode_nmea_armored(g_nmea_assembly.armored, fill,
                                         g_nmea_assembly.channel);
        memset(&g_nmea_assembly, 0, sizeof(g_nmea_assembly));
        return result;
    }
}

static void test_feed_decoded_bit(ais_ctx_t *ctx, int decoded, int *nrzi_level) {
    if (!decoded) *nrzi_level ^= 1;
    ais_process_demod_bit(ctx, (uint8_t)*nrzi_level);
}

static void test_feed_flag(ais_ctx_t *ctx, int *nrzi_level) {
    static const int flag_bits[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };
    for (size_t i = 0; i < sizeof(flag_bits) / sizeof(flag_bits[0]); i++) {
        test_feed_decoded_bit(ctx, flag_bits[i], nrzi_level);
    }
}

static void test_feed_hdlc_frame(ais_ctx_t *ctx, const uint8_t *info, size_t info_len,
                                 int *nrzi_level) {
    uint8_t bytes[1026];
    uint16_t fcs;
    int ones = 0;

    if (!ctx || !info || info_len > sizeof(bytes) - 2u) return;
    memcpy(bytes, info, info_len);
    fcs = crc16_hdlc(info, info_len);
    bytes[info_len] = (uint8_t)(fcs & 0xFFu);
    bytes[info_len + 1u] = (uint8_t)(fcs >> 8u);

    for (size_t i = 0; i < info_len + 2u; i++) {
        for (int b = 0; b < 8; b++) {
            int bit = (bytes[i] >> b) & 1;
            test_feed_decoded_bit(ctx, bit, nrzi_level);
            if (bit) {
                ones++;
                if (ones == 5) {
                    test_feed_decoded_bit(ctx, 0, nrzi_level);
                    ones = 0;
                }
            } else {
                ones = 0;
            }
        }
    }
    test_feed_flag(ctx, nrzi_level);
}

static int test_nmea_from_payload(const uint8_t *payload, size_t payload_len) {
    char armored[1400];
    char sentence[1500];
    size_t total_bits = payload_len * 8u;
    size_t armored_len = (total_bits + 5u) / 6u;
    int fill_bits = (int)(armored_len * 6u - total_bits);
    unsigned int checksum = 0u;

    if (!payload || armored_len + 1u > sizeof(armored)) return 0;
    for (size_t group = 0; group < armored_len; group++) {
        unsigned int value = 0u;
        for (int bit = 0; bit < 6; bit++) {
            size_t stream_bit = group * 6u + (size_t)bit;
            value <<= 1u;
            if (stream_bit < total_bits) {
                value |= (payload[stream_bit / 8u] >> (stream_bit % 8u)) & 1u;
            }
        }
        armored[group] = (char)(value < 40u ? value + 48u : value + 56u);
    }
    armored[armored_len] = '\0';
    (void)snprintf(sentence, sizeof(sentence), "!AIVDM,1,1,,A,%s,%d",
                   armored, fill_bits);
    for (const char *p = sentence + 1; *p; p++) checksum ^= (unsigned char)*p;
    (void)snprintf(sentence + strlen(sentence), sizeof(sentence) - strlen(sentence),
                   "*%02X", checksum);
    return ais_decode_nmea_sentence(sentence);
}

int ais_test_emit_examples(ais_ctx_t *ctx) {
    uint8_t position[21];
    uint8_t voyage[53];
    uint8_t aid[34];
    uint8_t aid_single_slot[21];
    int32_t lon_raw;
    int32_t lat_raw;
    int nrzi_level = 0;
    int nmea_ok;
    uint64_t valid_before;

    if (!ctx) return 0;
    ais_init(ctx);
    valid_before = ctx->frames_valid;
    memset(position, 0, sizeof(position));
    memset(voyage, 0, sizeof(voyage));
    memset(aid, 0, sizeof(aid));
    memset(aid_single_slot, 0, sizeof(aid_single_slot));

    set_bits_stream(position, 0, 6, 1u);
    set_bits_stream(position, 8, 30, 247320162u);
    set_bits_stream(position, 38, 4, 0u);
    set_bits_stream(position, 50, 10, 120u);
    lon_raw = (int32_t)lrint(12.4964 * 600000.0);
    lat_raw = (int32_t)lrint(41.9028 * 600000.0);
    set_bits_stream(position, 61, 28, encode_signed(lon_raw, 28));
    set_bits_stream(position, 89, 27, encode_signed(lat_raw, 27));
    set_bits_stream(position, 116, 12, 1875u);
    set_bits_stream(position, 128, 9, 188u);
    set_bits_stream(position, 137, 6, 30u);

    set_bits_stream(voyage, 0, 6, 5u);
    set_bits_stream(voyage, 8, 30, 247320162u);
    set_bits_stream(voyage, 40, 30, 9876543u);
    set_text_stream(voyage, 70, 7, "IABCD");
    set_text_stream(voyage, 112, 20, "CODEX TEST VESSEL");
    set_bits_stream(voyage, 232, 8, 70u);
    set_bits_stream(voyage, 240, 9, 80u);
    set_bits_stream(voyage, 249, 9, 20u);
    set_bits_stream(voyage, 258, 6, 8u);
    set_bits_stream(voyage, 264, 6, 8u);
    set_bits_stream(voyage, 274, 4, 8u);
    set_bits_stream(voyage, 278, 5, 15u);
    set_bits_stream(voyage, 283, 5, 14u);
    set_bits_stream(voyage, 288, 6, 30u);
    set_bits_stream(voyage, 294, 8, 65u);
    set_text_stream(voyage, 302, 20, "CIVITAVECCHIA");

    set_bits_stream(aid, 0, 6, 21u);
    set_bits_stream(aid, 8, 30, 992471001u);
    set_bits_stream(aid, 38, 5, 1u);
    set_text_stream(aid, 43, 20, "TEST LIGHTHOUSE");
    set_bits_stream(aid, 163, 1, 1u);
    set_bits_stream(aid, 164, 28, encode_signed(lon_raw, 28));
    set_bits_stream(aid, 192, 27, encode_signed(lat_raw, 27));
    set_bits_stream(aid, 253, 6, 30u);

    set_bits_stream(aid_single_slot, 0, 6, 28u);
    set_bits_stream(aid_single_slot, 8, 30, 992471002u);
    set_bits_stream(aid_single_slot, 38, 6, 30u);
    set_bits_stream(aid_single_slot, 44, 28, encode_signed(lon_raw, 28));
    set_bits_stream(aid_single_slot, 72, 27, encode_signed(lat_raw, 27));
    set_bits_stream(aid_single_slot, 101, 3, 1u);
    set_bits_stream(aid_single_slot, 104, 7, 5u);
    set_bits_stream(aid_single_slot, 111, 17, 4710u);
    set_bits_stream(aid_single_slot, 128, 4, 1u);
    set_bits_stream(aid_single_slot, 132, 9, 12u);
    set_bits_stream(aid_single_slot, 141, 11, 240u);
    set_bits_stream(aid_single_slot, 153, 1, 1u);
    set_bits_stream(aid_single_slot, 167, 1, 1u);

    fprintf(output_diagnostics(),
            "[AIS] offline protocol test (NRZI + HDLC + bit-stuffing + CRC)\n");
    ais_process_demod_bit(ctx, (uint8_t)nrzi_level);
    test_feed_flag(ctx, &nrzi_level);
    test_feed_hdlc_frame(ctx, position, sizeof(position), &nrzi_level);
    test_feed_hdlc_frame(ctx, voyage, sizeof(voyage), &nrzi_level);
    test_feed_hdlc_frame(ctx, aid, sizeof(aid), &nrzi_level);
    test_feed_hdlc_frame(ctx, aid_single_slot, sizeof(aid_single_slot), &nrzi_level);
    nmea_ok = test_nmea_from_payload(position, sizeof(position));

    fprintf(output_diagnostics(),
            "[AIS] test_result=%s valid=%llu expected=4 nmea=%s\n",
            ctx->frames_valid - valid_before == 4u && nmea_ok ? "PASS" : "FAIL",
            (unsigned long long)(ctx->frames_valid - valid_before),
            nmea_ok ? "PASS" : "FAIL");
    return ctx->frames_valid - valid_before == 4u && nmea_ok;
}
