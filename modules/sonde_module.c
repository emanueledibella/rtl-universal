#include "header/sonde_module.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dashboard.h"
#include "output.h"

#define RS41_INPUT_FS 240000
#define RS41_DEMOD_FS 48000
#define RS41_SYMBOL_RATE 4800u
#define RS41_HEADER_LEN 8u
#define RS41_MIN_FRAME_LEN 320u

static const uint8_t rs41_raw_sync[RS41_HEADER_LEN] = {
    0x10u, 0xB6u, 0xCAu, 0x11u, 0x22u, 0x96u, 0x12u, 0xF8u
};

static const uint8_t rs41_decoded_sync[RS41_HEADER_LEN] = {
    0x86u, 0x35u, 0xF4u, 0x40u, 0x93u, 0xDFu, 0x1Au, 0x60u
};

static const uint8_t rs41_whitening[64] = {
    0x96u, 0x83u, 0x3Eu, 0x51u, 0xB1u, 0x49u, 0x08u, 0x98u,
    0x32u, 0x05u, 0x59u, 0x0Eu, 0xF9u, 0x44u, 0xC6u, 0x26u,
    0x21u, 0x60u, 0xC2u, 0xEAu, 0x79u, 0x5Du, 0x6Du, 0xA1u,
    0x54u, 0x69u, 0x47u, 0x0Cu, 0xDCu, 0xE8u, 0x5Cu, 0xF1u,
    0xF7u, 0x76u, 0x82u, 0x7Fu, 0x07u, 0x99u, 0xA2u, 0x2Cu,
    0x93u, 0x7Cu, 0x30u, 0x63u, 0xF5u, 0x10u, 0x2Eu, 0x61u,
    0xD0u, 0xBCu, 0xB4u, 0xB6u, 0x06u, 0xAAu, 0xF4u, 0x23u,
    0x78u, 0x6Eu, 0x3Bu, 0xAEu, 0xBFu, 0x7Bu, 0x4Cu, 0xC1u
};

typedef struct {
    char serial[9];
    uint16_t frame_number;
    double battery_v;
    int has_status;
    double latitude;
    double longitude;
    double altitude_m;
    double speed_mps;
    double heading_deg;
    double climb_mps;
    unsigned int satellites;
    int has_position;
    unsigned int valid_blocks;
} rs41_observation_t;

static uint8_t rs_exp[512];
static uint8_t rs_log[256];
static uint8_t rs_generator[25];
static int rs_tables_ready;

static void rs_init_tables(void) {
    uint16_t value = 1u;
    uint8_t polynomial[25] = { 1u };
    unsigned int degree = 0u;
    if (rs_tables_ready) return;
    memset(rs_log, 0, sizeof(rs_log));
    for (unsigned int i = 0u; i < 255u; i++) {
        rs_exp[i] = (uint8_t)value;
        rs_log[(uint8_t)value] = (uint8_t)i;
        value <<= 1u;
        if ((value & 0x100u) != 0u) value ^= 0x11Du;
    }
    for (unsigned int i = 255u; i < 512u; i++) rs_exp[i] = rs_exp[i - 255u];
    for (unsigned int root = 0u; root < 24u; root++) {
        uint8_t next[25] = { 0u };
        for (unsigned int i = 0u; i <= degree; i++) {
            if (polynomial[i] != 0u) {
                next[i] ^= rs_exp[rs_log[polynomial[i]] + root];
            }
            next[i + 1u] ^= polynomial[i];
        }
        memcpy(polynomial, next, sizeof(polynomial));
        degree++;
    }
    memcpy(rs_generator, polynomial, sizeof(rs_generator));
    rs_tables_ready = 1;
}

static uint8_t rs_mul(uint8_t left, uint8_t right) {
    if (left == 0u || right == 0u) return 0u;
    return rs_exp[(unsigned int)rs_log[left] + rs_log[right]];
}

static uint8_t rs_div(uint8_t numerator, uint8_t denominator) {
    int exponent;
    if (numerator == 0u) return 0u;
    if (denominator == 0u) return 0u;
    exponent = (int)rs_log[numerator] - (int)rs_log[denominator];
    if (exponent < 0) exponent += 255;
    return rs_exp[(unsigned int)exponent];
}

static uint8_t rs_poly_eval(const uint8_t *polynomial, unsigned int degree,
                            uint8_t x) {
    uint8_t result = polynomial[degree];
    while (degree > 0u) {
        degree--;
        result = (uint8_t)(rs_mul(result, x) ^ polynomial[degree]);
    }
    return result;
}

static int rs_syndromes(const uint8_t codeword[255], uint8_t syndrome[24]) {
    int nonzero = 0;
    for (unsigned int root = 0u; root < 24u; root++) {
        syndrome[root] = rs_poly_eval(codeword, 254u, rs_exp[root]);
        if (syndrome[root] != 0u) nonzero = 1;
    }
    return nonzero;
}

/* Correct one conventional RS(255,231) codeword over GF(256), with the
 * 24 parity coefficients stored first as used by RS41. */
static int rs_correct_codeword(uint8_t codeword[255]) {
    uint8_t syndrome[24];
    uint8_t locator[25] = { 1u };
    uint8_t previous[25] = { 1u };
    uint8_t saved[25];
    uint8_t omega[24] = { 0u };
    unsigned int locator_degree = 0u;
    unsigned int shift = 1u;
    uint8_t discrepancy_scale = 1u;
    unsigned int positions[12];
    unsigned int found = 0u;

    rs_init_tables();
    if (!rs_syndromes(codeword, syndrome)) return 0;
    for (unsigned int n = 0u; n < 24u; n++) {
        uint8_t discrepancy = syndrome[n];
        for (unsigned int i = 1u; i <= locator_degree && i <= n; i++) {
            discrepancy ^= rs_mul(locator[i], syndrome[n - i]);
        }
        if (discrepancy == 0u) {
            shift++;
            continue;
        }
        memcpy(saved, locator, sizeof(saved));
        {
            uint8_t factor = rs_div(discrepancy, discrepancy_scale);
            for (unsigned int i = 0u; i + shift < 25u; i++) {
                locator[i + shift] ^= rs_mul(factor, previous[i]);
            }
        }
        if (2u * locator_degree <= n) {
            locator_degree = n + 1u - locator_degree;
            memcpy(previous, saved, sizeof(previous));
            discrepancy_scale = discrepancy;
            shift = 1u;
        } else {
            shift++;
        }
    }
    if (locator_degree == 0u || locator_degree > 12u) return -1;

    for (unsigned int position = 0u; position < 255u; position++) {
        unsigned int exponent = position == 0u ? 0u : 255u - position;
        if (rs_poly_eval(locator, locator_degree, rs_exp[exponent]) == 0u) {
            if (found >= 12u) return -1;
            positions[found++] = position;
        }
    }
    if (found != locator_degree) return -1;

    for (unsigned int i = 0u; i < 24u; i++) {
        for (unsigned int j = 0u; j <= locator_degree && i + j < 24u; j++) {
            omega[i + j] ^= rs_mul(syndrome[i], locator[j]);
        }
    }
    for (unsigned int error = 0u; error < found; error++) {
        unsigned int position = positions[error];
        unsigned int exponent = position == 0u ? 0u : 255u - position;
        uint8_t x = rs_exp[exponent];
        uint8_t derivative = 0u;
        uint8_t numerator = rs_poly_eval(omega, 23u, x);
        uint8_t inverse_x = position == 0u ? 1u : rs_exp[position];
        for (unsigned int power = 1u; power <= locator_degree; power += 2u) {
            uint8_t term = locator[power];
            if (power > 1u && term != 0u) {
                term = rs_mul(term, rs_exp[(exponent * (power - 1u)) % 255u]);
            }
            derivative ^= term;
        }
        if (derivative == 0u) return -1;
        codeword[position] ^= rs_mul(inverse_x, rs_div(numerator, derivative));
    }
    return rs_syndromes(codeword, syndrome) ? -1 : (int)found;
}

static int rs41_correct_frame(uint8_t *frame, size_t len, uint64_t *corrected) {
    uint8_t codeword[2][255];
    size_t effective_len;
    int result[2];
    if (!frame || len < RS41_MIN_FRAME_LEN) return -1;
    effective_len = frame[56] == 0x0Fu ? RS41_MIN_FRAME_LEN : SONDE_RS41_FRAME_LEN;
    for (unsigned int stream = 0u; stream < 2u; stream++) {
        for (unsigned int i = 0u; i < 24u; i++) {
            codeword[stream][i] = frame[8u + stream * 24u + i];
        }
        for (unsigned int i = 0u; i < 231u; i++) {
            size_t position = 56u + i * 2u + stream;
            codeword[stream][24u + i] = position < len && position < effective_len
                                            ? frame[position] : 0u;
        }
        result[stream] = rs_correct_codeword(codeword[stream]);
        if (result[stream] >= 0) {
            for (unsigned int i = 0u; i < 24u; i++) {
                frame[8u + stream * 24u + i] = codeword[stream][i];
            }
            for (unsigned int i = 0u; i < 231u; i++) {
                size_t position = 56u + i * 2u + stream;
                if (position < len && position < effective_len) {
                    frame[position] = codeword[stream][24u + i];
                }
            }
            if (corrected) *corrected += (uint64_t)result[stream];
        }
    }
    return result[0] >= 0 && result[1] >= 0 ? result[0] + result[1] : -1;
}

static void rs_encode_codeword(uint8_t codeword[255]) {
    uint8_t remainder[255];
    rs_init_tables();
    memcpy(remainder, codeword, sizeof(remainder));
    memset(remainder, 0, 24u);
    for (int degree = 254; degree >= 24; degree--) {
        uint8_t factor = remainder[degree];
        if (factor == 0u) continue;
        for (unsigned int i = 0u; i <= 24u; i++) {
            remainder[(unsigned int)degree - 24u + i]
                ^= rs_mul(factor, rs_generator[i]);
        }
    }
    memcpy(codeword, remainder, 24u);
}

static void rs41_encode_test_frame(uint8_t frame[SONDE_RS41_FRAME_LEN]) {
    uint8_t codeword[2][255];
    for (unsigned int stream = 0u; stream < 2u; stream++) {
        memset(codeword[stream], 0, sizeof(codeword[stream]));
        for (unsigned int i = 0u; i < 231u; i++) {
            codeword[stream][24u + i] = frame[56u + i * 2u + stream];
        }
        rs_encode_codeword(codeword[stream]);
        for (unsigned int i = 0u; i < 24u; i++) {
            frame[8u + stream * 24u + i] = codeword[stream][i];
        }
    }
}

static uint16_t read_u16_le(const uint8_t *data) {
    return (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8u);
}

static int16_t read_i16_le(const uint8_t *data) {
    return (int16_t)read_u16_le(data);
}

static int32_t read_i32_le(const uint8_t *data) {
    uint32_t value = (uint32_t)data[0]
                     | ((uint32_t)data[1] << 8u)
                     | ((uint32_t)data[2] << 16u)
                     | ((uint32_t)data[3] << 24u);
    return (int32_t)value;
}

static void write_u16_le(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8u);
}

static void write_i16_le(uint8_t *data, int16_t value) {
    write_u16_le(data, (uint16_t)value);
}

static void write_i32_le(uint8_t *data, int32_t value) {
    uint32_t uvalue = (uint32_t)value;
    data[0] = (uint8_t)uvalue;
    data[1] = (uint8_t)(uvalue >> 8u);
    data[2] = (uint8_t)(uvalue >> 16u);
    data[3] = (uint8_t)(uvalue >> 24u);
}

static uint16_t rs41_crc16(const uint8_t *data, size_t len) {
    uint16_t remainder = 0xFFFFu;
    for (size_t i = 0u; i < len; i++) {
        remainder ^= (uint16_t)((uint16_t)data[i] << 8u);
        for (unsigned int bit = 0u; bit < 8u; bit++) {
            remainder = (remainder & 0x8000u) != 0u
                            ? (uint16_t)((remainder << 1u) ^ 0x1021u)
                            : (uint16_t)(remainder << 1u);
        }
    }
    return remainder;
}

static uint64_t make_sync_pattern(void) {
    uint64_t pattern = 0u;
    for (size_t byte = 0u; byte < sizeof(rs41_raw_sync); byte++) {
        for (unsigned int bit = 0u; bit < 8u; bit++) {
            pattern = (pattern << 1u) | ((rs41_raw_sync[byte] >> bit) & 1u);
        }
    }
    return pattern;
}

static unsigned int bit_errors64(uint64_t value) {
#if defined(__clang__) || defined(__GNUC__)
    return (unsigned int)__builtin_popcountll(value);
#else
    unsigned int count = 0u;
    while (value != 0u) {
        value &= value - 1u;
        count++;
    }
    return count;
#endif
}

static void ecef_to_geodetic(const double xyz[3], double *latitude,
                             double *longitude, double *altitude) {
    const double a = 6378137.0;
    const double b = 6356752.31424518;
    const double e2 = (a * a - b * b) / (a * a);
    const double ep2 = (a * a - b * b) / (b * b);
    double p = hypot(xyz[0], xyz[1]);
    double theta = atan2(xyz[2] * a, p * b);
    double st = sin(theta);
    double ct = cos(theta);
    double phi = atan2(xyz[2] + ep2 * b * st * st * st,
                       p - e2 * a * ct * ct * ct);
    double radius = a / sqrt(1.0 - e2 * sin(phi) * sin(phi));

    *longitude = atan2(xyz[1], xyz[0]) * 180.0 / M_PI;
    *latitude = phi * 180.0 / M_PI;
    *altitude = fabs(cos(phi)) > 1e-9 ? p / cos(phi) - radius
                                      : fabs(xyz[2]) - b;
}

static int decode_gps3(const uint8_t *payload, size_t len,
                       rs41_observation_t *observation) {
    double xyz[3];
    double velocity[3];
    double phi;
    double lambda;
    double north;
    double east;
    double up;

    if (len < 21u || !observation) return 0;
    for (unsigned int axis = 0u; axis < 3u; axis++) {
        xyz[axis] = (double)read_i32_le(payload + axis * 4u) / 100.0;
        velocity[axis] = (double)read_i16_le(payload + 12u + axis * 2u) / 100.0;
    }
    ecef_to_geodetic(xyz, &observation->latitude, &observation->longitude,
                     &observation->altitude_m);
    if (!isfinite(observation->latitude) || !isfinite(observation->longitude)
        || !isfinite(observation->altitude_m)
        || observation->altitude_m < -1000.0 || observation->altitude_m > 80000.0) {
        return 0;
    }

    phi = observation->latitude * M_PI / 180.0;
    lambda = observation->longitude * M_PI / 180.0;
    north = -velocity[0] * sin(phi) * cos(lambda)
            - velocity[1] * sin(phi) * sin(lambda) + velocity[2] * cos(phi);
    east = -velocity[0] * sin(lambda) + velocity[1] * cos(lambda);
    up = velocity[0] * cos(phi) * cos(lambda)
         + velocity[1] * cos(phi) * sin(lambda) + velocity[2] * sin(phi);
    observation->speed_mps = hypot(north, east);
    observation->heading_deg = atan2(east, north) * 180.0 / M_PI;
    if (observation->heading_deg < 0.0) observation->heading_deg += 360.0;
    observation->climb_mps = up;
    observation->satellites = payload[20];
    observation->has_position = 1;
    return 1;
}

static void emit_observation(const rs41_observation_t *observation) {
    output_format_t format;
    if (!observation) return;
    format = output_get_format();
    if (format == OUTPUT_FORMAT_QUIET) return;
    if (format == OUTPUT_FORMAT_DASHBOARD) {
        dashboard_sonde_update_t update;
        memset(&update, 0, sizeof(update));
        update.serial = observation->serial;
        update.frame_number = observation->frame_number;
        update.battery_v = observation->battery_v;
        update.has_battery = observation->has_status;
        update.latitude = observation->latitude;
        update.longitude = observation->longitude;
        update.altitude_m = observation->altitude_m;
        update.speed_mps = observation->speed_mps;
        update.heading_deg = observation->heading_deg;
        update.climb_mps = observation->climb_mps;
        update.satellites = observation->satellites;
        update.has_position = observation->has_position;
        dashboard_update_sonde(&update);
    } else if (format == OUTPUT_FORMAT_JSON) {
        printf("{\"protocol\":\"rs41\",\"serial\":\"%s\",\"frame\":%u",
               observation->serial, observation->frame_number);
        if (observation->has_status) printf(",\"battery_v\":%.1f", observation->battery_v);
        if (observation->has_position) {
            printf(",\"latitude\":%.6f,\"longitude\":%.6f,\"altitude_m\":%.1f,"
                   "\"speed_mps\":%.2f,\"heading_deg\":%.1f,\"climb_mps\":%.2f,"
                   "\"satellites\":%u",
                   observation->latitude, observation->longitude,
                   observation->altitude_m, observation->speed_mps,
                   observation->heading_deg, observation->climb_mps,
                   observation->satellites);
        }
        printf(",\"valid_blocks\":%u}\n", observation->valid_blocks);
    } else if (format == OUTPUT_FORMAT_CSV) {
        printf("rs41,%s,%u,", observation->serial, observation->frame_number);
        if (observation->has_status) printf("%.1f", observation->battery_v);
        printf(",");
        if (observation->has_position) {
            printf("%.6f,%.6f,%.1f,%.2f,%.1f,%.2f,%u",
                   observation->latitude, observation->longitude,
                   observation->altitude_m, observation->speed_mps,
                   observation->heading_deg, observation->climb_mps,
                   observation->satellites);
        } else {
            printf(",,,,,,");
        }
        printf(",%u\n", observation->valid_blocks);
    } else {
        printf("[RS41] serial=%s frame=%u", observation->serial, observation->frame_number);
        if (observation->has_status) printf(" battery=%.1fV", observation->battery_v);
        if (observation->has_position) {
            printf(" lat=%.6f lon=%.6f alt=%.1fm speed=%.2fm/s heading=%.1fdeg "
                   "climb=%.2fm/s sats=%u",
                   observation->latitude, observation->longitude,
                   observation->altitude_m, observation->speed_mps,
                   observation->heading_deg, observation->climb_mps,
                   observation->satellites);
        }
        printf(" blocks=%u\n", observation->valid_blocks);
    }
}

static int decode_frame(sonde_module_t *ctx, uint8_t *frame, size_t len) {
    rs41_observation_t observation;
    size_t position = 0x39u;
    unsigned int invalid_blocks = 0u;

    if (!ctx || !frame || len < RS41_MIN_FRAME_LEN
        || memcmp(frame, rs41_decoded_sync, sizeof(rs41_decoded_sync)) != 0) {
        if (ctx) ctx->rejected_frames++;
        return 0;
    }
    memset(&observation, 0, sizeof(observation));
    if (rs41_correct_frame(frame, len, &ctx->fec_corrected_symbols) < 0) {
        ctx->fec_failures++;
    }
    while (position + 4u <= len) {
        uint8_t type = frame[position];
        size_t payload_len = frame[position + 1u];
        size_t end = position + 2u + payload_len + 2u;
        uint16_t received_crc;
        uint16_t computed_crc;

        if (type == 0u || type == 0xFFu) {
            position++;
            continue;
        }
        if (end > len) break;
        received_crc = read_u16_le(frame + position + 2u + payload_len);
        computed_crc = rs41_crc16(frame + position + 2u, payload_len);
        if (received_crc == computed_crc) {
            const uint8_t *payload = frame + position + 2u;
            observation.valid_blocks++;
            if (type == 0x79u && payload_len >= 11u) {
                observation.frame_number = read_u16_le(payload);
                memcpy(observation.serial, payload + 2u, 8u);
                observation.serial[8] = '\0';
                for (size_t i = 0u; i < 8u; i++) {
                    if (!isalnum((unsigned char)observation.serial[i])
                        && observation.serial[i] != '-' && observation.serial[i] != '_') {
                        observation.serial[i] = '?';
                    }
                }
                observation.battery_v = (double)payload[10] / 10.0;
                observation.has_status = 1;
            } else if (type == 0x7Bu) {
                (void)decode_gps3(payload, payload_len, &observation);
            }
        } else {
            invalid_blocks++;
        }
        position = end;
    }

    if (invalid_blocks > 0u) ctx->crc_errors += invalid_blocks;
    if (observation.valid_blocks == 0u
        || (!observation.has_status && !observation.has_position)) {
        ctx->rejected_frames++;
        return 0;
    }
    if (!observation.has_status) strcpy(observation.serial, "UNKNOWN");
    ctx->frames_valid++;
    emit_observation(&observation);
    return 1;
}

static void process_raw_frame(sonde_module_t *ctx) {
    uint8_t decoded[SONDE_RS41_FRAME_LEN];
    for (size_t i = 0u; i < sizeof(decoded); i++) {
        decoded[i] = (uint8_t)(ctx->raw_frame[i] ^ rs41_whitening[i % 64u]);
    }
    (void)decode_frame(ctx, decoded, sizeof(decoded));
}

static void sonde_on_bit(void *user, uint8_t input_bit) {
    sonde_module_t *ctx = (sonde_module_t *)user;
    uint8_t bit = input_bit & 1u;
    if (!ctx) return;
    ctx->demod_bits++;

    if (!ctx->capturing) {
        unsigned int normal_errors;
        unsigned int inverted_errors;
        ctx->sync_window = (ctx->sync_window << 1u) | bit;
        normal_errors = bit_errors64(ctx->sync_window ^ ctx->sync_pattern);
        inverted_errors = bit_errors64(ctx->sync_window ^ ~ctx->sync_pattern);
        if (normal_errors <= 2u || inverted_errors <= 2u) {
            ctx->capturing = 1;
            ctx->inverted = inverted_errors < normal_errors;
            memcpy(ctx->raw_frame, rs41_raw_sync, sizeof(rs41_raw_sync));
            ctx->raw_index = RS41_HEADER_LEN;
            ctx->raw_bit = 0u;
            ctx->frame_candidates++;
        }
        return;
    }

    if (ctx->inverted) bit ^= 1u;
    if (ctx->raw_bit == 0u) ctx->raw_frame[ctx->raw_index] = 0u;
    ctx->raw_frame[ctx->raw_index] |= (uint8_t)(bit << ctx->raw_bit);
    ctx->raw_bit++;
    if (ctx->raw_bit == 8u) {
        ctx->raw_bit = 0u;
        ctx->raw_index++;
        if (ctx->raw_index == SONDE_RS41_FRAME_LEN) {
            process_raw_frame(ctx);
            ctx->capturing = 0;
            ctx->sync_window = 0u;
        }
    }
}

void sonde_module_reset(sonde_module_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->sync_pattern = make_sync_pattern();
}

int sonde_module_init(sonde_module_t *ctx, const demod_config_t *cfg) {
    (void)cfg;
    sonde_module_reset(ctx);
    return ctx != NULL;
}

void sonde_module_get_demod_config(sonde_module_t *ctx, demod_config_t *cfg) {
    (void)ctx;
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->kind = DEMOD_KIND_GMSK;
    cfg->input_fs = RS41_INPUT_FS;
    cfg->output_fs = RS41_DEMOD_FS;
    cfg->u.gmsk.symbol_rate = RS41_SYMBOL_RATE;
    cfg->u.gmsk.m = 3u;
    cfg->u.gmsk.bt = 0.5f;
    cfg->u.gmsk.channel_count = 1u;
    cfg->u.gmsk.channel_offset_hz[0] = 0.0f;
}

demod_output_t sonde_module_get_demod_output(sonde_module_t *ctx) {
    demod_output_t output;
    memset(&output, 0, sizeof(output));
    output.on_bit = sonde_on_bit;
    output.user = ctx;
    return output;
}

void sonde_module_flush(sonde_module_t *ctx) {
    if (!ctx) return;
    if (!output_is_dashboard()) {
        fprintf(output_diagnostics(),
                "[RS41] bits=%llu candidates=%llu valid=%llu crc_errors=%llu rejected=%llu fec_corrected=%llu fec_failures=%llu\n",
                (unsigned long long)ctx->demod_bits,
                (unsigned long long)ctx->frame_candidates,
                (unsigned long long)ctx->frames_valid,
                (unsigned long long)ctx->crc_errors,
                (unsigned long long)ctx->rejected_frames,
                (unsigned long long)ctx->fec_corrected_symbols,
                (unsigned long long)ctx->fec_failures);
    }
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int sonde_module_decode_hex(sonde_module_t *ctx, const char *hex) {
    uint8_t frame[SONDE_RS41_FRAME_LEN];
    size_t count = 0u;
    int high = -1;
    if (!ctx || !hex) return 0;
    while (*hex) {
        int nibble;
        if (isspace((unsigned char)*hex) || *hex == ':' || *hex == '-') {
            hex++;
            continue;
        }
        nibble = hex_nibble(*hex++);
        if (nibble < 0) return 0;
        if (high < 0) high = nibble;
        else {
            if (count >= sizeof(frame)) return 0;
            frame[count++] = (uint8_t)((high << 4) | nibble);
            high = -1;
        }
    }
    if (high >= 0 || count < RS41_MIN_FRAME_LEN) return 0;
    return decode_frame(ctx, frame, count);
}

static void geodetic_to_ecef(double latitude, double longitude, double altitude,
                             int32_t result_cm[3]) {
    const double a = 6378137.0;
    const double e2 = 6.6943799901413165e-3;
    double phi = latitude * M_PI / 180.0;
    double lambda = longitude * M_PI / 180.0;
    double radius = a / sqrt(1.0 - e2 * sin(phi) * sin(phi));
    double x = (radius + altitude) * cos(phi) * cos(lambda);
    double y = (radius + altitude) * cos(phi) * sin(lambda);
    double z = (radius * (1.0 - e2) + altitude) * sin(phi);
    result_cm[0] = (int32_t)llround(x * 100.0);
    result_cm[1] = (int32_t)llround(y * 100.0);
    result_cm[2] = (int32_t)llround(z * 100.0);
}

static void add_test_block(uint8_t *frame, size_t position, uint8_t type,
                           const uint8_t *payload, size_t payload_len) {
    uint16_t crc;
    frame[position] = type;
    frame[position + 1u] = (uint8_t)payload_len;
    memcpy(frame + position + 2u, payload, payload_len);
    crc = rs41_crc16(payload, payload_len);
    write_u16_le(frame + position + 2u + payload_len, crc);
}

int sonde_module_run_test(sonde_module_t *ctx) {
    uint8_t frame[SONDE_RS41_FRAME_LEN];
    uint8_t status[40];
    uint8_t gps3[21];
    int32_t xyz[3];
    uint8_t raw[SONDE_RS41_FRAME_LEN];
    uint64_t before;
    int ok;

    if (!ctx) return 0;
    memset(frame, 0, sizeof(frame));
    memset(status, 0, sizeof(status));
    memset(gps3, 0, sizeof(gps3));
    memcpy(frame, rs41_decoded_sync, sizeof(rs41_decoded_sync));
    frame[56] = 0x0Fu;
    write_u16_le(status, 4242u);
    memcpy(status + 2u, "N1234567", 8u);
    status[10] = 29u;
    add_test_block(frame, 0x39u, 0x79u, status, sizeof(status));

    geodetic_to_ecef(41.9028, 12.4964, 12345.0, xyz);
    for (unsigned int axis = 0u; axis < 3u; axis++) {
        write_i32_le(gps3 + axis * 4u, xyz[axis]);
    }
    write_i16_le(gps3 + 12u, 320);
    write_i16_le(gps3 + 14u, 180);
    write_i16_le(gps3 + 16u, 450);
    gps3[20] = 11u;
    add_test_block(frame, 0x112u, 0x7Bu, gps3, sizeof(gps3));

    rs41_encode_test_frame(frame);
    frame[0x50u] ^= 0x41u;
    frame[0x81u] ^= 0xA2u;
    frame[0x118u] ^= 0x07u;
    frame[0x12Du] ^= 0x80u;

    for (size_t i = 0u; i < sizeof(raw); i++) {
        raw[i] = (uint8_t)(frame[i] ^ rs41_whitening[i % 64u]);
    }
    before = ctx->frames_valid;
    for (unsigned int noise = 0u; noise < 19u; noise++) {
        sonde_on_bit(ctx, (uint8_t)((noise * 5u + 1u) & 1u));
    }
    for (size_t byte = 0u; byte < sizeof(raw); byte++) {
        for (unsigned int bit = 0u; bit < 8u; bit++) {
            sonde_on_bit(ctx, (uint8_t)((raw[byte] >> bit) & 1u));
        }
    }
    ok = ctx->frames_valid == before + 1u;
    fprintf(output_diagnostics(), "[RS41] test_result=%s decoded=%llu expected=1\n",
            ok && ctx->frames_valid == before + 1u ? "PASS" : "FAIL",
            (unsigned long long)(ctx->frames_valid - before));
    return ok && ctx->frames_valid == before + 1u;
}
