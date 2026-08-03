#include "header/adsb_protocol.h"
#include "dashboard.h"
#include "output.h"
#include "utility.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define ADSB_ME_BITS 56u
#define ADSB_MAX_AIRCRAFT 2048u
#define ADSB_CPR_SCALE 131072.0
#define ADSB_CPR_PAIR_MAX_AGE 10
#define MODE_S_POLY 0xFFF409u
#define PI_D 3.14159265358979323846

typedef struct {
    uint32_t icao;
    char callsign[9];
    const char *category;

    int altitude_ft;
    bool has_altitude;
    bool altitude_is_gnss;

    double latitude;
    double longitude;
    bool has_position;

    double ground_speed_kt;
    double track_deg;
    bool has_ground_speed;
    bool has_track;

    int vertical_rate_fpm;
    bool has_vertical_rate;
    bool vertical_rate_is_gnss;

    double airspeed_kt;
    bool has_airspeed;
    bool airspeed_is_true;

    uint16_t squawk;
    bool has_squawk;
    uint8_t emergency_state;

    uint8_t adsb_version;
    bool has_adsb_version;

    double cpr_lat[2];
    double cpr_lon[2];
    time_t cpr_time[2];
    bool cpr_valid[2];
    bool cpr_surface[2];
    time_t last_seen;
} aircraft_t;

typedef enum {
    ADSB_TC_UNKNOWN = 0,
    ADSB_TC_IDENTIFICATION,
    ADSB_TC_SURFACE_POSITION,
    ADSB_TC_AIRBORNE_POSITION_BARO,
    ADSB_TC_AIRBORNE_VELOCITY,
    ADSB_TC_AIRBORNE_POSITION_GNSS,
    ADSB_TC_RESERVED,
    ADSB_TC_AIRCRAFT_STATUS,
    ADSB_TC_TARGET_STATE,
    ADSB_TC_OPERATIONAL_STATUS
} adsb_tc_category_t;

static aircraft_t g_aircraft[ADSB_MAX_AIRCRAFT];
static size_t g_aircraft_count;
static bool g_have_reference;
static double g_reference_lat;
static double g_reference_lon;

static uint32_t me_get_u32(uint64_t me, unsigned int start_bit,
                           unsigned int bit_len) {
    uint64_t mask;
    unsigned int shift;

    if (bit_len == 0u || bit_len > 32u) return 0u;
    if (start_bit + bit_len > ADSB_ME_BITS) return 0u;
    shift = ADSB_ME_BITS - start_bit - bit_len;
    mask = (((uint64_t)1u << bit_len) - 1u);
    return (uint32_t)((me >> shift) & mask);
}

static adsb_tc_category_t tc_category(uint8_t tc) {
    if (tc >= 1u && tc <= 4u) return ADSB_TC_IDENTIFICATION;
    if (tc >= 5u && tc <= 8u) return ADSB_TC_SURFACE_POSITION;
    if (tc >= 9u && tc <= 18u) return ADSB_TC_AIRBORNE_POSITION_BARO;
    if (tc == 19u) return ADSB_TC_AIRBORNE_VELOCITY;
    if (tc >= 20u && tc <= 22u) return ADSB_TC_AIRBORNE_POSITION_GNSS;
    if (tc >= 23u && tc <= 27u) return ADSB_TC_RESERVED;
    if (tc == 28u) return ADSB_TC_AIRCRAFT_STATUS;
    if (tc == 29u) return ADSB_TC_TARGET_STATE;
    if (tc == 31u) return ADSB_TC_OPERATIONAL_STATUS;
    return ADSB_TC_UNKNOWN;
}

static aircraft_t *find_aircraft(uint32_t icao) {
    for (size_t i = 0; i < g_aircraft_count; i++) {
        if (g_aircraft[i].icao == icao) return &g_aircraft[i];
    }
    return NULL;
}

static aircraft_t *get_aircraft(uint32_t icao) {
    aircraft_t *entry = find_aircraft(icao);
    if (entry) return entry;
    if (g_aircraft_count < ADSB_MAX_AIRCRAFT) {
        entry = &g_aircraft[g_aircraft_count++];
    } else {
        size_t oldest = 0u;
        for (size_t i = 1u; i < g_aircraft_count; i++) {
            if (g_aircraft[i].last_seen < g_aircraft[oldest].last_seen) oldest = i;
        }
        entry = &g_aircraft[oldest];
    }
    memset(entry, 0, sizeof(*entry));
    entry->icao = icao;
    entry->category = "Unknown";
    return entry;
}

static void print_aircraft(const aircraft_t *entry) {
    char timestamp[32] = "time-unavailable";
    struct tm local_tm;
    time_t now;

    if (!entry) return;
    if (output_get_format() == OUTPUT_FORMAT_QUIET || output_is_adsb_raw()) return;
    if (output_is_dashboard()) {
        dashboard_adsb_update_t update;
        memset(&update, 0, sizeof(update));
        update.icao = entry->icao;
        update.callsign = entry->callsign;
        update.category = entry->category;
        update.altitude_ft = entry->altitude_ft;
        update.has_altitude = entry->has_altitude;
        update.latitude = entry->latitude;
        update.longitude = entry->longitude;
        update.has_position = entry->has_position;
        update.speed_kt = entry->has_ground_speed ? entry->ground_speed_kt
                                                  : entry->airspeed_kt;
        update.has_speed = entry->has_ground_speed || entry->has_airspeed;
        update.track_deg = entry->track_deg;
        update.has_track = entry->has_track;
        update.vertical_rate_fpm = entry->vertical_rate_fpm;
        update.has_vertical_rate = entry->has_vertical_rate;
        update.adsb_version = entry->adsb_version;
        update.has_adsb_version = entry->has_adsb_version;
        dashboard_update_adsb(&update);
        return;
    }
    now = time(NULL);
    if (localtime_r(&now, &local_tm)) {
        (void)strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &local_tm);
    }

    if (output_get_format() == OUTPUT_FORMAT_JSON) {
        printf("{\"protocol\":\"adsb\",\"timestamp\":\"%s\",\"icao\":\"%06X\"",
               timestamp, entry->icao);
        if (entry->callsign[0]) printf(",\"callsign\":\"%s\"", entry->callsign);
        if (entry->category && strcmp(entry->category, "Unknown") != 0) {
            printf(",\"category\":\"%s\"", entry->category);
        }
        if (entry->has_altitude) {
            printf(",\"altitude_ft\":%d,\"altitude_source\":\"%s\"",
                   entry->altitude_ft, entry->altitude_is_gnss ? "gnss" : "barometric");
        }
        if (entry->has_position) {
            printf(",\"latitude\":%.6f,\"longitude\":%.6f",
                   entry->latitude, entry->longitude);
        }
        if (entry->has_ground_speed) printf(",\"ground_speed_kt\":%.1f", entry->ground_speed_kt);
        if (entry->has_track) printf(",\"track_deg\":%.1f", entry->track_deg);
        if (entry->has_vertical_rate) printf(",\"vertical_rate_fpm\":%d", entry->vertical_rate_fpm);
        if (entry->has_airspeed) printf(",\"airspeed_kt\":%.0f", entry->airspeed_kt);
        if (entry->has_squawk) printf(",\"squawk\":\"%04o\"", entry->squawk);
        if (entry->has_adsb_version) printf(",\"adsb_version\":%u", entry->adsb_version);
        printf("}\n");
        return;
    }
    if (output_get_format() == OUTPUT_FORMAT_CSV) {
        static int header_printed = 0;
        if (!header_printed) {
            printf("timestamp,protocol,icao,callsign,category,altitude_ft,latitude,longitude,speed_kt,track_deg,vertical_rate_fpm,squawk\n");
            header_printed = 1;
        }
        printf("%s,adsb,%06X,\"%s\",\"%s\",", timestamp, entry->icao,
               entry->callsign, entry->category ? entry->category : "");
        if (entry->has_altitude) printf("%d", entry->altitude_ft);
        printf(",");
        if (entry->has_position) printf("%.6f,%.6f", entry->latitude, entry->longitude);
        else printf(",");
        printf(",");
        if (entry->has_ground_speed) printf("%.1f", entry->ground_speed_kt);
        printf(",");
        if (entry->has_track) printf("%.1f", entry->track_deg);
        printf(",");
        if (entry->has_vertical_rate) printf("%d", entry->vertical_rate_fpm);
        printf(",");
        if (entry->has_squawk) printf("%04o", entry->squawk);
        printf("\n");
        return;
    }

    printf("[ADSB] [%s] icao=%06X", timestamp, entry->icao);
    if (entry->callsign[0]) printf(" callsign=%s", entry->callsign);
    if (entry->category && strcmp(entry->category, "Unknown") != 0) {
        printf(" category=\"%s\"", entry->category);
    }
    if (entry->has_altitude) {
        printf(" %s_alt=%dft", entry->altitude_is_gnss ? "gnss" : "baro",
               entry->altitude_ft);
    }
    if (entry->has_position) {
        printf(" lat=%.6f lon=%.6f", entry->latitude, entry->longitude);
    }
    if (entry->has_ground_speed) printf(" speed=%.1fkt", entry->ground_speed_kt);
    if (entry->has_track) printf(" track=%.1fdeg", entry->track_deg);
    if (entry->has_vertical_rate) {
        printf(" vertical_rate=%dfpm(%s)", entry->vertical_rate_fpm,
               entry->vertical_rate_is_gnss ? "gnss" : "baro");
    }
    if (entry->has_airspeed) {
        printf(" %s=%.0fkt", entry->airspeed_is_true ? "tas" : "ias",
               entry->airspeed_kt);
    }
    if (entry->has_squawk) printf(" squawk=%04o", entry->squawk);
    if (entry->has_adsb_version) printf(" version=%u", entry->adsb_version);
    printf("\n");
}

void adsb_protocol_reset(void) {
    memset(g_aircraft, 0, sizeof(g_aircraft));
    g_aircraft_count = 0u;
    g_have_reference = false;
    g_reference_lat = 0.0;
    g_reference_lon = 0.0;
}

int adsb_protocol_set_reference(double latitude, double longitude) {
    if (!isfinite(latitude) || !isfinite(longitude)) return 0;
    if (latitude < -90.0 || latitude > 90.0) return 0;
    if (longitude < -180.0 || longitude > 180.0) return 0;
    g_reference_lat = latitude;
    g_reference_lon = longitude;
    g_have_reference = true;
    return 1;
}

uint32_t adsb_frame_crc(const uint8_t *frame, size_t frame_bits) {
    uint32_t remainder = 0u;

    if (!frame || (frame_bits != 56u && frame_bits != 112u)) return UINT32_MAX;
    for (size_t i = 0; i < frame_bits; i++) {
        uint32_t input = (uint32_t)((frame[i / 8u] >> (7u - (i % 8u))) & 1u);
        uint32_t top = (remainder >> 23u) & 1u;
        remainder = ((remainder << 1u) & 0xFFFFFFu) | input;
        if (top) remainder ^= MODE_S_POLY;
    }
    return remainder;
}

static char callsign_char(uint8_t code) {
    if (code >= 1u && code <= 26u) return (char)('A' + code - 1u);
    if (code >= 48u && code <= 57u) return (char)code;
    return ' ';
}

static void decode_callsign(uint64_t me, char out[9]) {
    size_t end = 8u;

    for (size_t i = 0; i < 8u; i++) {
        out[i] = callsign_char((uint8_t)me_get_u32(
            me, 8u + (unsigned int)(i * 6u), 6u));
    }
    out[8] = '\0';
    while (end > 0u && out[end - 1u] == ' ') out[--end] = '\0';
}

static const char *emitter_category(uint8_t tc, uint8_t category) {
    static const char *const set_a[8] = {
        "No category information", "Light aircraft", "Small aircraft",
        "Large aircraft", "High-vortex aircraft", "Heavy aircraft",
        "High-performance aircraft", "Rotorcraft"
    };
    static const char *const set_b[8] = {
        "No category information", "Glider or sailplane", "Lighter-than-air",
        "Parachutist or skydiver", "Ultralight or hang-glider",
        "Reserved", "Unmanned aerial vehicle", "Space vehicle"
    };
    static const char *const set_c[8] = {
        "No category information", "Surface emergency vehicle",
        "Surface service vehicle", "Point obstacle", "Cluster obstacle",
        "Line obstacle", "Reserved", "Reserved"
    };

    if (category > 7u) return "Unknown";
    if (tc == 4u) return set_a[category];
    if (tc == 3u) return set_b[category];
    if (tc == 2u) return set_c[category];
    return category == 0u ? "No category information" : "Reserved";
}

static int cpr_nl(double latitude) {
    double lat = fabs(latitude);
    double a;
    double b;
    double arg;
    double denom;
    int nl;

    if (lat >= 87.0) return 1;
    a = 1.0 - cos(PI_D / 30.0);
    b = cos(lat * PI_D / 180.0);
    if (fabs(b) < 1e-12) return 1;
    arg = 1.0 - a / (b * b);
    if (arg < -1.0) arg = -1.0;
    if (arg > 1.0) arg = 1.0;
    denom = acos(arg);
    if (denom <= 0.0) return 1;
    nl = (int)floor(2.0 * PI_D / denom);
    return nl < 1 ? 1 : nl;
}

static bool cpr_decode_global(aircraft_t *entry) {
    double j;
    double lat_even;
    double lat_odd;
    double latitude;
    double longitude;
    double m;
    double ni;
    double dlon;
    int latest;
    time_t age;

    if (!entry || !entry->cpr_valid[0] || !entry->cpr_valid[1]) return false;
    if (entry->cpr_surface[0] || entry->cpr_surface[1]) return false;
    age = entry->cpr_time[0] - entry->cpr_time[1];
    if (age < 0) age = -age;
    if (age > ADSB_CPR_PAIR_MAX_AGE) return false;

    j = floor(59.0 * entry->cpr_lat[0] - 60.0 * entry->cpr_lat[1] + 0.5);
    lat_even = 6.0 * (positive_mod(j, 60.0) + entry->cpr_lat[0]);
    lat_odd = (360.0 / 59.0) * (positive_mod(j, 59.0) + entry->cpr_lat[1]);
    if (lat_even >= 270.0) lat_even -= 360.0;
    if (lat_odd >= 270.0) lat_odd -= 360.0;
    if (cpr_nl(lat_even) != cpr_nl(lat_odd)) return false;

    latest = entry->cpr_time[0] >= entry->cpr_time[1] ? 0 : 1;
    latitude = latest == 0 ? lat_even : lat_odd;
    ni = (double)(cpr_nl(latitude) - latest);
    if (ni < 1.0) ni = 1.0;
    m = floor(entry->cpr_lon[0] * (double)(cpr_nl(latitude) - 1)
              - entry->cpr_lon[1] * (double)cpr_nl(latitude) + 0.5);
    dlon = 360.0 / ni;
    longitude = dlon * (positive_mod(m, ni) + entry->cpr_lon[latest]);
    if (longitude >= 180.0) longitude -= 360.0;

    if (!isfinite(latitude) || !isfinite(longitude)) return false;
    if (latitude < -90.0 || latitude > 90.0) return false;
    if (longitude < -180.0 || longitude > 180.0) return false;
    entry->latitude = latitude;
    entry->longitude = longitude;
    entry->has_position = true;
    return true;
}

static bool cpr_decode_local(aircraft_t *entry, int odd, bool surface) {
    double ref_lat;
    double ref_lon;
    double dlat;
    double latitude;
    double dlon;
    double longitude;
    double j;
    double m;
    int ni;

    if (!entry || odd < 0 || odd > 1 || !entry->cpr_valid[odd]) return false;
    if (entry->has_position) {
        ref_lat = entry->latitude;
        ref_lon = entry->longitude;
    } else if (g_have_reference) {
        ref_lat = g_reference_lat;
        ref_lon = g_reference_lon;
    } else {
        return false;
    }

    dlat = (surface ? 90.0 : 360.0) / (60.0 - (double)odd);
    j = floor(ref_lat / dlat)
        + floor(0.5 + positive_mod(ref_lat, dlat) / dlat - entry->cpr_lat[odd]);
    latitude = dlat * (j + entry->cpr_lat[odd]);
    if (latitude > 90.0) latitude -= surface ? 180.0 : 360.0;
    if (latitude < -90.0) latitude += surface ? 180.0 : 360.0;
    if (latitude < -90.0 || latitude > 90.0) return false;

    ni = cpr_nl(latitude) - odd;
    if (ni < 1) ni = 1;
    dlon = (surface ? 90.0 : 360.0) / (double)ni;
    m = floor(ref_lon / dlon)
        + floor(0.5 + positive_mod(ref_lon, dlon) / dlon - entry->cpr_lon[odd]);
    longitude = dlon * (m + entry->cpr_lon[odd]);
    while (longitude > 180.0) longitude -= 360.0;
    while (longitude < -180.0) longitude += 360.0;

    entry->latitude = latitude;
    entry->longitude = longitude;
    entry->has_position = true;
    return true;
}

static unsigned int decode_id13_field(unsigned int id13) {
    unsigned int gillham = 0u;
    if (id13 & 0x1000u) gillham |= 0x0010u; /* C1 */
    if (id13 & 0x0800u) gillham |= 0x1000u; /* A1 */
    if (id13 & 0x0400u) gillham |= 0x0020u; /* C2 */
    if (id13 & 0x0200u) gillham |= 0x2000u; /* A2 */
    if (id13 & 0x0100u) gillham |= 0x0040u; /* C4 */
    if (id13 & 0x0080u) gillham |= 0x4000u; /* A4 */
    if (id13 & 0x0020u) gillham |= 0x0100u; /* B1 */
    if (id13 & 0x0010u) gillham |= 0x0001u; /* D1/Q */
    if (id13 & 0x0008u) gillham |= 0x0200u; /* B2 */
    if (id13 & 0x0004u) gillham |= 0x0002u; /* D2 */
    if (id13 & 0x0002u) gillham |= 0x0400u; /* B4 */
    if (id13 & 0x0001u) gillham |= 0x0004u; /* D4 */
    return gillham;
}

static int mode_a_to_mode_c(unsigned int mode_a) {
    unsigned int five_hundreds = 0u;
    unsigned int one_hundreds = 0u;

    if ((mode_a & 0xFFFF888Bu) != 0u || (mode_a & 0x0010u) == 0u) return -9999;
    if (mode_a & 0x0010u) one_hundreds ^= 0x007u;
    if (mode_a & 0x0020u) one_hundreds ^= 0x003u;
    if (mode_a & 0x0040u) one_hundreds ^= 0x001u;
    if ((one_hundreds & 5u) == 5u) one_hundreds ^= 2u;
    if (one_hundreds > 5u) return -9999;

    if (mode_a & 0x0002u) five_hundreds ^= 0x0FFu;
    if (mode_a & 0x0004u) five_hundreds ^= 0x07Fu;
    if (mode_a & 0x1000u) five_hundreds ^= 0x03Fu;
    if (mode_a & 0x2000u) five_hundreds ^= 0x01Fu;
    if (mode_a & 0x4000u) five_hundreds ^= 0x00Fu;
    if (mode_a & 0x0100u) five_hundreds ^= 0x007u;
    if (mode_a & 0x0200u) five_hundreds ^= 0x003u;
    if (mode_a & 0x0400u) five_hundreds ^= 0x001u;
    if (five_hundreds & 1u) one_hundreds = 6u - one_hundreds;
    return (int)(five_hundreds * 5u + one_hundreds) - 13;
}

static bool decode_altitude(uint16_t encoded, int *altitude_ft) {
    unsigned int n;
    int hundreds;

    if (!altitude_ft || encoded == 0u) return false;
    if ((encoded & 0x10u) != 0u) {
        n = ((unsigned int)(encoded & 0x0FE0u) >> 1u) | (encoded & 0x000Fu);
        *altitude_ft = (int)n * 25 - 1000;
        return true;
    }

    hundreds = mode_a_to_mode_c(decode_id13_field((unsigned int)encoded << 1u));
    if (hundreds < -12) return false;
    *altitude_ft = hundreds * 100;
    return true;
}

static double decode_surface_speed(uint8_t movement) {
    if (movement == 0u || movement >= 125u) return NAN;
    if (movement == 1u) return 0.0;
    if (movement <= 8u) return ((double)movement - 1.0) * 0.125;
    if (movement <= 12u) return 1.0 + ((double)movement - 9.0) * 0.25;
    if (movement <= 38u) return 2.0 + ((double)movement - 13.0) * 0.5;
    if (movement <= 93u) return 15.0 + ((double)movement - 39.0);
    if (movement <= 108u) return 70.0 + ((double)movement - 94.0) * 2.0;
    if (movement <= 123u) return 100.0 + ((double)movement - 109.0) * 5.0;
    return 175.0;
}

static uint16_t decode_squawk(uint16_t id13) {
    unsigned int a = ((id13 & 0x1000u) >> 9u)
                   | ((id13 & 0x0400u) >> 8u)
                   | ((id13 & 0x0100u) >> 7u);
    unsigned int b = ((id13 & 0x0020u) >> 3u)
                   | ((id13 & 0x0008u) >> 2u)
                   | ((id13 & 0x0002u) >> 1u);
    unsigned int c = ((id13 & 0x0800u) >> 8u)
                   | ((id13 & 0x0200u) >> 7u)
                   | ((id13 & 0x0080u) >> 6u);
    unsigned int d = ((id13 & 0x0010u) >> 4u)
                   | ((id13 & 0x0004u) >> 3u)
                   | (id13 & 0x0001u);
    return (uint16_t)((a << 9u) | (b << 6u) | (c << 3u) | d);
}

static const char *emergency_name(uint8_t emergency) {
    static const char *const names[8] = {
        "none", "general", "lifeguard", "minimum-fuel",
        "no-communications", "unlawful-interference", "downed-aircraft", "reserved"
    };
    return names[emergency & 7u];
}

static void handle_identification(aircraft_t *entry, uint64_t me, uint8_t tc) {
    uint8_t category = (uint8_t)me_get_u32(me, 5u, 3u);
    if (!entry) return;
    decode_callsign(me, entry->callsign);
    entry->category = emitter_category(tc, category);
    if (!output_is_human()) return;
    printf("[ADSB][identification] icao=%06X tc=%u category=%u callsign=%s\n",
           entry->icao, tc, category,
           entry->callsign[0] ? entry->callsign : "<unavailable>");
}

static void handle_position(aircraft_t *entry, uint64_t me, uint8_t tc,
                            bool surface, bool gnss_altitude) {
    int odd = (int)me_get_u32(me, 21u, 1u);
    uint32_t raw_lat = me_get_u32(me, 22u, 17u);
    uint32_t raw_lon = me_get_u32(me, 39u, 17u);
    bool position_ok;

    if (!entry) return;
    entry->cpr_lat[odd] = (double)raw_lat / ADSB_CPR_SCALE;
    entry->cpr_lon[odd] = (double)raw_lon / ADSB_CPR_SCALE;
    entry->cpr_time[odd] = time(NULL);
    entry->cpr_valid[odd] = true;
    entry->cpr_surface[odd] = surface;

    if (surface) {
        uint8_t movement = (uint8_t)me_get_u32(me, 5u, 7u);
        bool track_valid = me_get_u32(me, 12u, 1u) != 0u;
        uint8_t track = (uint8_t)me_get_u32(me, 13u, 7u);
        double speed = decode_surface_speed(movement);
        if (isfinite(speed)) {
            entry->ground_speed_kt = speed;
            entry->has_ground_speed = true;
        }
        if (track_valid) {
            entry->track_deg = (double)track * 360.0 / 128.0;
            entry->has_track = true;
        }
        position_ok = cpr_decode_local(entry, odd, true);
        if (!output_is_human()) return;
        printf("[ADSB][surface-position] icao=%06X tc=%u cpr=%s movement=%u",
               entry->icao, tc, odd ? "odd" : "even", movement);
        if (track_valid) printf(" track=%.1fdeg", entry->track_deg);
        if (!position_ok && !g_have_reference && !entry->has_position) {
            printf(" position=pending-reference(--lat/--lon)");
        }
        printf("\n");
        return;
    }

    {
        uint16_t encoded_alt = (uint16_t)me_get_u32(me, 8u, 12u);
        int altitude;
        if (decode_altitude(encoded_alt, &altitude)) {
            entry->altitude_ft = altitude;
            entry->has_altitude = true;
            entry->altitude_is_gnss = gnss_altitude;
        }
    }

    position_ok = cpr_decode_global(entry);
    if (!position_ok) position_ok = cpr_decode_local(entry, odd, false);
    if (!output_is_human()) return;
    printf("[ADSB][airborne-position] icao=%06X tc=%u altitude_source=%s cpr=%s",
           entry->icao, tc, gnss_altitude ? "gnss" : "barometric",
           odd ? "odd" : "even");
    if (!position_ok) printf(" position=pending-pair-or-reference");
    printf("\n");
}

static void handle_velocity(aircraft_t *entry, uint64_t me) {
    uint8_t subtype = (uint8_t)me_get_u32(me, 5u, 3u);
    uint32_t vertical_raw = me_get_u32(me, 37u, 9u);
    bool vertical_sign = me_get_u32(me, 36u, 1u) != 0u;
    bool vertical_gnss = me_get_u32(me, 35u, 1u) != 0u;

    if (!entry) return;
    if (vertical_raw > 0u) {
        int rate = ((int)vertical_raw - 1) * 64;
        entry->vertical_rate_fpm = vertical_sign ? -rate : rate;
        entry->vertical_rate_is_gnss = vertical_gnss;
        entry->has_vertical_rate = true;
    }

    if (subtype == 1u || subtype == 2u) {
        uint32_t ew_raw = me_get_u32(me, 14u, 10u);
        uint32_t ns_raw = me_get_u32(me, 25u, 10u);
        int scale = subtype == 2u ? 4 : 1;
        if (ew_raw > 0u && ns_raw > 0u) {
            double ew = (double)(((int)ew_raw - 1) * scale);
            double ns = (double)(((int)ns_raw - 1) * scale);
            if (me_get_u32(me, 13u, 1u)) ew = -ew;
            if (me_get_u32(me, 24u, 1u)) ns = -ns;
            entry->ground_speed_kt = hypot(ew, ns);
            entry->track_deg = atan2(ew, ns) * 180.0 / PI_D;
            if (entry->track_deg < 0.0) entry->track_deg += 360.0;
            entry->has_ground_speed = true;
            entry->has_track = true;
        }
        if (output_is_human()) {
            printf("[ADSB][velocity] icao=%06X subtype=%u vector=ground-speed\n",
                   entry->icao, subtype);
        }
    } else if (subtype == 3u || subtype == 4u) {
        bool heading_valid = me_get_u32(me, 13u, 1u) != 0u;
        uint32_t heading_raw = me_get_u32(me, 14u, 10u);
        uint32_t airspeed_raw = me_get_u32(me, 25u, 10u);
        bool true_airspeed = me_get_u32(me, 24u, 1u) != 0u;
        int scale = subtype == 4u ? 4 : 1;
        if (heading_valid) {
            entry->track_deg = (double)heading_raw * 360.0 / 1024.0;
            entry->has_track = true;
        }
        if (airspeed_raw > 0u) {
            entry->airspeed_kt = (double)(((int)airspeed_raw - 1) * scale);
            entry->airspeed_is_true = true_airspeed;
            entry->has_airspeed = true;
        }
        if (output_is_human()) {
            printf("[ADSB][velocity] icao=%06X subtype=%u vector=airspeed\n",
                   entry->icao, subtype);
        }
    } else {
        if (output_is_human()) {
            printf("[ADSB][velocity] icao=%06X subtype=%u unsupported-subtype\n",
                   entry->icao, subtype);
        }
    }
}

static void handle_aircraft_status(aircraft_t *entry, uint64_t me) {
    uint8_t subtype = (uint8_t)me_get_u32(me, 5u, 3u);
    if (!entry) return;
    if (!output_is_human()) {
        if (subtype == 1u) {
            entry->emergency_state = (uint8_t)me_get_u32(me, 8u, 3u);
            entry->squawk = decode_squawk((uint16_t)me_get_u32(me, 11u, 13u));
            entry->has_squawk = true;
        }
        return;
    }
    if (subtype == 1u) {
        entry->emergency_state = (uint8_t)me_get_u32(me, 8u, 3u);
        entry->squawk = decode_squawk((uint16_t)me_get_u32(me, 11u, 13u));
        entry->has_squawk = true;
        printf("[ADSB][status] icao=%06X emergency=%s squawk=%04o\n",
               entry->icao, emergency_name(entry->emergency_state), entry->squawk);
    } else if (subtype == 2u) {
        printf("[ADSB][status] icao=%06X ACAS-resolution-advisory=%012llX\n",
               entry->icao,
               (unsigned long long)(me & 0xFFFFFFFFFFFFULL));
    } else {
        printf("[ADSB][status] icao=%06X subtype=%u reserved\n", entry->icao, subtype);
    }
}

static void handle_target_state(aircraft_t *entry, uint64_t me) {
    uint8_t subtype = (uint8_t)me_get_u32(me, 5u, 2u);
    uint32_t selected_raw = me_get_u32(me, 9u, 11u);
    uint32_t baro_raw = me_get_u32(me, 20u, 9u);
    bool heading_valid = me_get_u32(me, 29u, 1u) != 0u;
    uint32_t heading_raw = me_get_u32(me, 30u, 9u);

    if (!entry) return;
    if (!output_is_human()) return;
    printf("[ADSB][target-state] icao=%06X subtype=%u", entry->icao, subtype);
    if (selected_raw > 0u) printf(" selected_altitude=%uft", (selected_raw - 1u) * 32u);
    if (baro_raw > 0u) printf(" baro=%.1fhPa", 800.0 + (double)(baro_raw - 1u) * 0.8);
    if (heading_valid) printf(" selected_heading=%.1fdeg", (double)heading_raw * 180.0 / 256.0);
    printf(" autopilot=%u vnav=%u altitude_hold=%u approach=%u lnav=%u tcas=%u\n",
           me_get_u32(me, 46u, 1u), me_get_u32(me, 47u, 1u),
           me_get_u32(me, 48u, 1u), me_get_u32(me, 50u, 1u),
           me_get_u32(me, 52u, 1u), me_get_u32(me, 51u, 1u));
}

static void handle_operational_status(aircraft_t *entry, uint64_t me) {
    uint8_t subtype = (uint8_t)me_get_u32(me, 5u, 3u);
    uint8_t version = (uint8_t)me_get_u32(me, 40u, 3u);
    uint8_t nacp = (uint8_t)me_get_u32(me, 44u, 4u);
    uint8_t sil = (uint8_t)me_get_u32(me, 50u, 2u);

    if (!entry) return;
    entry->adsb_version = version;
    entry->has_adsb_version = true;
    if (!output_is_human()) return;
    printf("[ADSB][operational-status] icao=%06X subtype=%s version=%u nacp=%u sil=%u",
           entry->icao, subtype == 0u ? "airborne" : (subtype == 1u ? "surface" : "reserved"),
           version, nacp, sil);
    if (subtype == 0u) {
        printf(" tcas=%u cdti=%u arv=%u target-state=%u",
               me_get_u32(me, 10u, 1u), me_get_u32(me, 11u, 1u),
               me_get_u32(me, 12u, 1u), me_get_u32(me, 13u, 1u));
    }
    printf("\n");
}

void protocol_handle_message(uint8_t df, uint8_t ca, uint32_t icao,
                             uint64_t me, uint32_t pi) {
    uint8_t tc;
    aircraft_t *entry;

    (void)ca;
    (void)pi;
    if (df != 17u && df != 18u) return;
    entry = get_aircraft(icao);
    if (!entry) {
        fprintf(stderr, "[ADSB] aircraft table full; dropping icao=%06X\n", icao);
        return;
    }
    entry->last_seen = time(NULL);
    tc = (uint8_t)me_get_u32(me, 0u, 5u);

    switch (tc_category(tc)) {
    case ADSB_TC_IDENTIFICATION:
        handle_identification(entry, me, tc);
        break;
    case ADSB_TC_SURFACE_POSITION:
        handle_position(entry, me, tc, true, false);
        break;
    case ADSB_TC_AIRBORNE_POSITION_BARO:
        handle_position(entry, me, tc, false, false);
        break;
    case ADSB_TC_AIRBORNE_VELOCITY:
        handle_velocity(entry, me);
        break;
    case ADSB_TC_AIRBORNE_POSITION_GNSS:
        handle_position(entry, me, tc, false, true);
        break;
    case ADSB_TC_AIRCRAFT_STATUS:
        handle_aircraft_status(entry, me);
        break;
    case ADSB_TC_TARGET_STATE:
        handle_target_state(entry, me);
        break;
    case ADSB_TC_OPERATIONAL_STATUS:
        handle_operational_status(entry, me);
        break;
    case ADSB_TC_RESERVED:
        if (output_is_human()) {
            printf("[ADSB][reserved] icao=%06X tc=%u raw_me=%014llX\n", entry->icao, tc,
                   (unsigned long long)me);
        }
        break;
    case ADSB_TC_UNKNOWN:
    default:
        if (output_is_human()) {
            printf("[ADSB][unknown] icao=%06X tc=%u raw_me=%014llX\n", entry->icao, tc,
                   (unsigned long long)me);
        }
        break;
    }
    print_aircraft(entry);
}

static int adsb_protocol_handle_frame_meta(const uint8_t *frame, size_t frame_bits,
                                           uint64_t timestamp_12mhz,
                                           uint8_t signal_level) {
    uint8_t df;
    uint8_t ca;
    uint32_t icao;
    uint64_t me;
    uint32_t parity;
    uint32_t crc;

    if (!frame || (frame_bits != 56u && frame_bits != 112u)) {
        fprintf(stderr, "[ADSB] frame length must be 56 or 112 bits\n");
        return 0;
    }
    df = (uint8_t)bits_get_u32(frame, 0, 5);
    if (frame_bits != 112u || (df != 17u && df != 18u)) {
        fprintf(stderr, "[ADSB] DF%u is Mode S but not a supported ADS-B extended squitter\n", df);
        return 0;
    }
    crc = adsb_frame_crc(frame, frame_bits);
    if (crc != 0u) {
        fprintf(stderr, "[ADSB] rejected frame: CRC remainder=%06X\n", crc);
        return 0;
    }

    ca = (uint8_t)bits_get_u32(frame, 5, 3);
    icao = bits_get_u32(frame, 8, 24);
    me = bits_get_u64(frame, 32, 56);
    parity = bits_get_u32(frame, 88, 24);
    output_emit_adsb_raw(frame, frame_bits, timestamp_12mhz, signal_level);
    if (output_is_human()) {
        printf("[ADSB] frame=valid df=%u ca/cf=%u icao=%06X type_code=%u\n",
               df, ca, icao, (unsigned int)((me >> 51u) & 0x1Fu));
    }
    protocol_handle_message(df, ca, icao, me, parity);
    return 1;
}

int adsb_protocol_handle_frame(const uint8_t *frame, size_t frame_bits) {
    return adsb_protocol_handle_frame_meta(frame, frame_bits, 0u, 0xFFu);
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)tolower((unsigned char)c);
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int parse_frame_hex(const char *hex, uint8_t frame[14], size_t *frame_bits) {
    size_t n;
    if (!hex || !frame || !frame_bits) return 0;
    while (*hex == '*' || isspace((unsigned char)*hex)) hex++;
    n = strlen(hex);
    while (n > 0u && isspace((unsigned char)hex[n - 1u])) n--;
    if (n != 14u && n != 28u) return 0;
    memset(frame, 0, 14u);
    for (size_t i = 0; i < n / 2u; i++) {
        int hi = hex_nibble(hex[i * 2u]);
        int lo = hex_nibble(hex[i * 2u + 1u]);
        if (hi < 0 || lo < 0) return 0;
        frame[i] = (uint8_t)((hi << 4) | lo);
    }
    *frame_bits = n * 4u;
    return 1;
}

int adsb_protocol_handle_frame_hex(const char *hex) {
    uint8_t frame[14];
    size_t frame_bits;
    if (!parse_frame_hex(hex, frame, &frame_bits)) {
        fprintf(stderr, "[ADSB] frame must contain 14 or 28 hexadecimal digits\n");
        return 0;
    }
    return adsb_protocol_handle_frame(frame, frame_bits);
}

static int parse_fixed_hex_u64(const char *text, size_t digits, uint64_t *value) {
    uint64_t parsed = 0u;
    if (!text || !value || digits > 16u) return 0;
    for (size_t i = 0; i < digits; i++) {
        int nibble = hex_nibble(text[i]);
        if (nibble < 0) return 0;
        parsed = (parsed << 4u) | (uint64_t)nibble;
    }
    *value = parsed;
    return 1;
}

int adsb_protocol_handle_avr_line(const char *line) {
    char frame_hex[29];
    uint8_t frame[14];
    size_t frame_bits;
    size_t digits = 0u;
    uint64_t timestamp = 0u;

    if (!line) return 0;
    while (isspace((unsigned char)*line)) line++;
    if (*line == '@') {
        if (!parse_fixed_hex_u64(line + 1, 12u, &timestamp)) {
            fprintf(stderr, "[ADSB][AVR] invalid 12-digit timestamp\n");
            return 0;
        }
        line += 13;
    } else if (*line == '*' || *line == '+') {
        line++;
    }
    while (isxdigit((unsigned char)line[digits]) && digits < 28u) digits++;
    if ((digits != 14u && digits != 28u)
        || (line[digits] != ';' && line[digits] != '\0'
            && line[digits] != '\r' && line[digits] != '\n')) {
        fprintf(stderr, "[ADSB][AVR] expected *HEX; or @TIMESTAMPHEX;\n");
        return 0;
    }
    memcpy(frame_hex, line, digits);
    frame_hex[digits] = '\0';
    if (!parse_frame_hex(frame_hex, frame, &frame_bits)) return 0;
    return adsb_protocol_handle_frame_meta(frame, frame_bits, timestamp, 0xFFu);
}

void adsb_beast_parser_init(adsb_beast_parser_t *parser) {
    if (parser) memset(parser, 0, sizeof(*parser));
}

static void beast_start_type(adsb_beast_parser_t *parser, uint8_t type) {
    parser->type = type;
    parser->payload_len = 0u;
    parser->in_frame = type == '1' || type == '2' || type == '3';
    parser->expected_len = type == '1' ? 9u : type == '2' ? 14u
                                                       : type == '3' ? 21u : 0u;
    parser->after_escape = 0;
}

static void beast_complete(adsb_beast_parser_t *parser) {
    uint64_t timestamp = 0u;
    size_t frame_bits = parser->type == '2' ? 56u : 112u;
    if (parser->type == '1') {
        parser->in_frame = 0;
        parser->payload_len = 0u;
        parser->expected_len = 0u;
        return;
    }
    for (size_t i = 0; i < 6u; i++) {
        timestamp = (timestamp << 8u) | parser->payload[i];
    }
    if (adsb_protocol_handle_frame_meta(parser->payload + 7u, frame_bits,
                                        timestamp, parser->payload[6u])) {
        parser->frames_valid++;
    } else {
        parser->frames_rejected++;
    }
    parser->in_frame = 0;
    parser->payload_len = 0u;
    parser->expected_len = 0u;
}

void adsb_beast_parser_feed(adsb_beast_parser_t *parser,
                            const uint8_t *data, size_t len) {
    if (!parser || (!data && len != 0u)) return;
    for (size_t i = 0; i < len; i++) {
        uint8_t value = data[i];
        if (!parser->in_frame) {
            if (parser->after_escape) {
                beast_start_type(parser, value);
            } else if (value == 0x1Au) {
                parser->after_escape = 1;
            }
            continue;
        }
        if (parser->after_escape) {
            if (value == 0x1Au) {
                value = 0x1Au;
                parser->after_escape = 0;
            } else {
                parser->frames_rejected++;
                beast_start_type(parser, value);
                continue;
            }
        } else if (value == 0x1Au) {
            parser->after_escape = 1;
            continue;
        }
        if (parser->payload_len < sizeof(parser->payload)) {
            parser->payload[parser->payload_len++] = value;
        }
        if (parser->payload_len == parser->expected_len) beast_complete(parser);
    }
}

int adsb_protocol_emit_test_examples(void) {
    static const char *const examples[] = {
        "8D4840D6202CC371C32CE0576098", /* identification: KLM1023 */
        "8D40621D58C382D690C8AC2863A7", /* airborne position, even */
        "8D40621D58C386435CC412692AD6", /* airborne position, odd */
        "8D485020994409940838175B284F"  /* airborne velocity */
    };
    uint8_t frame[14];
    size_t bits;
    int decoded = 0;

    fprintf(output_diagnostics(), "[ADSB] offline protocol test\n");
    for (size_t i = 0; i < sizeof(examples) / sizeof(examples[0]); i++) {
        fprintf(output_diagnostics(), "[ADSB] test_input=%s\n", examples[i]);
        if (parse_frame_hex(examples[i], frame, &bits)
            && adsb_protocol_handle_frame(frame, bits)) {
            decoded++;
        }
    }
    fprintf(output_diagnostics(), "[ADSB] test_result=%s decoded=%d expected=%zu\n",
            decoded == (int)(sizeof(examples) / sizeof(examples[0])) ? "PASS" : "FAIL",
            decoded, sizeof(examples) / sizeof(examples[0]));
    return decoded == (int)(sizeof(examples) / sizeof(examples[0]));
}
