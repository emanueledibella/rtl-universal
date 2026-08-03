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
    uint8_t last_df;
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

    uint8_t last_bds;
    int selected_altitude_ft;
    bool has_selected_altitude;
    double roll_deg;
    bool has_roll;
    double mach;
    bool has_mach;
    double temperature_c;
    bool has_temperature;
    double wind_speed_kt;
    double wind_direction_deg;
    bool has_wind;

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
        update.downlink_format = entry->last_df;
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
        update.squawk = entry->squawk;
        update.has_squawk = entry->has_squawk;
        dashboard_update_adsb(&update);
        return;
    }
    now = time(NULL);
    if (localtime_r(&now, &local_tm)) {
        (void)strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &local_tm);
    }

    if (output_get_format() == OUTPUT_FORMAT_JSON) {
        printf("{\"protocol\":\"%s\",\"timestamp\":\"%s\",\"df\":%u,\"icao\":\"%06X\"",
               entry->last_df == 17u || entry->last_df == 18u ? "adsb" : "mode-s",
               timestamp, entry->last_df, entry->icao);
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
        if (entry->last_bds) printf(",\"bds\":\"%X,%X\"", entry->last_bds >> 4u, entry->last_bds & 0x0Fu);
        if (entry->has_selected_altitude) printf(",\"selected_altitude_ft\":%d", entry->selected_altitude_ft);
        if (entry->has_roll) printf(",\"roll_deg\":%.2f", entry->roll_deg);
        if (entry->has_mach) printf(",\"mach\":%.3f", entry->mach);
        if (entry->has_temperature) printf(",\"temperature_c\":%.2f", entry->temperature_c);
        if (entry->has_wind) {
            printf(",\"wind_speed_kt\":%.0f,\"wind_direction_deg\":%.1f",
                   entry->wind_speed_kt, entry->wind_direction_deg);
        }
        printf("}\n");
        return;
    }
    if (output_get_format() == OUTPUT_FORMAT_CSV) {
        static int header_printed = 0;
        if (!header_printed) {
            printf("timestamp,protocol,df,icao,callsign,category,altitude_ft,latitude,longitude,speed_kt,track_deg,vertical_rate_fpm,squawk\n");
            header_printed = 1;
        }
        printf("%s,%s,%u,%06X,\"%s\",\"%s\",", timestamp,
               entry->last_df == 17u || entry->last_df == 18u ? "adsb" : "mode-s",
               entry->last_df, entry->icao,
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

    printf("[%s] [%s] df=%u icao=%06X",
           entry->last_df == 17u || entry->last_df == 18u ? "ADSB" : "MODE-S",
           timestamp, entry->last_df, entry->icao);
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
    if (entry->last_bds) printf(" bds=%X,%X", entry->last_bds >> 4u, entry->last_bds & 0x0Fu);
    if (entry->has_selected_altitude) printf(" selected_altitude=%dft", entry->selected_altitude_ft);
    if (entry->has_roll) printf(" roll=%.1fdeg", entry->roll_deg);
    if (entry->has_mach) printf(" mach=%.3f", entry->mach);
    if (entry->has_temperature) printf(" temperature=%.2fC", entry->temperature_c);
    if (entry->has_wind) printf(" wind=%.0fkt/%.1fdeg", entry->wind_speed_kt, entry->wind_direction_deg);
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

static bool decode_altitude_ac13(uint16_t encoded, int *altitude_ft,
                                 bool *metric) {
    unsigned int n;
    int hundreds;

    if (!altitude_ft || encoded == 0u) return false;
    if (metric) *metric = (encoded & 0x0040u) != 0u;
    if ((encoded & 0x0040u) != 0u) return false;
    if ((encoded & 0x0010u) != 0u) {
        n = ((unsigned int)(encoded & 0x1F80u) >> 2u)
            | ((unsigned int)(encoded & 0x0020u) >> 1u)
            | (encoded & 0x000Fu);
        *altitude_ft = (int)n * 25 - 1000;
        return true;
    }
    hundreds = mode_a_to_mode_c(decode_id13_field(encoded));
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

static int signed_bits(uint32_t value, unsigned int width) {
    uint32_t sign;
    uint32_t mask;
    if (width == 0u || width >= 32u) return (int)value;
    sign = (uint32_t)1u << (width - 1u);
    mask = ((uint32_t)1u << width) - 1u;
    value &= mask;
    return (value & sign) ? (int)(value | ~mask) : (int)value;
}

static bool status_consistent(uint64_t mb, unsigned int status_bit,
                              unsigned int value_bit, unsigned int value_len) {
    return me_get_u32(mb, status_bit, 1u) != 0u
           || me_get_u32(mb, value_bit, value_len) == 0u;
}

static bool bds40_valid(uint64_t mb) {
    return (me_get_u32(mb, 0u, 1u) || me_get_u32(mb, 13u, 1u)
            || me_get_u32(mb, 26u, 1u))
           && status_consistent(mb, 0u, 1u, 12u)
           && status_consistent(mb, 13u, 14u, 12u)
           && status_consistent(mb, 26u, 27u, 12u)
           && me_get_u32(mb, 39u, 8u) == 0u
           && me_get_u32(mb, 51u, 2u) == 0u;
}

static bool bds50_valid(uint64_t mb) {
    double roll = (double)signed_bits(me_get_u32(mb, 1u, 10u), 10u) * 45.0 / 256.0;
    double ground_speed = (double)me_get_u32(mb, 24u, 10u) * 2.0;
    double true_airspeed = (double)me_get_u32(mb, 46u, 10u) * 2.0;
    unsigned int statuses = me_get_u32(mb, 0u, 1u) + me_get_u32(mb, 11u, 1u)
                            + me_get_u32(mb, 23u, 1u) + me_get_u32(mb, 34u, 1u)
                            + me_get_u32(mb, 45u, 1u);
    return statuses >= 2u
           && status_consistent(mb, 0u, 1u, 10u)
           && status_consistent(mb, 11u, 12u, 11u)
           && status_consistent(mb, 23u, 24u, 10u)
           && status_consistent(mb, 34u, 35u, 10u)
           && status_consistent(mb, 45u, 46u, 10u)
           && (!me_get_u32(mb, 0u, 1u) || fabs(roll) <= 50.0)
           && (!me_get_u32(mb, 23u, 1u) || ground_speed <= 600.0)
           && (!me_get_u32(mb, 45u, 1u) || true_airspeed <= 500.0);
}

static bool bds60_valid(uint64_t mb) {
    double ias = (double)me_get_u32(mb, 13u, 10u);
    double mach = (double)me_get_u32(mb, 24u, 10u) * 0.004;
    double baro_rate = (double)signed_bits(me_get_u32(mb, 35u, 10u), 10u) * 32.0;
    double inertial_rate = (double)signed_bits(me_get_u32(mb, 46u, 10u), 10u) * 32.0;
    unsigned int statuses = me_get_u32(mb, 0u, 1u) + me_get_u32(mb, 12u, 1u)
                            + me_get_u32(mb, 23u, 1u) + me_get_u32(mb, 34u, 1u)
                            + me_get_u32(mb, 45u, 1u);
    return statuses >= 2u
           && status_consistent(mb, 0u, 1u, 11u)
           && status_consistent(mb, 12u, 13u, 10u)
           && status_consistent(mb, 23u, 24u, 10u)
           && status_consistent(mb, 34u, 35u, 10u)
           && status_consistent(mb, 45u, 46u, 10u)
           && (!me_get_u32(mb, 12u, 1u) || ias <= 500.0)
           && (!me_get_u32(mb, 23u, 1u) || mach <= 1.0)
           && (!me_get_u32(mb, 34u, 1u) || fabs(baro_rate) <= 6000.0)
           && (!me_get_u32(mb, 45u, 1u) || fabs(inertial_rate) <= 6000.0);
}

static bool bds44_valid(uint64_t mb) {
    double temperature = (double)signed_bits(me_get_u32(mb, 23u, 11u), 11u) * 0.25;
    return me_get_u32(mb, 0u, 4u) < 5u
           && status_consistent(mb, 4u, 5u, 18u)
           && status_consistent(mb, 34u, 35u, 11u)
           && status_consistent(mb, 46u, 47u, 2u)
           && status_consistent(mb, 49u, 50u, 6u)
           && temperature >= -80.0 && temperature <= 60.0;
}

static bool bds45_valid(uint64_t mb) {
    double temperature = (double)signed_bits(me_get_u32(mb, 16u, 10u), 10u) * 0.25;
    return status_consistent(mb, 0u, 1u, 2u)
           && status_consistent(mb, 3u, 4u, 2u)
           && status_consistent(mb, 6u, 7u, 2u)
           && status_consistent(mb, 9u, 10u, 2u)
           && status_consistent(mb, 12u, 13u, 2u)
           && status_consistent(mb, 15u, 16u, 10u)
           && status_consistent(mb, 26u, 27u, 11u)
           && status_consistent(mb, 38u, 39u, 12u)
           && me_get_u32(mb, 51u, 5u) == 0u
           && (!me_get_u32(mb, 15u, 1u)
               || (temperature >= -80.0 && temperature <= 60.0));
}

static uint8_t infer_bds(aircraft_t *entry, uint64_t mb, uint8_t parity_hint) {
    uint8_t explicit_code = (uint8_t)me_get_u32(mb, 0u, 8u);
    bool is40;
    bool is44;
    bool is45;
    bool is50;
    bool is60;
    unsigned int count;

    if (parity_hint != 0u) return parity_hint;
    if (explicit_code == 0x10u || explicit_code == 0x20u
        || explicit_code == 0x30u) return explicit_code;
    if (mb == 0u) return 0u;

    is40 = bds40_valid(mb);
    is44 = bds44_valid(mb);
    is45 = bds45_valid(mb);
    is50 = bds50_valid(mb);
    is60 = bds60_valid(mb);
    count = (unsigned int)is40 + (unsigned int)is44 + (unsigned int)is45
            + (unsigned int)is50 + (unsigned int)is60;
    if (count == 1u) {
        if (is40) return 0x40u;
        if (is44) return 0x44u;
        if (is45) return 0x45u;
        if (is50) return 0x50u;
        return 0x60u;
    }
    if (is50 && is60 && entry) {
        double gs50 = (double)me_get_u32(mb, 24u, 10u) * 2.0;
        double ias60 = (double)me_get_u32(mb, 13u, 10u);
        if (entry->has_ground_speed && entry->has_airspeed) {
            double delta50 = fabs(gs50 - entry->ground_speed_kt);
            double delta60 = fabs(ias60 - entry->airspeed_kt);
            if (delta50 + 30.0 < delta60) return 0x50u;
            if (delta60 + 30.0 < delta50) return 0x60u;
        } else if (entry->has_ground_speed && fabs(gs50 - entry->ground_speed_kt) < 100.0) {
            return 0x50u;
        }
    }
    return 0u;
}

static void decode_comm_b(aircraft_t *entry, uint64_t mb, uint8_t parity_hint) {
    uint8_t bds;
    if (!entry) return;
    bds = infer_bds(entry, mb, parity_hint);
    entry->last_bds = bds;

    if (bds == 0x20u) {
        decode_callsign(mb, entry->callsign);
    } else if (bds == 0x40u) {
        if (me_get_u32(mb, 0u, 1u)) {
            entry->selected_altitude_ft = (int)me_get_u32(mb, 1u, 12u) * 16;
            entry->has_selected_altitude = true;
        } else if (me_get_u32(mb, 13u, 1u)) {
            entry->selected_altitude_ft = (int)me_get_u32(mb, 14u, 12u) * 16;
            entry->has_selected_altitude = true;
        }
    } else if (bds == 0x50u) {
        if (me_get_u32(mb, 0u, 1u)) {
            entry->roll_deg = (double)signed_bits(me_get_u32(mb, 1u, 10u), 10u)
                              * 45.0 / 256.0;
            entry->has_roll = true;
        }
        if (me_get_u32(mb, 11u, 1u)) {
            entry->track_deg = (double)signed_bits(me_get_u32(mb, 12u, 11u), 11u)
                               * 90.0 / 512.0;
            if (entry->track_deg < 0.0) entry->track_deg += 360.0;
            entry->has_track = true;
        }
        if (me_get_u32(mb, 23u, 1u)) {
            entry->ground_speed_kt = (double)me_get_u32(mb, 24u, 10u) * 2.0;
            entry->has_ground_speed = true;
        }
        if (me_get_u32(mb, 45u, 1u)) {
            entry->airspeed_kt = (double)me_get_u32(mb, 46u, 10u) * 2.0;
            entry->airspeed_is_true = true;
            entry->has_airspeed = true;
        }
    } else if (bds == 0x60u) {
        if (me_get_u32(mb, 0u, 1u)) {
            entry->track_deg = (double)signed_bits(me_get_u32(mb, 1u, 11u), 11u)
                               * 90.0 / 512.0;
            if (entry->track_deg < 0.0) entry->track_deg += 360.0;
            entry->has_track = true;
        }
        if (me_get_u32(mb, 12u, 1u)) {
            entry->airspeed_kt = (double)me_get_u32(mb, 13u, 10u);
            entry->airspeed_is_true = false;
            entry->has_airspeed = true;
        }
        if (me_get_u32(mb, 23u, 1u)) {
            entry->mach = (double)me_get_u32(mb, 24u, 10u) * 0.004;
            entry->has_mach = true;
        }
        if (me_get_u32(mb, 34u, 1u)) {
            entry->vertical_rate_fpm = signed_bits(me_get_u32(mb, 35u, 10u), 10u) * 32;
            entry->vertical_rate_is_gnss = false;
            entry->has_vertical_rate = true;
        }
    } else if (bds == 0x44u) {
        entry->temperature_c = (double)signed_bits(me_get_u32(mb, 23u, 11u), 11u) * 0.25;
        entry->has_temperature = true;
        if (me_get_u32(mb, 4u, 1u)) {
            entry->wind_speed_kt = (double)me_get_u32(mb, 5u, 9u);
            entry->wind_direction_deg = (double)me_get_u32(mb, 14u, 9u) * 180.0 / 256.0;
            entry->has_wind = true;
        }
    } else if (bds == 0x45u && me_get_u32(mb, 15u, 1u)) {
        entry->temperature_c = (double)signed_bits(me_get_u32(mb, 16u, 10u), 10u) * 0.25;
        entry->has_temperature = true;
    }

    if (output_is_human()) {
        if (bds) {
            printf("[MODE-S][Comm-B] icao=%06X bds=%X,%X raw=%014llX\n",
                   entry->icao, bds >> 4u, bds & 0x0Fu,
                   (unsigned long long)mb);
        } else {
            printf("[MODE-S][Comm-B] icao=%06X bds=unknown raw=%014llX\n",
                   entry->icao, (unsigned long long)mb);
        }
    }
}

static const char *mode_s_df_name(uint8_t df) {
    switch (df) {
    case 0u: return "short-air-air-surveillance";
    case 4u: return "surveillance-altitude";
    case 5u: return "surveillance-identity";
    case 11u: return "all-call-reply";
    case 16u: return "long-air-air-surveillance";
    case 17u: return "extended-squitter";
    case 18u: return "extended-squitter-non-transponder";
    case 19u: return "military-extended-squitter";
    case 20u: return "comm-b-altitude";
    case 21u: return "comm-b-identity";
    case 24u: return "comm-d-extended-length";
    default: return "reserved";
    }
}

static void update_flight_status(aircraft_t *entry, uint8_t flight_status) {
    if (!entry || !output_is_human()) return;
    printf("[MODE-S][surveillance] icao=%06X flight_status=%u", entry->icao,
           flight_status);
    if (flight_status == 0u || flight_status == 2u) printf(" airborne=1");
    if (flight_status == 1u || flight_status == 3u) printf(" on_ground=1");
    if (flight_status == 2u || flight_status == 3u || flight_status == 4u) {
        printf(" alert=1");
    }
    if (flight_status == 4u || flight_status == 5u) printf(" spi=1");
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
    entry->last_df = df;
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

static uint8_t canonical_df(uint8_t raw_df) {
    return raw_df >= 24u ? 24u : raw_df;
}

static bool df_uses_address_parity(uint8_t df) {
    return df == 0u || df == 4u || df == 5u || df == 16u
           || df == 20u || df == 21u || df == 24u;
}

static aircraft_t *recover_address_parity(uint8_t df, uint32_t syndrome,
                                          uint8_t *bds_hint) {
    aircraft_t *entry = find_aircraft(syndrome);
    if (bds_hint) *bds_hint = 0u;
    if (entry || (df != 20u && df != 21u)) return entry;

    for (size_t i = 0u; i < g_aircraft_count; i++) {
        uint32_t overlay = syndrome ^ g_aircraft[i].icao;
        if (overlay > 0u && overlay <= 0xFFu) {
            if (bds_hint) *bds_hint = (uint8_t)overlay;
            return &g_aircraft[i];
        }
    }
    return NULL;
}

static int adsb_protocol_handle_frame_meta(const uint8_t *frame, size_t frame_bits,
                                           uint64_t timestamp_12mhz,
                                           uint8_t signal_level) {
    uint8_t raw_df;
    uint8_t df;
    uint8_t ca;
    uint8_t bds_hint = 0u;
    uint32_t icao = 0u;
    uint32_t syndrome;
    uint32_t parity;
    uint64_t message_field = 0u;
    aircraft_t *entry = NULL;

    if (!frame || (frame_bits != 56u && frame_bits != 112u)) {
        fprintf(stderr, "[MODE-S] frame length must be 56 or 112 bits\n");
        return 0;
    }
    raw_df = (uint8_t)bits_get_u32(frame, 0, 5);
    df = canonical_df(raw_df);
    if ((raw_df < 16u && frame_bits != 56u)
        || (raw_df >= 16u && frame_bits != 112u)) {
        if (output_is_human()) {
            fprintf(stderr, "[MODE-S] DF%u has an invalid frame length\n", raw_df);
        }
        return 0;
    }

    syndrome = adsb_frame_crc(frame, frame_bits);
    parity = bits_get_u32(frame, frame_bits - 24u, 24u);
    ca = (uint8_t)bits_get_u32(frame, 5u, 3u);
    if (frame_bits == 112u) message_field = bits_get_u64(frame, 32u, 56u);

    if (df == 11u) {
        icao = bits_get_u32(frame, 8u, 24u);
        if (syndrome > 0x7Fu) {
            if (output_is_human()) {
                fprintf(stderr, "[MODE-S] rejected DF11: parity syndrome=%06X\n",
                        syndrome);
            }
            return 0;
        }
        entry = get_aircraft(icao);
    } else if (df == 17u || df == 18u || df == 19u) {
        if (syndrome != 0u) {
            if (output_is_human()) {
                fprintf(stderr, "[MODE-S] rejected DF%u: CRC remainder=%06X\n",
                        df, syndrome);
            }
            return 0;
        }
        icao = bits_get_u32(frame, 8u, 24u);
        entry = get_aircraft(icao);
    } else if (df_uses_address_parity(df)) {
        entry = recover_address_parity(df, syndrome, &bds_hint);
        if (!entry) {
            if (output_is_human()) {
                fprintf(stderr,
                        "[MODE-S] rejected DF%u: ICAO recovered from parity is not in the recent-aircraft cache (%06X)\n",
                        df, syndrome);
            }
            return 0;
        }
        icao = entry->icao;
    } else {
        if (syndrome != 0u) {
            if (output_is_human()) {
                fprintf(stderr, "[MODE-S] rejected reserved DF%u: CRC=%06X\n",
                        raw_df, syndrome);
            }
            return 0;
        }
    }

    output_emit_adsb_raw(frame, frame_bits, timestamp_12mhz, signal_level);
    if (output_is_human()) {
        printf("[MODE-S] frame=valid df=%u name=%s bits=%zu",
               df, mode_s_df_name(df), frame_bits);
        if (entry) printf(" icao=%06X", icao);
        if (df == 11u) printf(" interrogator_id=%u", syndrome);
        printf("\n");
    }

    if (df == 17u || df == 18u) {
        protocol_handle_message(df, ca, icao, message_field, parity);
        return 1;
    }
    if (!entry) return 1;

    entry->last_df = df;
    entry->last_seen = time(NULL);
    entry->last_bds = 0u;
    if (df == 0u || df == 16u) {
        uint16_t ac = (uint16_t)bits_get_u32(frame, 19u, 13u);
        int altitude;
        bool metric;
        if (decode_altitude_ac13(ac, &altitude, &metric)) {
            entry->altitude_ft = altitude;
            entry->altitude_is_gnss = false;
            entry->has_altitude = true;
        }
        if (output_is_human()) {
            printf("[MODE-S][ACAS] icao=%06X vertical_status=%u sensitivity=%u reply_info=%u",
                   icao, bits_get_u32(frame, 5u, 1u),
                   bits_get_u32(frame, 8u, 3u), bits_get_u32(frame, 13u, 4u));
            if (entry->has_altitude) printf(" altitude=%dft", entry->altitude_ft);
            if (df == 16u) printf(" message=%014llX", (unsigned long long)message_field);
            printf("\n");
        }
    } else if (df == 4u || df == 20u) {
        uint16_t ac = (uint16_t)bits_get_u32(frame, 19u, 13u);
        int altitude;
        bool metric;
        update_flight_status(entry, ca);
        if (decode_altitude_ac13(ac, &altitude, &metric)) {
            entry->altitude_ft = altitude;
            entry->altitude_is_gnss = false;
            entry->has_altitude = true;
        }
        if (output_is_human() && entry->has_altitude) {
            printf("[MODE-S][altitude] icao=%06X altitude=%dft\n", icao,
                   entry->altitude_ft);
        }
        if (df == 20u) decode_comm_b(entry, message_field, bds_hint);
    } else if (df == 5u || df == 21u) {
        update_flight_status(entry, ca);
        entry->squawk = decode_squawk((uint16_t)bits_get_u32(frame, 19u, 13u));
        entry->has_squawk = true;
        if (output_is_human()) {
            printf("[MODE-S][identity] icao=%06X squawk=%04o\n", icao,
                   entry->squawk);
        }
        if (df == 21u) decode_comm_b(entry, message_field, bds_hint);
    } else if (df == 11u) {
        if (output_is_human()) {
            printf("[MODE-S][all-call] icao=%06X capability=%u\n", icao, ca);
        }
    } else if (df == 19u) {
        if (output_is_human()) {
            printf("[MODE-S][military-extended-squitter] icao=%06X application=%u raw=%014llX\n",
                   icao, ca, (unsigned long long)message_field);
        }
    } else if (df == 24u) {
        if (output_is_human()) {
            printf("[MODE-S][Comm-D] icao=%06X ke=%u segment=%u raw=%014llX\n",
                   icao, raw_df & 7u, bits_get_u32(frame, 5u, 4u),
                   (unsigned long long)message_field);
        }
    }
    print_aircraft(entry);
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

static void test_set_bits(uint8_t *frame, size_t start, size_t length,
                          uint64_t value) {
    for (size_t i = 0u; i < length; i++) {
        size_t bit = start + i;
        uint8_t mask = (uint8_t)(1u << (7u - (bit % 8u)));
        if ((value >> (length - i - 1u)) & 1u) frame[bit / 8u] |= mask;
        else frame[bit / 8u] &= (uint8_t)~mask;
    }
}

static uint16_t test_encode_altitude_ac13(int altitude_ft) {
    unsigned int n = (unsigned int)((altitude_ft + 1000) / 25);
    return (uint16_t)(((n & 0x07E0u) << 2u) | ((n & 0x0010u) << 1u)
                      | (n & 0x000Fu) | 0x0010u);
}

static uint8_t test_callsign_code(char c) {
    if (c >= 'A' && c <= 'Z') return (uint8_t)(c - 'A' + 1);
    if (c >= '0' && c <= '9') return (uint8_t)c;
    return 32u;
}

static uint64_t test_bds20(const char callsign[9]) {
    uint64_t mb = 0x20ULL << 48u;
    for (size_t i = 0u; i < 8u; i++) {
        mb |= (uint64_t)test_callsign_code(callsign[i]) << (42u - i * 6u);
    }
    return mb;
}

static uint64_t test_bds40(void) {
    uint64_t mb = 0u;
    unsigned int selected = 20000u / 16u;
    unsigned int pressure = (unsigned int)((1013.2 - 800.0) * 10.0 + 0.5);
    mb |= 1ULL << 55u;
    mb |= (uint64_t)selected << 43u;
    mb |= 1ULL << 42u;
    mb |= (uint64_t)selected << 30u;
    mb |= 1ULL << 29u;
    mb |= (uint64_t)pressure << 17u;
    return mb;
}

static void test_finalize_mode_s(uint8_t frame[14], size_t bits,
                                 uint32_t parity_overlay) {
    uint32_t checksum;
    test_set_bits(frame, bits - 24u, 24u, 0u);
    checksum = adsb_frame_crc(frame, bits);
    test_set_bits(frame, bits - 24u, 24u, checksum ^ parity_overlay);
}

static void test_build_mode_s(uint8_t frame[14], uint8_t df, uint32_t icao) {
    size_t bits = df < 16u ? 56u : 112u;
    uint8_t canonical = canonical_df(df);
    memset(frame, 0, 14u);
    test_set_bits(frame, 0u, 5u, df);
    test_set_bits(frame, 5u, 3u, 5u);

    if (canonical == 11u || canonical == 18u || canonical == 19u) {
        test_set_bits(frame, 8u, 24u, icao);
    }
    if (canonical == 0u || canonical == 4u || canonical == 16u
        || canonical == 20u) {
        test_set_bits(frame, 19u, 13u, test_encode_altitude_ac13(12000));
    } else if (canonical == 5u || canonical == 21u) {
        test_set_bits(frame, 19u, 13u, 0x0123u);
    }
    if (canonical == 20u) test_set_bits(frame, 32u, 56u, test_bds20("TEST123 "));
    if (canonical == 21u) test_set_bits(frame, 32u, 56u, test_bds40());

    test_finalize_mode_s(frame, bits,
                         df_uses_address_parity(canonical) ? icao : 0u);
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
    int mode_s_decoded = 0;
    static const uint8_t mode_s_dfs[] = {
        11u, 0u, 4u, 5u, 16u, 18u, 19u, 20u, 21u, 24u
    };
    const uint32_t mode_s_test_icao = 0xABCDEFu;

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
    for (size_t i = 0u; i < sizeof(mode_s_dfs) / sizeof(mode_s_dfs[0]); i++) {
        uint8_t df = mode_s_dfs[i];
        test_build_mode_s(frame, df, mode_s_test_icao);
        bits = df < 16u ? 56u : 112u;
        if (adsb_protocol_handle_frame(frame, bits)) mode_s_decoded++;
    }
    fprintf(output_diagnostics(),
            "[MODE-S] test_result=%s decoded=%d expected=%zu\n",
            mode_s_decoded == (int)(sizeof(mode_s_dfs) / sizeof(mode_s_dfs[0]))
                ? "PASS" : "FAIL",
            mode_s_decoded, sizeof(mode_s_dfs) / sizeof(mode_s_dfs[0]));
    return decoded == (int)(sizeof(examples) / sizeof(examples[0]))
           && mode_s_decoded == (int)(sizeof(mode_s_dfs) / sizeof(mode_s_dfs[0]));
}
