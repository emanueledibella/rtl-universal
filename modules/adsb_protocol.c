#include "header/adsb_protocol.h"
#include "utility.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#define ADSB_ME_BITS 56u
#define CPR_AIRBORNE_PAIR_MAX_DT 10u

static const char *const k_aircraft_category[5][8] = {
    [1] = {
        "Reserved", "Reserved", "Reserved", "Reserved",
        "Reserved", "Reserved", "Reserved", "Reserved"
    },
    [2] = {
        "No category information",
        "Surface emergency vehicle",
        "Surface service vehicle",
        "Ground obstruction",
        "Reserved", "Reserved", "Reserved", "Reserved"
    },
    [3] = {
        "No category information",
        "Glider, sailplane",
        "Lighter-than-air",
        "Parachutist, skydiver",
        "Ultralight, hang-glider, paraglider",
        "Reserved",
        "Unmanned aerial vehicle",
        "Space or transatmospheric vehicle"
    },
    [4] = {
        "No category information",
        "Light (less than 7000 kg)",
        "Medium 1 (between 7000 kg and 34000 kg)",
        "Medium 2 (between 34000 kg to 136000 kg)",
        "High vortex aircraft",
        "Heavy (larger than 136000 kg)",
        "High performance (>5 g acceleration) and high speed (>400 kt)",
        "Rotorcraft"
    }
};

typedef struct {
    uint32_t altitude;
    double latitude;
    double longitude;
    uint32_t velocity;
    char callsign[9];
    const char* category;
    double lat_cpr_odd;
    double lat_cpr_even;
    double lon_cpr_odd;
    double lon_cpr_even;
    uint32_t icao;
    uint32_t time_even;
    uint32_t time_odd;
} aircraft;

aircraft aircrafts[2048];
size_t num_aircrafts = 0;

typedef enum {
    ADSB_TC_CATEGORY_UNKNOWN = 0,
    ADSB_TC_CATEGORY_AIRCRAFT_IDENTIFICATION,
    ADSB_TC_CATEGORY_SURFACE_POSITION,
    ADSB_TC_CATEGORY_AIRBORNE_POSITION_BARO,
    ADSB_TC_CATEGORY_AIRBORNE_VELOCITIES,
    ADSB_TC_CATEGORY_AIRBORNE_POSITION_GNSS,
    ADSB_TC_CATEGORY_RESERVED,
    ADSB_TC_CATEGORY_AIRCRAFT_STATUS,
    ADSB_TC_CATEGORY_TARGET_STATE_AND_STATUS,
    ADSB_TC_CATEGORY_AIRCRAFT_OPERATION_STATUS
} adsb_tc_category_t;

static void handle_aircraft_identification(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void handle_surface_position(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void handle_airborne_position_baro(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void handle_airborne_velocities(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void handle_airborne_position_gnss(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void handle_reserved(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void handle_aircraft_status(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void handle_target_state_and_status(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void handle_aircraft_operation_status(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void handle_unknown_tc(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc);
static void dispatch_tc_category(adsb_tc_category_t category,
                                      uint8_t df,
                                      uint8_t ca,
                                      uint32_t icao,
                                      uint64_t me,
                                      uint32_t pi,
                                      uint8_t tc);
static char callsign_char(uint8_t code);
static char* get_callsign(uint64_t me);
static size_t search_aircraft_by_icao(uint32_t icao);
static uint32_t me_get_u32(uint64_t me, unsigned int start_bit, unsigned int bit_len);
static int cpr_nl(double lat, uint16_t n_z);

static void print_aircraft_info(size_t index) {
    if (index == (size_t)-1) return;
    aircraft *entry = &aircrafts[index];
    uint32_t timestamp = (uint32_t)time(NULL);
    printf("[adsb] [timestamp=%u] - icao=%06X alt=%u lat=%.6f lon=%.6f vel=%u callsign=%s category=%s\n",
           timestamp, entry->icao, entry->altitude, entry->latitude, entry->longitude, entry->velocity, entry->callsign, entry->category);
}

static adsb_tc_category_t tc_category_from_value(uint8_t tc) {
    if (tc >= 1u && tc <= 4u) return ADSB_TC_CATEGORY_AIRCRAFT_IDENTIFICATION;
    if (tc >= 5u && tc <= 8u) return ADSB_TC_CATEGORY_SURFACE_POSITION;
    if (tc >= 9u && tc <= 18u) return ADSB_TC_CATEGORY_AIRBORNE_POSITION_BARO;
    if (tc == 19u) return ADSB_TC_CATEGORY_AIRBORNE_VELOCITIES;
    if (tc >= 20u && tc <= 22u) return ADSB_TC_CATEGORY_AIRBORNE_POSITION_GNSS;
    if (tc >= 23u && tc <= 27u) return ADSB_TC_CATEGORY_RESERVED;
    if (tc == 28u) return ADSB_TC_CATEGORY_AIRCRAFT_STATUS;
    if (tc == 29u) return ADSB_TC_CATEGORY_TARGET_STATE_AND_STATUS;
    if (tc == 31u) return ADSB_TC_CATEGORY_AIRCRAFT_OPERATION_STATUS;
    return ADSB_TC_CATEGORY_UNKNOWN;
}

static size_t search_aircraft_by_icao(uint32_t icao) {
    for (size_t i = 0; i < num_aircrafts; i++) {
        if (aircrafts[i].icao == icao) {
            return i;
        }
    }
    return (size_t)-1;
}

static uint32_t me_get_u32(uint64_t me, unsigned int start_bit, unsigned int bit_len) {
    uint64_t mask;
    unsigned int shift;

    if (bit_len == 0u || bit_len > 32u) return 0u;
    if (start_bit + bit_len > ADSB_ME_BITS) return 0u;

    shift = ADSB_ME_BITS - start_bit - bit_len;
    mask = ((uint64_t)1u << bit_len) - 1u;
    return (uint32_t)((me >> shift) & mask);
}

static int cpr_nl(double lat, uint16_t n_z) {
    double abs_lat = fabs(lat);
    double denom;
    double a;
    double b;
    double arg;
    int nl;

    if (abs_lat >= 87.0) return 1;

    a = 1.0 - cos(M_PI / (2.0 * (double)n_z));
    b = cos(M_PI / 180.0 * abs_lat);
    if (b == 0.0) return 1;

    arg = 1.0 - (a / (b * b));
    if (arg < -1.0) arg = -1.0;
    if (arg > 1.0) arg = 1.0;

    denom = acos(arg);
    if (denom <= 0.0) return 1;

    nl = (int)floor((2.0 * M_PI) / denom);
    return (nl < 1) ? 1 : nl;
}

static void update_aircraft(
    aircraft *entry,
    uint32_t icao,
    uint32_t altitude,
    double latitude,
    double longitude,
    uint32_t velocity,
    const char *callsign,
    const char *category,
    double lat_cpr_odd,
    double lat_cpr_even,
    double lon_cpr_odd,
    double lon_cpr_even,
    uint32_t time_even,
    uint32_t time_odd
    ) {
    if (entry == NULL) return;

    if (icao != 0u) entry->icao = icao;
    if (altitude != 0u) entry->altitude = altitude;
    if (latitude != 0u) entry->latitude = latitude;
    if (longitude != 0u) entry->longitude = longitude;
    if (velocity != 0u) entry->velocity = velocity;
    if (lat_cpr_odd != 0u) entry->lat_cpr_odd = lat_cpr_odd;
    if (lat_cpr_even != 0u) entry->lat_cpr_even = lat_cpr_even;
    if (lon_cpr_odd != 0u) entry->lon_cpr_odd = lon_cpr_odd;
    if (lon_cpr_even != 0u) entry->lon_cpr_even = lon_cpr_even;

    if (callsign != NULL && callsign[0] != '\0') {
        strncpy(entry->callsign, callsign, sizeof(entry->callsign) - 1u);
        entry->callsign[sizeof(entry->callsign) - 1u] = '\0';
    }

    if (category != NULL) entry->category = category;
    if (time_even != 0u) entry->time_even = time_even;
    if (time_odd != 0u) entry->time_odd = time_odd;
}

static size_t add_aircraft(
    uint32_t icao,
    uint32_t altitude,
    double latitude,
    double longitude,
    uint32_t velocity,
    const char *callsign,
    const char *category,
    double lat_cpr_odd,
    double lat_cpr_even,
    double lon_cpr_odd,
    double lon_cpr_even,
    uint32_t time_even,
    uint32_t time_odd
    ) {
    aircraft *entry;

    if (num_aircrafts >= sizeof(aircrafts) / sizeof(aircrafts[0])) return (size_t)-1;
    size_t index_current_aircraft = num_aircrafts;
    entry = &aircrafts[num_aircrafts];
    memset(entry, 0, sizeof(*entry));
    update_aircraft(entry, icao, altitude, latitude, longitude, velocity, callsign, category, lat_cpr_odd, lat_cpr_even, lon_cpr_odd, lon_cpr_even, time_even, time_odd);

    num_aircrafts++;
    return index_current_aircraft;
}

size_t add_or_update_aircraft(
    uint32_t icao,
    uint32_t altitude,
    double latitude,
    double longitude,
    uint32_t velocity,
    const char *callsign,
    const char *category,
    double lat_cpr_odd,
    double lat_cpr_even,
    double lon_cpr_odd,
    double lon_cpr_even,
    uint32_t time_even,
    uint32_t time_odd
    ) {
    size_t index = search_aircraft_by_icao(icao);
    if (index != (size_t)-1) {
        update_aircraft(&aircrafts[index], icao, altitude, latitude, longitude, velocity, callsign, category, lat_cpr_odd, lat_cpr_even, lon_cpr_odd, lon_cpr_even, time_even, time_odd);
        return index;
    } else {
        return add_aircraft(icao, altitude, latitude, longitude, velocity, callsign, category, lat_cpr_odd, lat_cpr_even, lon_cpr_odd, lon_cpr_even, time_even, time_odd);
    }
}

static void dispatch_tc_category(adsb_tc_category_t category,
                                      uint8_t df,
                                      uint8_t ca,
                                      uint32_t icao,
                                      uint64_t me,
                                      uint32_t pi,
                                      uint8_t tc) {
    switch (category) {
    case ADSB_TC_CATEGORY_AIRCRAFT_IDENTIFICATION:
        handle_aircraft_identification(df, ca, icao, me, pi, tc);
        break;
    case ADSB_TC_CATEGORY_SURFACE_POSITION:
        handle_surface_position(df, ca, icao, me, pi, tc);
        break;
    case ADSB_TC_CATEGORY_AIRBORNE_POSITION_BARO:
        handle_airborne_position_baro(df, ca, icao, me, pi, tc);
        break;
    case ADSB_TC_CATEGORY_AIRBORNE_VELOCITIES:
        handle_airborne_velocities(df, ca, icao, me, pi, tc);
        break;
    case ADSB_TC_CATEGORY_AIRBORNE_POSITION_GNSS:
        handle_airborne_position_gnss(df, ca, icao, me, pi, tc);
        break;
    case ADSB_TC_CATEGORY_RESERVED:
        handle_reserved(df, ca, icao, me, pi, tc);
        break;
    case ADSB_TC_CATEGORY_AIRCRAFT_STATUS:
        handle_aircraft_status(df, ca, icao, me, pi, tc);
        break;
    case ADSB_TC_CATEGORY_TARGET_STATE_AND_STATUS:
        handle_target_state_and_status(df, ca, icao, me, pi, tc);
        break;
    case ADSB_TC_CATEGORY_AIRCRAFT_OPERATION_STATUS:
        handle_aircraft_operation_status(df, ca, icao, me, pi, tc);
        break;
    case ADSB_TC_CATEGORY_UNKNOWN:
    default:
        handle_unknown_tc(df, ca, icao, me, pi, tc);
        break;
    }
}

static char callsign_char(uint8_t code) {
    if (code >= 1u && code <= 26u) return (char)('A' + (code - 1u));
    if (code >= 48u && code <= 57u) return (char)code;
    if (code == 32u) return ' ';
    return ' ';
}

static char* get_callsign(uint64_t me) {
    static char callsign[9];
    size_t n;

    n = (sizeof(callsign) > 0u) ? (sizeof(callsign) - 1u) : 0u;
    if (n > 8u) n = 8u;

    for (size_t i = 0; i < n; i++) {
        uint8_t code = (uint8_t)me_get_u32(me, 8u + (unsigned int)(i * 6u), 6u);
        callsign[i] = callsign_char(code);
    }
    callsign[n] = '\0';

    while (n > 0u && callsign[n - 1u] == ' ') {
        callsign[n - 1u] = '\0';
        n--;
    }
    return callsign;
}

void protocol_handle_message(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi) {
    uint8_t tc;
    adsb_tc_category_t category;

    tc = (me >> 51) & 0x1F;     // 5 bit

    if (df != 17u && df != 18u) {
        // printf("[adsb][proto] df=%u ca=%u icao=%06X me=%014llX pi=%06X ignored: tc is meaningful here only for DF17/DF18\n",
        //        df,
        //        ca,
        //        icao,
        //        (unsigned long long)me,
        //        pi);
        return;
    }

    category = tc_category_from_value(tc);
    dispatch_tc_category(category, df, ca, icao, me, pi, tc);
    size_t index = search_aircraft_by_icao(icao);
    
    print_aircraft_info(index);

}

static const char *get_category(uint8_t tc, uint8_t ca) {
    if (tc > 4u || ca > 7u) return "Unknown";
    return k_aircraft_category[tc][ca];
}

static void handle_aircraft_identification(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    const char *category = get_category(tc, ca);

    char* callsign = get_callsign(me);
    add_or_update_aircraft(icao, 0u, 0u, 0u, 0u, callsign, category, 0u, 0u, 0u, 0u, 0u, 0u);

    // printf("[adsb][ident] df=%u ca=%u icao=%06X category=%s callsign=%s pi=%06X\n",
    //        df,
    //        ca,
    //        icao,
    //        category,
    //        callsign[0] ? callsign : "<empty>",
    //        pi);
}

double calc_latitude(uint16_t n_z, double lat_cpr_even, double lat_cpr_odd, uint32_t time_even, uint32_t time_odd) {
    double lat = 0;
    double dlat_even = 360.0 / (4 * n_z);
    double dlat_odd = 360.0 / (4 * n_z - 1);
    int nl_lat_even;
    int nl_lat_odd;
    // latitude zone index
    double j = floor(59 * lat_cpr_even - 60 * lat_cpr_odd + 0.5);

    double lat_even = dlat_even * (positive_mod(j, 60.0) + lat_cpr_even);
    double lat_odd = dlat_odd * (positive_mod(j, 59.0) + lat_cpr_odd);

    if (lat_even >= 270) lat_even -= 360;
    if (lat_odd >= 270) lat_odd -= 360;

    nl_lat_even = cpr_nl(lat_even, n_z);
    nl_lat_odd = cpr_nl(lat_odd, n_z);
    if (nl_lat_even != nl_lat_odd) {
        return 0;
    }
    if (time_even >= time_odd) {
        lat = lat_even;
    } else {
        lat = lat_odd;
    }
    return lat;
}

double calc_longitude(uint16_t n_z, double lat, double lon_cpr_even, double lon_cpr_odd, uint32_t time_even, uint32_t time_odd) {
    double lon = 0;
    int nl_lat = cpr_nl(lat, n_z);
    // longitude zone index
    double m = floor(lon_cpr_even * (nl_lat - 1) - lon_cpr_odd * nl_lat + 0.5);
    // numbers of longitude zones
    double n_even = (nl_lat < 1) ? 1.0 : (double)nl_lat;
    double n_odd = (nl_lat - 1 < 1) ? 1.0 : (double)(nl_lat - 1);
    // longitude zone size
    double dlon_even = 360.0 / n_even;
    double dlon_odd = 360.0 / n_odd;
    // longitude
    double lon_even = dlon_even * (positive_mod(m, n_even) + lon_cpr_even);
    double lon_odd = dlon_odd * (positive_mod(m, n_odd) + lon_cpr_odd);
    if (time_even >= time_odd) {
        lon = lon_even;
    } else {
        lon = lon_odd;
    }
    if (lon >= 180) lon -= 360;
    return lon;
}

void global_position_decoding(size_t index, uint16_t n_z, uint32_t icao) {
    double lat = 0;
    double lon = 0;
    //if I have lat I use Locally unambiguous position decoding
    if (index != (size_t)-1) {
        aircraft *entry = &aircrafts[index];
        uint32_t dt;

        if (entry->lat_cpr_even == 0.0 || entry->lat_cpr_odd == 0.0) return;
        if (entry->lon_cpr_even == 0.0 || entry->lon_cpr_odd == 0.0) return;
        if (entry->time_even == 0u || entry->time_odd == 0u) return;

        dt = (entry->time_even >= entry->time_odd)
            ? (entry->time_even - entry->time_odd)
            : (entry->time_odd - entry->time_even);
        if (dt > CPR_AIRBORNE_PAIR_MAX_DT) {
            return;
        }

        lat = calc_latitude(n_z, entry->lat_cpr_even, entry->lat_cpr_odd, entry->time_even, entry->time_odd);
        if (!isfinite(lat) || lat < -90.0 || lat > 90.0) {
            printf("[adsb][surface] Invalid latitude calculation for icao=%06X\n", icao);
            return;
        }

        lon = calc_longitude(n_z, lat, entry->lon_cpr_even, entry->lon_cpr_odd, entry->time_even, entry->time_odd);
        if (!isfinite(lon) || lon < -180.0 || lon > 180.0) {
            printf("[adsb][surface] Invalid longitude calculation for icao=%06X\n", icao);
            return;
        }

        add_or_update_aircraft(icao, 0u, lat, lon, 0u, NULL, NULL, 0u, 0u, 0u, 0u, 0u, 0u);
    }
}

void locally_unambiguous_position_decoding(aircraft *entry) {

}

static void handle_airborne_position_baro(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    uint8_t  ss      = (me >> 49) & 0x03;     // 2 bit
    uint8_t  saf     = (me >> 48) & 0x01;     // 1 bit
    uint16_t alt     = (me >> 36) & 0x0FFF;   // 12 bit
    uint8_t  t       = (me >> 35) & 0x01;     // 1 bit
    uint8_t  f       = (me >> 34) & 0x01;     // 1 bit
    double lat_cpr = (me >> 17) & 0x1FFFF;  // 17 bit
    double lon_cpr =  me        & 0x1FFFF;  // 17 bit

    // N_z represents the number of latitude zones between the equator and a pole. 
    // In Mode S, for surface position messages, N_z is 15.
    uint16_t n_z = 15;
    size_t index = search_aircraft_by_icao(icao);

    uint32_t timestamp = (uint32_t)time(NULL);

    printf(
        "[adsb][airborne position] df=%u ca=%u icao=%06X me=%014llX pi=%06X tc=%u ss=%u saf=%u alt=%u t=%u f=%u lat_cpr=%.5f lon_cpr=%.5f\n",
        df,
        ca,
        icao,
        (unsigned long long)me,
        pi,
        tc,
        ss,
        saf,
        alt,
        t,
        f,
        lat_cpr,
        lon_cpr
    );

    if (index != (size_t)-1) {
        // existing aircraft, check if we have a more recent position message
        aircraft *entry = &aircrafts[index];

        if (entry->latitude != 0u && entry->longitude != 0u) {
            //locally_unambiguous_position_decoding(entry);
        }

        if (f == 0 && entry->time_even >= timestamp) {
            // existing even message is more recent or same age, ignore this one
            return;
        }
        if (f == 1 && entry->time_odd >= timestamp) {
            // existing odd message is more recent or same age, ignore this one
            return;
        }
    }

    lat_cpr /= 131072.0;
    lon_cpr /= 131072.0;

    if (f == 0) {
        // even
        add_or_update_aircraft(icao, 0u, 0u, 0u, 0u, NULL, NULL,
                            0u, lat_cpr, 0u, lon_cpr,
                            timestamp, 0u);
    } else if (f == 1) {
        // odd
        add_or_update_aircraft(icao, 0u, 0u, 0u, 0u, NULL, NULL,
                            lat_cpr, 0u, lon_cpr, 0u,
                            0u, timestamp);
    }

    index = search_aircraft_by_icao(icao);
    global_position_decoding(index, n_z, icao);

}

static void handle_surface_position(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    (void)df;
    (void)ca;
    (void)icao;
    (void)me;
    (void)pi;
    (void)tc;
}

static void handle_airborne_velocities(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    (void)df;
    (void)ca;
    (void)icao;
    (void)me;
    (void)pi;
    (void)tc;
}

static void handle_airborne_position_gnss(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    (void)df;
    (void)ca;
    (void)icao;
    (void)me;
    (void)pi;
    (void)tc;
}

static void handle_reserved(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    (void)df;
    (void)ca;
    (void)icao;
    (void)me;
    (void)pi;
    (void)tc;
}

static void handle_aircraft_status(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    (void)df;
    (void)ca;
    (void)icao;
    (void)me;
    (void)pi;
    (void)tc;
}

static void handle_target_state_and_status(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    (void)df;
    (void)ca;
    (void)icao;
    (void)me;
    (void)pi;
    (void)tc;
}

static void handle_aircraft_operation_status(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    (void)df;
    (void)ca;
    (void)icao;
    (void)me;
    (void)pi;
    (void)tc;
}

static void handle_unknown_tc(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    (void)df;
    (void)ca;
    (void)icao;
    (void)me;
    (void)pi;
    (void)tc;
}
