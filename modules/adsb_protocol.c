#include "header/adsb_protocol.h"
#include "utility.h"

#include <stdio.h>
#include <string.h>

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
    uint8_t altitude;
    uint8_t latitude;
    uint8_t longitude;
    uint8_t velocity;
    char callsign[9];
    const char* category;
    uint16_t lat_cpr;
    uint16_t lon_cpr;
    uint32_t icao;
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

static void update_aircraft(
    aircraft *entry,
    uint32_t icao,
    uint8_t altitude,
    uint8_t latitude,
    uint8_t longitude,
    uint8_t velocity,
    const char *callsign,
    const char *category,
    uint16_t lat_cpr,
    uint16_t lon_cpr
    ) {
    if (entry == NULL) return;

    if (icao != 0u) entry->icao = icao;
    if (altitude != 0u) entry->altitude = altitude;
    if (latitude != 0u) entry->latitude = latitude;
    if (longitude != 0u) entry->longitude = longitude;
    if (velocity != 0u) entry->velocity = velocity;
    if (lat_cpr != 0u) entry->lat_cpr = lat_cpr;
    if (lon_cpr != 0u) entry->lon_cpr = lon_cpr;

    if (callsign != NULL && callsign[0] != '\0') {
        strncpy(entry->callsign, callsign, sizeof(entry->callsign) - 1u);
        entry->callsign[sizeof(entry->callsign) - 1u] = '\0';
    }

    if (category != NULL) entry->category = category;
}

static size_t add_aircraft(
    uint32_t icao,
    uint8_t altitude,
    uint8_t latitude,
    uint8_t longitude,
    uint8_t velocity,
    const char *callsign,
    const char *category,
    uint16_t lat_cpr,
    uint16_t lon_cpr
    ) {
    aircraft *entry;

    if (num_aircrafts >= sizeof(aircrafts) / sizeof(aircrafts[0])) return (size_t)-1;
    size_t index_current_aircraft = num_aircrafts;
    entry = &aircrafts[num_aircrafts];
    memset(entry, 0, sizeof(*entry));
    update_aircraft(entry, icao, altitude, latitude, longitude, velocity, callsign, category, lat_cpr, lon_cpr);

    num_aircrafts++;
    return index_current_aircraft;
}

size_t add_or_update_aircraft(
    uint32_t icao,
    uint8_t altitude,
    uint8_t latitude,
    uint8_t longitude,
    uint8_t velocity,
    const char *callsign,
    const char *category,
    uint16_t lat_cpr,
    uint16_t lon_cpr
    ) {
    size_t index = search_aircraft_by_icao(icao);
    if (index != (size_t)-1) {
        update_aircraft(&aircrafts[index], icao, altitude, latitude, longitude, velocity, callsign, category, lat_cpr, lon_cpr);
        return index;
    } else {
        return add_aircraft(icao, altitude, latitude, longitude, velocity, callsign, category, lat_cpr, lon_cpr);
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
        uint8_t code = (uint8_t)bits_get_u32((const uint8_t *)&me, 8 + (int)(i * 6u), 6);
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

    tc = (uint8_t)bits_get_u32((const uint8_t *)&me, 0, 5);

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
}

static const char *get_category(uint8_t tc, uint8_t ca) {
    if (tc > 4u || ca > 7u) return "Unknown";
    return k_aircraft_category[tc][ca];
}

static void handle_aircraft_identification(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    const char *category = get_category(tc, ca);

    char* callsign = get_callsign(me);
    add_or_update_aircraft(icao, 0u, 0u, 0u, 0u, callsign, category, 0u, 0u);

    printf("[adsb][ident] df=%u ca=%u icao=%06X category=%s callsign=%s pi=%06X\n",
           df,
           ca,
           icao,
           category,
           callsign[0] ? callsign : "<empty>",
           pi);
}

static void handle_surface_position(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
    uint8_t ss = (uint8_t)bits_get_u32((const uint8_t *)&me, 5, 2);
    uint8_t saf = (uint8_t)bits_get_u32((const uint8_t *)&me, 7, 1);
    uint8_t alt = (uint8_t)bits_get_u32((const uint8_t *)&me, 8, 12);
    uint8_t t = (uint8_t)bits_get_u32((const uint8_t *)&me, 20, 1);
    uint8_t f = (uint8_t)bits_get_u32((const uint8_t *)&me, 21, 1);
    uint8_t lat_cpr = (uint8_t)bits_get_u32((const uint8_t *)&me, 22, 17);
    uint8_t lon_cpr = (uint8_t)bits_get_u32((const uint8_t *)&me, 39, 17);
    // N_z represents the number of latitude zones between the equator and a pole. 
    // In Mode S, it is fixed to 15, but in ADS-B it can be 15 or 16 depending on the type of message. For surface position messages, N_z is 15.
    uint8_t n_z = 15;


}

static void handle_airborne_position_baro(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi, uint8_t tc) {
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
