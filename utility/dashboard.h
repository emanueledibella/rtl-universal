#pragma once

#include <stdint.h>

typedef struct {
    uint32_t icao;
    const char *callsign;
    const char *category;
    int altitude_ft;
    int has_altitude;
    double latitude;
    double longitude;
    int has_position;
    double speed_kt;
    int has_speed;
    double track_deg;
    int has_track;
    int vertical_rate_fpm;
    int has_vertical_rate;
    uint8_t adsb_version;
    int has_adsb_version;
} dashboard_adsb_update_t;

typedef struct {
    uint32_t mmsi;
    uint8_t message_type;
    char channel;
    const char *name;
    const char *callsign;
    uint8_t ship_type;
    int has_ship_type;
    double latitude;
    double longitude;
    int has_position;
    double speed_kt;
    int has_speed;
    double course_deg;
    int has_course;
    unsigned int heading_deg;
    int has_heading;
    unsigned int navigation_status;
    int has_navigation_status;
} dashboard_ais_update_t;

void dashboard_set_mode(const char *protocol);
void dashboard_configure_receiver(const char *protocol, double frequency_mhz,
                                  const char *gain, const char *device);
void dashboard_set_connection(const char *state);
void dashboard_update_stats(double signal_dbfs, double clipping_percent,
                            double frame_rate, double candidate_rate,
                            uint64_t valid, uint64_t crc_errors,
                            uint64_t non_adsb_candidates,
                            uint64_t quality_rejected,
                            uint64_t other_rejected);
void dashboard_update_adsb(const dashboard_adsb_update_t *update);
void dashboard_update_ais(const dashboard_ais_update_t *update);
void dashboard_force_render(void);
void dashboard_shutdown(void);
