#include "dashboard.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define DASHBOARD_MAX_ADSB 512u
#define DASHBOARD_MAX_AIS 512u
#define DASHBOARD_MAX_SONDE 64u
#define DASHBOARD_REDRAW_NS 250000000L
#define DASHBOARD_ADSB_TIMEOUT 120.0
#define DASHBOARD_AIS_TIMEOUT 600.0
#define DASHBOARD_SONDE_TIMEOUT 900.0

typedef struct {
    int active;
    uint32_t icao;
    uint8_t downlink_format;
    char callsign[9];
    char category[40];
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
    uint16_t squawk;
    int has_squawk;
    uint64_t messages;
    time_t last_seen;
} dashboard_adsb_row_t;

typedef struct {
    int active;
    uint32_t mmsi;
    uint8_t message_type;
    char channel;
    char name[32];
    char callsign[16];
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
    uint64_t messages;
    time_t last_seen;
} dashboard_ais_row_t;

typedef struct {
    int active;
    char serial[16];
    uint16_t frame_number;
    double battery_v;
    int has_battery;
    double latitude;
    double longitude;
    double altitude_m;
    double speed_mps;
    double heading_deg;
    double climb_mps;
    unsigned int satellites;
    int has_position;
    uint64_t messages;
    time_t last_seen;
} dashboard_sonde_row_t;

typedef struct {
    char protocol[16];
    char gain[32];
    char device[128];
    char connection[32];
    double frequency_mhz;
    double signal_dbfs;
    double clipping_percent;
    double frame_rate;
    double candidate_rate;
    uint64_t valid;
    uint64_t crc_errors;
    uint64_t non_adsb_candidates;
    uint64_t quality_rejected;
    uint64_t other_rejected;
    int have_stats;
    int initialized;
    int shutdown_registered;
    struct timespec last_render;
    dashboard_adsb_row_t adsb[DASHBOARD_MAX_ADSB];
    dashboard_ais_row_t ais[DASHBOARD_MAX_AIS];
    dashboard_sonde_row_t sonde[DASHBOARD_MAX_SONDE];
} dashboard_state_t;

static dashboard_state_t g_dashboard;

static int equals_icase(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b) {
        char ca = *a >= 'A' && *a <= 'Z' ? (char)(*a - 'A' + 'a') : *a;
        char cb = *b >= 'A' && *b <= 'Z' ? (char)(*b - 'A' + 'a') : *b;
        if (ca != cb) return 0;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static void copy_clean(char *destination, size_t size, const char *source) {
    size_t written = 0u;
    if (!destination || size == 0u) return;
    if (source) {
        while (*source && written + 1u < size) {
            unsigned char c = (unsigned char)*source++;
            destination[written++] = c >= 0x20u && c != 0x7Fu ? (char)c : ' ';
        }
    }
    destination[written] = '\0';
}

static double monotonic_delta(const struct timespec *newer,
                              const struct timespec *older) {
    return (double)(newer->tv_sec - older->tv_sec)
           + (double)(newer->tv_nsec - older->tv_nsec) / 1e9;
}

static void terminal_size(unsigned int *columns, unsigned int *rows) {
    struct winsize size;
    *columns = 120u;
    *rows = 30u;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0) {
        if (size.ws_col > 0u) *columns = size.ws_col;
        if (size.ws_row > 0u) *rows = size.ws_row;
    }
}

static void clear_line(void) {
    fputs("\033[2K", stdout);
}

static unsigned int age_seconds(time_t now, time_t then) {
    double age = difftime(now, then);
    if (age < 0.0) age = 0.0;
    if (age > 9999.0) age = 9999.0;
    return (unsigned int)age;
}

static size_t cleanup_and_count_adsb(time_t now) {
    size_t count = 0u;
    for (size_t i = 0; i < DASHBOARD_MAX_ADSB; i++) {
        if (g_dashboard.adsb[i].active
            && difftime(now, g_dashboard.adsb[i].last_seen) > DASHBOARD_ADSB_TIMEOUT) {
            memset(&g_dashboard.adsb[i], 0, sizeof(g_dashboard.adsb[i]));
        }
        if (g_dashboard.adsb[i].active) count++;
    }
    return count;
}

static size_t cleanup_and_count_ais(time_t now) {
    size_t count = 0u;
    for (size_t i = 0; i < DASHBOARD_MAX_AIS; i++) {
        if (g_dashboard.ais[i].active
            && difftime(now, g_dashboard.ais[i].last_seen) > DASHBOARD_AIS_TIMEOUT) {
            memset(&g_dashboard.ais[i], 0, sizeof(g_dashboard.ais[i]));
        }
        if (g_dashboard.ais[i].active) count++;
    }
    return count;
}

static size_t cleanup_and_count_sonde(time_t now) {
    size_t count = 0u;
    for (size_t i = 0u; i < DASHBOARD_MAX_SONDE; i++) {
        if (g_dashboard.sonde[i].active
            && difftime(now, g_dashboard.sonde[i].last_seen) > DASHBOARD_SONDE_TIMEOUT) {
            memset(&g_dashboard.sonde[i], 0, sizeof(g_dashboard.sonde[i]));
        }
        if (g_dashboard.sonde[i].active) count++;
    }
    return count;
}

static int newest_adsb_index(const unsigned char *used) {
    int best = -1;
    for (size_t i = 0; i < DASHBOARD_MAX_ADSB; i++) {
        if (!g_dashboard.adsb[i].active || used[i]) continue;
        if (best < 0 || g_dashboard.adsb[i].last_seen > g_dashboard.adsb[best].last_seen) {
            best = (int)i;
        }
    }
    return best;
}

static int newest_ais_index(const unsigned char *used) {
    int best = -1;
    for (size_t i = 0; i < DASHBOARD_MAX_AIS; i++) {
        if (!g_dashboard.ais[i].active || used[i]) continue;
        if (best < 0 || g_dashboard.ais[i].last_seen > g_dashboard.ais[best].last_seen) {
            best = (int)i;
        }
    }
    return best;
}

static int newest_sonde_index(const unsigned char *used) {
    int best = -1;
    for (size_t i = 0u; i < DASHBOARD_MAX_SONDE; i++) {
        if (!g_dashboard.sonde[i].active || used[i]) continue;
        if (best < 0 || g_dashboard.sonde[i].last_seen > g_dashboard.sonde[best].last_seen) {
            best = (int)i;
        }
    }
    return best;
}

static void render_adsb_rows(unsigned int columns, unsigned int max_rows,
                             time_t now) {
    unsigned char used[DASHBOARD_MAX_ADSB] = { 0 };
    clear_line();
    if (columns >= 112u) {
        fputs("  ICAO    DF SQWK  VOLO      QUOTA    SPD    TRK   V/RATE      LAT        LON       MSG   AGE\r\n", stdout);
    } else {
        fputs("  ICAO    DF SQWK  VOLO      QUOTA    SPD    TRK      MSG   AGE\r\n", stdout);
    }
    for (unsigned int row = 0u; row < max_rows; row++) {
        int index = newest_adsb_index(used);
        dashboard_adsb_row_t *item;
        unsigned int age;
        char stale;
        if (index < 0) break;
        used[index] = 1u;
        item = &g_dashboard.adsb[index];
        age = age_seconds(now, item->last_seen);
        stale = age > 30u ? '!' : ' ';
        clear_line();
        if (columns >= 112u) {
            printf("%c %06X  %2u ", stale, item->icao, item->downlink_format);
            if (item->has_squawk) printf("%04o  ", item->squawk); else printf("   -  ");
            printf("%-8.8s  ",
                   item->callsign[0] ? item->callsign : "-");
            if (item->has_altitude) printf("%7d", item->altitude_ft); else printf("%7s", "-");
            if (item->has_speed) printf("  %5.0f", item->speed_kt); else printf("  %5s", "-");
            if (item->has_track) printf("  %5.1f", item->track_deg); else printf("  %5s", "-");
            if (item->has_vertical_rate) printf("  %7d", item->vertical_rate_fpm); else printf("  %7s", "-");
            if (item->has_position) printf("  %9.5f  %10.5f", item->latitude, item->longitude);
            else printf("  %9s  %10s", "-", "-");
            printf("  %6llu  %3us\r\n", (unsigned long long)item->messages, age);
        } else {
            printf("%c %06X  %2u ", stale, item->icao, item->downlink_format);
            if (item->has_squawk) printf("%04o  ", item->squawk); else printf("   -  ");
            printf("%-8.8s  ",
                   item->callsign[0] ? item->callsign : "-");
            if (item->has_altitude) printf("%7d", item->altitude_ft); else printf("%7s", "-");
            if (item->has_speed) printf("  %5.0f", item->speed_kt); else printf("  %5s", "-");
            if (item->has_track) printf("  %5.1f", item->track_deg); else printf("  %5s", "-");
            printf("  %6llu  %3us\r\n", (unsigned long long)item->messages, age);
        }
    }
}

static void render_ais_rows(unsigned int columns, unsigned int max_rows,
                            time_t now) {
    unsigned char used[DASHBOARD_MAX_AIS] = { 0 };
    clear_line();
    if (columns >= 112u) {
        fputs("  MMSI       NOME                 TIPO CH   SOG    COG   HDG      LAT        LON       MSG   AGE\r\n", stdout);
    } else {
        fputs("  MMSI       NOME                 TIPO CH   SOG    COG      MSG   AGE\r\n", stdout);
    }
    for (unsigned int row = 0u; row < max_rows; row++) {
        int index = newest_ais_index(used);
        dashboard_ais_row_t *item;
        unsigned int age;
        char stale;
        if (index < 0) break;
        used[index] = 1u;
        item = &g_dashboard.ais[index];
        age = age_seconds(now, item->last_seen);
        stale = age > 30u ? '!' : ' ';
        clear_line();
        printf("%c %09u  %-20.20s  %4u  %c  ", stale, item->mmsi,
               item->name[0] ? item->name : item->callsign[0] ? item->callsign : "-",
               item->message_type, item->channel ? item->channel : '-');
        if (item->has_speed) printf("%5.1f", item->speed_kt); else printf("%5s", "-");
        if (item->has_course) printf("  %5.1f", item->course_deg); else printf("  %5s", "-");
        if (columns >= 112u) {
            if (item->has_heading) printf("  %4u", item->heading_deg); else printf("  %4s", "-");
            if (item->has_position) printf("  %9.5f  %10.5f", item->latitude, item->longitude);
            else printf("  %9s  %10s", "-", "-");
        }
        printf("  %6llu  %3us\r\n", (unsigned long long)item->messages, age);
    }
}

static void render_sonde_rows(unsigned int columns, unsigned int max_rows,
                              time_t now) {
    unsigned char used[DASHBOARD_MAX_SONDE] = { 0 };
    clear_line();
    if (columns >= 112u) {
        fputs("  SERIALE       FRAME   BATT      QUOTA   SPD   DIR  CLIMB SAT      LAT        LON       MSG   AGE\r\n", stdout);
    } else {
        fputs("  SERIALE       FRAME   BATT      QUOTA   SPD  CLIMB SAT      MSG   AGE\r\n", stdout);
    }
    for (unsigned int row = 0u; row < max_rows; row++) {
        int index = newest_sonde_index(used);
        dashboard_sonde_row_t *item;
        unsigned int age;
        char stale;
        if (index < 0) break;
        used[index] = 1u;
        item = &g_dashboard.sonde[index];
        age = age_seconds(now, item->last_seen);
        stale = age > 30u ? '!' : ' ';
        clear_line();
        printf("%c %-12.12s  %5u  ", stale, item->serial, item->frame_number);
        if (item->has_battery) printf("%4.1fV", item->battery_v); else printf("%5s", "-");
        if (item->has_position) {
            printf("  %8.0f %5.1f", item->altitude_m, item->speed_mps);
            if (columns >= 112u) printf(" %5.0f", item->heading_deg);
            printf(" %6.1f %3u", item->climb_mps, item->satellites);
            if (columns >= 112u) {
                printf("  %9.5f  %10.5f", item->latitude, item->longitude);
            }
        } else {
            printf("  %8s %5s", "-", "-");
            if (columns >= 112u) printf(" %5s", "-");
            printf(" %6s %3s", "-", "-");
            if (columns >= 112u) printf("  %9s  %10s", "-", "-");
        }
        printf("  %6llu  %3us\r\n", (unsigned long long)item->messages, age);
    }
}

static void dashboard_render(int force) {
    struct timespec now_mono;
    time_t now_wall;
    unsigned int columns;
    unsigned int rows;
    unsigned int available_rows;
    size_t tracked;

    if (clock_gettime(CLOCK_MONOTONIC, &now_mono) != 0) return;
    if (!force && g_dashboard.initialized
        && monotonic_delta(&now_mono, &g_dashboard.last_render)
               < (double)DASHBOARD_REDRAW_NS / 1e9) return;
    if (!g_dashboard.shutdown_registered) {
        (void)atexit(dashboard_shutdown);
        g_dashboard.shutdown_registered = 1;
    }
    if (!g_dashboard.initialized) {
        fputs("\033[?25l\033[2J", stdout);
        g_dashboard.initialized = 1;
    }
    g_dashboard.last_render = now_mono;
    now_wall = time(NULL);
    terminal_size(&columns, &rows);
    available_rows = rows > 8u ? rows - 8u : 1u;
    if (equals_icase(g_dashboard.protocol, "ais")) {
        tracked = cleanup_and_count_ais(now_wall);
    } else if (equals_icase(g_dashboard.protocol, "sonde")
               || equals_icase(g_dashboard.protocol, "rs41")) {
        tracked = cleanup_and_count_sonde(now_wall);
    } else {
        tracked = cleanup_and_count_adsb(now_wall);
    }

    fputs("\033[H", stdout);
    clear_line();
    printf("\033[1mrtl-universal | %s | stato=%s | elementi=%zu\033[0m\r\n",
           g_dashboard.protocol[0] ? g_dashboard.protocol : "receiver",
           g_dashboard.connection[0] ? g_dashboard.connection : "offline", tracked);
    clear_line();
    printf("device=%s | freq=", g_dashboard.device[0] ? g_dashboard.device : "-");
    if (g_dashboard.frequency_mhz > 0.0) printf("%.3f MHz", g_dashboard.frequency_mhz);
    else fputs("offline", stdout);
    printf(" | gain=%s\r\n", g_dashboard.gain[0] ? g_dashboard.gain : "-");
    clear_line();
    if (g_dashboard.have_stats) {
        printf("signal=%.1f dBFS | clip=%.2f%% | frame=%.2f/s | valid=%llu | CRC=%llu",
               g_dashboard.signal_dbfs, g_dashboard.clipping_percent,
               g_dashboard.frame_rate, (unsigned long long)g_dashboard.valid,
               (unsigned long long)g_dashboard.crc_errors);
        if (equals_icase(g_dashboard.protocol, "adsb")) {
            printf(" | candidati=%.1f/s | non-ADSB=%llu | qualita=%llu",
                   g_dashboard.candidate_rate,
                   (unsigned long long)g_dashboard.non_adsb_candidates,
                   (unsigned long long)g_dashboard.quality_rejected);
        } else {
            printf(" | altri scarti=%llu",
                   (unsigned long long)g_dashboard.other_rejected);
        }
    } else {
        fputs("in attesa dei primi dati...", stdout);
    }
    fputs("\r\n", stdout);
    clear_line();
    fputs("--------------------------------------------------------------------------------\r\n", stdout);

    if (equals_icase(g_dashboard.protocol, "ais")) {
        render_ais_rows(columns, available_rows, now_wall);
    } else if (equals_icase(g_dashboard.protocol, "sonde")
               || equals_icase(g_dashboard.protocol, "rs41")) {
        render_sonde_rows(columns, available_rows, now_wall);
    } else {
        render_adsb_rows(columns, available_rows, now_wall);
    }
    clear_line();
    fputs("--------------------------------------------------------------------------------\r\n", stdout);
    clear_line();
    fputs("Ctrl+C per uscire | usa --output log per il flusso dettagliato", stdout);
    fputs("\033[J", stdout);
    fflush(stdout);
}

void dashboard_set_mode(const char *protocol) {
    copy_clean(g_dashboard.protocol, sizeof(g_dashboard.protocol), protocol);
}

void dashboard_configure_receiver(const char *protocol, double frequency_mhz,
                                  const char *gain, const char *device) {
    dashboard_set_mode(protocol);
    g_dashboard.frequency_mhz = frequency_mhz;
    copy_clean(g_dashboard.gain, sizeof(g_dashboard.gain), gain);
    copy_clean(g_dashboard.device, sizeof(g_dashboard.device), device);
    dashboard_render(1);
}

void dashboard_set_connection(const char *state) {
    copy_clean(g_dashboard.connection, sizeof(g_dashboard.connection), state);
    dashboard_render(1);
}

void dashboard_update_stats(double signal_dbfs, double clipping_percent,
                            double frame_rate, double candidate_rate,
                            uint64_t valid, uint64_t crc_errors,
                            uint64_t non_adsb_candidates,
                            uint64_t quality_rejected,
                            uint64_t other_rejected) {
    g_dashboard.signal_dbfs = signal_dbfs;
    g_dashboard.clipping_percent = clipping_percent;
    g_dashboard.frame_rate = frame_rate;
    g_dashboard.candidate_rate = candidate_rate;
    g_dashboard.valid = valid;
    g_dashboard.crc_errors = crc_errors;
    g_dashboard.non_adsb_candidates = non_adsb_candidates;
    g_dashboard.quality_rejected = quality_rejected;
    g_dashboard.other_rejected = other_rejected;
    g_dashboard.have_stats = 1;
    dashboard_render(1);
}

static dashboard_adsb_row_t *get_adsb_row(uint32_t icao) {
    dashboard_adsb_row_t *oldest = NULL;
    dashboard_adsb_row_t *empty = NULL;
    for (size_t i = 0; i < DASHBOARD_MAX_ADSB; i++) {
        dashboard_adsb_row_t *row = &g_dashboard.adsb[i];
        if (row->active && row->icao == icao) return row;
        if (!row->active && !empty) empty = row;
        if (!oldest || row->last_seen < oldest->last_seen) oldest = row;
    }
    return empty ? empty : oldest;
}

void dashboard_update_adsb(const dashboard_adsb_update_t *update) {
    dashboard_adsb_row_t *row;
    if (!update || update->icao == 0u) return;
    dashboard_set_mode("ADSB");
    row = get_adsb_row(update->icao);
    if (!row) return;
    if (!row->active || row->icao != update->icao) memset(row, 0, sizeof(*row));
    row->active = 1;
    row->icao = update->icao;
    row->downlink_format = update->downlink_format;
    if (update->callsign && update->callsign[0]) {
        copy_clean(row->callsign, sizeof(row->callsign), update->callsign);
    }
    if (update->category && update->category[0]) {
        copy_clean(row->category, sizeof(row->category), update->category);
    }
    if (update->has_altitude) {
        row->altitude_ft = update->altitude_ft;
        row->has_altitude = 1;
    }
    if (update->has_position) {
        row->latitude = update->latitude;
        row->longitude = update->longitude;
        row->has_position = 1;
    }
    if (update->has_speed) {
        row->speed_kt = update->speed_kt;
        row->has_speed = 1;
    }
    if (update->has_track) {
        row->track_deg = update->track_deg;
        row->has_track = 1;
    }
    if (update->has_vertical_rate) {
        row->vertical_rate_fpm = update->vertical_rate_fpm;
        row->has_vertical_rate = 1;
    }
    if (update->has_adsb_version) {
        row->adsb_version = update->adsb_version;
        row->has_adsb_version = 1;
    }
    if (update->has_squawk) {
        row->squawk = update->squawk;
        row->has_squawk = 1;
    }
    row->messages++;
    row->last_seen = time(NULL);
    dashboard_render(0);
}

static dashboard_ais_row_t *get_ais_row(uint32_t mmsi) {
    dashboard_ais_row_t *oldest = NULL;
    dashboard_ais_row_t *empty = NULL;
    for (size_t i = 0; i < DASHBOARD_MAX_AIS; i++) {
        dashboard_ais_row_t *row = &g_dashboard.ais[i];
        if (row->active && row->mmsi == mmsi) return row;
        if (!row->active && !empty) empty = row;
        if (!oldest || row->last_seen < oldest->last_seen) oldest = row;
    }
    return empty ? empty : oldest;
}

void dashboard_update_ais(const dashboard_ais_update_t *update) {
    dashboard_ais_row_t *row;
    if (!update || update->mmsi == 0u) return;
    dashboard_set_mode("AIS");
    row = get_ais_row(update->mmsi);
    if (!row) return;
    if (!row->active || row->mmsi != update->mmsi) memset(row, 0, sizeof(*row));
    row->active = 1;
    row->mmsi = update->mmsi;
    row->message_type = update->message_type;
    if (update->channel == 'A' || update->channel == 'B') row->channel = update->channel;
    if (update->name && update->name[0]) copy_clean(row->name, sizeof(row->name), update->name);
    if (update->callsign && update->callsign[0]) {
        copy_clean(row->callsign, sizeof(row->callsign), update->callsign);
    }
    if (update->has_ship_type) {
        row->ship_type = update->ship_type;
        row->has_ship_type = 1;
    }
    if (update->has_position) {
        row->latitude = update->latitude;
        row->longitude = update->longitude;
        row->has_position = 1;
    }
    if (update->has_speed) {
        row->speed_kt = update->speed_kt;
        row->has_speed = 1;
    }
    if (update->has_course) {
        row->course_deg = update->course_deg;
        row->has_course = 1;
    }
    if (update->has_heading) {
        row->heading_deg = update->heading_deg;
        row->has_heading = 1;
    }
    if (update->has_navigation_status) {
        row->navigation_status = update->navigation_status;
        row->has_navigation_status = 1;
    }
    row->messages++;
    row->last_seen = time(NULL);
    dashboard_render(0);
}

static dashboard_sonde_row_t *get_sonde_row(const char *serial) {
    dashboard_sonde_row_t *oldest = NULL;
    dashboard_sonde_row_t *empty = NULL;
    for (size_t i = 0u; i < DASHBOARD_MAX_SONDE; i++) {
        dashboard_sonde_row_t *row = &g_dashboard.sonde[i];
        if (row->active && strcmp(row->serial, serial) == 0) return row;
        if (!row->active && !empty) empty = row;
        if (!oldest || row->last_seen < oldest->last_seen) oldest = row;
    }
    return empty ? empty : oldest;
}

void dashboard_update_sonde(const dashboard_sonde_update_t *update) {
    dashboard_sonde_row_t *row;
    if (!update || !update->serial || update->serial[0] == '\0') return;
    dashboard_set_mode("SONDE");
    row = get_sonde_row(update->serial);
    if (!row) return;
    if (!row->active || strcmp(row->serial, update->serial) != 0) {
        memset(row, 0, sizeof(*row));
    }
    row->active = 1;
    copy_clean(row->serial, sizeof(row->serial), update->serial);
    row->frame_number = update->frame_number;
    if (update->has_battery) {
        row->battery_v = update->battery_v;
        row->has_battery = 1;
    }
    if (update->has_position) {
        row->latitude = update->latitude;
        row->longitude = update->longitude;
        row->altitude_m = update->altitude_m;
        row->speed_mps = update->speed_mps;
        row->heading_deg = update->heading_deg;
        row->climb_mps = update->climb_mps;
        row->satellites = update->satellites;
        row->has_position = 1;
    }
    row->messages++;
    row->last_seen = time(NULL);
    dashboard_render(0);
}

void dashboard_force_render(void) {
    dashboard_render(1);
}

void dashboard_shutdown(void) {
    if (!g_dashboard.initialized) return;
    dashboard_render(1);
    fputs("\033[?25h\r\n", stdout);
    fflush(stdout);
    g_dashboard.initialized = 0;
}
