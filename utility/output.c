#include "output.h"

#include <ctype.h>
#include <string.h>
#include <time.h>

static output_format_t g_output_format = OUTPUT_FORMAT_DASHBOARD;

static int equals_icase(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

int output_set_format_name(const char *name) {
    if (!name || equals_icase(name, "dashboard") || equals_icase(name, "table")
        || equals_icase(name, "tui") || equals_icase(name, "ui")) {
        g_output_format = OUTPUT_FORMAT_DASHBOARD;
    } else if (equals_icase(name, "log") || equals_icase(name, "text")) {
        g_output_format = OUTPUT_FORMAT_LOG;
    } else if (equals_icase(name, "json") || equals_icase(name, "jsonl")) {
        g_output_format = OUTPUT_FORMAT_JSON;
    } else if (equals_icase(name, "csv")) {
        g_output_format = OUTPUT_FORMAT_CSV;
    } else if (equals_icase(name, "avr") || equals_icase(name, "raw")) {
        g_output_format = OUTPUT_FORMAT_AVR;
    } else if (equals_icase(name, "beast")) {
        g_output_format = OUTPUT_FORMAT_BEAST;
    } else if (equals_icase(name, "quiet") || equals_icase(name, "none")) {
        g_output_format = OUTPUT_FORMAT_QUIET;
    } else {
        return 0;
    }
    return 1;
}

output_format_t output_get_format(void) {
    return g_output_format;
}

const char *output_format_name(output_format_t format) {
    switch (format) {
    case OUTPUT_FORMAT_DASHBOARD: return "dashboard";
    case OUTPUT_FORMAT_LOG: return "log";
    case OUTPUT_FORMAT_JSON: return "json";
    case OUTPUT_FORMAT_CSV: return "csv";
    case OUTPUT_FORMAT_AVR: return "avr";
    case OUTPUT_FORMAT_BEAST: return "beast";
    case OUTPUT_FORMAT_QUIET: return "quiet";
    default: return "dashboard";
    }
}

int output_is_human(void) {
    return g_output_format == OUTPUT_FORMAT_LOG;
}

int output_is_dashboard(void) {
    return g_output_format == OUTPUT_FORMAT_DASHBOARD;
}

int output_is_log(void) {
    return g_output_format == OUTPUT_FORMAT_LOG;
}

int output_is_structured(void) {
    return g_output_format == OUTPUT_FORMAT_JSON || g_output_format == OUTPUT_FORMAT_CSV;
}

int output_is_adsb_raw(void) {
    return g_output_format == OUTPUT_FORMAT_AVR || g_output_format == OUTPUT_FORMAT_BEAST;
}

FILE *output_diagnostics(void) {
    return g_output_format == OUTPUT_FORMAT_LOG ? stdout : stderr;
}

static void beast_put_escaped(uint8_t value) {
    (void)fputc((int)value, stdout);
    if (value == 0x1Au) (void)fputc(0x1A, stdout);
}

static uint64_t default_timestamp_12mhz(void) {
    struct timespec ts;
    uint64_t ticks;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0u;
    ticks = (uint64_t)ts.tv_sec * 12000000ULL;
    ticks += (uint64_t)ts.tv_nsec * 12ULL / 1000ULL;
    return ticks & 0xFFFFFFFFFFFFULL;
}

void output_emit_adsb_raw(const uint8_t *frame, size_t frame_bits,
                          uint64_t timestamp_12mhz, uint8_t signal_level) {
    size_t bytes;
    if (!frame || (frame_bits != 56u && frame_bits != 112u)) return;
    bytes = frame_bits / 8u;

    if (g_output_format == OUTPUT_FORMAT_AVR) {
        (void)fputc('*', stdout);
        for (size_t i = 0; i < bytes; i++) (void)fprintf(stdout, "%02X", frame[i]);
        (void)fputs(";\n", stdout);
        (void)fflush(stdout);
        return;
    }
    if (g_output_format != OUTPUT_FORMAT_BEAST) return;

    if (timestamp_12mhz == 0u) timestamp_12mhz = default_timestamp_12mhz();
    (void)fputc(0x1A, stdout);
    (void)fputc(frame_bits == 56u ? '2' : '3', stdout);
    for (int shift = 40; shift >= 0; shift -= 8) {
        beast_put_escaped((uint8_t)(timestamp_12mhz >> (unsigned int)shift));
    }
    beast_put_escaped(signal_level);
    for (size_t i = 0; i < bytes; i++) beast_put_escaped(frame[i]);
    (void)fflush(stdout);
}
