#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

typedef enum {
    OUTPUT_FORMAT_DASHBOARD = 0,
    OUTPUT_FORMAT_LOG,
    OUTPUT_FORMAT_JSON,
    OUTPUT_FORMAT_CSV,
    OUTPUT_FORMAT_AVR,
    OUTPUT_FORMAT_BEAST,
    OUTPUT_FORMAT_QUIET
} output_format_t;

int output_set_format_name(const char *name);
output_format_t output_get_format(void);
const char *output_format_name(output_format_t format);
int output_is_human(void);
int output_is_dashboard(void);
int output_is_log(void);
int output_is_structured(void);
int output_is_adsb_raw(void);
FILE *output_diagnostics(void);

void output_emit_adsb_raw(const uint8_t *frame, size_t frame_bits,
                          uint64_t timestamp_12mhz, uint8_t signal_level);
