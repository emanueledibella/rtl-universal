#include <math.h>
#include <stdio.h>

#include "analog_frontend.h"

#define TEST_PI_F 3.14159265358979323846f
#define TEST_SAMPLE_RATE 240000
#define TEST_FILTER_WIDTH 12500

static float measure_filter_power(analog_filter_type_t type, float frequency_hz) {
    analog_frontend_config_t cfg = {type, TEST_FILTER_WIDTH, 0, 0.0f};
    analog_frontend_t frontend;
    float power = 0.0f;
    float filtered_i = 0.0f;
    float filtered_q = 0.0f;

    if (!analog_frontend_init(&frontend, &cfg, TEST_SAMPLE_RATE)) return -1.0f;
    for (int n = 0; n < 12000; n++) {
        float phase = 2.0f * TEST_PI_F * frequency_hz
                      * (float)n / (float)TEST_SAMPLE_RATE;
        (void)analog_frontend_process(&frontend,
                                      100.0f * cosf(phase),
                                      100.0f * sinf(phase),
                                      &filtered_i, &filtered_q);
        if (n >= 2000) {
            power += filtered_i * filtered_i + filtered_q * filtered_q;
        }
    }
    analog_frontend_destroy(&frontend);
    return power / 10000.0f;
}

static int test_filter(analog_filter_type_t type) {
    float passband_power = measure_filter_power(type, 2000.0f);
    float stopband_power = measure_filter_power(type, 30000.0f);

    return passband_power > 8000.0f
           && passband_power < 12000.0f
           && stopband_power >= 0.0f
           && stopband_power < passband_power * 0.001f;
}

static int test_squelch(void) {
    analog_frontend_config_t cfg = {ANALOG_FILTER_NONE, 0, 1, -20.0f};
    analog_frontend_t frontend;
    float filtered_i = 0.0f;
    float filtered_q = 0.0f;
    float level_dbfs = 0.0f;
    float threshold_dbfs = 0.0f;
    int reported_open = 0;
    int open = 0;

    if (!analog_frontend_init(&frontend, &cfg, 10000)) return 0;
    for (int n = 0; n < 1000; n++) {
        open = analog_frontend_process(&frontend, 0.0f, 0.0f,
                                       &filtered_i, &filtered_q);
    }
    if (open) {
        analog_frontend_destroy(&frontend);
        return 0;
    }
    for (int n = 0; n < 2000; n++) {
        open = analog_frontend_process(&frontend, 127.0f, 127.0f,
                                       &filtered_i, &filtered_q);
    }
    if (!open) {
        analog_frontend_destroy(&frontend);
        return 0;
    }
    if (!analog_frontend_get_squelch_status(&frontend, &level_dbfs,
                                            &threshold_dbfs, &reported_open)
        || !reported_open || threshold_dbfs != -20.0f || level_dbfs < -4.0f) {
        analog_frontend_destroy(&frontend);
        return 0;
    }
    for (int n = 0; n < 800; n++) {
        open = analog_frontend_process(&frontend, 0.0f, 0.0f,
                                       &filtered_i, &filtered_q);
    }
    analog_frontend_destroy(&frontend);
    return !open;
}

int main(void) {
    analog_frontend_config_t invalid = {ANALOG_FILTER_NONE, 1000, 0, 0.0f};
    analog_frontend_t frontend;

    if (analog_frontend_init(&frontend, &invalid, TEST_SAMPLE_RATE)) {
        fprintf(stderr, "analog frontend accepted an inconsistent filter config\n");
        analog_frontend_destroy(&frontend);
        return 1;
    }
    if (!test_filter(ANALOG_FILTER_FIR)) {
        fprintf(stderr, "FIR channel filter response test failed\n");
        return 1;
    }
    if (!test_filter(ANALOG_FILTER_IIR)) {
        fprintf(stderr, "IIR channel filter response test failed\n");
        return 1;
    }
    if (!test_squelch()) {
        fprintf(stderr, "squelch smoothing/closing test failed\n");
        return 1;
    }
    puts("[ANALOG] frontend filter/squelch test_result=PASS");
    return 0;
}
