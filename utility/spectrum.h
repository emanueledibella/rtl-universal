#pragma once

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct spectrum_analyzer spectrum_analyzer_t;

/*
 * Create a non-blocking spectrum tap for an RTL-SDR I/Q stream.
 *
 * The capture callback only copies an FFT-sized window at the requested
 * cadence. Windowing, FFT and JSON serialization run on a worker thread so a
 * slow UI cannot stall librtlsdr's asynchronous reader.
 */
spectrum_analyzer_t *spectrum_analyzer_create(int sample_rate,
                                              uint32_t center_frequency_hz,
                                              unsigned int fft_size,
                                              double frames_per_second,
                                              FILE *output);

void spectrum_analyzer_feed_u8(spectrum_analyzer_t *ctx,
                               const unsigned char *samples,
                               uint32_t length);

void spectrum_analyzer_destroy(spectrum_analyzer_t *ctx);

/* Exposed for deterministic tests and offline tooling. */
int spectrum_compute_dbfs(const unsigned char *samples,
                          unsigned int fft_size,
                          float *bins_dbfs);

#ifdef __cplusplus
}
#endif
