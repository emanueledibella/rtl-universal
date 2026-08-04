#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "spectrum.h"

#define FFT_SIZE 1024u

int main(void) {
    unsigned char samples[FFT_SIZE * 2u];
    float bins[FFT_SIZE];
    const unsigned int expected_bin = FFT_SIZE / 2u + 128u;
    unsigned int peak_bin = 0u;
    float peak = -1000.0f;

    for (unsigned int i = 0u; i < FFT_SIZE; i++) {
        double phase = 2.0 * M_PI * 128.0 * (double)i / (double)FFT_SIZE;
        samples[i * 2u] = (unsigned char)lrint(127.5 + 100.0 * cos(phase));
        samples[i * 2u + 1u] = (unsigned char)lrint(127.5 + 100.0 * sin(phase));
    }

    if (!spectrum_compute_dbfs(samples, FFT_SIZE, bins)) {
        fprintf(stderr, "spectrum FFT failed\n");
        return EXIT_FAILURE;
    }
    for (unsigned int i = 0u; i < FFT_SIZE; i++) {
        if (bins[i] > peak) {
            peak = bins[i];
            peak_bin = i;
        }
    }
    if (peak_bin != expected_bin || peak < -5.0f) {
        fprintf(stderr, "unexpected peak bin=%u level=%.1f expected=%u\n",
                peak_bin, peak, expected_bin);
        return EXIT_FAILURE;
    }
    printf("spectrum test PASS peak_bin=%u level=%.1f dBFS\n", peak_bin, peak);
    return EXIT_SUCCESS;
}
