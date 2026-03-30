#include "utility.h"
#include <math.h>

uint32_t bits_get_u32(const uint8_t *bytes, int start_bit, int bit_len) {
    uint32_t v = 0;
    for (int i = 0; i < bit_len; i++) {
        int bit_index = start_bit + i;
        int byte_i = bit_index / 8;
        int bit_i = 7 - (bit_index % 8);
        int bit = (bytes[byte_i] >> bit_i) & 1;
        v = (v << 1) | (uint32_t)bit;
    }
    return v;
}

uint64_t bits_get_u64(const uint8_t *bytes, int start_bit, int bit_len) {
    uint64_t v = 0;
    for (int i = 0; i < bit_len; i++) {
        int bit_index = start_bit + i;
        int byte_i = bit_index / 8;
        int bit_i = 7 - (bit_index % 8);
        int bit = (bytes[byte_i] >> bit_i) & 1;
        v = (v << 1) | (uint64_t)bit;
    }
    return v;
}

float mod(float a, float b) {
    return a - b * floor(a / b);
}

double positive_mod(double value, double modulus) {
    return value - modulus * floor(value / modulus);
}

uint32_t max(uint32_t a, uint32_t b) {
    return (a > b) ? a : b;
}