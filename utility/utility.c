#include "utility.h"

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
