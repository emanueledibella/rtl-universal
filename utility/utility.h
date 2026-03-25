#pragma once
#include <stdint.h>

uint32_t bits_get_u32(const uint8_t *bytes, int start_bit, int bit_len);
uint64_t bits_get_u64(const uint8_t *bytes, int start_bit, int bit_len);
