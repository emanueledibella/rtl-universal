#pragma once
#include <stdint.h>
#include <math.h>

uint32_t bits_get_u32(const uint8_t *bytes, int start_bit, int bit_len);
uint64_t bits_get_u64(const uint8_t *bytes, int start_bit, int bit_len);
float mod(float a, float b);
double positive_mod(double value, double modulus);
uint32_t max(uint32_t a, uint32_t b);