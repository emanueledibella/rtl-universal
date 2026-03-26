#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void protocol_handle_message(uint8_t df, uint8_t ca, uint32_t icao, uint64_t me, uint32_t pi);

#ifdef __cplusplus
}
#endif
