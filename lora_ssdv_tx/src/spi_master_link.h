#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Second SPI (HSPI); CS GPIO. Call once after Serial.begin. */
void spi_master_link_init(void);

/**
 * PING + READ all chunks into malloc'd buffer.
 * Caller must free(*out) with free().
 */
bool spi_master_link_fetch_jpeg_malloc(uint8_t **out, size_t *out_len);

#ifdef __cplusplus
}
#endif
