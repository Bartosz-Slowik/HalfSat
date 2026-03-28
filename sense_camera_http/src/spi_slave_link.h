#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Allocates shared `cam_export_link` PSRAM buffer (call before init). */
bool spi_slave_link_alloc_buffer(void);

/** Start SPI2 slave + background task (pins in .cpp). */
void spi_slave_link_init(void);

/** Copies JPEG into shared export buffer (UART + SPI readers use the same snapshot). */
bool spi_slave_link_publish_copy(const uint8_t *jpeg, size_t len);

#ifdef __cplusplus
}
#endif
