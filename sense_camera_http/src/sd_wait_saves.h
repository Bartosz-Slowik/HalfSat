#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** XIAO ESP32S3 Sense microSD SPI (expansion): SCK=7 MISO=8 MOSI=9 CS=21 */
bool sd_wait_saves_init(void);
bool sd_wait_saves_ready(void);
bool sd_wait_saves_write_jpeg(const uint8_t *jpeg, size_t len);

#ifdef __cplusplus
}
#endif
