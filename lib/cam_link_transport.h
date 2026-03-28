#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void cam_link_transport_init(void);

bool cam_link_probe(size_t *jpeg_size_out);

/** PING + READ full JPEG; caller must `free(*out)`. */
bool cam_link_fetch_jpeg_malloc(uint8_t **out, size_t *out_len);

#ifdef __cplusplus
}
#endif
