#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool cam_export_link_alloc(void);
void cam_export_link_publish(const uint8_t *jpeg, size_t len);
/** Current JPEG length (0 if none). Thread-safe. */
void cam_export_link_query_len(size_t *len_out);
/**
 * Copy up to `dst_cap` bytes from the export buffer at `offset`.
 * Returns bytes copied (0 if offset past end or on error). Thread-safe.
 */
size_t cam_export_link_read_range(size_t offset, uint8_t *dst, size_t dst_cap);

#ifdef __cplusplus
}
#endif
