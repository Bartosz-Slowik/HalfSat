#include "cam_export_link.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#ifndef CAM_EXPORT_JPEG_MAX
#define CAM_EXPORT_JPEG_MAX (384 * 1024)
#endif

static uint8_t *s_store = nullptr;
static size_t s_export_len = 0;
static SemaphoreHandle_t s_mu = nullptr;

static void ensure_mutex(void) {
  if (!s_mu) {
    s_mu = xSemaphoreCreateMutex();
  }
}

bool cam_export_link_alloc(void) {
  ensure_mutex();
  if (!s_mu) {
    return false;
  }
  if (s_store) {
    return true;
  }
  s_store =
      (uint8_t *)heap_caps_malloc(CAM_EXPORT_JPEG_MAX, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return s_store != nullptr;
}

void cam_export_link_publish(const uint8_t *jpeg, size_t len) {
  if (!s_store || jpeg == nullptr || len == 0 || len > CAM_EXPORT_JPEG_MAX || !s_mu) {
    return;
  }
  if (xSemaphoreTake(s_mu, pdMS_TO_TICKS(5000)) != pdTRUE) {
    return;
  }
  memcpy(s_store, jpeg, len);
  s_export_len = len;
  xSemaphoreGive(s_mu);
}

void cam_export_link_query_len(size_t *len_out) {
  if (!len_out || !s_mu) {
    return;
  }
  *len_out = 0;
  if (xSemaphoreTake(s_mu, pdMS_TO_TICKS(500)) != pdTRUE) {
    return;
  }
  *len_out = s_export_len;
  xSemaphoreGive(s_mu);
}

size_t cam_export_link_read_range(size_t offset, uint8_t *dst, size_t dst_cap) {
  if (!s_store || !dst || dst_cap == 0 || !s_mu) {
    return 0;
  }
  if (xSemaphoreTake(s_mu, pdMS_TO_TICKS(500)) != pdTRUE) {
    return 0;
  }
  size_t L = s_export_len;
  if (offset >= L) {
    xSemaphoreGive(s_mu);
    return 0;
  }
  size_t n = L - offset;
  if (n > dst_cap) {
    n = dst_cap;
  }
  memcpy(dst, s_store + offset, n);
  xSemaphoreGive(s_mu);
  return n;
}
