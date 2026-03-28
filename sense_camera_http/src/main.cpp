/**
 * XIAO ESP32-S3 Sense — camera + UART export to LoRa (no WiFi / no HTTP).
 *
 * Background task: burst N frames, pick sharpest JPEG (Laplacian on 8× decode), publish for UART slave.
 * Tune: CAM_MOTION_FRIENDLY, UART_BEST_BURST_N, UART_BEST_CYCLE_MS, JPG_BEST_* (see below).
 */

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "cam_export_link.h"
#include "camera_sense.h"
#include "img_converters.h"
#include "sd_wait_saves.h"
#include "uart_slave_link.h"

#ifndef JPG_BEST_MAX_BYTES
#define JPG_BEST_MAX_BYTES (384 * 1024)
#endif
#ifndef JPG_BEST_THUMB_W
#define JPG_BEST_THUMB_W 256
#endif
#ifndef JPG_BEST_THUMB_H
#define JPG_BEST_THUMB_H 192
#endif
#ifndef UART_BEST_BURST_N
#define UART_BEST_BURST_N 5
#endif
#ifndef UART_BEST_CYCLE_MS
#define UART_BEST_CYCLE_MS 2500
#endif
/** No PING/READ for this long → LoRa not fetching; write non-best burst frames to SD. */
#ifndef SD_SAVE_MIN_PEER_IDLE_MS
#define SD_SAVE_MIN_PEER_IDLE_MS 2500
#endif

static void printMem(const char *tag) {
  uint32_t heap = ESP.getFreeHeap();
  uint32_t psram = ESP.getPsramSize();
  uint32_t psramFree = psram ? heap_caps_get_free_size(MALLOC_CAP_SPIRAM) : 0;
  Serial.printf("[MEM %s] heap=%u psram_free=%u\n", tag, (unsigned)heap, (unsigned)psramFree);
}

static inline uint8_t rgb565ToY(uint16_t p) {
  uint32_t r = (p >> 11) & 0x1fu;
  uint32_t g = (p >> 5) & 0x3fu;
  uint32_t b = p & 0x1fu;
  r = (r << 3) | (r >> 2);
  g = (g << 2) | (g >> 4);
  b = (b << 3) | (b >> 2);
  return (uint8_t)((r * 38u + g * 75u + b * 15u) >> 7);
}

static uint64_t laplacianEnergyU8(const uint8_t *g, int w, int h) {
  if (w < 3 || h < 3) {
    return 0;
  }
  uint64_t acc = 0;
  for (int y = 1; y < h - 1; y++) {
    const uint8_t *row = g + y * w;
    for (int x = 1; x < w - 1; x++) {
      int c = (int)row[x];
      int lap = 4 * c - (int)row[x - 1] - (int)row[x + 1] - (int)row[x - w] - (int)row[x + w];
      acc += (uint32_t)(lap * lap);
    }
  }
  return acc;
}

static bool burstPickBestJpeg(int n, uint8_t *rgb565, uint8_t *gray, uint8_t *bestJpeg, size_t *bestLenOut,
                              uint64_t *scoreOut, int *okFramesOut, bool save_discards) {
  const size_t rgbCap = (size_t)JPG_BEST_THUMB_W * (size_t)JPG_BEST_THUMB_H * 2u;
  if (n < 2) {
    n = 2;
  }
  if (n > 12) {
    n = 12;
  }

  bool haveBest = false;
  uint64_t bestScore = 0;
  size_t bestLen = 0;
  int okFrames = 0;

  for (int i = 0; i < n; i++) {
    // SVGA JPEG decode can take seconds; without yielding, IDLE0 never runs and the task WDT aborts.
    vTaskDelay(pdMS_TO_TICKS(1));
    taskYIELD();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb || fb->format != PIXFORMAT_JPEG) {
      if (fb) {
        esp_camera_fb_return(fb);
      }
      continue;
    }
    if (fb->len > JPG_BEST_MAX_BYTES) {
      esp_camera_fb_return(fb);
      continue;
    }

    const int dw = (int)(fb->width / 8u);
    const int dh = (int)(fb->height / 8u);
    if (dw < 3 || dh < 3 || (size_t)dw * (size_t)dh * 2u > rgbCap) {
      esp_camera_fb_return(fb);
      continue;
    }

    if (!jpg2rgb565(fb->buf, fb->len, rgb565, JPG_SCALE_8X)) {
      esp_camera_fb_return(fb);
      continue;
    }
    taskYIELD();

    const size_t npix = (size_t)dw * (size_t)dh;
    for (size_t p = 0; p < npix; p++) {
      uint16_t pix = (uint16_t)rgb565[p * 2u] | ((uint16_t)rgb565[p * 2u + 1u] << 8);
      gray[p] = rgb565ToY(pix);
    }

    uint64_t sc = laplacianEnergyU8(gray, dw, dh);
    taskYIELD();
    okFrames++;
    if (!haveBest || sc > bestScore) {
      if (haveBest && save_discards && sd_wait_saves_ready()) {
        sd_wait_saves_write_jpeg(bestJpeg, bestLen);
      }
      haveBest = true;
      bestScore = sc;
      bestLen = fb->len;
      memcpy(bestJpeg, fb->buf, bestLen);
    } else if (save_discards && sd_wait_saves_ready()) {
      sd_wait_saves_write_jpeg(fb->buf, fb->len);
    }
    esp_camera_fb_return(fb);
  }

  if (!haveBest || bestLen == 0) {
    return false;
  }
  *bestLenOut = bestLen;
  *scoreOut = bestScore;
  *okFramesOut = okFrames;
  return true;
}

static void uartBestExportTask(void * /*arg*/) {
  const size_t rgbCap = (size_t)JPG_BEST_THUMB_W * (size_t)JPG_BEST_THUMB_H * 2u;
  const size_t grayCap = (size_t)JPG_BEST_THUMB_W * (size_t)JPG_BEST_THUMB_H;
  uint8_t *rgb565 =
      (uint8_t *)heap_caps_malloc(rgbCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  uint8_t *gray = (uint8_t *)heap_caps_malloc(grayCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  uint8_t *bestJpeg =
      (uint8_t *)heap_caps_malloc(JPG_BEST_MAX_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rgb565 || !gray || !bestJpeg) {
    Serial.println("[uart_best] PSRAM alloc failed");
    vTaskDelete(nullptr);
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(500));

  for (;;) {
    const uint32_t now = millis();
    const uint32_t lp = uart_slave_link_last_peer_ms();
    const bool uart_idle =
        (lp == 0) || ((now - lp) >= (uint32_t)SD_SAVE_MIN_PEER_IDLE_MS);

    size_t bestLen = 0;
    uint64_t score = 0;
    int okFrames = 0;
    if (burstPickBestJpeg(UART_BEST_BURST_N, rgb565, gray, bestJpeg, &bestLen, &score, &okFrames,
                          uart_idle)) {
      cam_export_link_publish(bestJpeg, bestLen);
      Serial.printf("[uart_best] export len=%u score=%llu ok=%d/%d\n", (unsigned)bestLen,
                    (unsigned long long)score, okFrames, UART_BEST_BURST_N);
    } else {
      Serial.println("[uart_best] burst failed — trying single-frame fallback");
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb) {
        if (fb->format == PIXFORMAT_JPEG && fb->len > 0 && fb->len <= JPG_BEST_MAX_BYTES) {
          cam_export_link_publish(fb->buf, fb->len);
          Serial.printf("[uart_best] fallback publish len=%u\n", (unsigned)fb->len);
        } else {
          Serial.println("[uart_best] fallback: frame not usable JPEG");
        }
        esp_camera_fb_return(fb);
      } else {
        Serial.println("[uart_best] fallback: no frame");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(UART_BEST_CYCLE_MS));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Sense camera → UART (no WiFi) ===");
  Serial.printf("chip=%s %u MHz\n", ESP.getChipModel(), ESP.getCpuFreqMHz());
  printMem("boot");

  if (!camera_sense_init()) {
    Serial.println("[HALT] camera init failed");
    while (true) {
      delay(1000);
    }
  }
  printMem("after cam");

  if (!cam_export_link_alloc()) {
    Serial.println("[HALT] cam_export PSRAM alloc failed");
    while (true) {
      delay(1000);
    }
  }

  uart_slave_link_init();
  (void)sd_wait_saves_init();

  camera_fb_t *fb0 = esp_camera_fb_get();
  if (fb0) {
    cam_export_link_publish(fb0->buf, fb0->len);
    esp_camera_fb_return(fb0);
  }
  xTaskCreatePinnedToCore(uartBestExportTask, "uart_best", 8192, nullptr, 3, nullptr, 0);

  Serial.println("UART slave + best-JPEG task running.");
  printMem("setup done");
}

void loop() {
  static uint32_t lastTick = 0;
  if (millis() - lastTick >= 10000) {
    lastTick = millis();
    uint32_t lp = uart_slave_link_last_peer_ms();
    if (lp != 0) {
      Serial.printf("[tick] heap=%u uart_peer_ago_ms=%u\n", (unsigned)ESP.getFreeHeap(),
                    (unsigned)(millis() - lp));
    } else {
      Serial.printf("[tick] heap=%u uart_peer=never\n", (unsigned)ESP.getFreeHeap());
    }
  }
  delay(20);
}
