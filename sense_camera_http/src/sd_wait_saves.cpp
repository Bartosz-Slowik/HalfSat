#include "sd_wait_saves.h"

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <stdio.h>

#ifndef SD_SPI_SCK
#define SD_SPI_SCK 7
#endif
#ifndef SD_SPI_MISO
#define SD_SPI_MISO 8
#endif
#ifndef SD_SPI_MOSI
#define SD_SPI_MOSI 9
#endif
#ifndef SD_SPI_CS
#define SD_SPI_CS 21
#endif

#ifndef SD_WAIT_JPEG_MAX
#define SD_WAIT_JPEG_MAX (512 * 1024)
#endif
#ifndef SD_WRITE_CHUNK
#define SD_WRITE_CHUNK 2048u
#endif
/** After this many consecutive open/write failures, stop touching the card until reboot. */
#ifndef SD_WAIT_FAIL_DISABLE_AFTER
#define SD_WAIT_FAIL_DISABLE_AFTER 4
#endif
/**
 * Spare JPEGs (UART idle discards) can flood the card on large XGA files. Cap average write rate so
 * ~14 GiB usable lasts at least ~3 h with large (~1–2 MiB) XGA spares: cap sustained rate with a
 * minimum gap between any two SD JPEG writes (discards + future uses). Default 2000 ms ≈ 5400
 * writes / 3 h → ~10.8 GiB if each ≈ 2 MiB (headroom for FAT, smaller files, or a larger card).
 * Tighten with -DSD_WAIT_MIN_MS_BETWEEN_JPEG=3000 for extra margin.
 */
#ifndef SD_WAIT_MIN_MS_BETWEEN_JPEG
#define SD_WAIT_MIN_MS_BETWEEN_JPEG 2000u
#endif

static bool s_mounted = false;
static bool s_spi_started = false;
static uint32_t s_seq = 0;
static uint8_t s_fail_streak = 0;
static uint32_t s_last_jpeg_write_ms = 0;

static void sd_wait_disable(const char *reason) {
  if (s_mounted) {
    SD.end();
    s_mounted = false;
  }
  s_fail_streak = 0;
  if (reason != nullptr) {
    Serial.printf("[sd_wait] disabled: %s\n", reason);
  }
}

bool sd_wait_saves_init(void) {
  s_mounted = false;
  s_fail_streak = 0;

  if (!s_spi_started) {
    SPI.begin(SD_SPI_SCK, SD_SPI_MISO, SD_SPI_MOSI, SD_SPI_CS);
    s_spi_started = true;
  }

  uint32_t freqs[] = {10000000u, 4000000u};
  for (unsigned fi = 0; fi < sizeof(freqs) / sizeof(freqs[0]); fi++) {
    if (SD.begin(SD_SPI_CS, SPI, freqs[fi])) {
      s_mounted = true;
      Serial.printf("[sd_wait] SD OK @ %u Hz — spare JPEGs when UART idle (min %u ms between writes)\n",
                    (unsigned)freqs[fi], (unsigned)SD_WAIT_MIN_MS_BETWEEN_JPEG);
      return true;
    }
  }

  Serial.println("[sd_wait] SD mount failed — spare saves off (no crash)");
  return false;
}

bool sd_wait_saves_ready(void) {
  return s_mounted;
}

bool sd_wait_saves_write_jpeg(const uint8_t *jpeg, size_t len) {
  if (!s_mounted || jpeg == nullptr || len == 0 || len > SD_WAIT_JPEG_MAX) {
    return false;
  }

  const uint32_t now = millis();
  if (s_last_jpeg_write_ms != 0u &&
      (uint32_t)(now - s_last_jpeg_write_ms) < (uint32_t)SD_WAIT_MIN_MS_BETWEEN_JPEG) {
    return false;
  }

  char path[40];
  const int plen = snprintf(path, sizeof(path), "/w%lu_%04u.jpg", (unsigned long)millis(),
                          (unsigned)++s_seq);
  if (plen <= 0 || (size_t)plen >= sizeof(path)) {
    return false;
  }

  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    if (++s_fail_streak >= SD_WAIT_FAIL_DISABLE_AFTER) {
      sd_wait_disable("open failures (bad card or full?)");
    }
    return false;
  }

  size_t off = 0;
  while (off < len) {
    const size_t n = (len - off > SD_WRITE_CHUNK) ? SD_WRITE_CHUNK : (len - off);
    const size_t w = f.write(jpeg + off, n);
    if (w != n) {
      f.close();
      SD.remove(path);
      if (++s_fail_streak >= SD_WAIT_FAIL_DISABLE_AFTER) {
        sd_wait_disable("write failures (card removed or bad?)");
      }
      return false;
    }
    off += w;
    yield();
  }
  f.close();
  s_fail_streak = 0;
  s_last_jpeg_write_ms = now;
  return true;
}
