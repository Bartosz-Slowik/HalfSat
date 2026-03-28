/**
 * SPI master on LoRa board (HSPI). SX1262 keeps default SPI (FSPI) on pins 7,8,9.
 */

#include "spi_master_link.h"

#include <Arduino.h>
#include <SPI.h>
#include <stdlib.h>
#include <string.h>

#include "spi_cam_link_proto.h"

// XIAO ESP32-S3: avoid 7,8,9,41 (radio). Must match Sense slave wiring.
static constexpr int PIN_SCK = 3;
static constexpr int PIN_MISO = 5;
static constexpr int PIN_MOSI = 6;
static constexpr int PIN_CS = 4;

static SPIClass spiLink(HSPI);

#ifndef SPI_MASTER_JPEG_MAX
#define SPI_MASTER_JPEG_MAX (384 * 1024)
#endif

static uint8_t s_tx[SPI_CAM_CHUNK];
static uint8_t s_rx[SPI_CAM_CHUNK];

static void transferPhaseAB(const spi_cam_hdr_t *hdr) {
  memset(s_tx, 0, sizeof(s_tx));
  memcpy(s_tx, hdr, sizeof(*hdr));
  digitalWrite(PIN_CS, LOW);
  spiLink.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  spiLink.transferBytes(s_tx, s_rx, SPI_CAM_CHUNK);
  spiLink.endTransaction();
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(10);

  memset(s_tx, 0, sizeof(s_tx));
  digitalWrite(PIN_CS, LOW);
  spiLink.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  spiLink.transferBytes(s_tx, s_rx, SPI_CAM_CHUNK);
  spiLink.endTransaction();
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(5);
}

void spi_master_link_init(void) {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  spiLink.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  Serial.printf("[spi_master] HSPI SCK=%d MOSI=%d MISO=%d CS=%d @ 8 MHz\n", PIN_SCK, PIN_MOSI, PIN_MISO,
                PIN_CS);
}

bool spi_master_link_fetch_jpeg_malloc(uint8_t **out, size_t *out_len) {
  if (!out || !out_len) {
    return false;
  }
  *out = nullptr;
  *out_len = 0;

  spi_cam_hdr_t ping = {SPI_CAM_MAGIC0, SPI_CAM_MAGIC1, SPI_CAM_CMD_PING, 0, 0};
  transferPhaseAB(&ping);

  uint32_t total = 0;
  memcpy(&total, s_rx, sizeof(total));
  if (total == 0 || total > SPI_MASTER_JPEG_MAX) {
    Serial.printf("[spi_master] PING bad total=%u\n", (unsigned)total);
    return false;
  }

  uint8_t *buf = (uint8_t *)malloc(total);
  if (!buf) {
    Serial.println("[spi_master] malloc failed");
    return false;
  }

  for (uint32_t off = 0; off < total; off += SPI_CAM_CHUNK) {
    spi_cam_hdr_t rd = {SPI_CAM_MAGIC0, SPI_CAM_MAGIC1, SPI_CAM_CMD_READ_REQ, 0, off};
    transferPhaseAB(&rd);
    uint32_t chunk = SPI_CAM_CHUNK;
    if (off + chunk > total) {
      chunk = total - off;
    }
    memcpy(buf + off, s_rx, chunk);
  }

  *out = buf;
  *out_len = (size_t)total;
  Serial.printf("[spi_master] fetched JPEG %u bytes\n", (unsigned)total);
  return true;
}
