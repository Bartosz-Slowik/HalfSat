/* XIAO ESP32-S3 + Wio-SX1262: SSDV TX, JPEG from Sense over UART. */

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Module.h>

extern "C" {
#include <ssdv.h>
}

#include "cam_link_transport.h"

// --- SPI (kit) — radio only ---
static constexpr uint8_t PIN_SCK = 7;
static constexpr uint8_t PIN_MISO = 8;
static constexpr uint8_t PIN_MOSI = 9;
static constexpr uint8_t PIN_CS = 41;

static constexpr uint8_t PIN_IRQ_DIO1 = 39;
static constexpr uint8_t PIN_RST = 42;
static constexpr uint8_t PIN_BUSY = 40;

static constexpr float RF_FREQ_MHZ = 869.4F;
static constexpr float LORA_BW_KHZ = 500.0F;
static constexpr uint8_t LORA_SF = 7;
static constexpr uint8_t LORA_CR = 5;
static constexpr uint8_t LORA_SYNC = 0x12;
static constexpr int8_t TX_POWER_DBM = 22;
static constexpr uint16_t PREAMBLE_LEN = 8;
static constexpr float TCXO_VOLTAGE = 1.8F;

static constexpr int SSDV_AIR_PKTLEN = 255;
/** Extra delay after each LoRa TX; E5 UART / receiver need margin vs bare airtime. */
static constexpr uint32_t SSDV_INTER_PACKET_MS = 8;
static constexpr int8_t SSDV_QUALITY = 6;
/** Silence between images so the E5+host can finish SSDV before the next image_id. */
static constexpr uint32_t AFTER_IMAGE_GAP_MS = 20000;
static char SSDV_CALLSIGN[] = "HABSAT";

static Module radioModule(PIN_CS, PIN_IRQ_DIO1, PIN_RST, PIN_BUSY, SPI,
                          SPISettings(2000000, MSBFIRST, SPI_MODE0));
static SX1262 radio(&radioModule);

static uint8_t ssdv_packet[256];
static ssdv_t ssdv_enc;

static void haltOnFail(int16_t code, const char *what) {
  if (code == RADIOLIB_ERR_NONE) {
    return;
  }
  Serial.printf("%s failed, code %d\n", what, (int)code);
  while (true) {
    delay(1000);
  }
}

static bool sendSsdvImage(const uint8_t *jpeg, size_t jpeg_len, uint8_t image_id) {
  if (ssdv_enc_init(&ssdv_enc, SSDV_TYPE_NOFEC, SSDV_CALLSIGN, image_id, SSDV_QUALITY,
                    SSDV_AIR_PKTLEN) != SSDV_OK) {
    Serial.println("ssdv_enc_init failed");
    return false;
  }
  if (ssdv_enc_set_buffer(&ssdv_enc, ssdv_packet) != SSDV_OK) {
    Serial.println("ssdv_enc_set_buffer failed");
    return false;
  }
  if (ssdv_enc_feed(&ssdv_enc, const_cast<uint8_t *>(jpeg), jpeg_len) != SSDV_OK) {
    Serial.println("ssdv_enc_feed failed");
    return false;
  }

  uint16_t pkt_count = 0;
  for (;;) {
    const char r = ssdv_enc_get_packet(&ssdv_enc);
    if (r == SSDV_OK) {
      pkt_count++;
      int16_t st = radio.transmit(ssdv_packet, (size_t)SSDV_AIR_PKTLEN);
      haltOnFail(st, "radio.transmit");
      delay(SSDV_INTER_PACKET_MS);
    } else if (r == SSDV_EOI) {
      Serial.printf("[tx] image #%u sent, %u SSDV packets\n", (unsigned)image_id,
                    (unsigned)pkt_count);
      return true;
    } else if (r == SSDV_FEED_ME) {
      Serial.println("ssdv: unexpected FEED_ME (truncated JPEG?)");
      return false;
    } else {
      Serial.printf(
          "ssdv_enc_get_packet error %d (SSDV_ERROR=-1: bad JPEG for SSDV? need WxH multiple of 16; "
          "no progressive)\n",
          r);
      return false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  Serial.println("XIAO ESP32S3 + Wio-SX1262 — SSDV JPEG TX (UART from Sense)");
  int16_t st = radio.begin(RF_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC, TX_POWER_DBM,
                           PREAMBLE_LEN, TCXO_VOLTAGE);
  haltOnFail(st, "radio.begin");

  Serial.printf("RF %.2f MHz SF%u BW%.0fk CR%u sync=0x%02X | SSDV NOFEC q=%d %d B pkt | callsign=%.6s\n",
                RF_FREQ_MHZ, (unsigned)LORA_SF, LORA_BW_KHZ, (unsigned)LORA_CR, LORA_SYNC,
                (int)SSDV_QUALITY, SSDV_AIR_PKTLEN, SSDV_CALLSIGN);

  cam_link_transport_init();

  bool linked = false;
  for (int attempt = 0; attempt < 12; attempt++) {
    size_t sz = 0;
    if (cam_link_probe(&sz)) {
      Serial.printf("[cam_link] probe OK — Sense replying, export %u bytes\n", (unsigned)sz);
      linked = true;
      break;
    }
    Serial.printf("[cam_link] probe try %d/12 — no reply (D2↔D2 D5↔D5 GND? baud both sides?)\n",
                  attempt + 1);
    delay(400);
  }
  if (!linked) {
    Serial.println("[cam_link] probe gave up; loop will keep retrying fetch");
  }
}

void loop() {
  static uint8_t image_id = 0;
  static uint32_t last_link_check_ms = 0;
  if (millis() - last_link_check_ms >= 15000) {
    last_link_check_ms = millis();
    size_t sz = 0;
    if (cam_link_probe(&sz)) {
      Serial.printf("[cam_link] link check OK, export %u B\n", (unsigned)sz);
    } else {
      Serial.println("[cam_link] link check FAIL (no PING reply)");
    }
  }

  if (image_id > 0) {
    Serial.printf("[tx] RF gap %lu ms after previous image\n", (unsigned long)AFTER_IMAGE_GAP_MS);
    delay(AFTER_IMAGE_GAP_MS);
  }

  uint8_t *jpeg = nullptr;
  size_t jpeg_len = 0;
  if (!cam_link_fetch_jpeg_malloc(&jpeg, &jpeg_len)) {
    Serial.println("UART JPEG fetch failed, retry…");
    delay(500);
    return;
  }
  Serial.printf("Encoding UART image #%u (%u bytes JPEG)\n", (unsigned)image_id,
                (unsigned)jpeg_len);
  if (!sendSsdvImage(jpeg, jpeg_len, image_id)) {
    Serial.println("SSDV encode/send aborted");
  }
  free(jpeg);
  image_id++;
}
