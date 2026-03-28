/**
 * SPI slave on Sense board: serves JPEG to LoRa master (two-phase A/B per chunk).
 * JPEG bytes live in `cam_export_link` (shared with UART transport).
 */

#include "spi_slave_link.h"

#include "cam_export_link.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#include "driver/spi_slave.h"
#include "spi_cam_link_proto.h"

static constexpr int PIN_SCK = 1;
static constexpr int PIN_MISO = 3;
static constexpr int PIN_MOSI = 2;
static constexpr int PIN_CS = 4;

static uint8_t *s_dma_rx_a = nullptr;
static uint8_t *s_dma_tx_a = nullptr;
static uint8_t *s_dma_rx_b = nullptr;
static uint8_t *s_dma_tx_b = nullptr;

static spi_slave_transaction_t s_ta;
static spi_slave_transaction_t s_tb;

static void spi_slave_task(void * /*arg*/) {
  memset(&s_ta, 0, sizeof(s_ta));
  s_ta.length = SPI_CAM_CHUNK * 8;
  s_ta.tx_buffer = s_dma_tx_a;
  s_ta.rx_buffer = s_dma_rx_a;
  memset(s_dma_tx_a, 0, SPI_CAM_CHUNK);
  memset(s_dma_rx_a, 0, SPI_CAM_CHUNK);
  spi_slave_queue_trans(SPI2_HOST, &s_ta, portMAX_DELAY);

  for (;;) {
    spi_slave_transaction_t *ret = nullptr;
    spi_slave_get_trans_result(SPI2_HOST, &ret, portMAX_DELAY);

    spi_cam_hdr_t hdr;
    memcpy(&hdr, s_dma_rx_a, sizeof(hdr));

    memset(s_dma_tx_b, 0, SPI_CAM_CHUNK);
    memset(s_dma_rx_b, 0, SPI_CAM_CHUNK);

    size_t len = 0;
    cam_export_link_query_len(&len);

    if (hdr.magic0 == SPI_CAM_MAGIC0 && hdr.magic1 == SPI_CAM_MAGIC1) {
      if (hdr.cmd == SPI_CAM_CMD_PING) {
        uint32_t sz = (uint32_t)len;
        memcpy(s_dma_tx_b, &sz, sizeof(sz));
      } else if (hdr.cmd == SPI_CAM_CMD_READ_REQ && len > 0) {
        const uint32_t off = hdr.offset_le;
        if (off < len) {
          (void)cam_export_link_read_range((size_t)off, s_dma_tx_b, SPI_CAM_CHUNK);
        }
      }
    }

    memset(&s_tb, 0, sizeof(s_tb));
    s_tb.length = SPI_CAM_CHUNK * 8;
    s_tb.tx_buffer = s_dma_tx_b;
    s_tb.rx_buffer = s_dma_rx_b;
    spi_slave_queue_trans(SPI2_HOST, &s_tb, portMAX_DELAY);
    spi_slave_get_trans_result(SPI2_HOST, &ret, portMAX_DELAY);

    memset(s_dma_tx_a, 0, SPI_CAM_CHUNK);
    memset(s_dma_rx_a, 0, SPI_CAM_CHUNK);
    s_ta.tx_buffer = s_dma_tx_a;
    s_ta.rx_buffer = s_dma_rx_a;
    spi_slave_queue_trans(SPI2_HOST, &s_ta, portMAX_DELAY);
  }
}

bool spi_slave_link_alloc_buffer(void) {
  return cam_export_link_alloc();
}

void spi_slave_link_init(void) {
  s_dma_rx_a = (uint8_t *)heap_caps_malloc(SPI_CAM_CHUNK, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  s_dma_tx_a = (uint8_t *)heap_caps_malloc(SPI_CAM_CHUNK, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  s_dma_rx_b = (uint8_t *)heap_caps_malloc(SPI_CAM_CHUNK, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  s_dma_tx_b = (uint8_t *)heap_caps_malloc(SPI_CAM_CHUNK, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (!s_dma_rx_a || !s_dma_tx_a || !s_dma_rx_b || !s_dma_tx_b) {
    return;
  }

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = PIN_MOSI;
  buscfg.miso_io_num = PIN_MISO;
  buscfg.sclk_io_num = PIN_SCK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = SPI_CAM_CHUNK;

  spi_slave_interface_config_t slvcfg = {};
  slvcfg.mode = 0;
  slvcfg.spics_io_num = PIN_CS;
  slvcfg.queue_size = 4;
  slvcfg.flags = 0;

  esp_err_t err = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    Serial.printf("[spi_slave] init failed %d\n", (int)err);
    return;
  }

  xTaskCreatePinnedToCore(spi_slave_task, "spi_slv", 8192, nullptr, 5, nullptr, 0);
  Serial.printf("[spi_slave] SPI2 slave SCK=%d MOSI=%d MISO=%d CS=%d chunk=%d\n", PIN_SCK, PIN_MOSI,
                PIN_MISO, PIN_CS, SPI_CAM_CHUNK);
}

bool spi_slave_link_publish_copy(const uint8_t *jpeg, size_t len) {
#ifndef CAM_EXPORT_JPEG_MAX
#define CAM_EXPORT_JPEG_MAX (384 * 1024)
#endif
  if (jpeg == nullptr || len == 0 || len > CAM_EXPORT_JPEG_MAX) {
    return false;
  }
  cam_export_link_publish(jpeg, len);
  return true;
}
