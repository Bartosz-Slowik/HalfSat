#include "uart_slave_link.h"

#include "cam_export_link.h"
#include "cam_link_proto.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

/* XIAO: D2=GPIO3 RX, D5=GPIO6 TX; match LoRa host pins. */
#ifndef CAM_LINK_UART_RX_PIN
#define CAM_LINK_UART_RX_PIN 3
#endif
#ifndef CAM_LINK_UART_TX_PIN
#define CAM_LINK_UART_TX_PIN 6
#endif
#ifndef CAM_LINK_UART_NUM
#define CAM_LINK_UART_NUM 1
#endif
#ifndef CAM_LINK_UART_BAUD
#define CAM_LINK_UART_BAUD 460800
#endif

static HardwareSerial uartPort(CAM_LINK_UART_NUM);

static volatile uint32_t s_last_peer_ms;

extern "C" uint32_t uart_slave_link_last_peer_ms(void) {
  return s_last_peer_ms;
}

static void read_cam_req_blocking(cam_link_req_t *req) {
  for (;;) {
    int c = uartPort.read();
    while (c < 0) {
      vTaskDelay(1);
      c = uartPort.read();
    }
    if ((uint8_t)c != CAM_LINK_MAGIC0) {
      continue;
    }
    uint8_t tail[7];
    if (uartPort.readBytes(tail, 7) != 7) {
      continue;
    }
    if (tail[0] != CAM_LINK_MAGIC1) {
      continue;
    }
    req->magic0 = CAM_LINK_MAGIC0;
    req->magic1 = tail[0];
    req->cmd = tail[1];
    req->reserved = tail[2];
    memcpy(&req->offset_le, tail + 3, 4);
    return;
  }
}

static void uart_slave_task(void * /*arg*/) {
  cam_link_req_t req;
  uint8_t chunk[CAM_LINK_CHUNK];

  for (;;) {
    read_cam_req_blocking(&req);
    if (req.cmd != CAM_LINK_CMD_PING && req.cmd != CAM_LINK_CMD_READ) {
      continue;
    }
    s_last_peer_ms = millis();

    size_t jpeg_len = 0;
    cam_export_link_query_len(&jpeg_len);

    if (req.cmd == CAM_LINK_CMD_PING) {
      cam_link_req_t rsp = {CAM_LINK_MAGIC0, CAM_LINK_MAGIC1, CAM_LINK_CMD_PING, 0,
                            (uint32_t)jpeg_len};
      uartPort.write(reinterpret_cast<const uint8_t *>(&rsp), sizeof(rsp));
    } else if (req.cmd == CAM_LINK_CMD_READ) {
      uint16_t plen = 0;
      if (jpeg_len > 0) {
        const uint32_t off = req.offset_le;
        if (off < jpeg_len) {
          const size_t n = cam_export_link_read_range((size_t)off, chunk, CAM_LINK_CHUNK);
          plen = (uint16_t)n;
        }
      }
      uint8_t hdr[4] = {CAM_LINK_MAGIC0, CAM_LINK_MAGIC1, (uint8_t)(plen & 0xFF),
                        (uint8_t)((plen >> 8) & 0xFF)};
      uartPort.write(hdr, sizeof(hdr));
      if (plen > 0) {
        uartPort.write(chunk, plen);
      }
    }
  }
}

extern "C" void uart_slave_link_init(void) {
  uartPort.setRxBufferSize(2048);
  uartPort.begin(CAM_LINK_UART_BAUD, SERIAL_8N1, CAM_LINK_UART_RX_PIN, CAM_LINK_UART_TX_PIN);
  uartPort.setTimeout(100);
  while (uartPort.available()) {
    (void)uartPort.read();
  }
  xTaskCreatePinnedToCore(uart_slave_task, "uart_slv", 6144, nullptr, 5, nullptr, 0);
  Serial.printf("[uart_slave] HW%u %u baud GPIO RX=%d TX=%d (XIAO D2↔D2 D5↔D5 + GND)\n",
                (unsigned)CAM_LINK_UART_NUM, (unsigned)CAM_LINK_UART_BAUD, CAM_LINK_UART_RX_PIN,
                CAM_LINK_UART_TX_PIN);
}
