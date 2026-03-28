#include "cam_link_transport.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdlib.h>

#include "cam_link_proto.h"

/* XIAO: D2=GPIO3 D5=GPIO6, straight + GND. */
#ifndef CAM_LINK_HOST_UART_RX_PIN
#define CAM_LINK_HOST_UART_RX_PIN 6
#endif
#ifndef CAM_LINK_HOST_UART_TX_PIN
#define CAM_LINK_HOST_UART_TX_PIN 3
#endif
#ifndef CAM_LINK_UART_NUM
#define CAM_LINK_UART_NUM 1
#endif
#ifndef CAM_LINK_UART_BAUD
#define CAM_LINK_UART_BAUD 460800
#endif
#ifndef CAM_LINK_JPEG_MAX
#define CAM_LINK_JPEG_MAX (384 * 1024)
#endif

static HardwareSerial s_uartCam(CAM_LINK_UART_NUM);

extern "C" void cam_link_transport_init(void) {
  s_uartCam.setRxBufferSize(8192);
  s_uartCam.begin(CAM_LINK_UART_BAUD, SERIAL_8N1, CAM_LINK_HOST_UART_RX_PIN,
                  CAM_LINK_HOST_UART_TX_PIN);
  s_uartCam.setTimeout(4000);
  while (s_uartCam.available()) {
    (void)s_uartCam.read();
  }
  Serial.printf("[cam_link] HW%u %u baud RX=%d TX=%d (XIAO D2↔D2 D5↔D5 + GND)\n",
                (unsigned)CAM_LINK_UART_NUM, (unsigned)CAM_LINK_UART_BAUD,
                CAM_LINK_HOST_UART_RX_PIN, CAM_LINK_HOST_UART_TX_PIN);
}

static bool cam_link_ping_once(size_t *jpeg_size_out) {
  cam_link_req_t ping = {CAM_LINK_MAGIC0, CAM_LINK_MAGIC1, CAM_LINK_CMD_PING, 0, 0};
  s_uartCam.write(reinterpret_cast<const uint8_t *>(&ping), sizeof(ping));
  s_uartCam.flush();
  delay(2);

  cam_link_req_t prsp;
  if (s_uartCam.readBytes(reinterpret_cast<uint8_t *>(&prsp), sizeof(prsp)) != sizeof(prsp)) {
    return false;
  }
  if (prsp.magic0 != CAM_LINK_MAGIC0 || prsp.magic1 != CAM_LINK_MAGIC1 ||
      prsp.cmd != CAM_LINK_CMD_PING) {
    return false;
  }
  uint32_t total = prsp.offset_le;
  if (total > CAM_LINK_JPEG_MAX) {
    return false;
  }
  if (jpeg_size_out) {
    *jpeg_size_out = (size_t)total;
  }
  return true;
}

extern "C" bool cam_link_probe(size_t *jpeg_size_out) {
  return cam_link_ping_once(jpeg_size_out);
}

extern "C" bool cam_link_fetch_jpeg_malloc(uint8_t **out, size_t *out_len) {
  if (!out || !out_len) {
    return false;
  }
  *out = nullptr;
  *out_len = 0;

  size_t total_sz = 0;
  if (!cam_link_ping_once(&total_sz)) {
    return false;
  }
  uint32_t total = (uint32_t)total_sz;
  if (total == 0) {
    return false;
  }

  uint8_t *buf = (uint8_t *)malloc(total);
  if (!buf) {
    Serial.println("[cam_link] malloc failed");
    return false;
  }

  uint32_t off = 0;
  while (off < total) {
    cam_link_req_t rd = {CAM_LINK_MAGIC0, CAM_LINK_MAGIC1, CAM_LINK_CMD_READ, 0, off};
    s_uartCam.write(reinterpret_cast<const uint8_t *>(&rd), sizeof(rd));
    s_uartCam.flush();
    delay(1);

    uint8_t hdr[4];
    if (s_uartCam.readBytes(hdr, sizeof(hdr)) != sizeof(hdr)) {
      Serial.println("[cam_link] READ hdr timeout");
      free(buf);
      return false;
    }
    if (hdr[0] != CAM_LINK_MAGIC0 || hdr[1] != CAM_LINK_MAGIC1) {
      Serial.println("[cam_link] READ bad magic");
      free(buf);
      return false;
    }
    uint16_t plen = (uint16_t)hdr[2] | ((uint16_t)hdr[3] << 8);
    if (plen > CAM_LINK_CHUNK) {
      free(buf);
      return false;
    }
    if (plen > 0) {
      if (s_uartCam.readBytes(buf + off, plen) != plen) {
        Serial.println("[cam_link] READ payload timeout");
        free(buf);
        return false;
      }
    }
    off += plen;
    if (plen == 0) {
      Serial.println("[cam_link] READ stalled (0 len)");
      free(buf);
      return false;
    }
  }

  *out = buf;
  *out_len = (size_t)total;
  Serial.printf("[cam_link] UART fetched JPEG %u bytes\n", (unsigned)total);
  return true;
}
