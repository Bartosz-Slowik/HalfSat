/* Sense ↔ LoRa JPEG: 8-byte `cam_link_req_t` (PING size / READ offset). UART: PING 8B rsp, READ 4B hdr + payload. */
#pragma once

#include <stdint.h>

#define CAM_LINK_MAGIC0 0xCAu
#define CAM_LINK_MAGIC1 0xFEu
#define CAM_LINK_CHUNK 1024

enum {
  CAM_LINK_CMD_PING = 0,
  CAM_LINK_CMD_READ = 1,
};

typedef struct __attribute__((packed)) {
  uint8_t magic0;
  uint8_t magic1;
  uint8_t cmd;
  uint8_t reserved;
  uint32_t offset_le;
} cam_link_req_t;
