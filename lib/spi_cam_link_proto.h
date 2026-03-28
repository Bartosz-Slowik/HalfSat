/** Backward-compatible aliases for SPI code — use `cam_link_proto.h` for new work. */
#pragma once

#include "cam_link_proto.h"

#define SPI_CAM_CHUNK CAM_LINK_CHUNK
#define SPI_CAM_MAGIC0 CAM_LINK_MAGIC0
#define SPI_CAM_MAGIC1 CAM_LINK_MAGIC1
#define SPI_CAM_CMD_PING CAM_LINK_CMD_PING
#define SPI_CAM_CMD_READ_REQ CAM_LINK_CMD_READ
typedef cam_link_req_t spi_cam_hdr_t;
