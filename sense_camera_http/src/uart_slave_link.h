#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** See .cpp for pins (default XIAO D2/D5 = GPIO 3 & 6). */
void uart_slave_link_init(void);

/** `millis()` when a valid PING/READ was last handled; 0 if never. */
uint32_t uart_slave_link_last_peer_ms(void);

#ifdef __cplusplus
}
#endif
