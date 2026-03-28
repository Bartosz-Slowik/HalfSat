#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ssdv.h"

typedef struct {
    ssdv_t dec;
    int pkt_size;
} ssdv_rx_t;

int ssdv_rx_create(ssdv_rx_t **out, int pkt_size) {
    ssdv_rx_t *h = (ssdv_rx_t *)calloc(1, sizeof(ssdv_rx_t));
    if (!h) return -1;
    h->pkt_size = pkt_size;
    if (ssdv_dec_init(&h->dec, pkt_size) != SSDV_OK) {
        free(h);
        return -2;
    }
    *out = h;
    return 0;
}

void ssdv_rx_destroy(ssdv_rx_t *h) {
    if (h) free(h);
}

int ssdv_rx_packet_ok(const uint8_t *packet, size_t packet_len, int pkt_size) {
    int errors = 0;
    if (!packet || packet_len < 15 || packet_len > SSDV_PKT_SIZE) return 0;
    if (pkt_size > 0 && (int)packet_len != pkt_size) return 0;
    return ssdv_dec_is_packet((uint8_t *)packet, (int)packet_len, &errors) == SSDV_OK ? 1 : 0;
}

/* Replay SSDV packets in ascending packet-ID order (first must be ID 0).
 * Gaps in the sequence are handled inside ssdv_dec_feed (gray fill). Caller must sort by ID. */
int ssdv_rx_preview_replay(const uint8_t *const *packets,
                           size_t num_packets,
                           int pkt_size,
                           uint8_t *out_jpeg,
                           size_t out_cap,
                           size_t *out_len) {
    ssdv_rx_t *h = NULL;
    uint8_t *jpeg = NULL;
    size_t jpeg_len = 0;

    if (!packets || num_packets == 0 || !out_jpeg || !out_len) return -1;
    if (ssdv_rx_create(&h, pkt_size) != 0) return -1;

    for (size_t i = 0; i < num_packets; i++) {
        const uint8_t *p = packets[i];
        int errors = 0;
        if (!p || ssdv_dec_is_packet((uint8_t *)p, pkt_size, &errors) != SSDV_OK) {
            ssdv_rx_destroy(h);
            return -2;
        }
        {
            uint16_t pid = ((uint16_t)p[7] << 8) | p[8];
            if (i == 0) {
                if (pid != 0) {
                    ssdv_rx_destroy(h);
                    return -5;
                }
            } else {
                uint16_t prev =
                    ((uint16_t)packets[i - 1][7] << 8) | packets[i - 1][8];
                if (pid <= prev) {
                    ssdv_rx_destroy(h);
                    return -4;
                }
            }
        }
        if (i == 0) {
            if (ssdv_dec_init(&h->dec, h->pkt_size) != SSDV_OK) {
                ssdv_rx_destroy(h);
                return -3;
            }
        }
        if (ssdv_dec_set_buffer(&h->dec, out_jpeg, out_cap) != SSDV_OK) {
            ssdv_rx_destroy(h);
            return -3;
        }
        {
            char r = ssdv_dec_feed(&h->dec, (uint8_t *)p);
            if (r != SSDV_OK && r != SSDV_FEED_ME) {
                ssdv_rx_destroy(h);
                return -3;
            }
        }
    }

    if (ssdv_dec_get_jpeg(&h->dec, &jpeg, &jpeg_len) != SSDV_OK) {
        ssdv_rx_destroy(h);
        return -3;
    }
    if (jpeg_len > out_cap) {
        ssdv_rx_destroy(h);
        return -3;
    }
    if (jpeg != out_jpeg) {
        memmove(out_jpeg, jpeg, jpeg_len);
    }
    *out_len = jpeg_len;
    ssdv_rx_destroy(h);
    return 0;
}

/* Return codes:
 *  1 => JPEG ready in out_jpeg/out_len
 *  0 => packet accepted, no full image yet
 * -1 => invalid args
 * -2 => packet not valid SSDV
 * -3 => decoder error
 */
int ssdv_rx_process(ssdv_rx_t *h,
                    const uint8_t *packet,
                    size_t packet_len,
                    uint8_t *out_jpeg,
                    size_t out_cap,
                    size_t *out_len,
                    uint8_t *image_id,
                    uint16_t *packet_id,
                    int *eoi_flag) {
    int errors = 0;
    char r;
    uint8_t *jpeg = NULL;
    size_t jpeg_len = 0;
    uint16_t pid;
    int eoi;

    if (!h || !packet || !out_jpeg || !out_len || packet_len < 15) return -1;

    if (packet_len > SSDV_PKT_SIZE) return -2;
    if (ssdv_dec_is_packet((uint8_t *)packet, (int)packet_len, &errors) != SSDV_OK) return -2;

    pid = ((uint16_t)packet[7] << 8) | packet[8];
    eoi = (packet[11] & 0x04) ? 1 : 0;

    if (packet_id) *packet_id = pid;
    if (image_id) *image_id = packet[6];
    if (eoi_flag) *eoi_flag = eoi;

    /* New image starts at packet_id 0 */
    if (pid == 0) {
        if (ssdv_dec_init(&h->dec, h->pkt_size) != SSDV_OK) return -3;
    }

    if (ssdv_dec_set_buffer(&h->dec, out_jpeg, out_cap) != SSDV_OK) return -3;
    r = ssdv_dec_feed(&h->dec, (uint8_t *)packet);
    if (r != SSDV_OK && r != SSDV_FEED_ME) return -3;

    if (eoi || r == SSDV_OK) {
        if (ssdv_dec_get_jpeg(&h->dec, &jpeg, &jpeg_len) != SSDV_OK) return -3;
        if (jpeg_len > out_cap) return -3;
        if (jpeg != out_jpeg) {
            memmove(out_jpeg, jpeg, jpeg_len);
        }
        *out_len = jpeg_len;
        return 1;
    }

    *out_len = 0;
    return 0;
}
