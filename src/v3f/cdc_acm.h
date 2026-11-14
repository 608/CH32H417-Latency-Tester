/*
 * cdc_acm.h - USBFS CDC-ACM device for PC communication.
 *
 * RX bytes are queued in the ISR and forwarded by cdc_acm_poll() to pc_link_rx_bytes().
 * TX pulls 64-byte packets from pc_link_tx_fill() and sends a ZLP after an exact
 * 64-byte packet when no more data is pending.
 */
#ifndef LAT_CDC_ACM_H
#define LAT_CDC_ACM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CdcAcmStats
{
    bool configured;
    bool dtr;
    uint32_t bus_resets;
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t rx_pauses;
} CdcAcmStats;

/* Build descriptors and start USBFS device. uuid12 is the 96-bit chip UID. */
void cdc_acm_start(const uint8_t uuid12[12]);

/* Poll from main loop. */
void cdc_acm_poll(void);

void cdc_acm_get_stats(CdcAcmStats *out);

#ifdef __cplusplus
}
#endif

#endif /* LAT_CDC_ACM_H */
