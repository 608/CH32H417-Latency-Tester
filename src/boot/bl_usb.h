/*
 * bl_usb.h - Bootloader USBFS vendor-class interface.
 *
 * Uses WinUSB via BOS MS OS 2.0 descriptors on interface 0 (EP1 OUT / EP2 IN, 64B).
 * NAKs OUT until the current 64-byte request's response is queued.
 */
#ifndef LAT_BL_USB_H
#define LAT_BL_USB_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bl_usb_start(const uint8_t uuid12[12]);

/* Poll from main loop; processes one pending request and queues response. */
bool bl_usb_poll(uint8_t *cmd, uint8_t *status);

/* True once the RESET_APP response transfer has completed. */
bool bl_usb_reset_ready(void);

bool bl_usb_configured(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_BL_USB_H */
