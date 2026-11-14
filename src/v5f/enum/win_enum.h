/*
 * win_enum.h - Windows USB enumeration emulation
 *
 * Uses only usbhs_host.h APIs; must be called from the main loop (not ISR).
 */
#ifndef LAT_WIN_ENUM_H
#define LAT_WIN_ENUM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call once at startup after usbhs_host_init(). */
void win_enum_init(void);

/* Call every main-loop iteration after usbhs_host_poll(). */
void win_enum_poll(uint64_t now);

/* PC command handlers (see return LAT_PC_ST_*). */
uint8_t win_enum_reenumerate(void);
uint8_t win_enum_device_info(uint8_t *dst, uint32_t cap, uint32_t *len);
uint8_t win_enum_object(uint8_t kind, uint8_t index, uint8_t *dst, uint32_t cap, uint32_t *len);

#ifdef __cplusplus
}
#endif

#endif /* LAT_WIN_ENUM_H */
