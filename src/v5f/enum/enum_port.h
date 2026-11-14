/*
 * enum_port.h - External platform hooks required by win_enum and app_init
 */
#ifndef LAT_ENUM_PORT_H
#define LAT_ENUM_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "win_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

void enum_port_log(const char *line);

/* Enqueue PC event (ENUM_PROGRESS, DEVICE_READY, ERROR); false if full. */
bool enum_port_emit(uint8_t pc_type, const uint8_t *body, uint32_t len);

/* Current PC session ID (changes discard unsent events). */
uint32_t enum_port_session(void);

/* Current TIM9 timestamp. */
uint64_t enum_port_now(void);

/* Current host profile. */
void enum_port_profile(WinHostProfile *out);

/* Vendor code table lookup. */
bool enum_port_vendor_lookup(uint16_t vid, uint16_t pid, uint16_t bcd_device, uint8_t *state, uint8_t *code);

/* Add/replace learned vendor code entry (dropped if table is full). */
void enum_port_vendor_learn(uint16_t vid, uint16_t pid, uint16_t bcd_device, uint8_t state, uint8_t code);

#ifdef __cplusplus
}
#endif

#endif /* LAT_ENUM_PORT_H */
