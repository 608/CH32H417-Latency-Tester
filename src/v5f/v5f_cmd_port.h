/*
 * v5f_cmd_port.h - Hardware abstraction for v5f_cmd.c
 */
#ifndef LAT_V5F_CMD_PORT_H
#define LAT_V5F_CMD_PORT_H

#include <stdint.h>

#include "lat_ipc.h"
#include "lat_ring.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Release pins in release_mask (BSHR) first, then assert assert_mask low (BCR). */
void v5f_port_trigger_write(uint8_t release_mask, uint8_t assert_mask);

void v5f_port_ping(const LatRingRecord *rec);

/* Fill the USBHS-host-owned fields in GET_STATUS. */
void v5f_port_host_status(LatHostStatus *status);

/* Export the enumeration record and return its size. */
uint32_t v5f_port_enum_trace_export(uint8_t *destination, uint32_t capacity);
/* Previous connection generation's trace; returns 0 if none saved. */
uint32_t v5f_port_enum_trace_previous(uint8_t *destination, uint32_t capacity);

/* Enumeration command handlers (return LAT_PC_ST_*). */
uint8_t v5f_port_device_info(uint8_t *resp, uint32_t cap, uint32_t *len);
uint8_t v5f_port_reenumerate(void);
uint8_t v5f_port_enum_object(uint8_t kind, uint8_t index, uint8_t *dst, uint32_t cap, uint32_t *len);

#ifdef __cplusplus
}
#endif

#endif /* LAT_V5F_CMD_PORT_H */
