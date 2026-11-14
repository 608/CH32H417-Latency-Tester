/*
 * pc_link_port.h - Hardware abstraction for pc_link.c (V3F vs host unit tests).
 */
#ifndef LAT_PC_LINK_PORT_H
#define LAT_PC_LINK_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "lat_ring.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Chip UID (16 bytes; first 8 bytes from ESIG, rest zero). */
void pc_port_uuid(uint8_t out[16]);

/* Build timestamp (12-byte ASCII from app metadata). */
void pc_port_build(uint8_t out[12]);

/* Random number for session ID generation. */
uint32_t pc_port_random(void);

/* True if V5F passed validation and is running. */
bool pc_port_v5f_running(void);

/* V5F diagnostic code when stopped (non-zero puts GET_STATUS in fault state). */
uint32_t pc_port_v5f_diag(void);

/* Release all triggers PA0..PA7. */
void pc_port_release_triggers(void);

/* Forward non-PC-link IPC records (PONG, UART logs, etc.). */
void pc_port_ipc_record(const LatRingRecord *rec);

/* True if UART log buffer can accept more output. */
bool pc_port_log_ready(void);

/* Emit one ASCII line (no newline) to UART log. */
void pc_port_log(const char *line);

/* Stop USB, write STAY boot magic, and reboot into bootloader (does not return). */
void pc_port_enter_bootloader(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_PC_LINK_PORT_H */
