/*
 * v5f_cmd.h - V5F PC command framework
 *
 * Processes LAT_CMD_PC records from IPC_CMD, sends LAT_EVT_PC_RESP to
 * IPC_EVT_SAFE, handles IPC_HEADER session/overflow/quiesce requests, and
 * forwards ISR events to the safe event ring.
 *
 * Hardware-independent; hardware access goes through v5f_cmd_port.h.
 */
#ifndef LAT_V5F_CMD_H
#define LAT_V5F_CMD_H

#include <stdbool.h>
#include <stdint.h>

#include "lat_ring.h"

#ifdef __cplusplus
extern "C" {
#endif

void v5f_cmd_init(void);

/* Poll per main loop: handle IPC requests, ISR events, and pending terminal events. */
void v5f_cmd_poll(void);

/* Pop and process IPC_CMD entries; defers if IPC_EVT_SAFE cannot fit a max response. */
void v5f_cmd_service(void);

/* Publish host status (GET_STATUS) and capabilities (every 1ms). */
void v5f_cmd_publish(void);

uint32_t v5f_cmd_session(void);

uint32_t v5f_cmd_count(void);
uint32_t v5f_cmd_deferred(void);

/*
 * Stage a terminal event (RUN_DONE 28B or POLL_RESULT 88B) to be sent once
 * space is available in IPC_EVT_SAFE. Returns false on length mismatch or if
 * the slot is still occupied.
 */
bool v5f_cmd_finish_run(uint8_t pc_type, const uint8_t *body, uint32_t len);

/*
 * Enqueue a reliable PC event into IPC_EVT_SAFE. Returns false if full;
 * returns true without enqueueing if no session is active or quiesced.
 */
bool v5f_cmd_emit_event(uint8_t pc_type, const uint8_t *body, uint32_t len);

/*
 * Enqueue a lossy event (LAT_EVT_MONITOR or LAT_EVT_PC_LOG). Drops and
 * increments ring drop count if full.
 */
bool v5f_cmd_emit_lossy(uint8_t ipc_type, uint8_t pc_type, const uint8_t *body, uint32_t len);

/*
 * Record an event from USBHS ISR (which must not write to IPC rings directly).
 * Single-writer only (USBHS ISR, or main loop with USBHS IRQ masked).
 * Sets overflow state if the 64-entry staging queue is full.
 */
bool v5f_cmd_isr_event(uint8_t pc_type, uint32_t word, uint8_t arg0, uint8_t arg1, uint16_t code);

const uint8_t *v5f_cmd_host_profile(void);
bool v5f_cmd_vendor_lookup(uint16_t vid, uint16_t pid, uint16_t bcd_device, uint8_t *state, uint8_t *code);
void v5f_cmd_vendor_learn(uint16_t vid, uint16_t pid, uint16_t bcd_device, uint8_t state, uint8_t code);

#ifdef __cplusplus
}
#endif

#endif /* LAT_V5F_CMD_H */
