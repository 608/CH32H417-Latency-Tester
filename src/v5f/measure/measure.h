/*
 * measure.h - V5F measurement engine
 *
 * Processes measurement commands after v5f_cmd.c validates framing, run_id, and ranges.
 * Only measure_isr_completion is called from the USBHS ISR; all other functions are
 * called from the main loop.
 */
#ifndef LAT_MEASURE_H
#define LAT_MEASURE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Periodic transaction completion record passed to measure_isr_completion. */
#define MEAS_RES_DATA     ((uint8_t)0u)
#define MEAS_RES_NAK      ((uint8_t)1u)
#define MEAS_RES_ERROR    ((uint8_t)2u) /* STALL, no response, timeout, or toggle mismatch */

typedef struct MeasCompletion
{
    uint64_t t1;          /* Timestamp at completion ISR entry (t1_irq) */
    uint32_t generation;
    uint32_t seq;
    uint16_t len;
    uint16_t frame;
    uint8_t  uframe;
    uint8_t  result;      /* MEAS_RES_* */
    uint8_t  reserved[2];
} MeasCompletion;

void measure_init(void);
void measure_poll(uint64_t now);

/* Called from USBHS ISR on iface/ep_in completion; `data` is valid only when result == MEAS_RES_DATA. */
void measure_isr_completion(uint8_t iface, uint8_t ep_in, const MeasCompletion *c, const uint8_t *data);

/* ---- Commands (called after v5f_cmd.c validation) ---- */
uint8_t measure_trial_start(const uint8_t *p, const uint8_t *watch, const uint8_t *calib);
uint8_t measure_trial_stop(uint32_t run_id);
uint8_t measure_poll_start(const uint8_t *p);
uint8_t measure_poll_stop(uint32_t run_id);
uint8_t measure_monitor_start(const uint8_t *p);
void measure_monitor_stop(void);
uint8_t measure_calib_mode(const uint8_t *p, const uint8_t *watch);
uint8_t measure_poll_object(uint8_t kind, uint8_t index, uint32_t source_id, uint8_t *dst, uint32_t cap,
                            uint32_t *len);

/* ---- State transitions (from v5f_cmd.c) ---- */
/* New session: stop everything without emitting RUN_DONE / POLL_RESULT. */
void measure_session_reset(void);
/* Event queue overflow: abort active trial or poll run with OVERFLOW. */
void measure_overflow(void);
/* Quiesce before jumping to bootloader: stop calib/monitor and release triggers. */
void measure_quiesce(void);

/* ---- Status ---- */
bool measure_trial_running(void);
bool measure_poll_running(void);
bool measure_calib_active(void);    /* mode 1 or 2 */
bool measure_monitor_active(void);
uint8_t measure_run_kind(void);     /* LAT_PC_RUN_* */
uint32_t measure_active_run_id(void);
uint8_t measure_trigger_asserted(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_MEASURE_H */
