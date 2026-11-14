/*
 * meas_port.h - Hardware and enumeration port for the measurement engine
 *
 * Implemented by meas_port_v5f.c on V5F (and mocked on host tests).
 * All meas_port_* functions are called from the main loop except meas_port_marker_high_now,
 * which is called from the USBHS ISR.
 */
#ifndef LAT_MEAS_PORT_H
#define LAT_MEAS_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MEAS_TICKS_PER_US       (100u)
#define MEAS_TICKS_PER_MS       (100000u)

/* ---- Trigger (TIM9_CH1 compare interrupt) ---- */
#define MEAS_TRIG_ASSERT        ((uint8_t)0x01u) /* BCR: drive Low */
#define MEAS_TRIG_RELEASE       ((uint8_t)0x02u) /* BSHR: release (high-Z) */
#define MEAS_TRIG_MARKER        ((uint8_t)0x80u) /* Read t_m after pin write, then drive PA7 High (calibration) */

/* Trigger pins T0..T7 = PA0..PA7 */
#define MEAS_PIN_MASK           ((uint8_t)0xFFu)
#define MEAS_PIN_REF            ((uint8_t)0x04u) /* T2 = PA2: calibration reference edge (TIM9_CH3) */
#define MEAS_PIN_MARKER         ((uint8_t)0x80u) /* T7 = PA7: calibration marker */

typedef struct MeasTrigResult
{
    uint64_t sched;    /* t0_sched (compare value) */
    uint64_t t_isr;    /* t0_isr (timestamp at ISR entry) */
    uint64_t t_tick;   /* t0_tick (timestamp just before BCR/BSHR write) */
    uint64_t t_m;      /* Timestamp just before PA7 write when MEAS_TRIG_MARKER is set, else 0 */
} MeasTrigResult;

/* 64-bit current time (TIM9 ticks, 10ns). */
uint64_t meas_port_now(void);

/*
 * Arm a single TIM9_CH1 compare interrupt at `when` (< 21s from now).
 * Overwrites any previous armed trigger; fires immediately if `when` has already passed.
 */
void meas_port_trig_arm(uint64_t when, uint8_t action, uint8_t mask);
/* Called from V5F TIM9 ISR on CH1 compare match; `t_isr` is the entry timestamp. */
void meas_tim9_ch1(uint64_t t_isr);

bool meas_port_trig_cancel(void);
bool meas_port_trig_fired(MeasTrigResult *out);

/* Immediate pin control: release_mask via BSHR first, then assert_mask via BCR. */
void meas_port_pins(uint8_t release_mask, uint8_t assert_mask);

uint32_t meas_port_hw_random(void);

/* Frame and microframe number at time t; returns false if no SOF reference. */
bool meas_port_frame_at(uint64_t t, uint16_t *frame, uint8_t *uframe);

/* ---- Enumerated device ---- */
typedef struct MeasDev
{
    uint32_t generation;    /* device_generation */
    uint32_t seq;           /* Enumeration sequence (changes on re-enumeration) */
    uint32_t period_ticks;  /* Interrupt IN polling period (ticks) */
    uint16_t mps;           /* wMaxPacketSize */
    uint8_t  speed;         /* 1 LS, 2 FS, 3 HS */
    uint8_t  reserved;
} MeasDev;

/*
 * Find interrupt IN endpoint on `iface`. If ep_in==0, uses first interrupt IN;
 * if iface==0xFF, uses default target chosen by enumeration and updates *iface and *ep_in.
 */
bool meas_port_dev_find(uint8_t *iface, uint8_t *ep_in, MeasDev *out);
bool meas_port_dev_alive(const MeasDev *dev);
/* Enable/disable forwarding of iface/ep_in completions to measure_isr_completion (ref-counted). */
bool meas_port_listen(uint8_t iface, uint8_t ep_in, bool on);

/* ---- Calibration mode ---- */
/*
 * on: configure PA2 as TIM9_CH3 compare output (AF4, push-pull, Low) and PA7 as push-pull (Low).
 * off: stop compare output and restore PA2/PA7 to open-drain. Leaves PA0/PA1 untouched.
 */
void meas_port_calib_pins(bool on);
void meas_port_calib_ref_arm(uint64_t when);
void meas_port_calib_ref_low(void);
void meas_port_marker(bool high);
/* Called from ISR: read timestamp, drive PA7 High, and return the timestamp. */
uint64_t meas_port_marker_high_now(void);
/*
 * Single-shot IN for c1 (calibration mode 2): begin stops periodic polling,
 * issue sends one IN, end restores periodic polling.
 */
bool meas_port_calib_in_begin(uint8_t iface, uint8_t ep_in);
bool meas_port_calib_in_issue(void);
void meas_port_calib_in_end(void);

/* ---- Polling measurement buffers ---- */
uint32_t *meas_port_poll_pool(uint32_t *words);
uint8_t *meas_port_poll_longs(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_MEAS_PORT_H */
