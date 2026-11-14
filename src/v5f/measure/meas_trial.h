/*
 * meas_trial.h - Trial state machine
 *
 * Hardware access goes through meas_port.h; called only from the main loop.
 */
#ifndef LAT_MEAS_TRIAL_H
#define LAT_MEAS_TRIAL_H

#include <stdbool.h>
#include <stdint.h>

#include "meas_port.h"
#include "meas_watch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MEAS_TRIAL_START_LEN    (24u)
#define MEAS_TRIAL_RESULT_HDR   (94u)
#define MEAS_TRIAL_RESULT_MAX   (MEAS_TRIAL_RESULT_HDR + MEAS_REPORT_MAX)
#define MEAS_RUN_DONE_LEN       (28u)
/* Lead time for arming trigger (margin for ISR latency and main loop, 20us). */
#define MEAS_TRIAL_LEAD         ((uint64_t)20u * MEAS_TICKS_PER_US)
/* Extra margin for timeout check (ISR timestamp to record delivery, 1ms). */
#define MEAS_TRIAL_MARGIN       ((uint64_t)1u * MEAS_TICKS_PER_MS)
/* Max random_max_us (kept below 21s TIM9 wraparound). */
#define MEAS_RANDOM_MAX_US      ((uint32_t)10000000u)

/* fail_kind */
#define MEAS_FAIL_NONE          ((uint8_t)0u)
#define MEAS_FAIL_TIMEOUT       ((uint8_t)1u)
#define MEAS_FAIL_NOT_RELEASED  ((uint8_t)2u)
#define MEAS_FAIL_CONFIRM       ((uint8_t)3u)
#define MEAS_FAIL_OVERFLOW      ((uint8_t)4u)
#define MEAS_FAIL_DETACHED      ((uint8_t)5u)
#define MEAS_FAIL_ABORTED       ((uint8_t)6u)
#define MEAS_FAIL_KINDS         (7u)

/* completion_reason */
#define MEAS_DONE_NORMAL        ((uint8_t)0u)
#define MEAS_DONE_ABORTED       ((uint8_t)1u)
#define MEAS_DONE_DETACHED      ((uint8_t)2u)
#define MEAS_DONE_INTERNAL      ((uint8_t)3u)
#define MEAS_DONE_OVERFLOW      ((uint8_t)4u)

/* c1_status */
#define MEAS_C1_MEASURED_FS     ((uint8_t)0u)
#define MEAS_C1_ESTIMATED_FS    ((uint8_t)1u)
#define MEAS_C1_UNKNOWN         ((uint8_t)2u)

/* speed */
#define MEAS_SPEED_LS           ((uint8_t)1u)
#define MEAS_SPEED_FS           ((uint8_t)2u)
#define MEAS_SPEED_HS           ((uint8_t)3u)

/* Calibration flags (SET_CALIBRATION) */
#define MEAS_CAL_C0_VALID       ((uint8_t)0x01u)
#define MEAS_CAL_CM_VALID       ((uint8_t)0x02u)
#define MEAS_CAL_C1_VALID       ((uint8_t)0x04u)

typedef struct MeasCalib
{
    int32_t  c0;
    int32_t  c_m;
    int32_t  c1_fs;
    uint32_t c0_unc;
    uint32_t c_m_unc;
    uint32_t c1_unc;
    uint8_t  valid;
    uint8_t  reserved[3];
} MeasCalib;

typedef struct MeasTrialCfg
{
    uint32_t run_id;
    uint8_t  trigger_mask;
    uint8_t  measure_release;
    uint8_t  settle_reports;
    uint8_t  reserved;
    uint16_t count;
    uint16_t timeout_ms;
    uint16_t release_timeout_ms;
    uint16_t inter_trial_ms;
    uint32_t random_max_us;
    uint32_t seed;
} MeasTrialCfg;

/* Single evaluated report produced by the ISR completion handler. */
typedef struct MeasRecord
{
    uint64_t t1;       /* t1_irq */
    uint32_t seq;
    uint16_t frame;
    uint16_t len;
    uint8_t  uframe;
    uint8_t  state;    /* MEAS_ST_* */
    uint8_t  reserved[2];
} MeasRecord;

typedef struct MeasTrial
{
    uint8_t  phase;
    uint8_t  active;          /* 1 until RUN_DONE is built */
    uint8_t  in_trial;        /* 1 while a trial is in progress (aborts emit a result) */
    uint8_t  result_pending;  /* 1 while waiting to send TRIAL_RESULT */
    uint8_t  done_pending;    /* 1 while waiting to send RUN_DONE */
    uint8_t  done_reason;
    uint8_t  fail_kind;
    uint8_t  rel_fail_kind;
    uint8_t  dirty;           /* Non-RELEASED report arrived before trigger fired */
    uint8_t  c1_status;
    uint8_t  settle_count;
    uint8_t  speed;
    uint16_t index;           /* Next trial_index (0-based) */
    uint16_t completed;
    uint16_t succeeded;
    uint16_t valid_count;
    uint16_t fail_counts[MEAS_FAIL_KINDS];
    uint16_t report_len;
    uint32_t rng;
    uint32_t random_max_ticks;
    uint32_t delay;           /* Press random delay (ticks) */
    uint32_t result_len;
    uint64_t settle_start;    /* Start of settle wait (base for release_timeout) */
    uint64_t settle_after;    /* Only count reports completed after this time */
    uint64_t deadline;
    uint64_t t0_sched;
    uint64_t t0_isr;
    uint64_t t0_tick;
    uint64_t t1;
    uint64_t confirm_t1;
    uint64_t rel_t0_tick;
    uint64_t rel_t1;
    uint16_t t1_frame;
    uint8_t  t1_uframe;
    uint8_t  pins;            /* Currently asserted trigger mask */
    MeasTrialCfg cfg;
    MeasWatch watch;
    MeasCalib cal;
    MeasDev dev;
    uint8_t  report[MEAS_REPORT_MAX];
    uint8_t  result[MEAS_TRIAL_RESULT_MAX];
    uint8_t  done_body[MEAS_RUN_DONE_LEN];
} MeasTrial;

/* Parse 24-byte TRIAL_START payload; range validation is done by caller. */
void meas_trial_cfg_parse(const uint8_t *p, MeasTrialCfg *cfg);
void meas_calib_parse(const uint8_t *p, MeasCalib *cal);
uint8_t meas_c1_status(const MeasCalib *cal, uint8_t speed);
bool meas_latency_valid(uint8_t fail_kind, uint64_t t1_irq, const MeasCalib *cal, uint8_t c1_status,
                        uint8_t measurement_kind);

void meas_trial_init(MeasTrial *t);
void meas_trial_start(MeasTrial *t, const MeasTrialCfg *cfg, const MeasWatch *w, const MeasCalib *cal,
                      const MeasDev *dev, uint64_t now);
void meas_trial_on_record(MeasTrial *t, const MeasRecord *r, const uint8_t *data, uint64_t now);
void meas_trial_poll(MeasTrial *t, uint64_t now);
void meas_trial_abort(MeasTrial *t, uint8_t reason, uint64_t now);
/* Stop without emitting TRIAL_RESULT or RUN_DONE (e.g. new session). */
void meas_trial_kill(MeasTrial *t);

bool meas_trial_result(const MeasTrial *t, const uint8_t **body, uint32_t *len);
void meas_trial_result_sent(MeasTrial *t, uint64_t now);
bool meas_trial_done(const MeasTrial *t, const uint8_t **body);
void meas_trial_done_sent(MeasTrial *t);

bool meas_trial_running(const MeasTrial *t);
uint8_t meas_trial_asserted(const MeasTrial *t);

#ifdef __cplusplus
}
#endif

#endif /* LAT_MEAS_TRIAL_H */
