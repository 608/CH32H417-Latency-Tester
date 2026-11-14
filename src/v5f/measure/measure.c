/*
 * measure.c - V5F measurement engine (see measure.h)
 *
 * All ISR-to-main-loop rings are SPSC: write entry, issue fence, then advance head.
 * Main loop only modifies ISR-read configuration while that mode (s.run or s.mon_on) is 0.
 */
#include "measure.h"

#include "lat_bytes.h"
#include "lat_fence.h"
#include "lat_fmt.h"
#include "lat_ipc.h"
#include "lat_pc_proto.h"
#include "meas_poll.h"
#include "meas_port.h"
#include "meas_trial.h"
#include "meas_watch.h"
#include "v5f_cmd.h"

#define REC_RING          (256u)   /* Evaluated records (32ms at 8kHz) */
#define CAP_SLOTS         (4u)     /* Captured reports on state change */
#define MON_RING          (8u)     /* Monitor reports */
#define MON_HDR_LEN       (16u)
#define CAL_LEAD_TICKS    ((uint64_t)200u * MEAS_TICKS_PER_US)
#define CAL_STUCK_TICKS   ((uint64_t)50u * MEAS_TICKS_PER_MS)
#define CAL_IN_WAIT_TICKS ((uint64_t)100u * MEAS_TICKS_PER_MS)
#define CAL_PERIOD_MIN_MS (10u)
#define CAL_PERIOD_MAX_MS (60000u)
#define CAL_T0_MASK       ((uint8_t)0x01u)   /* T0 = PA0 */
#define LOG_MAX           (100u)

_Static_assert((REC_RING & (REC_RING - 1u)) == 0u, "REC_RING must be a power of two");
_Static_assert((CAP_SLOTS & (CAP_SLOTS - 1u)) == 0u, "CAP_SLOTS must be a power of two");
_Static_assert((MON_RING & (MON_RING - 1u)) == 0u, "MON_RING must be a power of two");
_Static_assert(MEAS_TRIAL_RESULT_MAX <= LAT_IPC_ENV_BODY_MAX, "TRIAL_RESULT must fit an IPC record");
_Static_assert((MON_HDR_LEN + MEAS_REPORT_MAX) <= LAT_IPC_ENV_BODY_MAX, "MONITOR_REPORT must fit an IPC record");
_Static_assert(MEAS_RUN_DONE_LEN == LAT_PC_RUN_DONE_LEN, "RUN_DONE length");
_Static_assert(MEAS_POLL_RESULT_LEN == LAT_PC_POLL_RESULT_LEN, "POLL_RESULT length");

typedef struct CapSlot
{
    volatile uint32_t stamp;   /* Incremented by ISR on each write */
    uint32_t seq;
    uint16_t len;
    uint8_t  data[MEAS_REPORT_MAX];
} CapSlot;

typedef struct MonSlot
{
    uint64_t t1;
    uint32_t generation;
    uint16_t len;
    uint8_t  iface;
    uint8_t  ep;
    uint8_t  data[MEAS_REPORT_MAX];
} MonSlot;

typedef struct MeasState
{
    /* ---- Monitor ---- */
    volatile uint8_t mon_on;       /* Read by ISR */
    uint8_t  mon_iface;
    uint8_t  mon_ep;
    uint8_t  mon_change_only;
    uint8_t  mon_listening;
    uint8_t  mon_first;            /* ISR: always send next report */
    uint16_t mon_last_len;         /* ISR: length of last sent report */
    uint64_t mon_min_ticks;
    uint64_t mon_last_t1;
    MeasDev  mon_dev;
    uint32_t mon_drop;

    /* ---- Trial and polling ---- */
    volatile uint8_t run;          /* LAT_PC_RUN_NONE / TRIAL / POLL (read by ISR) */
    uint8_t  run_iface;
    uint8_t  run_ep;
    uint8_t  cap_last_state;       /* ISR: last evaluated state */
    uint32_t run_id;
    MeasDev  dev;
    MeasWatch watch;
    volatile uint32_t rec_drop;    /* ISR: dropped records when ring full */
    uint32_t rec_drop_seen;

    uint64_t poll_start;
    uint64_t poll_end;
    uint8_t  poll_finishing;       /* POLL_RESULT built, waiting for v5f_cmd_finish_run */
    uint8_t  poll_done_valid;      /* Last completed poll result is valid */
    uint32_t poll_done_run_id;
    uint8_t  poll_result[MEAS_POLL_RESULT_LEN];

    /* ---- Calibration mode ---- */
    uint8_t  cal_mode;             /* 0, 1, 2 */
    uint8_t  cal_phase;
    uint8_t  cal_iface;
    uint8_t  cal_ep;
    uint32_t cal_period;           /* Ticks */
    uint32_t cal_n;
    uint64_t cal_next;
    uint64_t cal_c;
    uint64_t cal_at;
    MeasDev  cal_dev;
    volatile uint8_t cal2_armed;   /* ISR: drive PA7 High on next data completion */
    volatile uint8_t cal2_done;
    uint64_t cal2_t1;
    uint64_t cal2_tm;
} MeasState;

static MeasState s;
static MeasTrial s_trial;
static MeasPoll s_poll;

static MeasRecord s_rec[REC_RING];
static volatile uint32_t s_rec_head;   /* Written by ISR */
static volatile uint32_t s_rec_tail;   /* Written by main loop */
static CapSlot s_cap[CAP_SLOTS];
static uint32_t s_cap_head;            /* ISR-only */
static uint8_t s_cap_copy[MEAS_REPORT_MAX];

static MonSlot s_mon[MON_RING];
static volatile uint32_t s_mon_head;
static volatile uint32_t s_mon_tail;
static uint8_t s_mon_last[MEAS_REPORT_MAX]; /* ISR: last sent payload */
static uint8_t s_out[MON_HDR_LEN + MEAS_REPORT_MAX];

/* ------------------------------------------------------------------ */
/* Logging                                                            */
/* ------------------------------------------------------------------ */

static void put_i64(LatStr *st, int64_t v)
{
    uint64_t m;

    if (v < 0)
    {
        lat_str_putc(st, '-');
        m = (uint64_t)(-v);
    }
    else
    {
        m = (uint64_t)v;
    }
    lat_str_u32(st, (m > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)m);
}

static void put_hex64(LatStr *st, uint64_t v)
{
    lat_str_hex32(st, (uint32_t)(v >> 32));
    lat_str_hex32(st, (uint32_t)v);
}

/* Send to both PC LOG event and UART log. */
static void meas_log(const char *text, uint32_t len)
{
    uint8_t body[1u + LOG_MAX];

    body[0] = 0u; /* Info */
    lat_bytes_copy(&body[1], (const uint8_t *)text, len);
    (void)v5f_cmd_emit_lossy(LAT_EVT_PC_LOG, LAT_PC_EVT_LOG, body, 1u + len);
    (void)lat_ring_push_lossy(&g_ring_evt_lossy, LAT_EVT_LOG, 0u, text, len);
}

/* ------------------------------------------------------------------ */
/* ISR completion handler                                             */
/* ------------------------------------------------------------------ */

static void isr_trial(const MeasCompletion *c, const uint8_t *data)
{
    uint32_t head = s_rec_head;
    uint8_t st = meas_watch_eval(&s.watch, data, c->len);
    MeasRecord *r;

    if ((st != MEAS_ST_NONE) && (st != s.cap_last_state))
    {
        CapSlot *cs = &s_cap[s_cap_head & (CAP_SLOTS - 1u)];

        s.cap_last_state = st;
        cs->stamp = cs->stamp + 1u;
        cs->seq = c->seq;
        cs->len = c->len;
        lat_bytes_copy(cs->data, data, c->len);
        LAT_FENCE_RW();
        cs->stamp = cs->stamp + 1u;
        s_cap_head++;
    }
    if ((uint32_t)(head - s_rec_tail) >= REC_RING)
    {
        s.rec_drop = s.rec_drop + 1u;
        return;
    }
    r = &s_rec[head & (REC_RING - 1u)];
    r->t1 = c->t1;
    r->seq = c->seq;
    r->frame = c->frame;
    r->uframe = c->uframe;
    r->len = c->len;
    r->state = st;
    LAT_FENCE_RW();
    s_rec_head = head + 1u;
}

static void isr_monitor(uint8_t iface, uint8_t ep, const MeasCompletion *c, const uint8_t *data)
{
    uint32_t head = s_mon_head;
    MonSlot *m;

    if (s.mon_first == 0u)
    {
        if ((s.mon_min_ticks != 0u) && ((c->t1 - s.mon_last_t1) < s.mon_min_ticks))
        {
            return;
        }
        if ((s.mon_change_only != 0u) && (c->len == s.mon_last_len) &&
            lat_bytes_equal(data, s_mon_last, c->len))
        {
            return;
        }
    }
    if ((uint32_t)(head - s_mon_tail) >= MON_RING)
    {
        s.mon_drop++; /* Do not update baseline; resend on next report */
        return;
    }
    m = &s_mon[head & (MON_RING - 1u)];
    m->t1 = c->t1;
    m->generation = c->generation;
    m->len = c->len;
    m->iface = iface;
    m->ep = ep;
    lat_bytes_copy(m->data, data, c->len);
    LAT_FENCE_RW();
    s_mon_head = head + 1u;
    s.mon_first = 0u;
    s.mon_last_t1 = c->t1;
    s.mon_last_len = c->len;
    lat_bytes_copy(s_mon_last, data, c->len);
}

void measure_isr_completion(uint8_t iface, uint8_t ep_in, const MeasCompletion *c, const uint8_t *data)
{
    bool is_data = (c->result == MEAS_RES_DATA) && (data != (const uint8_t *)0) && (c->len <= MEAS_REPORT_MAX);

    /* Calibration (c1): run first since it is timing-sensitive. */
    if ((s.cal2_armed != 0u) && is_data && (iface == s.cal_iface) && (ep_in == s.cal_ep))
    {
        uint64_t tm = meas_port_marker_high_now();

        s.cal2_armed = 0u;
        s.cal2_t1 = c->t1;
        s.cal2_tm = tm;
        LAT_FENCE_RW();
        s.cal2_done = 1u;
    }

    if ((s.run != LAT_PC_RUN_NONE) && (iface == s.run_iface) && (ep_in == s.run_ep))
    {
        if (s.run == LAT_PC_RUN_POLL)
        {
            if (c->result == MEAS_RES_DATA)
            {
                meas_poll_data(&s_poll, c->t1);
            }
            else if (c->result == MEAS_RES_NAK)
            {
                meas_poll_nak(&s_poll);
            }
            else
            {
                meas_poll_error(&s_poll);
            }
        }
        else if (is_data)
        {
            isr_trial(c, data);
        }
    }

    if ((s.mon_on != 0u) && is_data && (iface == s.mon_iface) && (ep_in == s.mon_ep))
    {
        isr_monitor(iface, ep_in, c, data);
    }
}

/* ------------------------------------------------------------------ */
/* Monitor                                                            */
/* ------------------------------------------------------------------ */

static void monitor_unlisten(void)
{
    if (s.mon_listening != 0u)
    {
        (void)meas_port_listen(s.mon_iface, s.mon_ep, false);
        s.mon_listening = 0u;
    }
}

void measure_monitor_stop(void)
{
    s.mon_on = 0u;
    LAT_FENCE_RW();
    monitor_unlisten();
    s_mon_tail = s_mon_head;
}

uint8_t measure_monitor_start(const uint8_t *p)
{
    uint8_t iface = p[0];
    uint8_t ep = p[1];
    MeasDev dev;

    measure_monitor_stop();
    if (!meas_port_dev_find(&iface, &ep, &dev))
    {
        return LAT_PC_ST_NO_DEVICE;
    }
    s.mon_iface = iface;
    s.mon_ep = ep;
    s.mon_change_only = (p[2] != 0u) ? 1u : 0u;
    s.mon_min_ticks = (uint64_t)lat_get_u32(&p[4]) * MEAS_TICKS_PER_US;
    s.mon_dev = dev;
    s.mon_first = 1u;
    s.mon_last_len = 0u;
    s.mon_last_t1 = 0u;
    if (!meas_port_listen(iface, ep, true))
    {
        return LAT_PC_ST_NO_DEVICE;
    }
    s.mon_listening = 1u;
    LAT_FENCE_RW();
    s.mon_on = 1u;
    return LAT_PC_ST_OK;
}

static void monitor_service(void)
{
    if (s.mon_on == 0u)
    {
        return;
    }
    /* After detach/re-enumeration, re-attach when the same interface reappears. */
    if ((s.mon_listening != 0u) && !meas_port_dev_alive(&s.mon_dev))
    {
        s.mon_on = 0u;
        LAT_FENCE_RW();
        monitor_unlisten();
        s.mon_on = 1u; /* Keep monitor active */
    }
    if (s.mon_listening == 0u)
    {
        uint8_t iface = s.mon_iface;
        uint8_t ep = s.mon_ep;
        MeasDev dev;

        if (meas_port_dev_find(&iface, &ep, &dev) && (iface == s.mon_iface) && (ep == s.mon_ep))
        {
            s.mon_on = 0u;
            LAT_FENCE_RW();
            s.mon_dev = dev;
            s.mon_first = 1u;
            s.mon_last_len = 0u;
            if (meas_port_listen(iface, ep, true))
            {
                s.mon_listening = 1u;
            }
            LAT_FENCE_RW();
            s.mon_on = 1u;
        }
    }

    for (;;)
    {
        uint32_t tail = s_mon_tail;
        const MonSlot *m;

        if (tail == s_mon_head)
        {
            break;
        }
        LAT_FENCE_RW();
        m = &s_mon[tail & (MON_RING - 1u)];
        lat_put_u32(&s_out[0], m->generation);
        s_out[4] = m->iface;
        s_out[5] = m->ep;
        lat_put_u16(&s_out[6], m->len);
        lat_put_u64(&s_out[8], m->t1);
        lat_bytes_copy(&s_out[MON_HDR_LEN], m->data, m->len);
        LAT_FENCE_RW();
        s_mon_tail = tail + 1u;
        /* Droppable ring; drops are counted in DROP_COUNTERS. */
        (void)v5f_cmd_emit_lossy(LAT_EVT_MONITOR, LAT_PC_EVT_MONITOR_REPORT, s_out, MON_HDR_LEN + (uint32_t)m->len);
    }
}

bool measure_monitor_active(void)
{
    return s.mon_on != 0u;
}

/* ------------------------------------------------------------------ */
/* Trial and polling common                                           */
/* ------------------------------------------------------------------ */

static void run_end(void)
{
    s.run = LAT_PC_RUN_NONE;
    LAT_FENCE_RW();
    (void)meas_port_listen(s.run_iface, s.run_ep, false);
}

/* Find and copy captured report matching `seq`. */
static const uint8_t *capture_find(uint32_t seq, uint16_t len)
{
    uint32_t i;

    for (i = 0u; i < CAP_SLOTS; i++)
    {
        CapSlot *cs = &s_cap[i];
        uint32_t st1 = cs->stamp;

        if ((cs->seq != seq) || (cs->len != len))
        {
            continue;
        }
        LAT_FENCE_RW();
        lat_bytes_copy(s_cap_copy, cs->data, len);
        LAT_FENCE_RW();
        if (cs->stamp == st1)
        {
            return s_cap_copy;
        }
    }
    return (const uint8_t *)0;
}

/* ------------------------------------------------------------------ */
/* Trial                                                              */
/* ------------------------------------------------------------------ */

uint8_t measure_trial_start(const uint8_t *p, const uint8_t *watch, const uint8_t *calib)
{
    MeasTrialCfg cfg;
    MeasWatch w;
    MeasCalib cal;
    MeasDev dev;
    uint8_t iface;
    uint8_t ep;

    if (s.run != LAT_PC_RUN_NONE)
    {
        return LAT_PC_ST_BUSY;
    }
    meas_trial_cfg_parse(p, &cfg);
    if (!meas_watch_parse(watch, MEAS_WATCH_LEN, &w))
    {
        return LAT_PC_ST_NOT_READY;
    }
    iface = w.iface;
    ep = w.ep_in;
    if (!meas_port_dev_find(&iface, &ep, &dev))
    {
        return LAT_PC_ST_NO_DEVICE;
    }
    w.iface = iface;
    w.ep_in = ep;
    meas_calib_parse(calib, &cal);

    s.watch = w;
    s.run_iface = iface;
    s.run_ep = ep;
    s.run_id = cfg.run_id;
    s.dev = dev;
    s.cap_last_state = MEAS_ST_NONE;
    s_rec_tail = s_rec_head;
    s.rec_drop_seen = s.rec_drop;
    if (!meas_port_listen(iface, ep, true))
    {
        return LAT_PC_ST_NO_DEVICE;
    }
    meas_trial_start(&s_trial, &cfg, &w, &cal, &dev, meas_port_now());
    LAT_FENCE_RW();
    s.run = LAT_PC_RUN_TRIAL;
    return LAT_PC_ST_OK;
}

uint8_t measure_trial_stop(uint32_t run_id)
{
    if ((s.run != LAT_PC_RUN_TRIAL) || (run_id != s.run_id))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    meas_trial_abort(&s_trial, MEAS_DONE_ABORTED, meas_port_now());
    return LAT_PC_ST_OK;
}

static void trial_service(uint64_t now)
{
    const uint8_t *body;
    uint32_t len;

    if (s.run != LAT_PC_RUN_TRIAL)
    {
        return;
    }

    /* Record ring dropped entries; abort with internal error. */
    if (s.rec_drop != s.rec_drop_seen)
    {
        s.rec_drop_seen = s.rec_drop;
        meas_trial_abort(&s_trial, MEAS_DONE_INTERNAL, now);
    }

    for (;;)
    {
        uint32_t tail = s_rec_tail;
        MeasRecord r;
        const uint8_t *data = (const uint8_t *)0;

        if (tail == s_rec_head)
        {
            break;
        }
        LAT_FENCE_RW();
        r = s_rec[tail & (REC_RING - 1u)];
        LAT_FENCE_RW();
        s_rec_tail = tail + 1u;
        if (r.state == MEAS_ST_PRESSED)
        {
            data = capture_find(r.seq, r.len);
        }
        meas_trial_on_record(&s_trial, &r, data, now);
    }
    meas_trial_poll(&s_trial, now);

    if (!meas_port_dev_alive(&s.dev))
    {
        meas_trial_abort(&s_trial, MEAS_DONE_DETACHED, now);
    }

    while (meas_trial_result(&s_trial, &body, &len))
    {
        if (!v5f_cmd_emit_event(LAT_PC_EVT_TRIAL_RESULT, body, len))
        {
            return; /* Non-droppable ring full; wait before advancing trial */
        }
        meas_trial_result_sent(&s_trial, now);
    }
    if (meas_trial_done(&s_trial, &body))
    {
        if (v5f_cmd_finish_run(LAT_PC_EVT_RUN_DONE, body, MEAS_RUN_DONE_LEN))
        {
            meas_trial_done_sent(&s_trial);
            run_end();
        }
    }
}

bool measure_trial_running(void)
{
    return s.run == LAT_PC_RUN_TRIAL;
}

/* ------------------------------------------------------------------ */
/* Polling measurement                                                */
/* ------------------------------------------------------------------ */

uint8_t measure_poll_start(const uint8_t *p)
{
    uint8_t iface = p[4];
    uint8_t ep = p[5];
    uint32_t words;
    uint32_t *pool;
    MeasDev dev;
    uint64_t now;

    if (s.run != LAT_PC_RUN_NONE)
    {
        return LAT_PC_ST_BUSY;
    }
    if (!meas_port_dev_find(&iface, &ep, &dev))
    {
        return LAT_PC_ST_NO_DEVICE;
    }
    pool = meas_port_poll_pool(&words);
    s.poll_done_valid = 0u; /* Previous poll result will be overwritten */
    meas_poll_begin(&s_poll, dev.period_ticks, p[6] != 0u, lat_get_u32(&p[12]), pool, words,
                    meas_port_poll_longs());
    s.run_iface = iface;
    s.run_ep = ep;
    s.run_id = lat_get_u32(&p[0]);
    s.dev = dev;
    s.poll_finishing = 0u;
    if (!meas_port_listen(iface, ep, true))
    {
        meas_poll_stop(&s_poll);
        return LAT_PC_ST_NO_DEVICE;
    }
    now = meas_port_now();
    s.poll_start = now;
    s.poll_end = now + ((uint64_t)lat_get_u32(&p[8]) * MEAS_TICKS_PER_MS);
    LAT_FENCE_RW();
    s.run = LAT_PC_RUN_POLL;
    return LAT_PC_ST_OK;
}

static void poll_finish(uint8_t reason, uint64_t now)
{
    uint64_t ms;

    if (s.poll_finishing != 0u)
    {
        return;
    }
    meas_poll_stop(&s_poll);
    LAT_FENCE_RW();
    ms = (now - s.poll_start) / MEAS_TICKS_PER_MS;
    meas_poll_result(&s_poll, s.run_id, reason, (ms > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)ms, s.poll_result);
    s.poll_finishing = 1u;
    s.poll_done_valid = 1u;
    s.poll_done_run_id = s.run_id;
}

uint8_t measure_poll_stop(uint32_t run_id)
{
    if ((s.run != LAT_PC_RUN_POLL) || (run_id != s.run_id))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    poll_finish(MEAS_DONE_ABORTED, meas_port_now());
    return LAT_PC_ST_OK;
}

static void poll_service(uint64_t now)
{
    if (s.run != LAT_PC_RUN_POLL)
    {
        return;
    }
    if (s.poll_finishing == 0u)
    {
        if (!meas_port_dev_alive(&s.dev))
        {
            poll_finish(MEAS_DONE_DETACHED, now);
        }
        else if ((int64_t)(now - s.poll_end) >= 0)
        {
            poll_finish(MEAS_DONE_NORMAL, now);
        }
    }
    if ((s.poll_finishing != 0u) &&
        v5f_cmd_finish_run(LAT_PC_EVT_POLL_RESULT, s.poll_result, MEAS_POLL_RESULT_LEN))
    {
        s.poll_finishing = 0u;
        run_end();
    }
}

bool measure_poll_running(void)
{
    return s.run == LAT_PC_RUN_POLL;
}

uint8_t measure_poll_object(uint8_t kind, uint8_t index, uint32_t source_id, uint8_t *dst, uint32_t cap,
                            uint32_t *len)
{
    uint32_t n;

    if (index != 0u)
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    if ((s.poll_done_valid == 0u) || (s.run == LAT_PC_RUN_POLL))
    {
        return LAT_PC_ST_NOT_READY;
    }
    if (source_id != s.poll_done_run_id)
    {
        return LAT_PC_ST_STALE_OBJECT;
    }
    n = meas_poll_export(&s_poll, kind, dst, cap);
    if (n == 0xFFFFFFFFu)
    {
        return LAT_PC_ST_INTERNAL;
    }
    *len = n;
    return LAT_PC_ST_OK;
}

/* ------------------------------------------------------------------ */
/* Calibration mode                                                   */
/* ------------------------------------------------------------------ */

static void calib_stop(void)
{
    if (s.cal_mode == 0u)
    {
        return;
    }
    if (s.cal_mode == 1u)
    {
        (void)meas_port_trig_cancel();
        meas_port_pins(CAL_T0_MASK, 0u);
    }
    else
    {
        s.cal2_armed = 0u;
        LAT_FENCE_RW();
        meas_port_calib_in_end();
        (void)meas_port_listen(s.cal_iface, s.cal_ep, false);
    }
    meas_port_calib_ref_low();
    meas_port_marker(false);
    meas_port_calib_pins(false);
    s.cal_mode = 0u;
    s.cal_phase = 0u;
}

uint8_t measure_calib_mode(const uint8_t *p, const uint8_t *watch)
{
    uint8_t mode = p[0];
    uint32_t period_ms = lat_get_u32(&p[4]);
    uint64_t now;

    if (mode == 0u)
    {
        calib_stop();
        return LAT_PC_ST_OK;
    }
    if ((period_ms < CAL_PERIOD_MIN_MS) || (period_ms > CAL_PERIOD_MAX_MS))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    calib_stop();
    now = meas_port_now();
    s.cal_period = period_ms * MEAS_TICKS_PER_MS;
    s.cal_n = 0u;
    s.cal_phase = 0u;
    s.cal_next = now + MEAS_TICKS_PER_MS;
    if (mode == 2u)
    {
        uint8_t iface = 0xFFu;
        uint8_t ep = 0u;
        MeasWatch w;

        if ((watch != (const uint8_t *)0) && meas_watch_parse(watch, MEAS_WATCH_LEN, &w))
        {
            iface = w.iface;
            ep = w.ep_in;
        }
        if (!meas_port_dev_find(&iface, &ep, &s.cal_dev))
        {
            return LAT_PC_ST_NO_DEVICE;
        }
        s.cal_iface = iface;
        s.cal_ep = ep;
        s.cal2_armed = 0u;
        s.cal2_done = 0u;
        if (!meas_port_listen(iface, ep, true))
        {
            return LAT_PC_ST_NO_DEVICE;
        }
        if (!meas_port_calib_in_begin(iface, ep))
        {
            (void)meas_port_listen(iface, ep, false);
            return LAT_PC_ST_NO_DEVICE;
        }
    }
    else
    {
        meas_port_pins(CAL_T0_MASK, 0u);
    }
    meas_port_calib_pins(true);
    s.cal_mode = mode;
    return LAT_PC_ST_OK;
}

static void calib1_service(uint64_t now)
{
    MeasTrigResult r;
    char buf[LOG_MAX + 1u];
    LatStr st;

    switch (s.cal_phase)
    {
    case 0u:
        if ((int64_t)(now - s.cal_next) >= 0)
        {
            /* Align reference edge (PA2, TIM9_CH3 compare) and T0 (ISR -> timestamp -> BCR) at C. */
            s.cal_c = now + CAL_LEAD_TICKS;
            meas_port_calib_ref_arm(s.cal_c);
            meas_port_trig_arm(s.cal_c, (uint8_t)(MEAS_TRIG_ASSERT | MEAS_TRIG_MARKER), CAL_T0_MASK);
            s.cal_phase = 1u;
        }
        break;
    case 1u:
        if (meas_port_trig_fired(&r))
        {
            lat_str_init(&st, buf, sizeof(buf));
            lat_str_puts(&st, "CAL1 n=");
            lat_str_u32(&st, s.cal_n);
            lat_str_puts(&st, " C=");
            put_hex64(&st, s.cal_c);
            lat_str_puts(&st, " T0ISR=");
            put_i64(&st, (int64_t)(r.t_isr - s.cal_c));
            lat_str_puts(&st, " T0TICK=");
            put_i64(&st, (int64_t)(r.t_tick - s.cal_c));
            lat_str_puts(&st, " TM=");
            put_i64(&st, (int64_t)(r.t_m - s.cal_c));
            meas_log(buf, st.len);
            s.cal_at = s.cal_c + (s.cal_period / 2u);
            s.cal_phase = 2u;
        }
        else if ((int64_t)(now - (s.cal_c + CAL_STUCK_TICKS)) > 0)
        {
            (void)meas_port_trig_cancel();
            meas_log("CAL1 trigger did not fire", 25u);
            s.cal_at = now;
            s.cal_phase = 2u;
        }
        break;
    default:
        if ((int64_t)(now - s.cal_at) >= 0)
        {
            meas_port_pins(CAL_T0_MASK, 0u);
            meas_port_calib_ref_low();
            meas_port_marker(false);
            s.cal_n++;
            s.cal_next = s.cal_c + s.cal_period;
            if ((int64_t)(now - s.cal_next) > 0)
            {
                s.cal_next = now;
            }
            s.cal_phase = 0u;
        }
        break;
    }
}

static void calib2_service(uint64_t now)
{
    char buf[LOG_MAX + 1u];
    LatStr st;

    if (!meas_port_dev_alive(&s.cal_dev))
    {
        meas_log("CAL2 device lost - calibration mode ended", 41u);
        calib_stop();
        return;
    }
    switch (s.cal_phase)
    {
    case 0u:
        if ((int64_t)(now - s.cal_next) >= 0)
        {
            s.cal2_done = 0u;
            LAT_FENCE_RW();
            s.cal2_armed = 1u;
            (void)meas_port_calib_in_issue();
            s.cal_c = now;
            s.cal_phase = 1u;
        }
        break;
    case 1u:
        if (s.cal2_done != 0u)
        {
            LAT_FENCE_RW();
            lat_str_init(&st, buf, sizeof(buf));
            lat_str_puts(&st, "CAL2 n=");
            lat_str_u32(&st, s.cal_n);
            lat_str_puts(&st, " T1=");
            put_hex64(&st, s.cal2_t1);
            lat_str_puts(&st, " TM=");
            put_i64(&st, (int64_t)(s.cal2_tm - s.cal2_t1));
            meas_log(buf, st.len);
            s.cal_at = now + (s.cal_period / 2u);
            s.cal_phase = 2u;
        }
        else if ((int64_t)(now - (s.cal_c + CAL_IN_WAIT_TICKS)) > 0)
        {
            s.cal2_armed = 0u;
            meas_log("CAL2 no IN data", 15u);
            s.cal_at = now;
            s.cal_phase = 2u;
        }
        break;
    default:
        if ((int64_t)(now - s.cal_at) >= 0)
        {
            meas_port_marker(false);
            s.cal_n++;
            s.cal_next = s.cal_c + s.cal_period;
            if ((int64_t)(now - s.cal_next) > 0)
            {
                s.cal_next = now;
            }
            s.cal_phase = 0u;
        }
        break;
    }
}

bool measure_calib_active(void)
{
    return s.cal_mode != 0u;
}

/* ------------------------------------------------------------------ */
/* Top-level                                                          */
/* ------------------------------------------------------------------ */

void measure_poll(uint64_t now)
{
    monitor_service();
    trial_service(now);
    poll_service(now);
    if (s.cal_mode == 1u)
    {
        calib1_service(now);
    }
    else if (s.cal_mode == 2u)
    {
        calib2_service(now);
    }
}

void measure_session_reset(void)
{
    calib_stop();
    measure_monitor_stop();
    if (s.run == LAT_PC_RUN_TRIAL)
    {
        meas_trial_kill(&s_trial);
    }
    else if (s.run == LAT_PC_RUN_POLL)
    {
        meas_poll_stop(&s_poll);
    }
    if (s.run != LAT_PC_RUN_NONE)
    {
        run_end();
    }
    s.poll_finishing = 0u;
    s.poll_done_valid = 0u;
    (void)meas_port_trig_cancel();
    meas_port_pins(MEAS_PIN_MASK, 0u);
}

void measure_overflow(void)
{
    uint64_t now = meas_port_now();

    if (s.run == LAT_PC_RUN_TRIAL)
    {
        meas_trial_abort(&s_trial, MEAS_DONE_OVERFLOW, now);
    }
    else if (s.run == LAT_PC_RUN_POLL)
    {
        poll_finish(MEAS_DONE_OVERFLOW, now);
    }
}

void measure_quiesce(void)
{
    calib_stop();
    measure_monitor_stop();
}

uint8_t measure_run_kind(void)
{
    if (s.run != LAT_PC_RUN_NONE)
    {
        return s.run;
    }
    return (s.cal_mode != 0u) ? LAT_PC_RUN_CALIBRATION : LAT_PC_RUN_NONE;
}

uint32_t measure_active_run_id(void)
{
    return (s.run != LAT_PC_RUN_NONE) ? s.run_id : 0u;
}

uint8_t measure_trigger_asserted(void)
{
    uint8_t m = 0u;

    if (s.run == LAT_PC_RUN_TRIAL)
    {
        m = meas_trial_asserted(&s_trial);
    }
    return m;
}

void measure_init(void)
{
    lat_bytes_fill((uint8_t *)&s, 0u, (uint32_t)sizeof(s));
    meas_trial_init(&s_trial);
    lat_bytes_fill((uint8_t *)&s_poll, 0u, (uint32_t)sizeof(s_poll));
    s_rec_head = 0u;
    s_rec_tail = 0u;
    s_mon_head = 0u;
    s_mon_tail = 0u;
    s_cap_head = 0u;
}
