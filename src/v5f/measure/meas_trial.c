/*
 * meas_trial.c - Trial state machine (see meas_trial.h)
 */
#include "meas_trial.h"

#include "lat_bytes.h"

#define TP_IDLE          ((uint8_t)0u)
#define TP_SETTLE        ((uint8_t)1u)
#define TP_TRIG_WAIT     ((uint8_t)2u)
#define TP_DETECT        ((uint8_t)3u)
#define TP_CONFIRM       ((uint8_t)4u)
#define TP_REL_TRIG_WAIT ((uint8_t)5u)
#define TP_REL_DETECT    ((uint8_t)6u)
#define TP_REL_CONFIRM   ((uint8_t)7u)
#define TP_EMIT          ((uint8_t)8u)
#define TP_INTER         ((uint8_t)9u)
#define TP_DONE          ((uint8_t)10u)

/* Internal error if an armed trigger does not fire within 50ms. */
#define TRIG_STUCK_TICKS ((uint64_t)50u * MEAS_TICKS_PER_MS)

/* ------------------------------------------------------------------ */
/* Parsing and rules                                                  */
/* ------------------------------------------------------------------ */

void meas_trial_cfg_parse(const uint8_t *p, MeasTrialCfg *cfg)
{
    lat_bytes_fill((uint8_t *)cfg, 0u, (uint32_t)sizeof(*cfg));
    cfg->run_id = lat_get_u32(&p[0]);
    cfg->trigger_mask = p[4];
    cfg->measure_release = p[5];
    cfg->settle_reports = p[6];
    cfg->count = lat_get_u16(&p[8]);
    cfg->timeout_ms = lat_get_u16(&p[10]);
    cfg->release_timeout_ms = lat_get_u16(&p[12]);
    cfg->inter_trial_ms = lat_get_u16(&p[14]);
    cfg->random_max_us = lat_get_u32(&p[16]);
    cfg->seed = lat_get_u32(&p[20]);
}

void meas_calib_parse(const uint8_t *p, MeasCalib *cal)
{
    lat_bytes_fill((uint8_t *)cal, 0u, (uint32_t)sizeof(*cal));
    cal->c0 = lat_get_i32(&p[0]);
    cal->c_m = lat_get_i32(&p[4]);
    cal->c1_fs = lat_get_i32(&p[8]);
    cal->c0_unc = lat_get_u32(&p[12]);
    cal->c_m_unc = lat_get_u32(&p[16]);
    cal->c1_unc = lat_get_u32(&p[20]);
    cal->valid = (uint8_t)(p[24] & 0x07u);
}

uint8_t meas_c1_status(const MeasCalib *cal, uint8_t speed)
{
    if ((cal->valid & MEAS_CAL_C1_VALID) == 0u)
    {
        return MEAS_C1_UNKNOWN; /* Before calibration */
    }
    if (speed == MEAS_SPEED_FS)
    {
        return MEAS_C1_MEASURED_FS;
    }
    if (speed == MEAS_SPEED_HS)
    {
        return MEAS_C1_ESTIMATED_FS;
    }
    return MEAS_C1_UNKNOWN; /* Low Speed */
}

bool meas_latency_valid(uint8_t fail_kind, uint64_t t1_irq, const MeasCalib *cal, uint8_t c1_status,
                        uint8_t measurement_kind)
{
    return (fail_kind == MEAS_FAIL_NONE) && (t1_irq != 0u) && ((cal->valid & MEAS_CAL_C0_VALID) != 0u) &&
           (c1_status != MEAS_C1_UNKNOWN) && (measurement_kind == 0u);
}

/* ------------------------------------------------------------------ */
/* Random                                                             */
/* ------------------------------------------------------------------ */

static uint32_t rnd32(MeasTrial *t)
{
    uint32_t x;

    /* xorshift32 (state is never 0). */
    x = t->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    t->rng = x;
    return x;
}

/* Uniform in [0, random_max_ticks]. */
static uint32_t rnd_delay(MeasTrial *t)
{
    return (uint32_t)(((uint64_t)rnd32(t) * ((uint64_t)t->random_max_ticks + 1u)) >> 32);
}

/* ------------------------------------------------------------------ */
/* Results                                                            */
/* ------------------------------------------------------------------ */

static void make_done(MeasTrial *t, uint8_t reason)
{
    uint8_t *b = t->done_body;
    uint32_t i;

    lat_bytes_fill(b, 0u, MEAS_RUN_DONE_LEN);
    lat_put_u32(&b[0], t->cfg.run_id);
    lat_put_u16(&b[4], t->cfg.count);
    lat_put_u16(&b[6], t->completed);
    lat_put_u16(&b[8], t->succeeded);
    lat_put_u16(&b[10], t->valid_count);
    for (i = 0u; i < MEAS_FAIL_KINDS; i++)
    {
        lat_put_u16(&b[12u + (2u * i)], t->fail_counts[i]);
    }
    b[26] = reason;
    t->done_reason = reason;
    t->done_pending = 1u;
    t->in_trial = 0u;
    t->phase = TP_DONE;
}

static void build_result(MeasTrial *t)
{
    uint8_t *b = t->result;
    uint16_t t0_frame = 0u;
    uint8_t t0_uframe = 0u;
    uint8_t valid = 0u;
    bool c0_ok = (t->cal.valid & MEAS_CAL_C0_VALID) != 0u;

    if (t->t0_tick != 0u)
    {
        (void)meas_port_frame_at(t->t0_tick, &t0_frame, &t0_uframe);
    }
    if (meas_latency_valid(t->fail_kind, t->t1, &t->cal, t->c1_status, 0u))
    {
        valid |= 0x01u;
    }
    if ((t->cfg.measure_release != 0u) &&
        meas_latency_valid(t->rel_fail_kind, t->rel_t1, &t->cal, t->c1_status, 0u))
    {
        valid |= 0x02u;
    }

    lat_bytes_fill(b, 0u, MEAS_TRIAL_RESULT_HDR);
    lat_put_u32(&b[0], t->cfg.run_id);
    lat_put_u16(&b[4], t->index);
    b[6] = t->fail_kind;
    b[7] = (t->cfg.measure_release != 0u) ? t->rel_fail_kind : 0u;
    b[8] = t->speed;
    b[9] = t->c1_status;
    b[10] = 0u; /* measurement_kind: estimated t1_wire - t0_wire */
    b[11] = valid;
    lat_put_u32(&b[12], t->delay);
    lat_put_u64(&b[16], t->t0_sched);
    lat_put_u64(&b[24], t->t0_isr);
    lat_put_u64(&b[32], t->t0_tick);
    lat_put_u64(&b[40], t->t1);
    lat_put_u64(&b[48], t->confirm_t1);
    lat_put_i32(&b[56], c0_ok ? t->cal.c0 : 0);
    lat_put_i32(&b[60], (t->c1_status != MEAS_C1_UNKNOWN) ? t->cal.c1_fs : 0);
    lat_put_u32(&b[64], (t->c1_status == MEAS_C1_MEASURED_FS) ? t->cal.c1_unc : 0xFFFFFFFFu);
    lat_put_u16(&b[68], t0_frame);
    b[70] = t0_uframe;
    lat_put_u16(&b[72], (t->t1 != 0u) ? t->t1_frame : 0u);
    b[74] = (t->t1 != 0u) ? t->t1_uframe : 0u;
    lat_put_u64(&b[76], t->rel_t0_tick);
    lat_put_u64(&b[84], t->rel_t1);
    lat_put_u16(&b[92], t->report_len);
    lat_bytes_copy(&b[MEAS_TRIAL_RESULT_HDR], t->report, t->report_len);
    t->result_len = MEAS_TRIAL_RESULT_HDR + (uint32_t)t->report_len;

    t->completed++;
    t->fail_counts[(t->fail_kind < MEAS_FAIL_KINDS) ? t->fail_kind : MEAS_FAIL_ABORTED]++;
    if (t->fail_kind == MEAS_FAIL_NONE)
    {
        t->succeeded++;
    }
    if ((valid & 0x01u) != 0u)
    {
        t->valid_count++;
    }
    t->index++;
    t->result_pending = 1u;
    t->in_trial = 0u;
    t->phase = TP_EMIT;
}

/* ------------------------------------------------------------------ */
/* Transitions                                                        */
/* ------------------------------------------------------------------ */

static void release_now(MeasTrial *t)
{
    meas_port_pins(t->cfg.trigger_mask, 0u);
    t->pins = 0u;
}

static void begin_trial(MeasTrial *t, uint64_t now)
{
    release_now(t); /* 1. Release trigger */
    t->in_trial = 1u;
    t->fail_kind = MEAS_FAIL_NONE;
    t->rel_fail_kind = MEAS_FAIL_NONE;
    t->dirty = 0u;
    t->settle_count = 0u;
    t->settle_start = now;
    t->settle_after = now;
    t->delay = 0u;
    t->t0_sched = 0u;
    t->t0_isr = 0u;
    t->t0_tick = 0u;
    t->t1 = 0u;
    t->confirm_t1 = 0u;
    t->rel_t0_tick = 0u;
    t->rel_t1 = 0u;
    t->t1_frame = 0u;
    t->t1_uframe = 0u;
    t->report_len = 0u;
    t->phase = TP_SETTLE;
}

static void arm_press(MeasTrial *t, uint64_t now)
{
    t->delay = rnd_delay(t);
    t->t0_sched = now + MEAS_TRIAL_LEAD + t->delay;
    meas_port_trig_arm(t->t0_sched, MEAS_TRIG_ASSERT, t->cfg.trigger_mask);
    t->phase = TP_TRIG_WAIT;
}

/* Press phase finished. */
static void finish_press(MeasTrial *t, uint8_t fail, uint64_t now)
{
    t->fail_kind = fail;
    if ((fail == MEAS_FAIL_NONE) && (t->cfg.measure_release != 0u))
    {
        uint64_t when = now + MEAS_TRIAL_LEAD + rnd_delay(t);

        meas_port_trig_arm(when, MEAS_TRIG_RELEASE, t->cfg.trigger_mask);
        t->deadline = when;
        t->phase = TP_REL_TRIG_WAIT;
        return;
    }
    release_now(t);
    t->rel_fail_kind = (t->cfg.measure_release != 0u) ? fail : MEAS_FAIL_NONE;
    build_result(t);
}

static void finish_release(MeasTrial *t, uint8_t fail)
{
    release_now(t); /* Ensure released */
    t->rel_fail_kind = fail;
    build_result(t);
}

/* Capture TIM9_CH1 trigger result if it has fired. */
static void check_fired(MeasTrial *t)
{
    MeasTrigResult r;

    if (((t->phase != TP_TRIG_WAIT) && (t->phase != TP_REL_TRIG_WAIT)) || !meas_port_trig_fired(&r))
    {
        return;
    }
    if (t->phase == TP_TRIG_WAIT)
    {
        t->t0_sched = r.sched;
        t->t0_isr = r.t_isr;
        t->t0_tick = r.t_tick;
        t->pins = t->cfg.trigger_mask;
        t->phase = TP_DETECT;
    }
    else
    {
        t->rel_t0_tick = r.t_tick;
        t->pins = 0u;
        t->phase = TP_REL_DETECT;
    }
}

void meas_trial_init(MeasTrial *t)
{
    lat_bytes_fill((uint8_t *)t, 0u, (uint32_t)sizeof(*t));
}

void meas_trial_start(MeasTrial *t, const MeasTrialCfg *cfg, const MeasWatch *w, const MeasCalib *cal,
                      const MeasDev *dev, uint64_t now)
{
    uint64_t max_ticks;

    meas_trial_init(t);
    t->cfg = *cfg;
    t->watch = *w;
    t->cal = *cal;
    t->dev = *dev;
    t->speed = dev->speed;
    t->c1_status = meas_c1_status(cal, dev->speed);
    /*
     * Seed 0: seed the PRNG once from the hardware RNG. Drawing every delay
     * from the hardware RNG skews the trigger phase against the polling.
     */
    t->rng = (cfg->seed != 0u) ? cfg->seed : meas_port_hw_random();
    if (t->rng == 0u)
    {
        t->rng = 0x9E3779B9u;
    }
    if (cfg->random_max_us == 0u)
    {
        /* max(polling_period, 1ms) */
        max_ticks = (dev->period_ticks > MEAS_TICKS_PER_MS) ? dev->period_ticks : MEAS_TICKS_PER_MS;
    }
    else
    {
        max_ticks = (uint64_t)cfg->random_max_us * MEAS_TICKS_PER_US;
    }
    t->random_max_ticks = (uint32_t)max_ticks;
    t->active = 1u;
    begin_trial(t, now);
}

void meas_trial_on_record(MeasTrial *t, const MeasRecord *r, const uint8_t *data, uint64_t now)
{
    uint64_t timeout = (uint64_t)t->cfg.timeout_ms * MEAS_TICKS_PER_MS;

    if (!t->active)
    {
        return;
    }
    check_fired(t);

    switch (t->phase)
    {
    case TP_SETTLE:
        if ((r->t1 <= t->settle_after) || (r->state == MEAS_ST_NONE))
        {
            return;
        }
        if (r->state == MEAS_ST_RELEASED)
        {
            t->settle_count++;
            if (t->settle_count >= t->cfg.settle_reports)
            {
                arm_press(t, now);
            }
        }
        else
        {
            t->settle_count = 0u;
        }
        return;

    case TP_TRIG_WAIT:
        /* Not fired yet; cancel and re-settle if released state broke. */
        if ((r->t1 <= t->settle_after) || (r->state == MEAS_ST_NONE) || (r->state == MEAS_ST_RELEASED))
        {
            return;
        }
        if (meas_port_trig_cancel())
        {
            t->settle_count = 0u;
            t->phase = TP_SETTLE;
            return;
        }
        check_fired(t); /* Fired before cancel */
        if (t->phase != TP_DETECT)
        {
            return;
        }
        /* Continue as DETECT */
        /* fall through */
    case TP_DETECT:
        if (r->t1 <= t->t0_tick)
        {
            if ((r->state == MEAS_ST_PRESSED) || (r->state == MEAS_ST_OTHER))
            {
                t->dirty = 1u; /* Released state broke before trigger */
            }
            return;
        }
        if ((r->t1 - t->t0_tick) > timeout)
        {
            finish_press(t, MEAS_FAIL_TIMEOUT, now);
            return;
        }
        if (r->state != MEAS_ST_PRESSED)
        {
            return;
        }
        t->t1 = r->t1;
        t->t1_frame = r->frame;
        t->t1_uframe = r->uframe;
        if ((data != (const uint8_t *)0) && (r->len <= MEAS_REPORT_MAX))
        {
            lat_bytes_copy(t->report, data, r->len);
            t->report_len = r->len;
        }
        if (t->dirty != 0u)
        {
            finish_press(t, MEAS_FAIL_NOT_RELEASED, now);
            return;
        }
        t->deadline = r->t1 + timeout;
        t->phase = TP_CONFIRM;
        return;

    case TP_CONFIRM:
        if ((r->t1 <= t->t1) || (r->state == MEAS_ST_NONE))
        {
            return;
        }
        t->confirm_t1 = r->t1;
        finish_press(t, (r->state == MEAS_ST_PRESSED) ? MEAS_FAIL_NONE : MEAS_FAIL_CONFIRM, now);
        return;

    case TP_REL_DETECT:
        if (r->t1 <= t->rel_t0_tick)
        {
            return;
        }
        if ((r->t1 - t->rel_t0_tick) > timeout)
        {
            finish_release(t, MEAS_FAIL_TIMEOUT);
            return;
        }
        if (r->state != MEAS_ST_RELEASED)
        {
            return;
        }
        t->rel_t1 = r->t1;
        t->phase = TP_REL_CONFIRM;
        return;

    case TP_REL_CONFIRM:
        if ((r->t1 <= t->rel_t1) || (r->state == MEAS_ST_NONE))
        {
            return;
        }
        finish_release(t, (r->state == MEAS_ST_RELEASED) ? MEAS_FAIL_NONE : MEAS_FAIL_CONFIRM);
        return;

    default:
        return;
    }
}

void meas_trial_poll(MeasTrial *t, uint64_t now)
{
    uint64_t timeout = (uint64_t)t->cfg.timeout_ms * MEAS_TICKS_PER_MS;

    if (!t->active)
    {
        return;
    }
    check_fired(t);

    switch (t->phase)
    {
    case TP_SETTLE:
        if ((int64_t)(now - t->settle_start) > (int64_t)((uint64_t)t->cfg.release_timeout_ms * MEAS_TICKS_PER_MS))
        {
            finish_press(t, MEAS_FAIL_NOT_RELEASED, now);
        }
        break;
    case TP_TRIG_WAIT:
        if ((int64_t)(now - (t->t0_sched + TRIG_STUCK_TICKS)) > 0)
        {
            meas_trial_abort(t, MEAS_DONE_INTERNAL, now); /* Trigger failed to fire */
        }
        break;
    case TP_REL_TRIG_WAIT:
        if ((int64_t)(now - (t->deadline + TRIG_STUCK_TICKS)) > 0)
        {
            meas_trial_abort(t, MEAS_DONE_INTERNAL, now);
        }
        break;
    case TP_DETECT:
        /* `now` is read at top of main loop and may precede `t0_tick`; compare signed. */
        if ((int64_t)(now - t->t0_tick) > (int64_t)(timeout + MEAS_TRIAL_MARGIN))
        {
            finish_press(t, MEAS_FAIL_TIMEOUT, now);
        }
        break;
    case TP_CONFIRM:
        if ((int64_t)(now - (t->deadline + MEAS_TRIAL_MARGIN)) > 0)
        {
            finish_press(t, MEAS_FAIL_TIMEOUT, now);
        }
        break;
    case TP_REL_DETECT:
        if ((int64_t)(now - t->rel_t0_tick) > (int64_t)(timeout + MEAS_TRIAL_MARGIN))
        {
            finish_release(t, MEAS_FAIL_TIMEOUT);
        }
        break;
    case TP_REL_CONFIRM:
        if ((int64_t)(now - t->rel_t1) > (int64_t)(timeout + MEAS_TRIAL_MARGIN))
        {
            finish_release(t, MEAS_FAIL_TIMEOUT);
        }
        break;
    case TP_INTER:
        if ((int64_t)(now - t->deadline) >= 0)
        {
            begin_trial(t, now);
        }
        break;
    default:
        break;
    }
}

void meas_trial_abort(MeasTrial *t, uint8_t reason, uint64_t now)
{
    uint8_t kind;

    (void)now;
    if (!t->active || (t->phase == TP_DONE))
    {
        return;
    }
    (void)meas_port_trig_cancel();
    release_now(t);
    switch (reason)
    {
    case MEAS_DONE_DETACHED:
        kind = MEAS_FAIL_DETACHED;
        break;
    case MEAS_DONE_OVERFLOW:
        kind = MEAS_FAIL_OVERFLOW;
        break;
    default:
        kind = MEAS_FAIL_ABORTED;
        break;
    }
    if (t->in_trial != 0u)
    {
        if (t->phase >= TP_REL_TRIG_WAIT)
        {
            t->rel_fail_kind = kind; /* Press phase succeeded */
        }
        else
        {
            t->fail_kind = kind;
            t->rel_fail_kind = (t->cfg.measure_release != 0u) ? kind : MEAS_FAIL_NONE;
        }
        build_result(t);
    }
    if (reason == MEAS_DONE_OVERFLOW)
    {
        t->result_pending = 0u; /* Cannot send on overflow; count only and drop */
    }
    make_done(t, reason);
}

void meas_trial_kill(MeasTrial *t)
{
    if (t->active)
    {
        (void)meas_port_trig_cancel();
        release_now(t);
    }
    t->active = 0u;
    t->phase = TP_IDLE;
    t->in_trial = 0u;
    t->result_pending = 0u;
    t->done_pending = 0u;
}

bool meas_trial_result(const MeasTrial *t, const uint8_t **body, uint32_t *len)
{
    if (!t->active || (t->result_pending == 0u))
    {
        return false;
    }
    *body = t->result;
    *len = t->result_len;
    return true;
}

void meas_trial_result_sent(MeasTrial *t, uint64_t now)
{
    t->result_pending = 0u;
    if (t->phase != TP_EMIT)
    {
        return; /* Already aborted (RUN_DONE built) */
    }
    if (t->index >= t->cfg.count)
    {
        make_done(t, MEAS_DONE_NORMAL);
        return;
    }
    t->deadline = now + ((uint64_t)t->cfg.inter_trial_ms * MEAS_TICKS_PER_MS);
    t->phase = TP_INTER;
}

bool meas_trial_done(const MeasTrial *t, const uint8_t **body)
{
    if (!t->active || (t->done_pending == 0u) || (t->result_pending != 0u))
    {
        return false;
    }
    *body = t->done_body;
    return true;
}

void meas_trial_done_sent(MeasTrial *t)
{
    t->done_pending = 0u;
    t->active = 0u;
    t->phase = TP_IDLE;
}

bool meas_trial_running(const MeasTrial *t)
{
    return t->active != 0u;
}

uint8_t meas_trial_asserted(const MeasTrial *t)
{
    return t->active ? t->pins : 0u;
}
