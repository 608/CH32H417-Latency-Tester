/*
 * usbhs_host_core.c - Hardware-independent USBHS host engine core
 *
 * Context ownership:
 *   - USBHS ISR : usbhs_core_isr(); owns scheduler, pipes, port state.
 *   - TIM9 ISR  : usbhs_core_tim9_ch2(); accesses only s_busy, s_pub,
 *                 s_slot (read-only), s_inf, and hardware issue.
 *   - Main loop : APIs; mask USBHS IRQ via usbhs_hw_lock() before mutating
 *                 pipes (TIM9 is never masked).
 */

#include "usbhs_host_core.h"

#include "lat_bytes.h"
#include "lat_fence.h"
#include "lat_pc_proto.h"

#define PIPE_CTRL          (0u)
#define PRIO_OUT           (0u)   /* Lower value = higher priority */
#define PRIO_IN            (1u)
#define PRIO_CTRL          (2u)
#define SRC_USBHS          (0u)
#define SRC_TIM9           (1u)

#define LATE_TOLERANCE     (2u * USBHS_TICKS_PER_US)   /* Threshold for sched_late_count */
#define SOF_LATE_MAX       (5u * USBHS_TICKS_PER_US)   /* Max allowed SOF IRQ jitter */
#define SOF_LATE_RUN       (3u)                        /* Consecutive late SOFs before reanchor */
#define SOF_CHECK_TICKS    ((uint64_t)10u * USBHS_TICKS_PER_MS)
#define RESET_LIMIT_TICKS  ((uint64_t)200u * USBHS_TICKS_PER_MS)
#define CTRL_TIMEOUT_DEF   (5000u * USBHS_TICKS_PER_MS)
#define SCHED_SPAN_MAX     ((uint64_t)20000u * USBHS_TICKS_PER_MS)

#define REPORT_RING        (16u)
#define EVENT_RING         (32u)
#define TRACE_RING         (64u)
#define GENERATION_RING    (64u)
#define RECORD_BYTES       (32768u)
#define RECORD_FIXED       LAT_PC_ENUM_RECORD_FIXED_LEN
#define PREV_TRACE_BYTES   LAT_PC_ENUM_RECORD_PREV_MAX

#define INTR_CLOSED        (0u)
#define INTR_RUN           (1u)
#define INTR_PAUSED        (2u)
#define INTR_HALTED        (3u)

_Static_assert((REPORT_RING & (REPORT_RING - 1u)) == 0u, "REPORT_RING must be a power of two");
_Static_assert((EVENT_RING & (EVENT_RING - 1u)) == 0u, "EVENT_RING must be a power of two");
_Static_assert((TRACE_RING & (TRACE_RING - 1u)) == 0u, "TRACE_RING must be a power of two");
_Static_assert((GENERATION_RING & (GENERATION_RING - 1u)) == 0u, "GENERATION_RING must be a power of two");
_Static_assert((USBHS_OUT_QUEUE & (USBHS_OUT_QUEUE - 1u)) == 0u, "USBHS_OUT_QUEUE must be a power of two");
_Static_assert(RECORD_FIXED == 53u, "enum trace v2 fixed record length");
_Static_assert(PREV_TRACE_BYTES == 8192u, "previous trace archive size");

/* Published schedule slot. */
typedef struct Item
{
    uint32_t sched;        /* Scheduled time (low 32 bits) */
    uint32_t gen;          /* Publication generation */
    const uint8_t *tx;     /* TX payload (copied to DMA at issue) */
    uint16_t len;          /* TX length, or RX max length for IN */
    uint8_t pipe;
    uint8_t epoch;         /* Prevents mismatch if pipe is reopened */
    uint8_t token;         /* USBHS_PID_SETUP / IN / OUT */
    uint8_t ep;
    uint8_t addr;
    uint8_t toggle;
    uint8_t count_late;    /* 1: count lateness (0 for immediate issues) */
    uint8_t reserved[3];
} Item;

/* In-flight transaction; written only by the issuer holding s_busy. */
typedef struct Inflight
{
    Item it;
    uint32_t issue_cnt;
    uint8_t valid;
    uint8_t src;
} Inflight;

typedef struct NakRun
{
    uint8_t open;
    uint8_t addr;
    uint8_t ep;
    uint8_t token;
    uint8_t toggle;
    uint8_t pipe;
    uint16_t step;
    uint32_t count;
    int32_t late;
    uint64_t first;
    uint64_t last;
} NakRun;

/* SOF timestamp to frame mapping; (seq & mask) == hardware FRAME number. */
typedef struct FrameModel
{
    uint8_t valid;
    uint8_t have_raw;
    uint8_t late_run;
    uint8_t reserved;
    uint32_t period;
    uint32_t mask;
    uint32_t last_raw;
    uint32_t seq;
    uint32_t anchor_time;
    uint32_t anchor_seq;
} FrameModel;

typedef struct Ctrl
{
    uint8_t active;
    uint8_t done;
    uint8_t pend;
    uint8_t inflight;
    uint8_t frame_rel;
    uint8_t count_late;
    uint8_t stage;
    uint8_t dir_in;
    uint8_t toggle;
    uint8_t first_pkt;
    uint8_t errors;
    uint8_t epoch;
    uint16_t wlength;
    uint16_t pkt_len;      /* Current DATA OUT packet length */
    uint16_t actual;       /* Bytes transferred in DATA stage */
    uint16_t reserved;
    uint32_t t_abs;        /* Scheduled time (fallback when no SOF anchor) */
    uint32_t f_seq;
    uint32_t f_off;
    uint64_t setup_target;
    uint64_t deadline;
    UsbhsCtrlReq req;
    UsbhsCtrlResult res;
    NakRun nak;
} Ctrl;

typedef struct OutPkt
{
    uint16_t len;
    uint8_t data[USBHS_OUT_MAX];
} OutPkt;

typedef struct Intr
{
    uint8_t used;
    uint8_t state;
    uint8_t dir_in;
    uint8_t toggle;
    uint8_t inflight;
    uint8_t epoch;
    uint8_t addr;
    uint8_t ep;
    uint8_t trace_budget;
    uint8_t err_immediate;
    uint8_t err_streak;
    uint8_t has_nb;        /* Use not_before for next schedule */
    uint8_t seq_init;
    uint8_t stall_pending; /* IN STALL: waiting for CLEAR_FEATURE */
    uint8_t reserved0;
    uint16_t mps;
    uint16_t step;
    uint32_t period;
    uint32_t next_seq;
    uint32_t slot;
    uint32_t err_pause;
    uint64_t not_before;
    uint32_t generation;
    UsbhsIntrHook hook;
    UsbhsIntrStats st;
    OutPkt q[USBHS_OUT_QUEUE];
    uint32_t q_head;       /* Written by main loop */
    uint32_t q_tail;       /* Written by ISR */
    NakRun nak;
} Intr;

typedef struct Core
{
    uint8_t port_state;
    uint8_t speed;
    uint8_t configured;
    uint8_t recovering;
    uint8_t epoch_counter;
    uint8_t pending_pipe;
    uint8_t pending_epoch;
    uint8_t last_slot;
    uint32_t generation;
    uint32_t trace_drop_base;
    uint32_t pub_gen;
    uint32_t pending_gen;  /* Published generation not yet confirmed as issued */
    uint64_t attach_time;
    uint64_t reset_start;
    uint64_t reset_done;
    uint64_t enabled_time;
    uint32_t reset_ticks;
    uint32_t sof_checked;
    uint64_t sof_next_check;
    uint32_t report_seq;
    FrameModel fm;
    Ctrl ctrl;
    Intr intr[USBHS_INTR_MAX];
    UsbhsStats st;
} Core;

typedef struct RepSlot
{
    UsbhsIntrDone d;
    uint8_t data[USBHS_PKT_MAX];
} RepSlot;

typedef struct GenerationBoundary
{
    uint32_t trace_pos;
    uint32_t generation;
    uint32_t drop_base;
} GenerationBoundary;

static Core s;
/* Shared between TIM9 ISR and USBHS ISR. */
static volatile uint32_t s_busy;   /* 1: transaction in flight */
static volatile uint32_t s_pub;    /* 0: none, else: (gen << 1) | slot_idx */
static Item s_slot[2];
static Inflight s_inf;
static volatile uint32_t s_sof_count;

/* SPSC queues from ISR (producer) to main loop (consumer). */
static RepSlot s_rep[REPORT_RING];
static volatile uint32_t s_rep_head;
static volatile uint32_t s_rep_tail;
static UsbhsEvent s_ev[EVENT_RING];
static volatile uint32_t s_ev_head;
static volatile uint32_t s_ev_tail;
static UsbhsTrace s_tr[TRACE_RING];
static volatile uint32_t s_tr_head;
static volatile uint32_t s_tr_tail;
static GenerationBoundary s_gen_boundary[GENERATION_RING];
static volatile uint32_t s_gen_head;
static volatile uint32_t s_gen_tail;

/* Enumeration record (main loop only). */
static uint8_t s_rec[RECORD_BYTES];
static uint32_t s_rec_len;
static uint16_t s_rec_count;
static uint32_t s_rec_dropped;
static uint32_t s_rec_generation;
static uint32_t s_rec_trace_drop_base;
static uint8_t s_prev_rec[PREV_TRACE_BYTES]
    __attribute__((section(".shm_spare"), aligned(64), used));
static uint32_t s_prev_rec_len;

static void core_recover(uint8_t cause, uint64_t now);

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static inline bool due32(uint32_t now, uint32_t t)
{
    return (int32_t)(now - t) >= 0;
}

static inline bool before32(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) < 0;
}

static uint32_t frame_ticks_for(uint8_t speed)
{
    return (speed == USBHS_SPEED_HS) ? USBHS_HS_UFRAME_TICKS : USBHS_FS_FRAME_TICKS;
}

static uint8_t speed_of(uint16_t port_status)
{
    uint16_t sp = (uint16_t)(port_status & USBHS_PS_SPEED_MASK);

    if (sp == USBHS_PS_HS)
    {
        return USBHS_SPEED_HS;
    }
    if (sp == USBHS_PS_LS)
    {
        return USBHS_SPEED_LS;
    }
    return USBHS_SPEED_FS;
}

static uint8_t trr_of(uint8_t pid)
{
    switch (pid)
    {
    case USBHS_PID_ACK:
    case USBHS_PID_NYET:
        return USBHS_TRR_ACK;
    case USBHS_PID_NAK:
        return USBHS_TRR_NAK;
    case USBHS_PID_STALL:
        return USBHS_TRR_STALL;
    case USBHS_PID_DATA0:
        return USBHS_TRR_DATA0;
    case USBHS_PID_DATA1:
        return USBHS_TRR_DATA1;
    default:
        return USBHS_TRR_OTHER;
    }
}

static uint8_t tok_of(uint8_t token)
{
    if (token == USBHS_PID_SETUP)
    {
        return USBHS_TOK_SETUP;
    }
    return (token == USBHS_PID_IN) ? USBHS_TOK_IN : USBHS_TOK_OUT;
}

/* ------------------------------------------------------------------ */
/* Queues (written by ISR or locked main loop, read by main loop)       */
/* ------------------------------------------------------------------ */

static void event_push(uint8_t type, uint8_t pipe, uint8_t result, uint32_t value, uint64_t time)
{
    uint32_t head = s_ev_head;
    UsbhsEvent *e;

    if ((uint32_t)(head - s_ev_tail) >= EVENT_RING)
    {
        s.st.event_drop++;
        return;
    }
    e = &s_ev[head & (EVENT_RING - 1u)];
    e->time = time;
    e->generation = s.generation;
    e->value = value;
    e->type = type;
    e->speed = s.speed;
    e->pipe = pipe;
    e->result = result;
    LAT_FENCE_RW();
    s_ev_head = head + 1u;
}

static UsbhsTrace *trace_begin(void)
{
    uint32_t head = s_tr_head;
    UsbhsTrace *t;

    if ((uint32_t)(head - s_tr_tail) >= TRACE_RING)
    {
        s.st.trace_drop++;
        return (UsbhsTrace *)0;
    }
    t = &s_tr[head & (TRACE_RING - 1u)];
    lat_bytes_fill((uint8_t *)t, 0u, (uint32_t)sizeof(*t));
    t->generation = s.generation;
    return t;
}

static void trace_commit(void)
{
    LAT_FENCE_RW();
    s_tr_head = s_tr_head + 1u;
}

static void generation_boundary_push(void)
{
    uint32_t head = s_gen_head;
    GenerationBoundary *b;

    if ((uint32_t)(head - s_gen_tail) >= GENERATION_RING)
    {
        s.st.trace_drop++;
        return;
    }
    b = &s_gen_boundary[head & (GENERATION_RING - 1u)];
    b->trace_pos = s_tr_head;
    b->generation = s.generation;
    b->drop_base = s.trace_drop_base;
    LAT_FENCE_RW();
    s_gen_head = head + 1u;
}

static void trace_port(uint8_t kind, uint64_t time, const uint8_t *data, uint8_t n)
{
    UsbhsTrace *t = trace_begin();

    if (t == (UsbhsTrace *)0)
    {
        return;
    }
    t->kind = kind;
    t->time = time;
    if (n > 0u)
    {
        lat_bytes_copy(t->data, data, n);
        t->data_saved = n;
    }
    trace_commit();
}

static void nak_flush(NakRun *r)
{
    UsbhsTrace *t;

    if (!r->open)
    {
        return;
    }
    r->open = 0u;
    t = trace_begin();
    if (t == (UsbhsTrace *)0)
    {
        return;
    }
    t->kind = USBHS_TRK_NAK_RUN;
    t->time = r->first;
    t->nak_last = r->last;
    t->nak_count = r->count;
    t->late = r->late;
    t->step = r->step;
    t->addr = r->addr;
    t->ep = r->ep;
    t->token = r->token;
    t->result = USBHS_TRR_NAK;
    t->toggle = r->toggle;
    t->pipe = r->pipe;
    trace_commit();
}

/* Coalesce consecutive NAKs into a NAK_RUN record; returns true if opened. */
static bool nak_note(NakRun *r, const Item *it, uint64_t issue, int32_t late, uint16_t step)
{
    uint8_t ep = (uint8_t)(it->ep | ((it->token == USBHS_PID_IN) ? 0x80u : 0u));

    if (r->open && (r->addr == it->addr) && (r->ep == ep) && (r->token == tok_of(it->token)) &&
        (r->step == step) && (r->toggle == it->toggle))
    {
        r->count++;
        r->last = issue;
        return false;
    }
    nak_flush(r);
    r->open = 1u;
    r->addr = it->addr;
    r->ep = ep;
    r->token = tok_of(it->token);
    r->toggle = it->toggle;
    r->pipe = it->pipe;
    r->step = step;
    r->count = 1u;
    r->late = late;
    r->first = issue;
    r->last = issue;
    return true;
}

static void trace_txn(const Item *it, uint8_t trr, uint16_t step, uint16_t length, const uint8_t *data,
                      uint64_t issue, uint64_t done, int32_t late, const uint8_t *setup)
{
    UsbhsTrace *t = trace_begin();
    uint16_t n = length;

    if (t == (UsbhsTrace *)0)
    {
        return;
    }
    t->kind = USBHS_TRK_TXN;
    t->time = issue;
    t->done = done;
    t->late = late;
    t->step = step;
    t->length = length;
    t->addr = it->addr;
    t->ep = (uint8_t)(it->ep | ((it->token == USBHS_PID_IN) ? 0x80u : 0u));
    t->token = tok_of(it->token);
    t->result = trr;
    t->toggle = it->toggle;
    t->pipe = it->pipe;
    if (setup != (const uint8_t *)0)
    {
        lat_bytes_copy(t->setup, setup, 8u);
    }
    if ((data != (const uint8_t *)0) && (n > 0u))
    {
        if (n > USBHS_TRACE_DATA_MAX)
        {
            n = USBHS_TRACE_DATA_MAX;
        }
        lat_bytes_copy(t->data, data, n);
        t->data_saved = (uint8_t)n;
    }
    trace_commit();
}

/* ------------------------------------------------------------------ */
/* SOF and frame mapping                                                */
/* ------------------------------------------------------------------ */

static void fm_setup(uint8_t speed)
{
    s.fm.valid = 0u;
    s.fm.have_raw = 0u;
    s.fm.late_run = 0u;
    s.fm.period = frame_ticks_for(speed);
    s.fm.mask = (speed == USBHS_SPEED_HS) ? 0x3FFFu : 0x7FFu;
}

static uint32_t fm_raw(uint32_t frame_reg)
{
    if (s.speed == USBHS_SPEED_HS)
    {
        return ((frame_reg & 0x7FFu) << 3) | ((frame_reg >> 16) & 7u);
    }
    return frame_reg & 0x7FFu;
}

/* Start timestamp of (micro)frame seq. */
static uint32_t fm_start(uint32_t seq)
{
    return s.fm.anchor_time + ((seq - s.fm.anchor_seq) * s.fm.period);
}

/* Extended (micro)frame sequence number containing timestamp t. */
static uint32_t fm_seq_at(uint32_t t)
{
    int32_t d = (int32_t)(t - s.fm.anchor_time);
    uint32_t q;

    if (d >= 0)
    {
        q = (uint32_t)d / s.fm.period;
        return s.fm.anchor_seq + q;
    }
    q = ((uint32_t)(-(d + 1)) / s.fm.period) + 1u;
    return s.fm.anchor_seq - q;
}

static void fm_anchor(uint32_t t)
{
    s.fm.anchor_time = t;
    s.fm.anchor_seq = s.fm.seq;
    s.fm.valid = 1u;
    s.fm.late_run = 0u;
    s.st.sof_reanchor++;
}

/*
 * Update SOF-to-frame anchor on SOF IRQ. Since ISR entry lags the actual SOF,
 * reanchor on earlier-than-predicted SOFs or after consecutive late SOFs.
 */
static void fm_sof(uint32_t t, uint32_t raw)
{
    FrameModel *m = &s.fm;
    int32_t lat;

    if (m->have_raw)
    {
        uint32_t d = (raw - m->last_raw) & m->mask;

        if (d == 0u)
        {
            s.st.sof_anomaly++;
            return;
        }
        m->seq += d;
    }
    else
    {
        m->seq = raw;
        m->have_raw = 1u;
    }
    m->last_raw = raw;
    if (!m->valid)
    {
        fm_anchor(t);
        return;
    }
    lat = (int32_t)(t - fm_start(m->seq));
    if (lat < 0)
    {
        fm_anchor(t);
    }
    else if (lat > (int32_t)SOF_LATE_MAX)
    {
        m->late_run++;
        if (m->late_run >= SOF_LATE_RUN)
        {
            fm_anchor(t);
        }
    }
    else
    {
        m->late_run = 0u;
    }
}

/* ------------------------------------------------------------------ */
/* Publish and issue                                                    */
/* ------------------------------------------------------------------ */

static void mark_inflight(uint8_t pipe, uint8_t epoch)
{
    if (pipe == PIPE_CTRL)
    {
        if (s.ctrl.active && (s.ctrl.epoch == epoch))
        {
            s.ctrl.pend = 0u;
            s.ctrl.inflight = 1u;
        }
        return;
    }
    if ((pipe <= USBHS_INTR_MAX) && s.intr[pipe - 1u].used && (s.intr[pipe - 1u].epoch == epoch))
    {
        s.intr[pipe - 1u].inflight = 1u;
    }
}

/*
 * Withdraw publication. If already popped (s_pub == 0 with pending_gen),
 * mark that pipe as in-flight. Must run with USBHS IRQ masked or inside it.
 */
static void unpublish_resolve(void)
{
    uint32_t prev = __atomic_exchange_n(&s_pub, 0u, __ATOMIC_SEQ_CST);

    if ((prev == 0u) && (s.pending_gen != 0u))
    {
        mark_inflight(s.pending_pipe, s.pending_epoch);
    }
    s.pending_gen = 0u;
}

static void publish(const Item *it)
{
    uint32_t slot = (uint32_t)(s.last_slot ^ 1u);
    uint32_t gen;

    s.pub_gen = (s.pub_gen + 1u) & 0x7FFFFFFFu;
    if (s.pub_gen == 0u)
    {
        s.pub_gen = 1u;
    }
    gen = s.pub_gen;
    s_slot[slot] = *it;
    s_slot[slot].gen = gen;
    s.last_slot = (uint8_t)slot;
    s.pending_gen = gen;
    s.pending_pipe = it->pipe;
    s.pending_epoch = it->epoch;
    /* Write slot before publishing in a single atomic store. */
    LAT_FENCE_RW();
    __atomic_store_n(&s_pub, (gen << 1) | slot, __ATOMIC_SEQ_CST);
}

bool usbhs_start_if_idle(uint8_t src)
{
    for (;;)
    {
        uint32_t pub;
        uint32_t cnt;

        if (__atomic_exchange_n(&s_busy, 1u, __ATOMIC_SEQ_CST) != 0u)
        {
            return false; /* Busy; USBHS ISR will re-evaluate after releasing s_busy */
        }
        pub = __atomic_load_n(&s_pub, __ATOMIC_SEQ_CST);
        cnt = usbhs_hw_cnt();
        if (pub != 0u)
        {
            const Item *it = &s_slot[pub & 1u];

            if (due32(cnt, it->sched))
            {
                if ((src == SRC_TIM9) && (it->token != USBHS_PID_IN) && (it->len > USBHS_TIM9_TX_MAX))
                {
                    /* Defer large TX copies to USBHS ISR to keep TIM9 ISR short. */
                    __atomic_store_n(&s_busy, 0u, __ATOMIC_SEQ_CST);
                    usbhs_hw_kick();
                    return false;
                }
                __atomic_store_n(&s_pub, 0u, __ATOMIC_SEQ_CST);
                s_inf.it = *it;
                s_inf.src = src;
                s_inf.valid = 1u;
                s_inf.issue_cnt = usbhs_hw_issue(it->addr, it->token, it->ep, it->toggle, it->len, it->tx);
                if (src == SRC_TIM9)
                {
                    s.st.issue_tim9++;
                }
                else
                {
                    s.st.issue_usbhs++;
                }
                return true;
            }
        }
        __atomic_store_n(&s_busy, 0u, __ATOMIC_SEQ_CST);
        if (src == SRC_TIM9)
        {
            return false;
        }
        /*
         * Re-check after releasing s_busy in case TIM9 CH2 fired while we held it.
         */
        pub = __atomic_load_n(&s_pub, __ATOMIC_SEQ_CST);
        if (pub == 0u)
        {
            return false;
        }
        if (!due32(usbhs_hw_cnt(), s_slot[pub & 1u].sched))
        {
            return false;
        }
    }
}

void usbhs_core_tim9_ch2(void)
{
    (void)usbhs_start_if_idle(SRC_TIM9);
}

/* Abort in-flight transaction and release s_busy (detach, reset, recovery). */
static void abort_inflight(void)
{
    unpublish_resolve();
    if (__atomic_load_n(&s_busy, __ATOMIC_SEQ_CST) != 0u)
    {
        usbhs_hw_abort();
        usbhs_hw_clear_transfer();
        s_inf.valid = 0u;
        __atomic_store_n(&s_busy, 0u, __ATOMIC_SEQ_CST);
    }
}

/* ------------------------------------------------------------------ */
/* Control transfers                                                    */
/* ------------------------------------------------------------------ */

static void ctrl_pend(const UsbhsWhen *w, uint64_t ref)
{
    Ctrl *c = &s.ctrl;
    uint32_t ref_lo = (uint32_t)ref;

    c->pend = 1u;
    c->inflight = 0u;
    c->count_late = (uint8_t)!((w->mode == USBHS_WHEN_DELAY) && (w->ticks == 0u));
    if (w->mode == USBHS_WHEN_NEXT_FRAME)
    {
        /* Fallback to +1 frame if no SOF anchor is available. */
        c->t_abs = ref_lo + s.fm.period + w->ticks;
        c->frame_rel = s.fm.valid;
        if (s.fm.valid)
        {
            c->f_seq = fm_seq_at(ref_lo) + 1u;
            c->f_off = w->ticks;
        }
    }
    else
    {
        c->frame_rel = 0u;
        c->t_abs = ref_lo + w->ticks;
    }
}

static uint32_t ctrl_time(void)
{
    const Ctrl *c = &s.ctrl;

    if (c->frame_rel && s.fm.valid)
    {
        return fm_start(c->f_seq) + c->f_off;
    }
    return c->t_abs;
}

static void ctrl_finish(uint8_t result, uint64_t t)
{
    Ctrl *c = &s.ctrl;

    nak_flush(&c->nak);
    c->res.result = result;
    c->res.stage = c->stage;
    c->res.actual = c->actual;
    c->res.end_time = t;
    c->active = 0u;
    c->pend = 0u;
    c->inflight = 0u;
    c->done = 1u;
    event_push(USBHS_EV_CTRL_DONE, PIPE_CTRL, result, c->res.actual, t);
}

/* Retry on no response, unexpected PID, or timeout up to error_retries. */
static void ctrl_error(uint8_t result, uint64_t t1)
{
    Ctrl *c = &s.ctrl;

    c->res.error_count++;
    c->errors++;
    if (c->errors > c->req.timing.error_retries)
    {
        ctrl_finish(result, t1);
        return;
    }
    ctrl_pend(&c->req.timing.error, t1);
}

/* NAK or toggle mismatch: use nak_first on stage start, nak_cont mid-DATA. */
static void ctrl_retry(uint64_t t1)
{
    Ctrl *c = &s.ctrl;

    c->res.nak_count++;
    ctrl_pend(c->first_pkt ? &c->req.timing.nak_first : &c->req.timing.nak_cont, t1);
}

static void ctrl_to_status(uint64_t t1)
{
    Ctrl *c = &s.ctrl;

    c->stage = USBHS_STAGE_STATUS;
    c->toggle = 1u;
    c->first_pkt = 1u;
    c->errors = 0u;
    ctrl_pend(&c->req.timing.status, t1);
}

static void ctrl_complete(const Item *it, bool timeout, uint8_t pid, uint16_t rx_len, uint64_t t1,
                          uint64_t issue, int32_t late)
{
    Ctrl *c = &s.ctrl;
    const uint8_t *rx = usbhs_hw_rx();
    uint8_t trr = timeout ? USBHS_TRR_TIMEOUT : trr_of(pid);
    bool data_pid = (pid == USBHS_PID_DATA0) || (pid == USBHS_PID_DATA1);
    bool ack = (pid == USBHS_PID_ACK) || (pid == USBHS_PID_NYET);
    bool match = data_pid && ((pid == USBHS_PID_DATA1) == (it->toggle != 0u));

    c->inflight = 0u;
    c->pend = 0u;
    if (!c->active || (it->epoch != c->epoch))
    {
        return; /* Completion of aborted transfer */
    }
    c->res.last_pid = pid;

    /* Coalesce NAKs; trace all other outcomes individually. */
    if (c->req.trace)
    {
        if (!timeout && (pid == USBHS_PID_NAK))
        {
            (void)nak_note(&c->nak, it, issue, late, c->req.step);
        }
        else
        {
            const uint8_t *data = (const uint8_t *)0;
            uint16_t len = 0u;

            nak_flush(&c->nak);
            if (it->token == USBHS_PID_IN)
            {
                if (data_pid && !timeout)
                {
                    data = rx;
                    len = rx_len;
                }
            }
            else if (it->token == USBHS_PID_OUT)
            {
                data = it->tx;
                len = it->len;
            }
            else
            {
                len = 8u;
            }
            trace_txn(it, trr, c->req.step, len, data, issue, timeout ? 0u : t1, late,
                      (it->token == USBHS_PID_SETUP) ? c->req.setup : (const uint8_t *)0);
        }
    }

    switch (c->stage)
    {
    case USBHS_STAGE_SETUP:
        if (c->res.setup_issue == 0u)
        {
            c->res.setup_issue = issue;
            c->res.setup_late = (int32_t)(issue - c->setup_target);
        }
        if (timeout)
        {
            ctrl_error(USBHS_RES_TIMEOUT, t1);
        }
        else if (ack)
        {
            c->res.setup_done = t1;
            c->errors = 0u;
            c->toggle = 1u;
            if (c->wlength > 0u)
            {
                c->stage = USBHS_STAGE_DATA;
                c->first_pkt = 1u;
                ctrl_pend(&c->req.timing.data_first, t1);
            }
            else
            {
                ctrl_to_status(t1);
            }
        }
        else if (pid == USBHS_PID_NAK)
        {
            ctrl_retry(t1); /* Protocol violation by device, but retry as stage-start NAK */
        }
        else if (pid == USBHS_PID_STALL)
        {
            ctrl_finish(USBHS_RES_STALL, t1);
        }
        else
        {
            ctrl_error(USBHS_RES_ERROR, t1);
        }
        break;

    case USBHS_STAGE_DATA:
        if (c->res.data_issue == 0u)
        {
            c->res.data_issue = issue;
        }
        if (timeout)
        {
            ctrl_error(USBHS_RES_TIMEOUT, t1);
        }
        else if (c->dir_in && match)
        {
            uint16_t n = rx_len;
            uint16_t room = (uint16_t)(c->wlength - c->actual);

            if (n > c->req.mps)
            {
                n = c->req.mps;
            }
            if (n > room)
            {
                n = room;
            }
            if ((c->req.buf != (uint8_t *)0) && (c->actual < c->req.buf_len))
            {
                uint16_t keep = n;

                if (keep > (uint16_t)(c->req.buf_len - c->actual))
                {
                    keep = (uint16_t)(c->req.buf_len - c->actual);
                }
                lat_bytes_copy(&c->req.buf[c->actual], rx, keep);
            }
            c->actual = (uint16_t)(c->actual + n);
            c->toggle ^= 1u;
            c->errors = 0u;
            c->res.data_done = t1;
            if ((rx_len < c->req.mps) || (c->actual >= c->wlength))
            {
                ctrl_to_status(t1); /* Short packet or wLength reached */
            }
            else
            {
                c->first_pkt = 0u;
                ctrl_pend(&c->req.timing.data_next, t1);
            }
        }
        else if (!c->dir_in && ack)
        {
            c->actual = (uint16_t)(c->actual + it->len);
            c->toggle ^= 1u;
            c->errors = 0u;
            c->res.data_done = t1;
            if (c->actual >= c->wlength)
            {
                ctrl_to_status(t1);
            }
            else
            {
                c->first_pkt = 0u;
                ctrl_pend(&c->req.timing.data_next, t1);
            }
        }
        else if ((pid == USBHS_PID_NAK) || (c->dir_in && data_pid))
        {
            ctrl_retry(t1); /* NAK or toggle mismatch (ACKed & dropped); keep toggle */
        }
        else if (pid == USBHS_PID_STALL)
        {
            ctrl_finish(USBHS_RES_STALL, t1);
        }
        else
        {
            ctrl_error(USBHS_RES_ERROR, t1);
        }
        break;

    default: /* STATUS */
        if (c->res.status_issue == 0u)
        {
            c->res.status_issue = issue;
        }
        if (timeout)
        {
            ctrl_error(USBHS_RES_TIMEOUT, t1);
        }
        else if (((it->token == USBHS_PID_IN) && match) || ((it->token == USBHS_PID_OUT) && ack))
        {
            c->res.status_done = t1;
            ctrl_finish(USBHS_RES_OK, t1);
        }
        else if ((pid == USBHS_PID_NAK) || data_pid)
        {
            ctrl_retry(t1);
        }
        else if (pid == USBHS_PID_STALL)
        {
            ctrl_finish(USBHS_RES_STALL, t1);
        }
        else
        {
            ctrl_error(USBHS_RES_ERROR, t1);
        }
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Periodic endpoints                                                   */
/* ------------------------------------------------------------------ */

static void report_push(const UsbhsIntrDone *d, const uint8_t *rx)
{
    uint32_t head = s_rep_head;
    RepSlot *r;

    if ((uint32_t)(head - s_rep_tail) >= REPORT_RING)
    {
        s.st.report_drop++;
        return;
    }
    r = &s_rep[head & (REPORT_RING - 1u)];
    r->d = *d;
    lat_bytes_copy(r->data, rx, d->len);
    LAT_FENCE_RW();
    s_rep_head = head + 1u;
}

static void intr_close(Intr *p)
{
    nak_flush(&p->nak);
    p->used = 0u;
    p->state = INTR_CLOSED;
    p->inflight = 0u;
}

static void close_all_intr(void)
{
    uint32_t i;

    for (i = 0u; i < USBHS_INTR_MAX; i++)
    {
        if (s.intr[i].used)
        {
            intr_close(&s.intr[i]);
        }
    }
}

static uint8_t trace_result_of(uint8_t result)
{
    if (result == USBHS_RES_STALL)
    {
        return USBHS_TRR_STALL;
    }
    if (result == USBHS_RES_TIMEOUT)
    {
        return USBHS_TRR_TIMEOUT;
    }
    return USBHS_TRR_OTHER;
}

static void trace_intr_state(const Intr *p, uint8_t kind, uint8_t result, uint64_t time, uint64_t resume_at)
{
    UsbhsTrace *t = trace_begin();

    if (t == (UsbhsTrace *)0)
    {
        return;
    }
    t->kind = kind;
    t->time = time;
    t->addr = p->addr;
    t->ep = (uint8_t)(p->ep | 0x80u);
    t->result = (kind == USBHS_TRK_IN_PAUSE) ? trace_result_of(result) : 0u;
    t->nak_count = p->err_streak;
    t->nak_last = (kind == USBHS_TRK_IN_PAUSE) ? resume_at : 0u;
    trace_commit();
}

static void intr_error_inc(Intr *p)
{
    if (p->err_streak != 0xFFu)
    {
        p->err_streak++;
    }
}

static void intr_complete(Intr *p, const Item *it, bool timeout, uint8_t pid, uint16_t rx_len, uint64_t t1,
                          uint64_t issue, int32_t late, uint32_t frame_reg)
{
    const uint8_t *rx = usbhs_hw_rx();
    UsbhsIntrDone d;
    uint8_t res;
    uint8_t trr = timeout ? USBHS_TRR_TIMEOUT : trr_of(pid);
    bool is_err;

    p->inflight = 0u;
    if (!p->used || (it->epoch != p->epoch))
    {
        return; /* Completion for closed pipe */
    }
    p->st.issued++;
    if ((late > 0) && ((uint32_t)late > p->st.late_max))
    {
        p->st.late_max = (uint32_t)late;
    }

    lat_bytes_fill((uint8_t *)&d, 0u, (uint32_t)sizeof(d));
    if (timeout)
    {
        res = USBHS_RES_TIMEOUT;
    }
    else if (p->dir_in)
    {
        if ((pid == USBHS_PID_DATA0) || (pid == USBHS_PID_DATA1))
        {
            if ((pid == USBHS_PID_DATA1) == (p->toggle != 0u))
            {
                res = USBHS_RES_OK;
                p->toggle ^= 1u;
                d.len = (rx_len > p->mps) ? p->mps : rx_len;
            }
            else
            {
                res = USBHS_RES_TOGGLE; /* Retransmitted data (ACKed & dropped); keep toggle */
            }
        }
        else if (pid == USBHS_PID_NAK)
        {
            res = USBHS_RES_NAK;
        }
        else if (pid == USBHS_PID_STALL)
        {
            res = USBHS_RES_STALL;
        }
        else
        {
            res = USBHS_RES_ERROR;
        }
    }
    else
    {
        if ((pid == USBHS_PID_ACK) || (pid == USBHS_PID_NYET))
        {
            res = USBHS_RES_OK;
            p->toggle ^= 1u;
            d.len = it->len;
            p->q_tail++;
        }
        else if (pid == USBHS_PID_NAK)
        {
            res = USBHS_RES_NAK; /* Retry same packet next period */
        }
        else if (pid == USBHS_PID_STALL)
        {
            res = USBHS_RES_STALL;
        }
        else
        {
            res = USBHS_RES_ERROR;
        }
    }
    is_err = (res == USBHS_RES_ERROR) || (res == USBHS_RES_TIMEOUT);

    switch (res)
    {
    case USBHS_RES_OK:
        p->st.data++;
        p->err_streak = 0u;
        break;
    case USBHS_RES_NAK:
        p->st.nak++;
        if (p->dir_in)
        {
            p->err_streak = 0u;
        }
        break;
    case USBHS_RES_TOGGLE:
        p->st.toggle_err++;
        break;
    case USBHS_RES_STALL:
        p->st.stall++;
        break;
    default:
        p->st.error++;
        break;
    }

    d.t1 = t1;
    d.issue = issue;
    d.generation = s.generation;
    d.frame = (uint16_t)(frame_reg & 0x7FFu);
    d.uframe = (s.speed == USBHS_SPEED_HS) ? (uint8_t)((frame_reg >> 16) & 7u) : 0u;
    d.pipe = it->pipe;
    d.result = res;
    d.pid = pid;
    if (p->dir_in && (res == USBHS_RES_OK))
    {
        d.seq = s.report_seq++;
        report_push(&d, rx);
    }
    if (p->hook != (UsbhsIntrHook)0)
    {
        p->hook(&d, (p->dir_in && (res == USBHS_RES_OK)) ? rx : (const uint8_t *)0);
    }
    if (!p->dir_in && (res == USBHS_RES_OK))
    {
        event_push(USBHS_EV_OUT_DONE, it->pipe, res, d.len, t1);
    }

    /* Trace first trace_budget normal txns and all errors. */
    if (res == USBHS_RES_NAK)
    {
        if (p->nak.open || (p->trace_budget > 0u))
        {
            if (nak_note(&p->nak, it, issue, late, p->step) && (p->trace_budget > 0u))
            {
                p->trace_budget--;
            }
        }
    }
    else
    {
        bool always = is_err || (res == USBHS_RES_STALL);

        nak_flush(&p->nak);
        if (always || (p->trace_budget > 0u))
        {
            const uint8_t *data = p->dir_in ? ((res == USBHS_RES_OK) ? rx : (const uint8_t *)0) : it->tx;
            uint16_t len = p->dir_in ? d.len : it->len;

            trace_txn(it, trr, p->step, len, data, issue, timeout ? 0u : t1, late, (const uint8_t *)0);
            if (!always)
            {
                p->trace_budget--;
            }
        }
    }

    /* IN treats STALL/timeout/other errors uniformly for consecutive error count. */
    if (p->dir_in && ((res == USBHS_RES_STALL) || is_err))
    {
        uint64_t resume_at;

        intr_error_inc(p);
        resume_at = (p->err_streak > p->err_immediate) ? t1 + p->err_pause : t1;
        if (res == USBHS_RES_STALL)
        {
            p->state = INTR_HALTED;
            p->stall_pending = 1u;
            p->not_before = resume_at;
            p->has_nb = 1u;
            trace_intr_state(p, USBHS_TRK_IN_PAUSE, res, t1, resume_at);
            event_push(USBHS_EV_PIPE_HALT, it->pipe, res, (uint32_t)(resume_at - t1), t1);
        }
        else if (p->err_streak > p->err_immediate)
        {
            p->state = INTR_PAUSED;
            p->not_before = resume_at;
            p->has_nb = 1u;
            trace_intr_state(p, USBHS_TRK_IN_PAUSE, res, t1, resume_at);
            event_push(USBHS_EV_PIPE_HALT, it->pipe, res, p->err_pause, t1);
        }
    }
    else if (res == USBHS_RES_STALL)
    {
        p->state = INTR_HALTED;
        event_push(USBHS_EV_PIPE_HALT, it->pipe, res, 0u, t1);
    }
    else if (is_err)
    {
        p->err_streak++;
        if (p->err_streak > p->err_immediate)
        {
            p->state = INTR_PAUSED;
            p->not_before = t1 + p->err_pause;
            p->has_nb = 1u;
            event_push(USBHS_EV_PIPE_HALT, it->pipe, res, p->err_pause, t1);
        }
    }
    /* Advance by period from scheduled frame, not from completion time. */
    p->next_seq += p->period;
}

/*
 * Next issue time for a periodic pipe; skips missed frames by multiples of period.
 */
static bool intr_candidate(Intr *p, uint32_t now_lo, uint32_t *t_out)
{
    uint32_t cur;

    if (!p->used || p->inflight || (p->state == INTR_HALTED) || !s.fm.valid)
    {
        return false;
    }
    if (p->dir_in && (p->state == INTR_PAUSED))
    {
        uint32_t resume = (uint32_t)p->not_before;

        if (!due32(now_lo, resume))
        {
            return false;
        }
        p->state = INTR_RUN;
        trace_intr_state(p, USBHS_TRK_IN_RESUME, 0u,
                         p->not_before + (uint32_t)(now_lo - resume), 0u);
    }
    cur = fm_seq_at(now_lo);
    if (!p->seq_init)
    {
        p->next_seq = cur;
        p->seq_init = 1u;
    }
    if (p->has_nb)
    {
        uint32_t nb = (uint32_t)p->not_before;
        uint32_t seq_nb = fm_seq_at(nb);

        if (before32(fm_start(seq_nb) + p->slot, nb))
        {
            seq_nb++;
        }
        if ((int32_t)(seq_nb - p->next_seq) > 0)
        {
            uint32_t k = ((seq_nb - p->next_seq) + p->period - 1u) / p->period;

            p->next_seq += k * p->period;
        }
        p->has_nb = 0u;
        p->state = INTR_RUN;
    }
    if ((int32_t)(cur - p->next_seq) > 0)
    {
        uint32_t k = ((cur - p->next_seq) + p->period - 1u) / p->period;

        p->next_seq += k * p->period;
        if (p->dir_in || (p->q_head != p->q_tail))
        {
            p->st.missed += k;
        }
    }
    if (!p->dir_in && (p->q_head == p->q_tail))
    {
        return false; /* Queue empty */
    }
    *t_out = fm_start(p->next_seq) + p->slot;
    return true;
}

/* ------------------------------------------------------------------ */
/* Completion and watchdog                                              */
/* ------------------------------------------------------------------ */

static void dispatch(const Inflight *inf, bool timeout, uint8_t pid, uint16_t rx_len, uint64_t t1, uint32_t frame_reg)
{
    const Item *it = &inf->it;
    uint32_t ticks = (uint32_t)t1 - inf->issue_cnt;
    int32_t late = (int32_t)(inf->issue_cnt - it->sched);
    uint64_t issue = t1 - (uint64_t)ticks;

    if (s.pending_gen == it->gen)
    {
        s.pending_gen = 0u; /* Confirmed popped */
    }
    if (!timeout && (ticks > s.st.txn_max_ticks))
    {
        s.st.txn_max_ticks = ticks;
    }
    if (it->count_late && (late > (int32_t)LATE_TOLERANCE))
    {
        s.st.sched_late_count++;
        if ((uint32_t)late > s.st.sched_late_max)
        {
            s.st.sched_late_max = (uint32_t)late;
        }
    }
    if (it->pipe == PIPE_CTRL)
    {
        ctrl_complete(it, timeout, pid, rx_len, t1, issue, late);
    }
    else if (it->pipe <= USBHS_INTR_MAX)
    {
        intr_complete(&s.intr[it->pipe - 1u], it, timeout, pid, rx_len, t1, issue, late, frame_reg);
    }
}

static void handle_transfer(const UsbhsSnap *sn)
{
    Inflight inf;

    if ((__atomic_load_n(&s_busy, __ATOMIC_SEQ_CST) == 0u) || !s_inf.valid)
    {
        s.st.spurious_transfer++;
        return;
    }
    inf = s_inf;
    s_inf.valid = 0u;
    dispatch(&inf, false, (uint8_t)(sn->int_st & 0x0Fu), sn->rx_len, sn->now, sn->frame);
    /* Copy RX data before releasing s_busy (TIM9 may issue immediately after). */
    __atomic_store_n(&s_busy, 0u, __ATOMIC_SEQ_CST);
}

/*
 * Watchdog for missing completion IRQ: if >1 frame elapsed since issue,
 * re-check INT_FLAG and HOST_ACTION. Returns false if recovery ran.
 */
static bool sof_watchdog(const UsbhsSnap *sn)
{
    Inflight inf;

    if ((__atomic_load_n(&s_busy, __ATOMIC_SEQ_CST) == 0u) || !s_inf.valid)
    {
        return true;
    }
    if ((uint32_t)((uint32_t)sn->now - s_inf.issue_cnt) < frame_ticks_for(s.speed))
    {
        return true;
    }
    if ((usbhs_hw_int_flag() & USBHS_IF_TRANSFER) != 0u)
    {
        return true; /* Completed after snapshot; next ISR will handle it */
    }
    if ((usbhs_hw_control() & USBHS_CTRL_HOST_ACTION) == 0u)
    {
        s.st.txn_timeout++;
        usbhs_hw_clear_transfer();
        inf = s_inf;
        s_inf.valid = 0u;
        dispatch(&inf, true, 0u, 0u, sn->now, sn->frame);
        __atomic_store_n(&s_busy, 0u, __ATOMIC_SEQ_CST);
        return true;
    }
    core_recover(USBHS_CAUSE_ACTION_STUCK, sn->now);
    return false;
}

/* ------------------------------------------------------------------ */
/* Port state                                                           */
/* ------------------------------------------------------------------ */

static void abort_all(uint8_t result, uint64_t now)
{
    abort_inflight();
    if (s.ctrl.active)
    {
        ctrl_finish(result, now);
    }
    close_all_intr();
}

static void do_attach(uint16_t port_status, uint64_t now)
{
    uint8_t sp;

    s.generation++;
    s.trace_drop_base = s.st.trace_drop;
    generation_boundary_push();
    s.speed = speed_of(port_status);
    s.port_state = USBHS_PORT_DEBOUNCE;
    s.attach_time = now;
    s.configured = 0u;
    sp = s.speed;
    usbhs_hw_notify(LAT_PC_EVT_DEVICE_ATTACHED, s.generation, s.speed, 0u, 0u);
    trace_port(USBHS_TRK_ATTACH, now, &sp, 1u);
    event_push(USBHS_EV_ATTACH, 0u, 0u, 0u, now);
}

static void do_detach(uint64_t now)
{
    abort_all(USBHS_RES_DETACHED, now);
    s.port_state = USBHS_PORT_DISCONNECTED;
    s.configured = 0u;
    fm_setup(USBHS_SPEED_FS);
    usbhs_hw_notify(LAT_PC_EVT_DEVICE_DETACHED, s.generation, 0u, 0u, 0u);
    trace_port(USBHS_TRK_DETACH, now, (const uint8_t *)0, 0u);
    event_push(USBHS_EV_DETACH, 0u, 0u, 0u, now);
    s.speed = USBHS_SPEED_NONE;
}

static void reset_complete(uint16_t port_status, uint64_t now)
{
    uint8_t d[5];

    s.speed = speed_of(port_status);
    s.port_state = USBHS_PORT_ENABLED;
    s.reset_done = now;
    s.reset_ticks = (uint32_t)(now - s.reset_start);
    s.enabled_time = now;
    fm_setup(s.speed);
    /* Detach clears SOF_EN; re-enable it after reset. */
    usbhs_hw_sof_enable();
    lat_put_u32(d, s.reset_ticks);
    d[4] = s.speed;
    trace_port(USBHS_TRK_RESET_END, now, d, 5u);
    trace_port(USBHS_TRK_SPEED, now, &d[4], 1u);
    event_push(USBHS_EV_RESET_DONE, 0u, 0u, s.reset_ticks, now);
}

static void handle_port(const UsbhsSnap *sn)
{
    bool conn = (sn->port_status & USBHS_PS_CONNECT) != 0u;

    if ((sn->port_chg & USBHS_PC_CONNECT) != 0u)
    {
        switch (s.port_state)
        {
        case USBHS_PORT_DISCONNECTED:
            if (conn)
            {
                do_attach(sn->port_status, sn->now);
            }
            break;
        case USBHS_PORT_DEBOUNCE:
            if (conn)
            {
                /* Debounce bounce: restart debounce timer. */
                s.attach_time = sn->now;
                event_push(USBHS_EV_BOUNCE, 0u, 0u, 0u, sn->now);
            }
            else
            {
                do_detach(sn->now);
            }
            break;
        case USBHS_PORT_RESETTING:
        case USBHS_PORT_ENABLED:
            if (!conn)
            {
                do_detach(sn->now);
            }
            else if ((s.port_state == USBHS_PORT_ENABLED) && ((sn->cfg & USBHS_CFG_SOF_EN) == 0u))
            {
                /* SOF_EN cleared: quick detach/reattach. */
                do_detach(sn->now);
                do_attach(sn->port_status, sn->now);
            }
            else
            {
                s.st.connect_glitch++; /* Ignore chirp/glitch during reset */
            }
            break;
        default:
            break;
        }
    }
    if (((sn->port_chg & USBHS_PC_RESET) != 0u) && (s.port_state == USBHS_PORT_RESETTING))
    {
        if (conn)
        {
            reset_complete(sn->port_status, sn->now);
        }
        else
        {
            do_detach(sn->now);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Scheduler                                                            */
/* ------------------------------------------------------------------ */

static void build_item(uint8_t pipe, uint32_t t, Item *it)
{
    lat_bytes_fill((uint8_t *)it, 0u, (uint32_t)sizeof(*it));
    it->sched = t;
    it->pipe = pipe;
    if (pipe == PIPE_CTRL)
    {
        Ctrl *c = &s.ctrl;

        it->epoch = c->epoch;
        it->addr = c->req.addr;
        it->ep = 0u;
        it->toggle = c->toggle;
        it->count_late = c->count_late;
        if (c->stage == USBHS_STAGE_SETUP)
        {
            it->token = USBHS_PID_SETUP;
            it->len = 8u;
            it->tx = c->req.setup;
            it->toggle = 0u;
        }
        else if (c->stage == USBHS_STAGE_DATA)
        {
            if (c->dir_in)
            {
                it->token = USBHS_PID_IN;
                it->len = c->req.mps;
            }
            else
            {
                uint16_t n = (uint16_t)(c->wlength - c->actual);

                if (n > c->req.mps)
                {
                    n = c->req.mps;
                }
                it->token = USBHS_PID_OUT;
                it->len = n;
                it->tx = &c->req.buf[c->actual];
                c->pkt_len = n;
            }
        }
        else
        {
            /* STATUS: OUT ZLP after IN DATA, else IN ZLP (always DATA1). */
            it->token = (c->dir_in && (c->wlength > 0u)) ? USBHS_PID_OUT : USBHS_PID_IN;
            it->len = 0u;
            it->toggle = 1u;
        }
        return;
    }
    {
        Intr *p = &s.intr[pipe - 1u];

        it->epoch = p->epoch;
        it->addr = p->addr;
        it->ep = p->ep;
        it->toggle = p->toggle;
        it->count_late = 1u;
        if (p->dir_in)
        {
            it->token = USBHS_PID_IN;
            it->len = p->mps;
        }
        else
        {
            const OutPkt *q = &p->q[p->q_tail & (USBHS_OUT_QUEUE - 1u)];

            it->token = USBHS_PID_OUT;
            it->len = q->len;
            it->tx = q->data;
        }
    }
}

typedef struct Cand
{
    uint8_t valid;
    uint8_t pipe;
    uint8_t prio;
    uint32_t t;
} Cand;

/*
 * Pick highest-priority overdue item (OUT > IN > ctrl), or earliest upcoming.
 */
static void consider(Cand *best, uint32_t now_lo, uint8_t pipe, uint8_t prio, uint32_t t)
{
    bool take;

    if (!best->valid)
    {
        take = true;
    }
    else
    {
        bool a_due = due32(now_lo, t);
        bool b_due = due32(now_lo, best->t);

        if (a_due != b_due)
        {
            take = a_due;
        }
        else if (a_due)
        {
            take = (prio < best->prio) || ((prio == best->prio) && before32(t, best->t));
        }
        else
        {
            take = before32(t, best->t) || ((t == best->t) && (prio < best->prio));
        }
    }
    if (take)
    {
        best->valid = 1u;
        best->pipe = pipe;
        best->prio = prio;
        best->t = t;
    }
}

static void schedule(uint64_t now)
{
    uint32_t now_lo = (uint32_t)now;
    Cand best;
    Item it;
    uint32_t i;
    Ctrl *c = &s.ctrl;

    unpublish_resolve();
    if (s.port_state != USBHS_PORT_ENABLED)
    {
        return;
    }
    lat_bytes_fill((uint8_t *)&best, 0u, (uint32_t)sizeof(best));

    if (c->active && c->pend && !c->inflight)
    {
        uint32_t t = ctrl_time();

        if ((int64_t)(now - c->deadline) >= 0)
        {
            ctrl_finish(USBHS_RES_TIMEOUT, now); /* T_CTRL_TIMEOUT */
        }
        else if (before32(t, (uint32_t)c->deadline))
        {
            consider(&best, now_lo, PIPE_CTRL, PRIO_CTRL, t);
        }
        /* Do not issue retries past timeout; ctrl_poll_timeout handles it on SOF. */
    }
    for (i = 0u; i < USBHS_INTR_MAX; i++)
    {
        uint32_t t;

        if (intr_candidate(&s.intr[i], now_lo, &t))
        {
            consider(&best, now_lo, (uint8_t)(i + 1u), s.intr[i].dir_in ? PRIO_IN : PRIO_OUT, t);
        }
    }
    if (!best.valid)
    {
        return;
    }
    build_item(best.pipe, best.t, &it);
    publish(&it);
    usbhs_hw_set_compare(it.sched);
    /* Issue immediately if compare time has already passed. */
    if (due32(usbhs_hw_cnt(), it.sched))
    {
        (void)usbhs_start_if_idle(SRC_USBHS);
    }
}

void usbhs_core_isr(const UsbhsSnap *sn)
{
    if (s.recovering)
    {
        return;
    }
    if ((sn->int_flag & USBHS_IF_TRANSFER) != 0u)
    {
        handle_transfer(sn);
    }
    if (sn->port_chg != 0u)
    {
        handle_port(sn);
    }
    if ((sn->int_flag & USBHS_IF_SOF_ACT) != 0u)
    {
        s_sof_count = s_sof_count + 1u;
        if (s.port_state == USBHS_PORT_ENABLED)
        {
            fm_sof((uint32_t)sn->now, fm_raw(sn->frame));
            if (!sof_watchdog(sn))
            {
                return; /* Recovery ran; skip scheduling */
            }
        }
    }
    schedule(sn->now);
}

/* ------------------------------------------------------------------ */
/* Recovery                                                             */
/* ------------------------------------------------------------------ */

static void core_recover(uint8_t cause, uint64_t now)
{
    uint8_t before = s.port_state;
    uint32_t old_gen = s.generation;
    uint16_t ps;
    bool connected;

    if (s.recovering)
    {
        return; /* Prevent re-entry */
    }
    s.recovering = 1u;
    usbhs_hw_irq_off();
    /* 1. Enter recovering state; 2. Abort in-flight txn and clear schedule. */
    s.port_state = USBHS_PORT_RECOVERING;
    abort_all(USBHS_RES_ABORTED, now);
    s.configured = 0u;
    s.st.recover_count++;
    /* 3. Reinit host controller; 4. Invalidate SOF anchor. */
    {
        bool ok = usbhs_hw_reinit();

        fm_setup(USBHS_SPEED_FS);
        if (!ok)
        {
            s.port_state = USBHS_PORT_HALTED;
            usbhs_hw_notify(LAT_PC_EVT_RECOVERED, 0u, cause, 1u, 0u);
            event_push(USBHS_EV_RECOVERED, 0u, 1u, cause, now);
            s.recovering = 0u;
            /* Fatal: keep USBHS and CH2 masked; re-enable TIM9 for timebase. */
            usbhs_hw_irq_on(false);
            return;
        }
    }
    /* 5. If a device is attached, restart from debounce as a new generation. */
    ps = usbhs_hw_port_status();
    connected = (ps & USBHS_PS_CONNECT) != 0u;
    if ((before != USBHS_PORT_DISCONNECTED) && (before != USBHS_PORT_HALTED))
    {
        usbhs_hw_notify(LAT_PC_EVT_DEVICE_DETACHED, old_gen, 0u, 0u, 0u);
        trace_port(USBHS_TRK_DETACH, now, (const uint8_t *)0, 0u);
        event_push(USBHS_EV_DETACH, 0u, 0u, 0u, now);
    }
    s.speed = USBHS_SPEED_NONE;
    s.port_state = USBHS_PORT_DISCONNECTED;
    if (connected)
    {
        do_attach(ps, now);
    }
    /* 6. Notify PC of recovery and cause. */
    usbhs_hw_notify(LAT_PC_EVT_RECOVERED, 0u, cause, 0u, 0u);
    event_push(USBHS_EV_RECOVERED, 0u, 0u, cause, now);
    /* 7. Clear flags, set compare, enable sources, re-enable IRQs. */
    usbhs_hw_clear_flags();
    usbhs_hw_set_compare(usbhs_hw_cnt() - 1u); /* Nothing published; push compare 1 wrap ahead */
    s.port_state = connected ? USBHS_PORT_DEBOUNCE : USBHS_PORT_DISCONNECTED;
    usbhs_hw_enable_sources();
    s.recovering = 0u;
    usbhs_hw_irq_on(true);
}

void usbhs_recover(uint8_t cause)
{
    core_recover(cause, usbhs_hw_now());
}

/* ------------------------------------------------------------------ */
/* Init and main loop                                                   */
/* ------------------------------------------------------------------ */

void usbhs_core_init(uint64_t now)
{
    lat_bytes_fill((uint8_t *)&s, 0u, (uint32_t)sizeof(s));
    s.port_state = USBHS_PORT_DISCONNECTED;
    fm_setup(USBHS_SPEED_FS);
    s.sof_next_check = now + SOF_CHECK_TICKS;
    s_busy = 0u;
    s_pub = 0u;
    s_inf.valid = 0u;
    s_sof_count = 0u;
    s_rep_head = 0u;
    s_rep_tail = 0u;
    s_ev_head = 0u;
    s_ev_tail = 0u;
    s_tr_head = 0u;
    s_tr_tail = 0u;
    s_gen_head = 0u;
    s_gen_tail = 0u;
    s_rec_len = 0u;
    s_rec_count = 0u;
    s_rec_dropped = 0u;
    s_rec_generation = 0u;
    s_rec_trace_drop_base = 0u;
    s_prev_rec_len = 0u;
}

void usbhs_core_boot_port(uint16_t port_status, uint64_t now)
{
    if (((port_status & USBHS_PS_CONNECT) != 0u) && (s.port_state == USBHS_PORT_DISCONNECTED))
    {
        do_attach(port_status, now);
    }
}

static void api_begin(void)
{
    usbhs_hw_lock();
}

static void api_end(bool kick)
{
    if (s.port_state == USBHS_PORT_HALTED)
    {
        return; /* Do not re-enable USBHS IRQ after fatal halt */
    }
    usbhs_hw_unlock();
    if (kick)
    {
        usbhs_hw_kick();
    }
}

static bool trace_pop(UsbhsTrace *out)
{
    uint32_t tail = s_tr_tail;

    if (tail == s_tr_head)
    {
        return false;
    }
    LAT_FENCE_RW();
    *out = s_tr[tail & (TRACE_RING - 1u)];
    LAT_FENCE_RW();
    s_tr_tail = tail + 1u;
    return true;
}

static uint32_t record_encode(uint8_t *p, const UsbhsTrace *t)
{
    uint32_t n = RECORD_FIXED + (uint32_t)t->data_saved;

    lat_put_u16(&p[0], (uint16_t)n);
    p[2] = t->kind;
    p[3] = t->addr;
    lat_put_u64(&p[4], t->time);
    p[12] = t->ep;
    p[13] = t->token;
    p[14] = t->result;
    p[15] = t->toggle;
    lat_put_u16(&p[16], t->step);
    lat_put_u16(&p[18], t->length);
    lat_put_i32(&p[20], t->late);
    lat_put_u32(&p[24], t->nak_count);
    lat_put_u64(&p[28], t->nak_last);
    lat_bytes_copy(&p[36], t->setup, 8u);
    lat_put_u64(&p[44], t->done);
    p[52] = t->data_saved;
    lat_bytes_copy(&p[53], t->data, t->data_saved);
    return n;
}

static void record_archive(uint32_t end_trace_drop)
{
    uint32_t src = 0u;
    uint32_t dst = LAT_PC_ENUM_RECORD_HDR_LEN;
    uint16_t count = 0u;
    uint32_t omitted = 0u;
    bool full = false;

    if (s_rec_generation == 0u)
    {
        return;
    }
    while (src < s_rec_len)
    {
        uint16_t n = lat_get_u16(&s_rec[src]);

        if ((n < RECORD_FIXED) || (n > (s_rec_len - src)))
        {
            omitted++;
            break;
        }
        if (full || ((uint32_t)n > (PREV_TRACE_BYTES - dst)))
        {
            omitted++;
            full = true;
        }
        else
        {
            lat_bytes_copy(&s_prev_rec[dst], &s_rec[src], n);
            dst += n;
            count++;
        }
        src += n;
    }
    lat_bytes_fill(s_prev_rec, 0u, LAT_PC_ENUM_RECORD_HDR_LEN);
    lat_put_u16(&s_prev_rec[0], LAT_PC_ENUM_RECORD_VERSION);
    lat_put_u16(&s_prev_rec[2], count);
    lat_put_u32(&s_prev_rec[4], (end_trace_drop - s_rec_trace_drop_base) + s_rec_dropped + omitted);
    lat_put_u32(&s_prev_rec[8], s_rec_generation);
    s_prev_rec_len = dst;
}

static void record_append(const UsbhsTrace *t)
{
    uint32_t n = RECORD_FIXED + (uint32_t)t->data_saved;

    if ((n > (RECORD_BYTES - s_rec_len)) || (s_rec_count == 0xFFFFu))
    {
        s_rec_dropped++;
        s.st.record_drop++;
        return;
    }
    (void)record_encode(&s_rec[s_rec_len], t);
    s_rec_len += n;
    s_rec_count++;
}

static void record_generation_start(const GenerationBoundary *b)
{
    record_archive(b->drop_base);
    s_rec_len = 0u;
    s_rec_count = 0u;
    s_rec_dropped = 0u;
    s_rec_generation = b->generation;
    s_rec_trace_drop_base = b->drop_base;
}

static void record_drain(void)
{
    UsbhsTrace t;

    for (;;)
    {
        while (s_gen_tail != s_gen_head)
        {
            const GenerationBoundary *b = &s_gen_boundary[s_gen_tail & (GENERATION_RING - 1u)];

            LAT_FENCE_RW();
            if (b->trace_pos != s_tr_tail)
            {
                break;
            }
            record_generation_start(b);
            LAT_FENCE_RW();
            s_gen_tail++;
        }
        if (!trace_pop(&t))
        {
            break;
        }
        record_append(&t);
    }
}

void usbhs_core_poll(uint64_t now)
{
    /* Check for stalled SOF every 10ms while enabled. */
    if ((int64_t)(now - s.sof_next_check) >= 0)
    {
        uint32_t c = s_sof_count;

        s.sof_next_check = now + SOF_CHECK_TICKS;
        if ((s.port_state == USBHS_PORT_ENABLED) && (c == s.sof_checked) &&
            ((now - s.enabled_time) >= SOF_CHECK_TICKS))
        {
            api_begin();
            if ((s.port_state == USBHS_PORT_ENABLED) && (s_sof_count == c))
            {
                core_recover(USBHS_CAUSE_SOF_STOPPED, now);
            }
            api_end(false);
        }
        s.sof_checked = s_sof_count;
    }

    /* Fallback if reset-done IRQ is missed: poll PORT_STATUS, recover if stuck. */
    if ((s.port_state == USBHS_PORT_RESETTING) && ((now - s.reset_start) >= RESET_LIMIT_TICKS))
    {
        api_begin();
        if (s.port_state == USBHS_PORT_RESETTING)
        {
            uint16_t ps = usbhs_hw_port_status();

            if (((ps & USBHS_PS_CONNECT) != 0u) && ((ps & USBHS_PS_RST) == 0u))
            {
                reset_complete(ps, now);
            }
            else
            {
                core_recover(USBHS_CAUSE_OTHER, now);
            }
        }
        api_end(true);
    }

    record_drain();
}

/* ------------------------------------------------------------------ */
/* API                                                                  */
/* ------------------------------------------------------------------ */

uint32_t usbhs_intr_period(uint8_t speed, uint8_t b_interval, uint8_t interval_mode)
{
    uint32_t v;

    if (speed == USBHS_SPEED_HS)
    {
        v = (b_interval < 1u) ? 1u : ((b_interval > 16u) ? 16u : b_interval);
        return 1u << (v - 1u);
    }
    v = (b_interval == 0u) ? 1u : b_interval;
    if (interval_mode == 0u)
    {
        uint32_t p = 1u;

        while ((p << 1) <= v)
        {
            p <<= 1;
        }
        return p;
    }
    return v;
}

void usbhs_ctrl_timing_default(UsbhsCtrlTiming *t)
{
    lat_bytes_fill((uint8_t *)t, 0u, (uint32_t)sizeof(*t));
    t->data_first.mode = USBHS_WHEN_DELAY;
    t->data_next.mode = USBHS_WHEN_DELAY;
    t->status.mode = USBHS_WHEN_DELAY;
    t->nak_first.mode = USBHS_WHEN_DELAY;
    t->nak_first.ticks = 10u * USBHS_TICKS_PER_US;
    t->nak_cont.mode = USBHS_WHEN_DELAY;
    t->nak_cont.ticks = 10u * USBHS_TICKS_PER_US;
    t->error.mode = USBHS_WHEN_DELAY;
    t->error.ticks = 10u * USBHS_TICKS_PER_US;
    t->timeout_ticks = CTRL_TIMEOUT_DEF;
    t->error_retries = 3u;
}

void usbhs_port_info(UsbhsPortInfo *out)
{
    api_begin();
    lat_bytes_fill((uint8_t *)out, 0u, (uint32_t)sizeof(*out));
    out->state = s.port_state;
    out->speed = s.speed;
    out->configured = s.configured;
    out->generation = s.generation;
    out->attach_time = s.attach_time;
    out->reset_start = s.reset_start;
    out->reset_done = s.reset_done;
    out->reset_ticks = s.reset_ticks;
    out->sof_count = s_sof_count;
    api_end(false);
}

bool usbhs_port_reset(uint8_t long_reset, uint16_t step)
{
    bool ok = false;
    uint64_t now;

    api_begin();
    now = usbhs_hw_now();
    if ((s.port_state == USBHS_PORT_DEBOUNCE) || (s.port_state == USBHS_PORT_ENABLED))
    {
        uint8_t lr = (uint8_t)(long_reset ? 1u : 0u);
        UsbhsTrace *t;

        abort_all(USBHS_RES_ABORTED, now);
        s.port_state = USBHS_PORT_RESETTING;
        s.configured = 0u;
        s.reset_start = now;
        fm_setup(USBHS_SPEED_FS);
        usbhs_hw_port_reset(lr);
        t = trace_begin();
        if (t != (UsbhsTrace *)0)
        {
            t->kind = USBHS_TRK_RESET_START;
            t->time = now;
            t->step = step;
            t->data[0] = lr;
            t->data_saved = 1u;
            trace_commit();
        }
        ok = true;
    }
    api_end(false);
    return ok;
}

uint32_t usbhs_frame_ticks(void)
{
    return frame_ticks_for(s.speed);
}

bool usbhs_frame_at(uint64_t t, uint16_t *frame, uint8_t *uframe)
{
    bool ok = false;

    api_begin();
    if (s.fm.valid)
    {
        uint32_t raw = fm_seq_at((uint32_t)t) & s.fm.mask;

        if (s.speed == USBHS_SPEED_HS)
        {
            *frame = (uint16_t)(raw >> 3);
            *uframe = (uint8_t)(raw & 7u);
        }
        else
        {
            *frame = (uint16_t)raw;
            *uframe = 0u;
        }
        ok = true;
    }
    api_end(false);
    return ok;
}

void usbhs_set_enum_state(bool configured, uint32_t enum_error)
{
    s.configured = (uint8_t)(configured ? 1u : 0u);
    s.st.enum_error = enum_error;
}

bool usbhs_ctrl_submit(const UsbhsCtrlReq *req)
{
    Ctrl *c = &s.ctrl;
    uint16_t wl = lat_get_u16(&req->setup[6]);
    bool dir_in = (req->setup[0] & 0x80u) != 0u;
    uint64_t now;
    uint64_t t0;

    if (((req->mps != 8u) && (req->mps != 16u) && (req->mps != 32u) && (req->mps != 64u)) ||
        (req->addr > 127u) ||
        ((wl > 0u) && (req->buf == (uint8_t *)0)) ||
        ((wl > 0u) && !dir_in && (req->buf_len < wl)))
    {
        return false;
    }
    api_begin();
    now = usbhs_hw_now();
    t0 = (req->start_at != 0u) ? req->start_at : now;
    if ((s.port_state != USBHS_PORT_ENABLED) || c->active ||
        (((int64_t)(t0 - now) > 0) && ((t0 - now) > SCHED_SPAN_MAX)))
    {
        api_end(false);
        return false;
    }
    lat_bytes_fill((uint8_t *)&c->res, 0u, (uint32_t)sizeof(c->res));
    lat_bytes_fill((uint8_t *)&c->nak, 0u, (uint32_t)sizeof(c->nak));
    c->req = *req;
    if (c->req.timing.timeout_ticks == 0u)
    {
        c->req.timing.timeout_ticks = CTRL_TIMEOUT_DEF;
    }
    c->active = 1u;
    c->done = 0u;
    c->inflight = 0u;
    c->stage = USBHS_STAGE_SETUP;
    c->dir_in = (uint8_t)(dir_in ? 1u : 0u);
    c->toggle = 0u;
    c->first_pkt = 1u;
    c->errors = 0u;
    c->wlength = wl;
    c->actual = 0u;
    c->epoch = ++s.epoch_counter;
    c->setup_target = t0;
    c->deadline = t0 + c->req.timing.timeout_ticks;
    c->res.step = req->step;
    c->res.generation = s.generation;
    c->pend = 1u;
    c->frame_rel = 0u;
    c->t_abs = (uint32_t)t0;
    c->count_late = (uint8_t)((req->start_at != 0u) ? 1u : 0u);
    api_end(true);
    return true;
}

bool usbhs_ctrl_poll(UsbhsCtrlResult *out)
{
    bool ok = false;

    api_begin();
    if (s.ctrl.done)
    {
        *out = s.ctrl.res;
        s.ctrl.done = 0u;
        ok = true;
    }
    api_end(false);
    return ok;
}

bool usbhs_ctrl_busy(void)
{
    return s.ctrl.active != 0u;
}

void usbhs_ctrl_abort(void)
{
    api_begin();
    unpublish_resolve();
    if (s.ctrl.active)
    {
        /* If already issued, completion will be ignored via transfer seq. */
        ctrl_finish(USBHS_RES_ABORTED, usbhs_hw_now());
    }
    api_end(true);
}

static Intr *intr_get(uint8_t pipe)
{
    if ((pipe == 0u) || (pipe > USBHS_INTR_MAX) || !s.intr[pipe - 1u].used)
    {
        return (Intr *)0;
    }
    return &s.intr[pipe - 1u];
}

uint8_t usbhs_intr_open(const UsbhsIntrCfg *cfg)
{
    uint8_t h = 0u;
    uint32_t i;

    if (((cfg->ep_addr & 0x0Fu) == 0u) || (cfg->mps == 0u) || (cfg->mps > USBHS_PKT_MAX) || (cfg->addr > 127u))
    {
        return 0u;
    }
    api_begin();
    if (s.port_state == USBHS_PORT_ENABLED)
    {
        for (i = 0u; i < USBHS_INTR_MAX; i++)
        {
            if (!s.intr[i].used)
            {
                Intr *p = &s.intr[i];
                uint64_t now = usbhs_hw_now();
                uint64_t nb = ((cfg->start_at != 0u) && ((int64_t)(cfg->start_at - now) > 0)) ? cfg->start_at : now;

                lat_bytes_fill((uint8_t *)p, 0u, (uint32_t)sizeof(*p));
                p->used = 1u;
                p->state = INTR_RUN;
                p->dir_in = (uint8_t)(((cfg->ep_addr & 0x80u) != 0u) ? 1u : 0u);
                p->toggle = (uint8_t)(cfg->toggle & 1u);
                p->epoch = ++s.epoch_counter;
                p->addr = cfg->addr;
                p->ep = (uint8_t)(cfg->ep_addr & 0x0Fu);
                p->trace_budget = cfg->trace_budget;
                p->err_immediate = cfg->err_immediate;
                p->mps = cfg->mps;
                p->step = cfg->step;
                p->period = usbhs_intr_period(s.speed, cfg->b_interval, cfg->interval_mode);
                p->slot = cfg->slot_ticks % frame_ticks_for(s.speed);
                p->err_pause = cfg->err_pause_ticks;
                p->generation = s.generation;
                p->hook = cfg->hook;
                p->not_before = nb; /* First issue at first slot >= nb */
                p->has_nb = 1u;
                p->st.period = p->period;
                h = (uint8_t)(i + 1u);
                break;
            }
        }
    }
    api_end(h != 0u);
    return h;
}

bool usbhs_intr_close(uint8_t pipe)
{
    Intr *p;
    bool ok = false;

    api_begin();
    unpublish_resolve();
    p = intr_get(pipe);
    if (p != (Intr *)0)
    {
        intr_close(p); /* In-flight completion will be ignored via transfer seq */
        ok = true;
    }
    api_end(true);
    return ok;
}

bool usbhs_intr_out_push(uint8_t pipe, const uint8_t *data, uint16_t len)
{
    Intr *p;
    bool ok = false;

    if ((len == 0u) || (len > USBHS_OUT_MAX))
    {
        return false;
    }
    api_begin();
    p = intr_get(pipe);
    if ((p != (Intr *)0) && !p->dir_in && ((uint32_t)(p->q_head - p->q_tail) < USBHS_OUT_QUEUE) &&
        (len <= p->mps))
    {
        OutPkt *q = &p->q[p->q_head & (USBHS_OUT_QUEUE - 1u)];

        q->len = len;
        lat_bytes_copy(q->data, data, len);
        p->q_head++;
        ok = true;
    }
    api_end(ok);
    return ok;
}

bool usbhs_intr_set_toggle(uint8_t pipe, uint8_t toggle)
{
    Intr *p;
    bool ok = false;

    api_begin();
    p = intr_get(pipe);
    if (p != (Intr *)0)
    {
        p->toggle = (uint8_t)(toggle & 1u);
        ok = true;
    }
    api_end(false);
    return ok;
}

bool usbhs_intr_resume(uint8_t pipe, uint64_t at)
{
    Intr *p;
    bool ok = false;

    api_begin();
    p = intr_get(pipe);
    if ((p != (Intr *)0) && !(p->dir_in && p->stall_pending))
    {
        uint64_t now = usbhs_hw_now();

        p->state = INTR_RUN;
        p->err_streak = 0u;
        p->not_before = ((at != 0u) && ((int64_t)(at - now) > 0)) ? at : now;
        p->has_nb = 1u;
        ok = true;
    }
    api_end(ok);
    return ok;
}

bool usbhs_intr_stats(uint8_t pipe, UsbhsIntrStats *out)
{
    Intr *p;
    bool ok = false;

    api_begin();
    p = intr_get(pipe);
    if (p != (Intr *)0)
    {
        *out = p->st;
        out->state = p->state;
        out->toggle = p->toggle;
        out->out_queued = (uint8_t)(p->q_head - p->q_tail);
        ok = true;
    }
    api_end(false);
    return ok;
}

bool usbhs_intr_recovery_pending(UsbhsIntrRecovery *out)
{
    uint64_t now;
    uint32_t i;
    bool found = false;

    api_begin();
    now = usbhs_hw_now();
    for (i = 0u; i < USBHS_INTR_MAX; i++)
    {
        Intr *p = &s.intr[i];

        if (p->used && p->dir_in && p->stall_pending &&
            ((int64_t)(now - p->not_before) >= 0))
        {
            out->at = p->not_before;
            out->generation = p->generation;
            out->pipe = (uint8_t)(i + 1u);
            out->address = p->addr;
            out->endpoint = (uint8_t)(p->ep | 0x80u);
            out->error_count = p->err_streak;
            found = true;
            break;
        }
    }
    api_end(false);
    return found;
}

bool usbhs_intr_recovery_complete(uint8_t pipe, uint32_t generation, bool success)
{
    Intr *p;
    uint64_t now;
    bool ok = false;

    api_begin();
    p = intr_get(pipe);
    now = usbhs_hw_now();
    if ((p != (Intr *)0) && p->dir_in && p->stall_pending &&
        (p->generation == generation) && (generation == s.generation))
    {
        if (success)
        {
            p->toggle = 0u;
            p->state = INTR_RUN;
            p->stall_pending = 0u;
            p->not_before = now;
            p->has_nb = 1u;
            trace_intr_state(p, USBHS_TRK_IN_RESUME, 0u, now, 0u);
        }
        else
        {
            p->st.error++;
            intr_error_inc(p);
            p->not_before = (p->err_streak > p->err_immediate) ? now + p->err_pause : now;
            p->has_nb = 1u;
            trace_intr_state(p, USBHS_TRK_IN_PAUSE, USBHS_RES_ERROR, now, p->not_before);
            event_push(USBHS_EV_PIPE_HALT, pipe, USBHS_RES_ERROR,
                       (uint32_t)(p->not_before - now), now);
        }
        ok = true;
    }
    api_end(ok && success);
    return ok;
}

bool usbhs_report_pop(UsbhsReport *out, uint8_t *data, uint16_t cap)
{
    uint32_t tail = s_rep_tail;
    const RepSlot *r;
    uint16_t n;

    if (tail == s_rep_head)
    {
        return false;
    }
    LAT_FENCE_RW();
    r = &s_rep[tail & (REPORT_RING - 1u)];
    *out = r->d;
    n = (r->d.len < cap) ? r->d.len : cap;
    if ((data != (uint8_t *)0) && (n > 0u))
    {
        lat_bytes_copy(data, r->data, n);
    }
    LAT_FENCE_RW();
    s_rep_tail = tail + 1u;
    return true;
}

bool usbhs_event_pop(UsbhsEvent *out)
{
    uint32_t tail = s_ev_tail;

    if (tail == s_ev_head)
    {
        return false;
    }
    LAT_FENCE_RW();
    *out = s_ev[tail & (EVENT_RING - 1u)];
    LAT_FENCE_RW();
    s_ev_tail = tail + 1u;
    return true;
}

uint32_t usbhs_record_export(uint8_t *dst, uint32_t cap)
{
    uint32_t total;
    uint32_t off;
    uint32_t i;
    uint8_t open_count = 0u;
    NakRun open[USBHS_INTR_MAX + 1u];

    api_begin();
    record_drain();
    if (s.ctrl.nak.open)
    {
        open[open_count++] = s.ctrl.nak;
    }
    for (i = 0u; i < USBHS_INTR_MAX; i++)
    {
        if (s.intr[i].used && s.intr[i].nak.open)
        {
            open[open_count++] = s.intr[i].nak;
        }
    }
    total = LAT_PC_ENUM_RECORD_HDR_LEN + s_rec_len + (uint32_t)open_count * RECORD_FIXED;
    if (cap < total)
    {
        api_end(false);
        return 0u;
    }
    lat_bytes_fill(dst, 0u, LAT_PC_ENUM_RECORD_HDR_LEN);
    lat_put_u16(&dst[0], LAT_PC_ENUM_RECORD_VERSION);
    lat_put_u16(&dst[2], (uint16_t)(s_rec_count + open_count));
    lat_put_u32(&dst[4], (s.st.trace_drop - s_rec_trace_drop_base) + s_rec_dropped);
    lat_put_u32(&dst[8], s_rec_generation);
    dst[12] = (open_count != 0u) ? 1u : 0u;
    dst[13] = open_count;
    lat_bytes_copy(&dst[LAT_PC_ENUM_RECORD_HDR_LEN], s_rec, s_rec_len);
    off = LAT_PC_ENUM_RECORD_HDR_LEN + s_rec_len;
    for (i = 0u; i < open_count; i++)
    {
        UsbhsTrace t;

        lat_bytes_fill((uint8_t *)&t, 0u, (uint32_t)sizeof(t));
        t.kind = USBHS_TRK_NAK_RUN;
        t.time = open[i].first;
        t.nak_last = open[i].last;
        t.nak_count = open[i].count;
        t.late = open[i].late;
        t.step = open[i].step;
        t.addr = open[i].addr;
        t.ep = open[i].ep;
        t.token = open[i].token;
        t.result = USBHS_TRR_NAK;
        t.toggle = open[i].toggle;
        off += record_encode(&dst[off], &t);
    }
    api_end(false);
    return total;
}

uint32_t usbhs_record_previous_export(uint8_t *dst, uint32_t cap)
{
    api_begin();
    record_drain();
    if ((s_prev_rec_len == 0u) || (cap < s_prev_rec_len))
    {
        api_end(false);
        return 0u;
    }
    lat_bytes_copy(dst, s_prev_rec, s_prev_rec_len);
    api_end(false);
    return s_prev_rec_len;
}

void usbhs_record_clear(void)
{
    s_rec_len = 0u;
    s_rec_count = 0u;
    s_rec_dropped = 0u;
    s_rec_generation = s.generation;
    s_rec_trace_drop_base = s.st.trace_drop;
}

bool usbhs_record_event(const UsbhsTrace *event)
{
    UsbhsTrace *t;
    bool ok = false;

    if ((event == (const UsbhsTrace *)0) || (event->data_saved > USBHS_TRACE_DATA_MAX))
    {
        return false;
    }
    api_begin();
    t = trace_begin();
    if (t != (UsbhsTrace *)0)
    {
        *t = *event;
        t->generation = s.generation;
        trace_commit();
        ok = true;
    }
    api_end(false);
    return ok;
}

void usbhs_stats(UsbhsStats *out)
{
    *out = s.st;
}
