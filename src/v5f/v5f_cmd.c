/*
 * v5f_cmd.c - V5F PC command framework (see v5f_cmd.h)
 *
 * Non-OK responses carry no payload.
 * Payloads are read/written byte-wise via lat_bytes.h without struct casts.
 */
#include "v5f_cmd.h"

#include "lat_bytes.h"
#include "lat_crc32.h"
#include "lat_ipc.h"
#include "lat_pc_proto.h"
#include "measure/meas_trial.h"
#include "measure/measure.h"
#include "v5f_cmd_port.h"

#define V5F_TRIGGER_ALL      ((uint8_t)0xFFu)
/* Power-of-two capacity for ISR-to-main event staging queue. */
#define V5F_ISR_RING         (64u)

_Static_assert((V5F_ISR_RING & (V5F_ISR_RING - 1u)) == 0u, "V5F_ISR_RING must be a power of two");

typedef struct TermPending
{
    bool pending;
    uint8_t pc_type;
    uint16_t len;
    uint8_t body[LAT_PC_POLL_RESULT_LEN];
} TermPending;

/* Single ISR event record (v5f_cmd_isr_event). */
typedef struct IsrEvent
{
    uint8_t pc_type;
    uint8_t arg0;
    uint8_t arg1;
    uint8_t reserved;
    uint16_t code;
    uint32_t word;
} IsrEvent;

typedef struct V5fCmdState
{
    uint32_t session;        /* Active session ID (0 = none) */
    bool overflow;           /* Rejects new trials/poll tests */
    bool quiesced;           /* Suppresses PC events */
    uint32_t last_run_id;    /* Last accepted run_id in this session */
    uint8_t trigger_asserted;

    uint8_t profile[LAT_PC_HOST_PROFILE_LEN];
    uint8_t vendor_count;
    uint8_t vendor[LAT_PC_VENDOR_TABLE_MAX * LAT_PC_VENDOR_ENTRY_LEN];
    uint8_t calib[LAT_PC_CALIBRATION_LEN];
    bool watch_set;
    uint8_t watch[LAT_PC_SET_WATCH_LEN];

    uint16_t object_id;
    uint32_t device_generation;

    TermPending term_run;
    TermPending term_poll;
    uint32_t isr_lost_seen;  /* Acknowledged isr.lost_count */

    uint32_t cmd_count;
    uint32_t cmd_deferred;
    uint32_t cmd_stale;
} V5fCmdState;

static V5fCmdState s;
static uint32_t s_crc_table[256];
/* Scratch buffers for popped command and outgoing response/event (1160B each). */
static LatRingRecord s_cmd;
static uint8_t s_out[LAT_RING_PAYLOAD_MAX];

/* SPSC queue from USBHS ISR (producer) to main loop (consumer). */
static IsrEvent s_isr_ring[V5F_ISR_RING];
static volatile uint32_t s_isr_head;   /* Written only by ISR */
static volatile uint32_t s_isr_tail;   /* Written only by main loop */

/* ------------------------------------------------------------------ */
/* TX                                                                   */
/* ------------------------------------------------------------------ */

static void env_put(uint8_t *e, uint8_t pc_type, uint8_t status, uint16_t seq)
{
    lat_put_u32(&e[LAT_IPC_ENV_OFF_SESSION], s.session);
    e[LAT_IPC_ENV_OFF_TYPE] = pc_type;
    e[LAT_IPC_ENV_OFF_STATUS] = status;
    lat_put_u16(&e[LAT_IPC_ENV_OFF_SEQ], seq);
}

/*
 * Enqueue a reliable PC event; returns false if full.
 * Drops and returns true if no session is active or quiesced.
 */
static bool emit_critical(uint8_t pc_type, const uint8_t *body, uint32_t len)
{
    if ((s.session == 0u) || s.quiesced)
    {
        return true;
    }
    if (len > LAT_IPC_ENV_BODY_MAX)
    {
        return true;
    }
    env_put(s_out, pc_type, 0u, 0u);
    lat_bytes_copy(&s_out[LAT_IPC_ENV_LEN], body, len);
    return lat_ring_try_push(&g_ring_evt_safe, LAT_EVT_PC_EVENT, 0u, s_out, LAT_IPC_ENV_LEN + len);
}

/*
 * Send response (payload already written at &s_out[LAT_IPC_ENV_LEN]).
 * Space was checked before popping the command.
 */
static void respond(uint8_t pc_type, uint16_t seq, uint8_t status, uint32_t len)
{
    if (status != LAT_PC_ST_OK)
    {
        len = 0u;
    }
    env_put(s_out, (uint8_t)(pc_type | LAT_PC_RESPONSE_BIT), status, seq);
    (void)lat_ring_push_nowait(&g_ring_evt_safe, LAT_EVT_PC_RESP, 0u, s_out, LAT_IPC_ENV_LEN + len);
}

/* ------------------------------------------------------------------ */
/* State                                                                */
/* ------------------------------------------------------------------ */

/* Measurement engine (src/v5f/measure/) state. */
static bool trial_running(void)
{
    return measure_trial_running();
}

static bool poll_running(void)
{
    return measure_poll_running();
}

static bool calibration_active(void)
{
    return measure_calib_active();
}

bool v5f_cmd_emit_event(uint8_t pc_type, const uint8_t *body, uint32_t len)
{
    return emit_critical(pc_type, body, len);
}

bool v5f_cmd_emit_lossy(uint8_t ipc_type, uint8_t pc_type, const uint8_t *body, uint32_t len)
{
    if ((s.session == 0u) || s.quiesced || (len > LAT_IPC_ENV_BODY_MAX))
    {
        return true; /* Drop */
    }
    env_put(s_out, pc_type, 0u, 0u);
    lat_bytes_copy(&s_out[LAT_IPC_ENV_LEN], body, len);
    return lat_ring_push_lossy(&g_ring_evt_lossy, ipc_type, 0u, s_out, LAT_IPC_ENV_LEN + len);
}

/* True if the terminal event slot is still in flight (V5F or V3F). */
static bool term_busy(bool run)
{
    const LatTermCounters *t = &g_ipc_header.term;

    if (run)
    {
        return s.term_run.pending ||
               (lat_ipc_word_load(&t->run_done_produced) != lat_ipc_load_acquire(&t->run_done_released));
    }
    return s.term_poll.pending ||
           (lat_ipc_word_load(&t->poll_result_produced) != lat_ipc_load_acquire(&t->poll_result_released));
}

static void release_all_triggers(void)
{
    v5f_port_trigger_write(V5F_TRIGGER_ALL, 0u);
    s.trigger_asserted = 0u;
}

static void object_invalidate(void)
{
    lat_ipc_store_release(&g_ipc_header.object.state, LAT_OBJ_STATE_NONE);
}

/* Defaults. */
static void set_defaults(void)
{
    lat_bytes_fill(s.profile, 0u, sizeof(s.profile));
    s.profile[0] = 0u; /* reserved */
    s.profile[1] = 1u; /* post_requests: send */
    s.profile[2] = 0u; /* interval_mode: round down to power of 2 */
    s.profile[3] = 0u; /* reset2_policy */
    s.profile[4] = 2u; /* app_init_switch sdl_open */
    s.profile[5] = 0u; /* app_init_sony none */

    lat_bytes_fill(s.calib, 0u, sizeof(s.calib));
    lat_put_u32(&s.calib[12], 0xFFFFFFFFu);
    lat_put_u32(&s.calib[16], 0xFFFFFFFFu);
    lat_put_u32(&s.calib[20], 0xFFFFFFFFu);
    s.calib[24] = 0u;

    s.vendor_count = 0u;
    s.watch_set = false;
}

/* Enter or leave quiesce (quiesce_req odd = quiesce). */
static void apply_quiesce(uint32_t req)
{
    bool q = (req & 1u) != 0u;

    if (q)
    {
        /* Stop calibration mode and monitor; release triggers. */
        measure_quiesce();
        release_all_triggers();
    }
    s.quiesced = q;
}

/*
 * Start a new session: stop active runs, release triggers, discard staged
 * objects/terminal events/unprocessed ISR events, and clear overflow/quiesce.
 */
static void stop_for_session(uint32_t new_session)
{
    LatSessionCtl *c = &g_ipc_header.session;
    uint32_t q;

    /* Abort runs without generating RUN_DONE / POLL_RESULT. */
    measure_session_reset();
    release_all_triggers();
    object_invalidate();
    s.term_run.pending = false;
    s.term_poll.pending = false;
    s.overflow = false;
    s.last_run_id = 0u;
    /* Discard pre-session ISR events; any unseen loss triggers overflow in the new session. */
    s_isr_tail = s_isr_head;
    {
        uint32_t lost = lat_ipc_word_load(&g_ipc_header.isr.lost_count);

        if ((lost != s.isr_lost_seen) && (s.session != 0u))
        {
            lat_ipc_store_release(&c->v5f_ovf_gen, lat_ipc_word_load(&c->v5f_ovf_gen) + 1u);
        }
        s.isr_lost_seen = lost;
    }
    lat_ipc_store_release(&c->ovf_ack, lat_ipc_load_acquire(&c->ovf_req));
    q = lat_ipc_load_acquire(&c->quiesce_req);
    apply_quiesce(q);
    lat_ipc_store_release(&c->quiesce_ack, q);
    s.session = new_session;
}

/* Convert one ISR event to a reliable PC event; returns false if ring full. */
static bool emit_isr_event(const IsrEvent *ev)
{
    uint8_t body[LAT_PC_ERROR_LEN];
    uint32_t len;

    lat_bytes_fill(body, 0u, sizeof(body));
    switch (ev->pc_type)
    {
    case LAT_PC_EVT_DEVICE_ATTACHED:
    case LAT_PC_EVT_DEVICE_DETACHED:
        lat_put_u32(&body[0], ev->word);
        body[4] = ev->arg0;
        len = LAT_PC_DEVICE_EVT_LEN;
        break;
    case LAT_PC_EVT_RECOVERED:
        body[0] = ev->arg0;
        body[1] = ev->arg1;
        len = LAT_PC_RECOVERED_LEN;
        break;
    case LAT_PC_EVT_ERROR:
        lat_put_u16(&body[0], ev->code);
        lat_put_u32(&body[4], ev->word);
        len = LAT_PC_ERROR_LEN;
        break;
    default:
        return true; /* Drop unknown type */
    }
    return emit_critical(ev->pc_type, body, len);
}

void v5f_cmd_poll(void)
{
    LatSessionCtl *c = &g_ipc_header.session;
    uint32_t req;

    /* Session stop/start (highest priority). */
    req = lat_ipc_load_acquire(&c->session_req);
    if (req != lat_ipc_word_load(&c->session_ack))
    {
        stop_for_session(req);
        /* Publish cleared overflow/state before acknowledging session_ack. */
        v5f_cmd_publish();
        lat_ipc_store_release(&c->session_ack, req);
        return;
    }

    /* ISR queue overflow violates reliable delivery; enter overflow state. */
    {
        uint32_t lost = lat_ipc_word_load(&g_ipc_header.isr.lost_count);

        if (lost != s.isr_lost_seen)
        {
            s.isr_lost_seen = lost;
            if (s.session != 0u)
            {
                measure_overflow();
                release_all_triggers();
                s.overflow = true;
                v5f_cmd_publish();
                lat_ipc_store_release(&c->v5f_ovf_gen, lat_ipc_word_load(&c->v5f_ovf_gen) + 1u);
            }
        }
    }

    /* Overflow: abort active runs, release triggers, reject new runs. */
    req = lat_ipc_load_acquire(&c->ovf_req);
    if (req != lat_ipc_word_load(&c->ovf_ack))
    {
        measure_overflow();
        release_all_triggers();
        s.overflow = true;
        v5f_cmd_publish();
        lat_ipc_store_release(&c->ovf_ack, req);
    }

    /* Quiesce: odd = quiesce, even = resume. */
    req = lat_ipc_load_acquire(&c->quiesce_req);
    if (req != lat_ipc_word_load(&c->quiesce_ack))
    {
        apply_quiesce(req);
        lat_ipc_store_release(&c->quiesce_ack, req);
    }

    /* Flush pending terminal event once space is available. */
    if (s.term_run.pending &&
        emit_critical(s.term_run.pc_type, s.term_run.body, s.term_run.len))
    {
        s.term_run.pending = false;
    }
    if (s.term_poll.pending &&
        emit_critical(s.term_poll.pc_type, s.term_poll.body, s.term_poll.len))
    {
        s.term_poll.pending = false;
    }

    /* Drain ISR events in order into the reliable event ring. */
    for (;;)
    {
        uint32_t tail = s_isr_tail;

        if (tail == s_isr_head)
        {
            break;
        }
        LAT_FENCE_RW();
        if (!emit_isr_event(&s_isr_ring[tail & (V5F_ISR_RING - 1u)]))
        {
            break; /* Safe ring full; retry next poll */
        }
        s_isr_tail = tail + 1u;
    }
}

bool v5f_cmd_isr_event(uint8_t pc_type, uint32_t word, uint8_t arg0, uint8_t arg1, uint16_t code)
{
    LatIsrEvents *e = &g_ipc_header.isr;
    uint32_t head = s_isr_head;
    bool queued;
    IsrEvent *ev;

    /* Publish ISR counters via seqlock. */
    lat_seqlock_write_begin(&e->seq);
    switch (pc_type)
    {
    case LAT_PC_EVT_DEVICE_ATTACHED:
        e->attach_count++;
        e->last_generation = word;
        e->last_speed = arg0;
        break;
    case LAT_PC_EVT_DEVICE_DETACHED:
        e->detach_count++;
        e->last_generation = word;
        break;
    case LAT_PC_EVT_RECOVERED:
        e->recover_count++;
        e->recover_cause = arg0;
        e->recover_result = arg1;
        break;
    default:
        e->error_count++;
        e->error_code = code;
        e->error_detail = word;
        break;
    }
    queued = (uint32_t)(head - s_isr_tail) < V5F_ISR_RING;
    if (!queued)
    {
        e->lost_count++;
    }
    lat_seqlock_write_end(&e->seq);
    if (!queued)
    {
        return false;
    }

    ev = &s_isr_ring[head & (V5F_ISR_RING - 1u)];
    ev->pc_type = pc_type;
    ev->arg0 = arg0;
    ev->arg1 = arg1;
    ev->reserved = 0u;
    ev->code = code;
    ev->word = word;
    /* Publish after writing all fields. */
    LAT_FENCE_RW();
    s_isr_head = head + 1u;
    return true;
}

bool v5f_cmd_finish_run(uint8_t pc_type, const uint8_t *body, uint32_t len)
{
    bool run = (pc_type == LAT_PC_EVT_RUN_DONE);
    TermPending *t = run ? &s.term_run : &s.term_poll;
    uint32_t *produced = run ? &g_ipc_header.term.run_done_produced : &g_ipc_header.term.poll_result_produced;
    uint32_t expect = run ? LAT_PC_RUN_DONE_LEN : LAT_PC_POLL_RESULT_LEN;

    if ((!run && (pc_type != LAT_PC_EVT_POLL_RESULT)) || (len != expect) || term_busy(run))
    {
        return false;
    }
    lat_bytes_copy(t->body, body, len);
    t->len = (uint16_t)len;
    t->pc_type = pc_type;
    t->pending = true;
    lat_ipc_store_release(produced, lat_ipc_word_load(produced) + 1u);
    if (emit_critical(pc_type, t->body, t->len))
    {
        t->pending = false;
    }
    return true;
}

void v5f_cmd_publish(void)
{
    LatHostStatus h;

    lat_bytes_fill((uint8_t *)&h, 0u, (uint32_t)sizeof(h));
    v5f_port_host_status(&h);
    if (h.device_generation != s.device_generation)
    {
        object_invalidate();
        s.device_generation = h.device_generation;
    }
    h.run_kind = measure_run_kind();
    h.active_run_id = measure_active_run_id();
    h.flags = (uint8_t)((s.overflow ? LAT_HOST_FLAG_OVERFLOW : 0u) | (s.quiesced ? LAT_HOST_FLAG_QUIESCED : 0u) |
                        (measure_calib_active() ? LAT_HOST_FLAG_CALIBRATION : 0u) |
                        (measure_monitor_active() ? LAT_HOST_FLAG_MONITOR : 0u));
    h.trigger_asserted = (uint8_t)(s.trigger_asserted | measure_trigger_asserted());
    lat_ipc_host_publish(&h);
    lat_ipc_word_store(&g_ipc_header.v5f_capabilities,
                       LAT_PC_CAP_ENUM_RECORD | LAT_PC_CAP_MONITOR | LAT_PC_CAP_TRIAL | LAT_PC_CAP_POLL |
                           LAT_PC_CAP_CALIBRATION);
}

/* ------------------------------------------------------------------ */
/* Commands                                                             */
/* ------------------------------------------------------------------ */

static uint8_t *out_body(void)
{
    return &s_out[LAT_IPC_ENV_LEN];
}

/*
 * Populate OBJECT_BUF and return *object_length, or return LAT_PC_ST_* on error.
 */
static uint8_t object_fill(uint8_t kind, uint8_t index, uint32_t source_id, uint32_t *object_length)
{
    if (kind >= LAT_PC_OBJ_POLL_RAW)
    {
        /* kind 7..9: last completed poll test. */
        return measure_poll_object(kind, index, source_id, g_object_buf.data, LAT_OBJECT_BUF_BYTES, object_length);
    }
    if (kind != LAT_PC_OBJ_ENUM_RECORD)
    {
        /* kind 1..5: configured device descriptors. */
        uint8_t est = v5f_port_enum_object(kind, index, g_object_buf.data, LAT_OBJECT_BUF_BYTES, object_length);

        return ((est == LAT_PC_ST_OK) && (source_id != s.device_generation)) ? LAT_PC_ST_STALE_OBJECT : est;
    }
    if (index > 1u)
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    /* Snapshot the enumeration trace at this point. */
    *object_length = (index == 0u)
                         ? v5f_port_enum_trace_export(g_object_buf.data, LAT_OBJECT_BUF_BYTES)
                         : v5f_port_enum_trace_previous(g_object_buf.data, LAT_OBJECT_BUF_BYTES);
    if ((index == 1u) && (*object_length == 0u))
    {
        return LAT_PC_ST_NO_DEVICE;
    }
    if ((*object_length < LAT_PC_ENUM_RECORD_HDR_LEN) || (*object_length > LAT_OBJECT_BUF_BYTES))
    {
        return LAT_PC_ST_INTERNAL;
    }
    if (source_id != lat_get_u32(&g_object_buf.data[8]))
    {
        return LAT_PC_ST_STALE_OBJECT;
    }
    return LAT_PC_ST_OK;
}

static uint8_t cmd_get_object(const uint8_t *p, uint32_t n, uint32_t *len)
{
    uint8_t kind;
    uint8_t index;
    uint8_t st;
    uint32_t source_id;
    uint32_t object_length = 0u;
    uint32_t crc;
    uint32_t i;
    LatObjectMeta *m = &g_ipc_header.object;
    uint8_t *r = out_body();

    if (n != LAT_PC_GET_OBJECT_REQ_LEN)
    {
        return LAT_PC_ST_BAD_LENGTH;
    }
    /* Invalidate previous staged object on next GET_OBJECT. */
    object_invalidate();
    kind = p[0];
    index = p[1];
    source_id = lat_get_u32(&p[4]);
    if ((kind == 0u) || (kind > LAT_PC_OBJ_POLL_LONG))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    lat_ipc_store_release(&m->state, LAT_OBJ_STATE_BUILDING);
    st = object_fill(kind, index, source_id, &object_length);
    if (st != LAT_PC_ST_OK)
    {
        object_invalidate();
        return st;
    }
    crc = LAT_CRC32_INIT;
    for (i = 0u; i < object_length; i++)
    {
        crc = lat_crc32_update(s_crc_table, crc, g_object_buf.data[i]);
    }
    crc = lat_crc32_final(crc);

    s.object_id = (uint16_t)(s.object_id + 1u);
    if (s.object_id == 0u)
    {
        s.object_id = 1u;
    }
    m->session = s.session;
    m->object_id = s.object_id;
    m->kind = kind;
    m->index = index;
    m->source_id = source_id;
    m->length = object_length;
    m->crc32 = crc;
    /* Publish object completion only after data and metadata are written. */
    LAT_FENCE_RW();
    lat_ipc_store_release(&m->state, LAT_OBJ_STATE_READY);

    lat_put_u16(&r[0], s.object_id);
    r[2] = kind;
    r[3] = index;
    lat_put_u32(&r[4], source_id);
    lat_put_u32(&r[8], object_length);
    lat_put_u32(&r[12], crc);
    *len = LAT_PC_GET_OBJECT_RESP_LEN;
    return LAT_PC_ST_OK;
}

static uint8_t cmd_set_host_profile(const uint8_t *p, uint32_t n)
{
    if (n != LAT_PC_HOST_PROFILE_LEN)
    {
        return LAT_PC_ST_BAD_LENGTH;
    }
    if ((p[1] > 1u) || (p[2] > 1u) || (p[3] != 0u) || (p[4] > 2u) ||
        ((p[5] != 0u) && (p[5] != 2u)))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    s.profile[0] = 0u;
    lat_bytes_copy(&s.profile[1], &p[1], 5u);
    s.profile[6] = 0u;
    s.profile[7] = 0u;
    return LAT_PC_ST_OK;
}

static uint8_t cmd_set_vendor_table(const uint8_t *p, uint32_t n)
{
    uint32_t count;
    uint32_t i;
    uint32_t j;

    if ((n < 2u) || (n != (2u + (LAT_PC_VENDOR_ENTRY_LEN * (uint32_t)p[0]))))
    {
        return LAT_PC_ST_BAD_LENGTH;
    }
    count = p[0];
    if (count > LAT_PC_VENDOR_TABLE_MAX)
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    /* Validate all entries before replacing the table. */
    for (i = 0u; i < count; i++)
    {
        const uint8_t *e = &p[2u + (i * LAT_PC_VENDOR_ENTRY_LEN)];

        if ((e[6] != LAT_PC_VENDOR_STATE_CODE) && (e[6] != LAT_PC_VENDOR_STATE_NONE))
        {
            return LAT_PC_ST_BAD_PARAM;
        }
        for (j = 0u; j < i; j++)
        {
            const uint8_t *f = &p[2u + (j * LAT_PC_VENDOR_ENTRY_LEN)];

            if (lat_bytes_equal(e, f, 6u))
            {
                return LAT_PC_ST_BAD_PARAM; /* Duplicate (vid, pid, bcd_device) */
            }
        }
    }
    lat_bytes_copy(s.vendor, &p[2], count * LAT_PC_VENDOR_ENTRY_LEN);
    s.vendor_count = (uint8_t)count;
    return LAT_PC_ST_OK;
}

static uint8_t cmd_trigger_manual(const uint8_t *p, uint32_t n)
{
    uint8_t assert_mask;
    uint8_t release_mask;

    if (n != LAT_PC_TRIGGER_MANUAL_LEN)
    {
        return LAT_PC_ST_BAD_LENGTH;
    }
    if (trial_running() || poll_running() || calibration_active())
    {
        return LAT_PC_ST_BUSY;
    }
    assert_mask = p[0];
    release_mask = p[1];
    if ((assert_mask & release_mask) != 0u)
    {
        return LAT_PC_ST_BAD_PARAM; /* Cannot assert and release the same pin */
    }
    v5f_port_trigger_write(release_mask, assert_mask);
    s.trigger_asserted = (uint8_t)((s.trigger_asserted & (uint8_t)~release_mask) | assert_mask);
    return LAT_PC_ST_OK;
}

static uint8_t cmd_set_watch(const uint8_t *p, uint32_t n)
{
    uint16_t min_length;
    uint16_t bit_offset;
    uint8_t bit_size;

    if (n != LAT_PC_SET_WATCH_LEN)
    {
        return LAT_PC_ST_BAD_LENGTH;
    }
    if (trial_running())
    {
        return LAT_PC_ST_BUSY;
    }
    min_length = lat_get_u16(&p[4]);
    bit_offset = lat_get_u16(&p[8]);
    bit_size = p[10];
    if ((p[2] > 1u) || (p[6] > 1u) || (bit_size < 1u) || (bit_size > 32u) || (min_length > 1024u) ||
        (((uint32_t)bit_offset + bit_size) > ((uint32_t)min_length * 8u)))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    if (p[6] == 0u)
    {
        if ((p[12] < 1u) || (p[12] > 4u))
        {
            return LAT_PC_ST_BAD_PARAM;
        }
    }
    else
    {
        int8_t direction = (int8_t)p[20];

        if ((direction < -1) || (direction > 1))
        {
            return LAT_PC_ST_BAD_PARAM;
        }
    }
    lat_bytes_copy(s.watch, p, LAT_PC_SET_WATCH_LEN);
    s.watch_set = true;
    return LAT_PC_ST_OK;
}

/* Validate run_id for TRIAL_START / POLL_TEST_START. */
static bool run_id_valid(uint32_t run_id)
{
    return (run_id != 0u) && (run_id > s.last_run_id);
}

/*
 * Busy if a run or non-zero calibration mode is active, or terminal slot is full.
 */
static uint8_t run_busy(bool is_trial)
{
    if (trial_running() || poll_running() || calibration_active())
    {
        return LAT_PC_ST_BUSY;
    }
    if (term_busy(is_trial))
    {
        return LAT_PC_ST_BUSY;
    }
    return LAT_PC_ST_OK;
}

/*
 * Check order: length -> run_id -> busy -> param range -> overflow -> watch -> device.
 * Updates s.last_run_id on acceptance.
 */
static uint8_t cmd_trial_start(const uint8_t *p, uint32_t n)
{
    uint8_t st;
    uint16_t count;

    if (n != LAT_PC_TRIAL_START_LEN)
    {
        return LAT_PC_ST_BAD_LENGTH;
    }
    if (!run_id_valid(lat_get_u32(&p[0])))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    st = run_busy(true);
    if (st != LAT_PC_ST_OK)
    {
        return st;
    }
    /* Parameter ranges (inter_trial_ms=0 is valid). */
    count = lat_get_u16(&p[8]);
    /* Cap random_max_us so scheduled triggers stay within the <21s TIM9 window. */
    if ((p[4] == 0u) || (p[5] > 1u) || (p[6] < 1u) || (p[6] > 16u) || (count < 1u) || (count > 10000u) ||
        (lat_get_u16(&p[10]) == 0u) || (lat_get_u16(&p[12]) == 0u) || (lat_get_u32(&p[16]) > MEAS_RANDOM_MAX_US))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    if (s.overflow)
    {
        return LAT_PC_ST_NOT_READY; /* Reject new runs while in overflow */
    }
    if (!s.watch_set)
    {
        return LAT_PC_ST_NOT_READY;
    }
    /* Device lookup (NO_DEVICE) and execution are handled by measure_poll(). */
    st = measure_trial_start(p, s.watch, s.calib);
    if (st == LAT_PC_ST_OK)
    {
        s.last_run_id = lat_get_u32(&p[0]);
    }
    return st;
}

static uint8_t cmd_poll_test_start(const uint8_t *p, uint32_t n)
{
    uint8_t st;
    uint32_t duration;

    if (n != LAT_PC_POLL_TEST_START_LEN)
    {
        return LAT_PC_ST_BAD_LENGTH;
    }
    if (!run_id_valid(lat_get_u32(&p[0])))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    st = run_busy(false);
    if (st != LAT_PC_ST_OK)
    {
        return st;
    }
    duration = lat_get_u32(&p[8]);
    if ((duration < 1000u) || (duration > 120000u) || (lat_get_u32(&p[12]) > LAT_PC_MAX_POLL_RAW_COUNT))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    if (s.overflow)
    {
        return LAT_PC_ST_NOT_READY;
    }
    st = measure_poll_start(p);
    if (st == LAT_PC_ST_OK)
    {
        s.last_run_id = lat_get_u32(&p[0]);
    }
    return st;
}

static void handle_pc(void)
{
    const uint8_t *e = s_cmd.payload;
    const uint8_t *p;
    uint32_t n;
    uint8_t type;
    uint16_t seq;
    uint8_t st = LAT_PC_ST_OK;
    uint32_t len = 0u;
    uint8_t *r = out_body();

    if (s_cmd.len < LAT_IPC_ENV_LEN)
    {
        s.cmd_stale++;
        return;
    }
    if ((s.session == 0u) || (lat_get_u32(&e[LAT_IPC_ENV_OFF_SESSION]) != s.session))
    {
        s.cmd_stale++; /* Ignore commands from previous session */
        return;
    }
    type = e[LAT_IPC_ENV_OFF_TYPE];
    seq = lat_get_u16(&e[LAT_IPC_ENV_OFF_SEQ]);
    p = &e[LAT_IPC_ENV_LEN];
    n = (uint32_t)s_cmd.len - LAT_IPC_ENV_LEN;

    switch (type)
    {
    case LAT_PC_GET_DEVICE_INFO:
        st = (n != 0u) ? LAT_PC_ST_BAD_LENGTH : v5f_port_device_info(r, LAT_IPC_ENV_BODY_MAX, &len);
        break;
    case LAT_PC_GET_OBJECT:
        st = cmd_get_object(p, n, &len);
        break;
    case LAT_PC_SET_HOST_PROFILE:
        st = cmd_set_host_profile(p, n);
        break;
    case LAT_PC_GET_HOST_PROFILE:
        if (n != 0u)
        {
            st = LAT_PC_ST_BAD_LENGTH;
            break;
        }
        lat_bytes_copy(r, s.profile, LAT_PC_HOST_PROFILE_LEN);
        len = LAT_PC_HOST_PROFILE_LEN;
        break;
    case LAT_PC_SET_VENDOR_TABLE:
        st = cmd_set_vendor_table(p, n);
        break;
    case LAT_PC_GET_VENDOR_TABLE:
        if (n != 0u)
        {
            st = LAT_PC_ST_BAD_LENGTH;
            break;
        }
        r[0] = s.vendor_count;
        r[1] = 0u;
        lat_bytes_copy(&r[2], s.vendor, (uint32_t)s.vendor_count * LAT_PC_VENDOR_ENTRY_LEN);
        len = 2u + ((uint32_t)s.vendor_count * LAT_PC_VENDOR_ENTRY_LEN);
        break;
    case LAT_PC_REENUMERATE:
        if (n != 0u)
        {
            st = LAT_PC_ST_BAD_LENGTH;
        }
        else if (trial_running() || poll_running())
        {
            st = LAT_PC_ST_BUSY;
        }
        else
        {
            st = v5f_port_reenumerate(); /* Invalidates staged object on acceptance */
            if (st == LAT_PC_ST_OK)
            {
                object_invalidate();
            }
        }
        break;
    case LAT_PC_TRIGGER_MANUAL:
        st = cmd_trigger_manual(p, n);
        break;
    case LAT_PC_MONITOR_START:
        st = (n != LAT_PC_MONITOR_START_LEN) ? LAT_PC_ST_BAD_LENGTH : measure_monitor_start(p);
        break;
    case LAT_PC_MONITOR_STOP:
        st = (n != 0u) ? LAT_PC_ST_BAD_LENGTH : LAT_PC_ST_OK;
        if (st == LAT_PC_ST_OK)
        {
            measure_monitor_stop();
        }
        break;
    case LAT_PC_SET_WATCH:
        st = cmd_set_watch(p, n);
        break;
    case LAT_PC_TRIAL_START:
        st = cmd_trial_start(p, n);
        break;
    case LAT_PC_TRIAL_STOP:
        /* Mismatched run_id returns BAD_PARAM. */
        st = (n != LAT_PC_RUN_STOP_LEN) ? LAT_PC_ST_BAD_LENGTH : measure_trial_stop(lat_get_u32(&p[0]));
        break;
    case LAT_PC_POLL_TEST_STOP:
        st = (n != LAT_PC_RUN_STOP_LEN) ? LAT_PC_ST_BAD_LENGTH : measure_poll_stop(lat_get_u32(&p[0]));
        break;
    case LAT_PC_POLL_TEST_START:
        st = cmd_poll_test_start(p, n);
        break;
    case LAT_PC_SET_CALIBRATION:
        if (n != LAT_PC_CALIBRATION_LEN)
        {
            st = LAT_PC_ST_BAD_LENGTH;
        }
        else if (trial_running())
        {
            st = LAT_PC_ST_BUSY;
        }
        else
        {
            lat_bytes_copy(s.calib, p, 24u);
            s.calib[24] = (uint8_t)(p[24] & 0x07u);
            s.calib[25] = 0u;
            s.calib[26] = 0u;
            s.calib[27] = 0u;
        }
        break;
    case LAT_PC_GET_CALIBRATION:
        if (n != 0u)
        {
            st = LAT_PC_ST_BAD_LENGTH;
            break;
        }
        lat_bytes_copy(r, s.calib, LAT_PC_CALIBRATION_LEN);
        len = LAT_PC_CALIBRATION_LEN;
        break;
    case LAT_PC_CALIBRATION_MODE:
        if (n != LAT_PC_CALIBRATION_MODE_LEN)
        {
            st = LAT_PC_ST_BAD_LENGTH;
        }
        else if (p[0] == 0u)
        {
            st = measure_calib_mode(p, (const uint8_t *)0); /* Disable is always accepted */
        }
        else if (p[0] > 2u)
        {
            st = LAT_PC_ST_BAD_PARAM;
        }
        else if (trial_running() || poll_running())
        {
            st = LAT_PC_ST_BUSY;
        }
        else
        {
            st = measure_calib_mode(p, s.watch_set ? s.watch : (const uint8_t *)0);
        }
        break;
    default:
        st = LAT_PC_ST_UNKNOWN_COMMAND;
        break;
    }
    respond(type, seq, st, len);
}

void v5f_cmd_service(void)
{
    while (lat_ring_used_units(&g_ring_cmd) != 0u)
    {
        /* Defer command if safe ring lacks space for a max-size response. */
        if (!lat_ring_can_push(&g_ring_evt_safe, LAT_RING_PAYLOAD_MAX))
        {
            s.cmd_deferred++;
            return;
        }
        if (!lat_ring_pop(&g_ring_cmd, &s_cmd))
        {
            return;
        }
        s.cmd_count++;
        switch (s_cmd.type)
        {
        case LAT_CMD_PING:
            v5f_port_ping(&s_cmd);
            break;
        case LAT_CMD_PC:
            handle_pc();
            break;
        default:
            break;
        }
    }
}

uint32_t v5f_cmd_session(void)
{
    return s.session;
}

/* Enumeration helpers (v5f_cmd.h) */

const uint8_t *v5f_cmd_host_profile(void)
{
    return s.profile;
}

bool v5f_cmd_vendor_lookup(uint16_t vid, uint16_t pid, uint16_t bcd_device, uint8_t *state, uint8_t *code)
{
    uint32_t i;

    for (i = 0u; i < s.vendor_count; i++)
    {
        const uint8_t *e = &s.vendor[i * LAT_PC_VENDOR_ENTRY_LEN];

        if ((lat_get_u16(&e[0]) == vid) && (lat_get_u16(&e[2]) == pid) && (lat_get_u16(&e[4]) == bcd_device))
        {
            *state = e[6];
            *code = e[7];
            return true;
        }
    }
    return false;
}

void v5f_cmd_vendor_learn(uint16_t vid, uint16_t pid, uint16_t bcd_device, uint8_t state, uint8_t code)
{
    uint32_t i;
    uint8_t *e = (uint8_t *)0;

    for (i = 0u; i < s.vendor_count; i++)
    {
        uint8_t *f = &s.vendor[i * LAT_PC_VENDOR_ENTRY_LEN];

        if ((lat_get_u16(&f[0]) == vid) && (lat_get_u16(&f[2]) == pid) && (lat_get_u16(&f[4]) == bcd_device))
        {
            e = f;
            break;
        }
    }
    if (e == (uint8_t *)0)
    {
        if (s.vendor_count >= LAT_PC_VENDOR_TABLE_MAX)
        {
            return; /* Table full; drop learned entry */
        }
        e = &s.vendor[(uint32_t)s.vendor_count * LAT_PC_VENDOR_ENTRY_LEN];
        s.vendor_count++;
    }
    lat_put_u16(&e[0], vid);
    lat_put_u16(&e[2], pid);
    lat_put_u16(&e[4], bcd_device);
    e[6] = state;
    e[7] = (state == LAT_PC_VENDOR_STATE_CODE) ? code : 0u;
}

uint32_t v5f_cmd_count(void)
{
    return s.cmd_count;
}

uint32_t v5f_cmd_deferred(void)
{
    return s.cmd_deferred;
}

void v5f_cmd_init(void)
{
    lat_bytes_fill((uint8_t *)&s, 0u, (uint32_t)sizeof(s));
    lat_crc32_table_init(s_crc_table);
    set_defaults();
    release_all_triggers();
    object_invalidate();
    s_isr_head = 0u;
    s_isr_tail = 0u;
    v5f_cmd_publish();
}
