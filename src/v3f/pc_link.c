/*
 * pc_link.c - V3F side of the PC communication protocol (see pc_link.h).
 */
#include "pc_link.h"

#include "lat_bytes.h"
#include "lat_cobs.h"
#include "lat_crc16.h"
#include "lat_ipc.h"
#include "lat_pc_proto.h"
#include "lat_version.h"
#include "pc_link_port.h"

/* ---- Configuration ---- */
#define PC_CRIT_BYTES          (8192u)  /* critical event TX queue (power of two) */
#define PC_LOSSY_BYTES         (4096u)  /* lossy event TX queue (power of two) */
#define PC_FWD_TIMEOUT_MS      (2000u)  /* V5F command forward timeout */
#define PC_SESSION_TIMEOUT_MS  (500u)   /* V5F session stop timeout */
#define PC_QUIESCE_TIMEOUT_MS  (200u)   /* V5F quiesce timeout */
#define PC_BOOT_TX_TIMEOUT_MS  (1000u)  /* ENTER_BOOTLOADER TX completion timeout */
#define PC_DROP_REPORT_MS      (100u)   /* minimum interval between DROP_COUNTERS */
#define PC_PUMP_MAX            (16u)    /* max IPC records drained per poll */
#define PC_MARK_MAX            (16u)
#define PC_SEQLOCK_TRIES       (16u)
#define PC_QREC_HDR            (4u)     /* queue record header: type, flags, len(u16) */
#define PC_TERM_BODY_MAX       (96u)

_Static_assert((PC_CRIT_BYTES & (PC_CRIT_BYTES - 1u)) == 0u, "PC_CRIT_BYTES must be a power of two");
_Static_assert((PC_LOSSY_BYTES & (PC_LOSSY_BYTES - 1u)) == 0u, "PC_LOSSY_BYTES must be a power of two");
_Static_assert(LAT_PC_POLL_RESULT_LEN <= PC_TERM_BODY_MAX, "terminal slot too small");
_Static_assert(LAT_COBS_MAX_ENCODED(LAT_PC_DECODED_MAX) == LAT_PC_ENCODED_MAX, "COBS bound mismatch");

typedef enum RespKind
{
    RESP_NONE = 0,
    RESP_LAST,     /* cached response for last command (last_*) */
    RESP_ADHOC,    /* bodyless response (SEQ_CONFLICT, HELLO BAD_LENGTH, etc.) */
    RESP_HELLO     /* current session HELLO response */
} RespKind;

typedef enum MarkKind
{
    MARK_NONE = 0,
    MARK_TERM_RUN,
    MARK_TERM_POLL,
    MARK_CRIT,        /* normal critical event (keeps ENTER_BOOTLOADER BUSY until TX completes) */
    MARK_BOOT
} MarkKind;

typedef enum TermState
{
    TERM_FREE = 0,
    TERM_WAITING,
    TERM_SENDING
} TermState;

typedef enum BootPhase
{
    BOOT_IDLE = 0,
    BOOT_QUIESCE,
    BOOT_SENDING,
    BOOT_DONE
} BootPhase;

typedef struct PcQueue
{
    uint8_t *buf;
    uint32_t size;   /* power of two */
    uint32_t head;   /* total bytes written */
    uint32_t tail;   /* total bytes read */
    uint32_t count;  /* record count */
} PcQueue;

typedef struct TermSlot
{
    TermState state;
    uint32_t pos;     /* position in critical queue before which to send */
    uint32_t order;
    uint16_t len;
    uint8_t body[PC_TERM_BODY_MAX];
} TermSlot;

typedef struct TxMark
{
    uint32_t end;
    MarkKind kind;
} TxMark;

typedef struct PcLinkState
{
    uint32_t now_ms;

    /* RX */
    uint8_t rx_buf[LAT_PC_ENCODED_MAX];
    uint32_t rx_len;
    bool rx_skip;
    uint8_t rx_dec[LAT_PC_DECODED_MAX];

    /* Session */
    uint32_t session;
    bool hello_pending;
    bool hello_timed_out;    /* V5F did not ack in time; returned INTERNAL */
    bool hello_unconfirmed;  /* established after timeout; PC does not know session_id yet */
    uint32_t hello_deadline;
    bool hello_valid;
    uint64_t hello_nonce;
    uint16_t hello_seq;
    uint8_t hello_req[LAT_PC_HELLO_REQ_LEN];
    uint8_t hello_resp[LAT_PC_HELLO_RESP_LEN];
    uint16_t evt_seq;

    /* Last command and response */
    bool last_valid;
    bool last_resp_ready;
    uint8_t last_type;
    uint16_t last_seq;
    uint16_t last_len;
    uint8_t last_payload[LAT_PC_PAYLOAD_MAX];
    uint8_t last_status;
    uint16_t last_resp_len;
    uint8_t last_resp[LAT_PC_PAYLOAD_MAX];

    /* Pending response (1 slot) */
    RespKind resp_kind;
    uint8_t adhoc_type;
    uint8_t adhoc_status;
    uint16_t adhoc_seq;
    uint32_t adhoc_session;
    bool resp_boot_mark;

    /* Command forwarded to V5F */
    bool fwd_active;
    uint8_t fwd_type;
    uint16_t fwd_seq;
    uint32_t fwd_deadline;

    /* TX queues */
    PcQueue crit;
    PcQueue lossy;
    uint32_t crit_enq;
    uint32_t crit_deq;
    TermSlot term[2];
    uint32_t term_order;
    bool staged;
    uint8_t staged_type;
    uint16_t staged_len;
    uint8_t staged_body[LAT_IPC_ENV_BODY_MAX];
    bool blocked;
    uint32_t blocked_since;
    bool overflow;
    bool ovf_notify;

    /* DROP_COUNTERS */
    uint32_t base_mon;
    uint32_t base_log;
    uint32_t v3f_drop_mon;
    uint32_t v3f_drop_log;
    uint32_t rep_mon;
    uint32_t rep_log;
    uint32_t drop_last_ms;
    bool drop_pending;

    /* TX */
    uint8_t tx_raw[LAT_PC_DECODED_MAX];
    uint8_t tx_enc[LAT_PC_WIRE_MAX];
    uint32_t tx_len;
    uint32_t tx_off;
    MarkKind tx_mark;
    uint32_t tx_stream;
    uint32_t tx_acked;
    TxMark marks[PC_MARK_MAX];
    uint32_t crit_inflight;  /* critical events popped from queue, awaiting USB TX completion */
    uint32_t v5f_ovf_seen;   /* last acknowledged v5f_ovf_gen */
    uint32_t mark_count;

    /* Bootloader handoff */
    BootPhase boot;
    uint32_t boot_deadline;
    bool boot_resp_done;

    PcLinkStats stats;
} PcLinkState;

static PcLinkState s;
static uint8_t s_crit_buf[PC_CRIT_BYTES];
static uint8_t s_lossy_buf[PC_LOSSY_BYTES];
/* Scratch buffer for IPC drain and V5F forwarding (1160 bytes). */
static LatRingRecord s_rec;

static bool time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

/* ------------------------------------------------------------------ */
/* Variable-length record FIFO                                          */
/* ------------------------------------------------------------------ */

static void q_init(PcQueue *q, uint8_t *buf, uint32_t size)
{
    q->buf = buf;
    q->size = size;
    q->head = 0u;
    q->tail = 0u;
    q->count = 0u;
}

static void q_clear(PcQueue *q)
{
    q->head = 0u;
    q->tail = 0u;
    q->count = 0u;
}

static void q_put_byte(PcQueue *q, uint32_t pos, uint8_t b)
{
    q->buf[pos & (q->size - 1u)] = b;
}

static uint8_t q_get_byte(const PcQueue *q, uint32_t pos)
{
    return q->buf[pos & (q->size - 1u)];
}

static bool q_push(PcQueue *q, uint8_t type, uint8_t flags, const uint8_t *body, uint32_t len)
{
    uint32_t need = PC_QREC_HDR + len;
    uint32_t i;

    if ((len > 0xFFFFu) || (need > (q->size - (q->head - q->tail))))
    {
        return false;
    }
    q_put_byte(q, q->head + 0u, type);
    q_put_byte(q, q->head + 1u, flags);
    q_put_byte(q, q->head + 2u, (uint8_t)len);
    q_put_byte(q, q->head + 3u, (uint8_t)(len >> 8));
    for (i = 0u; i < len; i++)
    {
        q_put_byte(q, q->head + PC_QREC_HDR + i, body[i]);
    }
    q->head += need;
    q->count++;
    return true;
}

/* Pop the front record. body must hold at least LAT_PC_PAYLOAD_MAX bytes. */
static bool q_pop(PcQueue *q, uint8_t *type, uint8_t *body, uint16_t *len)
{
    uint32_t n;
    uint32_t i;

    if (q->count == 0u)
    {
        return false;
    }
    *type = q_get_byte(q, q->tail + 0u);
    n = (uint32_t)q_get_byte(q, q->tail + 2u) | ((uint32_t)q_get_byte(q, q->tail + 3u) << 8);
    if (n > LAT_PC_PAYLOAD_MAX)
    {
        q_clear(q); /* corrupted (unreachable) */
        return false;
    }
    for (i = 0u; i < n; i++)
    {
        body[i] = q_get_byte(q, q->tail + PC_QREC_HDR + i);
    }
    *len = (uint16_t)n;
    q->tail += PC_QREC_HDR + n;
    q->count--;
    return true;
}

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static void compute_drops(uint32_t *mon, uint32_t *log)
{
    const LatRingHead *h = g_ring_evt_lossy.head;

    *mon = (lat_ipc_word_load(&h->lost_by_type[LAT_EVT_MONITOR]) - s.base_mon) + s.v3f_drop_mon;
    *log = (lat_ipc_word_load(&h->lost_by_type[LAT_EVT_PC_LOG]) - s.base_log) + s.v3f_drop_log;
}

static void drop_baseline(void)
{
    const LatRingHead *h = g_ring_evt_lossy.head;

    s.base_mon = lat_ipc_word_load(&h->lost_by_type[LAT_EVT_MONITOR]);
    s.base_log = lat_ipc_word_load(&h->lost_by_type[LAT_EVT_PC_LOG]);
    s.v3f_drop_mon = 0u;
    s.v3f_drop_log = 0u;
    s.rep_mon = 0u;
    s.rep_log = 0u;
    s.drop_pending = false;
    s.drop_last_ms = s.now_ms;
}

static bool term_is_run(uint8_t pc_type)
{
    return pc_type == LAT_PC_EVT_RUN_DONE;
}

static bool term_kind(uint8_t pc_type)
{
    return (pc_type == LAT_PC_EVT_RUN_DONE) || (pc_type == LAT_PC_EVT_POLL_RESULT);
}

/* Notify V5F that a terminal event hold slot was freed (sent or dropped). */
static void term_release(bool run)
{
    uint32_t *field = run ? &g_ipc_header.term.run_done_released : &g_ipc_header.term.poll_result_released;

    lat_ipc_store_release(field, lat_ipc_word_load(field) + 1u);
}

/* ------------------------------------------------------------------ */
/* Responses                                                            */
/* ------------------------------------------------------------------ */

/* Commit response for the last command; non-OK responses have zero body length. */
static void finish(uint8_t status, const uint8_t *body, uint32_t len)
{
    if (status != LAT_PC_ST_OK)
    {
        len = 0u;
    }
    if (len > LAT_PC_PAYLOAD_MAX)
    {
        status = LAT_PC_ST_INTERNAL;
        len = 0u;
    }
    if ((len != 0u) && (body != s.last_resp))
    {
        lat_bytes_copy(s.last_resp, body, len);
    }
    s.last_status = status;
    s.last_resp_len = (uint16_t)len;
    s.last_resp_ready = true;
    s.resp_kind = RESP_LAST;
}

/* Bodyless response using current session_id (0 before session established). */
static void respond_adhoc(uint8_t type, uint16_t seq, uint8_t status)
{
    s.adhoc_type = type;
    s.adhoc_seq = seq;
    s.adhoc_status = status;
    s.adhoc_session = s.hello_pending ? 0u : s.session;
    s.resp_kind = RESP_ADHOC;
}

/* ------------------------------------------------------------------ */
/* Session management                                                   */
/* ------------------------------------------------------------------ */

static void discard_tx_queues(void)
{
    uint32_t i;

    s.resp_kind = RESP_NONE;
    s.resp_boot_mark = false;
    q_clear(&s.crit);
    q_clear(&s.lossy);
    s.crit_enq = 0u;
    s.crit_deq = 0u;
    s.term[0].state = TERM_FREE;
    s.term[1].state = TERM_FREE;
    s.staged = false;
    s.blocked = false;
    /* Finish any in-flight TX frame (PC drops it as wrong session).
     * Invalidate terminal marks so hold counts stay aligned with V5F. */
    if ((s.tx_mark == MARK_TERM_RUN) || (s.tx_mark == MARK_TERM_POLL) || (s.tx_mark == MARK_CRIT))
    {
        s.tx_mark = MARK_NONE;
    }
    for (i = 0u; i < s.mark_count; i++)
    {
        if ((s.marks[i].kind == MARK_TERM_RUN) || (s.marks[i].kind == MARK_TERM_POLL) ||
            (s.marks[i].kind == MARK_CRIT))
        {
            s.marks[i].kind = MARK_NONE;
        }
    }
    s.crit_inflight = 0u;
}

static uint32_t new_session_id(void)
{
    uint32_t id;
    uint32_t tries;

    for (tries = 0u; tries < 64u; tries++)
    {
        id = pc_port_random();
        if ((id != 0u) && (id != s.session))
        {
            return id;
        }
    }
    return s.session + 1u + ((s.session + 1u == 0u) ? 1u : 0u);
}

static void build_hello_response(void)
{
    uint8_t *r = s.hello_resp;
    uint32_t caps = pc_port_v5f_running() ? lat_ipc_word_load(&g_ipc_header.v5f_capabilities) : 0u;

    lat_bytes_fill(r, 0u, LAT_PC_HELLO_RESP_LEN);
    lat_put_u32(&r[0], LAT_PC_HELLO_MAGIC);
    r[4] = LAT_PC_VERSION;
    lat_put_u16(&r[6], (uint16_t)LAT_FW_VERSION_MAJOR);
    lat_put_u16(&r[8], (uint16_t)LAT_FW_VERSION_MINOR);
    pc_port_build(&r[10]);
    pc_port_uuid(&r[22]);
    lat_put_u32(&r[38], caps);
    lat_put_u16(&r[42], (uint16_t)LAT_PC_PAYLOAD_MAX);
    lat_put_u32(&r[46], LAT_PC_MAX_POLL_RAW_COUNT);
    lat_put_u64(&r[50], s.hello_nonce);
    lat_put_u32(&r[58], s.session);
}

static void start_new_session(uint64_t nonce, uint16_t seq, const uint8_t *req)
{
    uint32_t q = lat_ipc_word_load(&g_ipc_header.session.quiesce_req);

    s.session = new_session_id();
    s.hello_pending = true;
    s.hello_timed_out = false;
    s.hello_unconfirmed = false;
    s.hello_deadline = s.now_ms + PC_SESSION_TIMEOUT_MS;
    s.hello_valid = true;
    s.hello_nonce = nonce;
    s.hello_seq = seq;
    lat_bytes_copy(s.hello_req, req, LAT_PC_HELLO_REQ_LEN);
    s.evt_seq = 0u;
    s.last_valid = false;
    s.last_resp_ready = false;
    s.fwd_active = false;
    discard_tx_queues();
    if (s.overflow)
    {
        s.overflow = false;
        s.ovf_notify = true;
    }
    /* RX accumulation is already empty when processing this frame. */
    pc_port_release_triggers();
    /* Return quiesce control to even (released) before requesting session stop. */
    if ((q & 1u) != 0u)
    {
        lat_ipc_store_release(&g_ipc_header.session.quiesce_req, q + 1u);
    }
    lat_ipc_store_release(&g_ipc_header.session.session_req, s.session);
}

static void session_poll(void)
{
    uint32_t ack;

    if (!s.hello_pending)
    {
        return;
    }
    ack = lat_ipc_load_acquire(&g_ipc_header.session.session_ack);
    /*
     * Wait for V5F to ack session stop before replying (unless V5F is stopped).
     * On timeout, return INTERNAL without establishing a session and keep waiting
     * for V5F ack; PC's HELLO retry will then succeed.
     */
    if ((ack != s.session) && pc_port_v5f_running())
    {
        if (!s.hello_timed_out && time_reached(s.now_ms, s.hello_deadline))
        {
            s.hello_timed_out = true;
            respond_adhoc(LAT_PC_HELLO, s.hello_seq, LAT_PC_ST_INTERNAL);
            pc_port_log("[v3f] pc: v5f did not acknowledge the session stop within 500ms - HELLO INTERNAL");
        }
        return;
    }
    /* If V5F entered overflow before V3F, report it after this response. */
    {
        uint32_t gen = lat_ipc_load_acquire(&g_ipc_header.session.v5f_ovf_gen);

        if (gen != s.v5f_ovf_seen)
        {
            s.v5f_ovf_seen = gen;
            s.ovf_notify = true;
        }
    }
    /* V5F discarded its hold slots on stop; sync local hold count. */
    lat_ipc_store_release(&g_ipc_header.term.run_done_released,
                          lat_ipc_load_acquire(&g_ipc_header.term.run_done_produced));
    lat_ipc_store_release(&g_ipc_header.term.poll_result_released,
                          lat_ipc_load_acquire(&g_ipc_header.term.poll_result_produced));
    drop_baseline();
    build_hello_response();
    s.hello_pending = false;
    if (s.hello_timed_out)
    {
        /*
         * Already replied with INTERNAL; establish session silently and wait
         * for PC to retry HELLO before sending events.
         */
        s.hello_unconfirmed = true;
    }
    else
    {
        s.resp_kind = RESP_HELLO;
    }

    if (s.ovf_notify)
    {
        uint8_t body[LAT_PC_ERROR_LEN];

        s.ovf_notify = false;
        lat_bytes_fill(body, 0u, sizeof(body));
        lat_put_u16(&body[0], LAT_PC_ERR_EVENT_OVERFLOW);
        if (q_push(&s.crit, LAT_PC_EVT_ERROR, 0u, body, sizeof(body)))
        {
            s.crit_enq++;
        }
    }
}

static void handle_hello(uint16_t seq, uint32_t session_hdr, const uint8_t *p, uint32_t n)
{
    uint64_t nonce;

    if (session_hdr != 0u)
    {
        s.stats.rx_dropped++;
        return;
    }
    if (n != LAT_PC_HELLO_REQ_LEN)
    {
        respond_adhoc(LAT_PC_HELLO, seq, LAT_PC_ST_BAD_LENGTH);
        return;
    }
    nonce = lat_get_u64(&p[4]);
    if ((s.session != 0u) && s.hello_valid && (nonce == s.hello_nonce))
    {
        if ((seq == s.hello_seq) && lat_bytes_equal(p, s.hello_req, LAT_PC_HELLO_REQ_LEN))
        {
            if (!s.hello_pending)
            {
                s.resp_kind = RESP_HELLO; /* resend previous HELLO response */
            }
            else if (s.hello_timed_out)
            {
                /* Still waiting for V5F ack (session_poll will reply on arrival). */
                respond_adhoc(LAT_PC_HELLO, seq, LAT_PC_ST_INTERNAL);
            }
            return;
        }
        respond_adhoc(LAT_PC_HELLO, seq, LAT_PC_ST_SEQ_CONFLICT);
        return;
    }
    if (p[0] != LAT_PC_VERSION)
    {
        respond_adhoc(LAT_PC_HELLO, seq, LAT_PC_ST_BAD_PARAM);
        return;
    }
    start_new_session(nonce, seq, p);
}

/* ------------------------------------------------------------------ */
/* V3F-handled commands                                                 */
/* ------------------------------------------------------------------ */

static void cmd_get_status(uint32_t n)
{
    uint8_t *r = s.last_resp;
    LatHostStatus h;
    uint64_t now = 0u;
    uint32_t mon;
    uint32_t log;

    if (n != 0u)
    {
        finish(LAT_PC_ST_BAD_LENGTH, 0, 0u);
        return;
    }
    lat_bytes_fill((uint8_t *)&h, 0u, (uint32_t)sizeof(h));
    if (pc_port_v5f_running())
    {
        if (!lat_ipc_host_read(&h, PC_SEQLOCK_TRIES) || !lat_ipc_time_read(&now, PC_SEQLOCK_TRIES))
        {
            finish(LAT_PC_ST_INTERNAL, 0, 0u);
            return;
        }
    }
    else
    {
        h.host_state = (pc_port_v5f_diag() != 0u) ? LAT_PC_HOST_ERROR_HALT : LAT_PC_HOST_BOOTING;
    }
    compute_drops(&mon, &log);

    lat_bytes_fill(r, 0u, LAT_PC_GET_STATUS_RESP_LEN);
    r[0] = h.host_state;
    r[1] = h.speed;
    lat_put_u32(&r[4], h.device_generation);
    lat_put_u32(&r[8], h.enum_error);
    lat_put_u32(&r[12], h.sched_late_count);
    lat_put_u32(&r[16], h.sched_late_max);
    lat_put_u32(&r[20], h.txn_max_ticks);
    lat_put_u32(&r[24], h.recover_count);
    lat_put_u32(&r[28], mon);
    lat_put_u32(&r[32], log);
    lat_put_u64(&r[36], now);
    lat_put_u32(&r[44], h.active_run_id);
    r[48] = h.run_kind;
    finish(LAT_PC_ST_OK, r, LAT_PC_GET_STATUS_RESP_LEN);
}

static void cmd_read_object(const uint8_t *p, uint32_t n)
{
    const LatObjectMeta *m = &g_ipc_header.object;
    uint8_t *r = s.last_resp;
    uint16_t object_id;
    uint16_t max_length;
    uint32_t offset;
    uint32_t total;
    uint32_t crc;
    uint32_t dl;

    if (n != LAT_PC_READ_OBJECT_REQ_LEN)
    {
        finish(LAT_PC_ST_BAD_LENGTH, 0, 0u);
        return;
    }
    object_id = lat_get_u16(&p[0]);
    max_length = lat_get_u16(&p[2]);
    offset = lat_get_u32(&p[4]);
    if ((max_length == 0u) || (max_length > LAT_PC_READ_OBJECT_MAX))
    {
        finish(LAT_PC_ST_BAD_PARAM, 0, 0u);
        return;
    }
    if ((lat_ipc_load_acquire(&m->state) != LAT_OBJ_STATE_READY) || (m->object_id != object_id) ||
        (m->object_id == 0u) || (lat_ipc_word_load(&m->session) != s.session))
    {
        finish(LAT_PC_ST_STALE_OBJECT, 0, 0u);
        return;
    }
    total = lat_ipc_word_load(&m->length);
    crc = lat_ipc_word_load(&m->crc32);
    if (total > LAT_OBJECT_BUF_BYTES)
    {
        finish(LAT_PC_ST_INTERNAL, 0, 0u);
        return;
    }
    if (offset > total)
    {
        finish(LAT_PC_ST_BAD_PARAM, 0, 0u);
        return;
    }
    dl = total - offset;
    if (dl > max_length)
    {
        dl = max_length;
    }
    lat_bytes_copy(&r[LAT_PC_READ_OBJECT_HDR_LEN], &g_object_buf.data[offset], dl);
    /* Verify object was not recreated while reading. */
    LAT_FENCE_RW();
    if ((lat_ipc_load_acquire(&m->state) != LAT_OBJ_STATE_READY) || (m->object_id != object_id))
    {
        finish(LAT_PC_ST_STALE_OBJECT, 0, 0u);
        return;
    }
    lat_put_u16(&r[0], object_id);
    lat_put_u16(&r[2], (uint16_t)dl);
    lat_put_u32(&r[4], offset);
    lat_put_u32(&r[8], total);
    lat_put_u32(&r[12], crc);
    finish(LAT_PC_ST_OK, r, LAT_PC_READ_OBJECT_HDR_LEN + dl);
}

static void pump_safe(void);

static bool critical_pending(void)
{
    return (s.crit.count != 0u) || s.staged || (s.crit_inflight != 0u) || (s.term[0].state != TERM_FREE) ||
           (s.term[1].state != TERM_FREE) || (lat_ring_used_units(&g_ring_evt_safe) != 0u);
}

/*
 * ENTER_BOOTLOADER is BUSY while a trial/poll is active, terminal events are
 * held, or critical events are queued (drain safe ring before checking).
 */
static bool boot_busy(void)
{
    const LatTermCounters *t = &g_ipc_header.term;

    pump_safe();
    if (pc_port_v5f_running())
    {
        LatHostStatus h;

        if (!lat_ipc_host_read(&h, PC_SEQLOCK_TRIES) || (h.run_kind == LAT_PC_RUN_TRIAL) ||
            (h.run_kind == LAT_PC_RUN_POLL))
        {
            return true;
        }
    }
    return (lat_ipc_load_acquire(&t->run_done_produced) != lat_ipc_word_load(&t->run_done_released)) ||
           (lat_ipc_load_acquire(&t->poll_result_produced) != lat_ipc_word_load(&t->poll_result_released)) ||
           critical_pending();
}

/* Quiesce control (odd = quiesce, even = release; see LatSessionCtl). */
static void quiesce_request(bool on)
{
    uint32_t q = lat_ipc_word_load(&g_ipc_header.session.quiesce_req);

    if (((q & 1u) != 0u) != on)
    {
        lat_ipc_store_release(&g_ipc_header.session.quiesce_req, q + 1u);
    }
}

static void cmd_enter_bootloader(const uint8_t *p, uint32_t n)
{
    if (n != LAT_PC_ENTER_BL_LEN)
    {
        finish(LAT_PC_ST_BAD_LENGTH, 0, 0u);
        return;
    }
    if (lat_get_u32(p) != LAT_PC_BOOT_CONFIRM)
    {
        finish(LAT_PC_ST_BAD_PARAM, 0, 0u);
        return;
    }
    if (boot_busy())
    {
        finish(LAT_PC_ST_BUSY, 0, 0u);
        return;
    }
    /* 1. Ask V5F to stop cal/monitor and stop producing events. */
    s.boot = BOOT_QUIESCE;
    s.boot_deadline = s.now_ms + PC_QUIESCE_TIMEOUT_MS;
    s.boot_resp_done = false;
    quiesce_request(true);
    pc_port_log("[v3f] pc: ENTER_BOOTLOADER accepted - quiescing v5f");
}

static void boot_poll(void)
{
    switch (s.boot)
    {
    case BOOT_QUIESCE:
    {
        uint32_t req = lat_ipc_word_load(&g_ipc_header.session.quiesce_req);
        uint32_t ack = lat_ipc_load_acquire(&g_ipc_header.session.quiesce_ack);

        if ((ack != req) && pc_port_v5f_running())
        {
            if (time_reached(s.now_ms, s.boot_deadline))
            {
                /* V5F failed to quiesce; abort handoff. */
                quiesce_request(false);
                s.boot = BOOT_IDLE;
                finish(LAT_PC_ST_INTERNAL, 0, 0u);
                pc_port_log("[v3f] pc: v5f did not acknowledge quiesce within 200ms - ENTER_BOOTLOADER INTERNAL");
            }
            return;
        }
        /* Abort if critical events were produced before quiesce took effect. */
        if (boot_busy())
        {
            quiesce_request(false);
            s.boot = BOOT_IDLE;
            finish(LAT_PC_ST_BUSY, 0, 0u);
            return;
        }
        /* 2. Release triggers. 3. Drop lossy TX queue. */
        pc_port_release_triggers();
        q_clear(&s.lossy);
        s.drop_pending = false;
        /* 4. Send response and wait up to 1s for USB TX completion. */
        s.resp_boot_mark = true;
        finish(LAT_PC_ST_OK, 0, 0u);
        s.boot = BOOT_SENDING;
        s.boot_deadline = s.now_ms + PC_BOOT_TX_TIMEOUT_MS;
        break;
    }
    case BOOT_SENDING:
        if (s.boot_resp_done || time_reached(s.now_ms, s.boot_deadline))
        {
            s.boot = BOOT_DONE;
            pc_port_log(s.boot_resp_done ? "[v3f] pc: response sent - rebooting into the bootloader"
                                         : "[v3f] pc: response not confirmed within 1s - rebooting into the bootloader");
            /* 5. Write STAY boot magic and reboot. */
            pc_port_enter_bootloader();
        }
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Forwarding to V5F                                                    */
/* ------------------------------------------------------------------ */

/* Expected request length for forwarded commands; 0xFFFE = unknown. */
static uint16_t forward_length(uint8_t type)
{
    switch (type)
    {
    case LAT_PC_GET_DEVICE_INFO:  return 0u;
    case LAT_PC_GET_OBJECT:       return LAT_PC_GET_OBJECT_REQ_LEN;
    case LAT_PC_SET_HOST_PROFILE: return LAT_PC_HOST_PROFILE_LEN;
    case LAT_PC_GET_HOST_PROFILE: return 0u;
    case LAT_PC_SET_VENDOR_TABLE: return LAT_PC_LEN_VARIABLE;
    case LAT_PC_GET_VENDOR_TABLE: return 0u;
    case LAT_PC_REENUMERATE:      return 0u;
    case LAT_PC_TRIGGER_MANUAL:   return LAT_PC_TRIGGER_MANUAL_LEN;
    case LAT_PC_MONITOR_START:    return LAT_PC_MONITOR_START_LEN;
    case LAT_PC_MONITOR_STOP:     return 0u;
    case LAT_PC_SET_WATCH:        return LAT_PC_SET_WATCH_LEN;
    case LAT_PC_TRIAL_START:      return LAT_PC_TRIAL_START_LEN;
    case LAT_PC_TRIAL_STOP:       return LAT_PC_RUN_STOP_LEN;
    case LAT_PC_POLL_TEST_START:  return LAT_PC_POLL_TEST_START_LEN;
    case LAT_PC_POLL_TEST_STOP:   return LAT_PC_RUN_STOP_LEN;
    case LAT_PC_SET_CALIBRATION:  return LAT_PC_CALIBRATION_LEN;
    case LAT_PC_GET_CALIBRATION:  return 0u;
    case LAT_PC_CALIBRATION_MODE: return LAT_PC_CALIBRATION_MODE_LEN;
    default:                      return 0xFFFEu;
    }
}

static void forward_command(uint8_t type, uint16_t seq, const uint8_t *p, uint32_t n)
{
    uint16_t want = forward_length(type);
    uint8_t *rec = s_rec.payload;

    if (want == 0xFFFEu)
    {
        finish(LAT_PC_ST_UNKNOWN_COMMAND, 0, 0u);
        return;
    }
    if (want == LAT_PC_LEN_VARIABLE)
    {
        /* SET_VENDOR_TABLE: 2 + 8 * count */
        if ((n < 2u) || (n != (2u + (LAT_PC_VENDOR_ENTRY_LEN * (uint32_t)p[0]))))
        {
            finish(LAT_PC_ST_BAD_LENGTH, 0, 0u);
            return;
        }
    }
    else if (n != want)
    {
        finish(LAT_PC_ST_BAD_LENGTH, 0, 0u);
        return;
    }
    if (n > LAT_IPC_ENV_BODY_MAX)
    {
        finish(LAT_PC_ST_BAD_LENGTH, 0, 0u);
        return;
    }
    if (!pc_port_v5f_running())
    {
        finish(LAT_PC_ST_INTERNAL, 0, 0u);
        return;
    }
    lat_put_u32(&rec[LAT_IPC_ENV_OFF_SESSION], s.session);
    rec[LAT_IPC_ENV_OFF_TYPE] = type;
    rec[LAT_IPC_ENV_OFF_STATUS] = 0u;
    lat_put_u16(&rec[LAT_IPC_ENV_OFF_SEQ], seq);
    lat_bytes_copy(&rec[LAT_IPC_ENV_LEN], p, n);
    if (!lat_ring_try_push(&g_ring_cmd, LAT_CMD_PC, 0u, rec, LAT_IPC_ENV_LEN + n))
    {
        finish(LAT_PC_ST_INTERNAL, 0, 0u);
        return;
    }
    s.fwd_active = true;
    s.fwd_type = type;
    s.fwd_seq = seq;
    s.fwd_deadline = s.now_ms + PC_FWD_TIMEOUT_MS;
}

static void fwd_poll(void)
{
    if (s.fwd_active && time_reached(s.now_ms, s.fwd_deadline))
    {
        s.fwd_active = false;
        s.stats.fwd_timeouts++;
        finish(LAT_PC_ST_INTERNAL, 0, 0u);
        pc_port_log("[v3f] pc: v5f did not answer a forwarded command within 2s (INTERNAL)");
    }
}

/* ------------------------------------------------------------------ */
/* RX frame handling                                                    */
/* ------------------------------------------------------------------ */

static void handle_command(uint8_t type, uint16_t seq, uint32_t session, const uint8_t *p, uint32_t n)
{
    if (s.boot != BOOT_IDLE)
    {
        s.stats.rx_dropped++;
        return;
    }
    if (type == LAT_PC_HELLO)
    {
        handle_hello(seq, session, p, n);
        return;
    }
    if ((type < LAT_PC_CMD_MIN) || (type > LAT_PC_CMD_MAX))
    {
        s.stats.rx_dropped++; /* command type out of range */
        return;
    }
    if ((s.session == 0u) || s.hello_pending || (session != s.session))
    {
        s.stats.rx_dropped++;
        return;
    }
    /* Retransmission check */
    if (s.last_valid && (seq == s.last_seq))
    {
        if ((type == s.last_type) && (n == s.last_len) && lat_bytes_equal(p, s.last_payload, n))
        {
            if (s.last_resp_ready)
            {
                s.resp_kind = RESP_LAST;
            }
            return; /* still in flight; wait for response */
        }
        respond_adhoc(type, seq, LAT_PC_ST_SEQ_CONFLICT);
        return;
    }
    if (s.fwd_active)
    {
        s.stats.rx_dropped++; /* new command while previous is in flight */
        return;
    }

    s.last_valid = true;
    s.last_resp_ready = false;
    s.last_type = type;
    s.last_seq = seq;
    s.last_len = (uint16_t)n;
    lat_bytes_copy(s.last_payload, p, n);

    switch (type)
    {
    case LAT_PC_GET_STATUS:
        cmd_get_status(n);
        break;
    case LAT_PC_ENTER_BOOTLOADER:
        cmd_enter_bootloader(p, n);
        break;
    case LAT_PC_READ_OBJECT:
        cmd_read_object(p, n);
        break;
    default:
        forward_command(type, seq, p, n);
        break;
    }
}

static void process_frame(const uint8_t *enc, uint32_t enc_len)
{
    uint32_t n = 0u;
    uint32_t length;
    uint16_t crc;
    const uint8_t *f = s.rx_dec;

    if (!lat_cobs_decode(enc, enc_len, s.rx_dec, LAT_PC_DECODED_MAX, &n))
    {
        s.stats.rx_bad_cobs++;
        return;
    }
    if (n < (LAT_PC_HEADER_LEN + LAT_PC_CRC_LEN))
    {
        s.stats.rx_bad_length++;
        return;
    }
    length = lat_get_u16(&f[LAT_PC_OFF_LENGTH]);
    if ((length > LAT_PC_PAYLOAD_MAX) || (n != (LAT_PC_HEADER_LEN + length + LAT_PC_CRC_LEN)))
    {
        s.stats.rx_bad_length++;
        return;
    }
    crc = lat_get_u16(&f[LAT_PC_HEADER_LEN + length]);
    if (crc != lat_crc16(f, LAT_PC_HEADER_LEN + length))
    {
        s.stats.rx_bad_crc++;
        return;
    }
    if (f[LAT_PC_OFF_VERSION] != LAT_PC_VERSION)
    {
        s.stats.rx_bad_length++;
        return;
    }
    s.stats.rx_frames++;
    handle_command(f[LAT_PC_OFF_TYPE], lat_get_u16(&f[LAT_PC_OFF_SEQ]), lat_get_u32(&f[LAT_PC_OFF_SESSION]),
                   &f[LAT_PC_HEADER_LEN], length);
}

void pc_link_rx_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    for (i = 0u; i < len; i++)
    {
        uint8_t b = data[i];

        if (b == 0u)
        {
            if (!s.rx_skip && (s.rx_len != 0u))
            {
                process_frame(s.rx_buf, s.rx_len);
            }
            s.rx_skip = false;
            s.rx_len = 0u;
            continue;
        }
        if (s.rx_skip)
        {
            continue;
        }
        if (s.rx_len >= LAT_PC_ENCODED_MAX)
        {
            /* Exceeded 1172 bytes (including delimiter); skip to next 0x00. */
            s.stats.rx_overrun++;
            s.rx_skip = true;
            s.rx_len = 0u;
            continue;
        }
        s.rx_buf[s.rx_len++] = b;
    }
}

/* ------------------------------------------------------------------ */
/* IPC drain from V5F                                                   */
/* ------------------------------------------------------------------ */

static bool place_staged(void)
{
    if (term_kind(s.staged_type))
    {
        bool run = term_is_run(s.staged_type);
        TermSlot *t = &s.term[run ? 0 : 1];

        if (t->state != TERM_FREE)
        {
            return false;
        }
        if (s.staged_len > PC_TERM_BODY_MAX)
        {
            s.stats.v5f_stale++;
            term_release(run);
            s.staged = false;
            return true;
        }
        lat_bytes_copy(t->body, s.staged_body, s.staged_len);
        t->len = s.staged_len;
        t->pos = s.crit_enq;
        t->order = s.term_order++;
        t->state = TERM_WAITING;
        s.staged = false;
        return true;
    }
    if (!q_push(&s.crit, s.staged_type, 0u, s.staged_body, s.staged_len))
    {
        return false;
    }
    s.crit_enq++;
    s.staged = false;
    return true;
}

static void accept_v5f_response(void)
{
    const uint8_t *e = s_rec.payload;

    if (s_rec.len < LAT_IPC_ENV_LEN)
    {
        s.stats.v5f_stale++;
        return;
    }
    if (!s.fwd_active || (lat_get_u32(&e[LAT_IPC_ENV_OFF_SESSION]) != s.session) ||
        (e[LAT_IPC_ENV_OFF_TYPE] != (uint8_t)(s.fwd_type | LAT_PC_RESPONSE_BIT)) ||
        (lat_get_u16(&e[LAT_IPC_ENV_OFF_SEQ]) != s.fwd_seq))
    {
        s.stats.v5f_stale++;
        return;
    }
    s.fwd_active = false;
    finish(e[LAT_IPC_ENV_OFF_STATUS], &e[LAT_IPC_ENV_LEN], (uint32_t)s_rec.len - LAT_IPC_ENV_LEN);
}

static void accept_v5f_critical(void)
{
    const uint8_t *e = s_rec.payload;
    uint32_t session;
    uint8_t pc_type;

    if (s_rec.len < LAT_IPC_ENV_LEN)
    {
        s.stats.v5f_stale++;
        return;
    }
    session = lat_get_u32(&e[LAT_IPC_ENV_OFF_SESSION]);
    pc_type = e[LAT_IPC_ENV_OFF_TYPE];
    if ((session == 0u) || (session != s.session) || (pc_type < LAT_PC_EVT_MIN) || (pc_type > LAT_PC_EVT_MAX))
    {
        /* Stale session event; hold count is resynced on session switch. */
        s.stats.v5f_stale++;
        return;
    }
    s.staged_type = pc_type;
    s.staged_len = (uint16_t)(s_rec.len - LAT_IPC_ENV_LEN);
    lat_bytes_copy(s.staged_body, &e[LAT_IPC_ENV_LEN], s.staged_len);
    s.staged = true;
    (void)place_staged();
}

static void pump_safe(void)
{
    uint32_t n;

    for (n = 0u; n < PC_PUMP_MAX; n++)
    {
        if (s.staged && !place_staged())
        {
            return;
        }
        if (!lat_ring_pop(&g_ring_evt_safe, &s_rec))
        {
            return;
        }
        switch (s_rec.type)
        {
        case LAT_EVT_PC_RESP:
            accept_v5f_response();
            break;
        case LAT_EVT_PC_EVENT:
            accept_v5f_critical();
            break;
        default:
            pc_port_ipc_record(&s_rec);
            break;
        }
    }
}

static void pump_lossy(void)
{
    uint32_t n;

    for (n = 0u; n < PC_PUMP_MAX; n++)
    {
        const uint8_t *e;
        bool is_monitor;

        if (!pc_port_log_ready())
        {
            return;
        }
        if (!lat_ring_pop(&g_ring_evt_lossy, &s_rec))
        {
            return;
        }
        if ((s_rec.type != LAT_EVT_MONITOR) && (s_rec.type != LAT_EVT_PC_LOG))
        {
            pc_port_ipc_record(&s_rec);
            continue;
        }
        e = s_rec.payload;
        is_monitor = (s_rec.type == LAT_EVT_MONITOR);
        if ((s_rec.len < LAT_IPC_ENV_LEN) || (s.session == 0u) || s.hello_pending || (s.boot != BOOT_IDLE) ||
            (lat_get_u32(&e[LAT_IPC_ENV_OFF_SESSION]) != s.session))
        {
            continue; /* no session or stale session */
        }
        if (!q_push(&s.lossy, e[LAT_IPC_ENV_OFF_TYPE], 0u, &e[LAT_IPC_ENV_LEN], (uint32_t)s_rec.len - LAT_IPC_ENV_LEN))
        {
            if (is_monitor)
            {
                s.v3f_drop_mon++;
            }
            else
            {
                s.v3f_drop_log++;
            }
        }
    }
}

static void overflow_poll(void)
{
    /* Mirror V5F-initiated overflow so it is reported on the next session. */
    if (!s.hello_pending)
    {
        uint32_t gen = lat_ipc_load_acquire(&g_ipc_header.session.v5f_ovf_gen);

        if (gen != s.v5f_ovf_seen)
        {
            s.v5f_ovf_seen = gen;
            if ((s.session != 0u) && !s.overflow)
            {
                s.overflow = true;
                pc_port_release_triggers();
                pc_port_log("[v3f] pc: v5f reports EVENT_OVERFLOW (critical events lost)");
            }
        }
    }
    if (!s.staged)
    {
        s.blocked = false;
        return;
    }
    if (!s.blocked)
    {
        s.blocked = true;
        s.blocked_since = s.now_ms;
        return;
    }
    if (!s.overflow && ((uint32_t)(s.now_ms - s.blocked_since) >= LAT_PC_OVERFLOW_MS))
    {
        s.overflow = true;
        pc_port_release_triggers();
        lat_ipc_store_release(&g_ipc_header.session.ovf_req, lat_ipc_word_load(&g_ipc_header.session.ovf_req) + 1u);
        pc_port_log("[v3f] pc: critical events blocked for 5s - EVENT_OVERFLOW state");
    }
}

static void drop_poll(void)
{
    uint32_t mon;
    uint32_t log;

    if ((s.session == 0u) || s.hello_pending || s.drop_pending)
    {
        return;
    }
    compute_drops(&mon, &log);
    if (((mon != s.rep_mon) || (log != s.rep_log)) &&
        ((uint32_t)(s.now_ms - s.drop_last_ms) >= PC_DROP_REPORT_MS))
    {
        s.drop_pending = true;
    }
}

void pc_link_poll(uint32_t now_ms)
{
    s.now_ms = now_ms;
    pump_safe();
    pump_lossy();
    session_poll();
    fwd_poll();
    overflow_poll();
    drop_poll();
    boot_poll();
}

/* ------------------------------------------------------------------ */
/* TX                                                                   */
/* ------------------------------------------------------------------ */

/* Build, CRC-16, and COBS-encode one frame into tx_enc. */
static void tx_load(uint8_t type, uint8_t status, uint16_t seq, uint32_t session, const uint8_t *body, uint32_t len,
                    MarkKind mark)
{
    uint8_t *f = s.tx_raw;
    uint32_t enc;

    f[LAT_PC_OFF_VERSION] = LAT_PC_VERSION;
    f[LAT_PC_OFF_TYPE] = type;
    f[LAT_PC_OFF_FLAGS] = 0u;
    f[LAT_PC_OFF_STATUS] = status;
    lat_put_u16(&f[LAT_PC_OFF_SEQ], seq);
    lat_put_u16(&f[LAT_PC_OFF_LENGTH], (uint16_t)len);
    lat_put_u32(&f[LAT_PC_OFF_SESSION], session);
    if ((len != 0u) && (body != &f[LAT_PC_HEADER_LEN]))
    {
        lat_bytes_copy(&f[LAT_PC_HEADER_LEN], body, len);
    }
    lat_put_u16(&f[LAT_PC_HEADER_LEN + len], lat_crc16(f, LAT_PC_HEADER_LEN + len));
    enc = lat_cobs_encode(f, LAT_PC_HEADER_LEN + len + LAT_PC_CRC_LEN, s.tx_enc, LAT_PC_ENCODED_MAX);
    s.tx_enc[enc] = 0u;
    s.tx_len = enc + 1u;
    s.tx_off = 0u;
    s.tx_mark = mark;
    s.stats.tx_frames++;
}

static void tx_load_event(uint8_t type, const uint8_t *body, uint32_t len, MarkKind mark)
{
    uint16_t seq = s.evt_seq;

    s.evt_seq = (uint16_t)(s.evt_seq + 1u);
    tx_load(type, 0u, seq, s.session, body, len, mark);
}

static TermSlot *term_ready(void)
{
    TermSlot *best = 0;
    uint32_t i;

    for (i = 0u; i < 2u; i++)
    {
        TermSlot *t = &s.term[i];

        if ((t->state == TERM_WAITING) && (t->pos == s.crit_deq))
        {
            if ((best == 0) || ((int32_t)(t->order - best->order) < 0))
            {
                best = t;
            }
        }
    }
    return best;
}

/* Pick next frame by priority; returns false if nothing to send. */
static bool tx_next(void)
{
    uint8_t type;
    uint16_t len;
    TermSlot *t;

    switch (s.resp_kind)
    {
    case RESP_LAST:
    {
        MarkKind mark = s.resp_boot_mark ? MARK_BOOT : MARK_NONE;

        s.resp_boot_mark = false;
        s.resp_kind = RESP_NONE;
        tx_load((uint8_t)(s.last_type | LAT_PC_RESPONSE_BIT), s.last_status, s.last_seq, s.session, s.last_resp,
                s.last_resp_len, mark);
        return true;
    }
    case RESP_ADHOC:
        s.resp_kind = RESP_NONE;
        tx_load((uint8_t)(s.adhoc_type | LAT_PC_RESPONSE_BIT), s.adhoc_status, s.adhoc_seq, s.adhoc_session, 0, 0u,
                MARK_NONE);
        return true;
    case RESP_HELLO:
        s.resp_kind = RESP_NONE;
        s.hello_unconfirmed = false; /* allow events after this response */
        tx_load((uint8_t)(LAT_PC_HELLO | LAT_PC_RESPONSE_BIT), LAT_PC_ST_OK, s.hello_seq, s.session, s.hello_resp,
                LAT_PC_HELLO_RESP_LEN, MARK_NONE);
        return true;
    default:
        break;
    }
    if (s.hello_pending || s.hello_unconfirmed || (s.session == 0u))
    {
        return false;
    }

    t = term_ready();
    if (t != 0)
    {
        bool run = (t == &s.term[0]);

        t->state = TERM_SENDING;
        tx_load_event(run ? LAT_PC_EVT_RUN_DONE : LAT_PC_EVT_POLL_RESULT, t->body, t->len,
                      run ? MARK_TERM_RUN : MARK_TERM_POLL);
        return true;
    }
    if (q_pop(&s.crit, &type, &s.tx_raw[LAT_PC_HEADER_LEN], &len))
    {
        s.crit_deq++;
        s.crit_inflight++;
        tx_load_event(type, &s.tx_raw[LAT_PC_HEADER_LEN], len, MARK_CRIT);
        return true;
    }
    if (s.drop_pending)
    {
        uint8_t body[LAT_PC_DROP_COUNTERS_LEN];
        uint32_t mon;
        uint32_t log;

        compute_drops(&mon, &log);
        lat_put_u32(&body[0], mon);
        lat_put_u32(&body[4], log);
        s.rep_mon = mon;
        s.rep_log = log;
        s.drop_last_ms = s.now_ms;
        s.drop_pending = false;
        tx_load_event(LAT_PC_EVT_DROP_COUNTERS, body, sizeof(body), MARK_NONE);
        return true;
    }
    if (q_pop(&s.lossy, &type, &s.tx_raw[LAT_PC_HEADER_LEN], &len))
    {
        tx_load_event(type, &s.tx_raw[LAT_PC_HEADER_LEN], len, MARK_NONE);
        return true;
    }
    return false;
}

static void mark_complete(MarkKind kind)
{
    switch (kind)
    {
    case MARK_TERM_RUN:
        s.term[0].state = TERM_FREE;
        term_release(true);
        break;
    case MARK_TERM_POLL:
        s.term[1].state = TERM_FREE;
        term_release(false);
        break;
    case MARK_BOOT:
        s.boot_resp_done = true;
        break;
    case MARK_CRIT:
        if (s.crit_inflight != 0u)
        {
            s.crit_inflight--;
        }
        break;
    default:
        break;
    }
}

static void marks_process(void)
{
    while ((s.mark_count != 0u) && ((int32_t)(s.tx_acked - s.marks[0].end) >= 0))
    {
        MarkKind kind = s.marks[0].kind;
        uint32_t i;

        for (i = 1u; i < s.mark_count; i++)
        {
            s.marks[i - 1u] = s.marks[i];
        }
        s.mark_count--;
        mark_complete(kind);
    }
}

static void mark_add(MarkKind kind, uint32_t end)
{
    if (s.mark_count >= PC_MARK_MAX)
    {
        /* Mark count is bounded by frames per 64-byte packet; if full, treat as completed. */
        mark_complete(kind);
        return;
    }
    s.marks[s.mark_count].end = end;
    s.marks[s.mark_count].kind = kind;
    s.mark_count++;
}

uint32_t pc_link_tx_fill(uint8_t *out, uint32_t max)
{
    uint32_t n = 0u;

    while (n < max)
    {
        if (s.tx_off < s.tx_len)
        {
            uint32_t c = s.tx_len - s.tx_off;

            if (c > (max - n))
            {
                c = max - n;
            }
            lat_bytes_copy(&out[n], &s.tx_enc[s.tx_off], c);
            s.tx_off += c;
            n += c;
            s.tx_stream += c;
            if ((s.tx_off == s.tx_len) && (s.tx_mark != MARK_NONE))
            {
                mark_add(s.tx_mark, s.tx_stream);
                s.tx_mark = MARK_NONE;
            }
            continue;
        }
        if (!tx_next())
        {
            break;
        }
    }
    return n;
}

void pc_link_tx_done(uint32_t n)
{
    s.tx_acked += n;
    marks_process();
}

void pc_link_rx_reset(void)
{
    /* USB reconnected: discard partial RX frame. */
    s.rx_len = 0u;
    s.rx_skip = false;
}

bool pc_link_tx_pending(void)
{
    if ((s.tx_off < s.tx_len) || (s.resp_kind != RESP_NONE))
    {
        return true;
    }
    if (s.hello_pending || s.hello_unconfirmed || (s.session == 0u))
    {
        return false;
    }
    return (term_ready() != 0) || (s.crit.count != 0u) || s.drop_pending || (s.lossy.count != 0u);
}

#define PC_LOG_TEXT_MAX  (240u)

void pc_link_push_log(const char *text, uint32_t len)
{
    uint8_t body[1u + PC_LOG_TEXT_MAX];

    if ((s.session == 0u) || s.hello_pending || s.hello_unconfirmed || (s.boot != BOOT_IDLE))
    {
        return;
    }
    if (len > PC_LOG_TEXT_MAX)
    {
        len = PC_LOG_TEXT_MAX;
    }
    body[0] = 0u; /* level: info */
    lat_bytes_copy(&body[1], (const uint8_t *)text, len);
    if (!q_push(&s.lossy, LAT_PC_EVT_LOG, 0u, body, len + 1u))
    {
        s.v3f_drop_log++;
    }
}

void pc_link_get_stats(PcLinkStats *out)
{
    s.stats.session_id = s.session;
    compute_drops(&s.stats.drop_monitor, &s.stats.drop_log);
    s.stats.overflow = s.overflow;
    s.stats.hello_pending = s.hello_pending;
    *out = s.stats;
}

void pc_link_init(void)
{
    lat_bytes_fill((uint8_t *)&s, 0u, (uint32_t)sizeof(s));
    q_init(&s.crit, s_crit_buf, PC_CRIT_BYTES);
    q_init(&s.lossy, s_lossy_buf, PC_LOSSY_BYTES);
}
