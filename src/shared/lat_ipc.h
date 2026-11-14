/*
 * lat_ipc.h - V3F/V5F shared SRAM layout and inter-core IPC.
 *
 * All shared regions are NOLOAD sections at fixed addresses matching the linker
 * scripts. V3F zeros them at startup, sets boot_state=READY, and wakes V5F;
 * V5F validates magic/ABI/size/epoch/rings before running.
 *
 * First 24 bytes of LatIpcHeader (magic..v5f_diag) are ABI-invariant so an
 * ABI-mismatched V5F can still report diagnostics.
 *
 * Session stop, overflow, and quiesce requests use IPC_HEADER fields instead of
 * IPC_CMD so they are not blocked if IPC_EVT_SAFE is full.
 */
#ifndef LAT_IPC_H
#define LAT_IPC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lat_fence.h"
#include "lat_ring.h"
#include "lat_seqlock.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Fixed shared SRAM addresses (must match linker scripts) ---- */
#define LAT_SHM_USBHS_DMA_ADDR     ((uint32_t)0x20150000u)
#define LAT_SHM_USBHS_DMA_SIZE     ((uint32_t)0x00002000u)
#define LAT_SHM_OBJECT_BUF_ADDR    ((uint32_t)0x20152000u)
#define LAT_SHM_OBJECT_BUF_SIZE    ((uint32_t)0x00010000u)
#define LAT_SHM_IPC_HEADER_ADDR    ((uint32_t)0x20162000u)
#define LAT_SHM_IPC_HEADER_SIZE    ((uint32_t)0x00001000u)
#define LAT_SHM_IPC_CMD_ADDR       ((uint32_t)0x20163000u)
#define LAT_SHM_IPC_CMD_SIZE       ((uint32_t)0x00002100u)
#define LAT_SHM_IPC_EVT_SAFE_ADDR  ((uint32_t)0x20165100u)
#define LAT_SHM_IPC_EVT_SAFE_SIZE  ((uint32_t)0x00004100u)
#define LAT_SHM_IPC_EVT_LOSSY_ADDR ((uint32_t)0x20169200u)
#define LAT_SHM_IPC_EVT_LOSSY_SIZE ((uint32_t)0x00008100u)
#define LAT_SHM_SPARE_ADDR         ((uint32_t)0x20171300u)
#define LAT_SHM_SPARE_SIZE         ((uint32_t)0x0000ECF8u)
#define LAT_SHM_BOOT_MAGIC_ADDR    ((uint32_t)0x2017FFF8u)
#define LAT_SHM_BOOT_MAGIC_SIZE    ((uint32_t)0x00000008u)

/* V5F flash base for NVIC_WakeUp_V5F (1024-byte aligned). */
#define LAT_V5F_FLASH_BASE         ((uint32_t)0x00030000u)

/* ---- ABI ---- */
#define LAT_IPC_MAGIC              ((uint32_t)0x4954414Cu) /* 'LATI' */
#define LAT_IPC_ABI_VERSION        ((uint16_t)3u)

/* ---- boot_state ---- */
#define LAT_BOOT_STATE_NONE        ((uint32_t)0u)
#define LAT_BOOT_STATE_READY       ((uint32_t)1u) /* V3F finished shared SRAM init */
#define LAT_BOOT_STATE_V5F_RUNNING ((uint32_t)2u) /* V5F validated and started TIM9 */

/* ---- V5F diagnostic codes (v5f_diag) ---- */
#define LAT_V5F_DIAG_NONE          ((uint32_t)0u)
/* V5F does not write v5f_diag on magic mismatch (memory may be uninitialized). */
#define LAT_V5F_DIAG_MAGIC         ((uint32_t)0xE500u)
#define LAT_V5F_DIAG_NOT_READY     ((uint32_t)0xE501u)
#define LAT_V5F_DIAG_ABI           ((uint32_t)0xE502u)
#define LAT_V5F_DIAG_SIZE          ((uint32_t)0xE503u)
#define LAT_V5F_DIAG_EPOCH         ((uint32_t)0xE504u)
#define LAT_V5F_DIAG_RING          ((uint32_t)0xE505u)

/* ---- Ring data sizes (power of 2; region = 256 + data) ---- */
#define LAT_IPC_CMD_DATA_BYTES       ((uint32_t)8192u)
#define LAT_IPC_EVT_SAFE_DATA_BYTES  ((uint32_t)16384u)
#define LAT_IPC_EVT_LOSSY_DATA_BYTES ((uint32_t)32768u)

/* ---- Object buffer ---- */
#define LAT_OBJECT_BUF_BYTES       ((uint32_t)65536u)

/* ---- Commands (V3F -> V5F, IPC_CMD) ---- */
#define LAT_CMD_PING               ((uint8_t)1u)
#define LAT_CMD_PC                 ((uint8_t)2u)  /* Forwarded PC command with envelope */

/*
 * ---- Events (V5F -> V3F) ----
 * Values must be < 16 (used as index into lost_by_type[]).
 */
#define LAT_EVT_LOG                ((uint8_t)1u)  /* LOSSY: ASCII line for UART (no envelope) */
#define LAT_EVT_PONG               ((uint8_t)2u)  /* SAFE: PING reply */
#define LAT_EVT_MONITOR            ((uint8_t)3u)  /* LOSSY: PC MONITOR_REPORT (with envelope) */
#define LAT_EVT_PC_LOG             ((uint8_t)4u)  /* LOSSY: PC LOG event (with envelope) */
#define LAT_EVT_PC_RESP            ((uint8_t)5u)  /* SAFE: PC command response (with envelope) */
#define LAT_EVT_PC_EVENT           ((uint8_t)6u)  /* SAFE: PC critical event (with envelope) */

/*
 * ---- PC envelope (8-byte prefix on IPC payload) ----
 *   +0 u32 session  +4 u8 pc_type  +5 u8 status  +6 u16 seq  +8 body...
 */
#define LAT_IPC_ENV_OFF_SESSION    (0u)
#define LAT_IPC_ENV_OFF_TYPE       (4u)
#define LAT_IPC_ENV_OFF_STATUS     (5u)
#define LAT_IPC_ENV_OFF_SEQ        (6u)
#define LAT_IPC_ENV_LEN            (8u)
#define LAT_IPC_ENV_BODY_MAX       (LAT_RING_PAYLOAD_MAX - LAT_IPC_ENV_LEN)

/* USBHS DMA buffer size (1024 bytes each, 4-byte aligned). */
#define LAT_USBHS_DMA_BUF_SIZE     ((uint32_t)1024u)

typedef struct LatPingCmd
{
    uint32_t ping_id;
    uint32_t v3f_ticks;   /* V3F SysTick0 timestamp (10 ns units), echoed */
} LatPingCmd;

typedef struct LatPongEvt
{
    uint32_t ping_id;
    uint32_t v3f_ticks;   /* Echoed from PING */
    uint32_t tim9_lo;     /* Low 32 bits of tim9_now() at reply */
    uint32_t tim9_hi;     /* High 32 bits of tim9_now() at reply */
    uint32_t v5f_heartbeat;
} LatPongEvt;

_Static_assert(sizeof(LatPingCmd) <= LAT_RING_PAYLOAD_MAX, "LatPingCmd too large");
_Static_assert(sizeof(LatPongEvt) <= LAT_RING_PAYLOAD_MAX, "LatPongEvt too large");

/*
 * Published TIM9 time (updated every 1 ms by V5F, protected by seqlock).
 */
typedef struct LatTimeSnapshot
{
    uint32_t seq;
    uint32_t epoch;   /* Upper 32 bits */
    uint32_t cnt;     /* Lower 32 bits (TIM9->CNT_32) */
} LatTimeSnapshot;

_Static_assert(sizeof(LatTimeSnapshot) == 12u, "LatTimeSnapshot layout changed - bump LAT_IPC_ABI_VERSION");

/* V5F status snapshot (V5F-written, seqlock-protected). */
typedef struct LatV5fStatus
{
    uint32_t seq;
    uint32_t tim9_isr_count;     /* TIM9 ISR entries */
    uint32_t tim9_update_count;  /* TIM9 wrap events handled */
    uint32_t tim9_backsteps;     /* tim9_now() monotonicity violations (0 = normal) */
    uint32_t loop_count;         /* V5F main-loop iterations */
    uint32_t cmd_count;          /* Commands processed */
    uint32_t cmd_deferred;       /* Commands deferred waiting for safe-ring space */
    uint32_t reserved;
} LatV5fStatus;

_Static_assert(sizeof(LatV5fStatus) == 32u, "LatV5fStatus layout changed - bump LAT_IPC_ABI_VERSION");

/*
 * Session, overflow, and quiesce handshake.
 * *_req is V3F-written, *_ack is V5F-written.
 *   session_req  Non-zero session ID; V5F stops active work, clears state,
 *                syncs ovf_ack/quiesce_ack, and writes session_ack last.
 *   ovf_req      Overflow request counter; V5F aborts active run/poll.
 *   v5f_ovf_gen  Incremented by V5F on internal event loss (before session_ack).
 *   quiesce_req  Odd = quiesce before bootloader, even = resume.
 */
typedef struct LatSessionCtl
{
    uint32_t session_req;
    uint32_t session_ack;
    uint32_t ovf_req;
    uint32_t ovf_ack;
    uint32_t quiesce_req;
    uint32_t quiesce_ack;
    uint32_t v5f_ovf_gen;   /* V5F-written internal overflow generation */
    uint32_t reserved;
} LatSessionCtl;

_Static_assert(sizeof(LatSessionCtl) == 32u, "LatSessionCtl layout changed - bump LAT_IPC_ABI_VERSION");
_Static_assert(offsetof(LatSessionCtl, v5f_ovf_gen) == 24u, "LatSessionCtl.v5f_ovf_gen offset");

/*
 * Terminal event (RUN_DONE, POLL_RESULT) holdback counters.
 * *_produced is incremented by V5F; *_released is incremented by V3F after
 * USB TX or session drop. Start requests return BUSY while produced != released.
 */
typedef struct LatTermCounters
{
    uint32_t run_done_produced;     /* V5F */
    uint32_t poll_result_produced;  /* V5F */
    uint32_t run_done_released;     /* V3F */
    uint32_t poll_result_released;  /* V3F */
} LatTermCounters;

_Static_assert(sizeof(LatTermCounters) == 16u, "LatTermCounters layout changed - bump LAT_IPC_ABI_VERSION");

/* LatHostStatus.flags */
#define LAT_HOST_FLAG_OVERFLOW     ((uint8_t)0x01u) /* Overflow state (rejects new runs) */
#define LAT_HOST_FLAG_CALIBRATION  ((uint8_t)0x02u) /* Calibration mode (1/2) active */
#define LAT_HOST_FLAG_MONITOR      ((uint8_t)0x04u) /* Monitor active */
#define LAT_HOST_FLAG_QUIESCED     ((uint8_t)0x08u) /* Quiesced (no events emitted) */

/*
 * USBHS host and measurement status (GET_STATUS, seqlock-protected by V5F).
 */
typedef struct LatHostStatus
{
    uint32_t seq;               /* +0 */
    uint8_t  host_state;        /* +4  LAT_PC_HOST_* */
    uint8_t  speed;             /* +5  0=none, 1=LS, 2=FS, 3=HS */
    uint8_t  run_kind;          /* +6  LAT_PC_RUN_* */
    uint8_t  flags;             /* +7  LAT_HOST_FLAG_* */
    uint32_t device_generation; /* +8 */
    uint32_t enum_error;        /* +12 */
    uint32_t sched_late_count;  /* +16 */
    uint32_t sched_late_max;    /* +20 */
    uint32_t txn_max_ticks;     /* +24 */
    uint32_t recover_count;     /* +28 */
    uint32_t active_run_id;     /* +32 */
    uint8_t  trigger_asserted;  /* +36 Asserted low triggers (bit n = Tn) */
    uint8_t  reserved0[3];      /* +37 */
    uint32_t reserved1[2];      /* +40 */
} LatHostStatus;

_Static_assert(sizeof(LatHostStatus) == 48u, "LatHostStatus layout changed - bump LAT_IPC_ABI_VERSION");
_Static_assert(offsetof(LatHostStatus, device_generation) == 8u, "LatHostStatus.device_generation offset");
_Static_assert(offsetof(LatHostStatus, active_run_id) == 32u, "LatHostStatus.active_run_id offset");
_Static_assert(offsetof(LatHostStatus, trigger_asserted) == 36u, "LatHostStatus.trigger_asserted offset");

/* LatObjectMeta.state */
#define LAT_OBJ_STATE_NONE         ((uint32_t)0u) /* No object (invalid) */
#define LAT_OBJ_STATE_BUILDING     ((uint32_t)1u) /* V5F writing OBJECT_BUF */
#define LAT_OBJ_STATE_READY        ((uint32_t)2u) /* Ready until next invalidation */

/*
 * Object handoff.
 * V5F sets BUILDING, writes OBJECT_BUF + fields, then release-stores READY.
 * V3F acquire-loads READY, reads fields/data, then re-checks state & object_id.
 */
typedef struct LatObjectMeta
{
    uint32_t state;       /* +0  LAT_OBJ_STATE_* */
    uint32_t session;     /* +4  Session ID when created */
    uint16_t object_id;   /* +8  Non-zero */
    uint8_t  kind;        /* +10 LAT_PC_OBJ_* */
    uint8_t  index;       /* +11 */
    uint32_t source_id;   /* +12 */
    uint32_t length;      /* +16 total_length from OBJECT_BUF base */
    uint32_t crc32;       /* +20 CRC-32/ISO-HDLC */
    uint32_t reserved[2]; /* +24 */
} LatObjectMeta;

_Static_assert(sizeof(LatObjectMeta) == 32u, "LatObjectMeta layout changed - bump LAT_IPC_ABI_VERSION");
_Static_assert(offsetof(LatObjectMeta, object_id) == 8u, "LatObjectMeta.object_id offset");
_Static_assert(offsetof(LatObjectMeta, source_id) == 12u, "LatObjectMeta.source_id offset");

/*
 * V5F ISR event summary (V5F ISR-written, seqlock-protected).
 */
typedef struct LatIsrEvents
{
    uint32_t seq;             /* +0 */
    uint32_t attach_count;    /* +4  Attach count */
    uint32_t detach_count;    /* +8  Detach count */
    uint32_t recover_count;   /* +12 Recovery count */
    uint32_t error_count;     /* +16 Error count */
    uint32_t last_generation; /* +20 Last attach/detach device_generation */
    uint8_t  last_speed;      /* +24 Last attached device speed */
    uint8_t  recover_cause;   /* +25 RECOVERED cause */
    uint8_t  recover_result;  /* +26 RECOVERED result */
    uint8_t  reserved0;       /* +27 */
    uint16_t error_code;      /* +28 ERROR code */
    uint16_t reserved1;       /* +30 */
    uint32_t error_detail;    /* +32 ERROR detail */
    uint32_t lost_count;      /* +36 Lost count in V5F internal queue (triggers overflow) */
    uint32_t reserved2[2];    /* +40 */
} LatIsrEvents;

_Static_assert(sizeof(LatIsrEvents) == 48u, "LatIsrEvents layout changed - bump LAT_IPC_ABI_VERSION");
_Static_assert(offsetof(LatIsrEvents, error_code) == 28u, "LatIsrEvents.error_code offset");

typedef struct LatIpcHeader
{
    /* ---- Fixed header (ABI-invariant) ---- */
    uint32_t magic;           /* +0  LAT_IPC_MAGIC */
    uint16_t abi_version;     /* +4  LAT_IPC_ABI_VERSION */
    uint16_t struct_size;     /* +6  sizeof(LatIpcHeader) */
    uint32_t boot_state;      /* +8  LAT_BOOT_STATE_* */
    uint32_t boot_epoch;      /* +12 Boot generation (0 = invalid) */
    uint32_t boot_epoch_inv;  /* +16 ~boot_epoch */
    uint32_t v5f_diag;        /* +20 LAT_V5F_DIAG_* */
    /* ---- ABI-versioned fields ---- */
    uint32_t v3f_heartbeat;   /* +24 Incremented every 1 ms by V3F */
    uint32_t v5f_heartbeat;   /* +28 Incremented every 1 ms by V5F (TIM9) */
    LatTimeSnapshot time;     /* +32 Published time (1 ms, V5F) */
    uint32_t v5f_capabilities;/* +44 HELLO capabilities (LAT_PC_CAP_*), V5F-written */
    LatV5fStatus v5f_status;  /* +48 */
    LatSessionCtl session;    /* +80 Session/overflow/quiesce handshake */
    LatTermCounters term;     /* +112 Terminal event holdback counters */
    LatHostStatus host;       /* +128 GET_STATUS snapshot (V5F, seqlock) */
    LatObjectMeta object;     /* +176 Object metadata */
    LatIsrEvents isr;         /* +208 V5F ISR event summary */
} LatIpcHeader;

_Static_assert(offsetof(LatIpcHeader, magic) == 0u, "fixed prefix: magic");
_Static_assert(offsetof(LatIpcHeader, abi_version) == 4u, "fixed prefix: abi_version");
_Static_assert(offsetof(LatIpcHeader, struct_size) == 6u, "fixed prefix: struct_size");
_Static_assert(offsetof(LatIpcHeader, boot_state) == 8u, "fixed prefix: boot_state");
_Static_assert(offsetof(LatIpcHeader, boot_epoch) == 12u, "fixed prefix: boot_epoch");
_Static_assert(offsetof(LatIpcHeader, boot_epoch_inv) == 16u, "fixed prefix: boot_epoch_inv");
_Static_assert(offsetof(LatIpcHeader, v5f_diag) == 20u, "fixed prefix: v5f_diag");
_Static_assert(offsetof(LatIpcHeader, time) == 32u, "LatIpcHeader.time offset changed");
_Static_assert(offsetof(LatIpcHeader, v5f_capabilities) == 44u, "LatIpcHeader.v5f_capabilities offset changed");
_Static_assert(offsetof(LatIpcHeader, v5f_status) == 48u, "LatIpcHeader.v5f_status offset changed");
_Static_assert(offsetof(LatIpcHeader, session) == 80u, "LatIpcHeader.session offset changed");
_Static_assert(offsetof(LatIpcHeader, term) == 112u, "LatIpcHeader.term offset changed");
_Static_assert(offsetof(LatIpcHeader, host) == 128u, "LatIpcHeader.host offset changed");
_Static_assert(offsetof(LatIpcHeader, object) == 176u, "LatIpcHeader.object offset changed");
_Static_assert(offsetof(LatIpcHeader, isr) == 208u, "LatIpcHeader.isr offset changed");
_Static_assert(sizeof(LatIpcHeader) == 256u, "LatIpcHeader layout changed - bump LAT_IPC_ABI_VERSION");
_Static_assert(sizeof(LatIpcHeader) <= LAT_SHM_IPC_HEADER_SIZE, "LatIpcHeader does not fit IPC_HEADER");

/* Ring regions: 256-byte header+stats followed by data area. */
typedef struct LatIpcCmdRing
{
    LatRingHead head;
    uint8_t data[LAT_IPC_CMD_DATA_BYTES];
} LatIpcCmdRing;

typedef struct LatIpcEvtSafeRing
{
    LatRingHead head;
    uint8_t data[LAT_IPC_EVT_SAFE_DATA_BYTES];
} LatIpcEvtSafeRing;

typedef struct LatIpcEvtLossyRing
{
    LatRingHead head;
    uint8_t data[LAT_IPC_EVT_LOSSY_DATA_BYTES];
} LatIpcEvtLossyRing;

typedef struct LatUsbhsDma
{
    uint8_t tx[LAT_USBHS_DMA_BUF_SIZE];
    uint8_t rx[LAT_USBHS_DMA_BUF_SIZE];
} LatUsbhsDma;

typedef struct LatObjectBuf
{
    uint8_t data[LAT_OBJECT_BUF_BYTES];
} LatObjectBuf;

#define LAT_IS_POW2(x) (((x) != 0u) && (((x) & ((x) - 1u)) == 0u))

_Static_assert(LAT_IS_POW2(LAT_IPC_CMD_DATA_BYTES / LAT_RING_UNIT), "CMD ring units must be a power of two");
_Static_assert(LAT_IS_POW2(LAT_IPC_EVT_SAFE_DATA_BYTES / LAT_RING_UNIT), "SAFE ring units must be a power of two");
_Static_assert(LAT_IS_POW2(LAT_IPC_EVT_LOSSY_DATA_BYTES / LAT_RING_UNIT), "LOSSY ring units must be a power of two");
_Static_assert((LAT_IPC_EVT_LOSSY_DATA_BYTES / LAT_RING_UNIT) <= 0xFFFFu, "ring capacity must fit the u16 header field");
_Static_assert((LAT_IPC_CMD_DATA_BYTES / LAT_RING_UNIT) >= LAT_RING_MAX_RECORD_UNITS, "CMD ring must hold a maximum-size record");
_Static_assert((LAT_IPC_EVT_SAFE_DATA_BYTES / LAT_RING_UNIT) >= LAT_RING_MAX_RECORD_UNITS, "SAFE ring must hold a maximum-size record");
_Static_assert(offsetof(LatIpcCmdRing, data) == LAT_RING_HEAD_AREA, "CMD ring data offset");
_Static_assert(offsetof(LatIpcEvtSafeRing, data) == LAT_RING_HEAD_AREA, "SAFE ring data offset");
_Static_assert(offsetof(LatIpcEvtLossyRing, data) == LAT_RING_HEAD_AREA, "LOSSY ring data offset");
_Static_assert(sizeof(LatIpcCmdRing) == LAT_SHM_IPC_CMD_SIZE, "LatIpcCmdRing must fill IPC_CMD exactly");
_Static_assert(sizeof(LatIpcEvtSafeRing) == LAT_SHM_IPC_EVT_SAFE_SIZE, "LatIpcEvtSafeRing must fill IPC_EVT_SAFE exactly");
_Static_assert(sizeof(LatIpcEvtLossyRing) == LAT_SHM_IPC_EVT_LOSSY_SIZE, "LatIpcEvtLossyRing must fill IPC_EVT_LOSSY exactly");
_Static_assert(sizeof(LatUsbhsDma) <= LAT_SHM_USBHS_DMA_SIZE, "LatUsbhsDma does not fit USBHS_DMA");
_Static_assert(offsetof(LatUsbhsDma, rx) % 4u == 0u, "USBHS RX buffer must be 4-byte aligned");
_Static_assert(sizeof(LatObjectBuf) == LAT_SHM_OBJECT_BUF_SIZE, "LatObjectBuf must fill OBJECT_BUF exactly");

/* Defined in src/shared/lat_ipc.c (linked by both V3F and V5F). */
extern LatUsbhsDma g_usbhs_dma;
extern LatObjectBuf g_object_buf;
extern LatIpcHeader g_ipc_header;
extern LatIpcCmdRing g_ipc_cmd;
extern LatIpcEvtSafeRing g_ipc_evt_safe;
extern LatIpcEvtLossyRing g_ipc_evt_lossy;

/* Ring handles (lat_ipc.c). */
extern const LatRing g_ring_cmd;
extern const LatRing g_ring_evt_safe;
extern const LatRing g_ring_evt_lossy;

/* ---- Header accessors ---- */

static inline uint32_t lat_ipc_boot_state_load(void)
{
    return __atomic_load_n(&g_ipc_header.boot_state, __ATOMIC_ACQUIRE);
}

static inline void lat_ipc_boot_state_store(uint32_t state)
{
    __atomic_store_n(&g_ipc_header.boot_state, state, __ATOMIC_RELEASE);
}

static inline uint32_t lat_ipc_v5f_diag_load(void)
{
    return __atomic_load_n(&g_ipc_header.v5f_diag, __ATOMIC_ACQUIRE);
}

static inline void lat_ipc_word_store(uint32_t *field, uint32_t value)
{
    __atomic_store_n(field, value, __ATOMIC_RELAXED);
}

static inline uint32_t lat_ipc_word_load(const uint32_t *field)
{
    return __atomic_load_n(field, __ATOMIC_RELAXED);
}

/* V5F: publish TIM9 time (every 1 ms). */
static inline void lat_ipc_time_publish(uint64_t now)
{
    LatTimeSnapshot *t = &g_ipc_header.time;

    lat_seqlock_write_begin(&t->seq);
    t->epoch = (uint32_t)(now >> 32);
    t->cnt = (uint32_t)now;
    lat_seqlock_write_end(&t->seq);
}

/* V3F: read published time; returns false if unstable after max_tries. */
static inline bool lat_ipc_time_read(uint64_t *out, uint32_t max_tries)
{
    const LatTimeSnapshot *t = &g_ipc_header.time;
    uint32_t tries;

    for (tries = 0u; tries < max_tries; tries++)
    {
        uint32_t s = lat_seqlock_read_begin(&t->seq);
        uint32_t epoch = lat_ipc_word_load(&t->epoch);
        uint32_t cnt = lat_ipc_word_load(&t->cnt);

        if (!lat_seqlock_read_retry(&t->seq, s))
        {
            *out = ((uint64_t)epoch << 32) | (uint64_t)cnt;
            return true;
        }
    }
    return false;
}

static inline uint32_t lat_ipc_load_acquire(const uint32_t *field)
{
    return __atomic_load_n(field, __ATOMIC_ACQUIRE);
}

static inline void lat_ipc_store_release(uint32_t *field, uint32_t value)
{
    __atomic_store_n(field, value, __ATOMIC_RELEASE);
}

/* V5F: publish host status (GET_STATUS); does not overwrite seq/reserved. */
static inline void lat_ipc_host_publish(const LatHostStatus *src)
{
    LatHostStatus *h = &g_ipc_header.host;

    lat_seqlock_write_begin(&h->seq);
    h->host_state = src->host_state;
    h->speed = src->speed;
    h->run_kind = src->run_kind;
    h->flags = src->flags;
    h->device_generation = src->device_generation;
    h->enum_error = src->enum_error;
    h->sched_late_count = src->sched_late_count;
    h->sched_late_max = src->sched_late_max;
    h->txn_max_ticks = src->txn_max_ticks;
    h->recover_count = src->recover_count;
    h->active_run_id = src->active_run_id;
    h->trigger_asserted = src->trigger_asserted;
    lat_seqlock_write_end(&h->seq);
}

/* V3F: read published host status; returns false if unstable after max_tries. */
static inline bool lat_ipc_host_read(LatHostStatus *out, uint32_t max_tries)
{
    const LatHostStatus *h = &g_ipc_header.host;
    uint32_t tries;

    for (tries = 0u; tries < max_tries; tries++)
    {
        uint32_t s = lat_seqlock_read_begin(&h->seq);

        out->host_state = h->host_state;
        out->speed = h->speed;
        out->run_kind = h->run_kind;
        out->flags = h->flags;
        out->device_generation = h->device_generation;
        out->enum_error = h->enum_error;
        out->sched_late_count = h->sched_late_count;
        out->sched_late_max = h->sched_late_max;
        out->txn_max_ticks = h->txn_max_ticks;
        out->recover_count = h->recover_count;
        out->active_run_id = h->active_run_id;
        out->trigger_asserted = h->trigger_asserted;
        if (!lat_seqlock_read_retry(&h->seq, s))
        {
            return true;
        }
    }
    return false;
}

/*
 * lat_ipc_v5f_validate: called by V5F at startup; returns LAT_V5F_DIAG_NONE
 * if valid. Acquire-loads boot_state first so V3F's init writes are visible.
 */
static inline uint32_t lat_ipc_v5f_validate(void)
{
    uint32_t state = lat_ipc_boot_state_load();
    uint32_t epoch;

    if (g_ipc_header.magic != LAT_IPC_MAGIC)
    {
        return LAT_V5F_DIAG_MAGIC;
    }
    if (state != LAT_BOOT_STATE_READY)
    {
        return LAT_V5F_DIAG_NOT_READY;
    }
    if (g_ipc_header.abi_version != LAT_IPC_ABI_VERSION)
    {
        return LAT_V5F_DIAG_ABI;
    }
    if (g_ipc_header.struct_size != (uint16_t)sizeof(LatIpcHeader))
    {
        return LAT_V5F_DIAG_SIZE;
    }
    epoch = g_ipc_header.boot_epoch;
    if ((epoch == 0u) || (g_ipc_header.boot_epoch_inv != ~epoch))
    {
        return LAT_V5F_DIAG_EPOCH;
    }
    if (!lat_ring_header_matches(&g_ring_cmd) ||
        !lat_ring_header_matches(&g_ring_evt_safe) ||
        !lat_ring_header_matches(&g_ring_evt_lossy))
    {
        return LAT_V5F_DIAG_RING;
    }
    return LAT_V5F_DIAG_NONE;
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_IPC_H */
