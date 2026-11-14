/*
 * usbhs_host.h - V5F USBHS host engine
 *
 * Software-scheduled single-transaction USBHS host engine: port state/reset/
 * recovery, TIM9 CH2 SOF-aligned scheduler (periodic OUT > periodic IN > ctrl),
 * control transfers, interrupt IN/OUT, and transaction tracing.
 *
 * Context rules:
 *   - usbhs_* APIs are called from the main loop (briefly masking USBHS IRQ).
 *   - UsbhsIntrCfg.hook runs inside USBHS ISR.
 *   - All timestamps are 64-bit TIM9 ticks (10ns); scheduled times must be
 *     < 0x80000000 ticks (~21s) ahead of now.
 */
#ifndef LAT_USBHS_HOST_H
#define LAT_USBHS_HOST_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Time constants (10ns ticks) */
#define USBHS_TICKS_PER_US        (100u)
#define USBHS_TICKS_PER_MS        (100000u)
#define USBHS_FS_FRAME_TICKS      (100000u)  /* 1ms Full/Low Speed frame */
#define USBHS_HS_UFRAME_TICKS     (12500u)   /* 125us High Speed microframe */

/* Buffer and table limits */
#define USBHS_PKT_MAX             (1024u)    /* DMA buffer size (lat_ipc.h) */
#define USBHS_INTR_MAX            (16u)      /* Max open periodic endpoints */
#define USBHS_OUT_QUEUE           (4u)       /* Periodic OUT queue depth */
#define USBHS_OUT_MAX             (64u)      /* Max periodic OUT packet */
#define USBHS_TIM9_TX_MAX         (64u)      /* Max TX bytes from TIM9 ISR */
#define USBHS_TRACE_DATA_MAX      (64u)      /* Max traced payload bytes */

/* Speed */
#define USBHS_SPEED_NONE          ((uint8_t)0u)
#define USBHS_SPEED_LS            ((uint8_t)1u)
#define USBHS_SPEED_FS            ((uint8_t)2u)
#define USBHS_SPEED_HS            ((uint8_t)3u)

/* Port states */
#define USBHS_PORT_DISCONNECTED   ((uint8_t)0u)
#define USBHS_PORT_DEBOUNCE       ((uint8_t)1u) /* Attached, waiting for reset */
#define USBHS_PORT_RESETTING      ((uint8_t)2u)
#define USBHS_PORT_ENABLED        ((uint8_t)3u) /* Reset done, SOF active */
#define USBHS_PORT_RECOVERING     ((uint8_t)4u)
#define USBHS_PORT_HALTED         ((uint8_t)5u) /* Fatal recovery failure */

/* Transfer results */
#define USBHS_RES_OK              ((uint8_t)0u)
#define USBHS_RES_NAK             ((uint8_t)1u)
#define USBHS_RES_STALL           ((uint8_t)2u)
#define USBHS_RES_TIMEOUT         ((uint8_t)3u) /* No completion IRQ or ctrl timeout */
#define USBHS_RES_ERROR           ((uint8_t)4u) /* No response, unexpected PID, etc. */
#define USBHS_RES_DETACHED        ((uint8_t)5u)
#define USBHS_RES_ABORTED         ((uint8_t)6u)
#define USBHS_RES_TOGGLE          ((uint8_t)7u) /* Data toggle mismatch (ACKed & dropped) */

/* Control transfer stages */
#define USBHS_STAGE_SETUP         ((uint8_t)0u)
#define USBHS_STAGE_DATA          ((uint8_t)1u)
#define USBHS_STAGE_STATUS        ((uint8_t)2u)

/* Recovery causes */
#define USBHS_CAUSE_TXN_TIMEOUT   ((uint8_t)0u)
#define USBHS_CAUSE_SOF_STOPPED   ((uint8_t)1u)
#define USBHS_CAUSE_ACTION_STUCK  ((uint8_t)2u)
#define USBHS_CAUSE_OTHER         ((uint8_t)3u)

/* Trace kinds/tokens/results */
#define USBHS_TRK_TXN             ((uint8_t)0u)
#define USBHS_TRK_RESET_START     ((uint8_t)1u) /* data[0] = 1 if long reset */
#define USBHS_TRK_RESET_END       ((uint8_t)2u) /* data[0..3]=ticks(LE), data[4]=speed */
#define USBHS_TRK_ATTACH          ((uint8_t)3u) /* data[0] = pre-reset speed */
#define USBHS_TRK_DETACH          ((uint8_t)4u)
#define USBHS_TRK_SPEED           ((uint8_t)5u) /* data[0] = post-reset speed */
#define USBHS_TRK_NAK_RUN         ((uint8_t)6u)
#define USBHS_TRK_IN_PAUSE        ((uint8_t)7u)
#define USBHS_TRK_IN_RESUME       ((uint8_t)8u)
#define USBHS_TRK_REENUMERATE     ((uint8_t)9u)
#define USBHS_TRK_ENUM_SETTINGS   ((uint8_t)10u)

#define USBHS_TOK_SETUP           ((uint8_t)0u)
#define USBHS_TOK_IN              ((uint8_t)1u)
#define USBHS_TOK_OUT             ((uint8_t)2u)

#define USBHS_TRR_ACK             ((uint8_t)0u) /* NYET is also recorded as ACK */
#define USBHS_TRR_NAK             ((uint8_t)1u)
#define USBHS_TRR_STALL           ((uint8_t)2u)
#define USBHS_TRR_DATA0           ((uint8_t)3u)
#define USBHS_TRR_DATA1           ((uint8_t)4u)
#define USBHS_TRR_TIMEOUT         ((uint8_t)5u)
#define USBHS_TRR_OTHER           ((uint8_t)6u)

/* Main-loop events (usbhs_event_pop) */
#define USBHS_EV_ATTACH           ((uint8_t)1u)
#define USBHS_EV_DETACH           ((uint8_t)2u)
#define USBHS_EV_RESET_DONE       ((uint8_t)3u) /* value = reset duration ticks */
#define USBHS_EV_CTRL_DONE        ((uint8_t)4u)
#define USBHS_EV_OUT_DONE         ((uint8_t)5u) /* value = bytes sent */
#define USBHS_EV_PIPE_HALT        ((uint8_t)6u) /* value = pause ticks */
#define USBHS_EV_RECOVERED        ((uint8_t)7u) /* result: 0=ok, 1=fatal; value=cause */
#define USBHS_EV_BOUNCE           ((uint8_t)8u) /* Debounce attach time updated */

/* Scheduling modes for control stages and NAK retries */
#define USBHS_WHEN_DELAY          ((uint8_t)0u) /* ref_time + ticks */
#define USBHS_WHEN_NEXT_FRAME     ((uint8_t)1u) /* next (micro)frame start + ticks */

typedef struct UsbhsWhen
{
    uint8_t  mode;         /* USBHS_WHEN_* */
    uint8_t  reserved[3];
    uint32_t ticks;
} UsbhsWhen;

/* Control transfer timing rules relative to previous transaction completion. */
typedef struct UsbhsCtrlTiming
{
    UsbhsWhen data_first;    /* SETUP done -> first DATA */
    UsbhsWhen data_next;     /* DATA done -> next DATA */
    UsbhsWhen status;        /* Last DATA (or SETUP) done -> STATUS */
    UsbhsWhen nak_first;     /* First packet of stage NAK/toggle retry */
    UsbhsWhen nak_cont;      /* Mid-DATA stage NAK/toggle retry */
    UsbhsWhen error;         /* Error/timeout retry */
    uint32_t  timeout_ticks; /* Overall transfer timeout from SETUP start_at */
    uint8_t   error_retries;
    uint8_t   reserved[3];
} UsbhsCtrlTiming;

typedef struct UsbhsCtrlReq
{
    uint8_t   setup[8];
    uint8_t   addr;
    uint8_t   mps;         /* EP0 max packet size (8, 16, 32, 64) */
    uint16_t  step;        /* Trace step ID */
    uint8_t  *buf;         /* IN: rx dest, OUT: tx data */
    uint16_t  buf_len;
    uint8_t   trace;       /* 1: record transactions */
    uint8_t   reserved;
    uint64_t  start_at;    /* Scheduled SETUP time (0 = immediate) */
    UsbhsCtrlTiming timing;
} UsbhsCtrlReq;

typedef struct UsbhsCtrlResult
{
    uint8_t  result;       /* USBHS_RES_* */
    uint8_t  stage;        /* USBHS_STAGE_* */
    uint8_t  last_pid;     /* Low 4 bits of INT_ST */
    uint8_t  reserved;
    uint16_t actual;       /* Bytes transferred in DATA stage */
    uint16_t step;
    uint32_t generation;
    uint32_t nak_count;    /* NAK + toggle mismatch count */
    uint32_t error_count;
    int32_t  setup_late;   /* First SETUP issue - start_at */
    uint64_t setup_issue;
    uint64_t setup_done;
    uint64_t data_issue;
    uint64_t data_done;
    uint64_t status_issue;
    uint64_t status_done;
    uint64_t end_time;
} UsbhsCtrlResult;

/* Periodic transfer completion (used by hook and usbhs_report_pop). */
typedef struct UsbhsIntrDone
{
    uint64_t t1;           /* Completion ISR entry timestamp */
    uint64_t issue;
    uint32_t generation;
    uint32_t seq;          /* Report sequence number */
    uint16_t len;
    uint16_t frame;
    uint8_t  uframe;       /* High Speed only */
    uint8_t  pipe;
    uint8_t  result;       /* USBHS_RES_* */
    uint8_t  pid;          /* Low 4 bits of INT_ST */
} UsbhsIntrDone;

/*
 * Periodic completion hook called from USBHS ISR.
 * For IN data, data points to the DMA buffer and is valid only until return.
 * Must not block.
 */
typedef void (*UsbhsIntrHook)(const UsbhsIntrDone *done, const uint8_t *data);

typedef struct UsbhsIntrCfg
{
    uint8_t   addr;
    uint8_t   ep_addr;          /* bit7 = IN */
    uint16_t  mps;              /* wMaxPacketSize low 11 bits (1..1024) */
    uint8_t   b_interval;
    uint8_t   interval_mode;    /* 0: round down to power of 2, 1: raw (FS/LS only) */
    uint8_t   trace_budget;     /* Max normal txns to trace (errors always traced) */
    uint8_t   err_immediate;    /* Consecutive errors before pause */
    uint16_t  step;
    uint8_t   toggle;
    uint8_t   reserved;
    uint32_t  slot_ticks;       /* Offset from (micro)frame start */
    uint32_t  err_pause_ticks;  /* Pause duration after > err_immediate errors */
    uint64_t  start_at;         /* Earliest slot time (0 = next slot) */
    UsbhsIntrHook hook;         /* Optional */
} UsbhsIntrCfg;

typedef struct UsbhsIntrStats
{
    uint32_t issued;
    uint32_t data;              /* IN data received or OUT ACKed */
    uint32_t nak;
    uint32_t toggle_err;
    uint32_t stall;
    uint32_t error;
    uint32_t missed;            /* Missed scheduled frame */
    uint32_t late_max;
    uint8_t  state;             /* 0: closed, 1: active, 2: paused, 3: halted (STALL) */
    uint8_t  toggle;
    uint8_t  out_queued;
    uint8_t  reserved;
    uint32_t period;            /* Period in (micro)frames */
} UsbhsIntrStats;

typedef UsbhsIntrDone UsbhsReport;

typedef struct UsbhsEvent
{
    uint64_t time;
    uint32_t generation;
    uint32_t value;
    uint8_t  type;              /* USBHS_EV_* */
    uint8_t  speed;
    uint8_t  pipe;
    uint8_t  result;
} UsbhsEvent;

/* Trace record. */
typedef struct UsbhsTrace
{
    uint64_t time;
    uint64_t done;
    uint64_t nak_last;
    uint32_t nak_count;
    int32_t  late;
    uint16_t step;
    uint16_t length;
    uint8_t  kind;              /* USBHS_TRK_* */
    uint8_t  addr;
    uint8_t  ep;                /* bit7 = IN */
    uint8_t  token;             /* USBHS_TOK_* */
    uint8_t  result;            /* USBHS_TRR_* */
    uint8_t  toggle;
    uint8_t  pipe;              /* 0 = ctrl, 1+ = periodic */
    uint8_t  data_saved;
    uint8_t  setup[8];
    uint8_t  data[USBHS_TRACE_DATA_MAX];
    uint32_t generation;
} UsbhsTrace;

/* IN pipe pending CLEAR_FEATURE(ENDPOINT_HALT). */
typedef struct UsbhsIntrRecovery
{
    uint64_t at;
    uint32_t generation;
    uint8_t  pipe;
    uint8_t  address;
    uint8_t  endpoint;          /* bit7 = IN */
    uint8_t  error_count;
} UsbhsIntrRecovery;

typedef struct UsbhsPortInfo
{
    uint8_t  state;             /* USBHS_PORT_* */
    uint8_t  speed;
    uint8_t  configured;
    uint8_t  reserved;
    uint32_t generation;        /* Increments on every attach/recovery */
    uint64_t attach_time;
    uint64_t reset_start;
    uint64_t reset_done;
    uint32_t reset_ticks;
    uint32_t sof_count;
} UsbhsPortInfo;

typedef struct UsbhsStats
{
    uint32_t sched_late_count;  /* Issued >= 2us late */
    uint32_t sched_late_max;
    uint32_t txn_max_ticks;
    uint32_t recover_count;
    uint32_t issue_tim9;
    uint32_t issue_usbhs;
    uint32_t txn_timeout;       /* HOST_ACTION cleared without completion IRQ */
    uint32_t spurious_transfer;
    uint32_t sof_anomaly;       /* SOF frame number did not advance */
    uint32_t sof_reanchor;
    uint32_t connect_glitch;
    uint32_t report_drop;
    uint32_t event_drop;
    uint32_t trace_drop;
    uint32_t record_drop;
    uint32_t enum_error;
} UsbhsStats;

/* Hardware layer (src/v5f/usbhs_host.c) */

typedef void (*UsbhsLogFn)(const char *line);

/*
 * Initialize USBHS host and enable IRQs. TIM9 must already be running.
 * USBHS IRQ priority is set one preemption level below TIM9.
 */
void usbhs_host_init(UsbhsLogFn log);

void usbhs_host_poll(uint64_t now);

void usbhs_host_tim9_ch2(void);

/* Core API (src/v5f/usbhs_host_core.c) */

uint32_t usbhs_intr_period(uint8_t speed, uint8_t b_interval, uint8_t interval_mode);

void usbhs_ctrl_timing_default(UsbhsCtrlTiming *t);

/* Port */
void usbhs_port_info(UsbhsPortInfo *out);
bool usbhs_port_reset(uint8_t long_reset, uint16_t step);
uint32_t usbhs_frame_ticks(void);
bool usbhs_frame_at(uint64_t t, uint16_t *frame, uint8_t *uframe);
void usbhs_set_enum_state(bool configured, uint32_t enum_error);

/* Control transfers (one active at a time) */
bool usbhs_ctrl_submit(const UsbhsCtrlReq *req);
bool usbhs_ctrl_poll(UsbhsCtrlResult *out);
bool usbhs_ctrl_busy(void);
void usbhs_ctrl_abort(void);

/* Periodic endpoints (handle 1..USBHS_INTR_MAX, 0 on failure) */
uint8_t usbhs_intr_open(const UsbhsIntrCfg *cfg);
bool usbhs_intr_close(uint8_t pipe);
bool usbhs_intr_out_push(uint8_t pipe, const uint8_t *data, uint16_t len);
bool usbhs_intr_set_toggle(uint8_t pipe, uint8_t toggle);
bool usbhs_intr_resume(uint8_t pipe, uint64_t at);
bool usbhs_intr_stats(uint8_t pipe, UsbhsIntrStats *out);
bool usbhs_intr_recovery_pending(UsbhsIntrRecovery *out);
bool usbhs_intr_recovery_complete(uint8_t pipe, uint32_t generation, bool success);

bool usbhs_report_pop(UsbhsReport *out, uint8_t *data, uint16_t cap);
bool usbhs_event_pop(UsbhsEvent *out);

uint32_t usbhs_record_export(uint8_t *dst, uint32_t cap);
uint32_t usbhs_record_previous_export(uint8_t *dst, uint32_t cap);
void usbhs_record_clear(void);
bool usbhs_record_event(const UsbhsTrace *event);

void usbhs_stats(UsbhsStats *out);

void usbhs_recover(uint8_t cause);

#ifdef __cplusplus
}
#endif

#endif /* LAT_USBHS_HOST_H */
