/*
 * win_enum.c - Windows USB enumeration emulation
 *
 * Procedures are queued as Op sequences and executed from the head:
 *   - Control-transfer Ops compute a target timestamp (start_at) for the engine.
 *   - Bus resets, pipe opens, and OUT submissions run in the main loop at target time.
 *   - Layer transitions and length-dependent follow-ups prepend new Ops at the head.
 *   - Concurrent actions (XUSB X6/X7, HID D3a, LED, app init) run as Timed actions.
 */

#include "win_enum.h"

#include "app_init.h"
#include "enum_api.h"
#include "enum_port.h"
#include "lat_bytes.h"
#include "lat_fmt.h"
#include "lat_pc_proto.h"
#include "usb_desc.h"
#include "usbhs_host.h"
#include "win_profile.h"

/* Sizes */
#define OPQ_MAX            (64u)
#define TA_MAX             (24u)
#define CFG_BUF            (1024u)
#define BOS_BUF            (512u)
#define MSOS_BUF           (256u)
#define STR_SLOTS          (4u)
#define STR_BUF            (256u)
#define RPT_POOL           (4096u)
#define SCRATCH_BUF        (1024u)
#define REPORT_BUF         (USBHS_PKT_MAX)
#define EVQ_MAX            (16u)
#define MAX_ATTEMPTS       (3u)
#define LOG_MAX            (120u)

/* States */
#define ST_IDLE            (0u)   /* No device */
#define ST_RUN             (1u)   /* Enumeration or post-requests running */
#define ST_FAILED          (2u)   /* Halted after 3 failures */
#define ST_HUB             (3u)   /* Halted: hub detected */

#define WAIT_NONE          (0u)
#define WAIT_CTRL          (1u)
#define WAIT_RESET         (2u)
#define WAIT_RECOVERY      (3u)

/* Op kinds */
#define OK_CTRL            (1u)
#define OK_RESET           (2u)
#define OK_CALL            (3u)   /* arg0 = plan ID */
#define OK_PUSH            (4u)   /* arg0 = iface index, push data/len to OUT */
#define OK_OPEN            (5u)   /* arg0 = iface index, arg1 = 0 IN / 1 OUT */
#define OK_APP_GIP         (6u)   /* arg0 = GIP iface index */
#define OK_READY           (7u)
#define OK_WAIT_DS5        (8u)   /* DualSense: wait up to 16ms for 64-byte 0x01 */
#define OK_APP_DONE        (9u)

/* Timing rules */
#define WH_GAP             (0u)   /* prev_done + ticks */
#define WH_RESET           (1u)   /* reset_done + ticks */
#define WH_ANCHOR          (2u)   /* max(set_cfg_done + ticks, prev_done + WP_ANCHOR_MIN_GAP) */
#define WH_HUB             (3u)   /* Hub-layer request spacing */
#define WH_DEBOUNCE        (4u)   /* End of debounce */
#define WH_NOW             (5u)

/* Op flags */
#define OF_REQUIRED        (0x01u) /* Retry enumeration on failure */
#define OF_ADDR0           (0x02u) /* Send to address 0 */

/* Response handlers */
#define PO_NONE            (0u)
#define PO_DEV64           (1u)
#define PO_SETADDR         (2u)
#define PO_DEV18           (3u)
#define PO_CFG             (4u)
#define PO_CFG_RETRY       (5u)
#define PO_BOS             (6u)
#define PO_MSOS2           (7u)
#define PO_EE              (8u)
#define PO_MSOS1_HDR       (9u)   /* arg1 = 0 hub (H9), 1 usbccgp (C3) */
#define PO_MSOS1_FULL      (10u)
#define PO_STR             (11u)  /* arg0 = string index */
#define PO_SETCFG          (12u)
#define PO_STR_LEN         (13u)  /* Retry with wLength = bLength; arg0 = index, arg1 = sub-step */
#define PO_REPORT          (14u)  /* arg0 = iface index */
#define PO_C1_CFG9         (15u)

/* Plan IDs */
#define PL_AFTER_H3        (1u)
#define PL_AFTER_CFG       (2u)
#define PL_AFTER_BOS       (3u)
#define PL_MSOS1           (4u)
#define PL_FUNCTION        (5u)
#define PL_AFTER_SETCFG    (6u)
#define PL_DRIVERS         (7u)
#define PL_POST            (8u)

/* Timed actions */
#define TA_OPEN_IN         (1u)
#define TA_OPEN_OUT        (2u)
#define TA_PUSH            (3u)
#define TA_APP_SWITCH      (4u)
#define TA_RESUME_OUT      (5u)   /* idx = OUT pipe */

/* ENUM_PROGRESS failure codes */
#define ER_STALL           (1u)
#define ER_TIMEOUT         (2u)
#define ER_ERROR           (3u)
#define ER_HUB             (4u)
#define ER_BAD_DESC        (5u)
#define ER_PORT            (6u)

/* Vendor code table state (GET_DEVICE_INFO vendor_code_state) */
#define VC_UNKNOWN         (0u)
#define VC_CODE            (1u)
#define VC_NONE            (2u)

typedef struct Op
{
    uint8_t  kind;
    uint8_t  when;
    uint8_t  flags;
    uint8_t  post;
    uint16_t step;
    uint8_t  arg0;
    uint8_t  arg1;
    uint32_t ticks;
    uint8_t  setup[8];
    uint8_t  len;
    uint8_t  reserved[3];
    uint8_t  data[64];
} Op;

typedef struct Timed
{
    uint8_t  used;
    uint8_t  kind;
    uint8_t  idx;
    uint8_t  len;
    uint16_t step;
    uint8_t  ep_slot;
    uint8_t  reserved;
    uint64_t at;
    uint8_t  data[64];
} Timed;

typedef struct PendEv
{
    uint8_t type;
    uint8_t len;
    uint8_t body[8];
} PendEv;

typedef struct StrSlot
{
    uint8_t  used;
    uint8_t  index;
    uint16_t len;
    uint8_t  data[STR_BUF];
} StrSlot;

typedef struct WinEnum
{
    uint8_t  st;
    uint8_t  wait;
    uint8_t  attempt;
    uint8_t  force_reset2;
    uint8_t  speed;
    uint8_t  addr;           /* Current request address (0 before SET_ADDRESS) */
    uint8_t  new_addr;       /* Address assigned by SET_ADDRESS */
    uint8_t  addr_counter;   /* PC-W: increments per connection (1..127) */
    uint8_t  mps0;
    uint8_t  composite;
    uint8_t  classification;
    uint8_t  protocol;
    uint8_t  vc_state;       /* Current vendor code table state (VC_*) */
    uint8_t  vc_state_start; /* Vendor code table state at enum start */
    uint8_t  vendor_code;
    uint8_t  msos2_done;
    uint8_t  cfg_retried;
    uint8_t  cfg_complete;
    uint8_t  bos_retried;
    uint8_t  xusb_compat;    /* MS OS compat ID "XUSB10" on interface 0 */
    uint8_t  skip_ops;       /* Ops to skip when OK_WAIT_DS5 times out */
    uint8_t  ds5_seen;
    uint8_t  hub_progress;
    uint8_t  sony_app;       /* DS4/DualSense sdl_open state (ENUM_APP_*) */
    uint8_t  recovery_pipe;
    uint32_t recovery_generation;
    uint32_t gen;
    uint32_t enum_seq;
    uint32_t session_seen;
    uint64_t t_debounce;
    uint64_t t_prev;
    uint64_t t_reset_end;
    uint64_t t_setcfg;
    uint64_t t_ds5_wait;
    WinHostProfile prof;
    WinTiming tm;

    Op       cur;
    Op       q[OPQ_MAX];
    uint8_t  qn;
    uint8_t  ins;            /* Insertion cursor */
    Timed    ta[TA_MAX];

    uint8_t  dev64[64];
    uint16_t dev64_len;
    uint8_t  dev[18];
    uint16_t dev_len;
    UdDevice dd;
    uint8_t  cfg[CFG_BUF];
    uint16_t cfg_len;
    UdConfig cc;
    uint8_t  bos[BOS_BUF];
    uint16_t bos_len;
    uint8_t  msos1[MSOS_BUF];
    uint16_t msos1_len;
    StrSlot  str[STR_SLOTS];
    uint8_t  rpt[RPT_POOL];
    uint16_t rpt_used;
    uint16_t rpt_off[UD_IFACE_MAX];
    uint16_t rpt_len[UD_IFACE_MAX];
    UdHidInfo hid[UD_IFACE_MAX];
    uint8_t  pipe_in[UD_IFACE_MAX][UD_EP_IN_MAX];
    uint8_t  pipe_out[UD_IFACE_MAX];
    uint8_t  poll_in[UD_IFACE_MAX][UD_EP_IN_MAX];
    uint8_t  poll_out[UD_IFACE_MAX];
    uint8_t  func[UD_IFACE_MAX];
    uint8_t  out_stall_count[USBHS_INTR_MAX + 1u];
    uint8_t  paused[UD_IFACE_MAX][UD_EP_IN_MAX]; /* IN pipes paused by enum_set_polling */

    EnumDevice pub;
    EnumIface  extra_in[UD_IFACE_MAX][UD_EP_IN_MAX - 1u];
    EnumReportFn sink;
    PendEv   evq[EVQ_MAX];
    uint8_t  evn;
    uint8_t  scratch[SCRATCH_BUF];
    uint8_t  report[REPORT_BUF];
} WinEnum;

static WinEnum w;
/* Per-pipe ISR hook (read from USBHS ISR). */
static volatile EnumIsrHook s_hook[USBHS_INTR_MAX + 1u];

static void plan(uint8_t id, uint64_t now);

/* ------------------------------------------------------------------ */
/* Logging and events */
/* ------------------------------------------------------------------ */

static void put_hex(LatStr *s, uint32_t v, uint32_t digits)
{
    static const char h[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    uint32_t i;

    for (i = digits; i > 0u; i--)
    {
        lat_str_putc(s, h[(v >> ((i - 1u) * 4u)) & 0xFu]);
    }
}

static const char *speed_name(uint8_t sp)
{
    return (sp == USBHS_SPEED_HS) ? "HS" : (sp == USBHS_SPEED_FS) ? "FS" : (sp == USBHS_SPEED_LS) ? "LS" : "none";
}

static void log_kv(const char *head, const char *k1, uint32_t v1, const char *k2, uint32_t v2)
{
    char buf[LOG_MAX + 1u];
    LatStr s;

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[enum] ");
    lat_str_puts(&s, head);
    if (k1 != (const char *)0)
    {
        lat_str_puts(&s, k1);
        lat_str_u32(&s, v1);
    }
    if (k2 != (const char *)0)
    {
        lat_str_puts(&s, k2);
        lat_str_u32(&s, v2);
    }
    enum_port_log(buf);
}

static void ev_push(uint8_t type, const uint8_t *body, uint8_t len)
{
    PendEv *e;

    if (w.evn >= EVQ_MAX)
    {
        enum_port_log("[enum] WARN event queue full, dropped");
        return;
    }
    e = &w.evq[w.evn++];
    e->type = type;
    e->len = len;
    lat_bytes_copy(e->body, body, len);
}

static void ev_flush(void)
{
    uint32_t sess = enum_port_session();

    if (sess != w.session_seen)
    {
        /* New session: discard events queued for previous session */
        w.session_seen = sess;
        w.evn = 0u;
        return;
    }
    while (w.evn > 0u)
    {
        if (!enum_port_emit(w.evq[0].type, w.evq[0].body, w.evq[0].len))
        {
            return;
        }
        w.evn--;
        if (w.evn > 0u)
        {
            uint8_t i;

            for (i = 0u; i < w.evn; i++)
            {
                w.evq[i] = w.evq[i + 1u];
            }
        }
    }
}

static void progress(uint16_t step, uint8_t result)
{
    uint8_t b[8];

    lat_bytes_fill(b, 0u, sizeof(b));
    lat_put_u32(&b[0], w.gen);
    lat_put_u16(&b[4], step);
    b[6] = result;
    ev_push(LAT_PC_EVT_ENUM_PROGRESS, b, 8u);
}

static void error_event(uint16_t code, uint32_t detail)
{
    uint8_t b[8];

    lat_bytes_fill(b, 0u, sizeof(b));
    lat_put_u16(&b[0], code);
    lat_put_u32(&b[4], detail);
    ev_push(LAT_PC_EVT_ERROR, b, 8u);
}

/* ------------------------------------------------------------------ */
/* Op queue */
/* ------------------------------------------------------------------ */

static void q_clear(void)
{
    w.qn = 0u;
    w.ins = 0u;
    w.skip_ops = 0u;
}

/* Reset insertion cursor to head (call before response handling and planning). */
static void ins_begin(void)
{
    w.ins = 0u;
}

static Op *ins_op(uint8_t kind, uint8_t when, uint32_t ticks, uint16_t step)
{
    Op *op;
    uint8_t i;

    if (w.qn >= OPQ_MAX)
    {
        enum_port_log("[enum] ERROR op queue full");
        return (Op *)0;
    }
    for (i = w.qn; i > w.ins; i--)
    {
        w.q[i] = w.q[i - 1u];
    }
    op = &w.q[w.ins];
    lat_bytes_fill((uint8_t *)op, 0u, (uint32_t)sizeof(*op));
    op->kind = kind;
    op->when = when;
    op->ticks = ticks;
    op->step = step;
    w.qn++;
    w.ins++;
    return op;
}

static Op *ins_ctrl(uint8_t when, uint32_t ticks, uint16_t step, uint8_t bm, uint8_t req, uint16_t value,
                    uint16_t index, uint16_t length, uint8_t post)
{
    Op *op = ins_op(OK_CTRL, when, ticks, step);

    if (op != (Op *)0)
    {
        op->setup[0] = bm;
        op->setup[1] = req;
        lat_put_u16(&op->setup[2], value);
        lat_put_u16(&op->setup[4], index);
        lat_put_u16(&op->setup[6], length);
        op->post = post;
    }
    return op;
}

static Op *ins_get_desc(uint8_t when, uint32_t ticks, uint16_t step, uint8_t type, uint8_t index, uint16_t langid,
                        uint16_t length, uint8_t post)
{
    return ins_ctrl(when, ticks, step, 0x80u, 0x06u, (uint16_t)(((uint16_t)type << 8) | index), langid, length, post);
}

static void q_pop(Op *out)
{
    uint8_t i;

    *out = w.q[0];
    for (i = 1u; i < w.qn; i++)
    {
        w.q[i - 1u] = w.q[i];
    }
    w.qn--;
}

/* ------------------------------------------------------------------ */
/* Timed actions */
/* ------------------------------------------------------------------ */

static Timed *ta_add(uint8_t kind, uint8_t idx, uint64_t at, uint16_t step)
{
    uint8_t i;

    for (i = 0u; i < TA_MAX; i++)
    {
        if (!w.ta[i].used)
        {
            Timed *t = &w.ta[i];

            lat_bytes_fill((uint8_t *)t, 0u, (uint32_t)sizeof(*t));
            t->used = 1u;
            t->kind = kind;
            t->idx = idx;
            t->at = at;
            t->step = step;
            return t;
        }
    }
    enum_port_log("[enum] ERROR timed action table full");
    return (Timed *)0;
}

static void ta_open_iface(uint8_t idx, uint64_t at, uint16_t step, bool open_in, bool open_out)
{
    const UdIface *f = &w.cc.iface[idx];
    uint8_t order;

    for (order = 0u; order < f->intr_count; order++)
    {
        uint8_t k;

        if (open_in)
        {
            for (k = 0u; k < f->ep_in_count; k++)
            {
                if (w.poll_in[idx][k] && (f->in_order[k] == order))
                {
                    Timed *t = ta_add(TA_OPEN_IN, idx, at, step);

                    if (t != (Timed *)0)
                    {
                        t->ep_slot = k;
                    }
                }
            }
        }
        if (open_out && w.poll_out[idx] && (f->ep_out != 0u) && (f->out_order == order))
        {
            (void)ta_add(TA_OPEN_OUT, idx, at, step);
        }
    }
}

static void select_periodic(void)
{
    uint16_t iface;
    uint8_t used = 0u;

    lat_bytes_fill((uint8_t *)w.poll_in, 0u, sizeof(w.poll_in));
    lat_bytes_fill(w.poll_out, 0u, sizeof(w.poll_out));
    for (iface = 0u; iface <= 0xFFu; iface++)
    {
        uint8_t i;

        for (i = 0u; i < w.cc.iface_count; i++)
        {
            const UdIface *f = &w.cc.iface[i];
            uint8_t order;

            if (f->number != (uint8_t)iface)
            {
                continue;
            }
            if (w.func[i] == ENUM_FUNC_NONE)
            {
                continue;
            }
            for (order = 0u; order < f->intr_count; order++)
            {
                uint8_t k;

                for (k = 0u; k < f->ep_in_count; k++)
                {
                    if (f->in_order[k] == order)
                    {
                        if (used < USBHS_INTR_MAX)
                        {
                            w.poll_in[i][k] = 1u;
                            used++;
                        }
                        else
                        {
                            log_kv("periodic endpoint limit", " iface=", f->number, " ep=", f->ep_in[k]);
                        }
                    }
                }
                if ((f->ep_out != 0u) && (f->out_order == order))
                {
                    if (used < USBHS_INTR_MAX)
                    {
                        w.poll_out[i] = 1u;
                        used++;
                    }
                    else
                    {
                        log_kv("periodic endpoint limit", " iface=", f->number, " ep=", f->ep_out);
                    }
                }
            }
        }
    }
}

static void log_per_iface_overflow(void)
{
    uint16_t pos;
    uint8_t iface = 0u;
    uint8_t in_count = 0u;
    bool active = false;

    if ((w.cfg_len < 9u) || (w.cfg[0] < 9u))
    {
        return;
    }
    pos = w.cfg[0];
    while ((uint32_t)pos + 2u <= w.cfg_len)
    {
        uint8_t bl = w.cfg[pos];
        uint8_t type = w.cfg[pos + 1u];

        if ((bl < 2u) || ((uint32_t)pos + bl > w.cfg_len))
        {
            break;
        }
        if ((type == 0x04u) && (bl >= 9u))
        {
            iface = w.cfg[pos + 2u];
            active = w.cfg[pos + 3u] == 0u;
            in_count = 0u;
        }
        else if (active && (type == 0x05u) && (bl >= 7u) &&
                 ((w.cfg[pos + 2u] & 0x80u) != 0u) && ((w.cfg[pos + 3u] & 0x03u) == 0x03u))
        {
            in_count++;
            if (in_count > UD_EP_IN_MAX)
            {
                log_kv("IN endpoint limit", " iface=", iface, " ep=", w.cfg[pos + 2u]);
            }
        }
        pos = (uint16_t)(pos + bl);
    }
}

static bool ta_open_pending(void)
{
    uint8_t i;

    for (i = 0u; i < TA_MAX; i++)
    {
        if (w.ta[i].used && ((w.ta[i].kind == TA_OPEN_IN) || (w.ta[i].kind == TA_OPEN_OUT)))
        {
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* Device info */
/* ------------------------------------------------------------------ */

static uint8_t func_of(const UdIface *f)
{
    if (f->cls == 0x03u)
    {
        return ENUM_FUNC_HID;
    }
    if ((f->cls == 0xFFu) && (f->subcls == 0x5Du) && (f->proto == 0x01u))
    {
        return ENUM_FUNC_XUSB;
    }
    if ((f->cls == 0xFFu) && (f->subcls == 0x47u) && (f->proto == 0xD0u) && (f->number == 0u))
    {
        return ENUM_FUNC_GIP; /* Only interface 0 is treated as GIP (xpad) */
    }
    return ENUM_FUNC_NONE;
}

/* Device classification; call after parsing configuration descriptor (H7). */
static void classify(void)
{
    const UdIface *f0 = (w.cc.iface_count > 0u) ? &w.cc.iface[0] : (const UdIface *)0;
    bool iad = (w.dd.cls == 0xEFu) && (w.dd.subcls == 0x02u) && (w.dd.proto == 0x01u);
    uint8_t i;

    w.composite = (uint8_t)((((w.dd.cls == 0x00u) || iad) && (w.cc.num_interfaces >= 2u)) ? 1u : 0u);
    for (i = 0u; i < w.cc.iface_count; i++)
    {
        w.func[i] = func_of(&w.cc.iface[i]);
    }
    if (w.composite)
    {
        w.classification = ENUM_CLASS_COMPOSITE;
    }
    else if ((f0 != (const UdIface *)0) && (w.dd.cls == 0x00u) && (w.cc.iface_count == 1u) && (f0->cls == 0x03u))
    {
        w.classification = ENUM_CLASS_SINGLE_HID;
    }
    else if ((f0 != (const UdIface *)0) && ((w.func[0] == ENUM_FUNC_XUSB) || w.xusb_compat))
    {
        w.classification = ENUM_CLASS_SINGLE_XUSB;
        w.func[0] = ENUM_FUNC_XUSB;
    }
    else if ((f0 != (const UdIface *)0) && (w.func[0] == ENUM_FUNC_GIP))
    {
        w.classification = ENUM_CLASS_GIP;
    }
    else
    {
        w.classification = ENUM_CLASS_OTHER;
        for (i = 0u; i < w.cc.iface_count; i++)
        {
            w.func[i] = ENUM_FUNC_NONE; /* Configured, but no IN polling */
        }
    }

    /* Protocol detection (selects app init and default measurement target) */
    w.protocol = ENUM_PROTO_UNKNOWN;
    if ((w.dd.vid == 0x057Eu) && (w.dd.pid == 0x2009u))
    {
        w.protocol = ENUM_PROTO_SWITCH_PRO;
    }
    else if ((w.dd.vid == 0x054Cu) &&
             ((w.dd.pid == 0x05C4u) || (w.dd.pid == 0x09CCu) || (w.dd.pid == 0x0BA0u) || (w.dd.pid == 0x05C5u)))
    {
        w.protocol = ENUM_PROTO_DS4;
    }
    else if ((w.dd.vid == 0x054Cu) && ((w.dd.pid == 0x0CE6u) || (w.dd.pid == 0x0DF2u) || (w.dd.pid == 0x0E5Fu)))
    {
        w.protocol = ENUM_PROTO_DUALSENSE;
    }
    else
    {
        for (i = 0u; i < w.cc.iface_count; i++)
        {
            if (w.func[i] == ENUM_FUNC_XUSB)
            {
                w.protocol = ENUM_PROTO_XUSB;
                break;
            }
            if (w.func[i] == ENUM_FUNC_GIP)
            {
                w.protocol = ENUM_PROTO_GIP;
                break;
            }
        }
        if (w.protocol == ENUM_PROTO_UNKNOWN)
        {
            for (i = 0u; i < w.cc.iface_count; i++)
            {
                if (w.func[i] == ENUM_FUNC_HID)
                {
                    w.protocol = ENUM_PROTO_GENERIC_HID;
                    break;
                }
            }
        }
    }
}

static uint8_t first_iface_with(uint8_t func)
{
    uint8_t i;

    for (i = 0u; i < w.cc.iface_count; i++)
    {
        if ((w.func[i] == func) && (w.cc.iface[i].ep_in_count != 0u))
        {
            return i;
        }
    }
    return 0xFFu;
}

/* Default measurement target (interface carrying input reports). */
static uint8_t pick_target(void)
{
    uint8_t i;

    if (w.protocol == ENUM_PROTO_XUSB)
    {
        return first_iface_with(ENUM_FUNC_XUSB);
    }
    if (w.protocol == ENUM_PROTO_GIP)
    {
        return first_iface_with(ENUM_FUNC_GIP);
    }
    for (i = 0u; i < w.cc.iface_count; i++)
    {
        if ((w.func[i] == ENUM_FUNC_HID) && (w.cc.iface[i].ep_in_count != 0u) && w.hid[i].gamepad)
        {
            return i;
        }
    }
    return first_iface_with(ENUM_FUNC_HID);
}

static void publish(void)
{
    EnumDevice *p = &w.pub;
    uint8_t i;
    uint8_t tgt;

    lat_bytes_fill((uint8_t *)p, 0u, (uint32_t)sizeof(*p));
    lat_bytes_fill((uint8_t *)w.extra_in, 0u, (uint32_t)sizeof(w.extra_in));
    p->generation = w.gen;
    p->enum_seq = w.enum_seq;
    p->ready = 1u;
    p->speed = w.speed;
    p->address = w.addr;
    p->classification = w.classification;
    p->protocol = w.protocol;
    p->app_init = (w.sony_app != ENUM_APP_NONE) ? w.sony_app : app_state();
    p->device_class = w.dd.cls;
    p->mps0 = w.mps0;
    p->vid = w.dd.vid;
    p->pid = w.dd.pid;
    p->bcd_usb = w.dd.bcd_usb;
    p->bcd_device = w.dd.bcd_device;
    p->num_interfaces = w.cc.num_interfaces;
    p->iface_count = (w.cc.iface_count > ENUM_IFACE_MAX) ? ENUM_IFACE_MAX : w.cc.iface_count;
    p->frame_ticks = usbhs_frame_ticks();
    for (i = 0u; i < p->iface_count; i++)
    {
        const UdIface *f = &w.cc.iface[i];
        EnumIface *e = &p->iface[i];

        e->number = f->number;
        e->alt = f->alt;
        e->cls = f->cls;
        e->subcls = f->subcls;
        e->proto = f->proto;
        e->function = w.func[i];
        e->ep_in = f->ep_in[0];
        e->ep_out = f->ep_out;
        e->in_mps = f->in_mps[0];
        e->out_mps = f->out_mps;
        e->in_interval = f->in_interval[0];
        e->out_interval = f->out_interval;
        e->pipe_in = w.pipe_in[i][0];
        e->pipe_out = w.pipe_out[i];
        e->in_period = (w.pipe_in[i][0] != 0u) ? usbhs_intr_period(w.speed, f->in_interval[0], w.prof.interval_mode) : 0u;
        e->report_desc_len = w.rpt_len[i];
        e->report_desc = (w.rpt_len[i] > 0u) ? &w.rpt[w.rpt_off[i]] : (const uint8_t *)0;
        {
            uint8_t k;

            for (k = 1u; k < f->ep_in_count; k++)
            {
                EnumIface *x = &w.extra_in[i][k - 1u];

                *x = *e;
                x->ep_in = f->ep_in[k];
                x->in_mps = f->in_mps[k];
                x->in_interval = f->in_interval[k];
                x->pipe_in = w.pipe_in[i][k];
                x->in_period = (x->pipe_in != 0u) ? usbhs_intr_period(w.speed, x->in_interval, w.prof.interval_mode) : 0u;
            }
        }
    }
    tgt = pick_target();
    p->target_iface = (tgt < w.cc.iface_count) ? w.cc.iface[tgt].number : 0xFFu;
    p->target_ep_in = (tgt < w.cc.iface_count) ? w.cc.iface[tgt].ep_in[0] : 0u;
}

/* ------------------------------------------------------------------ */
/* Reset and state initialization */
/* ------------------------------------------------------------------ */

static void clear_hooks(void)
{
    uint8_t i;

    for (i = 0u; i <= USBHS_INTR_MAX; i++)
    {
        s_hook[i] = (EnumIsrHook)0;
    }
}

/*
 * Clear pipe numbers without closing: engine closes all periodic pipes on
 * disconnect/bus reset, so avoid closing a handle that may be reused.
 */
static void forget_pipes(void)
{
    clear_hooks();
    lat_bytes_fill((uint8_t *)w.pipe_in, 0u, sizeof(w.pipe_in));
    lat_bytes_fill(w.pipe_out, 0u, sizeof(w.pipe_out));
}

/* Discard per-attempt state (keep learned vendor code and connection info). */
static void clear_attempt(void)
{
    q_clear();
    lat_bytes_fill((uint8_t *)w.ta, 0u, (uint32_t)sizeof(w.ta));
    forget_pipes();
    app_reset();
    if (w.pub.ready)
    {
        usbhs_set_enum_state(false, 0u);
    }
    w.pub.ready = 0u;
    w.wait = WAIT_NONE;
    w.addr = 0u;
    w.dev64_len = 0u;
    w.dev_len = 0u;
    w.cfg_len = 0u;
    w.bos_len = 0u;
    w.msos1_len = 0u;
    w.rpt_used = 0u;
    w.msos2_done = 0u;
    w.cfg_retried = 0u;
    w.cfg_complete = 0u;
    w.bos_retried = 0u;
    w.xusb_compat = 0u;
    w.ds5_seen = 0u;
    w.composite = 0u;
    w.classification = ENUM_CLASS_OTHER;
    w.protocol = ENUM_PROTO_UNKNOWN;
    w.hub_progress = 0u;
    w.sony_app = ENUM_APP_NONE;
    w.mps0 = 0u;
    w.t_ds5_wait = 0u;
    w.recovery_pipe = 0u;
    w.recovery_generation = 0u;
    lat_bytes_fill((uint8_t *)&w.dd, 0u, (uint32_t)sizeof(w.dd));
    lat_bytes_fill((uint8_t *)&w.cc, 0u, (uint32_t)sizeof(w.cc));
    lat_bytes_fill((uint8_t *)w.str, 0u, (uint32_t)sizeof(w.str));
    lat_bytes_fill((uint8_t *)w.rpt_off, 0u, (uint32_t)sizeof(w.rpt_off));
    lat_bytes_fill((uint8_t *)w.rpt_len, 0u, (uint32_t)sizeof(w.rpt_len));
    lat_bytes_fill((uint8_t *)w.hid, 0u, (uint32_t)sizeof(w.hid));
    lat_bytes_fill(w.func, 0u, sizeof(w.func));
    lat_bytes_fill(w.out_stall_count, 0u, sizeof(w.out_stall_count));
    lat_bytes_fill((uint8_t *)w.paused, 0u, sizeof(w.paused));
}

static void load_profile(void)
{
    static const WinTiming pcw = WP_PCW_TIMING_INIT;

    enum_port_profile(&w.prof);
    w.tm = pcw;
}

static void trace_settings(uint64_t now, uint8_t state, uint8_t code)
{
    UsbhsTrace t;

    lat_bytes_fill((uint8_t *)&t, 0u, (uint32_t)sizeof(t));
    t.kind = USBHS_TRK_ENUM_SETTINGS;
    t.time = now;
    t.data_saved = 10u;
    t.data[0] = 0u;
    t.data[1] = w.prof.post_requests;
    t.data[2] = w.prof.interval_mode;
    t.data[3] = w.prof.reset2_policy;
    t.data[4] = w.prof.app_switch;
    t.data[5] = w.prof.app_sony;
    t.data[8] = state;
    t.data[9] = code;
    (void)usbhs_record_event(&t);
}

/* Start one enumeration attempt: bus reset (H2) -> H3 -> plan. */
static void start_attempt(uint8_t reset_when)
{
    Op *op;

    clear_attempt();
    trace_settings(enum_port_now(), 0u, 0u);
    w.st = ST_RUN;
    ins_begin();
    (void)ins_op(OK_RESET, reset_when, 0u, WS('H', 2, 0));
    op = ins_get_desc(WH_RESET, w.tm.first, WS('H', 3, 0), 0x01u, 0u, 0u, 64u, PO_DEV64);
    if (op != (Op *)0)
    {
        op->flags = OF_REQUIRED | OF_ADDR0;
    }
    op = ins_op(OK_CALL, WH_NOW, 0u, 0u);
    if (op != (Op *)0)
    {
        op->arg0 = PL_AFTER_H3;
    }
}

static void give_up(uint16_t step, uint8_t result)
{
    uint32_t code = ((uint32_t)step << 8) | result;

    clear_attempt();
    w.st = ST_FAILED;
    usbhs_set_enum_state(false, code);
    progress(step, result);
    error_event(LAT_PC_ERR_ENUM_FAILED, ((uint32_t)step << 16) | result);
    log_kv("enumeration failed 3 times, stopped", " step=", step, " result=", result);
}

/* Required request failed: retry with 2-reset sequence, up to 3 attempts. */
static void required_failed(uint16_t step, uint8_t result)
{
    log_kv("required request failed", " step=", step, " result=", result);
    if (w.attempt >= MAX_ATTEMPTS)
    {
        give_up(step, result);
        return;
    }
    w.attempt++;
    w.force_reset2 = 1u;
    log_kv("retry enumeration", " attempt=", w.attempt, (const char *)0, 0u);
    start_attempt(WH_NOW);
}

/* ------------------------------------------------------------------ */
/* Timing */
/* ------------------------------------------------------------------ */

static uint64_t op_time(const Op *op, uint64_t now)
{
    uint64_t t;

    switch (op->when)
    {
    case WH_GAP:
        return w.t_prev + op->ticks;
    case WH_RESET:
        return w.t_reset_end + op->ticks;
    case WH_ANCHOR:
        t = w.t_setcfg + op->ticks;
        if ((int64_t)((w.t_prev + WP_ANCHOR_MIN_GAP) - t) > 0)
        {
            t = w.t_prev + WP_ANCHOR_MIN_GAP;
        }
        return t;
    case WH_HUB:
        return w.t_prev + WP_PCW_HUB_GAP;
    case WH_DEBOUNCE:
        return w.t_debounce;
    default:
        return now;
    }
}

static void ctrl_timing(UsbhsCtrlTiming *t)
{
    usbhs_ctrl_timing_default(t);
    /* PC-W: back-to-back stages; NAK retry ~9..14us. */
    t->nak_first.ticks = WP_PCW_NAK;
    t->nak_cont.ticks = WP_PCW_NAK;
    t->timeout_ticks = w.tm.ctrl_timeout;
}

/* ------------------------------------------------------------------ */
/* Response buffers */
/* ------------------------------------------------------------------ */

static StrSlot *str_slot(uint8_t index, bool create)
{
    uint8_t i;

    for (i = 0u; i < STR_SLOTS; i++)
    {
        if (w.str[i].used && (w.str[i].index == index))
        {
            return &w.str[i];
        }
    }
    if (!create)
    {
        return (StrSlot *)0;
    }
    for (i = 0u; i < STR_SLOTS; i++)
    {
        if (!w.str[i].used)
        {
            w.str[i].used = 1u;
            w.str[i].index = index;
            w.str[i].len = 0u;
            return &w.str[i];
        }
    }
    return (StrSlot *)0;
}

static void dest_for(const Op *op, uint8_t **buf, uint16_t *cap)
{
    switch (op->post)
    {
    case PO_DEV64:
        *buf = w.dev64;
        *cap = (uint16_t)sizeof(w.dev64);
        return;
    case PO_DEV18:
        *buf = w.dev;
        *cap = (uint16_t)sizeof(w.dev);
        return;
    case PO_CFG:
    case PO_CFG_RETRY:
    case PO_C1_CFG9:
        *buf = w.cfg;
        *cap = (uint16_t)sizeof(w.cfg);
        return;
    case PO_BOS:
        *buf = w.bos;
        *cap = (uint16_t)sizeof(w.bos);
        return;
    case PO_MSOS1_HDR:
    case PO_MSOS1_FULL:
        *buf = w.msos1;
        *cap = (uint16_t)sizeof(w.msos1);
        return;
    case PO_STR:
    {
        StrSlot *s = str_slot(op->arg0, true);

        if (s != (StrSlot *)0)
        {
            *buf = s->data;
            *cap = (uint16_t)sizeof(s->data);
            return;
        }
        break;
    }
    case PO_REPORT:
        if (w.rpt_used < RPT_POOL)
        {
            *buf = &w.rpt[w.rpt_used];
            *cap = (uint16_t)(RPT_POOL - w.rpt_used);
            return;
        }
        break;
    default:
        break;
    }
    *buf = w.scratch;
    *cap = (uint16_t)sizeof(w.scratch);
}

/* ------------------------------------------------------------------ */
/* Response handling */
/* ------------------------------------------------------------------ */

static uint8_t res_code(uint8_t r)
{
    return (r == USBHS_RES_STALL) ? ER_STALL : (r == USBHS_RES_TIMEOUT) ? ER_TIMEOUT : ER_ERROR;
}

static void learn_vendor(uint8_t state, uint8_t code)
{
    enum_port_vendor_learn(w.dd.vid, w.dd.pid, w.dd.bcd_device, state, code);
    w.vc_state = (state == LAT_PC_VENDOR_STATE_CODE) ? VC_CODE : VC_NONE;
    w.vendor_code = code;
    log_kv("learned MS OS vendor code", " state=", state, " code=", code);
}

/* Request MS OS 1.0 compat ID header (16 bytes) for H9a and C3. */
static void ins_msos1_header(uint8_t when, uint32_t ticks, uint8_t layer)
{
    Op *op = ins_ctrl(when, ticks, (layer == 0u) ? WS('H', 9, 1) : WS('C', 3, 0), 0xC0u, w.vendor_code, 0u,
                      0x0004u, 16u, PO_MSOS1_HDR);

    if (op != (Op *)0)
    {
        op->arg1 = layer;
    }
}

static void post_ok(const Op *op, const UsbhsCtrlResult *r, uint64_t now)
{
    uint16_t n = r->actual;

    ins_begin();
    switch (op->post)
    {
    case PO_DEV64:
        w.dev64_len = n;
        if (!ud_parse_device(w.dev64, n, &w.dd) || (n < 8u))
        {
            required_failed(op->step, ER_BAD_DESC);
            return;
        }
        w.mps0 = w.dev64[7];
        if ((w.mps0 != 8u) && (w.mps0 != 16u) && (w.mps0 != 32u) && (w.mps0 != 64u))
        {
            w.mps0 = (w.speed == USBHS_SPEED_LS) ? 8u : 64u;
        }
        break;
    case PO_SETADDR:
        w.addr = w.new_addr;
        break;
    case PO_DEV18:
        w.dev_len = n;
        if ((n < 18u) || !ud_parse_device(w.dev, n, &w.dd))
        {
            required_failed(op->step, ER_BAD_DESC);
            return;
        }
        break;
    case PO_CFG:
    case PO_CFG_RETRY:
        if (n > w.cfg_len)
        {
            w.cfg_len = (n > CFG_BUF) ? (uint16_t)CFG_BUF : n;
        }
        if ((op->post == PO_CFG) && (op->flags & OF_REQUIRED) && (n < 9u))
        {
            required_failed(op->step, ER_BAD_DESC);
            return;
        }
        break;
    case PO_C1_CFG9:
        if (n < 9u)
        {
            required_failed(op->step, ER_BAD_DESC);
            return;
        }
        break;
    case PO_BOS:
        w.bos_len = (n > BOS_BUF) ? (uint16_t)BOS_BUF : n;
        break;
    case PO_EE:
    {
        uint8_t vc;

        if (ud_msos10_string(w.scratch, n, &vc))
        {
            learn_vendor(LAT_PC_VENDOR_STATE_CODE, vc);
            if (!w.composite)
            {
                ins_msos1_header(WH_HUB, 0u, 0u); /* Follow with H9a */
            }
        }
        else
        {
            learn_vendor(LAT_PC_VENDOR_STATE_NONE, 0u);
        }
        break;
    }
    case PO_MSOS1_HDR:
    {
        uint32_t dw = ud_msos10_length(w.msos1, n);

        w.msos1_len = (n > MSOS_BUF) ? (uint16_t)MSOS_BUF : n;
        if (dw > 16u)
        {
            uint16_t ll = (dw > 0xFFFFu) ? 0xFFFFu : (uint16_t)dw;

            (void)ins_ctrl((op->arg1 == 0u) ? WH_HUB : WH_GAP, (op->arg1 == 0u) ? 0u : WP_C3_FULL_GAP,
                           (op->arg1 == 0u) ? WS('H', 9, 2) : WS('C', 3, 1), 0xC0u, w.vendor_code, 0u, 0x0004u, ll,
                           PO_MSOS1_FULL);
        }
        break;
    }
    case PO_MSOS1_FULL:
        w.msos1_len = (n > MSOS_BUF) ? (uint16_t)MSOS_BUF : n;
        w.xusb_compat = (uint8_t)(ud_msos10_has_compat(w.msos1, w.msos1_len, 0u, "XUSB10") ? 1u : 0u);
        break;
    case PO_STR:
    {
        StrSlot *s = str_slot(op->arg0, false);

        if (s != (StrSlot *)0)
        {
            s->len = (n > STR_BUF) ? (uint16_t)STR_BUF : n;
        }
        break;
    }
    case PO_SETCFG:
        w.t_setcfg = (r->status_done != 0u) ? r->status_done : r->end_time;
        progress(op->step, 0u);
        log_kv("configured", " addr=", w.addr, " class=", w.classification);
        break;
    case PO_STR_LEN:
        if ((n >= 1u) && (w.scratch[0] >= 2u))
        {
            (void)ins_get_desc(WH_GAP, (op->arg1 == 0u) ? WP_C4_SECOND_GAP : WP_X2B_GAP, (uint16_t)(op->step + 1u),
                               0x03u, op->arg0, 0x0409u, w.scratch[0], PO_NONE);
        }
        break;
    case PO_REPORT:
    {
        uint8_t i = op->arg0;

        if (i < UD_IFACE_MAX)
        {
            uint16_t keep = n;

            if (keep > (uint16_t)(RPT_POOL - w.rpt_used))
            {
                keep = (uint16_t)(RPT_POOL - w.rpt_used);
            }
            w.rpt_off[i] = w.rpt_used;
            w.rpt_len[i] = keep;
            w.rpt_used = (uint16_t)(w.rpt_used + keep);
            ud_parse_hid_report(&w.rpt[w.rpt_off[i]], keep, &w.hid[i]);
        }
        break;
    }
    default:
        break;
    }
    (void)now;
}

/* Optional request failed (STALL, etc.); advance to next step. */
static void post_fail(const Op *op, const UsbhsCtrlResult *r)
{
    ins_begin();
    if (op->post == PO_EE)
    {
        /* STALL or 5s NAK timeout: record as no MS OS descriptor */
        learn_vendor(LAT_PC_VENDOR_STATE_NONE, 0u);
    }
    if ((op->post == PO_CFG_RETRY) || (op->post == PO_BOS))
    {
        /* H7 retry failure is recovered in C1; BOS STALL continues. */
    }
    log_kv("request not completed (continue)", " step=", op->step, " result=", r->result);
}

static void ctrl_done(const UsbhsCtrlResult *r, uint64_t now)
{
    Op op = w.cur;

    w.wait = WAIT_NONE;
    if (r->generation != w.gen)
    {
        return;
    }
    if ((r->result == USBHS_RES_DETACHED) || (r->result == USBHS_RES_ABORTED))
    {
        return; /* Disconnect or reset already handled */
    }
    w.t_prev = (r->status_done != 0u) ? r->status_done : r->end_time;
    if (r->result == USBHS_RES_OK)
    {
        post_ok(&op, r, now);
        return;
    }
    if ((op.flags & OF_REQUIRED) != 0u)
    {
        required_failed(op.step, res_code(r->result));
        return;
    }
    post_fail(&op, r);
}

/* ------------------------------------------------------------------ */
/* Planning */
/* ------------------------------------------------------------------ */

static Op *ins_call(uint8_t id)
{
    Op *op = ins_op(OK_CALL, WH_NOW, 0u, 0u);

    if (op != (Op *)0)
    {
        op->arg0 = id;
    }
    return op;
}

/* H4-H7 */
static void plan_after_h3(void)
{
    bool reset2;
    Op *op;

    if (w.dd.cls == 0x09u)
    {
        /* Hubs are not supported */
        clear_attempt();
        w.st = ST_HUB;
        usbhs_set_enum_state(false, ((uint32_t)WS('H', 3, 0) << 8) | ER_HUB);
        progress(WS('H', 3, 0), ER_HUB);
        error_event(LAT_PC_ERR_HUB_UNSUPPORTED, 0u);
        enum_port_log("[enum] hub (class 0x09) is not supported, stopped");
        return;
    }
    {
        char buf[LOG_MAX + 1u];
        LatStr s;

        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[enum] H3 bcdUSB=");
        put_hex(&s, w.dd.bcd_usb, 4u);
        lat_str_puts(&s, " mps0=");
        lat_str_u32(&s, w.mps0);
        lat_str_puts(&s, " class=");
        put_hex(&s, w.dd.cls, 2u);
        lat_str_puts(&s, " len=");
        lat_str_u32(&s, w.dev64_len);
        enum_port_log(buf);
    }
    progress(WS('H', 3, 0), 0u);

    /* H4: second reset (bcdUSB < 0x0201 or retry) */
    reset2 = (w.force_reset2 != 0u) || (w.dd.bcd_usb < 0x0201u);
    if (w.tm.addr_fixed != 0u)
    {
        w.new_addr = w.tm.addr_fixed;
    }
    else
    {
        w.addr_counter = (uint8_t)((w.addr_counter >= 127u) ? 1u : (w.addr_counter + 1u));
        w.new_addr = w.addr_counter;
    }
    ins_begin();
    if (reset2)
    {
        (void)ins_op(OK_RESET, WH_GAP, w.tm.reset2_gap, WS('H', 4, 0));
        op = ins_ctrl(WH_RESET, w.tm.addr, WS('H', 5, 0), 0x00u, 0x05u, w.new_addr, 0u, 0u, PO_SETADDR);
    }
    else
    {
        op = ins_ctrl(WH_GAP, w.tm.addr_noreset, WS('H', 5, 0), 0x00u, 0x05u, w.new_addr, 0u, 0u, PO_SETADDR);
    }
    if (op != (Op *)0)
    {
        op->flags = OF_REQUIRED | OF_ADDR0;
    }
    op = ins_get_desc(WH_GAP, w.tm.recovery, WS('H', 6, 0), 0x01u, 0u, 0u, 18u, PO_DEV18);
    if (op != (Op *)0)
    {
        op->flags = OF_REQUIRED;
    }
    op = ins_get_desc(WH_GAP, w.tm.cfg, WS('H', 7, 0), 0x02u, 0u, 0u, 255u, PO_CFG);
    if (op != (Op *)0)
    {
        op->flags = OF_REQUIRED;
    }
    (void)ins_call(PL_AFTER_CFG);
}

/* After H7: fetch full config if needed, classify, then H8 (BOS). */
static void plan_after_cfg(void)
{
    if (!ud_parse_config(w.cfg, w.cfg_len, &w.cc))
    {
        required_failed(WS('H', 7, 0), ER_BAD_DESC);
        return;
    }
    ins_begin();
    if ((w.cc.total_length > w.cfg_len) && (w.cc.total_length > 255u) && !w.cfg_retried)
    {
        /* wTotalLength > 255: re-fetch with full wTotalLength */
        w.cfg_retried = 1u;
        (void)ins_get_desc(WH_HUB, 0u, WS('H', 7, 1), 0x02u, 0u, 0u, w.cc.total_length, PO_CFG_RETRY);
        (void)ins_call(PL_AFTER_CFG);
        return;
    }
    w.cfg_complete = (uint8_t)((w.cfg_len >= w.cc.total_length) ? 1u : 0u);
    log_per_iface_overflow();
    /* Vendor code table lookup */
    {
        uint8_t state = 0u;
        uint8_t code = 0u;

        if (enum_port_vendor_lookup(w.dd.vid, w.dd.pid, w.dd.bcd_device, &state, &code))
        {
            w.vc_state = (state == LAT_PC_VENDOR_STATE_CODE) ? VC_CODE : VC_NONE;
            w.vendor_code = code;
        }
        else
        {
            w.vc_state = VC_UNKNOWN;
            w.vendor_code = 0u;
        }
        w.vc_state_start = w.vc_state;
        trace_settings(enum_port_now(), state, code);
    }
    classify();
    select_periodic();
    log_kv("H7 config", " len=", w.cfg_len, " ifaces=", w.cc.num_interfaces);
    progress(WS('H', 7, 0), 0u);

    /* H8: BOS(255) if bcdUSB >= 0x0201 */
    if (w.dd.bcd_usb >= 0x0201u)
    {
        (void)ins_get_desc(WH_HUB, 0u, WS('H', 8, 0), 0x0Fu, 0u, 0u, 255u, PO_BOS);
        (void)ins_call(PL_AFTER_BOS);
    }
    else
    {
        (void)ins_call(PL_MSOS1);
    }
}

static void plan_after_bos(void)
{
    uint16_t total = ud_bos_total(w.bos, w.bos_len);
    uint8_t vc = 0u;
    uint16_t set_len = 0u;

    ins_begin();
    if ((w.bos_len >= 5u) && (total > w.bos_len) && !w.bos_retried)
    {
        /* Re-fetch BOS if wTotalLength > received length */
        w.bos_retried = 1u;
        (void)ins_get_desc(WH_HUB, 0u, WS('H', 8, 1), 0x0Fu, 0u, 0u, total, PO_BOS);
        (void)ins_call(PL_AFTER_BOS);
        return;
    }
    if ((w.bos_len >= 5u) && ud_bos_msos20(w.bos, w.bos_len, WP_WIN_VERSION, &vc, &set_len) && (set_len > 0u))
    {
        /* MS OS 2.0 descriptor set (skip H9) */
        w.msos2_done = 1u;
        (void)ins_ctrl(WH_HUB, 0u, WS('H', 8, 2), 0xC0u, vc, 0u, 0x0007u, set_len, PO_MSOS2);
    }
    (void)ins_call(PL_MSOS1);
}

/* H9-H11 */
static void plan_msos1(void)
{
    ins_begin();
    if (!w.msos2_done)
    {
        if (w.vc_state == VC_CODE)
        {
            if (!w.composite)
            {
                ins_msos1_header(WH_HUB, 0u, 0u); /* a) Non-composite */
            }
            /* b) Composite handled in usbccgp (C3) */
        }
        else if (w.vc_state == VC_UNKNOWN)
        {
            /* d) First connection: string 0xEE */
            (void)ins_get_desc(WH_HUB, 0u, WS('H', 9, 0), 0x03u, 0xEEu, 0u, 0x12u, PO_EE);
        }
        /* c) No MS OS descriptor */
    }
    /* H10: iSerial -> String 0 -> iProduct (iManufacturer not requested) */
    if (w.dd.i_serial != 0u)
    {
        Op *op = ins_get_desc(WH_HUB, 0u, WS('H', 10, 0), 0x03u, w.dd.i_serial, 0x0409u, 255u, PO_STR);

        if (op != (Op *)0)
        {
            op->arg0 = w.dd.i_serial;
        }
    }
    {
        Op *op = ins_get_desc(WH_HUB, 0u, WS('H', 10, 1), 0x03u, 0u, 0u, 255u, PO_STR);

        if (op != (Op *)0)
        {
            op->arg0 = 0u;
        }
    }
    if (w.dd.i_product != 0u)
    {
        Op *op = ins_get_desc(WH_HUB, 0u, WS('H', 10, 2), 0x03u, w.dd.i_product, 0x0409u, 255u, PO_STR);

        if (op != (Op *)0)
        {
            op->arg0 = w.dd.i_product;
        }
    }
    /* H11: Device Qualifier (skip for High Speed or bcdUSB < 0x0200) */
    if ((w.speed != USBHS_SPEED_HS) && (w.dd.bcd_usb >= 0x0200u))
    {
        (void)ins_get_desc(WH_HUB, 0u, WS('H', 11, 0), 0x06u, 0u, 0u, 10u, PO_NONE);
    }
    (void)ins_call(PL_FUNCTION);
}

/* H12 -> C1, C2 */
static void plan_function(void)
{
    bool c1 = (w.speed != USBHS_SPEED_HS) || !w.cfg_complete;
    uint8_t cv = (w.cc.config_value != 0u) ? w.cc.config_value : 1u;
    Op *op;

    classify(); /* Apply H9 compat ID ("XUSB10") */
    progress(WS('H', 12, 0), 0u);
    log_kv("hub layer done", " class=", w.classification, " proto=", w.protocol);
    ins_begin();
    if (c1)
    {
        op = ins_get_desc(WH_GAP, w.tm.handover, WS('C', 1, 0), 0x01u, 0u, 0u, 18u, PO_DEV18);
        if (op != (Op *)0)
        {
            op->flags = OF_REQUIRED;
        }
        op = ins_get_desc(WH_GAP, WP_C1_CFG9_GAP, WS('C', 1, 1), 0x02u, 0u, 0u, 9u, PO_C1_CFG9);
        if (op != (Op *)0)
        {
            op->flags = OF_REQUIRED;
        }
        op = ins_get_desc(WH_GAP, WP_C1_CFGALL_GAP, WS('C', 1, 2), 0x02u, 0u, 0u, w.cc.total_length, PO_CFG);
        if (op != (Op *)0)
        {
            op->flags = OF_REQUIRED;
        }
        op = ins_ctrl(WH_GAP, WP_C2_GAP, WS('C', 2, 0), 0x00u, 0x09u, cv, 0u, 0u, PO_SETCFG);
    }
    else
    {
        op = ins_ctrl(WH_GAP, w.tm.handover, WS('C', 2, 0), 0x00u, 0x09u, cv, 0u, 0u, PO_SETCFG);
    }
    if (op != (Op *)0)
    {
        op->flags = OF_REQUIRED;
    }
    (void)ins_call(PL_AFTER_SETCFG);
}

/* C3..C4 (composite only) */
static void plan_after_setcfg(void)
{
    ins_begin();
    if (w.classification == ENUM_CLASS_COMPOSITE)
    {
        bool c3 = (w.vc_state == VC_CODE) && !w.msos2_done;
        uint8_t n = w.cc.iface_count;
        uint8_t k;

        if (c3)
        {
            ins_msos1_header(WH_GAP, w.tm.cfg_settle, 1u);
        }
        /* C4: per interface, iProduct(4) -> iProduct(bLength) */
        if (w.dd.i_product != 0u)
        {
            for (k = 0u; k < n; k++)
            {
                uint32_t gap = (k == 0u) ? (c3 ? WP_C4_FIRST_GAP : w.tm.cfg_settle) : WP_C4_NEXT_IF_GAP;
                Op *op = ins_get_desc(WH_GAP, gap, WS('C', 4, (uint8_t)(k * 2u)), 0x03u, w.dd.i_product, 0x0409u, 4u,
                                      PO_STR_LEN);

                if (op != (Op *)0)
                {
                    op->arg0 = w.dd.i_product;
                    op->arg1 = 0u;
                }
            }
        }
    }
    (void)ins_call(PL_DRIVERS);
}

/* Function drivers (in interface order) */
static void plan_drivers(void)
{
    uint8_t i;
    uint8_t pos;
    uint8_t order[UD_IFACE_MAX];
    uint8_t prev = 0u;           /* 0 initial, 1 after XUSB, 2 after HID, 3 after GIP */
    Op *op;

    ins_begin();
    for (pos = 0u; pos < w.cc.iface_count; pos++)
    {
        uint8_t best = 0xFFu;

        for (i = 0u; i < w.cc.iface_count; i++)
        {
            uint8_t k;
            bool used = false;

            for (k = 0u; k < pos; k++)
            {
                used = used || (order[k] == i);
            }
            if (!used && ((best == 0xFFu) || (w.cc.iface[i].number < w.cc.iface[best].number)))
            {
                best = i;
            }
        }
        order[pos] = best;
    }
    for (pos = 0u; pos < w.cc.iface_count; pos++)
    {
        i = order[pos];
        const UdIface *f = &w.cc.iface[i];
        uint8_t ii = f->number;

        switch (w.func[i])
        {
        case ENUM_FUNC_XUSB:
            /* X1..X5 (timed from SET_CONFIGURATION status ACK) */
            (void)ins_ctrl(WH_ANCHOR, WP_X1_AT, WS('X', 1, i), 0x80u, 0x00u, 0u, 0u, 2u, PO_NONE);
            if (w.dd.i_product != 0u)
            {
                op = ins_get_desc(WH_ANCHOR, WP_X2_AT, WS('X', 2, (uint8_t)(i * 2u)), 0x03u, w.dd.i_product, 0x0409u, 2u,
                                  PO_STR_LEN);
                if (op != (Op *)0)
                {
                    op->arg0 = w.dd.i_product;
                    op->arg1 = 1u;
                }
            }
            (void)ins_ctrl(WH_ANCHOR, WP_X3_AT, WS('X', 3, i), 0xC1u, 0x01u, 0x0100u, ii, 20u, PO_NONE);
            (void)ins_ctrl(WH_ANCHOR, WP_X4_AT, WS('X', 4, i), 0xC1u, 0x01u, 0x0000u, ii, 8u, PO_NONE);
            (void)ins_ctrl(WH_ANCHOR, WP_X5_AT, WS('X', 5, i), 0xC0u, 0x01u, 0x0000u, 0u, 4u, PO_NONE);
            /* X6: start IN polling; X7: 4 LED OUTs (one per OUT period) */
            if (f->ep_in_count != 0u)
            {
                ta_open_iface(i, w.t_setcfg + WP_X6_AT, WS('X', 6, i), true, false);
            }
            if ((f->ep_out != 0u) && w.poll_out[i])
            {
                static const uint8_t led[4][3] = {{0x01u, 0x03u, 0x02u}, {0x02u, 0x08u, 0x03u},
                                                  {0x01u, 0x03u, 0x02u}, {0x01u, 0x03u, 0x06u}};
                uint8_t k;

                (void)ta_add(TA_OPEN_OUT, i, w.t_setcfg + WP_X7_AT, WS('X', 7, i));
                for (k = 0u; k < 4u; k++)
                {
                    Timed *t = ta_add(TA_PUSH, i, w.t_setcfg + WP_X7_AT, WS('X', 7, i));

                    if (t != (Timed *)0)
                    {
                        lat_bytes_copy(t->data, led[k], 3u);
                        t->len = 3u;
                    }
                }
            }
            prev = 1u;
            break;

        case ENUM_FUNC_HID:
        {
            /* After previous HID, space by D4 from D3a */
            uint32_t gap = (prev == 0u) ? WP_C5_GAP : (prev == 2u) ? WP_D4_DELAY : WP_G2_GAP;
            uint16_t ll = (uint16_t)(f->report_len + 64u);

            /* D1 SET_IDLE (duration 0, report ID 0) */
            (void)ins_ctrl((prev == 1u) ? WH_ANCHOR : WH_GAP, (prev == 1u) ? (WP_X5_AT + WP_D1_AFTER_X5_GAP) : gap,
                           WS('D', 1, i), 0x21u, 0x0Au, 0u, ii, 0u, PO_NONE);
            /* D2 GET_DESCRIPTOR(Report) wLength = wDescriptorLength + 64 */
            op = ins_ctrl(WH_GAP, WP_D2_GAP, WS('D', 2, i), 0x81u, 0x06u, 0x2200u, ii, ll, PO_REPORT);
            if (op != (Op *)0)
            {
                op->arg0 = i;
            }
            /* D3a scheduled from D2 completion in run_queue OK_OPEN */
            op = ins_op(OK_OPEN, WH_GAP, WP_D3A_DELAY, WS('D', 3, i));
            if (op != (Op *)0)
            {
                op->arg0 = i;
                op->arg1 = 2u; /* IN and OUT (if present) */
            }
            prev = 2u;
            break;
        }

        case ENUM_FUNC_GIP:
            /* G2: SET_INTERFACE(1, 0) if interface 1 exists */
            if (ud_iface_index(&w.cc, 1u) != 0xFFu)
            {
                (void)ins_ctrl(WH_GAP, (prev == 0u) ? WP_C5_GAP : WP_G2_GAP, WS('G', 2, 0), 0x01u, 0x0Bu, 0u, 1u, 0u,
                               PO_NONE);
            }
            /* G3: start interface 0 IN polling; G4: init */
            op = ins_op(OK_OPEN, WH_GAP, 0u, WS('G', 3, 0));
            if (op != (Op *)0)
            {
                op->arg0 = i;
                op->arg1 = 3u; /* Immediate IN and OUT */
            }
            op = ins_op(OK_APP_GIP, WH_NOW, 0u, WS('G', 4, 0));
            if (op != (Op *)0)
            {
                op->arg0 = i;
            }
            prev = 3u;
            break;

        default:
            break;
        }
    }
    (void)ins_op(OK_READY, WH_NOW, 0u, WS('C', 5, 0));
    (void)ins_call(PL_POST);
}

/* Post-config: app layer, keyboard LED, post-enum requests */
static void plan_post(void)
{
    uint8_t i;
    uint8_t hid_i = pick_target(); /* HID interface carrying input reports (gamepad preferred) */
    Op *op;

    if ((hid_i >= w.cc.iface_count) || (w.func[hid_i] != ENUM_FUNC_HID))
    {
        hid_i = 0xFFu;
    }
    ins_begin();
    /* Switch Pro */
    if ((w.protocol == ENUM_PROTO_SWITCH_PRO) && (w.prof.app_switch != WP_APP_NONE) && (hid_i != 0xFFu))
    {
        Timed *t = ta_add(TA_APP_SWITCH, hid_i, w.t_setcfg + WP_APP_START_AT, WS('S', 0, 0));

        if (t != (Timed *)0)
        {
            t->len = w.prof.app_switch;
        }
    }
    /* DS4, DualSense (sdl_open only) */
    if ((w.prof.app_sony == WP_APP_SDL_OPEN) && (hid_i != 0xFFu) &&
        ((w.protocol == ENUM_PROTO_DS4) || (w.protocol == ENUM_PROTO_DUALSENSE)))
    {
        uint8_t ii = w.cc.iface[hid_i].number;

        w.sony_app = ENUM_APP_RUNNING;
        if (w.protocol == ENUM_PROTO_DS4)
        {
            /* GET_REPORT(Feature 0x12) wLength 16 -> output report 0x05 (32 bytes) */
            (void)ins_ctrl(WH_ANCHOR, WP_APP_START_AT, WS('A', 1, 0), 0xA1u, 0x01u, 0x0312u, ii, 16u, PO_NONE);
            op = ins_op(OK_PUSH, WH_GAP, 0u, WS('A', 2, 0));
            if (op != (Op *)0)
            {
                op->arg0 = hid_i;
                op->len = 32u;
                op->data[0] = 0x05u;
                op->data[8] = 0x40u; /* Lightbar B (player 0 blue); valid flags = 0 */
            }
        }
        else
        {
            op = ins_op(OK_WAIT_DS5, WH_ANCHOR, WP_APP_START_AT, WS('A', 3, 0));
            (void)ins_ctrl(WH_GAP, WP_ANCHOR_MIN_GAP, WS('A', 4, 0), 0xA1u, 0x01u, 0x0309u, ii, 20u, PO_NONE);
            (void)ins_ctrl(WH_GAP, WP_ANCHOR_MIN_GAP, WS('A', 5, 0), 0xA1u, 0x01u, 0x0320u, ii, 64u, PO_NONE);
            op = ins_op(OK_PUSH, WH_GAP, 0u, WS('A', 6, 0));
            if (op != (Op *)0)
            {
                op->arg0 = hid_i;
                op->len = 48u;
                op->data[0] = 0x02u; /* Send A: all zeros */
            }
            op = ins_op(OK_PUSH, WH_GAP, WP_MS(10), WS('A', 7, 0));
            if (op != (Op *)0)
            {
                op->arg0 = hid_i;
                op->len = 48u;
                op->data[0] = 0x02u; /* Send B: lightbar and player LEDs */
                op->data[2] = 0x14u;
                op->data[44] = 0x24u;
                op->data[47] = 0x40u;
            }
        }
        (void)ins_op(OK_APP_DONE, WH_NOW, 0u, 0u);
    }
    /* Keyboard LED (NumLock on) */
    for (i = 0u; i < w.cc.iface_count; i++)
    {
        if ((w.func[i] == ENUM_FUNC_HID) && w.hid[i].kbd_led)
        {
            uint8_t rid = w.hid[i].kbd_led_report_id;

            if ((w.cc.iface[i].ep_out != 0u) && (w.pipe_out[i] != 0u))
            {
                Timed *t = ta_add(TA_PUSH, i, w.t_setcfg + WP_LED_OUT_AT, WS('K', 1, i));

                if (t != (Timed *)0)
                {
                    uint8_t n = 0u;

                    if (rid != 0u)
                    {
                        t->data[n++] = rid;
                    }
                    t->data[n++] = 0x01u;
                    t->len = n;
                }
            }
            else
            {
                op = ins_ctrl(WH_ANCHOR, WP_LED_SETREPORT_AT, WS('K', 2, i), 0x21u, 0x09u,
                              (uint16_t)(0x0200u | rid), w.cc.iface[i].number, (rid != 0u) ? 2u : 1u, PO_NONE);
                if (op != (Op *)0)
                {
                    uint8_t n = 0u;

                    if (rid != 0u)
                    {
                        op->data[n++] = rid;
                    }
                    op->data[n++] = 0x01u;
                    op->len = n;
                }
            }
        }
    }
    /* Post-enum requests (timed from SET_CONFIGURATION status ACK) */
    if (w.prof.post_requests != 0u)
    {
        /* {row, kind (0 String0, 1 iProduct, 2 iSerial, 3 DevQual, 4 BOS), wLength, time (10us)} */
        static const uint32_t rows[31][4] = {
            {27u, 0u, 255u, 8449u},  {28u, 1u, 255u, 8528u},  {29u, 0u, 255u, 9599u},  {30u, 0u, 255u, 9644u},
            {31u, 1u, 255u, 10109u}, {32u, 1u, 255u, 10144u}, {33u, 0u, 255u, 10701u}, {34u, 1u, 255u, 10884u},
            {35u, 1u, 1026u, 10979u}, {36u, 2u, 1026u, 11001u}, {37u, 0u, 255u, 11804u}, {38u, 1u, 1026u, 11899u},
            {39u, 2u, 1026u, 11920u}, {40u, 1u, 255u, 12326u}, {41u, 1u, 1026u, 12927u}, {42u, 2u, 1026u, 12967u},
            {43u, 0u, 4u, 13046u},    {44u, 1u, 255u, 13079u}, {45u, 0u, 4u, 13119u},   {46u, 3u, 10u, 13199u},
            {47u, 4u, 5u, 13292u},    {48u, 1u, 1026u, 13472u}, {49u, 2u, 1026u, 13500u}, {50u, 1u, 522u, 50122u},
            {51u, 1u, 522u, 50167u},  {52u, 1u, 522u, 51125u}, {53u, 1u, 522u, 51192u}, {54u, 1u, 522u, 211919u},
            {55u, 1u, 522u, 211963u}, {56u, 1u, 522u, 213342u}, {57u, 1u, 522u, 213377u}};
        uint8_t r;

        for (r = 0u; r < 31u; r++)
        {
            uint8_t kind = (uint8_t)rows[r][1];
            uint32_t at = rows[r][3] * 1000u; /* 10us -> ticks */
            uint16_t step = WS_POST(rows[r][0]);
            uint16_t wl = (uint16_t)rows[r][2];

            switch (kind)
            {
            case 0u:
                (void)ins_get_desc(WH_ANCHOR, at, step, 0x03u, 0u, 0u, wl, PO_NONE);
                break;
            case 1u:
                if (w.dd.i_product != 0u)
                {
                    (void)ins_get_desc(WH_ANCHOR, at, step, 0x03u, w.dd.i_product, 0x0409u, wl, PO_NONE);
                }
                break;
            case 2u:
                if (w.dd.i_serial != 0u)
                {
                    (void)ins_get_desc(WH_ANCHOR, at, step, 0x03u, w.dd.i_serial, 0x0409u, wl, PO_NONE);
                }
                break;
            case 3u:
                (void)ins_get_desc(WH_ANCHOR, at, step, 0x06u, 0u, 0u, wl, PO_NONE);
                break;
            default:
                (void)ins_get_desc(WH_ANCHOR, at, step, 0x0Fu, 0u, 0u, wl, PO_NONE);
                break;
            }
        }
    }
}

static void plan(uint8_t id, uint64_t now)
{
    (void)now;
    switch (id)
    {
    case PL_AFTER_H3:
        plan_after_h3();
        break;
    case PL_AFTER_CFG:
        plan_after_cfg();
        break;
    case PL_AFTER_BOS:
        plan_after_bos();
        break;
    case PL_MSOS1:
        plan_msos1();
        break;
    case PL_FUNCTION:
        plan_function();
        break;
    case PL_AFTER_SETCFG:
        plan_after_setcfg();
        break;
    case PL_DRIVERS:
        plan_drivers();
        break;
    case PL_POST:
        plan_post();
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Pipes */
/* ------------------------------------------------------------------ */

static void isr_dispatch(const UsbhsIntrDone *d, const uint8_t *data)
{
    EnumIsrHook h = (d->pipe <= USBHS_INTR_MAX) ? s_hook[d->pipe] : (EnumIsrHook)0;

    if (h != (EnumIsrHook)0)
    {
        h(d, data);
    }
}

static uint8_t open_pipe(uint8_t idx, bool in, uint8_t ep_slot, uint64_t at, uint16_t step)
{
    const UdIface *f = &w.cc.iface[idx];
    UsbhsIntrCfg cfg;
    bool hs = (w.speed == USBHS_SPEED_HS);
    uint8_t h;

    lat_bytes_fill((uint8_t *)&cfg, 0u, (uint32_t)sizeof(cfg));
    cfg.addr = w.addr;
    if (in && (ep_slot >= f->ep_in_count))
    {
        return 0u;
    }
    cfg.ep_addr = in ? f->ep_in[ep_slot] : f->ep_out;
    cfg.mps = in ? f->in_mps[ep_slot] : f->out_mps;
    cfg.b_interval = in ? f->in_interval[ep_slot] : f->out_interval;
    cfg.interval_mode = w.prof.interval_mode;
    cfg.trace_budget = WP_INTR_TRACE_BUDGET;
    cfg.err_immediate = WP_INTR_ERR_IMMEDIATE;
    cfg.step = step;
    cfg.slot_ticks = in ? (hs ? w.tm.slot_in_hs : w.tm.slot_in_fs) : (hs ? w.tm.slot_out_hs : w.tm.slot_out_fs);
    cfg.err_pause_ticks = WP_INTR_ERR_PAUSE;
    cfg.start_at = at;
    cfg.hook = isr_dispatch;
    if ((cfg.ep_addr == 0u) || (cfg.mps == 0u))
    {
        return 0u;
    }
    h = usbhs_intr_open(&cfg);
    if (h == 0u)
    {
        log_kv("intr open failed", " ep=", cfg.ep_addr, (const char *)0, 0u);
        return 0u;
    }
    if (in)
    {
        w.pipe_in[idx][ep_slot] = h;
    }
    else
    {
        w.pipe_out[idx] = h;
    }
    return h;
}

static bool push_out(uint8_t idx, const uint8_t *data, uint8_t len, uint16_t step)
{
    if ((idx < UD_IFACE_MAX) && (w.pipe_out[idx] != 0u))
    {
        uint16_t n = len;

        if (n > w.cc.iface[idx].out_mps)
        {
            n = w.cc.iface[idx].out_mps;
        }
        return usbhs_intr_out_push(w.pipe_out[idx], data, n);
    }
    (void)step;
    return false;
}

static void app_switch_begin(uint8_t idx, uint8_t mode, uint64_t now)
{
    AppTarget t;

    lat_bytes_fill((uint8_t *)&t, 0u, (uint32_t)sizeof(t));
    t.vid = w.dd.vid;
    t.pid = w.dd.pid;
    t.in_pipe = w.pipe_in[idx][0];
    t.out_pipe = w.pipe_out[idx];
    t.out_len = (w.cc.iface[idx].out_mps < 64u) ? w.cc.iface[idx].out_mps : 64u;
    app_switch_start(&t, mode, now);
}

static void timed_run(uint64_t now)
{
    uint8_t i;

    for (i = 0u; i < TA_MAX; i++)
    {
        Timed *t = &w.ta[i];

        if (!t->used || ((int64_t)(now - t->at) < 0))
        {
            continue;
        }
        t->used = 0u;
        switch (t->kind)
        {
        case TA_OPEN_IN:
            if ((t->idx < w.cc.iface_count) && (t->ep_slot < UD_EP_IN_MAX) &&
                (w.pipe_in[t->idx][t->ep_slot] == 0u))
            {
                (void)open_pipe(t->idx, true, t->ep_slot, t->at, t->step);
            }
            break;
        case TA_OPEN_OUT:
            if ((t->idx < w.cc.iface_count) && (w.pipe_out[t->idx] == 0u))
            {
                (void)open_pipe(t->idx, false, 0u, t->at, t->step);
            }
            break;
        case TA_PUSH:
            if (!push_out(t->idx, t->data, t->len, t->step))
            {
                log_kv("OUT push failed", " step=", t->step, (const char *)0, 0u);
            }
            break;
        case TA_APP_SWITCH:
            app_switch_begin(t->idx, t->len, now);
            break;
        case TA_RESUME_OUT:
            (void)usbhs_intr_set_toggle(t->idx, 0u);
            (void)usbhs_intr_resume(t->idx, 0u);
            break;
        default:
            break;
        }
    }
    if (w.pub.ready)
    {
        w.pub.app_init = (w.sony_app != ENUM_APP_NONE) ? w.sony_app : app_state();
    }
}

/* ------------------------------------------------------------------ */
/* Op execution */
/* ------------------------------------------------------------------ */

static void do_ready(void)
{
    uint8_t b[8];

    w.enum_seq++;
    publish();
    usbhs_set_enum_state(true, 0u);
    lat_bytes_fill(b, 0u, sizeof(b));
    lat_put_u32(&b[0], w.gen);
    b[4] = w.classification;
    ev_push(LAT_PC_EVT_DEVICE_READY, b, 8u);
    {
        char buf[LOG_MAX + 1u];
        LatStr s;

        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[enum] ready id=");
        put_hex(&s, w.dd.vid, 4u);
        lat_str_putc(&s, ':');
        put_hex(&s, w.dd.pid, 4u);
        lat_str_puts(&s, " sp=");
        lat_str_puts(&s, speed_name(w.speed));
        lat_str_puts(&s, " addr=");
        lat_str_u32(&s, w.addr);
        lat_str_puts(&s, " class=");
        lat_str_u32(&s, w.classification);
        lat_str_puts(&s, " proto=");
        lat_str_u32(&s, w.protocol);
        lat_str_puts(&s, " target=");
        lat_str_u32(&s, w.pub.target_iface);
        enum_port_log(buf);
    }
}

static void submit_ctrl(uint64_t t, uint64_t now)
{
    UsbhsCtrlReq req;
    uint8_t *buf;
    uint16_t cap;
    uint16_t wl = lat_get_u16(&w.cur.setup[6]);
    bool out = (w.cur.setup[0] & 0x80u) == 0u;

    lat_bytes_fill((uint8_t *)&req, 0u, (uint32_t)sizeof(req));
    lat_bytes_copy(req.setup, w.cur.setup, 8u);
    req.addr = ((w.cur.flags & OF_ADDR0) != 0u) ? 0u : w.addr;
    req.mps = (w.mps0 != 0u) ? w.mps0 : ((w.speed == USBHS_SPEED_LS) ? 8u : 64u);
    req.step = w.cur.step;
    req.trace = 1u;
    if (out && (wl > 0u))
    {
        req.buf = w.cur.data;
        req.buf_len = (uint16_t)sizeof(w.cur.data);
        if (wl > sizeof(w.cur.data))
        {
            /* Max 64 bytes payload (all emulated requests fit) */
            lat_put_u16(&req.setup[6], (uint16_t)sizeof(w.cur.data));
        }
    }
    else if (wl > 0u)
    {
        dest_for(&w.cur, &buf, &cap);
        req.buf = buf;
        req.buf_len = cap;
    }
    req.start_at = (t == 0u) ? 1u : t;
    ctrl_timing(&req.timing);
    if (!usbhs_ctrl_submit(&req))
    {
        /* Port not active (disconnect/recovery); wait for event */
        UsbhsPortInfo pi;

        usbhs_port_info(&pi);
        if (pi.state == USBHS_PORT_ENABLED)
        {
            log_kv("ctrl submit rejected", " step=", w.cur.step, (const char *)0, 0u);
            if ((w.cur.flags & OF_REQUIRED) != 0u)
            {
                required_failed(w.cur.step, ER_PORT);
            }
            return;
        }
        w.wait = WAIT_NONE;
        w.st = ST_IDLE;
        (void)now;
        return;
    }
    w.wait = WAIT_CTRL;
}

static void recovery_poll(uint64_t now)
{
    UsbhsIntrRecovery rec;
    UsbhsCtrlReq req;

    if ((w.wait != WAIT_NONE) || usbhs_ctrl_busy() || !usbhs_intr_recovery_pending(&rec))
    {
        return;
    }
    lat_bytes_fill((uint8_t *)&req, 0u, (uint32_t)sizeof(req));
    req.setup[0] = 0x02u;
    req.setup[1] = 0x01u;
    lat_put_u16(&req.setup[4], rec.endpoint);
    req.addr = rec.address;
    req.mps = w.mps0;
    req.start_at = now;
    req.step = WS('R', 1, 0);
    req.trace = 1u;
    ctrl_timing(&req.timing);
    if (usbhs_ctrl_submit(&req))
    {
        w.recovery_pipe = rec.pipe;
        w.recovery_generation = rec.generation;
        w.wait = WAIT_RECOVERY;
    }
}

static void recovery_done(const UsbhsCtrlResult *r)
{
    bool success = (r->result == USBHS_RES_OK);

    (void)usbhs_intr_recovery_complete(w.recovery_pipe, w.recovery_generation, success);
    w.recovery_pipe = 0u;
    w.recovery_generation = 0u;
    w.wait = WAIT_NONE;
}

static void run_queue(uint64_t now)
{
    while ((w.st == ST_RUN) && (w.wait == WAIT_NONE) && (w.qn > 0u))
    {
        Op *op = &w.q[0];
        uint64_t t = op_time(op, now);
        bool due = (int64_t)(now - t) >= 0;

        if (w.skip_ops > 0u)
        {
            Op dummy;

            q_pop(&dummy);
            w.skip_ops--;
            continue;
        }
        switch (op->kind)
        {
        case OK_CTRL:
            if (usbhs_ctrl_busy())
            {
                return; /* Wait for previous transfer to finish */
            }
            q_pop(&w.cur);
            submit_ctrl(t, now);
            break;

        case OK_RESET:
            if (!due)
            {
                return;
            }
            q_pop(&w.cur);
            if (usbhs_port_reset(w.tm.long_reset, w.cur.step))
            {
                w.wait = WAIT_RESET;
                w.addr = 0u;
            }
            else
            {
                /* Device detached; wait for connect event */
                w.st = ST_IDLE;
                q_clear();
            }
            break;

        case OK_CALL:
        {
            Op c;

            q_pop(&c);
            plan(c.arg0, now);
            break;
        }

        case OK_OPEN:
        {
            Op c;
            uint64_t at;

            q_pop(&c);
            at = t;
            if (c.arg0 < w.cc.iface_count)
            {
                /* D3a: schedule IN (and OUT) polling at target time */
                ta_open_iface(c.arg0, at, c.step, true, true);
            }
            /* Next Op spacing (D4) is measured from D3a */
            w.t_prev = at;
            break;
        }

        case OK_APP_GIP:
        {
            Op c;
            AppTarget a;

            q_pop(&c);
            timed_run(now); /* Open G3 pipes first */
            lat_bytes_fill((uint8_t *)&a, 0u, (uint32_t)sizeof(a));
            a.vid = w.dd.vid;
            a.pid = w.dd.pid;
            a.in_pipe = w.pipe_in[c.arg0][0];
            a.out_pipe = w.pipe_out[c.arg0];
            a.out_len = w.cc.iface[c.arg0].out_mps;
            app_gip_start(&a, now);
            break;
        }

        case OK_PUSH:
        {
            Op c;

            if (!due)
            {
                return;
            }
            q_pop(&c);
            if (w.pipe_out[c.arg0] != 0u)
            {
                (void)push_out(c.arg0, c.data, c.len, c.step);
            }
            else if (c.arg0 < w.cc.iface_count)
            {
                /* No OUT endpoint: send via SET_REPORT(Output) */
                Op *s;

                ins_begin();
                s = ins_ctrl(WH_NOW, 0u, c.step, 0x21u, 0x09u, (uint16_t)(0x0200u | c.data[0]),
                             w.cc.iface[c.arg0].number, c.len, PO_NONE);
                if (s != (Op *)0)
                {
                    lat_bytes_copy(s->data, c.data, c.len);
                    s->len = c.len;
                }
            }
            w.t_prev = now;
            break;
        }

        case OK_WAIT_DS5:
            if (!due)
            {
                return;
            }
            if (w.t_ds5_wait == 0u)
            {
                w.t_ds5_wait = now;
            }
            if (!w.ds5_seen && ((now - w.t_ds5_wait) < WP_MS(16)))
            {
                return;
            }
            {
                Op c;

                q_pop(&c);
                if (!w.ds5_seen)
                {
                    w.skip_ops = 2u; /* No 64-byte input: skip 0x09 and 0x20 (SDL) */
                }
                w.t_prev = now;
                w.t_ds5_wait = 0u;
            }
            break;

        case OK_APP_DONE:
        {
            Op c;

            q_pop(&c);
            w.sony_app = ENUM_APP_DONE;
            break;
        }

        case OK_READY:
            if (ta_open_pending())
            {
                /* Wait until D3a/X6/X7 pipes are open */
                timed_run(now);
                if (ta_open_pending())
                {
                    return;
                }
            }
            {
                Op c;

                q_pop(&c);
                do_ready();
            }
            break;

        default:
        {
            Op c;

            q_pop(&c);
            break;
        }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Events */
/* ------------------------------------------------------------------ */

static void on_attach(const UsbhsEvent *ev)
{
    clear_attempt();
    w.gen = ev->generation;
    w.speed = ev->speed;
    w.attempt = 1u;
    w.force_reset2 = 0u;
    load_profile();
    w.t_debounce = ev->time + w.tm.debounce;
    w.pub.generation = w.gen;
    log_kv("attach", " gen=", w.gen, " speed=", w.speed);
    start_attempt(WH_DEBOUNCE);
}

static void on_detach(const UsbhsEvent *ev)
{
    if (ev->generation != w.gen)
    {
        return;
    }
    clear_attempt();
    w.st = ST_IDLE;
    w.speed = USBHS_SPEED_NONE;
    log_kv("detach", " gen=", ev->generation, (const char *)0, 0u);
}

static void on_pipe_halt(const UsbhsEvent *ev, uint64_t now)
{
    uint8_t i;

    log_kv("pipe halt", " pipe=", ev->pipe, " res=", ev->result);
    if ((ev->result != USBHS_RES_STALL) || (ev->pipe == 0u) || (ev->pipe > USBHS_INTR_MAX))
    {
        return;
    }
    for (i = 0u; i < w.cc.iface_count; i++)
    {
        if (w.pipe_out[i] == ev->pipe)
        {
            uint8_t c = w.out_stall_count[ev->pipe];

            if (c < 0xFFu)
            {
                w.out_stall_count[ev->pipe] = (uint8_t)(c + 1u);
            }
            (void)ta_add(TA_RESUME_OUT, ev->pipe,
                         (c < WP_INTR_ERR_IMMEDIATE) ? now : (now + WP_INTR_ERR_PAUSE), 0u);
            return;
        }
    }
}

static void handle_event(const UsbhsEvent *ev, uint64_t now)
{
    switch (ev->type)
    {
    case USBHS_EV_ATTACH:
        on_attach(ev);
        break;
    case USBHS_EV_BOUNCE:
        if (ev->generation == w.gen)
        {
            w.t_debounce = ev->time + w.tm.debounce;
        }
        break;
    case USBHS_EV_DETACH:
        on_detach(ev);
        break;
    case USBHS_EV_RESET_DONE:
        if ((w.wait == WAIT_RESET) && (ev->generation == w.gen))
        {
            w.wait = WAIT_NONE;
            w.t_reset_end = ev->time;
            w.t_prev = ev->time;
            w.speed = ev->speed;
            log_kv("reset done", " speed=", ev->speed, " ticks=", ev->value);
        }
        break;
    case USBHS_EV_CTRL_DONE:
    {
        UsbhsCtrlResult r;

        if (usbhs_ctrl_poll(&r))
        {
            if (w.wait == WAIT_RECOVERY)
            {
                recovery_done(&r);
            }
            else if (w.wait == WAIT_CTRL)
            {
                ctrl_done(&r, now);
            }
        }
        break;
    }
    case USBHS_EV_OUT_DONE:
        app_on_out_done(ev->pipe, ev->time);
        break;
    case USBHS_EV_PIPE_HALT:
        on_pipe_halt(ev, now);
        break;
    case USBHS_EV_RECOVERED:
        log_kv("engine recovered", " cause=", ev->value, " fatal=", ev->result);
        break;
    default:
        break;
    }
}

static void handle_reports(void)
{
    UsbhsReport rep;

    while (usbhs_report_pop(&rep, w.report, (uint16_t)sizeof(w.report)))
    {
        uint8_t i;
        uint8_t iface = 0xFFu;
        uint8_t ep = 0u;
        uint16_t n = (rep.len > sizeof(w.report)) ? (uint16_t)sizeof(w.report) : rep.len;

        if (rep.generation != w.gen)
        {
            continue;
        }
        for (i = 0u; i < w.cc.iface_count; i++)
        {
            uint8_t k;

            for (k = 0u; k < w.cc.iface[i].ep_in_count; k++)
            {
                if ((w.pipe_in[i][k] != 0u) && (w.pipe_in[i][k] == rep.pipe))
                {
                    iface = w.cc.iface[i].number;
                    ep = w.cc.iface[i].ep_in[k];
                    break;
                }
            }
            if (iface != 0xFFu)
            {
                break;
            }
        }
        if ((w.protocol == ENUM_PROTO_DUALSENSE) && (n == 64u) && (w.report[0] == 0x01u))
        {
            w.ds5_seen = 1u;
        }
        app_on_report(rep.pipe, w.report, n, rep.t1);
        if ((w.sink != (EnumReportFn)0) && (iface != 0xFFu))
        {
            w.sink(iface, ep, &rep, w.report);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public functions */
/* ------------------------------------------------------------------ */

static bool any_paused(void)
{
    uint8_t i;

    for (i = 0u; i < UD_IFACE_MAX; i++)
    {
        uint8_t k;

        for (k = 0u; k < UD_EP_IN_MAX; k++)
        {
            if (w.paused[i][k])
            {
                return true;
            }
        }
    }
    return false;
}

void win_enum_init(void)
{
    lat_bytes_fill((uint8_t *)&w, 0u, (uint32_t)sizeof(w));
    clear_hooks();
    app_reset();
    w.st = ST_IDLE;
    w.session_seen = enum_port_session();
    load_profile();
}

void win_enum_poll(uint64_t now)
{
    UsbhsEvent ev;

    while (usbhs_event_pop(&ev))
    {
        handle_event(&ev, now);
    }
    handle_reports();
    recovery_poll(now);
    timed_run(now);
    if (!any_paused())
    {
        app_poll(now); /* Pause steady-state app OUT traffic while IN is paused for calibration */
    }
    run_queue(now);
    ev_flush();
}

uint8_t win_enum_reenumerate(void)
{
    UsbhsPortInfo pi;
    UsbhsTrace t;

    usbhs_port_info(&pi);
    if ((pi.state == USBHS_PORT_DISCONNECTED) || (pi.state == USBHS_PORT_HALTED))
    {
        return LAT_PC_ST_NO_DEVICE;
    }
    if (pi.state != USBHS_PORT_ENABLED)
    {
        return LAT_PC_ST_BUSY; /* Debouncing, resetting, or recovering */
    }
    if ((w.st == ST_RUN) && !w.pub.ready)
    {
        return LAT_PC_ST_BUSY; /* Enumeration in progress */
    }
    if (pi.generation != w.gen)
    {
        return LAT_PC_ST_BUSY; /* Connect event pending */
    }
    load_profile();
    w.attempt = 1u;
    w.force_reset2 = 0u;
    log_kv("reenumerate", " gen=", w.gen, (const char *)0, 0u);
    lat_bytes_fill((uint8_t *)&t, 0u, (uint32_t)sizeof(t));
    t.kind = USBHS_TRK_REENUMERATE;
    t.time = enum_port_now();
    t.addr = w.addr;
    (void)usbhs_record_event(&t);
    start_attempt(WH_NOW);
    return LAT_PC_ST_OK;
}

uint8_t win_enum_device_info(uint8_t *dst, uint32_t cap, uint32_t *len)
{
    const EnumDevice *p = &w.pub;
    uint32_t n;
    uint32_t i;

    if (!p->ready)
    {
        return LAT_PC_ST_NO_DEVICE;
    }
    n = LAT_PC_DEVICE_INFO_HDR_LEN + (12u * (uint32_t)p->iface_count);
    if (cap < n)
    {
        return LAT_PC_ST_INTERNAL;
    }
    lat_bytes_fill(dst, 0u, n);
    lat_put_u32(&dst[0], p->generation);
    dst[4] = p->speed;
    dst[5] = p->address;
    lat_put_u16(&dst[6], p->vid);
    lat_put_u16(&dst[8], p->pid);
    lat_put_u16(&dst[10], p->bcd_usb);
    lat_put_u16(&dst[12], p->bcd_device);
    dst[14] = p->device_class;
    dst[15] = p->classification;
    dst[16] = p->num_interfaces;
    dst[17] = w.vc_state_start;
    dst[18] = w.vendor_code;
    dst[19] = p->iface_count;
    for (i = 0u; i < p->iface_count; i++)
    {
        const EnumIface *f = &p->iface[i];
        uint8_t *e = &dst[LAT_PC_DEVICE_INFO_HDR_LEN + (12u * i)];

        e[0] = f->number;
        e[1] = f->cls;
        e[2] = f->subcls;
        e[3] = f->proto;
        e[4] = f->ep_in;
        e[5] = f->ep_out;
        lat_put_u16(&e[6], f->in_mps);
        e[8] = f->in_interval;
        e[9] = f->alt;
        lat_put_u16(&e[10], f->report_desc_len);
    }
    *len = n;
    return LAT_PC_ST_OK;
}

uint8_t win_enum_object(uint8_t kind, uint8_t index, uint8_t *dst, uint32_t cap, uint32_t *len)
{
    const uint8_t *src = (const uint8_t *)0;
    uint32_t n = 0u;

    if (!w.pub.ready)
    {
        return LAT_PC_ST_NO_DEVICE;
    }
    switch (kind)
    {
    case LAT_PC_OBJ_DEVICE_DESC:
        src = w.dev;
        n = w.dev_len;
        break;
    case LAT_PC_OBJ_CONFIG_DESC:
        src = w.cfg;
        n = w.cfg_len;
        break;
    case LAT_PC_OBJ_HID_REPORT_DESC:
    {
        uint8_t i = ud_iface_index(&w.cc, index);

        if ((i == 0xFFu) || (w.rpt_len[i] == 0u))
        {
            return LAT_PC_ST_BAD_PARAM;
        }
        src = &w.rpt[w.rpt_off[i]];
        n = w.rpt_len[i];
        break;
    }
    case LAT_PC_OBJ_STRING_DESC:
    {
        const StrSlot *s = str_slot(index, false);

        if ((s == (const StrSlot *)0) || (s->len == 0u))
        {
            return LAT_PC_ST_BAD_PARAM;
        }
        src = s->data;
        n = s->len;
        break;
    }
    case LAT_PC_OBJ_BOS_DESC:
        if (w.bos_len == 0u)
        {
            return LAT_PC_ST_BAD_PARAM;
        }
        src = w.bos;
        n = w.bos_len;
        break;
    default:
        return LAT_PC_ST_BAD_PARAM;
    }
    if (((kind != LAT_PC_OBJ_HID_REPORT_DESC) && (kind != LAT_PC_OBJ_STRING_DESC) && (index != 0u)) || (n == 0u))
    {
        return LAT_PC_ST_BAD_PARAM;
    }
    if (n > cap)
    {
        return LAT_PC_ST_INTERNAL;
    }
    lat_bytes_copy(dst, src, n);
    *len = n;
    return LAT_PC_ST_OK;
}

/* ---- enum_api.h ---- */

const EnumDevice *enum_device(void)
{
    return w.pub.ready ? &w.pub : (const EnumDevice *)0;
}

uint32_t enum_seq(void)
{
    return w.enum_seq;
}

const EnumIface *enum_find_iface(uint8_t iface, uint8_t ep_in)
{
    uint8_t i;

    if (!w.pub.ready)
    {
        return (const EnumIface *)0;
    }
    for (i = 0u; i < w.pub.iface_count; i++)
    {
        const EnumIface *f = &w.pub.iface[i];

        if ((f->number == iface) && ((ep_in == 0u) || (f->ep_in == ep_in)))
        {
            return f;
        }
        if (f->number == iface)
        {
            uint8_t k;

            for (k = 1u; k < w.cc.iface[i].ep_in_count; k++)
            {
                const EnumIface *x = &w.extra_in[i][k - 1u];

                if (x->ep_in == ep_in)
                {
                    return x;
                }
            }
        }
    }
    return (const EnumIface *)0;
}

bool enum_set_isr_hook(uint8_t iface, uint8_t ep_in, EnumIsrHook hook)
{
    const EnumIface *f = enum_find_iface(iface, ep_in);

    if ((f == (const EnumIface *)0) || (f->pipe_in == 0u) || (f->pipe_in > USBHS_INTR_MAX))
    {
        return false;
    }
    s_hook[f->pipe_in] = hook;
    return true;
}

void enum_set_report_sink(EnumReportFn fn)
{
    w.sink = fn;
}

static EnumIface *published_in(uint8_t i, uint8_t k)
{
    return (k == 0u) ? &w.pub.iface[i] : &w.extra_in[i][k - 1u];
}

static void pause_one(uint8_t i, uint8_t k)
{
    EnumIface *f = published_in(i, k);

    if (w.pipe_in[i][k] != 0u)
    {
        if (w.pipe_in[i][k] <= USBHS_INTR_MAX)
        {
            s_hook[w.pipe_in[i][k]] = (EnumIsrHook)0;
        }
        (void)usbhs_intr_close(w.pipe_in[i][k]);
        w.pipe_in[i][k] = 0u;
        w.paused[i][k] = 1u;
    }
    f->pipe_in = 0u;
    f->in_period = 0u;
}

static bool resume_one(uint8_t i, uint8_t k)
{
    EnumIface *f = published_in(i, k);

    if (!w.poll_in[i][k])
    {
        return false;
    }
    if (w.pipe_in[i][k] == 0u)
    {
        (void)open_pipe(i, true, k, 0u, WS('D', 3, i));
    }
    if (w.pipe_in[i][k] != 0u)
    {
        w.paused[i][k] = 0u;
    }
    f->pipe_in = w.pipe_in[i][k];
    f->in_period = (f->pipe_in != 0u) ? usbhs_intr_period(w.speed, f->in_interval, w.prof.interval_mode) : 0u;
    return f->pipe_in != 0u;
}

bool enum_set_polling(uint8_t iface, uint8_t ep_in, bool run)
{
    uint8_t i;

    if (!w.pub.ready)
    {
        return false;
    }
    for (i = 0u; i < w.pub.iface_count; i++)
    {
        uint8_t k;

        if (w.pub.iface[i].number != iface)
        {
            continue;
        }
        for (k = 0u; k < w.cc.iface[i].ep_in_count; k++)
        {
            if ((ep_in == 0u) || (w.cc.iface[i].ep_in[k] == ep_in))
            {
                if (!run)
                {
                    pause_one(i, k);
                    return true;
                }
                return resume_one(i, k);
            }
        }
    }
    return false;
}

bool enum_set_polling_all(bool run)
{
    uint8_t i;
    bool ok = true;

    if (!w.pub.ready)
    {
        return false;
    }
    for (i = 0u; i < w.pub.iface_count; i++)
    {
        uint8_t k;

        for (k = 0u; k < w.cc.iface[i].ep_in_count; k++)
        {
            if (!run)
            {
                if (w.pipe_in[i][k] != 0u)
                {
                    pause_one(i, k);
                }
            }
            else if (w.paused[i][k])
            {
                ok = resume_one(i, k) && ok;
            }
        }
    }
    return ok;
}

bool enum_polling_paused(void)
{
    return any_paused();
}
