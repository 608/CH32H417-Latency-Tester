/*
 * app_init.c - Interrupt-OUT application-layer initialization (see app_init.h)
 *
 * Switch Pro:
 *   - All OUT packets are zero-padded to 64 bytes (or wMaxPacketSize if smaller).
 *   - 4-bit packet number increments on every 0x01/0x10 build, including retries.
 *   - Reply timeout is 100ms from OUT completion, up to 5 attempts; S2 retries
 *     for 5s from first send (Machenike F1 workaround).
 *   - OUT completion timeout is 1000ms.
 * GIP:
 *   - Send next packet after previous OUT completes; no reply wait.
 *   - ACK guide button (0x07) only when option byte == 0x30.
 */

#include "app_init.h"

#include "enum_api.h"
#include "enum_port.h"
#include "lat_bytes.h"
#include "lat_fmt.h"
#include "usbhs_host.h"

#define APP_TICKS_MS           ((uint64_t)USBHS_TICKS_PER_MS)
#define SW_REPLY_WAIT          (100u * APP_TICKS_MS)
#define SW_TRIES               (5u)
#define SW_S1_ROUNDS           (3u)
#define SW_S2_SPAN             (5000u * APP_TICKS_MS)
#define SW_OUT_LIMIT           (1000u * APP_TICKS_MS)
#define SW_STEADY_GAP          (100u * APP_TICKS_MS)
#define SW_STEP_STEADY         (14u)
#define SW_PKT                 (64u)

#define EX_NONE                (0u)
#define EX_81                  (1u)
#define EX_21                  (2u)

#define GIP_PKT_MAX            (8u)
#define GIP_LEN_MAX            (13u)

typedef struct SwApp
{
    uint8_t  active;
    uint8_t  mode;
    uint8_t  step;         /* 1..13, SW_STEP_STEADY for steady state */
    uint8_t  round;        /* S1 retry count (0..2) */
    uint8_t  tries;
    uint8_t  pn;           /* 4-bit packet number */
    uint8_t  wait_out;     /* Waiting for OUT completion */
    uint8_t  expect;       /* EX_* */
    uint8_t  exp_id;
    uint8_t  spi_len;      /* SPI address/length to match (0 = no match) */
    uint8_t  spi[5];
    uint8_t  first_mode;   /* First 0x30/0x31/0x3F seen before S2 */
    uint8_t  s2_sent;
    uint8_t  magic_ok;     /* S6: both user calibration magics valid */
    uint8_t  state;        /* ENUM_APP_* */
    uint64_t deadline;
    uint64_t push_t;
    uint64_t s2_first;
    uint64_t last30;
    uint64_t last8004;
    AppTarget t;
} SwApp;

typedef struct GipApp
{
    uint8_t  active;
    uint8_t  wait_announce;
    uint8_t  next;
    uint8_t  count;
    uint8_t  ss;
    uint8_t  outstanding;  /* In-flight OUTs (including ACK) */
    uint8_t  state;
    uint8_t  len[GIP_PKT_MAX];
    uint8_t  pkt[GIP_PKT_MAX][GIP_LEN_MAX];
    AppTarget t;
} GipApp;

static SwApp s_sw;
static GipApp s_gip;

static void app_log2(const char *a, uint32_t v1, const char *b, uint32_t v2)
{
    char buf[96];
    LatStr s;

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, a);
    lat_str_u32(&s, v1);
    if (b != (const char *)0)
    {
        lat_str_puts(&s, b);
        lat_str_u32(&s, v2);
    }
    enum_port_log(buf);
}

void app_reset(void)
{
    lat_bytes_fill((uint8_t *)&s_sw, 0u, (uint32_t)sizeof(s_sw));
    lat_bytes_fill((uint8_t *)&s_gip, 0u, (uint32_t)sizeof(s_gip));
}

uint8_t app_state(void)
{
    if (s_sw.state != ENUM_APP_NONE)
    {
        return s_sw.state;
    }
    return s_gip.state;
}

uint8_t app_switch_step(void)
{
    return s_sw.step;
}

/* ------------------------------------------------------------------ */
/* Switch Pro                                                           */
/* ------------------------------------------------------------------ */

static bool sw_push(const uint8_t *body, uint8_t n, uint64_t now)
{
    uint8_t pkt[SW_PKT];
    uint16_t len = s_sw.t.out_len;

    if ((len == 0u) || (len > SW_PKT))
    {
        len = SW_PKT;
    }
    lat_bytes_fill(pkt, 0u, sizeof(pkt));
    lat_bytes_copy(pkt, body, n);
    if (!usbhs_intr_out_push(s_sw.t.out_pipe, pkt, len))
    {
        return false;
    }
    s_sw.push_t = now;
    return true;
}

/* 01 nn 00 01 40 40 00 01 40 40 <sub> <data> */
static bool sw_subcmd(uint8_t sub, const uint8_t *data, uint8_t n, uint64_t now)
{
    static const uint8_t rumble[8] = {0x00u, 0x01u, 0x40u, 0x40u, 0x00u, 0x01u, 0x40u, 0x40u};
    uint8_t b[24];

    b[0] = 0x01u;
    b[1] = s_sw.pn;
    s_sw.pn = (uint8_t)((s_sw.pn + 1u) & 0x0Fu); /* Increment on build like SDL, even if send fails */
    lat_bytes_copy(&b[2], rumble, 8u);
    b[10] = sub;
    lat_bytes_copy(&b[11], data, n);
    return sw_push(b, (uint8_t)(11u + n), now);
}

static bool sw_usbcmd(uint8_t cc, uint64_t now)
{
    uint8_t b[2];

    b[0] = 0x80u;
    b[1] = cc;
    return sw_push(b, 2u, now);
}

static void sw_advance(uint64_t now);

/* Send one attempt for current s_sw.step. */
static void sw_send(uint64_t now)
{
    static const uint8_t spi_8010[5] = {0x10u, 0x80u, 0x00u, 0x00u, 0x16u};
    static const uint8_t spi_603d[5] = {0x3Du, 0x60u, 0x00u, 0x00u, 0x12u};
    static const uint8_t spi_6020[5] = {0x20u, 0x60u, 0x00u, 0x00u, 0x18u};
    static const uint8_t spi_8026[5] = {0x26u, 0x80u, 0x00u, 0x00u, 0x14u};
    static const uint8_t one[1] = {0x01u};
    static const uint8_t x30[1] = {0x30u};
    bool ok;

    s_sw.expect = EX_NONE;
    s_sw.spi_len = 0u;
    switch (s_sw.step)
    {
    case 1u:
        ok = sw_usbcmd(0x01u, now);
        s_sw.expect = EX_81;
        s_sw.exp_id = 0x01u;
        break;
    case 2u:
        if (!s_sw.s2_sent)
        {
            s_sw.s2_sent = 1u;
            s_sw.s2_first = now;
        }
        ok = sw_usbcmd(0x02u, now);
        s_sw.expect = EX_81;
        s_sw.exp_id = 0x02u;
        break;
    case 3u:
        ok = sw_usbcmd(0x03u, now);
        s_sw.expect = EX_81;
        s_sw.exp_id = 0x03u;
        break;
    case 4u:
        ok = sw_usbcmd(0x02u, now);
        s_sw.expect = EX_81;
        s_sw.exp_id = 0x02u;
        break;
    case 5u:
    case 12u:
        ok = sw_usbcmd(0x04u, now);
        s_sw.last8004 = now;
        break;
    case 6u:
    case 7u:
    case 8u:
    case 9u:
    {
        const uint8_t *a = (s_sw.step == 6u) ? spi_8010 : (s_sw.step == 7u) ? spi_603d : (s_sw.step == 8u) ? spi_6020 : spi_8026;

        ok = sw_subcmd(0x10u, a, 5u, now);
        s_sw.expect = EX_21;
        s_sw.exp_id = 0x10u;
        s_sw.spi_len = 5u;
        lat_bytes_copy(s_sw.spi, a, 5u);
        break;
    }
    case 10u:
        ok = sw_subcmd(0x48u, one, 1u, now);
        s_sw.expect = EX_21;
        s_sw.exp_id = 0x48u;
        break;
    case 11u:
        ok = sw_subcmd(0x03u, x30, 1u, now);
        s_sw.expect = EX_21;
        s_sw.exp_id = 0x03u;
        break;
    case 13u:
        ok = sw_subcmd(0x30u, one, 1u, now);
        s_sw.expect = EX_21;
        s_sw.exp_id = 0x30u;
        break;
    default:
        return;
    }
    s_sw.wait_out = ok ? 1u : 0u;
    if (!ok)
    {
        /* Queue full: treat as failed attempt (handled by timeout on next poll) */
        s_sw.deadline = now;
        s_sw.expect = (s_sw.expect == EX_NONE) ? EX_81 : s_sw.expect;
    }
}

static void sw_fail(const char *why)
{
    s_sw.active = 0u;
    s_sw.state = ENUM_APP_FAILED;
    enum_port_log(why);
}

static void sw_advance(uint64_t now)
{
    s_sw.tries = 0u;
    s_sw.wait_out = 0u;
    s_sw.expect = EX_NONE;
    for (;;)
    {
        s_sw.step++;
        if ((s_sw.mode == APP_SW_INPUT_MINIMAL) && (s_sw.step > 5u))
        {
            s_sw.step = SW_STEP_STEADY;
        }
        if (s_sw.step > 13u)
        {
            s_sw.step = SW_STEP_STEADY;
            s_sw.state = ENUM_APP_DONE;
            s_sw.last30 = now;
            app_log2("[app] switch init done pn=", s_sw.pn, (const char *)0, 0u);
            return;
        }
        if ((s_sw.step == 7u) && s_sw.magic_ok)
        {
            continue; /* Skip factory cal when user cal is valid */
        }
        if ((s_sw.step == 11u) && (s_sw.first_mode == 0x30u))
        {
            continue; /* Skip input mode set if 0x30 already received */
        }
        break;
    }
    sw_send(now);
}

/* One reply-wait attempt failed. */
static void sw_attempt_failed(uint64_t now)
{
    s_sw.tries++;
    s_sw.wait_out = 0u;
    if (s_sw.step == 2u)
    {
        if ((now - s_sw.s2_first) < SW_S2_SPAN)
        {
            sw_send(now);
        }
        else
        {
            sw_fail("[app] switch S2 (80 02) no reply for 5 s, stop");
        }
        return;
    }
    if (s_sw.tries < SW_TRIES)
    {
        sw_send(now);
        return;
    }
    if (s_sw.step == 1u)
    {
        s_sw.round++;
        s_sw.tries = 0u;
        if (s_sw.round < SW_S1_ROUNDS)
        {
            sw_send(now);
            return;
        }
        sw_advance(now);
        return;
    }
    if (s_sw.step == 7u)
    {
        sw_fail("[app] switch S7 (stick calibration) failed, stop");
        return;
    }
    app_log2("[app] switch step failed (ignored) S", s_sw.step, (const char *)0, 0u);
    sw_advance(now);
}

static void sw_success(uint64_t now)
{
    if (s_sw.step == 1u)
    {
        s_sw.round++;
        s_sw.tries = 0u;
        if (s_sw.round < SW_S1_ROUNDS)
        {
            s_sw.wait_out = 0u;
            sw_send(now);
            return;
        }
    }
    sw_advance(now);
}

void app_switch_start(const AppTarget *t, uint8_t mode, uint64_t now)
{
    lat_bytes_fill((uint8_t *)&s_sw, 0u, (uint32_t)sizeof(s_sw));
    s_sw.t = *t;
    s_sw.mode = mode;
    if ((t->out_pipe == 0u) || (t->in_pipe == 0u))
    {
        s_sw.state = ENUM_APP_FAILED;
        enum_port_log("[app] switch: no interrupt OUT/IN pipe");
        return;
    }
    s_sw.active = 1u;
    s_sw.state = ENUM_APP_RUNNING;
    s_sw.step = (mode == APP_SW_INPUT_MINIMAL) ? 2u : 1u;
    app_log2("[app] switch init start mode=", mode, (const char *)0, 0u);
    sw_send(now);
}

static void sw_report(const uint8_t *d, uint16_t len, uint64_t t1)
{
    bool hit = false;

    if (len == 0u)
    {
        return;
    }
    if (((d[0] == 0x30u) || (d[0] == 0x31u) || (d[0] == 0x3Fu)) && (s_sw.first_mode == 0u) && !s_sw.s2_sent)
    {
        s_sw.first_mode = d[0];
    }
    if (d[0] == 0x30u)
    {
        s_sw.last30 = t1;
    }
    if (!s_sw.active || (s_sw.step == SW_STEP_STEADY))
    {
        return;
    }
    if ((s_sw.expect == EX_81) && (len >= 2u) && (d[0] == 0x81u) && (d[1] == s_sw.exp_id))
    {
        hit = true;
    }
    else if ((s_sw.expect == EX_21) && (len >= 15u) && (d[0] == 0x21u) && ((d[13] & 0x80u) != 0u) &&
             (d[14] == s_sw.exp_id))
    {
        hit = (s_sw.spi_len == 0u) || ((len >= 20u) && lat_bytes_equal(&d[15], s_sw.spi, 5u));
        if (hit && (s_sw.step == 6u))
        {
            s_sw.magic_ok = (uint8_t)((len >= 33u) && (d[20] == 0xB2u) && (d[21] == 0xA1u) && (d[31] == 0xB2u) &&
                                      (d[32] == 0xA1u));
        }
    }
    if (hit)
    {
        sw_success(t1);
    }
}

static void sw_out_done(uint64_t t)
{
    if (!s_sw.active || !s_sw.wait_out)
    {
        return;
    }
    s_sw.wait_out = 0u;
    if (s_sw.expect == EX_NONE)
    {
        sw_advance(t); /* 80 04 expects no reply */
        return;
    }
    s_sw.deadline = t + SW_REPLY_WAIT;
}

static void sw_poll(uint64_t now)
{
    if (!s_sw.active)
    {
        return;
    }
    if (s_sw.step == SW_STEP_STEADY)
    {
        /* Steady state: resend 80 04 if 0x30 is absent for >= 100ms (max once per 100ms) */
        if (((now - s_sw.last30) >= SW_STEADY_GAP) && ((now - s_sw.last8004) >= SW_STEADY_GAP))
        {
            s_sw.last8004 = now;
            (void)sw_usbcmd(0x04u, now);
        }
        return;
    }
    if (s_sw.wait_out)
    {
        if ((now - s_sw.push_t) >= SW_OUT_LIMIT)
        {
            sw_attempt_failed(now);
        }
        return;
    }
    if ((s_sw.expect != EX_NONE) && ((int64_t)(now - s_sw.deadline) >= 0))
    {
        sw_attempt_failed(now);
    }
}

/* ------------------------------------------------------------------ */
/* GIP                                                                  */
/* ------------------------------------------------------------------ */

static void gip_add(const uint8_t *p, uint8_t n)
{
    if (s_gip.count < GIP_PKT_MAX)
    {
        lat_bytes_copy(s_gip.pkt[s_gip.count], p, n);
        s_gip.len[s_gip.count] = n;
        s_gip.count++;
    }
}

static bool gip_id(uint16_t vid, uint16_t pid)
{
    return (s_gip.t.vid == vid) && (s_gip.t.pid == pid);
}

static void gip_send_next(void)
{
    uint8_t b[GIP_LEN_MAX];
    uint8_t n;

    if (!s_gip.active || s_gip.wait_announce || (s_gip.outstanding != 0u))
    {
        return;
    }
    if (s_gip.next >= s_gip.count)
    {
        if (s_gip.state == ENUM_APP_RUNNING)
        {
            s_gip.state = ENUM_APP_DONE;
            app_log2("[app] gip init done packets=", s_gip.count, " ss=", s_gip.ss);
        }
        return;
    }
    n = s_gip.len[s_gip.next];
    lat_bytes_copy(b, s_gip.pkt[s_gip.next], n);
    b[2] = s_gip.ss; /* Starts at 0 and increments per send, matching xpad */
    if (usbhs_intr_out_push(s_gip.t.out_pipe, b, n))
    {
        s_gip.ss = (uint8_t)(s_gip.ss + 1u);
        s_gip.next++;
        s_gip.outstanding++;
    }
}

void app_gip_start(const AppTarget *t, uint64_t now)
{
    static const uint8_t p_hori[13] = {0x01u, 0x20u, 0x00u, 0x09u, 0x00u, 0x04u, 0x20u, 0x3Au, 0x00u, 0x00u, 0x00u, 0x80u, 0x00u};
    static const uint8_t p_power[5] = {0x05u, 0x20u, 0x00u, 0x01u, 0x00u};
    static const uint8_t p_s_init[5] = {0x05u, 0x20u, 0x00u, 0x0Fu, 0x06u};
    static const uint8_t p_elite[6] = {0x4Du, 0x10u, 0x00u, 0x02u, 0x07u, 0x00u};
    static const uint8_t p_led[7] = {0x0Au, 0x20u, 0x00u, 0x03u, 0x00u, 0x01u, 0x14u};
    static const uint8_t p_auth[6] = {0x06u, 0x20u, 0x00u, 0x02u, 0x01u, 0x00u};
    static const uint8_t p_rumble_on[13] = {0x09u, 0x00u, 0x00u, 0x09u, 0x00u, 0x0Fu, 0x00u, 0x00u, 0x1Du, 0x1Du, 0xFFu, 0x00u, 0x00u};
    static const uint8_t p_rumble_off[13] = {0x09u, 0x00u, 0x00u, 0x09u, 0x00u, 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};
    bool powera;

    (void)now;
    lat_bytes_fill((uint8_t *)&s_gip, 0u, (uint32_t)sizeof(s_gip));
    s_gip.t = *t;
    if (t->out_pipe == 0u)
    {
        s_gip.state = ENUM_APP_FAILED;
        enum_port_log("[app] gip: no interrupt OUT pipe");
        return;
    }
    /* GIP init sequence */
    if (gip_id(0x0E6Fu, 0x0165u) || gip_id(0x0F0Du, 0x0067u))
    {
        gip_add(p_hori, 13u);
    }
    gip_add(p_power, 5u);
    if (gip_id(0x045Eu, 0x02EAu) || gip_id(0x045Eu, 0x0B00u))
    {
        gip_add(p_s_init, 5u);
    }
    if (gip_id(0x045Eu, 0x0B00u))
    {
        gip_add(p_elite, 6u);
    }
    gip_add(p_led, 7u);
    gip_add(p_auth, 6u);
    powera = gip_id(0x24C6u, 0x541Au) || gip_id(0x24C6u, 0x542Au) || gip_id(0x24C6u, 0x543Au);
    if (powera)
    {
        gip_add(p_rumble_on, 13u);
        gip_add(p_rumble_off, 13u);
    }
    s_gip.active = 1u;
    s_gip.state = ENUM_APP_RUNNING;
    /* ByoWave (366c:0005) waits for announce before starting */
    s_gip.wait_announce = gip_id(0x366Cu, 0x0005u) ? 1u : 0u;
    app_log2("[app] gip init start packets=", s_gip.count, " wait_announce=", s_gip.wait_announce);
    gip_send_next();
}

static void gip_report(const uint8_t *d, uint16_t len)
{
    if (!s_gip.active || (len < 3u))
    {
        return;
    }
    if ((d[0] == 0x02u) && s_gip.wait_announce)
    {
        s_gip.wait_announce = 0u;
        gip_send_next();
        return;
    }
    if ((d[0] == 0x07u) && (d[1] == 0x30u))
    {
        uint8_t ack[13] = {0x01u, 0x20u, 0x00u, 0x09u, 0x00u, 0x07u, 0x20u, 0x02u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};

        ack[2] = d[2]; /* Echo received sequence number */
        if (usbhs_intr_out_push(s_gip.t.out_pipe, ack, 13u))
        {
            s_gip.outstanding++;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Common */
/* ------------------------------------------------------------------ */

void app_on_report(uint8_t pipe, const uint8_t *d, uint16_t len, uint64_t t1)
{
    if ((s_sw.state != ENUM_APP_NONE) && (pipe == s_sw.t.in_pipe))
    {
        sw_report(d, len, t1);
    }
    if ((s_gip.state != ENUM_APP_NONE) && (pipe == s_gip.t.in_pipe))
    {
        gip_report(d, len);
    }
}

void app_on_out_done(uint8_t pipe, uint64_t t)
{
    if ((s_sw.state != ENUM_APP_NONE) && (pipe == s_sw.t.out_pipe))
    {
        sw_out_done(t);
    }
    if ((s_gip.state != ENUM_APP_NONE) && (pipe == s_gip.t.out_pipe))
    {
        if (s_gip.outstanding > 0u)
        {
            s_gip.outstanding--;
        }
        gip_send_next();
    }
}

void app_poll(uint64_t now)
{
    sw_poll(now);
}
