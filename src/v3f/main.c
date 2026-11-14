/*
 * CH32H417 latency tester - V3F (boot/housekeeping core) main
 *
 * Boot sequence:
 *   1. SystemInit() -> SystemAndCoreClockUpdate()
 *   2. UART (USART6, PC10)
 *   3. Verify PPRE2 (TIM1/8/9/10 divider == 1)
 *   4. Enable GPIOA/TIM9 clocks; configure PA0..PA7 as open-drain released (high-Z)
 *   5. Enable USBHS PLL, UTMI, and USBHS bus clock (USBHS driver runs on V5F)
 *   6. Route USBHS_IRQn (56) and TIM9_IRQn (90) to V5F and read back to verify
 *   7. Re-verify PPRE2
 *   8. Zero-init shared SRAM, increment boot_epoch, release-store READY, wake V5F
 *   9. Route USBFS_IRQn (67) to V3F, enable USBFS clock (48MHz), start CDC-ACM
 *
 * UART output is ASCII only. Do not use SDK Delay_Us/Delay_Ms (overwrites SysTick0).
 */
#include <stdbool.h>
#include <stdint.h>

#include "ch32h417.h"

#include "boot_magic.h"
#include "cdc_acm.h"
#include "console.h"
#include "irq_route.h"
#include "lat_fmt.h"
#include "lat_ipc.h"
#include "lat_uart.h"
#include "lat_version.h"
#include "pc_link.h"
#include "pc_link_port.h"
#include "usbfs_dev.h"
#include "v3f_time.h"

/* ---- V3F diagnostic codes ---- */
#define V3F_DIAG_PPRE2_TIM_DIV    ((uint32_t)0xD001u) /* TIM1/8/9/10 divider != 1 */
#define V3F_DIAG_SHM_ADDRESS      ((uint32_t)0xD002u) /* shared SRAM address mismatch */
#define V3F_DIAG_IRQ_ROUTE_USBHS  ((uint32_t)0xD003u) /* failed to route USBHS_IRQn to V5F */
#define V3F_DIAG_IRQ_ROUTE_TIM9   ((uint32_t)0xD004u) /* failed to route TIM9_IRQn to V5F */
#define V3F_DIAG_V5F_NOT_RUNNING  ((uint32_t)0xD005u) /* V5F failed to start in time */
#define V3F_DIAG_USBFS            ((uint32_t)0xD006u) /* USBFS init failed (IRQ route or clock) */

/* Trigger outputs T0..T7 = PA0..PA7. */
#define V3F_TRIGGER_PINS          ((uint16_t)0x00FFu)

#define V3F_V5F_START_TIMEOUT_MS  (1000u)
#define V3F_PERIOD_MS             (1000u)
/* Max seqlock retries. */
#define V3F_SEQLOCK_TRIES         (16u)

typedef struct V3fState
{
    uint32_t epoch;
    bool v5f_running;
    bool ping_outstanding;
    uint32_t ping_id;
    uint32_t ping_ok;
    uint32_t ping_timeout;
    uint32_t ping_mismatch;
    uint32_t cmd_full;
    uint32_t heartbeat;
    uint32_t last_v5f_heartbeat;
    uint64_t next_ms;
    uint64_t next_period;
    uint32_t uptime_s;
    bool usb_started;
} V3fState;

static V3fState s_v3f;

/* ------------------------------------------------------------------ */
/* Output helpers                                                       */
/* ------------------------------------------------------------------ */

static void out_line(LatStr *s)
{
    pc_link_push_log(s->buf, s->len);
    lat_str_puts(s, "\r\n");
    con_puts(s->buf);
}

/* Halt and emit diagnostic code every second; V5F is not woken. */
static void v3f_halt(uint32_t diag, const char *what, uint32_t detail) __attribute__((noreturn));
static void v3f_halt(uint32_t diag, const char *what, uint32_t detail)
{
    char buf[160];
    LatStr s;

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[v3f] HALT diag=0x");
    lat_str_hex32(&s, diag);
    lat_str_puts(&s, " ");
    lat_str_puts(&s, what);
    lat_str_puts(&s, " detail=0x");
    lat_str_hex32(&s, detail);
    lat_str_puts(&s, " - v5f is not started\r\n");

    for (;;)
    {
        con_puts(buf);
        con_flush();
        v3f_delay_ms(1000u);
    }
}

/* ------------------------------------------------------------------ */
/* Initialization                                                       */
/* ------------------------------------------------------------------ */

/*
 * Verify TIM1/8/9/10 input clock divider is 1 (RCC_PPRE2_TIM_DIV1 = 0)
 * so TIM9 runs at 100MHz.
 */
static void ppre2_check_or_halt(const char *when)
{
    uint32_t cfgr0 = RCC->CFGR0;
    char buf[96];
    LatStr s;

    if ((cfgr0 & RCC_PPRE2) != RCC_PPRE2_TIM_DIV1)
    {
        v3f_halt(V3F_DIAG_PPRE2_TIM_DIV, "PPRE2 TIM prescaler is not DIV1 (CFGR0)", cfgr0);
    }
    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[v3f] ppre2 check (");
    lat_str_puts(&s, when);
    lat_str_puts(&s, "): TIM DIV1 OK, CFGR0=0x");
    lat_str_hex32(&s, cfgr0);
    out_line(&s);
    con_flush();
}

/* Verify shared SRAM symbols match lat_ipc.h fixed addresses. */
static void shm_address_check_or_halt(void)
{
    static const struct
    {
        const char *name;
        uint32_t actual;
        uint32_t expected;
    } table[] = {
        { "g_usbhs_dma", (uint32_t)(uintptr_t)&g_usbhs_dma, LAT_SHM_USBHS_DMA_ADDR },
        { "g_object_buf", (uint32_t)(uintptr_t)&g_object_buf, LAT_SHM_OBJECT_BUF_ADDR },
        { "g_ipc_header", (uint32_t)(uintptr_t)&g_ipc_header, LAT_SHM_IPC_HEADER_ADDR },
        { "g_ipc_cmd", (uint32_t)(uintptr_t)&g_ipc_cmd, LAT_SHM_IPC_CMD_ADDR },
        { "g_ipc_evt_safe", (uint32_t)(uintptr_t)&g_ipc_evt_safe, LAT_SHM_IPC_EVT_SAFE_ADDR },
        { "g_ipc_evt_lossy", (uint32_t)(uintptr_t)&g_ipc_evt_lossy, LAT_SHM_IPC_EVT_LOSSY_ADDR },
        { "g_boot_magic", (uint32_t)(uintptr_t)&g_boot_magic, LAT_SHM_BOOT_MAGIC_ADDR },
    };
    uint32_t i;

    for (i = 0u; i < (uint32_t)(sizeof(table) / sizeof(table[0])); i++)
    {
        if (table[i].actual != table[i].expected)
        {
            v3f_halt(V3F_DIAG_SHM_ADDRESS, table[i].name, table[i].actual);
        }
    }
}

/*
 * V3F owns RCC: enable GPIOA/TIM9 clocks for V5F and preset PA0..PA7
 * output data to 1 (open-drain high-Z) before configuring as outputs.
 */
static void gpio_and_timer_clock_init(void)
{
    GPIO_InitTypeDef gpio_cfg = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO |
                          RCC_HB2Periph_GPIOA |
                          RCC_HB2Periph_GPIOB |
                          RCC_HB2Periph_GPIOC |
                          RCC_HB2Periph_GPIOD |
                          RCC_HB2Periph_GPIOE |
                          RCC_HB2Periph_GPIOF |
                          RCC_HB2Periph_TIM9,
                          ENABLE);
    /* Hardware RNG used by V5F for random trial delays. */
    RCC_HBPeriphClockCmd(RCC_HBPeriph_RNG, ENABLE);

    GPIOA->BSHR = V3F_TRIGGER_PINS;
    gpio_cfg.GPIO_Pin = V3F_TRIGGER_PINS;
    gpio_cfg.GPIO_Speed = GPIO_Speed_Very_High;
    gpio_cfg.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &gpio_cfg);
    GPIOA->BSHR = V3F_TRIGGER_PINS;
}

/*
 * Enable USBHS PLL, UTMI PHY clock, and USBHS bus clock before waking V5F.
 * RCC_UTMIcmd(ENABLE) is required for the USB PHY.
 */
static void usbhs_clock_init(void)
{
    /* At 480MHz, SystemInit() uses USBHS PLL as system PLL input; skip reconfig. */
    if ((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS)
    {
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig(RCC_USBHSPLLSource_HSE);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_USBHSPLLRDY) == RESET)
        {
        }
    }
    RCC_UTMIcmd(ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);
}

static bool route_one(IRQn_Type irq, const char *name)
{
    bool ok = irq_route_to_core(irq, Core_ID_V5F);
    char buf[96];
    LatStr s;

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[v3f] irq_route ");
    lat_str_puts(&s, name);
    lat_str_puts(&s, "(");
    lat_str_u32(&s, (uint32_t)irq);
    lat_str_puts(&s, ") -> V5F: ");
    lat_str_puts(&s, ok ? "OK" : "FAIL");
    lat_str_puts(&s, " (IALLOCR=0x");
    lat_str_hex32(&s, (uint32_t)PFIC->IALLOCR[(uint32_t)irq]);
    lat_str_puts(&s, ")");
    out_line(&s);
    con_flush();
    return ok;
}

/*
 * Use PFIC->IALLOCR directly (SDK NVIC_SetAllocateIRQ ignores IRQ >= 32).
 */
static void irq_routing_or_halt(void)
{
    if (!route_one(USBHS_IRQn, "USBHS_IRQn"))
    {
        v3f_halt(V3F_DIAG_IRQ_ROUTE_USBHS, "USBHS_IRQn route readback mismatch",
                 (uint32_t)PFIC->IALLOCR[(uint32_t)USBHS_IRQn]);
    }
    if (!route_one(TIM9_IRQn, "TIM9_IRQn"))
    {
        v3f_halt(V3F_DIAG_IRQ_ROUTE_TIM9, "TIM9_IRQn route readback mismatch",
                 (uint32_t)PFIC->IALLOCR[(uint32_t)TIM9_IRQn]);
    }
}

static void zero_words(void *p, uint32_t bytes)
{
    volatile uint32_t *w = (volatile uint32_t *)p;
    uint32_t i;

    for (i = 0u; i < (bytes / 4u); i++)
    {
        w[i] = 0u;
    }
}

/*
 * Zero-init shared SRAM, init header/rings, and publish READY.
 * Increments previous valid boot_epoch, or starts at 1 (0 is invalid).
 */
static uint32_t ipc_init_shared(void)
{
    const volatile LatIpcHeader *old = &g_ipc_header;
    uint32_t epoch = 1u;

    if ((old->magic == LAT_IPC_MAGIC) && (old->boot_epoch_inv == ~old->boot_epoch))
    {
        epoch = old->boot_epoch + 1u;
        if (epoch == 0u)
        {
            epoch = 1u;
        }
    }

    zero_words(&g_usbhs_dma, sizeof(g_usbhs_dma));
    zero_words(&g_object_buf, sizeof(g_object_buf));
    zero_words(&g_ipc_header, sizeof(g_ipc_header));
    zero_words(&g_ipc_cmd, sizeof(g_ipc_cmd));
    zero_words(&g_ipc_evt_safe, sizeof(g_ipc_evt_safe));
    zero_words(&g_ipc_evt_lossy, sizeof(g_ipc_evt_lossy));

    g_ipc_header.magic = LAT_IPC_MAGIC;
    g_ipc_header.abi_version = LAT_IPC_ABI_VERSION;
    g_ipc_header.struct_size = (uint16_t)sizeof(LatIpcHeader);
    g_ipc_header.boot_epoch = epoch;
    g_ipc_header.boot_epoch_inv = ~epoch;
    g_ipc_header.v5f_diag = LAT_V5F_DIAG_NONE;

    lat_ring_setup(&g_ring_cmd);
    lat_ring_setup(&g_ring_evt_safe);
    lat_ring_setup(&g_ring_evt_lossy);

    /* Publish all shared SRAM writes before V5F's acquire load in lat_ipc_v5f_validate(). */
    lat_ipc_boot_state_store(LAT_BOOT_STATE_READY);
    return epoch;
}

static void print_memory_map(void)
{
    char buf[200];
    LatStr s;

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[v3f] shm dma=0x");
    lat_str_hex32(&s, (uint32_t)(uintptr_t)&g_usbhs_dma);
    lat_str_puts(&s, " obj=0x");
    lat_str_hex32(&s, (uint32_t)(uintptr_t)&g_object_buf);
    lat_str_puts(&s, " hdr=0x");
    lat_str_hex32(&s, (uint32_t)(uintptr_t)&g_ipc_header);
    lat_str_puts(&s, " cmd=0x");
    lat_str_hex32(&s, (uint32_t)(uintptr_t)&g_ipc_cmd);
    lat_str_puts(&s, " safe=0x");
    lat_str_hex32(&s, (uint32_t)(uintptr_t)&g_ipc_evt_safe);
    lat_str_puts(&s, " lossy=0x");
    lat_str_hex32(&s, (uint32_t)(uintptr_t)&g_ipc_evt_lossy);
    lat_str_puts(&s, " bm=0x");
    lat_str_hex32(&s, (uint32_t)(uintptr_t)&g_boot_magic);
    out_line(&s);
    con_flush();
}

/* Wait for V5F to pass validation and publish RUNNING. */
static bool wait_v5f_running(uint32_t timeout_ms)
{
    const uint32_t per_ms = v3f_ticks_per_us() * 1000u;
    uint32_t start = v3f_ticks();
    uint32_t elapsed_ms = 0u;

    while (elapsed_ms < timeout_ms)
    {
        if (lat_ipc_boot_state_load() == LAT_BOOT_STATE_V5F_RUNNING)
        {
            return true;
        }
        elapsed_ms = (uint32_t)(v3f_ticks() - start) / per_ms;
    }
    return lat_ipc_boot_state_load() == LAT_BOOT_STATE_V5F_RUNNING;
}

/* ------------------------------------------------------------------ */
/* Main loop                                                            */
/* ------------------------------------------------------------------ */

static void handle_log(const LatRingRecord *rec)
{
    char buf[LAT_RING_PAYLOAD_MAX + 3u];
    uint32_t i;

    for (i = 0u; i < rec->len; i++)
    {
        uint8_t c = rec->payload[i];

        /* Emit printable ASCII only. */
        buf[i] = ((c >= 0x20u) && (c <= 0x7Eu)) ? (char)c : '?';
    }
    pc_link_push_log(buf, i);
    buf[i++] = '\r';
    buf[i++] = '\n';
    buf[i] = '\0';
    con_puts(buf);
}

static void handle_pong(const LatRingRecord *rec)
{
    uint32_t now_ticks = v3f_ticks();
    LatPongEvt pong;
    uint8_t *dst = (uint8_t *)&pong;
    char buf[160];
    LatStr s;
    uint32_t i;

    if (rec->len < sizeof(pong))
    {
        return;
    }
    for (i = 0u; i < sizeof(pong); i++)
    {
        dst[i] = rec->payload[i];
    }

    lat_str_init(&s, buf, sizeof(buf));
    if (s_v3f.ping_outstanding && (pong.ping_id == s_v3f.ping_id))
    {
        uint32_t rtt_ticks = now_ticks - pong.v3f_ticks;

        s_v3f.ping_outstanding = false;
        s_v3f.ping_ok++;
        lat_str_puts(&s, "[v3f] ping id=");
        lat_str_u32(&s, pong.ping_id);
        lat_str_puts(&s, " rtt_ns=");
        lat_str_u32(&s, (uint32_t)(((uint64_t)rtt_ticks * 1000u) / v3f_ticks_per_us()));
        lat_str_puts(&s, " v5f_tim9=");
        lat_str_hex32(&s, pong.tim9_hi);
        lat_str_putc(&s, ':');
        lat_str_hex32(&s, pong.tim9_lo);
        lat_str_puts(&s, " hb5=");
        lat_str_u32(&s, pong.v5f_heartbeat);
    }
    else
    {
        s_v3f.ping_mismatch++;
        lat_str_puts(&s, "[v3f] pong id=");
        lat_str_u32(&s, pong.ping_id);
        lat_str_puts(&s, " does not match the outstanding ping");
    }
    out_line(&s);
}

static void handle_other(const char *ring, const LatRingRecord *rec)
{
    char buf[96];
    LatStr s;

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[v3f] unknown event ring=");
    lat_str_puts(&s, ring);
    lat_str_puts(&s, " type=");
    lat_str_u32(&s, rec->type);
    lat_str_puts(&s, " len=");
    lat_str_u32(&s, rec->len);
    out_line(&s);
}

/*
 * Emit non-PC-link IPC records (PONG, UART logs) drained by pc_link.c.
 * Never stalls safe ring drain; con_puts drops if TX buffer is full.
 */
void pc_port_ipc_record(const LatRingRecord *rec)
{
    switch (rec->type)
    {
    case LAT_EVT_PONG:
        handle_pong(rec);
        break;
    case LAT_EVT_LOG:
        handle_log(rec);
        break;
    default:
        handle_other((rec->type == LAT_EVT_LOG) ? "lossy" : "ipc", rec);
        break;
    }
}

static void send_ping(void)
{
    LatPingCmd cmd;
    char buf[96];
    LatStr s;

    lat_str_init(&s, buf, sizeof(buf));
    if (s_v3f.ping_outstanding)
    {
        s_v3f.ping_timeout++;
        lat_str_puts(&s, "[v3f] ping id=");
        lat_str_u32(&s, s_v3f.ping_id);
        lat_str_puts(&s, " timeout (no pong within 1s)");
        out_line(&s);
        s_v3f.ping_outstanding = false;
    }

    s_v3f.ping_id++;
    cmd.ping_id = s_v3f.ping_id;
    cmd.v3f_ticks = v3f_ticks();
    if (lat_ring_try_push(&g_ring_cmd, LAT_CMD_PING, 0u, &cmd, sizeof(cmd)))
    {
        s_v3f.ping_outstanding = true;
    }
    else
    {
        s_v3f.cmd_full++;
        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[v3f] ping id=");
        lat_str_u32(&s, s_v3f.ping_id);
        lat_str_puts(&s, " not sent: command ring full");
        out_line(&s);
    }
}

static bool read_v5f_status(LatV5fStatus *out)
{
    const LatV5fStatus *st = &g_ipc_header.v5f_status;
    uint32_t tries;

    for (tries = 0u; tries < V3F_SEQLOCK_TRIES; tries++)
    {
        uint32_t seq = lat_seqlock_read_begin(&st->seq);

        out->tim9_isr_count = lat_ipc_word_load(&st->tim9_isr_count);
        out->tim9_update_count = lat_ipc_word_load(&st->tim9_update_count);
        out->tim9_backsteps = lat_ipc_word_load(&st->tim9_backsteps);
        out->loop_count = lat_ipc_word_load(&st->loop_count);
        out->cmd_count = lat_ipc_word_load(&st->cmd_count);
        out->cmd_deferred = lat_ipc_word_load(&st->cmd_deferred);
        if (!lat_seqlock_read_retry(&st->seq, seq))
        {
            return true;
        }
    }
    return false;
}

static void print_status(void)
{
    const LatRingHead *lossy = g_ring_evt_lossy.head;
    const LatRingHead *safe = g_ring_evt_safe.head;
    LatV5fStatus st = {0};
    uint64_t t9 = 0u;
    bool st_ok;
    bool t9_ok;
    uint32_t hb5;
    char buf[320];
    LatStr s;

    hb5 = lat_ipc_word_load(&g_ipc_header.v5f_heartbeat);
    st_ok = read_v5f_status(&st);
    t9_ok = lat_ipc_time_read(&t9, V3F_SEQLOCK_TRIES);

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[v3f] st up=");
    lat_str_u32(&s, s_v3f.uptime_s);
    lat_str_puts(&s, "s v5f=");
    if (s_v3f.v5f_running)
    {
        lat_str_puts(&s, "RUN");
    }
    else
    {
        lat_str_puts(&s, "DOWN(state=");
        lat_str_u32(&s, lat_ipc_boot_state_load());
        lat_str_puts(&s, " diag=0x");
        lat_str_hex32(&s, lat_ipc_v5f_diag_load());
        lat_str_puts(&s, ")");
    }
    lat_str_puts(&s, " hb5=");
    lat_str_u32(&s, hb5);
    lat_str_puts(&s, "(+");
    lat_str_u32(&s, hb5 - s_v3f.last_v5f_heartbeat);
    lat_str_puts(&s, ") tim9=");
    if (t9_ok)
    {
        lat_str_hex32(&s, (uint32_t)(t9 >> 32));
        lat_str_putc(&s, ':');
        lat_str_hex32(&s, (uint32_t)t9);
    }
    else
    {
        lat_str_puts(&s, "busy");
    }
    if (st_ok)
    {
        lat_str_puts(&s, " isr=");
        lat_str_u32(&s, st.tim9_isr_count);
        lat_str_puts(&s, " upd=");
        lat_str_u32(&s, st.tim9_update_count);
        lat_str_puts(&s, " bk=");
        lat_str_u32(&s, st.tim9_backsteps);
        lat_str_puts(&s, " cmd5=");
        lat_str_u32(&s, st.cmd_count);
        lat_str_puts(&s, " defer5=");
        lat_str_u32(&s, st.cmd_deferred);
    }
    else
    {
        lat_str_puts(&s, " status=busy");
    }
    lat_str_puts(&s, " lossy_drop=");
    lat_str_u32(&s, lat_ipc_word_load(&lossy->hdr.drop_count));
    lat_str_puts(&s, "(log=");
    lat_str_u32(&s, lat_ipc_word_load(&lossy->lost_by_type[LAT_EVT_LOG]));
    lat_str_puts(&s, " mon=");
    lat_str_u32(&s, lat_ipc_word_load(&lossy->lost_by_type[LAT_EVT_MONITOR]));
    lat_str_puts(&s, ") safe_ovf=");
    lat_str_u32(&s, lat_ipc_word_load(&safe->hdr.overflow_count));
    lat_str_puts(&s, " cmd_full=");
    lat_str_u32(&s, s_v3f.cmd_full);
    lat_str_puts(&s, " con_drop=");
    lat_str_u32(&s, con_drop_count());
    lat_str_puts(&s, " ping ok=");
    lat_str_u32(&s, s_v3f.ping_ok);
    lat_str_puts(&s, " to=");
    lat_str_u32(&s, s_v3f.ping_timeout);
    out_line(&s);

    s_v3f.last_v5f_heartbeat = hb5;
}

/* PC link status (once per second). */
static void print_pc_status(void)
{
    PcLinkStats pc;
    CdcAcmStats usb;
    char buf[320];
    LatStr s;

    if (!s_v3f.usb_started)
    {
        return;
    }
    pc_link_get_stats(&pc);
    cdc_acm_get_stats(&usb);

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[v3f] pc usb=");
    lat_str_puts(&s, usb.configured ? "CFG" : "---");
    lat_str_puts(&s, " dtr=");
    lat_str_u32(&s, usb.dtr ? 1u : 0u);
    lat_str_puts(&s, " rst=");
    lat_str_u32(&s, usb.bus_resets);
    lat_str_puts(&s, " sess=0x");
    lat_str_hex32(&s, pc.session_id);
    lat_str_puts(&s, pc.hello_pending ? "(pending)" : "");
    lat_str_puts(&s, " rx=");
    lat_str_u32(&s, pc.rx_frames);
    lat_str_puts(&s, "/");
    lat_str_u32(&s, usb.rx_bytes);
    lat_str_puts(&s, "B bad(cobs/len/crc/ovr)=");
    lat_str_u32(&s, pc.rx_bad_cobs);
    lat_str_putc(&s, '/');
    lat_str_u32(&s, pc.rx_bad_length);
    lat_str_putc(&s, '/');
    lat_str_u32(&s, pc.rx_bad_crc);
    lat_str_putc(&s, '/');
    lat_str_u32(&s, pc.rx_overrun);
    lat_str_puts(&s, " drop=");
    lat_str_u32(&s, pc.rx_dropped);
    lat_str_puts(&s, " tx=");
    lat_str_u32(&s, pc.tx_frames);
    lat_str_puts(&s, "/");
    lat_str_u32(&s, usb.tx_bytes);
    lat_str_puts(&s, "B fwd_to=");
    lat_str_u32(&s, pc.fwd_timeouts);
    lat_str_puts(&s, " stale5=");
    lat_str_u32(&s, pc.v5f_stale);
    lat_str_puts(&s, " drop(mon/log)=");
    lat_str_u32(&s, pc.drop_monitor);
    lat_str_putc(&s, '/');
    lat_str_u32(&s, pc.drop_log);
    lat_str_puts(&s, pc.overflow ? " OVERFLOW" : "");
    out_line(&s);

    /* USBFS diagnostics. */
    {
        UsbfsDiag dg;
        uint32_t i;

        usbfs_dev_diag(&dg);
        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[v3f] usbfs isr=");
        lat_str_u32(&s, dg.isr);
        lat_str_puts(&s, " setup=");
        lat_str_u32(&s, dg.xfer_setup);
        lat_str_puts(&s, " in=");
        lat_str_u32(&s, dg.xfer_in);
        lat_str_puts(&s, " out=");
        lat_str_u32(&s, dg.xfer_out);
        lat_str_puts(&s, " oth=");
        lat_str_u32(&s, dg.xfer_other);
        lat_str_puts(&s, " rst=");
        lat_str_u32(&s, dg.bus_reset);
        lat_str_puts(&s, " susp=");
        lat_str_u32(&s, dg.suspend);
        lat_str_puts(&s, " stall=");
        lat_str_u32(&s, dg.stall);
        lat_str_puts(&s, " in_sent=");
        lat_str_u32(&s, dg.ep0_in_sent);
        lat_str_puts(&s, " setaddr=");
        lat_str_u32(&s, dg.set_address);
        lat_str_puts(&s, " fg/st/mis/udev/base/addr/t0/r0/tlen=");
        lat_str_hex32(&s, ((uint32_t)dg.last_int_fg << 24) | ((uint32_t)dg.last_int_st << 16) |
                              ((uint32_t)dg.mis_st << 8) | dg.udev_ctrl);
        lat_str_putc(&s, ':');
        lat_str_hex32(&s, ((uint32_t)dg.base_ctrl << 24) | ((uint32_t)dg.dev_addr << 16) |
                              ((uint32_t)dg.ep0_tctrl << 8) | dg.ep0_rctrl);
        lat_str_putc(&s, ':');
        lat_str_u32(&s, dg.ep0_tlen);
        lat_str_puts(&s, " rxlen=");
        lat_str_u32(&s, dg.last_rx_len);
        lat_str_puts(&s, " setup8=");
        for (i = 0u; i < 8u; i += 4u)
        {
            lat_str_hex32(&s, ((uint32_t)dg.last_setup[i] << 24) | ((uint32_t)dg.last_setup[i + 1u] << 16) |
                                  ((uint32_t)dg.last_setup[i + 2u] << 8) | dg.last_setup[i + 3u]);
        }
        out_line(&s);
    }
}

/*
 * Route USBFS_IRQn (67) to V3F, enable USBFS clock, and start CDC-ACM.
 * On failure, log diagnostic code and continue without USB.
 */
static void usb_start(void)
{
    uint8_t uuid[16];
    bool route_ok = irq_route_to_core(USBFS_IRQn, Core_ID_V3F);
    char buf[128];
    LatStr s;

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[v3f] irq_route USBFS_IRQn(");
    lat_str_u32(&s, (uint32_t)USBFS_IRQn);
    lat_str_puts(&s, ") -> V3F: ");
    lat_str_puts(&s, route_ok ? "OK" : "FAIL");
    lat_str_puts(&s, " (IALLOCR=0x");
    lat_str_hex32(&s, (uint32_t)PFIC->IALLOCR[(uint32_t)USBFS_IRQn]);
    lat_str_puts(&s, ")");
    out_line(&s);
    if (!route_ok)
    {
        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[v3f] ERROR diag=0x");
        lat_str_hex32(&s, V3F_DIAG_USBFS);
        lat_str_puts(&s, " USBFS_IRQn is not routed to V3F - USB is not started");
        out_line(&s);
        con_flush();
        return;
    }
    if (!usbfs_dev_clock_init())
    {
        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[v3f] ERROR diag=0x");
        lat_str_hex32(&s, V3F_DIAG_USBFS);
        lat_str_puts(&s, " USBHS PLL is not ready - USB is not started");
        out_line(&s);
        con_flush();
        return;
    }
    pc_port_uuid(uuid);
    cdc_acm_start(uuid);
    s_v3f.usb_started = true;
    con_puts("[v3f] usbfs cdc-acm started (16C0:05E1, EP 0x01 OUT / 0x82 IN / 0x83 notify)\r\n");
    con_flush();
}

int main(void)
{
    uint64_t ticks_per_ms;
    uint64_t now;

    /* 1. Clocks (V3F startup jumps to main without calling SystemInit). */
    SystemInit();
    SystemAndCoreClockUpdate();
    v3f_time_init();
    ticks_per_ms = (uint64_t)v3f_ticks_per_us() * 1000u;

    /* 2. UART */
    lat_uart_init();
    {
        char buf[160];
        LatStr s;

        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "\r\n[v3f] CH32H417 latency tester app ");
        lat_str_u32(&s, LAT_FW_VERSION_MAJOR);
        lat_str_putc(&s, '.');
        lat_str_u32(&s, LAT_FW_VERSION_MINOR);
        lat_str_puts(&s, " core=");
        lat_str_u32(&s, SystemCoreClock);
        lat_str_puts(&s, " hclk=");
        lat_str_u32(&s, HCLKClock);
        out_line(&s);
        if (HCLKClock != 100000000u)
        {
            con_puts("[v3f] WARN hclk is not 100MHz (HSE start failure?)\r\n");
        }
        con_flush();
    }

    /* 3. TIM9 input clock (PPRE2) */
    ppre2_check_or_halt("after SystemInit");
    shm_address_check_or_halt();
    print_memory_map();

    /* 4. GPIO/TIM9 clocks and trigger outputs */
    gpio_and_timer_clock_init();
    con_puts("[v3f] gpio clocks on, tim9 clock on, PA0-PA7 open-drain released\r\n");

    /* 5. USBHS clocks */
    usbhs_clock_init();
    con_puts("[v3f] usbhs pll/utmi/bus clock ready\r\n");
    con_flush();

    /* Let clocks settle before touching PFIC. */
    v3f_delay_ms(10u);

    /* 6. IRQ routing */
    irq_routing_or_halt();

    /* 7. Re-verify PPRE2 */
    ppre2_check_or_halt("before v5f wake");

    /* 8. Shared SRAM init and V5F wake */
    s_v3f.epoch = ipc_init_shared();
    {
        char buf[96];
        LatStr s;

        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[v3f] ipc ready epoch=");
        lat_str_u32(&s, s_v3f.epoch);
        lat_str_puts(&s, " - waking v5f at 0x");
        lat_str_hex32(&s, LAT_V5F_FLASH_BASE);
        out_line(&s);
        con_flush();
    }
    /* Fence READY release store before NVIC_WakeUp_V5F (two MMIO writes). */
    LAT_FENCE_IORW();
    NVIC_WakeUp_V5F(LAT_V5F_FLASH_BASE);

    s_v3f.v5f_running = wait_v5f_running(V3F_V5F_START_TIMEOUT_MS);
    if (s_v3f.v5f_running)
    {
        con_puts("[v3f] v5f running\r\n");
    }
    else
    {
        char buf[128];
        LatStr s;

        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[v3f] ERROR diag=0x");
        lat_str_hex32(&s, V3F_DIAG_V5F_NOT_RUNNING);
        lat_str_puts(&s, " v5f not running: boot_state=");
        lat_str_u32(&s, lat_ipc_boot_state_load());
        lat_str_puts(&s, " v5f_diag=0x");
        lat_str_hex32(&s, lat_ipc_v5f_diag_load());
        out_line(&s);
    }
    con_flush();

    /* 9. PC link (USBFS CDC-ACM) */
    pc_link_init();
    usb_start();

    now = v3f_ticks64();
    s_v3f.next_ms = now + ticks_per_ms;
    s_v3f.next_period = now + ((uint64_t)V3F_PERIOD_MS * ticks_per_ms);
    s_v3f.last_v5f_heartbeat = lat_ipc_word_load(&g_ipc_header.v5f_heartbeat);

    for (;;)
    {
        now = v3f_ticks64();

        /* pc_link drains V5F events even if USB is down. */
        pc_link_poll((uint32_t)(now / ticks_per_ms));
        if (s_v3f.usb_started)
        {
            cdc_acm_poll();
        }
        con_poll();

        /* 1ms: V3F heartbeat */
        while (now >= s_v3f.next_ms)
        {
            s_v3f.heartbeat++;
            s_v3f.next_ms += ticks_per_ms;
        }
        lat_ipc_word_store(&g_ipc_header.v3f_heartbeat, s_v3f.heartbeat);

        /* 1s: status log and PING */
        if (now >= s_v3f.next_period)
        {
            s_v3f.next_period += (uint64_t)V3F_PERIOD_MS * ticks_per_ms;
            s_v3f.uptime_s++;

            if (!s_v3f.v5f_running &&
                (lat_ipc_boot_state_load() == LAT_BOOT_STATE_V5F_RUNNING))
            {
                s_v3f.v5f_running = true;
                con_puts("[v3f] v5f running (late start)\r\n");
            }

            print_status();
            print_pc_status();
            if (s_v3f.v5f_running)
            {
                send_ping();
            }
        }
    }
}
