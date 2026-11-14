/*
 * usbhs_host.c - USBHS host engine hardware layer
 *
 * Hardware quirks:
 *   - Write only SOF_CNT_EN to FRAME (writing SOF_CNT_CLR leaves CLR stuck and
 *     suppresses SOF). CFG = SOF_EN | DMA_EN | PHY_SUSPENDM.
 *   - Interrupt flags are write-1-to-clear; write back the read value directly.
 *   - DMA buffers must reside in shared SRAM USBHS_DMA (g_usbhs_dma).
 *   - Detach clears CFG.SOF_EN; re-enable it after reset completes.
 *   - USBHS IRQ priority is 0x20 so TIM9 (0x00) can preempt it (IPRIOR[7:5]).
 */

#include "usbhs_host.h"
#include "usbhs_host_core.h"

#include "ch32h417.h"
#include "ch32h417_usb.h"
#include "lat_bytes.h"
#include "lat_fence.h"
#include "lat_fmt.h"
#include "lat_ipc.h"
#include "lat_pc_proto.h"
#include "tim9_time.h"
#include "v5f_cmd.h"

#define USBHS_IRQ_PRIORITY   ((uint8_t)0x20u)
#define HOST_LOG_MAX         (120u)
#define HOST_STATS_LOG_TICKS ((uint64_t)10000u * USBHS_TICKS_PER_MS)

/* Verify core constants match SDK definitions. */
_Static_assert(USBHS_PID_OUT == USB_PID_OUT, "PID OUT");
_Static_assert(USBHS_PID_IN == USB_PID_IN, "PID IN");
_Static_assert(USBHS_PID_SETUP == USB_PID_SETUP, "PID SETUP");
_Static_assert(USBHS_PID_ACK == USB_PID_ACK, "PID ACK");
_Static_assert(USBHS_PID_NYET == USB_PID_NYET, "PID NYET");
_Static_assert(USBHS_PID_NAK == USB_PID_NAK, "PID NAK");
_Static_assert(USBHS_PID_STALL == USB_PID_STALL, "PID STALL");
_Static_assert(USBHS_PID_DATA0 == USB_PID_DATA0, "PID DATA0");
_Static_assert(USBHS_PID_DATA1 == USB_PID_DATA1, "PID DATA1");
_Static_assert(USBHS_IF_SOF_ACT == USBHS_UHIF_SOF_ACT, "INT_FLAG SOF_ACT");
_Static_assert(USBHS_IF_TRANSFER == USBHS_UHIF_TRANSFER, "INT_FLAG TRANSFER");
_Static_assert(USBHS_PC_RESET == USBHS_UHIF_PORT_RESET, "PORT_STATUS_CHG RESET");
_Static_assert(USBHS_PC_CONNECT == USBHS_UHIF_PORT_CONNECT, "PORT_STATUS_CHG CONNECT");
_Static_assert(USBHS_PS_SPEED_MASK == USBHS_UHIS_PORT_SPEED_MASK, "PORT_STATUS speed");
_Static_assert(USBHS_PS_HS == USBHS_UHIS_PORT_HS, "PORT_STATUS HS");
_Static_assert(USBHS_PS_LS == USBHS_UHIS_PORT_LS, "PORT_STATUS LS");
_Static_assert(USBHS_PS_RST == USBHS_UHIS_PORT_RST, "PORT_STATUS RST");
_Static_assert(USBHS_PS_CONNECT == USBHS_UHIS_PORT_CONNECT, "PORT_STATUS CONNECT");
_Static_assert(USBHS_CFG_SOF_EN == USBHS_UH_SOF_EN, "CFG SOF_EN");
_Static_assert(USBHS_CTRL_HOST_ACTION == USBHS_UH_HOST_ACTION, "CONTROL HOST_ACTION");
_Static_assert(USBHS_PKT_MAX == LAT_USBHS_DMA_BUF_SIZE, "DMA buffer size");
_Static_assert((USBHS_IRQ_PRIORITY & 0xE0u) > (LAT_TIM9_IRQ_PRIORITY & 0xE0u),
               "TIM9 must be able to preempt USBHS (IPRIOR bit[7:5])");

static UsbhsLogFn s_log;
static uint64_t s_next_stats_log;

/* ------------------------------------------------------------------ */
/* Registers                                                            */
/* ------------------------------------------------------------------ */

static void clear_usb_flags(void)
{
    uint8_t f = USBHSH->INT_FLAG;
    uint8_t c = USBHSH->PORT_STATUS_CHG;

    if (f != 0u)
    {
        USBHSH->INT_FLAG = f;
    }
    if (c != 0u)
    {
        USBHSH->PORT_STATUS_CHG = c;
    }
}

/* Initialize USBHS host controller and verify via readback. */
static bool regs_init(void)
{
    uint8_t cfg;

    USBHSH->CFG = USBHS_RST_LINK | USBHS_UH_PHY_SUSPENDM;
    USBHSH->INT_EN = 0u;
    USBHSH->PORT_INT_EN = 0u;
    USBHSH->CONTROL = 0u;
    USBHSH->TX_DMA = (uint32_t)(uintptr_t)g_usbhs_dma.tx;
    USBHSH->RX_DMA = (uint32_t)(uintptr_t)g_usbhs_dma.rx;
    USBHSH->TX_LEN = 0u;
    USBHSH->RX_MAX_LEN = 64u;
    USBHSH->DEV_ADDR = 0u;
    USBHSH->PORT_CFG = USBHS_UH_PD_EN | USBHS_UH_HOST_EN;
    /* Do not write SOF_CNT_CLR; it remains stuck and suppresses SOF. */
    USBHSH->FRAME = USBHS_UH_SOF_CNT_EN;
    clear_usb_flags();
    USBHSH->CFG = USBHS_UH_SOF_EN | USBHS_UH_DMA_EN | USBHS_UH_PHY_SUSPENDM;

    cfg = USBHSH->CFG;
    /* Hardware may clear SOF_EN when no device is attached, so exclude it from check. */
    return ((cfg & (USBHS_RST_LINK | USBHS_UH_DMA_EN | USBHS_UH_PHY_SUSPENDM)) ==
            (USBHS_UH_DMA_EN | USBHS_UH_PHY_SUSPENDM)) &&
           ((USBHSH->PORT_CFG & USBHS_UH_HOST_EN) != 0u) &&
           ((USBHSH->FRAME & (USBHS_UH_SOF_CNT_EN | USBHS_UH_SOF_CNT_CLR)) == USBHS_UH_SOF_CNT_EN);
}

uint32_t usbhs_hw_cnt(void)
{
    return TIM9->CNT_32;
}

uint64_t usbhs_hw_now(void)
{
    return tim9_now();
}

uint32_t usbhs_hw_issue(uint8_t addr, uint8_t token, uint8_t ep, uint8_t toggle, uint16_t len, const uint8_t *tx)
{
    uint32_t control = USBHS_UH_HOST_ACTION | (uint32_t)token | ((uint32_t)(ep & 0x0Fu) << 4);
    uint32_t cnt;

    if (toggle != 0u)
    {
        control |= USBHS_UH_T_TOG_DATA1;
    }
    USBHSH->DEV_ADDR = (uint8_t)(addr & USBHS_UH_DEV_ADDR);
    if (token == USB_PID_IN)
    {
        /* RX_MAX_LEN = min(wMaxPacketSize, DMA buffer size). */
        USBHSH->RX_MAX_LEN = (len > LAT_USBHS_DMA_BUF_SIZE) ? LAT_USBHS_DMA_BUF_SIZE : len;
    }
    else
    {
        if (len > LAT_USBHS_DMA_BUF_SIZE)
        {
            len = (uint16_t)LAT_USBHS_DMA_BUF_SIZE;
        }
        if ((tx != (const uint8_t *)0) && (len > 0u))
        {
            lat_bytes_copy(g_usbhs_dma.tx, tx, len);
        }
        USBHSH->TX_LEN = len;
    }
    USBHSH->INT_FLAG = USBHS_UHIF_TRANSFER;
    /* Flush TX buffer writes before starting DMA. */
    LAT_FENCE_IORW();
    cnt = TIM9->CNT_32;
    USBHSH->CONTROL = control;
    return cnt;
}

const uint8_t *usbhs_hw_rx(void)
{
    LAT_FENCE_IORW();
    return g_usbhs_dma.rx;
}

uint8_t usbhs_hw_int_flag(void)
{
    return USBHSH->INT_FLAG;
}

uint32_t usbhs_hw_control(void)
{
    return USBHSH->CONTROL;
}

void usbhs_hw_clear_transfer(void)
{
    USBHSH->INT_FLAG = USBHS_UHIF_TRANSFER;
}

void usbhs_hw_abort(void)
{
    USBHSH->CONTROL &= ~USBHS_UH_HOST_ACTION;
}

void usbhs_hw_set_compare(uint32_t t)
{
    TIM9->CH2CVR_32 = t;
}

void usbhs_hw_port_reset(uint8_t long_reset)
{
    USBHSH->DEV_ADDR = 0u;
    if (long_reset != 0u)
    {
        USBHSH->PORT_CTRL |= USBHS_UH_BUS_RST_LONG;
    }
    else
    {
        USBHSH->PORT_CTRL &= ~USBHS_UH_BUS_RST_LONG;
    }
    USBHSH->PORT_CTRL |= USBHS_UH_SET_PORT_RESET;
}

void usbhs_hw_sof_enable(void)
{
    USBHSH->CFG |= USBHS_UH_SOF_EN;
}

uint16_t usbhs_hw_port_status(void)
{
    return USBHSH->PORT_STATUS;
}

void usbhs_hw_irq_off(void)
{
    NVIC_DisableIRQ(USBHS_IRQn);
    NVIC_DisableIRQ(TIM9_IRQn);
    LAT_FENCE_IORW();
    /* Disable TIM9 CH2 compare IRQ while TIM9 IRQ is masked. */
    TIM9->DMAINTENR &= (uint16_t)~TIM_CC2IE;
}

bool usbhs_hw_reinit(void)
{
    return regs_init();
}

void usbhs_hw_clear_flags(void)
{
    clear_usb_flags();
    TIM9->INTFR = (uint16_t)~TIM_CC2IF; /* Write-0 clears */
}

void usbhs_hw_enable_sources(void)
{
    USBHSH->INT_EN = USBHS_UHIE_TRANSFER | USBHS_UHIE_SOF_ACT;
    USBHSH->PORT_INT_EN = USBHS_UHIE_PORT_CONNECT | USBHS_UHIE_PORT_RESET;
    TIM9->DMAINTENR |= TIM_CC2IE; /* TIM9 IRQ is masked here */
}

void usbhs_hw_irq_on(bool usbhs)
{
    NVIC_EnableIRQ(TIM9_IRQn);
    if (usbhs)
    {
        NVIC_EnableIRQ(USBHS_IRQn);
    }
}

void usbhs_hw_lock(void)
{
    NVIC_DisableIRQ(USBHS_IRQn);
    LAT_FENCE_IORW();
}

void usbhs_hw_unlock(void)
{
    NVIC_EnableIRQ(USBHS_IRQn);
}

void usbhs_hw_kick(void)
{
    NVIC_SetPendingIRQ(USBHS_IRQn);
}

void usbhs_hw_notify(uint8_t pc_type, uint32_t word, uint8_t arg0, uint8_t arg1, uint16_t code)
{
    (void)v5f_cmd_isr_event(pc_type, word, arg0, arg1, code);
}

/* ------------------------------------------------------------------ */
/* Interrupts                                                           */
/* ------------------------------------------------------------------ */

/*
 * Capture hardware snapshot and clear only the observed flags before
 * handing off to usbhs_core_isr().
 */
void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBHS_IRQHandler(void)
{
    UsbhsSnap sn;
    uint8_t f = USBHSH->INT_FLAG;
    uint8_t pc = USBHSH->PORT_STATUS_CHG;

    sn.now = tim9_now();
    sn.int_st = USBHSH->INT_ST;
    sn.rx_len = (uint16_t)(USBHSH->RX_LEN & USBHS_UH_RX_LEN);
    sn.frame = USBHSH->FRAME;
    sn.port_status = USBHSH->PORT_STATUS;
    sn.cfg = (pc != 0u) ? USBHSH->CFG : 0u;
    if (f != 0u)
    {
        USBHSH->INT_FLAG = f;
    }
    if (pc != 0u)
    {
        USBHSH->PORT_STATUS_CHG = pc;
    }
    sn.int_flag = f;
    sn.port_chg = pc;
    usbhs_core_isr(&sn);
}

void usbhs_host_tim9_ch2(void)
{
    usbhs_core_tim9_ch2();
}

/* ------------------------------------------------------------------ */
/* Logging                                                              */
/* ------------------------------------------------------------------ */

static void logs(const char *line)
{
    if (s_log != (UsbhsLogFn)0)
    {
        s_log(line);
    }
}

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
    switch (sp)
    {
    case USBHS_SPEED_LS:
        return "LS";
    case USBHS_SPEED_FS:
        return "FS";
    case USBHS_SPEED_HS:
        return "HS";
    default:
        return "none";
    }
}

static void log_stats(uint64_t now)
{
    char buf[HOST_LOG_MAX + 1u];
    LatStr s;
    UsbhsPortInfo pi;
    UsbhsStats st;

    usbhs_port_info(&pi);
    usbhs_stats(&st);
    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[usbhs] st port=");
    lat_str_u32(&s, pi.state);
    lat_str_puts(&s, " sp=");
    lat_str_puts(&s, speed_name(pi.speed));
    lat_str_puts(&s, " gen=");
    lat_str_u32(&s, pi.generation);
    lat_str_puts(&s, " sof=");
    lat_str_u32(&s, pi.sof_count);
    lat_str_puts(&s, " iss=");
    lat_str_u32(&s, st.issue_tim9);
    lat_str_putc(&s, '/');
    lat_str_u32(&s, st.issue_usbhs);
    lat_str_puts(&s, " late=");
    lat_str_u32(&s, st.sched_late_count);
    lat_str_putc(&s, '/');
    lat_str_u32(&s, st.sched_late_max);
    lat_str_puts(&s, " txmax=");
    lat_str_u32(&s, st.txn_max_ticks);
    lat_str_puts(&s, " to=");
    lat_str_u32(&s, st.txn_timeout);
    lat_str_puts(&s, " rec=");
    lat_str_u32(&s, st.recover_count);
    lat_str_puts(&s, " spur=");
    lat_str_u32(&s, st.spurious_transfer);
    lat_str_puts(&s, " anch=");
    lat_str_u32(&s, st.sof_reanchor);
    (void)now;
    logs(buf);
}

/* ------------------------------------------------------------------ */
/* Init and main loop                                                   */
/* ------------------------------------------------------------------ */

void usbhs_host_init(UsbhsLogFn log)
{
    char buf[HOST_LOG_MAX + 1u];
    LatStr s;
    uint64_t now;
    uint16_t ps;
    bool ok;

    s_log = log;

    NVIC_DisableIRQ(USBHS_IRQn);
    NVIC_SetPriority(USBHS_IRQn, USBHS_IRQ_PRIORITY);
    ok = regs_init();
    now = tim9_now();
    usbhs_core_init(now);
    s_next_stats_log = now + HOST_STATS_LOG_TICKS;

    /* Configure TIM9 CH2 for scheduler compare (frozen mode, no pin output). */
    NVIC_DisableIRQ(TIM9_IRQn);
    TIM9->CH2CVR_32 = TIM9->CNT_32 - 1u;
    TIM9->INTFR = (uint16_t)~TIM_CC2IF;
    TIM9->DMAINTENR |= TIM_CC2IE;
    NVIC_EnableIRQ(TIM9_IRQn);

    clear_usb_flags();
    ps = USBHSH->PORT_STATUS;
    /* Handle any device already attached at boot before enabling IRQs. */
    usbhs_core_boot_port(ps, now);

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[usbhs] init ");
    lat_str_puts(&s, ok ? "ok" : "READBACK-FAIL");
    lat_str_puts(&s, " cfg=");
    put_hex(&s, USBHSH->CFG, 2u);
    lat_str_puts(&s, " ps=");
    put_hex(&s, ps, 4u);
    lat_str_puts(&s, " frame=");
    put_hex(&s, USBHSH->FRAME, 8u);
    lat_str_puts(&s, " irq_prio=");
    put_hex(&s, USBHS_IRQ_PRIORITY, 2u);
    lat_str_puts(&s, " dma=");
    put_hex(&s, (uint32_t)(uintptr_t)g_usbhs_dma.tx, 8u);
    logs(buf);

    if (!ok)
    {
        /* Init readback mismatch: attempt recovery (halts if fatal). */
        usbhs_recover(USBHS_CAUSE_OTHER);
        return;
    }
    USBHSH->INT_EN = USBHS_UHIE_TRANSFER | USBHS_UHIE_SOF_ACT;
    USBHSH->PORT_INT_EN = USBHS_UHIE_PORT_CONNECT | USBHS_UHIE_PORT_RESET;
    NVIC_ClearPendingIRQ(USBHS_IRQn);
    NVIC_EnableIRQ(USBHS_IRQn);
}

void usbhs_host_poll(uint64_t now)
{
    usbhs_core_poll(now);
    if ((int64_t)(now - s_next_stats_log) >= 0)
    {
        s_next_stats_log = now + HOST_STATS_LOG_TICKS;
        log_stats(now);
    }
}
