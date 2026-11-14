/*
 * meas_port_v5f.c - V5F hardware and enumeration port for the measurement engine
 *
 * Trigger (TIM9_CH1 compare ISR):
 *   Single-shot: main loop writes CH1CVR_32 then sets armed=1 (and triggers SWEVGR CC1G
 *   if already past). ISR fires only if time has passed and atomic exchange of armed succeeds.
 *   T0..T7 (PA0..PA7) are open-drain: BCR asserts Low, BSHR releases.
 * Calibration mode:
 *   PA2 = TIM9_CH3 (AF4) compare output (push-pull), PA7 = push-pull marker.
 *   PA0/PA1 are left untouched (wired to button/stick).
 */
#include "meas_port.h"

#include "ch32h417.h"
#include "lat_bytes.h"
#include "lat_fence.h"
#include "meas_poll.h"
#include "measure.h"
#include "tim9_time.h"

#include "enum/enum_api.h"
#include "usbhs_host.h"

#define PIN_MASK32        ((uint32_t)0x00FFu)
#define PA2_BIT           ((uint32_t)0x0004u)
#define PA7_BIT           ((uint32_t)0x0080u)
/* 4-bit CFGLR encoding (matches GPIO_Init in ch32h417_gpio.c) */
#define CFG_OUT_OD        ((uint32_t)0x5u)
#define CFG_OUT_PP        ((uint32_t)0x1u)
#define CFG_AF_PP         ((uint32_t)0x9u)
#define AF_TIM9_CH3_PA2   ((uint32_t)4u)    /* PA2 = TIM9_CH3(AF4) */
/* OC3M in CHCTLR2 (bit[6:4]) */
#define OC3M_MASK         ((uint16_t)0x0070u)
#define OC3M_ACTIVE       ((uint16_t)0x0010u) /* Active on match */
#define OC3M_FORCE_LOW    ((uint16_t)0x0040u) /* Force inactive */

#define LISTEN_MAX        (4u)
#define POOL_WORDS        (24576u)          /* 96KB: histogram + raw data */

/* ---- Polling measurement buffers ---- */
static uint32_t s_pool[POOL_WORDS];
__attribute__((section(".shm_spare"), aligned(64), used))
static uint8_t s_longs[MEAS_POLL_LONG_MAX * MEAS_POLL_LONG_ENTRY];

uint32_t *meas_port_poll_pool(uint32_t *words)
{
    *words = POOL_WORDS;
    return s_pool;
}

uint8_t *meas_port_poll_longs(void)
{
    return s_longs;
}

uint64_t meas_port_now(void)
{
    return tim9_now();
}

void meas_port_pins(uint8_t release_mask, uint8_t assert_mask)
{
    if (release_mask != 0u)
    {
        GPIOA->BSHR = (uint32_t)release_mask & PIN_MASK32;
    }
    if (assert_mask != 0u)
    {
        GPIOA->BCR = (uint32_t)assert_mask & PIN_MASK32;
    }
}

/* ------------------------------------------------------------------ */
/* Trigger (TIM9_CH1)                                                 */
/* ------------------------------------------------------------------ */
static volatile uint32_t s_trig_armed;
static volatile uint32_t s_trig_fired;
static uint32_t s_trig_lo;
static uint64_t s_trig_when;
static uint8_t s_trig_action;
static uint8_t s_trig_mask;
static MeasTrigResult s_trig_res;

void meas_port_trig_arm(uint64_t when, uint8_t action, uint8_t mask)
{
    (void)__atomic_exchange_n(&s_trig_armed, 0u, __ATOMIC_ACQ_REL);
    s_trig_fired = 0u;
    s_trig_when = when;
    s_trig_lo = (uint32_t)when;
    s_trig_action = action;
    s_trig_mask = mask;
    TIM9->CH1CVR_32 = (uint32_t)when;
    LAT_FENCE_IORW();
    __atomic_store_n(&s_trig_armed, 1u, __ATOMIC_RELEASE);
    /* If already past the compare value, trigger CC1 event in software. */
    if ((int32_t)(TIM9->CNT_32 - (uint32_t)when) >= 0)
    {
        TIM9->SWEVGR = TIM_CC1G;
    }
}

bool meas_port_trig_cancel(void)
{
    return __atomic_exchange_n(&s_trig_armed, 0u, __ATOMIC_ACQ_REL) != 0u;
}

bool meas_port_trig_fired(MeasTrigResult *out)
{
    if (s_trig_fired == 0u)
    {
        return false;
    }
    LAT_FENCE_RW();
    *out = s_trig_res;
    s_trig_fired = 0u;
    return true;
}

/* Called from TIM9 ISR on CH1 compare match. */
void meas_tim9_ch1(uint64_t t_isr)
{
    uint64_t tick;
    uint64_t tm = 0u;

    if ((s_trig_armed == 0u) || ((int32_t)((uint32_t)t_isr - s_trig_lo) < 0))
    {
        return;
    }
    if (__atomic_exchange_n(&s_trig_armed, 0u, __ATOMIC_ACQ_REL) == 0u)
    {
        return;
    }
    tick = tim9_now();
    if ((s_trig_action & MEAS_TRIG_ASSERT) != 0u)
    {
        GPIOA->BCR = (uint32_t)s_trig_mask & PIN_MASK32;
    }
    else
    {
        GPIOA->BSHR = (uint32_t)s_trig_mask & PIN_MASK32;
    }
    if ((s_trig_action & MEAS_TRIG_MARKER) != 0u)
    {
        tm = tim9_now();
        GPIOA->BSHR = PA7_BIT;
    }
    s_trig_res.sched = s_trig_when;
    s_trig_res.t_isr = t_isr;
    s_trig_res.t_tick = tick;
    s_trig_res.t_m = tm;
    LAT_FENCE_RW();
    s_trig_fired = 1u;
}

/* ------------------------------------------------------------------ */
/* Random                                                             */
/* ------------------------------------------------------------------ */

uint32_t meas_port_hw_random(void)
{
    static uint32_t fallback;
    uint32_t i;

    /* RNG clock (RCC_HBPeriph_RNG) is enabled by V3F. */
    if ((RNG->CR & RNG_CR_RNGEN) == 0u)
    {
        RNG->CR |= RNG_CR_RNGEN;
    }
    for (i = 0u; i < 2000u; i++)
    {
        uint32_t sr = RNG->SR;

        if ((sr & (RNG_SR_CECS | RNG_SR_SECS)) != 0u)
        {
            break; /* Clock or seed error */
        }
        if ((sr & RNG_SR_DRDY) != 0u)
        {
            return RNG->DR;
        }
    }
    /* Fallback xorshift32 seeded from timer if RNG is unavailable. */
    if (fallback == 0u)
    {
        fallback = TIM9->CNT_32 | 1u;
    }
    fallback ^= TIM9->CNT_32;
    fallback ^= fallback << 13;
    fallback ^= fallback >> 17;
    fallback ^= fallback << 5;
    return fallback;
}

bool meas_port_frame_at(uint64_t t, uint16_t *frame, uint8_t *uframe)
{
    return usbhs_frame_at(t, frame, uframe);
}

/* ------------------------------------------------------------------ */
/* Device (enum_api.h)                                                */
/* ------------------------------------------------------------------ */

typedef struct Listen
{
    uint8_t iface;
    uint8_t ep;
    uint8_t pipe;
    uint8_t refs;
} Listen;

static Listen s_listen[LISTEN_MAX];

bool meas_port_dev_find(uint8_t *iface, uint8_t *ep_in, MeasDev *out)
{
    const EnumDevice *d = enum_device();
    const EnumIface *f;

    if ((d == (const EnumDevice *)0) || (d->ready == 0u))
    {
        return false;
    }
    if (*iface == 0xFFu)
    {
        if (d->target_iface == 0xFFu)
        {
            return false;
        }
        *iface = d->target_iface;
        *ep_in = d->target_ep_in;
    }
    f = enum_find_iface(*iface, *ep_in);
    if ((f == (const EnumIface *)0) || (f->ep_in == 0u) || (f->pipe_in == 0u) || (f->in_period == 0u))
    {
        return false;
    }
    *ep_in = f->ep_in;
    lat_bytes_fill((uint8_t *)out, 0u, (uint32_t)sizeof(*out));
    out->generation = d->generation;
    out->seq = d->enum_seq;
    out->period_ticks = f->in_period * d->frame_ticks;
    out->mps = f->in_mps;
    out->speed = d->speed;
    return true;
}

bool meas_port_dev_alive(const MeasDev *dev)
{
    const EnumDevice *d = enum_device();

    return (d != (const EnumDevice *)0) && (d->ready != 0u) && (d->generation == dev->generation) &&
           (d->enum_seq == dev->seq);
}

/* USBHS ISR hook; dispatches by pipe number. */
static void port_hook(const UsbhsIntrDone *d, const uint8_t *data)
{
    uint32_t i;
    MeasCompletion c;

    for (i = 0u; i < LISTEN_MAX; i++)
    {
        const Listen *l = &s_listen[i];

        if ((l->refs == 0u) || (l->pipe != d->pipe))
        {
            continue;
        }
        c.t1 = d->t1;
        c.generation = d->generation;
        c.seq = d->seq;
        c.len = d->len;
        c.frame = d->frame;
        c.uframe = d->uframe;
        if (d->result == USBHS_RES_OK)
        {
            c.result = MEAS_RES_DATA;
        }
        else if (d->result == USBHS_RES_NAK)
        {
            c.result = MEAS_RES_NAK;
        }
        else
        {
            c.result = MEAS_RES_ERROR;
        }
        c.reserved[0] = 0u;
        c.reserved[1] = 0u;
        measure_isr_completion(l->iface, l->ep, &c, data);
        return;
    }
}

bool meas_port_listen(uint8_t iface, uint8_t ep_in, bool on)
{
    uint32_t i;
    Listen *l = (Listen *)0;
    Listen *free_slot = (Listen *)0;

    for (i = 0u; i < LISTEN_MAX; i++)
    {
        if (s_listen[i].refs == 0u)
        {
            if (free_slot == (Listen *)0)
            {
                free_slot = &s_listen[i];
            }
        }
        else if ((s_listen[i].iface == iface) && (s_listen[i].ep == ep_in))
        {
            l = &s_listen[i];
        }
    }

    if (!on)
    {
        if (l == (Listen *)0)
        {
            return false;
        }
        l->refs--;
        if (l->refs == 0u)
        {
            (void)enum_set_isr_hook(iface, ep_in, (EnumIsrHook)0);
        }
        return true;
    }

    {
        const EnumIface *f = enum_find_iface(iface, ep_in);
        uint8_t pipe;

        if ((f == (const EnumIface *)0) || (f->pipe_in == 0u))
        {
            return false;
        }
        pipe = f->pipe_in;
        if (l == (Listen *)0)
        {
            if (free_slot == (Listen *)0)
            {
                return false;
            }
            l = free_slot;
            l->iface = iface;
            l->ep = ep_in;
        }
        /* Store pipe before the hook can be invoked. */
        l->pipe = pipe;
        LAT_FENCE_RW();
        l->refs++;
        if (!enum_set_isr_hook(iface, ep_in, port_hook))
        {
            l->refs--;
            return false;
        }
        return true;
    }
}

/* ------------------------------------------------------------------ */
/* Calibration mode                                                   */
/* ------------------------------------------------------------------ */

static void cfg_pin(uint32_t pin, uint32_t nibble)
{
    uint32_t shift = pin * 4u;

    GPIOA->CFGLR = (GPIOA->CFGLR & ~((uint32_t)0xFu << shift)) | (nibble << shift);
}

static void af_pin(uint32_t pin, uint32_t af)
{
    uint32_t shift = pin * 4u;

    AFIO->GPIOA_AFLR = (AFIO->GPIOA_AFLR & ~((uint32_t)0xFu << shift)) | (af << shift);
}

void meas_port_calib_pins(bool on)
{
    if (on)
    {
        /* Force CH3 Low before connecting PA2 to TIM9_CH3. */
        TIM9->CHCTLR2 = (uint16_t)((TIM9->CHCTLR2 & (uint16_t)~(OC3M_MASK | TIM_CC3S | TIM_OC3PE)) | OC3M_FORCE_LOW);
        TIM9->CCER = (uint16_t)((TIM9->CCER & (uint16_t)~TIM_CC3P) | TIM_CC3E);
        TIM9->BDTR |= TIM_MOE;
        af_pin(2u, AF_TIM9_CH3_PA2);
        cfg_pin(2u, CFG_AF_PP);
        GPIOA->BCR = PA7_BIT;
        cfg_pin(7u, CFG_OUT_PP);
    }
    else
    {
        GPIOA->BSHR = PA2_BIT | PA7_BIT;
        cfg_pin(2u, CFG_OUT_OD);
        cfg_pin(7u, CFG_OUT_OD);
        af_pin(2u, 0u);
        TIM9->CCER &= (uint16_t)~TIM_CC3E;
        TIM9->CHCTLR2 = (uint16_t)(TIM9->CHCTLR2 & (uint16_t)~OC3M_MASK);
        TIM9->BDTR &= (uint16_t)~TIM_MOE;
        GPIOA->BSHR = PA2_BIT | PA7_BIT;
    }
}

void meas_port_calib_ref_arm(uint64_t when)
{
    TIM9->CH3CVR_32 = (uint32_t)when;
    TIM9->CHCTLR2 = (uint16_t)((TIM9->CHCTLR2 & (uint16_t)~OC3M_MASK) | OC3M_ACTIVE);
}

void meas_port_calib_ref_low(void)
{
    TIM9->CHCTLR2 = (uint16_t)((TIM9->CHCTLR2 & (uint16_t)~OC3M_MASK) | OC3M_FORCE_LOW);
}

void meas_port_marker(bool high)
{
    if (high)
    {
        GPIOA->BSHR = PA7_BIT;
    }
    else
    {
        GPIOA->BCR = PA7_BIT;
    }
}

uint64_t meas_port_marker_high_now(void)
{
    uint64_t t = tim9_now();

    GPIOA->BSHR = PA7_BIT;
    return t;
}

/*
 * Single-shot IN for c1. enum_api.h has no pause-polling API, so polling continues
 * and measure.c (cal2_armed) raises PA7 on the first IN data completion after issue.
 */
bool meas_port_calib_in_begin(uint8_t iface, uint8_t ep_in)
{
    const EnumIface *f = enum_find_iface(iface, ep_in);

    return (f != (const EnumIface *)0) && (f->pipe_in != 0u);
}

bool meas_port_calib_in_issue(void)
{
    return true;
}

void meas_port_calib_in_end(void)
{
}
