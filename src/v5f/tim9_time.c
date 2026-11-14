/*
 * tim9_time.c - V5F 64-bit timebase (TIM9)
 *
 * Read TIM9->CNT_32 directly; SDK TIM_GetCounter() only returns 16 bits.
 */
#include "tim9_time.h"

#include "ch32h417.h"
#include "measure/meas_port.h"
#include "usbhs_host.h"

/* 4-byte aligned volatile so the compiler does not merge the two epoch reads. */
static volatile uint32_t s_tim9_epoch __attribute__((aligned(4)));
static volatile uint32_t s_tim9_isr_count;
static volatile uint32_t s_tim9_update_count;
static volatile uint32_t s_tim9_last_isr_cnt;

#define LAT_TIM9_READ_EPOCH()      (s_tim9_epoch)
#define LAT_TIM9_READ_CNT()        (TIM9->CNT_32)
#define LAT_TIM9_READ_INTFR()      ((uint16_t)TIM9->INTFR)
#define LAT_TIM9_WRITE_INTFR(v)    (TIM9->INTFR = (uint16_t)(v))
#define LAT_TIM9_EPOCH_INCREMENT() (s_tim9_epoch = s_tim9_epoch + 1u)

#include "tim9_time_core.h"

void tim9_time_init(void)
{
    NVIC_DisableIRQ(TIM9_IRQn);

    TIM9->CTLR1 = 0u;
    TIM9->DMAINTENR = 0u;
    TIM9->PSC = 0u;
    TIM9->ATRLR_32 = 0xFFFFFFFFu;
    TIM9->CNT_32 = 0u;
    /* Apply PSC/ARR via UG, then clear the resulting UIF. */
    TIM9->SWEVGR = TIM_UG;
    TIM9->INTFR = 0u;

    s_tim9_epoch = 0u;
    s_tim9_isr_count = 0u;
    s_tim9_update_count = 0u;

    /* Enable update + CH1 (trigger); CH2 is enabled by usbhs_host_init(). */
    TIM9->CH1CVR_32 = 0u;
    TIM9->DMAINTENR = (uint16_t)(TIM_UIE | TIM_CC1IE);

    NVIC_SetPriority(TIM9_IRQn, LAT_TIM9_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(TIM9_IRQn);
    NVIC_EnableIRQ(TIM9_IRQn);

    TIM9->CTLR1 = TIM_CEN;
}

uint64_t tim9_now(void)
{
    return lat_tim9_now_core();
}

uint32_t tim9_epoch_value(void)
{
    return s_tim9_epoch;
}

uint32_t tim9_isr_count(void)
{
    return s_tim9_isr_count;
}

uint32_t tim9_update_count(void)
{
    return s_tim9_update_count;
}

void TIM9_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM9_IRQHandler(void)
{
    uint32_t cnt;
    uint16_t snapshot = lat_tim9_isr_begin(&cnt);

    s_tim9_last_isr_cnt = cnt;
    s_tim9_isr_count = s_tim9_isr_count + 1u;

    /* Order: CH1 (trigger), CH2 (schedule), then update (epoch). */
    if ((snapshot & TIM_CC1IF) != 0u)
    {
        meas_tim9_ch1(lat_tim9_compose(s_tim9_epoch, cnt, snapshot));
    }
    if ((snapshot & TIM_CC2IF) != 0u)
    {
        usbhs_host_tim9_ch2();
    }

    if ((snapshot & LAT_TIM9_UIF) != 0u)
    {
        s_tim9_update_count = s_tim9_update_count + 1u;
    }

    lat_tim9_isr_end(snapshot);
}

