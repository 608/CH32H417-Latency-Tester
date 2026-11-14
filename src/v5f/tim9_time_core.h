/*
 * tim9_time_core.h - 64-bit TIM9 extension core
 *
 * Define these macros before including:
 *   LAT_TIM9_READ_EPOCH()      Read upper 32-bit epoch (uint32_t)
 *   LAT_TIM9_READ_CNT()        Read TIM9->CNT_32 (uint32_t)
 *   LAT_TIM9_READ_INTFR()      Read TIM9->INTFR (uint16_t)
 *   LAT_TIM9_WRITE_INTFR(v)    Write TIM9->INTFR (write-0 clears bits)
 *   LAT_TIM9_EPOCH_INCREMENT() Increment epoch by 1
 *
 * Rollover handling:
 *   lat_tim9_now_core() retries if epoch changes around CNT_32/INTFR reads.
 *   If UIF is set and CNT_32 < 0x80000000, a rollover occurred before the ISR
 *   ran, so epoch + 1 is used. Assumes < 0x80000000 ticks (~21.47s) between
 *   CNT_32 and INTFR reads and for ISR latency.
 *   In the ISR, UIF must not be cleared before incrementing epoch, and
 *   lat_tim9_now_core() must not run between incrementing epoch and clearing UIF.
 */
#ifndef LAT_TIM9_TIME_CORE_H
#define LAT_TIM9_TIME_CORE_H

#include <stdint.h>

#if !defined(LAT_TIM9_READ_EPOCH) || !defined(LAT_TIM9_READ_CNT) || \
    !defined(LAT_TIM9_READ_INTFR) || !defined(LAT_TIM9_WRITE_INTFR) || \
    !defined(LAT_TIM9_EPOCH_INCREMENT)
#error "define the LAT_TIM9_* access macros before including tim9_time_core.h"
#endif

#define LAT_TIM9_UIF       ((uint16_t)0x0001u) /* TIM_UIF */
#define LAT_TIM9_HALF_SPAN ((uint32_t)0x80000000u)

static inline uint64_t lat_tim9_compose(uint32_t epoch, uint32_t cnt, uint16_t intfr)
{
    uint32_t high = epoch;

    if (((intfr & LAT_TIM9_UIF) != 0u) && (cnt < LAT_TIM9_HALF_SPAN))
    {
        high += 1u;
    }
    return ((uint64_t)high << 32) | (uint64_t)cnt;
}

/* Current 64-bit time in 10ns ticks; safe from any context. */
static inline uint64_t lat_tim9_now_core(void)
{
    for (;;)
    {
        uint32_t epoch_before = LAT_TIM9_READ_EPOCH();
        uint32_t cnt = LAT_TIM9_READ_CNT();
        uint16_t intfr = LAT_TIM9_READ_INTFR();
        uint32_t epoch_after = LAT_TIM9_READ_EPOCH();

        if (epoch_before == epoch_after)
        {
            return lat_tim9_compose(epoch_before, cnt, intfr);
        }
    }
}

static inline uint16_t lat_tim9_isr_begin(uint32_t *cnt_out)
{
    *cnt_out = LAT_TIM9_READ_CNT();
    return LAT_TIM9_READ_INTFR();
}

static inline void lat_tim9_isr_end(uint16_t snapshot)
{
    if ((snapshot & LAT_TIM9_UIF) != 0u)
    {
        LAT_TIM9_EPOCH_INCREMENT();
    }
    LAT_TIM9_WRITE_INTFR((uint16_t)~snapshot);
}

#endif /* LAT_TIM9_TIME_CORE_H */
