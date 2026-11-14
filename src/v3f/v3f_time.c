/*
 * v3f_time.c - V3F SysTick0 timebase (free-running 32-bit up-counter, no IRQ).
 */
#include "v3f_time.h"

#include "ch32h417.h"

static uint32_t s_ticks_per_us = 100u;
static uint32_t s_last_tick;
static uint64_t s_ticks64;

void v3f_time_init(void)
{
    s_ticks_per_us = HCLKClock / 1000000u;
    if (s_ticks_per_us == 0u)
    {
        s_ticks_per_us = 1u;
    }

    SysTick0->CTLR = 0u;
    SysTick0->CNT = 0u;
    SysTick0->CMP = 0xFFFFFFFFu;
    SysTick0->CTLR = (1u << 2) | (1u << 0);

    s_last_tick = 0u;
    s_ticks64 = 0u;
}

uint32_t v3f_ticks(void)
{
    return SysTick0->CNT;
}

uint32_t v3f_ticks_per_us(void)
{
    return s_ticks_per_us;
}

uint64_t v3f_ticks64(void)
{
    uint32_t now = SysTick0->CNT;

    s_ticks64 += (uint64_t)(uint32_t)(now - s_last_tick);
    s_last_tick = now;
    return s_ticks64;
}

void v3f_delay_ms(uint32_t ms)
{
    const uint32_t ticks_per_ms = s_ticks_per_us * 1000u;

    while (ms-- > 0u)
    {
        uint32_t start = SysTick0->CNT;

        while ((uint32_t)(SysTick0->CNT - start) < ticks_per_ms)
        {
        }
    }
}
