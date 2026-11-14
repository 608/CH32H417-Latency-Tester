/*
 * bl_time.c - Bootloader timing (see bl_time.h).
 *
 * SysTick0->CTLR: bit0=enable, bit2=HCLK source. Free-runs 32-bit with IRQ disabled.
 */
#include "bl_time.h"

#include "ch32h417.h"

static uint32_t s_ticks_per_us = 100u;

void bl_time_init(void)
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
}

uint32_t bl_ticks(void)
{
    return SysTick0->CNT;
}

uint32_t bl_ticks_per_us(void)
{
    return s_ticks_per_us;
}

uint32_t bl_elapsed_us(uint32_t start)
{
    return (uint32_t)(SysTick0->CNT - start) / s_ticks_per_us;
}

void bl_delay_ms(uint32_t ms)
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

void bl_time_stop(void)
{
    SysTick0->CTLR = 0u;
}
