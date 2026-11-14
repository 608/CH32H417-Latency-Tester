/*
 * v3f_time.h - V3F free-running SysTick0 timebase (HCLK, 100MHz, wraps ~42.9s).
 * V3F runs in User mode, so M-mode CSRs (mcycle) cannot be read;
 * SDK Delay_Us/Delay_Ms must not be used as they overwrite SysTick0.
 */
#ifndef LAT_V3F_TIME_H
#define LAT_V3F_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void v3f_time_init(void);

/* Current SysTick0 count (HCLK ticks). */
uint32_t v3f_ticks(void);

/* Ticks per microsecond (100 at 100MHz). */
uint32_t v3f_ticks_per_us(void);

/* 64-bit tick count since boot; must be called at least once every 42s. */
uint64_t v3f_ticks64(void);

/* Busy-wait delay. */
void v3f_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* LAT_V3F_TIME_H */
