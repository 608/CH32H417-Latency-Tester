/*
 * bl_time.h - Bootloader timing via free-running SysTick0 at HCLK.
 *
 * SDK Delay_Us/Delay_Ms reconfigure SysTick0, so the bootloader uses this
 * module instead (including bounded flash wait timeouts).
 */
#ifndef LAT_BL_TIME_H
#define LAT_BL_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bl_time_init(void);

/* Current SysTick0 HCLK count (wraps every ~42.9 s). */
uint32_t bl_ticks(void);

uint32_t bl_ticks_per_us(void);

/* Elapsed microseconds since start (valid for intervals < 42 s). */
uint32_t bl_elapsed_us(uint32_t start);

void bl_delay_ms(uint32_t ms);

/* Stop SysTick0 before jumping to the application so its reset state matches. */
void bl_time_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_BL_TIME_H */
