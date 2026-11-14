/*
 * tim9_time.h - V5F 64-bit timebase (TIM9, 100MHz, 10ns resolution)
 *
 * TIM9 free-runs at 100MHz (PSC=0, ARR=0xFFFFFFFF, ~42.9s period); update
 * interrupts increment the upper 32-bit epoch. V3F enables TIM9 RCC and
 * routes TIM9_IRQn to V5F; V5F must not touch RCC.
 */
#ifndef LAT_TIM9_TIME_H
#define LAT_TIM9_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Highest priority on V5F (USBHS must be lower). */
#define LAT_TIM9_IRQ_PRIORITY ((uint8_t)0x00u)

void tim9_time_init(void);

/* Current 64-bit time in 10ns ticks. */
uint64_t tim9_now(void);

uint32_t tim9_epoch_value(void);
uint32_t tim9_isr_count(void);
uint32_t tim9_update_count(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_TIM9_TIME_H */
