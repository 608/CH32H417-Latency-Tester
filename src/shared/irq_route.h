/*
 * irq_route.h - Route PFIC external interrupts between V3F and V5F.
 *
 * SDK NVIC_SetAllocateIRQ() is a no-op for IRQn > 31, so this writes directly
 * to PFIC->IALLOCR[irq] (bit0: 0=V3F, 1=V5F) and reads back to verify.
 */
#ifndef LAT_IRQ_ROUTE_H
#define LAT_IRQ_ROUTE_H

#include <stdbool.h>
#include <stdint.h>

#include "ch32h417.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * irq_route_to_core: route external IRQ (> 31) to core (Core_ID_V3F / Core_ID_V5F)
 * and verify by readback. Returns false for core-internal IRQs (<= 31) or mismatch.
 */
bool irq_route_to_core(IRQn_Type irq, uint8_t core);

/* Return current core assignment (0=V3F, 1=V5F). */
uint32_t irq_route_read(IRQn_Type irq);

#ifdef __cplusplus
}
#endif

#endif /* LAT_IRQ_ROUTE_H */
