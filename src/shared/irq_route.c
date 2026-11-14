/*
 * irq_route.c - Direct PFIC->IALLOCR interrupt routing (see irq_route.h).
 */
#include "irq_route.h"

#include "lat_fence.h"

bool irq_route_to_core(IRQn_Type irq, uint8_t core)
{
    if ((uint32_t)irq <= 31u)
    {
        return false;
    }

    PFIC->IALLOCR[(uint32_t)irq] = core;

    /* Ensure PFIC write is visible to both cores before readback. */
    LAT_FENCE_IORW();

    return irq_route_read(irq) == (uint32_t)core;
}

uint32_t irq_route_read(IRQn_Type irq)
{
    return ((PFIC->IALLOCR[(uint32_t)irq] & 0x01u) != 0u) ? 1u : 0u;
}
