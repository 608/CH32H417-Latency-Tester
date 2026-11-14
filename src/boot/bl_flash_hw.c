/*
 * bl_flash_hw.c - CH32H417 hardware flash HAL.
 *
 * - Uses 256-byte fast page programming only (word program fails on H417).
 * - Uses sector erase (PER); checks FLASH_CFGR0 bit28 for 8 KB vs 4 KB unit.
 * - Erase/program use the 0x08000000 alias; reads use the execution address
 *   (erased words read as 0xE339E339).
 * - All bootloader code runs from RAM_CODE, so instruction fetch is unaffected.
 */
#include "bl_flash_hal.h"

#include "ch32h417.h"

#include "bl_time.h"
#include "lat_fence.h"

#define FLASH_ALIAS_BASE   ((uint32_t)0x08000000u)
#define FLASH_KEY1_VALUE   ((uint32_t)0x45670123u)
#define FLASH_KEY2_VALUE   ((uint32_t)0xCDEF89ABu)

/* FLASH->CTLR */
#define CR_PER             ((uint32_t)0x00000002u)
#define CR_STRT            ((uint32_t)0x00000040u)
#define CR_LOCK            ((uint32_t)0x00000080u)
#define CR_FLOCK           ((uint32_t)0x00008000u)
#define CR_PAGE_PG         ((uint32_t)0x00010000u)
#define CR_PG_STRT         ((uint32_t)0x00200000u)
/* FLASH->STATR */
#define SR_BSY             ((uint32_t)0x00000001u)
#define SR_WR_BSY          ((uint32_t)0x00000002u)
#define SR_WRPRTERR        ((uint32_t)0x00000010u)
#define SR_EOP             ((uint32_t)0x00000020u)
/* FLASH_CFGR0 bit28: 1 indicates 8 KB erase unit */
#define CFGR0_8K_ERASE     ((uint32_t)(1u << 28))

/* Bounded wait timeouts (us). */
#define WAIT_IDLE_US       (20000u)
#define WAIT_ERASE_US      (500000u)
#define WAIT_WORD_US       (2000u)
#define WAIT_PROGRAM_US    (50000u)

static bool s_unlocked;

static bool wait_clear(uint32_t mask, uint32_t timeout_us)
{
    uint32_t start = bl_ticks();

    for (;;)
    {
        LAT_FENCE_IORW();
        if ((FLASH->STATR & mask) == 0u)
        {
            return true;
        }
        if (bl_elapsed_us(start) >= timeout_us)
        {
            return false;
        }
    }
}

static void unlock(void)
{
    if (!s_unlocked)
    {
        FLASH->STATR = SR_EOP | SR_WRPRTERR;
        FLASH->KEYR = FLASH_KEY1_VALUE;
        FLASH->KEYR = FLASH_KEY2_VALUE;
        FLASH->MODEKEYR = FLASH_KEY1_VALUE;
        FLASH->MODEKEYR = FLASH_KEY2_VALUE;
        s_unlocked = true;
    }
}

void bl_hal_lock(void)
{
    if (s_unlocked)
    {
        /* Wait for any in-flight operation (bounded) before locking. */
        (void)wait_clear(SR_BSY | SR_WR_BSY, WAIT_ERASE_US);
        FLASH->CTLR |= CR_FLOCK;
        FLASH->CTLR |= CR_LOCK;
        s_unlocked = false;
    }
}

BlStatus bl_hal_erase_sector(uint32_t exec_addr)
{
    uint32_t unit = (((*(volatile uint32_t *)FLASH_CFGR0_BASE) & CFGR0_8K_ERASE) != 0u) ? 8192u : 4096u;
    uint32_t addr;

    if ((exec_addr % BL_FLASH_SECTOR_SIZE) != 0u)
    {
        return BL_STATUS_ALIGN;
    }
    unlock();
    for (addr = exec_addr; addr < (exec_addr + BL_FLASH_SECTOR_SIZE); addr += unit)
    {
        bool ok;

        if (!wait_clear(SR_BSY, WAIT_IDLE_US))
        {
            return BL_STATUS_TIMEOUT;
        }
        FLASH->STATR = SR_EOP | SR_WRPRTERR;
        FLASH->CTLR |= CR_PER;
        FLASH->ADDR = FLASH_ALIAS_BASE + addr;
        FLASH->CTLR |= CR_STRT;
        ok = wait_clear(SR_BSY, WAIT_ERASE_US);
        FLASH->CTLR &= ~CR_PER;
        if (!ok)
        {
            return BL_STATUS_TIMEOUT;
        }
        if ((FLASH->STATR & SR_WRPRTERR) != 0u)
        {
            FLASH->STATR = SR_WRPRTERR;
            return BL_STATUS_FLASH_FAIL;
        }
        FLASH->STATR = SR_EOP;
    }
    LAT_FENCE_IORW();
    return BL_STATUS_OK;
}

BlStatus bl_hal_program_page(uint32_t exec_addr, const uint8_t page[BL_FLASH_PAGE_SIZE])
{
    volatile uint32_t *dst = (volatile uint32_t *)(uintptr_t)(FLASH_ALIAS_BASE + exec_addr);
    uint32_t i;
    bool ok;

    if ((exec_addr % BL_FLASH_PAGE_SIZE) != 0u)
    {
        return BL_STATUS_ALIGN;
    }
    unlock();
    if (!wait_clear(SR_BSY, WAIT_IDLE_US))
    {
        return BL_STATUS_TIMEOUT;
    }
    FLASH->STATR = SR_EOP | SR_WRPRTERR;
    FLASH->CTLR |= CR_PAGE_PG;
    if (!wait_clear(SR_BSY | SR_WR_BSY, WAIT_IDLE_US))
    {
        FLASH->CTLR &= ~CR_PAGE_PG;
        return BL_STATUS_TIMEOUT;
    }
    for (i = 0u; i < (BL_FLASH_PAGE_SIZE / 4u); i++)
    {
        const uint8_t *w = &page[i * 4u];

        dst[i] = (uint32_t)w[0] | ((uint32_t)w[1] << 8) | ((uint32_t)w[2] << 16) | ((uint32_t)w[3] << 24);
        if (!wait_clear(SR_WR_BSY, WAIT_WORD_US))
        {
            FLASH->CTLR &= ~CR_PAGE_PG;
            return BL_STATUS_TIMEOUT;
        }
    }
    FLASH->CTLR |= CR_PG_STRT;
    ok = wait_clear(SR_BSY, WAIT_PROGRAM_US);
    FLASH->CTLR &= ~CR_PAGE_PG;
    if (!ok)
    {
        return BL_STATUS_TIMEOUT;
    }
    if ((FLASH->STATR & SR_WRPRTERR) != 0u)
    {
        FLASH->STATR = SR_WRPRTERR;
        return BL_STATUS_FLASH_FAIL;
    }
    FLASH->STATR = SR_EOP;
    LAT_FENCE_IORW();
    return BL_STATUS_OK;
}

const uint8_t *bl_hal_read_ptr(uint32_t exec_addr)
{
    return (const uint8_t *)(uintptr_t)exec_addr;
}
