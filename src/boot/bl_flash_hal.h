/*
 * bl_flash_hal.h - Flash HAL used by bl_flash.c.
 *
 * Addresses are execution addresses (>= 0x10000); the hardware implementation
 * translates them to the 0x08000000 alias for erase and program.
 */
#ifndef LAT_BL_FLASH_HAL_H
#define LAT_BL_FLASH_HAL_H

#include <stdint.h>

#include "bl_flash.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Erase an 8 KB sector (BL_FLASH_SECTOR_SIZE aligned). */
BlStatus bl_hal_erase_sector(uint32_t exec_addr);

/* Program a 256-byte page (BL_FLASH_PAGE_SIZE aligned). Caller verifies readback. */
BlStatus bl_hal_program_page(uint32_t exec_addr, const uint8_t page[BL_FLASH_PAGE_SIZE]);

/* Pointer for reading from execution address (erased words read as 0xE339E339). */
const uint8_t *bl_hal_read_ptr(uint32_t exec_addr);

/* Lock flash against further erase/program operations. */
void bl_hal_lock(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_BL_FLASH_HAL_H */
