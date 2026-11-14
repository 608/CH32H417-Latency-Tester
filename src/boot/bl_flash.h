/*
 * bl_flash.h - Bootloader application flash state machine.
 *
 * Sequence:
 *   ERASE     Starts at 0x10000; erases the first 8 KB sector (0x10000-0x11FFF)
 *             containing metadata, invalidating any existing app.
 *   WRITE     Sequential writes only (1..56 bytes in [0x10000, 0x70000)).
 *             Programs and verifies each 256-byte page; erases subsequent
 *             sectors just before their first write. The metadata page
 *             (0x10400-0x104FF) is held in RAM until FINALIZE.
 *   FINALIZE  Programs and verifies the held metadata page only after verifying
 *             byte count, metadata fields, received CRC-32, and flash readback CRC-32.
 *   Any failure aborts the sequence until the next ERASE.
 */
#ifndef LAT_BL_FLASH_H
#define LAT_BL_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#include "app_metadata.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum BlStatus
{
    BL_STATUS_OK = 0x00,
    BL_STATUS_RANGE = 0x01,
    BL_STATUS_ALIGN = 0x02,
    BL_STATUS_SEQUENCE = 0x03,
    BL_STATUS_NO_APP = 0x04,
    BL_STATUS_FLASH_FAIL = 0x05,
    BL_STATUS_BUSY = 0x06,
    BL_STATUS_CRC_FAIL = 0x07,
    BL_STATUS_TIMEOUT = 0x08,
    BL_STATUS_UNKNOWN_COMMAND = 0x09
} BlStatus;

#define BL_FLASH_PAGE_SIZE    ((uint32_t)256u)    /* FLASH_ProgramPage_Fast unit */
#define BL_FLASH_SECTOR_SIZE  ((uint32_t)8192u)   /* FLASH_ErasePage unit */
#define BL_APP_BASE           APP_EXEC_BASE                                  /* 0x10000 */
#define BL_APP_LIMIT          ((uint32_t)(APP_EXEC_BASE + APP_IMAGE_SIZE_MAX)) /* 0x70000 (exclusive) */
#define BL_META_PAGE          ((uint32_t)(APP_EXEC_BASE + (APP_META_OFFSET & ~(BL_FLASH_PAGE_SIZE - 1u)))) /* 0x10400 */
#define BL_WRITE_MAX          (56u)

_Static_assert((BL_APP_BASE % BL_FLASH_SECTOR_SIZE) == 0u, "application base must be sector aligned");
_Static_assert((BL_APP_LIMIT % BL_FLASH_SECTOR_SIZE) == 0u, "application limit must be sector aligned");
_Static_assert((APP_META_OFFSET % BL_FLASH_PAGE_SIZE) + 32u <= BL_FLASH_PAGE_SIZE, "metadata must fit in one page");
_Static_assert(BL_META_PAGE < (BL_APP_BASE + BL_FLASH_SECTOR_SIZE), "metadata page must be in the first sector");

void bl_flash_init(void);

/* ERASE: offset must be 0x10000; size_hint=0 skips the size check. */
BlStatus bl_flash_erase(uint32_t offset, uint32_t size_hint);

BlStatus bl_flash_write(uint32_t offset, const uint8_t *data, uint32_t len);

/* FINALIZE: verifies (a)-(d) before programming the held metadata page. */
BlStatus bl_flash_finalize(uint32_t expected_crc, uint32_t expected_size, uint32_t *calc_crc);

uint32_t bl_flash_next_address(void);
uint32_t bl_flash_page_fill(void);
bool bl_flash_programming(void);

/*
 * Returns true if RESET_APP is allowed; false while programming or after a
 * flash timeout (until the next successful ERASE).
 */
bool bl_flash_boot_allowed(void);

/* Verify the application in flash using app_image_check.h; returns NULL if valid. */
const char *bl_flash_check_app(app_metadata_t *meta);

#ifdef __cplusplus
}
#endif

#endif /* LAT_BL_FLASH_H */
