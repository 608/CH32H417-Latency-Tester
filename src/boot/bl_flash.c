/*
 * bl_flash.c - Bootloader application flash state machine (see bl_flash.h).
 */
#include "bl_flash.h"

#include "app_image_check.h"
#include "bl_flash_hal.h"
#include "lat_bytes.h"
#include "lat_crc32.h"

static uint8_t s_page[BL_FLASH_PAGE_SIZE] __attribute__((aligned(4)));
static uint8_t s_meta_page[BL_FLASH_PAGE_SIZE] __attribute__((aligned(4)));
static uint32_t s_crc_table[256];
static bool s_meta_held;
static bool s_programming;
static uint32_t s_next;          /* Next expected write address */
static uint32_t s_page_base;     /* Base address of s_page */
static uint32_t s_page_fill;     /* Bytes buffered in s_page */
static uint32_t s_erased_end;    /* Sectors erased up to [BL_APP_BASE, s_erased_end) */
static uint32_t s_size_hint;
static uint32_t s_crc;           /* Running CRC-32 of received bytes */
static bool s_fault;             /* Flash timeout occurred (operation may still be active) */

/* Record flash timeouts to block RESET_APP until the next successful ERASE. */
static BlStatus note(BlStatus st)
{
    if (st == BL_STATUS_TIMEOUT)
    {
        s_fault = true;
    }
    return st;
}

static void abort_programming(void)
{
    s_programming = false;
    s_meta_held = false;
    s_page_fill = 0u;
    bl_hal_lock();
}

static bool in_crc_field(uint32_t exec_addr)
{
    uint32_t start = BL_APP_BASE + APP_META_CRC_FIELD_OFFSET;

    return (exec_addr >= start) && (exec_addr < (start + 4u));
}

/* Erase any unerased sectors up to the sector containing target_addr. */
static BlStatus ensure_erased(uint32_t exec_addr)
{
    while (s_erased_end <= exec_addr)
    {
        BlStatus st;

        if (s_erased_end >= BL_APP_LIMIT)
        {
            return BL_STATUS_RANGE;
        }
        st = note(bl_hal_erase_sector(s_erased_end));
        if (st != BL_STATUS_OK)
        {
            return st;
        }
        s_erased_end += BL_FLASH_SECTOR_SIZE;
    }
    return BL_STATUS_OK;
}

static BlStatus program_and_verify(uint32_t base, const uint8_t *page)
{
    BlStatus st;

    if (((base % BL_FLASH_PAGE_SIZE) != 0u) || (base < BL_APP_BASE) ||
        ((base + BL_FLASH_PAGE_SIZE) > BL_APP_LIMIT))
    {
        return BL_STATUS_RANGE;
    }
    st = ensure_erased(base);
    if (st != BL_STATUS_OK)
    {
        return st;
    }
    st = note(bl_hal_program_page(base, page));
    if (st != BL_STATUS_OK)
    {
        return st;
    }
    if (!lat_bytes_equal(bl_hal_read_ptr(base), page, BL_FLASH_PAGE_SIZE))
    {
        return BL_STATUS_FLASH_FAIL;
    }
    return BL_STATUS_OK;
}

/* Program a full page, or hold the metadata page in RAM until FINALIZE. */
static BlStatus flush_page(void)
{
    BlStatus st = BL_STATUS_OK;

    if (s_page_base == BL_META_PAGE)
    {
        lat_bytes_copy(s_meta_page, s_page, BL_FLASH_PAGE_SIZE);
        s_meta_held = true;
    }
    else
    {
        st = program_and_verify(s_page_base, s_page);
    }
    s_page_fill = 0u;
    s_page_base += BL_FLASH_PAGE_SIZE;
    lat_bytes_fill(s_page, 0xFFu, BL_FLASH_PAGE_SIZE);
    return st;
}

void bl_flash_init(void)
{
    lat_crc32_table_init(s_crc_table);
    s_programming = false;
    s_meta_held = false;
    s_next = BL_APP_BASE;
    s_page_base = BL_APP_BASE;
    s_page_fill = 0u;
    s_erased_end = BL_APP_BASE;
    s_size_hint = 0u;
    s_crc = LAT_CRC32_INIT;
    s_fault = false;
    lat_bytes_fill(s_page, 0xFFu, BL_FLASH_PAGE_SIZE);
}

BlStatus bl_flash_erase(uint32_t offset, uint32_t size_hint)
{
    BlStatus st;

    abort_programming();
    if (offset != BL_APP_BASE)
    {
        return BL_STATUS_RANGE;
    }
    if ((size_hint != 0u) && ((size_hint <= APP_IMAGE_SIZE_MIN_EXCL) || (size_hint > APP_IMAGE_SIZE_MAX)))
    {
        return BL_STATUS_RANGE;
    }
    /* Erase the first sector (containing metadata) to invalidate any existing app. */
    st = note(bl_hal_erase_sector(BL_APP_BASE));
    if (st != BL_STATUS_OK)
    {
        bl_hal_lock();
        return st;
    }
    s_fault = false;
    s_erased_end = BL_APP_BASE + BL_FLASH_SECTOR_SIZE;
    s_next = BL_APP_BASE;
    s_page_base = BL_APP_BASE;
    s_page_fill = 0u;
    s_meta_held = false;
    s_size_hint = size_hint;
    s_crc = LAT_CRC32_INIT;
    lat_bytes_fill(s_page, 0xFFu, BL_FLASH_PAGE_SIZE);
    s_programming = true;
    return BL_STATUS_OK;
}

BlStatus bl_flash_write(uint32_t offset, const uint8_t *data, uint32_t len)
{
    uint32_t i;

    if (!s_programming)
    {
        return BL_STATUS_SEQUENCE;
    }
    if ((len == 0u) || (len > BL_WRITE_MAX))
    {
        return BL_STATUS_RANGE;
    }
    if ((offset < BL_APP_BASE) || (offset >= BL_APP_LIMIT) || (len > (BL_APP_LIMIT - offset)))
    {
        return BL_STATUS_RANGE;
    }
    if (offset != s_next)
    {
        return BL_STATUS_SEQUENCE;
    }
    if ((s_size_hint != 0u) && ((offset + len) > (BL_APP_BASE + s_size_hint)))
    {
        return BL_STATUS_RANGE;
    }

    for (i = 0u; i < len; i++)
    {
        uint32_t addr = offset + i;
        uint8_t b = data[i];

        s_crc = lat_crc32_update(s_crc_table, s_crc, in_crc_field(addr) ? 0u : b);
        s_page[addr - s_page_base] = b;
        s_page_fill++;
        if (s_page_fill == BL_FLASH_PAGE_SIZE)
        {
            BlStatus st = flush_page();

            if (st != BL_STATUS_OK)
            {
                abort_programming();
                return st;
            }
        }
    }
    s_next = offset + len;
    return BL_STATUS_OK;
}

BlStatus bl_flash_finalize(uint32_t expected_crc, uint32_t expected_size, uint32_t *calc_crc)
{
    const uint8_t *m = s_meta_page + (APP_META_OFFSET % BL_FLASH_PAGE_SIZE);
    const uint8_t *flash;
    uint32_t crc;
    uint32_t i;
    BlStatus st;
    app_metadata_t meta;

    *calc_crc = 0u;
    if (!s_programming)
    {
        return BL_STATUS_SEQUENCE;
    }
    if ((expected_size <= APP_IMAGE_SIZE_MIN_EXCL) || (expected_size > APP_IMAGE_SIZE_MAX))
    {
        abort_programming();
        return BL_STATUS_RANGE;
    }
    /* (a) Received byte count */
    if ((s_next - BL_APP_BASE) != expected_size)
    {
        abort_programming();
        return BL_STATUS_SEQUENCE;
    }
    /* Pad and program the final partial page (never the metadata page). */
    if (s_page_fill != 0u)
    {
        st = flush_page();
        if (st != BL_STATUS_OK)
        {
            abort_programming();
            return st;
        }
    }
    /* (b) Held metadata */
    if (!s_meta_held || (lat_get_u32(&m[0]) != APP_META_MAGIC) ||
        (lat_get_u32(&m[28]) != APP_META_FORMAT_VERSION) || (lat_get_u32(&m[12]) != expected_size) ||
        (lat_get_u32(&m[8]) != expected_crc))
    {
        abort_programming();
        return BL_STATUS_NO_APP;
    }
    /* (c) Received data CRC */
    crc = lat_crc32_final(s_crc);
    *calc_crc = crc;
    if (crc != expected_crc)
    {
        abort_programming();
        return BL_STATUS_CRC_FAIL;
    }
    /* (d) Flash readback CRC (substituting held metadata page) */
    flash = bl_hal_read_ptr(BL_APP_BASE);
    crc = LAT_CRC32_INIT;
    for (i = 0u; i < expected_size; i++)
    {
        uint32_t addr = BL_APP_BASE + i;
        uint8_t b;

        if ((addr >= BL_META_PAGE) && (addr < (BL_META_PAGE + BL_FLASH_PAGE_SIZE)))
        {
            b = s_meta_page[addr - BL_META_PAGE];
        }
        else
        {
            b = flash[i];
        }
        crc = lat_crc32_update(s_crc_table, crc, in_crc_field(addr) ? 0u : b);
    }
    crc = lat_crc32_final(crc);
    *calc_crc = crc;
    if (crc != expected_crc)
    {
        abort_programming();
        return BL_STATUS_CRC_FAIL;
    }
    /* Program the held metadata page last. */
    st = program_and_verify(BL_META_PAGE, s_meta_page);
    if (st != BL_STATUS_OK)
    {
        abort_programming();
        return st;
    }
    s_programming = false;
    s_meta_held = false;
    bl_hal_lock();
    /* Final verification using the same check as boot. */
    if (bl_flash_check_app(&meta) != 0)
    {
        return BL_STATUS_FLASH_FAIL;
    }
    return BL_STATUS_OK;
}

uint32_t bl_flash_next_address(void)
{
    return s_next;
}

uint32_t bl_flash_page_fill(void)
{
    return s_page_fill;
}

bool bl_flash_programming(void)
{
    return s_programming;
}

bool bl_flash_boot_allowed(void)
{
    return !s_programming && !s_fault;
}

const char *bl_flash_check_app(app_metadata_t *meta)
{
    return app_image_check(bl_hal_read_ptr(BL_APP_BASE), APP_IMAGE_SIZE_MAX, meta, s_crc_table);
}
