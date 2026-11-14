/*
 * pc_link_port.c - V3F hardware port for pc_link.c (pc_port_ipc_record is in main.c).
 */
#include "pc_link_port.h"

#include "ch32h417.h"

#include "app_metadata.h"
#include "boot_magic.h"
#include "console.h"
#include "lat_fence.h"
#include "lat_ipc.h"
#include "usbfs_dev.h"
#include "v3f_time.h"

/* ESIG chip UID at 0x1FFFF7E8 (8 bytes; next 4 bytes read as erased 0xE339E339 on silicon). */
#define V3F_ESIG_UNIID_ADDR  ((uintptr_t)0x1FFFF7E8u)
/* Trigger outputs T0..T7 = PA0..PA7. */
#define V3F_TRIGGER_MASK     ((uint16_t)0x00FFu)
/* Reserve space in con TX buffer before draining lossy logs. */
#define V3F_LOG_RESERVE      ((uint32_t)1216u)

static uint32_t s_rng;

void pc_port_uuid(uint8_t out[16])
{
    const volatile uint32_t *uid = (const volatile uint32_t *)V3F_ESIG_UNIID_ADDR;
    uint32_t w;
    uint32_t b;

    for (b = 0u; b < 16u; b++)
    {
        out[b] = 0u;
    }
    for (w = 0u; w < 2u; w++)
    {
        uint32_t value = uid[w];

        for (b = 0u; b < 4u; b++)
        {
            out[(w * 4u) + b] = (uint8_t)(value >> (8u * b));
        }
    }
}

void pc_port_build(uint8_t out[12])
{
    /* Read Flash metadata patched by tools/make_app_image.py (avoid RAM copy constant folding). */
    const volatile uint8_t *m = (const volatile uint8_t *)(uintptr_t)(APP_EXEC_BASE + APP_META_OFFSET + 16u);
    uint32_t i;

    for (i = 0u; i < APP_META_BUILD_LEN; i++)
    {
        uint8_t c = m[i];

        out[i] = ((c >= 0x20u) && (c <= 0x7Eu)) ? c : (uint8_t)'?';
    }
}

uint32_t pc_port_random(void)
{
    uint32_t x;

    if (s_rng == 0u)
    {
        uint8_t uid[16];
        uint32_t i;

        pc_port_uuid(uid);
        s_rng = 0x9E3779B9u ^ g_ipc_header.boot_epoch;
        for (i = 0u; i < 12u; i++)
        {
            s_rng = (s_rng * 31u) ^ uid[i];
        }
    }
    x = s_rng ^ v3f_ticks();
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng = (x != 0u) ? x : 0x2545F491u;
    return s_rng;
}

bool pc_port_v5f_running(void)
{
    return lat_ipc_boot_state_load() == LAT_BOOT_STATE_V5F_RUNNING;
}

uint32_t pc_port_v5f_diag(void)
{
    return lat_ipc_v5f_diag_load();
}

void pc_port_release_triggers(void)
{
    /* Atomic BSHR write; safe against concurrent V5F writes. */
    GPIOA->BSHR = V3F_TRIGGER_MASK;
}

bool pc_port_log_ready(void)
{
    return con_free() >= V3F_LOG_RESERVE;
}

void pc_port_log(const char *line)
{
    con_puts(line);
    con_puts("\r\n");
}

void pc_port_enter_bootloader(void)
{
    con_puts("[v3f] stopping USB and resetting into the bootloader (boot magic STAY)\r\n");
    con_flush();
    usbfs_dev_stop();
    /* Wait for host to detect disconnect after releasing pull-up. */
    v3f_delay_ms(20u);
    boot_magic_set(LAT_BOOT_MAGIC_STAY);
    LAT_FENCE_IORW();
    NVIC_SystemReset();
    for (;;)
    {
    }
}
