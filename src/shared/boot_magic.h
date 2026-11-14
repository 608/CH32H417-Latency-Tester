/*
 * boot_magic.h - One-shot bootloader request in NOLOAD SRAM.
 *
 * 8 bytes at 0x2017FFF8 (value and its bitwise inverse to reject uninitialized
 * SRAM). Bootloader consumes and clears it on startup.
 */
#ifndef LAT_BOOT_MAGIC_H
#define LAT_BOOT_MAGIC_H

#include <stddef.h>
#include <stdint.h>

#include "lat_fence.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Request to stay in bootloader. */
#define LAT_BOOT_MAGIC_STAY          ((uint32_t)0x59415453u) /* 'STAY' */
/* Request to jump to app after update without checking PB6. */
#define LAT_BOOT_MAGIC_APP_IMMEDIATE ((uint32_t)0x49505041u) /* 'APPI' */

typedef struct BootMagic
{
    uint32_t magic;
    uint32_t magic_inv; /* ~magic */
} BootMagic;

_Static_assert(sizeof(BootMagic) == 8u, "BootMagic must be 8 bytes (BOOT_MAGIC region)");
_Static_assert(offsetof(BootMagic, magic) == 0u, "BootMagic.magic must be the first field");

/* Defined in src/shared/boot_magic.c (.boot_magic section at 0x2017FFF8). */
extern BootMagic g_boot_magic;

/* Returns valid magic or 0; always clears the region. */
static inline uint32_t boot_magic_consume(void)
{
    volatile BootMagic *bm = &g_boot_magic;
    uint32_t magic = bm->magic;
    uint32_t inv = bm->magic_inv;

    bm->magic = 0u;
    bm->magic_inv = 0u;
    LAT_FENCE_RW();
    return (inv == ~magic) ? magic : 0u;
}

static inline void boot_magic_set(uint32_t magic)
{
    volatile BootMagic *bm = &g_boot_magic;

    bm->magic = magic;
    bm->magic_inv = ~magic;
    LAT_FENCE_RW();
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_BOOT_MAGIC_H */
