/*
 * boot_magic.c - BootMagic storage in .boot_magic (NOLOAD) at 0x2017FFF8.
 */
#include "boot_magic.h"

__attribute__((section(".boot_magic"), aligned(4), used)) BootMagic g_boot_magic;
