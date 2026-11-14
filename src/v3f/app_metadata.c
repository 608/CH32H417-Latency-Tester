/*
 * app_metadata.c - V3F application metadata (src/shared/app_metadata.h).
 * Fixed at Flash 0x10400 by Link_CH32H417_v3f.ld; crc32, image_size, and build
 * are patched by tools/make_app_image.py.
 */
#include "app_metadata.h"

#include "lat_version.h"

const app_metadata_t g_app_metadata __attribute__((section(".app_meta"), used)) =
{
    .magic = APP_META_MAGIC,
    .fw_major = (uint16_t)LAT_FW_VERSION_MAJOR,
    .fw_minor = (uint16_t)LAT_FW_VERSION_MINOR,
    .crc32 = 0u,           /* patched by tools/make_app_image.py */
    .image_size = 0u,      /* patched by tools/make_app_image.py */
    .build = {0},          /* patched by tools/make_app_image.py */
    .format_version = APP_META_FORMAT_VERSION,
};
