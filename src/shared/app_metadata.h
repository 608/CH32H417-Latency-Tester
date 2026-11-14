/*
 * app_metadata.h - Application image metadata.
 *
 * 32-byte struct placed at +0x400 from V3F app base (Flash 0x10400).
 * CRC-32/ISO-HDLC covers [0, image_size) with the crc32 field zeroed,
 * including 0xFF padding between V3F and V5F.
 * Populated by tools/make_app_image.py.
 */
#ifndef LAT_APP_METADATA_H
#define LAT_APP_METADATA_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* V3F app execution base (BL jump target; must match Link_CH32H417_v3f.ld). */
#define APP_EXEC_BASE            ((uint32_t)0x00010000u)
/* V5F binary offset within the app image (Flash 0x30000). */
#define APP_V5F_OFFSET           ((uint32_t)0x00020000u)
/* Metadata offset from app base (must match Link_CH32H417_v3f.ld). */
#define APP_META_OFFSET          ((uint32_t)0x00000400u)

#define APP_META_MAGIC           ((uint32_t)0x4D54414Cu) /* 'LATM' */
#define APP_META_FORMAT_VERSION  ((uint32_t)1u)
#define APP_META_BUILD_LEN       (12u)

/* image_size must be in (APP_IMAGE_SIZE_MIN_EXCL, APP_IMAGE_SIZE_MAX] (384 KB max). */
#define APP_IMAGE_SIZE_MIN_EXCL  ((uint32_t)0x00020000u)
#define APP_IMAGE_SIZE_MAX       ((uint32_t)0x00060000u)

typedef struct __attribute__((packed)) AppMetadata
{
    uint32_t magic;
    uint16_t fw_major;
    uint16_t fw_minor;
    uint32_t crc32;
    uint32_t image_size;
    char     build[APP_META_BUILD_LEN];
    uint32_t format_version;
} app_metadata_t;

_Static_assert(sizeof(app_metadata_t) == 32u, "app_metadata_t must be 32 bytes");
_Static_assert(offsetof(app_metadata_t, fw_major) == 4u, "app_metadata_t.fw_major must be at +4");
_Static_assert(offsetof(app_metadata_t, crc32) == 8u, "app_metadata_t.crc32 must be at +8");
_Static_assert(offsetof(app_metadata_t, image_size) == 12u, "app_metadata_t.image_size must be at +12");
_Static_assert(offsetof(app_metadata_t, build) == 16u, "app_metadata_t.build must be at +16");
_Static_assert(offsetof(app_metadata_t, format_version) == 28u, "app_metadata_t.format_version must be at +28");

/* Byte range zeroed during CRC-32 calculation (offset from app base). */
#define APP_META_CRC_FIELD_OFFSET (APP_META_OFFSET + (uint32_t)offsetof(app_metadata_t, crc32))

#ifdef __cplusplus
}
#endif

#endif /* LAT_APP_METADATA_H */
