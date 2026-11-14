/*
 * app_image_check.h - Application image verification.
 *
 * Checks:
 *   1. First 4 bytes are not erased (0xE339E339), 0, or 0xFFFFFFFF.
 *   2. Metadata at +0x400 has valid magic and format_version.
 *   3. 0x20000 < image_size <= 0x60000 and within available bytes.
 *   4. CRC-32/ISO-HDLC over [0, image_size) with crc32 field zeroed matches.
 */
#ifndef LAT_APP_IMAGE_CHECK_H
#define LAT_APP_IMAGE_CHECK_H

#include <stdint.h>

#include "app_metadata.h"
#include "lat_crc32.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Erased flash word value when read from execution address. */
#define APP_FLASH_ERASED_WORD ((uint32_t)0xE339E339u)

static inline uint32_t app_image_read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/*
 * app_image_check: verify available bytes starting at image.
 * Returns NULL on success (copying metadata to *meta) or an ASCII reason string.
 * table is a 256-entry scratch buffer for CRC-32.
 */
static inline const char *app_image_check(const uint8_t *image, uint32_t available,
                                          app_metadata_t *meta, uint32_t table[256])
{
    const uint8_t *m = image + APP_META_OFFSET;
    uint32_t first_word;
    uint32_t crc;
    uint32_t i;

    if (available < (APP_META_OFFSET + (uint32_t)sizeof(app_metadata_t)))
    {
        return "image too short for metadata";
    }
    first_word = app_image_read_u32(image);
    if ((first_word == APP_FLASH_ERASED_WORD) || (first_word == 0u) || (first_word == 0xFFFFFFFFu))
    {
        return "no application image";
    }

    meta->magic = app_image_read_u32(m + 0u);
    meta->fw_major = (uint16_t)(m[4] | (m[5] << 8));
    meta->fw_minor = (uint16_t)(m[6] | (m[7] << 8));
    meta->crc32 = app_image_read_u32(m + 8u);
    meta->image_size = app_image_read_u32(m + 12u);
    for (i = 0u; i < APP_META_BUILD_LEN; i++)
    {
        meta->build[i] = (char)m[16u + i];
    }
    meta->format_version = app_image_read_u32(m + 28u);

    if (meta->magic != APP_META_MAGIC)
    {
        return "metadata magic mismatch";
    }
    if (meta->format_version != APP_META_FORMAT_VERSION)
    {
        return "metadata format_version mismatch";
    }
    if ((meta->image_size <= APP_IMAGE_SIZE_MIN_EXCL) || (meta->image_size > APP_IMAGE_SIZE_MAX))
    {
        return "image_size out of range";
    }
    if (meta->image_size > available)
    {
        return "image_size exceeds the readable area";
    }

    /* Always compute and verify CRC even if stored crc32 is 0. */
    lat_crc32_table_init(table);
    crc = LAT_CRC32_INIT;
    for (i = 0u; i < meta->image_size; i++)
    {
        uint8_t byte = image[i];

        if ((i >= APP_META_CRC_FIELD_OFFSET) && (i < (APP_META_CRC_FIELD_OFFSET + 4u)))
        {
            byte = 0u;
        }
        crc = lat_crc32_update(table, crc, byte);
    }
    crc = lat_crc32_final(crc);
    if (crc != meta->crc32)
    {
        return "CRC32 mismatch";
    }
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_APP_IMAGE_CHECK_H */
