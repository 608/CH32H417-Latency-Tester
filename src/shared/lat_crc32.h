/*
 * lat_crc32.h - CRC-32/ISO-HDLC.
 *
 * Reflected poly 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF (matches zlib.crc32).
 *
 * Usage:
 *     static uint32_t table[256];
 *     lat_crc32_table_init(table);
 *     uint32_t crc = LAT_CRC32_INIT;
 *     crc = lat_crc32_update(table, crc, byte);
 *     crc = lat_crc32_final(crc);
 */
#ifndef LAT_CRC32_H
#define LAT_CRC32_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LAT_CRC32_INIT ((uint32_t)0xFFFFFFFFu)

static inline void lat_crc32_table_init(uint32_t table[256])
{
    uint32_t i;
    uint32_t bit;

    for (i = 0u; i < 256u; i++)
    {
        uint32_t c = i;

        for (bit = 0u; bit < 8u; bit++)
        {
            c = ((c & 1u) != 0u) ? ((c >> 1) ^ 0xEDB88320u) : (c >> 1);
        }
        table[i] = c;
    }
}

static inline uint32_t lat_crc32_update(const uint32_t table[256], uint32_t crc, uint8_t byte)
{
    return (crc >> 8) ^ table[(crc ^ (uint32_t)byte) & 0xFFu];
}

static inline uint32_t lat_crc32_final(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFu;
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_CRC32_H */
