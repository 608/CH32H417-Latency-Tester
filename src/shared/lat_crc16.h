/*
 * lat_crc16.h - CRC-16/CCITT-FALSE.
 *
 * Poly 0x1021, init 0xFFFF, no reflection, no final XOR ("123456789" -> 0x29B1).
 * Bit-by-bit (tableless) since max frame size is small.
 */
#ifndef LAT_CRC16_H
#define LAT_CRC16_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LAT_CRC16_INIT ((uint16_t)0xFFFFu)

static inline uint16_t lat_crc16_update(uint16_t crc, uint8_t byte)
{
    uint32_t bit;

    crc = (uint16_t)(crc ^ ((uint16_t)byte << 8));
    for (bit = 0u; bit < 8u; bit++)
    {
        if ((crc & 0x8000u) != 0u)
        {
            crc = (uint16_t)((crc << 1) ^ 0x1021u);
        }
        else
        {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static inline uint16_t lat_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = LAT_CRC16_INIT;
    uint32_t i;

    for (i = 0u; i < len; i++)
    {
        crc = lat_crc16_update(crc, data[i]);
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_CRC16_H */
