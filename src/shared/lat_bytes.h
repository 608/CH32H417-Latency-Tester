/*
 * lat_bytes.h - Unaligned little-endian byte read/write helpers.
 *
 * Used by PC protocol and inter-core IPC to avoid unaligned struct casts.
 */
#ifndef LAT_BYTES_H
#define LAT_BYTES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline uint16_t lat_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t lat_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint64_t lat_get_u64(const uint8_t *p)
{
    return (uint64_t)lat_get_u32(p) | ((uint64_t)lat_get_u32(p + 4) << 32);
}

static inline int32_t lat_get_i32(const uint8_t *p)
{
    return (int32_t)lat_get_u32(p);
}

static inline void lat_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void lat_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline void lat_put_u64(uint8_t *p, uint64_t v)
{
    lat_put_u32(p, (uint32_t)v);
    lat_put_u32(p + 4, (uint32_t)(v >> 32));
}

static inline void lat_put_i32(uint8_t *p, int32_t v)
{
    lat_put_u32(p, (uint32_t)v);
}

static inline void lat_bytes_copy(uint8_t *dst, const uint8_t *src, uint32_t len)
{
    uint32_t i;

    for (i = 0u; i < len; i++)
    {
        dst[i] = src[i];
    }
}

static inline void lat_bytes_fill(uint8_t *dst, uint8_t value, uint32_t len)
{
    uint32_t i;

    for (i = 0u; i < len; i++)
    {
        dst[i] = value;
    }
}

/* Returns 1 if equal, 0 otherwise. */
static inline int lat_bytes_equal(const uint8_t *a, const uint8_t *b, uint32_t len)
{
    uint32_t i;

    for (i = 0u; i < len; i++)
    {
        if (a[i] != b[i])
        {
            return 0;
        }
    }
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_BYTES_H */
