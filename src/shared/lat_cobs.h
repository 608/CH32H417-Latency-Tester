/*
 * lat_cobs.h - COBS (Consistent Overhead Byte Stuffing) encode/decode.
 *
 * Standard COBS without trailing 0x01 on exact 254-byte non-zero blocks;
 * 0x00 delimiter is added/stripped by the caller.
 */
#ifndef LAT_COBS_H
#define LAT_COBS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum encoded length for n input bytes. */
#define LAT_COBS_MAX_ENCODED(n) ((uint32_t)(n) + (((uint32_t)(n) + 253u) / 254u) + (((n) == 0u) ? 1u : 0u))

/*
 * lat_cobs_encode: encode len bytes from in into out; returns written length,
 * or 0 if out_cap is insufficient.
 */
static inline uint32_t lat_cobs_encode(const uint8_t *in, uint32_t len, uint8_t *out, uint32_t out_cap)
{
    uint32_t code_index = 0u;
    uint32_t o = 1u;
    uint8_t code = 1u;
    uint32_t i;

    if (out_cap == 0u)
    {
        return 0u;
    }
    for (i = 0u; i < len; i++)
    {
        uint8_t b = in[i];

        if (b == 0u)
        {
            out[code_index] = code;
            if (o >= out_cap)
            {
                return 0u;
            }
            code_index = o++;
            code = 1u;
        }
        else
        {
            if (o >= out_cap)
            {
                return 0u;
            }
            out[o++] = b;
            code++;
            /* Close 254-byte block; do not emit a trailing 0x01 at end of input. */
            if ((code == 0xFFu) && ((i + 1u) < len))
            {
                out[code_index] = code;
                if (o >= out_cap)
                {
                    return 0u;
                }
                code_index = o++;
                code = 1u;
            }
        }
    }
    out[code_index] = code;
    return o;
}

/*
 * lat_cobs_decode: decode len bytes (without 0x00 delimiter) into out;
 * returns true and sets *out_len, or false on invalid input/overflow.
 */
static inline bool lat_cobs_decode(const uint8_t *in, uint32_t len, uint8_t *out, uint32_t out_cap,
                                   uint32_t *out_len)
{
    uint32_t i = 0u;
    uint32_t o = 0u;

    if (len == 0u)
    {
        return false;
    }
    while (i < len)
    {
        uint8_t code = in[i++];
        uint32_t j;

        if (code == 0u)
        {
            return false;
        }
        for (j = 1u; j < code; j++)
        {
            uint8_t b;

            if (i >= len)
            {
                return false;
            }
            b = in[i++];
            if (b == 0u)
            {
                return false;
            }
            if (o >= out_cap)
            {
                return false;
            }
            out[o++] = b;
        }
        if ((code != 0xFFu) && (i < len))
        {
            if (o >= out_cap)
            {
                return false;
            }
            out[o++] = 0u;
        }
    }
    *out_len = o;
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_COBS_H */
