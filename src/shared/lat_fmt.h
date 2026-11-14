/*
 * lat_fmt.h - Minimal ASCII formatting into a fixed-size buffer.
 *
 * Avoids printf to save V5F ITCM and V3F RAM_CODE. Truncates on overflow
 * and always NUL-terminates.
 */
#ifndef LAT_FMT_H
#define LAT_FMT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LatStr
{
    char *buf;
    uint32_t cap; /* Buffer capacity including NUL terminator */
    uint32_t len;
} LatStr;

static inline void lat_str_init(LatStr *s, char *buf, uint32_t cap)
{
    s->buf = buf;
    s->cap = cap;
    s->len = 0u;
    if (cap != 0u)
    {
        buf[0] = '\0';
    }
}

static inline void lat_str_putc(LatStr *s, char c)
{
    if ((s->len + 1u) < s->cap)
    {
        s->buf[s->len++] = c;
        s->buf[s->len] = '\0';
    }
}

static inline void lat_str_puts(LatStr *s, const char *text)
{
    while (*text != '\0')
    {
        lat_str_putc(s, *text++);
    }
}

static inline void lat_str_u32(LatStr *s, uint32_t value)
{
    char digits[10];
    uint32_t n = 0u;

    do
    {
        digits[n++] = (char)('0' + (value % 10u));
        value /= 10u;
    } while (value != 0u);

    while (n > 0u)
    {
        lat_str_putc(s, digits[--n]);
    }
}

/* 8-digit uppercase hex without prefix. */
static inline void lat_str_hex32(LatStr *s, uint32_t value)
{
    static const char hex_digits[16] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    int shift;

    for (shift = 28; shift >= 0; shift -= 4)
    {
        lat_str_putc(s, hex_digits[(value >> (uint32_t)shift) & 0xFu]);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_FMT_H */
