/*
 * meas_watch.c - SET_WATCH parsing and evaluation (see meas_watch.h)
 */
#include "meas_watch.h"

#include "lat_bytes.h"

bool meas_watch_parse(const uint8_t *p, uint32_t n, MeasWatch *w)
{
    uint32_t i;

    if (n != MEAS_WATCH_LEN)
    {
        return false;
    }
    lat_bytes_fill((uint8_t *)w, 0u, (uint32_t)sizeof(*w));
    w->iface = p[0];
    w->ep_in = p[1];
    w->report_id_mode = p[2];
    w->report_id = p[3];
    w->min_length = lat_get_u16(&p[4]);
    w->kind = p[6];
    w->bit_offset = lat_get_u16(&p[8]);
    w->bit_size = p[10];
    w->is_signed = (p[11] != 0u) ? 1u : 0u;
    if ((w->report_id_mode > 1u) || (w->kind > MEAS_WATCH_KIND_AXIS) || (w->bit_size < 1u) || (w->bit_size > 32u) ||
        (w->min_length > MEAS_REPORT_MAX) ||
        (((uint32_t)w->bit_offset + w->bit_size) > ((uint32_t)w->min_length * 8u)))
    {
        return false;
    }
    if (w->kind == MEAS_WATCH_KIND_SET)
    {
        w->value_count = p[12];
        if ((w->value_count < 1u) || (w->value_count > MEAS_WATCH_VALUES))
        {
            return false;
        }
        for (i = 0u; i < MEAS_WATCH_VALUES; i++)
        {
            w->pressed_values[i] = lat_get_u32(&p[16u + (4u * i)]);
        }
    }
    else
    {
        w->neutral = lat_get_i32(&p[12]);
        w->threshold = lat_get_u32(&p[16]);
        w->direction = (int8_t)p[20];
        if ((w->direction < -1) || (w->direction > 1))
        {
            return false;
        }
    }
    return true;
}

uint32_t meas_watch_extract(const uint8_t *data, uint16_t bit_offset, uint8_t bit_size, uint8_t is_signed)
{
    uint32_t byte = (uint32_t)bit_offset >> 3;
    uint32_t shift = (uint32_t)bit_offset & 7u;
    uint32_t nbytes = (shift + bit_size + 7u) >> 3; /* 1..5 */
    uint64_t acc = 0u;
    uint32_t i;
    uint32_t v;

    for (i = 0u; i < nbytes; i++)
    {
        acc |= (uint64_t)data[byte + i] << (8u * i);
    }
    acc >>= shift;
    if (bit_size < 32u)
    {
        v = (uint32_t)acc & ((1u << bit_size) - 1u);
        if ((is_signed != 0u) && ((v >> (bit_size - 1u)) != 0u))
        {
            v |= ~((1u << bit_size) - 1u);
        }
    }
    else
    {
        v = (uint32_t)acc;
    }
    return v;
}

uint8_t meas_watch_eval(const MeasWatch *w, const uint8_t *data, uint16_t len)
{
    uint32_t v;

    if ((len < w->min_length) || (len == 0u))
    {
        return MEAS_ST_NONE;
    }
    if ((w->report_id_mode != 0u) && (data[0] != w->report_id))
    {
        return MEAS_ST_NONE;
    }
    if (((uint32_t)w->bit_offset + w->bit_size) > ((uint32_t)len * 8u))
    {
        return MEAS_ST_NONE;
    }
    v = meas_watch_extract(data, w->bit_offset, w->bit_size, w->is_signed);

    if (w->kind == MEAS_WATCH_KIND_SET)
    {
        uint32_t i;

        for (i = 0u; i < w->value_count; i++)
        {
            if (v == w->pressed_values[i])
            {
                return MEAS_ST_PRESSED;
            }
        }
        return MEAS_ST_RELEASED;
    }
    else
    {
        int64_t value = (w->is_signed != 0u) ? (int64_t)(int32_t)v : (int64_t)v;
        int64_t d = value - (int64_t)w->neutral;
        int64_t th = (int64_t)w->threshold;
        int64_t ad = (d < 0) ? -d : d;

        if (ad < th)
        {
            return MEAS_ST_RELEASED;
        }
        if (w->direction > 0)
        {
            return (d >= th) ? MEAS_ST_PRESSED : MEAS_ST_OTHER;
        }
        if (w->direction < 0)
        {
            return (d <= -th) ? MEAS_ST_PRESSED : MEAS_ST_OTHER;
        }
        return MEAS_ST_PRESSED;
    }
}
