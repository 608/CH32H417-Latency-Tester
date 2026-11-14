/*
 * meas_poll.c - Polling interval measurement aggregation (see meas_poll.h)
 */
#include "meas_poll.h"

#include "lat_bytes.h"

uint32_t meas_poll_bin_count(uint32_t period_ticks)
{
    uint64_t bins = ((uint64_t)period_ticks / MEAS_POLL_BIN_WIDTH) * 4u;

    if (bins > MEAS_POLL_BINS_MAX)
    {
        bins = MEAS_POLL_BINS_MAX;
    }
    if (bins == 0u)
    {
        bins = 1u;
    }
    return (uint32_t)bins;
}

uint32_t meas_poll_long_threshold(uint32_t period_ticks)
{
    uint64_t twice = (uint64_t)period_ticks * 2u;
    uint64_t floor = 1000u * MEAS_POLL_BIN_WIDTH; /* 1000us */

    return (uint32_t)((twice > floor) ? twice : floor);
}

void meas_poll_begin(MeasPoll *s, uint32_t period_ticks, bool keep_raw, uint32_t raw_max, uint32_t *pool,
                     uint32_t pool_words, uint8_t *longs)
{
    uint32_t rest;

    lat_bytes_fill((uint8_t *)s, 0u, (uint32_t)sizeof(*s));
    s->bin_count = meas_poll_bin_count(period_ticks);
    s->long_threshold = meas_poll_long_threshold(period_ticks);
    s->keep_raw = keep_raw ? 1u : 0u;
    s->raw_max = raw_max;
    s->hist = pool;
    rest = (pool_words > s->bin_count) ? (pool_words - s->bin_count) : 0u;
    s->raw = &pool[s->bin_count];
    s->raw_cap = keep_raw ? ((raw_max < rest) ? raw_max : rest) : 0u;
    s->longs = longs;
    lat_bytes_fill((uint8_t *)pool, 0u, s->bin_count * 4u);
    s->active = 1u;
}

static void long_put(MeasPoll *s, uint64_t end, uint64_t interval)
{
    uint8_t *e;
    uint64_t rel;

    if (s->long_count >= MEAS_POLL_LONG_MAX)
    {
        s->long_truncated = 1u;
        return;
    }
    /* Pack into 12 bytes: 48-bit relative end time from first_data_time, 48-bit interval. */
    rel = end - s->first_data_time;
    e = &s->longs[s->long_count * MEAS_POLL_LONG_ENTRY];
    lat_put_u32(&e[0], (uint32_t)rel);
    lat_put_u16(&e[4], (uint16_t)(rel >> 32));
    lat_put_u16(&e[6], (uint16_t)interval);
    lat_put_u32(&e[8], (uint32_t)(interval >> 16));
    s->long_count++;
}

void meas_poll_long_get(const MeasPoll *s, uint32_t i, uint64_t *end, uint64_t *interval)
{
    const uint8_t *e = &s->longs[i * MEAS_POLL_LONG_ENTRY];
    uint64_t rel = (uint64_t)lat_get_u32(&e[0]) | ((uint64_t)lat_get_u16(&e[4]) << 32);

    *end = s->first_data_time + rel;
    *interval = (uint64_t)lat_get_u16(&e[6]) | ((uint64_t)lat_get_u32(&e[8]) << 16);
}

void meas_poll_data(MeasPoll *s, uint64_t t1)
{
    uint64_t iv;

    if (!s->active)
    {
        return;
    }
    s->data_count++;
    if (s->data_count == 1u)
    {
        s->first_data_time = t1;
        s->last_t1 = t1;
        return;
    }
    iv = t1 - s->last_t1;
    s->last_t1 = t1;
    s->interval_count++;
    if ((s->interval_count == 1u) || (iv < s->min_interval))
    {
        s->min_interval = iv;
    }
    if (iv > s->max_interval)
    {
        s->max_interval = iv;
    }
    if (iv < ((uint64_t)s->bin_count * MEAS_POLL_BIN_WIDTH))
    {
        s->hist[(uint32_t)(iv / MEAS_POLL_BIN_WIDTH)]++;
    }
    else
    {
        s->overflow_count++;
    }
    if ((iv >= s->long_threshold) || (iv >= ((uint64_t)s->bin_count * MEAS_POLL_BIN_WIDTH)))
    {
        long_put(s, t1, iv);
    }
    if (iv < MEAS_POLL_50MS)
    {
        s->count_below_50ms++;
        s->sum_below_50ms += iv;
    }
    if (s->keep_raw != 0u)
    {
        if (s->raw_count < s->raw_cap)
        {
            s->raw[s->raw_count++] = (iv > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)iv;
        }
        else
        {
            s->raw_truncated = 1u;
        }
    }
}

void meas_poll_nak(MeasPoll *s)
{
    if (s->active)
    {
        s->nak_count++;
    }
}

void meas_poll_error(MeasPoll *s)
{
    if (s->active)
    {
        s->error_count++;
    }
}

void meas_poll_stop(MeasPoll *s)
{
    s->active = 0u;
}

void meas_poll_result(const MeasPoll *s, uint32_t run_id, uint8_t reason, uint32_t duration_ms, uint8_t *out)
{
    lat_bytes_fill(out, 0u, MEAS_POLL_RESULT_LEN);
    lat_put_u32(&out[0], run_id);
    out[4] = reason;
    out[5] = s->raw_truncated;
    out[6] = s->long_truncated;
    lat_put_u32(&out[8], duration_ms);
    lat_put_u32(&out[12], s->data_count);
    lat_put_u32(&out[16], s->nak_count);
    lat_put_u32(&out[20], s->error_count);
    lat_put_u32(&out[24], s->interval_count);
    lat_put_u32(&out[28], s->overflow_count);
    lat_put_u32(&out[32], MEAS_POLL_BIN_WIDTH);
    lat_put_u32(&out[36], s->bin_count);
    lat_put_u64(&out[40], (s->interval_count != 0u) ? s->min_interval : 0u);
    lat_put_u64(&out[48], (s->interval_count != 0u) ? s->max_interval : 0u);
    lat_put_u64(&out[56], s->sum_below_50ms);
    lat_put_u32(&out[64], s->count_below_50ms);
    lat_put_u32(&out[68], s->long_threshold);
    lat_put_u32(&out[72], s->long_count);
    lat_put_u32(&out[76], s->raw_count);
    lat_put_u64(&out[80], (s->data_count != 0u) ? s->first_data_time : 0u);
}

uint32_t meas_poll_export(const MeasPoll *s, uint8_t kind, uint8_t *dst, uint32_t cap)
{
    uint32_t i;
    uint32_t n;

    switch (kind)
    {
    case 7u:
        n = s->raw_count * 4u;
        if (n > cap)
        {
            return 0xFFFFFFFFu;
        }
        for (i = 0u; i < s->raw_count; i++)
        {
            lat_put_u32(&dst[4u * i], s->raw[i]);
        }
        return n;
    case 8u:
        n = s->bin_count * 4u;
        if (n > cap)
        {
            return 0xFFFFFFFFu;
        }
        for (i = 0u; i < s->bin_count; i++)
        {
            lat_put_u32(&dst[4u * i], s->hist[i]);
        }
        return n;
    case 9u:
        n = s->long_count * 16u;
        if (n > cap)
        {
            return 0xFFFFFFFFu;
        }
        for (i = 0u; i < s->long_count; i++)
        {
            uint64_t end;
            uint64_t iv;

            meas_poll_long_get(s, i, &end, &iv);
            lat_put_u64(&dst[16u * i], end);
            lat_put_u64(&dst[(16u * i) + 8u], iv);
        }
        return n;
    default:
        return 0xFFFFFFFFu;
    }
}
