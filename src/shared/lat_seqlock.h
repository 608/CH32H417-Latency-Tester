/*
 * lat_seqlock.h - Single-writer seqlock for status snapshots.
 *
 * Bound reader retries if writer can preempt reader on the same core.
 */
#ifndef LAT_SEQLOCK_H
#define LAT_SEQLOCK_H

#include <stdbool.h>
#include <stdint.h>

#include "lat_fence.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline void lat_seqlock_write_begin(uint32_t *seq)
{
    uint32_t s = __atomic_load_n(seq, __ATOMIC_RELAXED);

    __atomic_store_n(seq, s + 1u, __ATOMIC_RELAXED); /* Odd: write in progress */
    LAT_FENCE_RW();
}

static inline void lat_seqlock_write_end(uint32_t *seq)
{
    uint32_t s;

    LAT_FENCE_RW();
    s = __atomic_load_n(seq, __ATOMIC_RELAXED);
    __atomic_store_n(seq, s + 1u, __ATOMIC_RELAXED); /* Even: stable */
}

static inline uint32_t lat_seqlock_read_begin(const uint32_t *seq)
{
    uint32_t s = __atomic_load_n(seq, __ATOMIC_RELAXED);

    LAT_FENCE_RW();
    return s;
}

/* Returns true if the read must be retried. */
static inline bool lat_seqlock_read_retry(const uint32_t *seq, uint32_t begin)
{
    LAT_FENCE_RW();
    if ((begin & 1u) != 0u)
    {
        return true;
    }
    return __atomic_load_n(seq, __ATOMIC_RELAXED) != begin;
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_SEQLOCK_H */
