/*
 * lat_ring.h - Single-producer, single-consumer (SPSC) variable-length ring buffer.
 *
 * Allocates in 8-byte units (LAT_RING_UNIT) instead of fixed max-size slots
 * so small reports/logs do not waste 90% of the buffer. Each record is an
 * 8-byte header (fits in 1 unit) plus payload rounded up to 8 bytes.
 *
 * Full-ring policies:
 *   lat_ring_push_lossy   Drop and increment drop_count + lost_by_type[type].
 *   lat_ring_try_push     Return false without counting (main-loop retry).
 *   lat_ring_push_nowait  Increment overflow_count + lost_by_type[type] (ISR).
 *
 * Callers must not access the same ring from multiple contexts without locking.
 */
#ifndef LAT_RING_H
#define LAT_RING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lat_fence.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LAT_RING_ABI_VERSION   ((uint16_t)1u)
/* Allocation unit (slot_size in ring header). */
#define LAT_RING_UNIT          ((uint32_t)8u)
/* Max payload bytes per record (matches PC protocol max). */
#define LAT_RING_PAYLOAD_MAX   ((uint32_t)1152u)
/* Number of per-type loss counters; types >= this map to lost_by_type[0]. */
#define LAT_RING_TYPE_COUNT    ((uint32_t)16u)
/* Byte offset from ring base to data area (header + stats). */
#define LAT_RING_HEAD_AREA     ((uint32_t)256u)
/* Max units per record. */
#define LAT_RING_MAX_RECORD_UNITS (1u + ((LAT_RING_PAYLOAD_MAX + LAT_RING_UNIT - 1u) / LAT_RING_UNIT))

/* Ring header (24 bytes). */
typedef struct LatRingHdr
{
    uint16_t abi_version;      /* +0  LAT_RING_ABI_VERSION */
    uint16_t slot_size;        /* +2  Allocation unit in bytes (LAT_RING_UNIT) */
    uint16_t capacity;         /* +4  Data area units (power of 2) */
    uint16_t reserved;         /* +6 */
    volatile uint32_t head;    /* +8  Producer-written */
    volatile uint32_t tail;    /* +12 Consumer-written */
    uint32_t drop_count;       /* +16 Producer: dropped on full (lossy ring) */
    uint32_t overflow_count;   /* +20 Producer: failed writes on safe ring (non-zero is fault) */
} LatRingHdr;

_Static_assert(sizeof(LatRingHdr) == 24u, "LatRingHdr must be 24 bytes");
_Static_assert(offsetof(LatRingHdr, head) == 8u, "LatRingHdr.head offset changed");
_Static_assert(offsetof(LatRingHdr, tail) == 12u, "LatRingHdr.tail offset changed");
_Static_assert(offsetof(LatRingHdr, drop_count) == 16u, "LatRingHdr.drop_count offset changed");
_Static_assert(offsetof(LatRingHdr, overflow_count) == 20u, "LatRingHdr.overflow_count offset changed");

/* First 256 bytes of ring region: header followed by per-writer stats. */
typedef struct LatRingHead
{
    LatRingHdr hdr;                               /* +0 */
    uint32_t pad0[2];                             /* +24 */
    /* Producer-written (+32) */
    uint32_t push_count;                          /* Total pushed; next record seq */
    uint32_t reject_count;                        /* Rejected due to oversized payload */
    uint32_t lost_by_type[LAT_RING_TYPE_COUNT];   /* Dropped or overflowed count by type */
    uint32_t pad1[6];
    /* Consumer-written (+128) */
    uint32_t pop_count;                           /* Total popped */
    uint32_t resync_count;                        /* Tail resyncs to head on corruption */
    uint32_t pad2[30];
} LatRingHead;

_Static_assert(sizeof(LatRingHead) == LAT_RING_HEAD_AREA, "LatRingHead must be 256 bytes");
_Static_assert(offsetof(LatRingHead, push_count) == 32u, "LatRingHead.push_count offset changed");
_Static_assert(offsetof(LatRingHead, pop_count) == 128u, "LatRingHead.pop_count offset changed");

/* Record header at start of each entry (8 bytes, 1 unit). */
typedef struct LatRingRecHdr
{
    uint16_t len;    /* Payload bytes */
    uint8_t  type;   /* LAT_CMD_* / LAT_EVT_* */
    uint8_t  flags;  /* Type-specific flags */
    uint32_t seq;    /* Producer push_count */
} LatRingRecHdr;

_Static_assert(sizeof(LatRingRecHdr) == LAT_RING_UNIT, "LatRingRecHdr must fit one unit");

/* Popped record (consumer scratch). */
typedef struct LatRingRecord
{
    uint16_t len;
    uint8_t  type;
    uint8_t  flags;
    uint32_t seq;
    uint8_t  payload[LAT_RING_PAYLOAD_MAX];
} LatRingRecord;

/*
 * Per-core ring handle; units is compile-time constant, shared capacity is
 * checked at startup.
 */
typedef struct LatRing
{
    LatRingHead *head;
    uint8_t *data;
    uint32_t units;   /* Power of 2 */
} LatRing;

static inline uint32_t lat_ring_load(const volatile uint32_t *p)
{
    return __atomic_load_n(p, __ATOMIC_RELAXED);
}

static inline void lat_ring_store(volatile uint32_t *p, uint32_t value)
{
    __atomic_store_n(p, value, __ATOMIC_RELAXED);
}

static inline uint32_t lat_ring_type_index(uint8_t type)
{
    return ((uint32_t)type < LAT_RING_TYPE_COUNT) ? (uint32_t)type : 0u;
}

/* Units occupied by a record with len payload bytes. */
static inline uint32_t lat_ring_record_units(uint32_t len)
{
    return 1u + ((len + LAT_RING_UNIT - 1u) / LAT_RING_UNIT);
}

/*
 * lat_ring_setup: write fixed header fields to a zeroed ring region
 * (called by V3F before starting V5F).
 */
static inline void lat_ring_setup(const LatRing *r)
{
    r->head->hdr.abi_version = LAT_RING_ABI_VERSION;
    r->head->hdr.slot_size = (uint16_t)LAT_RING_UNIT;
    r->head->hdr.capacity = (uint16_t)r->units;
    r->head->hdr.head = 0u;
    r->head->hdr.tail = 0u;
}

/* Check whether shared header matches this ring definition. */
static inline bool lat_ring_header_matches(const LatRing *r)
{
    return (r->head->hdr.abi_version == LAT_RING_ABI_VERSION) &&
           (r->head->hdr.slot_size == (uint16_t)LAT_RING_UNIT) &&
           (r->head->hdr.capacity == (uint16_t)r->units);
}

/* Approximate unread units (callable from either side). */
static inline uint32_t lat_ring_used_units(const LatRing *r)
{
    return lat_ring_load(&r->head->hdr.head) - lat_ring_load(&r->head->hdr.tail);
}

/* Free units on producer side. */
static inline uint32_t lat_ring_free_units(const LatRing *r)
{
    uint32_t used = lat_ring_used_units(r);

    return (used >= r->units) ? 0u : (r->units - used);
}

/* True if a record of len payload bytes fits now (producer side). */
static inline bool lat_ring_can_push(const LatRing *r, uint32_t len)
{
    return (len <= LAT_RING_PAYLOAD_MAX) && (lat_ring_free_units(r) >= lat_ring_record_units(len));
}

typedef enum LatRingFullAction
{
    LAT_RING_FULL_DROP = 0,     /* Drop and increment drop_count */
    LAT_RING_FULL_KEEP = 1,     /* Return false without counting (caller retries) */
    LAT_RING_FULL_OVERFLOW = 2  /* Increment overflow_count (safe ring fault) */
} LatRingFullAction;

static inline bool lat_ring_push_impl(const LatRing *r, uint8_t type, uint8_t flags,
                                      const void *payload, uint32_t len,
                                      LatRingFullAction on_full)
{
    LatRingHead *h = r->head;
    const uint8_t *src = (const uint8_t *)payload;
    const uint32_t byte_mask = (r->units * LAT_RING_UNIT) - 1u;
    LatRingRecHdr *rec;
    uint32_t need;
    uint32_t head;
    uint32_t tail;
    uint32_t used;
    uint32_t base;
    uint32_t i;

    if (len > LAT_RING_PAYLOAD_MAX)
    {
        h->reject_count++;
        return false;
    }
    need = lat_ring_record_units(len);

    head = lat_ring_load(&h->hdr.head);
    tail = lat_ring_load(&h->hdr.tail);
    /* Wait for consumer tail before reusing slots. */
    LAT_FENCE_RW();

    used = head - tail;
    if ((used > r->units) || ((r->units - used) < need))
    {
        uint32_t index = lat_ring_type_index(type);

        if (on_full == LAT_RING_FULL_DROP)
        {
            h->hdr.drop_count++;
            h->lost_by_type[index]++;
        }
        else if (on_full == LAT_RING_FULL_OVERFLOW)
        {
            h->hdr.overflow_count++;
            h->lost_by_type[index]++;
        }
        return false;
    }

    base = (head & (r->units - 1u)) * LAT_RING_UNIT;
    rec = (LatRingRecHdr *)(void *)&r->data[base];
    rec->len = (uint16_t)len;
    rec->type = type;
    rec->flags = flags;
    rec->seq = h->push_count;
    for (i = 0u; i < len; i++)
    {
        r->data[(base + LAT_RING_UNIT + i) & byte_mask] = src[i];
    }

    /* Publish head only after payload is written. */
    LAT_FENCE_RW();
    lat_ring_store(&h->hdr.head, head + need);
    LAT_FENCE_RW();
    h->push_count++;
    return true;
}

static inline bool lat_ring_push_lossy(const LatRing *r, uint8_t type, uint8_t flags,
                                       const void *payload, uint32_t len)
{
    return lat_ring_push_impl(r, type, flags, payload, len, LAT_RING_FULL_DROP);
}

static inline bool lat_ring_try_push(const LatRing *r, uint8_t type, uint8_t flags,
                                     const void *payload, uint32_t len)
{
    return lat_ring_push_impl(r, type, flags, payload, len, LAT_RING_FULL_KEEP);
}

static inline bool lat_ring_push_nowait(const LatRing *r, uint8_t type, uint8_t flags,
                                        const void *payload, uint32_t len)
{
    return lat_ring_push_impl(r, type, flags, payload, len, LAT_RING_FULL_OVERFLOW);
}

/*
 * lat_ring_pop: pop one record into *out (consumer side); returns false if empty.
 * Resyncs tail to head and increments resync_count on header/index corruption.
 */
static inline bool lat_ring_pop(const LatRing *r, LatRingRecord *out)
{
    LatRingHead *h = r->head;
    const uint32_t byte_mask = (r->units * LAT_RING_UNIT) - 1u;
    const LatRingRecHdr *rec;
    uint32_t head;
    uint32_t tail;
    uint32_t avail;
    uint32_t base;
    uint32_t len;
    uint32_t need;
    uint32_t i;

    tail = lat_ring_load(&h->hdr.tail);
    head = lat_ring_load(&h->hdr.head);
    /* Read head before reading payload. */
    LAT_FENCE_RW();

    if (head == tail)
    {
        return false;
    }

    avail = head - tail;
    base = (tail & (r->units - 1u)) * LAT_RING_UNIT;
    rec = (const LatRingRecHdr *)(const void *)&r->data[base];
    len = rec->len;
    need = lat_ring_record_units(len);
    if ((avail > r->units) || (len > LAT_RING_PAYLOAD_MAX) || (need > avail))
    {
        h->resync_count++;
        lat_ring_store(&h->hdr.tail, head);
        LAT_FENCE_RW();
        return false;
    }

    out->len = (uint16_t)len;
    out->type = rec->type;
    out->flags = rec->flags;
    out->seq = rec->seq;
    for (i = 0u; i < len; i++)
    {
        out->payload[i] = r->data[(base + LAT_RING_UNIT + i) & byte_mask];
    }

    /* Advance tail only after payload is read. */
    LAT_FENCE_RW();
    lat_ring_store(&h->hdr.tail, tail + need);
    LAT_FENCE_RW();
    h->pop_count++;
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* LAT_RING_H */
