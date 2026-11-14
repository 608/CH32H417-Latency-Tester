/*
 * meas_poll.h - Polling interval measurement aggregation
 *
 * Aggregation (meas_poll_data/nak/error) is called from USBHS ISR.
 * Main loop must set active=0 (meas_poll_stop) before reading results.
 */
#ifndef LAT_MEAS_POLL_H
#define LAT_MEAS_POLL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MEAS_POLL_BIN_WIDTH     (100u)          /* 1us */
#define MEAS_POLL_BINS_MAX      (16384u)
#define MEAS_POLL_LONG_MAX      (4096u)
#define MEAS_POLL_LONG_ENTRY    (12u)           /* Packed storage format (exported as 16 bytes) */
#define MEAS_POLL_50MS          ((uint64_t)5000000u)
#define MEAS_POLL_RESULT_LEN    (88u)
#define MEAS_POLL_START_LEN     (16u)

typedef struct MeasPoll
{
    volatile uint8_t active;
    uint8_t  keep_raw;
    uint8_t  raw_truncated;
    uint8_t  long_truncated;
    uint32_t bin_count;
    uint32_t long_threshold;
    uint32_t raw_max;         /* Upper limit requested by PC */
    uint32_t raw_cap;         /* Actual capacity (<= raw_max) */
    uint32_t data_count;
    uint32_t nak_count;
    uint32_t error_count;
    uint32_t interval_count;
    uint32_t overflow_count;
    uint32_t count_below_50ms;
    uint32_t long_count;
    uint32_t raw_count;
    uint64_t min_interval;
    uint64_t max_interval;
    uint64_t sum_below_50ms;
    uint64_t first_data_time;
    uint64_t last_t1;
    uint32_t *hist;           /* bin_count entries */
    uint32_t *raw;            /* raw_cap entries */
    uint8_t  *longs;          /* MEAS_POLL_LONG_MAX * MEAS_POLL_LONG_ENTRY bytes */
} MeasPoll;

uint32_t meas_poll_bin_count(uint32_t period_ticks);
uint32_t meas_poll_long_threshold(uint32_t period_ticks);

/*
 * Start aggregation. pool[0..bin_count-1] is used for histogram, remainder for raw data.
 * longs must be MEAS_POLL_LONG_MAX * MEAS_POLL_LONG_ENTRY bytes.
 */
void meas_poll_begin(MeasPoll *s, uint32_t period_ticks, bool keep_raw, uint32_t raw_max, uint32_t *pool,
                     uint32_t pool_words, uint8_t *longs);
void meas_poll_data(MeasPoll *s, uint64_t t1);
void meas_poll_nak(MeasPoll *s);
void meas_poll_error(MeasPoll *s);
void meas_poll_stop(MeasPoll *s);

/* Build 88-byte POLL_RESULT payload. */
void meas_poll_result(const MeasPoll *s, uint32_t run_id, uint8_t reason, uint32_t duration_ms, uint8_t *out);
/*
 * Export GET_OBJECT kind 7 (raw), 8 (histogram), or 9 (long intervals) to dst.
 * Returns byte length, or 0xFFFFFFFF if cap is insufficient.
 */
uint32_t meas_poll_export(const MeasPoll *s, uint8_t kind, uint8_t *dst, uint32_t cap);
void meas_poll_long_get(const MeasPoll *s, uint32_t i, uint64_t *end, uint64_t *interval);

#ifdef __cplusplus
}
#endif

#endif /* LAT_MEAS_POLL_H */
