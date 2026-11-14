/*
 * meas_watch.h - SET_WATCH spec parsing and evaluation
 *
 * Hardware-independent; called from USBHS ISR, so must not block or copy large buffers.
 */
#ifndef LAT_MEAS_WATCH_H
#define LAT_MEAS_WATCH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MEAS_WATCH_LEN        (32u)
#define MEAS_WATCH_VALUES     (4u)
#define MEAS_REPORT_MAX       (1024u)

/* Evaluation result */
#define MEAS_ST_NONE          ((uint8_t)0u) /* Not evaluated (too short, wrong report ID, or out of bounds) */
#define MEAS_ST_RELEASED      ((uint8_t)1u)
#define MEAS_ST_PRESSED       ((uint8_t)2u)
#define MEAS_ST_OTHER         ((uint8_t)3u) /* Axis deflected to opposite side */

#define MEAS_WATCH_KIND_SET   ((uint8_t)0u)
#define MEAS_WATCH_KIND_AXIS  ((uint8_t)1u)

typedef struct MeasWatch
{
    uint8_t  iface;
    uint8_t  ep_in;
    uint8_t  report_id_mode;   /* 0: no condition, 1: first byte must match report_id */
    uint8_t  report_id;
    uint16_t min_length;
    uint8_t  kind;             /* MEAS_WATCH_KIND_* */
    uint8_t  is_signed;
    uint16_t bit_offset;
    uint8_t  bit_size;         /* 1..32 */
    uint8_t  value_count;      /* kind 0: 1..4 */
    int8_t   direction;        /* kind 1: +1, -1, 0 */
    uint8_t  reserved[3];
    uint32_t pressed_values[MEAS_WATCH_VALUES];
    int32_t  neutral;
    uint32_t threshold;
} MeasWatch;

/* Parse 32-byte SET_WATCH payload; returns false if out of range. */
bool meas_watch_parse(const uint8_t *p, uint32_t n, MeasWatch *w);

/* Extract bit_size bits from bit_offset (with sign extension if is_signed). Caller must check bounds. */
uint32_t meas_watch_extract(const uint8_t *data, uint16_t bit_offset, uint8_t bit_size, uint8_t is_signed);

uint8_t meas_watch_eval(const MeasWatch *w, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* LAT_MEAS_WATCH_H */
