/*
 * console.h - Non-blocking UART TX ring buffer for V3F (not interrupt-safe).
 */
#ifndef LAT_CON_H
#define LAT_CON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CON_TX_BUF_SIZE ((uint32_t)8192u) /* power of two */

/* Enqueue string; drops the whole string if insufficient space. */
void con_puts(const char *s);

/* Flush as much as UART TXDR accepts without blocking. */
void con_poll(void);

/* Block until buffer is empty and last byte finishes transmitting. */
void con_flush(void);

/* Free space in bytes. */
uint32_t con_free(void);

/* Number of dropped strings. */
uint32_t con_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_CON_H */
