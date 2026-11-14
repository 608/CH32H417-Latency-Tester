/*
 * lat_uart.h - UART debug log (USART6 TX, PC10, AF8, 115200 bps 8N1).
 *
 * Used by BL and V3F (PA12 is shared with USBFS DP). PC10 is in VIO18 domain;
 * short EVT P9 pins 9-10 for 3.3V. Output is ASCII only.
 */
#ifndef LAT_UART_H
#define LAT_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LAT_UART_BAUD_RATE 115200u

/* Initialize USART6 and PC10 (enables USART6, GPIOC, AFIO clocks). */
void lat_uart_init(void);

/* Blocking single-char transmit. */
void lat_uart_putc_blocking(char c);

/* Blocking string transmit; waits for TC after the last char. */
void lat_uart_puts_blocking(const char *s);

/* Non-blocking single-char transmit; returns false if TXE is not set. */
bool lat_uart_try_putc(char c);

/* Wait for transmission complete (TC). */
void lat_uart_wait_tc(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_UART_H */
