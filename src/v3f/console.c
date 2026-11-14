/*
 * console.c - Non-blocking UART TX ring buffer for V3F.
 */
#include "console.h"

#include "lat_uart.h"

_Static_assert((CON_TX_BUF_SIZE & (CON_TX_BUF_SIZE - 1u)) == 0u, "CON_TX_BUF_SIZE must be a power of two");

static char s_buf[CON_TX_BUF_SIZE];
static uint32_t s_head; /* total bytes written */
static uint32_t s_tail; /* total bytes sent */
static uint32_t s_drop;

uint32_t con_free(void)
{
    return CON_TX_BUF_SIZE - (s_head - s_tail);
}

uint32_t con_drop_count(void)
{
    return s_drop;
}

void con_puts(const char *s)
{
    uint32_t len = 0u;
    uint32_t i;

    while (s[len] != '\0')
    {
        len++;
    }
    if (len > con_free())
    {
        s_drop++;
        return;
    }
    for (i = 0u; i < len; i++)
    {
        s_buf[(s_head + i) & (CON_TX_BUF_SIZE - 1u)] = s[i];
    }
    s_head += len;
}

void con_poll(void)
{
    while (s_tail != s_head)
    {
        if (!lat_uart_try_putc(s_buf[s_tail & (CON_TX_BUF_SIZE - 1u)]))
        {
            return;
        }
        s_tail++;
    }
}

void con_flush(void)
{
    while (s_tail != s_head)
    {
        lat_uart_putc_blocking(s_buf[s_tail & (CON_TX_BUF_SIZE - 1u)]);
        s_tail++;
    }
    lat_uart_wait_tc();
}
