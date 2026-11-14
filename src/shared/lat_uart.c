/*
 * lat_uart.c - UART debug log (USART6 TX, PC10, AF8). See lat_uart.h.
 *
 * USART6_TX AF depends on pin (PA12=AF6, PC10=AF8).
 */
#include "lat_uart.h"

#include "ch32h417.h"

void lat_uart_init(void)
{
    GPIO_InitTypeDef gpio_cfg = {0};
    USART_InitTypeDef usart_cfg = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART6, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOC, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF8);

    gpio_cfg.GPIO_Pin = GPIO_Pin_10;
    gpio_cfg.GPIO_Speed = GPIO_Speed_Very_High;
    gpio_cfg.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio_cfg);

    usart_cfg.USART_BaudRate = LAT_UART_BAUD_RATE;
    usart_cfg.USART_WordLength = USART_WordLength_8b;
    usart_cfg.USART_StopBits = USART_StopBits_1;
    usart_cfg.USART_Parity = USART_Parity_No;
    usart_cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_cfg.USART_Mode = USART_Mode_Tx;
    USART_Init(USART6, &usart_cfg);
    USART_Cmd(USART6, ENABLE);
}

void lat_uart_putc_blocking(char c)
{
    while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART6, (uint16_t)(uint8_t)c);
}

void lat_uart_puts_blocking(const char *s)
{
    while (*s != '\0')
    {
        lat_uart_putc_blocking(*s++);
    }
    lat_uart_wait_tc();
}

bool lat_uart_try_putc(char c)
{
    if (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
    {
        return false;
    }
    USART_SendData(USART6, (uint16_t)(uint8_t)c);
    return true;
}

void lat_uart_wait_tc(void)
{
    while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
    {
    }
}
