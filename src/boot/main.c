/*
 * CH32H417 latency tester - bootloader.
 *
 * Boot decision (in priority order; boot magic at 0x2017FFF8 is always cleared):
 *   1. Boot magic == STAY                        -> stay in BL
 *   2. Boot magic == APP_IMMEDIATE && app valid  -> jump to app (skip PB6)
 *   3. PB6 low (jumper to P5 KEY)                -> stay in BL
 *   4. App invalid                               -> stay in BL
 *   5. Otherwise                                 -> jump to app
 *
 * App jump sequence:
 *   1. Clear all PFIC external interrupt enables and pending flags.
 *   2. Set FLASH->ACTLR for HCLK/2.
 *   3. Switch SYSCLK to HSI and disable PLL so app startup can reconfigure it.
 *   4. Enable and pend Software_IRQn; SW_Handler disables IRQs in mstatus,
 *      executes fence.i, and branches to 0x10000 (main runs in U-mode).
 *
 * BL runs on V3F only without starting V5F or modifying IALLOCR.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ch32h417.h"

#include "app_image_check.h"
#include "app_metadata.h"
#include "bl_flash.h"
#include "bl_proto.h"
#include "bl_time.h"
#include "bl_usb.h"
#include "boot_magic.h"
#include "lat_fence.h"
#include "lat_fmt.h"
#include "lat_uart.h"
#include "usbfs_dev.h"

/* Stay-in-BL input: PB6 (P5 pin 4), internal pull-up, active low. */
#define BL_BOOT_PIN_PORT   GPIOB
#define BL_BOOT_PIN        GPIO_Pin_6
#define BL_BOOT_PIN_CLOCK  RCC_HB2Periph_GPIOB

/* ESIG chip-unique ID (8 bytes from 0x1FFFF7E8). */
#define BL_ESIG_UNIID_ADDR ((uintptr_t)0x1FFFF7E8u)

#define BL_STATUS_LOG_MS   (10000u)

static void bl_puts(const char *s)
{
    lat_uart_puts_blocking(s);
}

static void bl_put_line2(const char *a, const char *b)
{
    bl_puts(a);
    bl_puts(b);
    bl_puts("\r\n");
}

static void boot_pin_init(void)
{
    GPIO_InitTypeDef gpio_cfg = {0};

    RCC_HB2PeriphClockCmd(BL_BOOT_PIN_CLOCK, ENABLE);
    gpio_cfg.GPIO_Pin = BL_BOOT_PIN;
    gpio_cfg.GPIO_Speed = GPIO_Speed_Low;
    gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BL_BOOT_PIN_PORT, &gpio_cfg);
}

static bool boot_pin_is_low(void)
{
    return (BL_BOOT_PIN_PORT->INDR & BL_BOOT_PIN) == 0u;
}

/* Verify application image; returns true if valid. */
static bool app_validate(app_metadata_t *meta, const char **reason)
{
    const char *why = bl_flash_check_app(meta);

    if (why != NULL)
    {
        *reason = why;
        return false;
    }
    return true;
}

void bl_port_uuid(uint8_t out[16])
{
    const volatile uint32_t *uid = (const volatile uint32_t *)BL_ESIG_UNIID_ADDR;
    uint32_t w;
    uint32_t b;

    for (b = 0u; b < 16u; b++)
    {
        out[b] = 0u;
    }
    for (w = 0u; w < 2u; w++)
    {
        uint32_t value = uid[w];

        for (b = 0u; b < 4u; b++)
        {
            out[(w * 4u) + b] = (uint8_t)(value >> (8u * b));
        }
    }
}

void SW_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SW_Handler(void)
{
    NVIC_DisableIRQ(Software_IRQn);
    NVIC_ClearPendingIRQ(Software_IRQn);

    /* Disable IRQs in M-mode (MIE=0), sync instruction fetch, and jump to app. */
    __asm volatile(
        "csrw mstatus, %0\n"
        "fence.i\n"
        :
        : "r"(0x6080u)
        : "memory");

    __asm volatile(
        "li a6, %0\n"
        "jr a6\n"
        :
        : "i"(APP_EXEC_BASE)
        : "a6");

    for (;;)
    {
    }
}

static void jump_to_application(void) __attribute__((noreturn));
static void jump_to_application(void)
{
    uint32_t actlr;
    uint32_t i;

    /* Restore SysTick0 to stopped state before entering app. */
    bl_time_stop();

    /* 1. Clear all external interrupt enables and pending flags (IRQ >= 16). */
    PFIC->IRER[0] = 0xFFFF0000u;
    PFIC->IPRR[0] = 0xFFFF0000u;
    for (i = 1u; i < 8u; i++)
    {
        PFIC->IRER[i] = 0xFFFFFFFFu;
        PFIC->IPRR[i] = 0xFFFFFFFFu;
    }

    /* 2. Match app SystemInit() flash latency before fetching >= 0x10000. */
    actlr = FLASH->ACTLR;
    actlr &= ~((uint32_t)0x3u);
    actlr |= FLASH_ACTLR_LATENCY_HCLK_DIV2;
    FLASH->ACTLR = actlr;

    /* 3. Switch to HSI and stop PLL so app startup can reconfigure it. */
    RCC->CFGR0 = 0u;
    while ((RCC->CFGR0 & RCC_SWS) != 0u)
    {
    }
    RCC->PLLCFGR &= ~RCC_SYSPLL_GATE;
    RCC->CTLR &= ~RCC_PLLON;
    while ((RCC->CTLR & RCC_PLLRDY) != 0u)
    {
    }
    RCC->PLLCFGR = 0u;

    /* 4. Enable and pend Software_IRQn (transfers to SW_Handler). */
    NVIC_EnableIRQ(Software_IRQn);
    NVIC_SetPendingIRQ(Software_IRQn);

    for (;;)
    {
        /* Unreachable: SW_Handler branches to app. */
    }
}

/* After RESET_APP: detach USB, write APP_IMMEDIATE, and reset. */
static void reset_to_application(void) __attribute__((noreturn));
static void reset_to_application(void)
{
    bl_puts("[bl] RESET_APP: stopping USB, boot magic APP_IMMEDIATE, system reset\r\n");
    usbfs_dev_stop();
    /* Wait for host to detect disconnect after pull-up removal. */
    bl_delay_ms(20u);
    boot_magic_set(LAT_BOOT_MAGIC_APP_IMMEDIATE);
    LAT_FENCE_IORW();
    NVIC_SystemReset();
    for (;;)
    {
    }
}

static const char *cmd_name(uint8_t cmd)
{
    switch (cmd)
    {
    case BL_CMD_INFO:      return "INFO";
    case BL_CMD_ERASE:     return "ERASE";
    case BL_CMD_WRITE:     return "WRITE";
    case BL_CMD_FINALIZE:  return "FINALIZE";
    case BL_CMD_RESET_APP: return "RESET_APP";
    case BL_CMD_READ_UUID: return "READ_UUID";
    default:               return "UNKNOWN";
    }
}

static const char *status_name(uint8_t st)
{
    switch (st)
    {
    case BL_STATUS_OK:              return "OK";
    case BL_STATUS_RANGE:           return "RANGE";
    case BL_STATUS_ALIGN:           return "ALIGN";
    case BL_STATUS_SEQUENCE:        return "SEQUENCE";
    case BL_STATUS_NO_APP:          return "NO_APP";
    case BL_STATUS_FLASH_FAIL:      return "FLASH_FAIL";
    case BL_STATUS_BUSY:            return "BUSY";
    case BL_STATUS_CRC_FAIL:        return "CRC_FAIL";
    case BL_STATUS_TIMEOUT:         return "TIMEOUT";
    case BL_STATUS_UNKNOWN_COMMAND: return "UNKNOWN_COMMAND";
    default:                        return "?";
    }
}

/* Log processed request over UART; log successful WRITEs every 64 KB only. */
static void log_command(uint8_t cmd, uint8_t st)
{
    static uint32_t last_progress;
    uint32_t next = bl_flash_next_address();
    char buf[96];
    LatStr s;

    if ((cmd == BL_CMD_WRITE) && (st == BL_STATUS_OK))
    {
        if ((next - BL_APP_BASE) < (last_progress + 0x10000u))
        {
            return;
        }
        last_progress = (next - BL_APP_BASE) & ~0xFFFFu;
    }
    if (cmd == BL_CMD_ERASE)
    {
        last_progress = 0u;
    }
    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, "[bl] ");
    lat_str_puts(&s, cmd_name(cmd));
    lat_str_puts(&s, " ");
    lat_str_puts(&s, status_name(st));
    if ((cmd == BL_CMD_ERASE) || (cmd == BL_CMD_WRITE) || (cmd == BL_CMD_FINALIZE))
    {
        lat_str_puts(&s, " next=0x");
        lat_str_hex32(&s, next);
    }
    lat_str_puts(&s, "\r\n");
    bl_puts(buf);
}

static void usb_update_mode(const char *stay_reason) __attribute__((noreturn));
static void usb_update_mode(const char *stay_reason)
{
    uint8_t uuid[16];
    uint32_t last_log;
    bool was_configured = false;

    if ((PFIC->IALLOCR[(uint32_t)USBFS_IRQn] & 0x01u) != 0u)
    {
        bl_puts("[bl] WARN USBFS_IRQn is allocated to V5F (IALLOCR bit0=1) - USB interrupts will not reach V3F\r\n");
    }
    if (!usbfs_dev_clock_init())
    {
        for (;;)
        {
            bl_put_line2("[bl] staying in bootloader: ", stay_reason);
            bl_puts("[bl] ERROR USBHS PLL did not lock - USB update is not available\r\n");
            bl_delay_ms(2000u);
        }
    }
    bl_proto_init();
    bl_port_uuid(uuid);
    bl_usb_start(uuid);
    bl_puts("[bl] USB update mode ready (16C0:05DC, WinUSB, bulk OUT 0x01 / IN 0x82, 64-byte frames)\r\n");

    last_log = bl_ticks();
    for (;;)
    {
        uint8_t cmd;
        uint8_t st;
        bool configured = bl_usb_configured();

        if (bl_usb_poll(&cmd, &st))
        {
            log_command(cmd, st);
        }
        if (bl_usb_reset_ready())
        {
            reset_to_application();
        }
        if (configured != was_configured)
        {
            was_configured = configured;
            bl_puts(configured ? "[bl] usb configured\r\n" : "[bl] usb not configured\r\n");
        }
        if (bl_elapsed_us(last_log) >= (BL_STATUS_LOG_MS * 1000u))
        {
            last_log = bl_ticks();
            bl_put_line2("[bl] waiting for USB update, stay reason: ", stay_reason);
        }
    }
}

int main(void)
{
    app_metadata_t meta = {0};
    const char *reason = "not evaluated";
    const char *stay_reason = NULL;
    uint32_t magic;
    bool pin_low;

    SystemInit();
    SystemAndCoreClockUpdate();
    bl_time_init();

    lat_uart_init();
    boot_pin_init();
    bl_flash_init();

    bl_puts("\r\n[bl] CH32H417 latency tester bootloader 1.0\r\n");

    magic = boot_magic_consume();
    /* Allow internal pull-up to settle before sampling PB6. */
    bl_delay_ms(1u);
    pin_low = boot_pin_is_low();

    if (magic == LAT_BOOT_MAGIC_STAY)
    {
        stay_reason = "boot magic STAY";
    }
    else if ((magic == LAT_BOOT_MAGIC_APP_IMMEDIATE) && app_validate(&meta, &reason))
    {
        stay_reason = NULL;
    }
    else if (pin_low)
    {
        stay_reason = "PB6 is low (bootloader entry input)";
    }
    else if (!app_validate(&meta, &reason))
    {
        stay_reason = reason;
    }

    if (stay_reason == NULL)
    {
        char buf[128];
        LatStr s;

        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[bl] app ok size=0x");
        lat_str_hex32(&s, meta.image_size);
        lat_str_puts(&s, " crc=0x");
        lat_str_hex32(&s, meta.crc32);
        lat_str_puts(&s, " fw=");
        lat_str_u32(&s, meta.fw_major);
        lat_str_putc(&s, '.');
        lat_str_u32(&s, meta.fw_minor);
        lat_str_puts(&s, (magic == LAT_BOOT_MAGIC_APP_IMMEDIATE) ? " (magic APP_IMMEDIATE)" : "");
        lat_str_puts(&s, " - jumping to 0x00010000\r\n");
        bl_puts(buf);
        jump_to_application();
    }

    bl_put_line2("[bl] staying in bootloader: ", stay_reason);
    usb_update_mode(stay_reason);
}
