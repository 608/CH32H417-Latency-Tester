/*
 * usbhs_host_core.h - Boundary between USBHS host core and hardware
 *
 * The core (usbhs_host_core.c) does not touch registers directly.
 * Single-owner issue rule:
 *   Only usbhs_hw_issue() inside usbhs_start_if_idle() writes CONTROL/DMA TX,
 *   except usbhs_hw_abort() on detach/recovery. Both TIM9 CH2 ISR and USBHS
 *   ISR can call usbhs_start_if_idle(); s_busy atomic swap prevents double
 *   issue, and a double-buffered schedule slot word is swapped atomically.
 */
#ifndef LAT_USBHS_HOST_CORE_H
#define LAT_USBHS_HOST_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "usbhs_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Register constants (matching ch32h417_usb.h so core stays SDK-independent). */
#define USBHS_PID_OUT             ((uint8_t)0x01u)
#define USBHS_PID_ACK             ((uint8_t)0x02u)
#define USBHS_PID_DATA0           ((uint8_t)0x03u)
#define USBHS_PID_NYET            ((uint8_t)0x06u)
#define USBHS_PID_IN              ((uint8_t)0x09u)
#define USBHS_PID_NAK             ((uint8_t)0x0Au)
#define USBHS_PID_DATA1           ((uint8_t)0x0Bu)
#define USBHS_PID_SETUP           ((uint8_t)0x0Du)
#define USBHS_PID_STALL           ((uint8_t)0x0Eu)

#define USBHS_IF_SOF_ACT          ((uint8_t)0x20u) /* USBHS_UHIF_SOF_ACT */
#define USBHS_IF_TRANSFER         ((uint8_t)0x10u) /* USBHS_UHIF_TRANSFER */
#define USBHS_PC_RESET            ((uint8_t)0x10u) /* USBHS_UHIF_PORT_RESET */
#define USBHS_PC_CONNECT          ((uint8_t)0x01u) /* USBHS_UHIF_PORT_CONNECT */
#define USBHS_PS_SPEED_MASK       ((uint16_t)0x0600u)
#define USBHS_PS_HS               ((uint16_t)0x0400u)
#define USBHS_PS_LS               ((uint16_t)0x0200u)
#define USBHS_PS_RST              ((uint16_t)0x0010u)
#define USBHS_PS_CONNECT          ((uint16_t)0x0001u)
#define USBHS_CFG_SOF_EN          ((uint8_t)0x20u)
#define USBHS_CTRL_HOST_ACTION    ((uint32_t)0x00010000u)

/* Snapshot captured at USBHS ISR entry. */
typedef struct UsbhsSnap
{
    uint64_t now;         /* tim9_now() */
    uint32_t frame;       /* FRAME */
    uint16_t port_status; /* PORT_STATUS */
    uint16_t rx_len;      /* RX_LEN low 11 bits */
    uint8_t  int_flag;    /* INT_FLAG (already cleared) */
    uint8_t  port_chg;    /* PORT_STATUS_CHG (already cleared) */
    uint8_t  int_st;      /* INT_ST (response PID) */
    uint8_t  cfg;         /* CFG (read only when port_chg != 0) */
} UsbhsSnap;

/* Core entry points */
void usbhs_core_init(uint64_t now);
/* Handle a device already attached at boot (call before enabling IRQs). */
void usbhs_core_boot_port(uint16_t port_status, uint64_t now);
void usbhs_core_isr(const UsbhsSnap *sn);
void usbhs_core_tim9_ch2(void);
void usbhs_core_poll(uint64_t now);
/* src: 0 = USBHS ISR / main loop, 1 = TIM9 ISR */
bool usbhs_start_if_idle(uint8_t src);

/* Hardware callbacks */
uint32_t usbhs_hw_cnt(void);                /* TIM9->CNT_32 */
uint64_t usbhs_hw_now(void);                /* tim9_now() */
/* Issue one transaction; returns TIM9->CNT_32 read just before writing CONTROL. */
uint32_t usbhs_hw_issue(uint8_t addr, uint8_t token, uint8_t ep, uint8_t toggle,
                        uint16_t len, const uint8_t *tx);
const uint8_t *usbhs_hw_rx(void);           /* DMA RX buffer */
uint8_t  usbhs_hw_int_flag(void);
uint32_t usbhs_hw_control(void);
void     usbhs_hw_clear_transfer(void);     /* INT_FLAG = TRANSFER */
void     usbhs_hw_abort(void);              /* Clear HOST_ACTION */
void     usbhs_hw_set_compare(uint32_t t);  /* TIM9->CH2CVR_32 */
void     usbhs_hw_port_reset(uint8_t long_reset);
void     usbhs_hw_sof_enable(void);         /* CFG |= SOF_EN */
uint16_t usbhs_hw_port_status(void);
void     usbhs_hw_irq_off(void);
bool     usbhs_hw_reinit(void);
void     usbhs_hw_clear_flags(void);
void     usbhs_hw_enable_sources(void);
void     usbhs_hw_irq_on(bool usbhs);
void     usbhs_hw_lock(void);
void     usbhs_hw_unlock(void);
void     usbhs_hw_kick(void);
void     usbhs_hw_notify(uint8_t pc_type, uint32_t word, uint8_t arg0, uint8_t arg1, uint16_t code);

#ifdef __cplusplus
}
#endif

#endif /* LAT_USBHS_HOST_CORE_H */
