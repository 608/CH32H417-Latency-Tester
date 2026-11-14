/*
 * usbfs_dev.h - Shared USBFS device driver (V3F only, IRQ-driven).
 *
 * 48 MHz clock comes from USBHS PLL (480 MHz) / 10; pins PA11=DM, PA12=DP.
 * Uses software data toggle (AUTO_TOG disabled); DMA buffers live in V3F .bss
 * (4-byte aligned). EP1..7 are unidirectional (IN or OUT only, max 64 bytes).
 */
#ifndef LAT_USBFS_DEV_H
#define LAT_USBFS_DEV_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USBFS_EP0_SIZE        (64u)
#define USBFS_EP_PACKET_MAX   (64u)
#define USBFS_EP_NUM          (8u)

/* Standard request constants (USB 2.0 section 9.4). */
#define USBFS_REQ_DIR_IN      ((uint8_t)0x80u)
#define USBFS_REQ_TYPE_MASK   ((uint8_t)0x60u)
#define USBFS_REQ_TYPE_STD    ((uint8_t)0x00u)
#define USBFS_REQ_TYPE_CLASS  ((uint8_t)0x20u)
#define USBFS_REQ_TYPE_VENDOR ((uint8_t)0x40u)
#define USBFS_REQ_RECIP_MASK  ((uint8_t)0x1Fu)
#define USBFS_REQ_RECIP_DEV   ((uint8_t)0x00u)
#define USBFS_REQ_RECIP_IF    ((uint8_t)0x01u)
#define USBFS_REQ_RECIP_EP    ((uint8_t)0x02u)

#define USBFS_DESC_DEVICE     ((uint8_t)0x01u)
#define USBFS_DESC_CONFIG     ((uint8_t)0x02u)
#define USBFS_DESC_STRING     ((uint8_t)0x03u)
#define USBFS_DESC_BOS        ((uint8_t)0x0Fu)

typedef struct UsbfsSetup
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} UsbfsSetup;

/*
 * Control transfer reply for class/vendor hooks:
 *   IN:  set in_data/in_len (kept valid until transfer completes).
 *   OUT: set out_buf/out_cap (STALLs if out_cap < wLength); ctrl_out_done fires.
 *   No-data: return true with both unset to send zero-length status IN.
 */
typedef struct UsbfsCtrlReply
{
    const uint8_t *in_data;
    uint16_t in_len;
    uint8_t *out_buf;
    uint16_t out_cap;
} UsbfsCtrlReply;

/*
 * Read-only diagnostic snapshot (usbfs_dev_diag).
 */
typedef struct UsbfsDiag
{
    uint32_t isr;          /* USBFS_IRQHandler entries */
    uint32_t xfer_setup;   /* SETUP tokens */
    uint32_t xfer_in;      /* IN tokens */
    uint32_t xfer_out;     /* OUT tokens */
    uint32_t xfer_other;   /* Other TRANSFER tokens */
    uint32_t bus_reset;    /* Bus resets */
    uint32_t suspend;      /* Suspend/resume IRQs */
    uint32_t stall;        /* ep0_stall() calls */
    uint32_t ep0_in_sent;  /* EP0 IN packets queued */
    uint32_t set_address;  /* SET_ADDRESS applied */
    uint8_t last_int_fg;   /* Last TRANSFER IRQ INT_FG */
    uint8_t last_int_st;   /* Last TRANSFER IRQ INT_ST */
    uint8_t mis_st;        /* MIS_ST (live read) */
    uint8_t udev_ctrl;     /* UDEV_CTRL (live read) */
    uint8_t base_ctrl;     /* BASE_CTRL (live read) */
    uint8_t dev_addr;      /* DEV_ADDR (live read) */
    uint8_t ep0_tctrl;     /* UEP0_TX_CTRL (live read) */
    uint8_t ep0_rctrl;     /* UEP0_RX_CTRL (live read) */
    uint8_t ep0_tlen;      /* UEP0_TX_LEN (live read) */
    uint8_t last_setup[8]; /* Last 8-byte SETUP packet */
    uint16_t last_rx_len;  /* Last TRANSFER IRQ RX_LEN */
} UsbfsDiag;

typedef struct UsbfsDevClass
{
    /* Descriptors (lengths read from descriptor headers; bos=NULL STALLs). */
    const uint8_t *device_desc;
    const uint8_t *config_desc;
    const uint8_t *bos_desc;
    const uint8_t *const *strings;
    uint8_t string_count;

    /* Endpoint bitmasks (bit n = EPn, n = 1..7); IN and OUT must not overlap. */
    uint8_t ep_in_mask;
    uint8_t ep_out_mask;

    /* Hooks called from ISR (NULL if unused). */
    /* Class request; return false to STALL. */
    bool (*class_setup)(const UsbfsSetup *setup, UsbfsCtrlReply *reply);
    /* Vendor request (e.g. MS OS 2.0 descriptors); return false to STALL. */
    bool (*vendor_setup)(const UsbfsSetup *setup, UsbfsCtrlReply *reply);
    /* EP0 OUT data stage completed. */
    void (*ctrl_out_done)(const UsbfsSetup *setup, const uint8_t *data, uint16_t len);
    /* SET_CONFIGURATION (0 = unconfigured; also called with 0 on bus reset). */
    void (*configured)(uint8_t config_value);
    /* OUT data received (TOG_OK only); return false to NAK EP until resumed. */
    bool (*ep_out)(uint8_t ep, const uint8_t *data, uint16_t len);
    /* IN transfer ACKed by host. */
    void (*ep_in_done)(uint8_t ep);
} UsbfsDevClass;

/*
 * Prepare USBFS clock; starts USBHS PLL if not already running (bounded wait).
 */
bool usbfs_dev_clock_init(void);

/* Initialize device, enable pull-up, and enable USBFS_IRQn. */
void usbfs_dev_start(const UsbfsDevClass *cls, uint8_t irq_priority);

/*
 * Stop device (before bootloader/reset): disable IRQ, reset SIE, drop pull-up.
 * Caller should wait a few ms for host disconnect before resetting.
 */
void usbfs_dev_stop(void);

/*
 * Mask USBFS_IRQn around shared main-loop/ISR state access (non-nestable).
 */
void usbfs_dev_lock(void);
void usbfs_dev_unlock(void);

/* True if configured (SET_CONFIGURATION != 0). */
bool usbfs_dev_configured(void);

/* Bus reset counter. */
uint32_t usbfs_dev_reset_count(void);

/*
 * EP1..7 reset counter (bus reset, SET_CONFIGURATION, ENDPOINT_HALT).
 * When this changes, any in-flight IN was cancelled without completion hook.
 */
uint32_t usbfs_dev_ep_reset_count(void);

/* True if IN endpoint is currently transmitting. */
bool usbfs_ep_in_busy(uint8_t ep);

/* True if IN endpoint is configured, idle, and not stalled. */
bool usbfs_ep_in_ready(uint8_t ep);

/*
 * Start IN transfer of len bytes (0..64); returns false if busy/stalled/invalid.
 */
bool usbfs_ep_in_send(uint8_t ep, const uint8_t *data, uint16_t len);

/* Resume an OUT endpoint paused by ep_out returning false. */
void usbfs_ep_out_resume(uint8_t ep);

/* True if OUT endpoint is paused. */
bool usbfs_ep_out_paused(uint8_t ep);

/* Read diagnostic snapshot (safe before usbfs_dev_start). */
void usbfs_dev_diag(UsbfsDiag *out);

/*
 * Build UTF-16LE string descriptor from ASCII; returns bLength.
 */
uint8_t usbfs_make_string_desc(uint8_t *out, uint32_t cap, const char *ascii);

/* Build uppercase hex string descriptor from raw bytes (for serial number). */
uint8_t usbfs_make_hex_string_desc(uint8_t *out, uint32_t cap, const uint8_t *bytes, uint32_t n);

#ifdef __cplusplus
}
#endif

#endif /* LAT_USBFS_DEV_H */
