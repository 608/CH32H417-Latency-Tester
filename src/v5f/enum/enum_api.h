/*
 * enum_api.h - Public interface exposed by enumeration to measurement
 *
 * Context:
 *   - enum_* functions must be called from the main loop (not from ISR).
 *   - enum_set_isr_hook callbacks run in USBHS ISR context.
 *   - enum_set_report_sink callbacks run in main-loop context (inside enum_poll).
 *
 * Lifetime:
 *   - EnumDevice and its report_desc pointers remain valid until disconnect or
 *     re-enumeration (ready becomes 0 and enum_seq changes).
 *   - ISR hooks are cleared on disconnect or re-enumeration; re-register after DEVICE_READY.
 *   - Report sink is preserved across disconnects; use generation and enum_seq() to filter stale reports.
 *
 * Keep backward compatibility when extending (append new fields at the end).
 */
#ifndef LAT_ENUM_API_H
#define LAT_ENUM_API_H

#include <stdbool.h>
#include <stdint.h>

#include "usbhs_host.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENUM_IFACE_MAX            (8u)   /* Matches GET_DEVICE_INFO iface_count limit */

/* Device classification */
#define ENUM_CLASS_OTHER          ((uint8_t)0u)
#define ENUM_CLASS_COMPOSITE      ((uint8_t)1u)
#define ENUM_CLASS_SINGLE_HID     ((uint8_t)2u)
#define ENUM_CLASS_SINGLE_XUSB    ((uint8_t)3u)
#define ENUM_CLASS_GIP            ((uint8_t)4u)

/* Detected protocol */
#define ENUM_PROTO_UNKNOWN        ((uint8_t)0u)
#define ENUM_PROTO_GENERIC_HID    ((uint8_t)1u)
#define ENUM_PROTO_DS4            ((uint8_t)2u)  /* 054C:05C4/09CC/0BA0/05C5 */
#define ENUM_PROTO_DUALSENSE      ((uint8_t)3u)  /* 054C:0CE6/0DF2/0E5F */
#define ENUM_PROTO_SWITCH_PRO     ((uint8_t)4u)  /* 057E:2009 */
#define ENUM_PROTO_XUSB           ((uint8_t)5u)  /* FF/5D/01 */
#define ENUM_PROTO_GIP            ((uint8_t)6u)  /* FF/47/D0 */

/* Function driver per interface */
#define ENUM_FUNC_NONE            ((uint8_t)0u)
#define ENUM_FUNC_HID             ((uint8_t)1u)
#define ENUM_FUNC_XUSB            ((uint8_t)2u)
#define ENUM_FUNC_GIP             ((uint8_t)3u)

/* Application-layer init state (Switch Pro, DS4, DualSense) */
#define ENUM_APP_NONE             ((uint8_t)0u)
#define ENUM_APP_RUNNING          ((uint8_t)1u)
#define ENUM_APP_DONE             ((uint8_t)2u)
#define ENUM_APP_FAILED           ((uint8_t)3u)  /* Required response timed out */

typedef struct EnumIface
{
    uint8_t  number;           /* bInterfaceNumber */
    uint8_t  alt;              /* bAlternateSetting (always 0) */
    uint8_t  cls;
    uint8_t  subcls;
    uint8_t  proto;
    uint8_t  function;         /* ENUM_FUNC_* */
    uint8_t  ep_in;            /* First interrupt IN endpoint (0 if none) */
    uint8_t  ep_out;           /* First interrupt OUT endpoint (0 if none) */
    uint16_t in_mps;           /* wMaxPacketSize[10:0] */
    uint16_t out_mps;
    uint8_t  in_interval;      /* Descriptor bInterval */
    uint8_t  out_interval;
    uint8_t  pipe_in;          /* Engine pipe (0 if closed) */
    uint8_t  pipe_out;
    uint32_t in_period;        /* IN period in (micro)frames (0 if closed) */
    uint16_t report_desc_len;  /* HID report descriptor length (0 if non-HID) */
    uint8_t  reserved[2];
    const uint8_t *report_desc;/* HID report descriptor (NULL if none) */
} EnumIface;

typedef struct EnumDevice
{
    uint32_t generation;       /* usbhs_port_info generation (per connection) */
    uint32_t enum_seq;         /* Incremented on each completed enumeration */
    uint8_t  ready;            /* 1: configured and polling IN */
    uint8_t  speed;            /* USBHS_SPEED_* */
    uint8_t  address;
    uint8_t  classification;   /* ENUM_CLASS_* */
    uint8_t  protocol;         /* ENUM_PROTO_* */
    uint8_t  app_init;         /* ENUM_APP_* */
    uint8_t  device_class;
    uint8_t  mps0;
    uint16_t vid;
    uint16_t pid;
    uint16_t bcd_usb;
    uint16_t bcd_device;
    uint8_t  num_interfaces;   /* Configuration descriptor bNumInterfaces */
    uint8_t  iface_count;      /* Number of entries in iface[] (max ENUM_IFACE_MAX) */
    uint8_t  target_iface;     /* Default measurement interface (0xFF if none) */
    uint8_t  target_ep_in;
    uint32_t frame_ticks;      /* (Micro)frame duration in timer ticks */
    EnumIface iface[ENUM_IFACE_MAX];
} EnumDevice;

/*
 * ISR callback for periodic transactions (same signature as UsbhsIntrHook).
 * Called on every completion (including NAK/error); data is only valid for IN data.
 */
typedef UsbhsIntrHook EnumIsrHook;

/*
 * Main-loop callback for each IN report drained from the engine queue.
 * rep->t1 is the ISR entry timestamp; data is valid only during the callback.
 */
typedef void (*EnumReportFn)(uint8_t iface, uint8_t ep_in, const UsbhsReport *rep, const uint8_t *data);

const EnumDevice *enum_device(void);

uint32_t enum_seq(void);

/* Find interface by (iface, ep_in); if ep_in == 0, match iface only. */
const EnumIface *enum_find_iface(uint8_t iface, uint8_t ep_in);

/* Attach/detach ISR hook on (iface, ep_in) pipe; returns false if pipe is closed. */
bool enum_set_isr_hook(uint8_t iface, uint8_t ep_in, EnumIsrHook hook);

void enum_set_report_sink(EnumReportFn fn);

/*
 * Pause (run = false) or resume (run = true) periodic polling on (iface, ep_in).
 * Pausing closes the pipe (pipe_in becomes 0); resuming opens a new pipe on the
 * next SLOT (hook must be re-attached). Cleared on disconnect or re-enumeration.
 */
bool enum_set_polling(uint8_t iface, uint8_t ep_in, bool run);

/*
 * Pause or resume all periodic IN polling on the configured device.
 * While paused, steady-state app-layer OUT traffic (e.g. Switch Pro 80 04) is
 * also halted; interrupt OUT queue is unaffected. Cleared on disconnect or re-enumeration.
 */
bool enum_set_polling_all(bool run);

bool enum_polling_paused(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_ENUM_API_H */
