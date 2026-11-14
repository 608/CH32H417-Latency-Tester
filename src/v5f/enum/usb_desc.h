/*
 * usb_desc.h - Hardware-independent USB descriptor parsers
 */
#ifndef LAT_USB_DESC_H
#define LAT_USB_DESC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UD_IFACE_MAX          (8u)
#define UD_EP_IN_MAX          (4u)

typedef struct UdIface
{
    uint8_t  number;
    uint8_t  alt;
    uint8_t  cls;
    uint8_t  subcls;
    uint8_t  proto;
    uint8_t  ep_in[UD_EP_IN_MAX]; /* Interrupt IN endpoints in descriptor order */
    uint8_t  ep_in_count;
    uint8_t  ep_in_dropped;
    uint8_t  ep_out;          /* First interrupt OUT endpoint (0 if none) */
    uint8_t  in_interval[UD_EP_IN_MAX];
    uint8_t  out_interval;
    uint8_t  in_order[UD_EP_IN_MAX];
    uint8_t  out_order;
    uint8_t  intr_count;
    uint8_t  has_hid;
    uint16_t in_mps[UD_EP_IN_MAX];
    uint16_t out_mps;
    uint16_t report_len;      /* HID descriptor wDescriptorLength */
} UdIface;

typedef struct UdConfig
{
    uint16_t total_length;    /* wTotalLength */
    uint8_t  num_interfaces;  /* bNumInterfaces */
    uint8_t  config_value;    /* bConfigurationValue */
    uint8_t  iface_count;     /* Alt-setting 0 interfaces (max UD_IFACE_MAX) */
    uint8_t  reserved[3];
    UdIface  iface[UD_IFACE_MAX];
} UdConfig;

typedef struct UdDevice
{
    uint16_t bcd_usb;
    uint16_t vid;
    uint16_t pid;
    uint16_t bcd_device;
    uint8_t  cls;
    uint8_t  subcls;
    uint8_t  proto;
    uint8_t  mps0;
    uint8_t  i_manufacturer;
    uint8_t  i_product;
    uint8_t  i_serial;
    uint8_t  num_configs;
} UdDevice;

/* Parse device descriptor (>= 8 bytes); 8..17 bytes fills available fields only. */
bool ud_parse_device(const uint8_t *d, uint16_t len, UdDevice *out);

/* Parse configuration descriptor; keeps alt-setting 0 interfaces only. */
bool ud_parse_config(const uint8_t *d, uint16_t len, UdConfig *out);

/* Return index in iface[] for interface number, or 0xFF if not found. */
uint8_t ud_iface_index(const UdConfig *c, uint8_t number);

/*
 * Find MS OS 2.0 platform capability in BOS with the highest dwWindowsVersion
 * <= win_version, returning bMS_VendorCode and descriptor set length.
 */
bool ud_bos_msos20(const uint8_t *d, uint16_t len, uint32_t win_version, uint8_t *vendor_code,
                   uint16_t *set_length);

/* Return BOS wTotalLength (0 if len < 5). */
uint16_t ud_bos_total(const uint8_t *d, uint16_t len);

/* Extract MS OS 1.0 vendor code if string 0xEE has "MSFT100" signature. */
bool ud_msos10_string(const uint8_t *d, uint16_t len, uint8_t *vendor_code);

/* Return MS OS 1.0 compat ID dwLength (0 if len < 16). */
uint32_t ud_msos10_length(const uint8_t *d, uint16_t len);

/* Check if compat ID descriptor has an entry for iface matching 8-byte zero-padded id. */
bool ud_msos10_has_compat(const uint8_t *d, uint16_t len, uint8_t iface, const char *id);

typedef struct UdHidInfo
{
    uint8_t kbd_led;          /* Keyboard collection (01/06) has LED output (page 08) */
    uint8_t kbd_led_report_id;/* Report ID for LED output (0 if none) */
    uint8_t gamepad;          /* Has Application collection 01/05 or 01/04 */
    uint8_t uses_report_ids;
} UdHidInfo;

void ud_parse_hid_report(const uint8_t *d, uint16_t len, UdHidInfo *out);

#ifdef __cplusplus
}
#endif

#endif /* LAT_USB_DESC_H */
