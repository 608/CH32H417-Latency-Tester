/*
 * bl_usb.c - Bootloader USBFS transport (see bl_usb.h).
 *
 * Includes USB 2.0 Extension (no LPM) and MS OS 2.0 platform capability in BOS.
 */
#include "bl_usb.h"

#include "bl_proto.h"
#include "usbfs_dev.h"

#define BL_EP_OUT            (1u)   /* 0x01 Bulk OUT */
#define BL_EP_IN             (2u)   /* 0x82 Bulk IN */
#define BL_MS_VENDOR_CODE    ((uint8_t)0x01u)
#define BL_MS_OS_20_INDEX    ((uint16_t)0x0007u)  /* MS_OS_20_DESCRIPTOR_INDEX */

static const uint8_t s_device_desc[18] = {
    0x12, 0x01,
    0x10, 0x02,             /* bcdUSB 2.10 (enables BOS) */
    0x00, 0x00, 0x00,       /* Class defined at interface level */
    0x40,                   /* bMaxPacketSize0 */
    0xC0, 0x16,             /* idVendor 0x16C0 */
    0xDC, 0x05,             /* idProduct 0x05DC */
    (uint8_t)BL_VERSION_MINOR, (uint8_t)BL_VERSION_MAJOR, /* bcdDevice */
    0x01, 0x02, 0x03,
    0x01
};

static const uint8_t s_config_desc[32] = {
    0x09, 0x02, 32, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
    /* Interface 0: vendor class, 2 endpoints */
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x00,
    /* EP 0x01 Bulk OUT 64B */
    0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,
    /* EP 0x82 Bulk IN 64B */
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00
};

/* MS OS 2.0 descriptor set (header + CompatibleID "WINUSB", 30 bytes). */
static const uint8_t s_ms_os_20_set[30] = {
    0x0A, 0x00, 0x00, 0x00,             /* wLength 10, MS_OS_20_SET_HEADER_DESCRIPTOR */
    0x00, 0x00, 0x03, 0x06,             /* dwWindowsVersion 0x06030000(Windows 8.1) */
    0x1E, 0x00,                         /* wTotalLength 30 */
    0x14, 0x00, 0x03, 0x00,             /* wLength 20, MS_OS_20_FEATURE_COMPATBLE_ID */
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* BOS: USB 2.0 Extension + MS OS 2.0 platform capability (40 bytes). */
static const uint8_t s_bos_desc[40] = {
    0x05, 0x0F, 40, 0x00, 0x02,
    /* USB 2.0 Extension (no LPM) */
    0x07, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00,
    /* Platform capability: MS OS 2.0 {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F} */
    0x1C, 0x10, 0x05, 0x00,
    0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C,
    0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
    0x00, 0x00, 0x03, 0x06,             /* dwWindowsVersion */
    0x1E, 0x00,                         /* wMSOSDescriptorSetTotalLength 30 */
    BL_MS_VENDOR_CODE,                  /* bMS_VendorCode */
    0x00                                /* bAltEnumCode */
};

_Static_assert(sizeof(s_ms_os_20_set) == 30u, "MS OS 2.0 descriptor set length changed");
_Static_assert(sizeof(s_bos_desc) == 40u, "BOS descriptor length changed");

static const uint8_t s_str_lang[4] = { 0x04, 0x03, 0x09, 0x04 };
static uint8_t s_str_manufacturer[2u + (2u * 21u)];
static uint8_t s_str_product[2u + (2u * 34u)];
static uint8_t s_str_serial[2u + (2u * 24u)];
static const uint8_t *s_strings[4];

/* Pending request (written by ISR, read by main loop) and response. */
static uint8_t s_req[BL_FRAME_SIZE];
static volatile uint16_t s_req_len;
static volatile uint32_t s_req_epoch;   /* Bus reset epoch when request arrived */
static volatile bool s_req_ready;
static uint8_t s_work[BL_FRAME_SIZE];
static uint8_t s_resp[BL_FRAME_SIZE];
static volatile uint32_t s_in_done;
static uint32_t s_in_done_seen;
static bool s_resp_in_flight;
static bool s_reset_after_resp;
static bool s_reset_ready;
static bool s_resp_resend;
static uint32_t s_reset_seen;
static uint32_t s_ep_reset_seen;

/* ---- usbfs_dev hooks (called from ISR) ---- */

static bool bl_vendor_setup(const UsbfsSetup *setup, UsbfsCtrlReply *reply)
{
    if (((setup->bmRequestType & USBFS_REQ_DIR_IN) != 0u) &&
        ((setup->bmRequestType & USBFS_REQ_RECIP_MASK) == USBFS_REQ_RECIP_DEV) &&
        (setup->bRequest == BL_MS_VENDOR_CODE) && (setup->wIndex == BL_MS_OS_20_INDEX))
    {
        reply->in_data = s_ms_os_20_set;
        reply->in_len = (uint16_t)sizeof(s_ms_os_20_set);
        return true;
    }
    return false;
}

static bool bl_ep_out(uint8_t ep, const uint8_t *data, uint16_t len)
{
    uint32_t i;

    if (ep != BL_EP_OUT)
    {
        return true;
    }
    for (i = 0u; (i < len) && (i < BL_FRAME_SIZE); i++)
    {
        s_req[i] = data[i];
    }
    s_req_len = len;
    s_req_epoch = usbfs_dev_reset_count();
    s_req_ready = true;
    return false; /* NAK further OUT until response is queued */
}

static void bl_ep_in_done(uint8_t ep)
{
    if (ep == BL_EP_IN)
    {
        s_in_done++;
    }
}

static const UsbfsDevClass s_bl_class = {
    s_device_desc,
    s_config_desc,
    s_bos_desc,
    s_strings,
    4u,
    (uint8_t)(1u << BL_EP_IN),
    (uint8_t)(1u << BL_EP_OUT),
    0,
    bl_vendor_setup,
    0,
    0,
    bl_ep_out,
    bl_ep_in_done
};

/* ---- Main loop ---- */

void bl_usb_start(const uint8_t uuid12[12])
{
    (void)usbfs_make_string_desc(s_str_manufacturer, sizeof(s_str_manufacturer), "yu_gen_00@yahoo.co.jp");
    (void)usbfs_make_string_desc(s_str_product, sizeof(s_str_product), "CH32H417 Latency Tester Bootloader");
    (void)usbfs_make_hex_string_desc(s_str_serial, sizeof(s_str_serial), uuid12, 8u);
    s_strings[0] = s_str_lang;
    s_strings[1] = s_str_manufacturer;
    s_strings[2] = s_str_product;
    s_strings[3] = s_str_serial;

    s_req_ready = false;
    s_in_done = 0u;
    s_in_done_seen = 0u;
    s_resp_in_flight = false;
    s_reset_after_resp = false;
    s_reset_ready = false;
    s_resp_resend = false;
    s_reset_seen = 0u;
    usbfs_dev_start(&s_bl_class, 0x80u);
    s_ep_reset_seen = usbfs_dev_ep_reset_count();
}

bool bl_usb_poll(uint8_t *cmd, uint8_t *status)
{
    uint32_t resets;
    uint32_t ep_resets;
    uint32_t now_resets;
    uint32_t epoch;
    uint16_t len;
    uint32_t i;

    /* 1. Check response completion before handling bus reset. */
    if (s_resp_in_flight && (s_in_done != s_in_done_seen))
    {
        s_in_done_seen = s_in_done;
        s_resp_in_flight = false;
        if (s_reset_after_resp)
        {
            /* Host received RESET_APP response; keep across any subsequent bus reset. */
            s_reset_after_resp = false;
            s_reset_ready = true;
            return false;
        }
        usbfs_ep_out_resume(BL_EP_OUT);
    }

    /* 2. Bus reset: drop in-flight response (host will retry). On re-config or
     *    ENDPOINT_HALT, resend the cancelled response. */
    resets = usbfs_dev_reset_count();
    ep_resets = usbfs_dev_ep_reset_count();
    if (resets != s_reset_seen)
    {
        s_reset_seen = resets;
        s_ep_reset_seen = ep_resets;
        s_resp_in_flight = false;
        s_resp_resend = false;
        s_reset_after_resp = false;
        s_in_done_seen = s_in_done;
    }
    else if (ep_resets != s_ep_reset_seen)
    {
        s_ep_reset_seen = ep_resets;
        if (s_resp_in_flight)
        {
            s_resp_in_flight = false;
            s_resp_resend = true;
        }
        s_in_done_seen = s_in_done;
    }
    if (s_reset_ready)
    {
        return false; /* Waiting for reboot */
    }
    if (s_resp_resend)
    {
        if (usbfs_ep_in_ready(BL_EP_IN) && usbfs_ep_in_send(BL_EP_IN, s_resp, BL_FRAME_SIZE))
        {
            s_resp_resend = false;
            s_resp_in_flight = true;
        }
        return false;
    }

    if (s_resp_in_flight || !s_req_ready || !usbfs_ep_in_ready(BL_EP_IN))
    {
        return false;
    }
    /* 3. Copy request atomically with its bus-reset epoch. */
    usbfs_dev_lock();
    now_resets = usbfs_dev_reset_count();
    for (i = 0u; i < BL_FRAME_SIZE; i++)
    {
        s_work[i] = s_req[i];
    }
    len = s_req_len;
    epoch = s_req_epoch;
    s_req_ready = false;
    usbfs_dev_unlock();
    if (epoch != now_resets)
    {
        return false; /* Stale request from before bus reset */
    }

    bl_proto_handle(s_work, len, s_resp);
    *cmd = s_resp[0];
    *status = s_resp[1];
    if (bl_proto_take_reset_request())
    {
        s_reset_after_resp = true;
    }
    if (usbfs_dev_reset_count() != epoch)
    {
        /* Bus reset occurred during processing; drop response. */
        s_reset_after_resp = false;
        return true;
    }
    if (usbfs_ep_in_send(BL_EP_IN, s_resp, BL_FRAME_SIZE))
    {
        s_resp_in_flight = true;
    }
    else
    {
        s_resp_resend = true; /* Retry on next poll if TX collided with re-config */
    }
    return true;
}

bool bl_usb_reset_ready(void)
{
    return s_reset_ready;
}

bool bl_usb_configured(void)
{
    return usbfs_dev_configured();
}
