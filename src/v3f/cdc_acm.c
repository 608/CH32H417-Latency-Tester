/*
 * cdc_acm.c - USBFS CDC-ACM device (see cdc_acm.h).
 */
#include "cdc_acm.h"

#include "lat_bytes.h"
#include "lat_version.h"
#include "pc_link.h"
#include "usbfs_dev.h"

/* Endpoints (usbfs_dev.h: do not share an endpoint number between IN and OUT) */
#define CDC_EP_DATA_OUT   (1u)   /* 0x01 Bulk OUT */
#define CDC_EP_DATA_IN    (2u)   /* 0x82 Bulk IN */
#define CDC_EP_NOTIFY     (3u)   /* 0x83 Interrupt IN (unused) */
#define CDC_PACKET        (64u)
#define CDC_IF_COMM       (0u)

/* CDC class requests */
#define CDC_SET_LINE_CODING        ((uint8_t)0x20u)
#define CDC_GET_LINE_CODING        ((uint8_t)0x21u)
#define CDC_SET_CONTROL_LINE_STATE ((uint8_t)0x22u)
#define CDC_SEND_BREAK             ((uint8_t)0x23u)

/* RX ring buffer (written by ISR, read by main loop; power of two). */
#define CDC_RX_RING       (1024u)

_Static_assert((CDC_RX_RING & (CDC_RX_RING - 1u)) == 0u, "CDC_RX_RING must be a power of two");

static const uint8_t s_device_desc[18] = {
    0x12, 0x01,             /* bLength, DEVICE */
    0x00, 0x02,             /* bcdUSB 2.00 */
    0xEF, 0x02, 0x01,       /* Composite device (IAD) */
    0x40,                   /* bMaxPacketSize0 */
    0xC0, 0x16,             /* idVendor 0x16C0 */
    0xE1, 0x05,             /* idProduct 0x05E1 */
    (uint8_t)LAT_FW_VERSION_MINOR, (uint8_t)LAT_FW_VERSION_MAJOR, /* bcdDevice = major.minor */
    0x01, 0x02, 0x03,       /* iManufacturer, iProduct, iSerialNumber */
    0x01                    /* bNumConfigurations */
};

static const uint8_t s_config_desc[75] = {
    /* Configuration */
    0x09, 0x02, 75, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,
    /* IAD: interfaces 0..1, CDC/ACM */
    0x08, 0x0B, 0x00, 0x02, 0x02, 0x02, 0x01, 0x00,
    /* Interface 0: Communication class (CDC, ACM, AT) */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* Header functional descriptor (CDC 1.10) */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* Call Management (data interface 1) */
    0x05, 0x24, 0x01, 0x00, 0x01,
    /* ACM(Set/Get_Line_Coding, Set_Control_Line_State, Serial_State) */
    0x04, 0x24, 0x02, 0x02,
    /* Union (comm 0, data 1) */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* EP 0x83 Interrupt IN 16B, 16ms */
    0x07, 0x05, 0x83, 0x03, 0x10, 0x00, 0x10,
    /* Interface 1: Data class */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* EP 0x01 Bulk OUT 64B */
    0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,
    /* EP 0x82 Bulk IN 64B */
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00
};

static const uint8_t s_str_lang[4] = { 0x04, 0x03, 0x09, 0x04 };
static uint8_t s_str_manufacturer[2u + (2u * 21u)];
static uint8_t s_str_product[2u + (2u * 23u)];
static uint8_t s_str_serial[2u + (2u * 24u)];
static const uint8_t *s_strings[4];

/* Line coding (unused, returned on GET_LINE_CODING); default 115200bps 8N1. */
static uint8_t s_line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 };
static uint8_t s_line_coding_rx[7];
static volatile uint16_t s_line_state;

static uint8_t s_rx_ring[CDC_RX_RING];
static volatile uint32_t s_rx_head;   /* written by ISR */
static volatile uint32_t s_rx_tail;   /* written by main loop */
static volatile uint32_t s_in_done;   /* ISR: bulk IN completion count */
static volatile uint32_t s_rx_pauses;
/* ISR: deconfigure/bus-reset count and RX head position at that moment */
static volatile uint32_t s_unconfig_count;
static volatile uint32_t s_unconfig_rx_mark;

static uint32_t s_in_done_seen;
static bool s_in_flight;
static bool s_resend;        /* s_tx_packet transfer unconfirmed; resend as-is */
static uint32_t s_in_flight_len;
static bool s_last_full;
static uint32_t s_unconfig_seen;
static uint32_t s_ep_reset_seen;
static uint32_t s_rx_bytes;
static uint32_t s_tx_bytes;
static uint8_t s_tx_packet[CDC_PACKET];

static uint32_t rx_free(void)
{
    return CDC_RX_RING - (s_rx_head - s_rx_tail);
}

/* ---- usbfs_dev hooks (called from ISR) ---- */

static bool cdc_class_setup(const UsbfsSetup *setup, UsbfsCtrlReply *reply)
{
    if (((setup->bmRequestType & USBFS_REQ_RECIP_MASK) != USBFS_REQ_RECIP_IF) ||
        ((uint8_t)setup->wIndex != CDC_IF_COMM))
    {
        return false;
    }
    switch (setup->bRequest)
    {
    case CDC_SET_LINE_CODING:
        reply->out_buf = s_line_coding_rx;
        reply->out_cap = (uint16_t)sizeof(s_line_coding_rx);
        return setup->wLength == sizeof(s_line_coding_rx);
    case CDC_GET_LINE_CODING:
        reply->in_data = s_line_coding;
        reply->in_len = (uint16_t)sizeof(s_line_coding);
        return true;
    case CDC_SET_CONTROL_LINE_STATE:
        s_line_state = setup->wValue;
        return true;
    case CDC_SEND_BREAK:
        return true;
    default:
        return false;
    }
}

static void cdc_ctrl_out_done(const UsbfsSetup *setup, const uint8_t *data, uint16_t len)
{
    if ((setup->bRequest == CDC_SET_LINE_CODING) && (len == sizeof(s_line_coding)))
    {
        lat_bytes_copy(s_line_coding, data, len);
    }
}

static bool cdc_ep_out(uint8_t ep, const uint8_t *data, uint16_t len)
{
    uint32_t head = s_rx_head;
    uint32_t i;

    if (ep != CDC_EP_DATA_OUT)
    {
        return true;
    }
    /* Guaranteed to fit since we only ACK when >= 64 bytes are free. */
    for (i = 0u; (i < len) && ((head - s_rx_tail) < CDC_RX_RING); i++)
    {
        s_rx_ring[head & (CDC_RX_RING - 1u)] = data[i];
        head++;
    }
    s_rx_head = head;
    if (rx_free() < CDC_PACKET)
    {
        s_rx_pauses++;
        return false; /* NAK until main loop drains ring */
    }
    return true;
}

/* Deconfigure / bus reset (value 0): mark earlier RX bytes for discard. */
static void cdc_configured(uint8_t value)
{
    if (value == 0u)
    {
        s_unconfig_rx_mark = s_rx_head;
        s_unconfig_count++;
    }
}

static void cdc_ep_in_done(uint8_t ep)
{
    if (ep == CDC_EP_DATA_IN)
    {
        s_in_done++;
    }
}

static const UsbfsDevClass s_cdc_class = {
    s_device_desc,
    s_config_desc,
    0,                   /* no BOS (bcdUSB 2.00) */
    s_strings,
    4u,
    (uint8_t)((1u << CDC_EP_DATA_IN) | (1u << CDC_EP_NOTIFY)),
    (uint8_t)(1u << CDC_EP_DATA_OUT),
    cdc_class_setup,
    0,                   /* no vendor requests */
    cdc_ctrl_out_done,
    cdc_configured,
    cdc_ep_out,
    cdc_ep_in_done
};

/* ---- Main loop ---- */

void cdc_acm_start(const uint8_t uuid12[12])
{
    (void)usbfs_make_string_desc(s_str_manufacturer, sizeof(s_str_manufacturer), "yu_gen_00@yahoo.co.jp");
    (void)usbfs_make_string_desc(s_str_product, sizeof(s_str_product), "CH32H417 Latency Tester");
    (void)usbfs_make_hex_string_desc(s_str_serial, sizeof(s_str_serial), uuid12, 8u);
    s_strings[0] = s_str_lang;
    s_strings[1] = s_str_manufacturer;
    s_strings[2] = s_str_product;
    s_strings[3] = s_str_serial;

    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_in_done = 0u;
    s_in_done_seen = 0u;
    s_in_flight = false;
    s_resend = false;
    s_last_full = false;
    s_unconfig_count = 0u;
    s_unconfig_seen = 0u;
    usbfs_dev_start(&s_cdc_class, 0x80u);
    s_ep_reset_seen = usbfs_dev_ep_reset_count();
}

void cdc_acm_poll(void)
{
    bool configured = usbfs_dev_configured();
    uint32_t unconfig = s_unconfig_count;
    uint32_t ep_resets = usbfs_dev_ep_reset_count();

    /* TX completion (check before bus reset handling). */
    if (s_in_flight && (s_in_done != s_in_done_seen))
    {
        s_in_done_seen = s_in_done;
        s_in_flight = false;
        pc_link_tx_done(s_in_flight_len);
        s_tx_bytes += s_in_flight_len;
    }

    /*
     * Endpoint init (bus reset, SET_CONFIGURATION, ENDPOINT_HALT) cancels any
     * in-flight IN transfer. Resend unconfirmed packets; PC COBS framing discards
     * any corrupted partial frame at 0x00 delimiters.
     */
    if (ep_resets != s_ep_reset_seen)
    {
        s_ep_reset_seen = ep_resets;
        if (s_in_flight)
        {
            s_in_flight = false;
            s_resend = true;
        }
        s_in_done_seen = s_in_done;
    }

    /* Deconfigure or bus reset: discard earlier RX bytes and partial frame. */
    if (unconfig != s_unconfig_seen)
    {
        uint32_t mark = s_unconfig_rx_mark;

        s_unconfig_seen = unconfig;
        if ((int32_t)(mark - s_rx_tail) > 0)
        {
            s_rx_tail = mark;
        }
        pc_link_rx_reset();
    }

    /* RX */
    for (;;)
    {
        uint32_t tail = s_rx_tail;
        uint32_t head = s_rx_head;
        uint32_t pos;
        uint32_t chunk;

        if ((head == tail) || (s_unconfig_count != s_unconfig_seen))
        {
            break; /* Bus reset occurred mid-drain; discard on next poll */
        }
        pos = tail & (CDC_RX_RING - 1u);
        chunk = head - tail;
        if (chunk > (CDC_RX_RING - pos))
        {
            chunk = CDC_RX_RING - pos;
        }
        pc_link_rx_bytes(&s_rx_ring[pos], chunk);
        s_rx_bytes += chunk;
        s_rx_tail = tail + chunk;
    }
    if (usbfs_ep_out_paused(CDC_EP_DATA_OUT) && (rx_free() >= CDC_PACKET))
    {
        usbfs_ep_out_resume(CDC_EP_DATA_OUT);
    }

    /* TX */
    if (!configured || s_in_flight || !usbfs_ep_in_ready(CDC_EP_DATA_IN))
    {
        return;
    }
    {
        uint32_t n;

        if (s_resend)
        {
            n = s_in_flight_len;
        }
        else
        {
            n = pc_link_tx_fill(s_tx_packet, CDC_PACKET);
            if ((n == 0u) && !s_last_full)
            {
                return;
            }
        }
        s_in_flight_len = n;
        if (usbfs_ep_in_send(CDC_EP_DATA_IN, s_tx_packet, (uint16_t)n))
        {
            s_in_flight = true;
            s_resend = false;
            /* Send ZLP after an exact 64-byte packet when no more data is pending. */
            s_last_full = (n == CDC_PACKET);
        }
        else
        {
            /* Bus reset raced with the previous check; resend on next poll. */
            s_resend = true;
        }
    }
}

void cdc_acm_get_stats(CdcAcmStats *out)
{
    out->configured = usbfs_dev_configured();
    out->dtr = (s_line_state & 0x0001u) != 0u;
    out->bus_resets = usbfs_dev_reset_count();
    out->rx_bytes = s_rx_bytes;
    out->tx_bytes = s_tx_bytes;
    out->rx_pauses = s_rx_pauses;
}
