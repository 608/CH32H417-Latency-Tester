/*
 * usb_desc.c - USB descriptor parsers (see usb_desc.h)
 */
#include "usb_desc.h"

#include "lat_bytes.h"

#define DT_DEVICE        (0x01u)
#define DT_CONFIG        (0x02u)
#define DT_INTERFACE     (0x04u)
#define DT_ENDPOINT      (0x05u)
#define DT_BOS           (0x0Fu)
#define DT_DEVCAP        (0x10u)
#define DT_HID           (0x21u)
#define DT_REPORT        (0x22u)
#define CAP_PLATFORM     (0x05u)
#define EP_XFER_INTR     (0x03u)

/* {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F} (MS OS 2.0 platform capability) */
static const uint8_t k_msos20_uuid[16] = {0xDFu, 0x60u, 0xDDu, 0xD8u, 0x89u, 0x45u, 0xC7u, 0x4Cu,
                                          0x9Cu, 0xD2u, 0x65u, 0x9Du, 0x9Eu, 0x64u, 0x8Au, 0x9Fu};
/* "MSFT100"(UTF-16LE) */
static const uint8_t k_msft100[14] = {0x4Du, 0x00u, 0x53u, 0x00u, 0x46u, 0x00u, 0x54u, 0x00u,
                                      0x31u, 0x00u, 0x30u, 0x00u, 0x30u, 0x00u};

bool ud_parse_device(const uint8_t *d, uint16_t len, UdDevice *out)
{
    lat_bytes_fill((uint8_t *)out, 0u, (uint32_t)sizeof(*out));
    if ((len < 8u) || (d[1] != DT_DEVICE))
    {
        return false;
    }
    out->bcd_usb = lat_get_u16(&d[2]);
    out->cls = d[4];
    out->subcls = d[5];
    out->proto = d[6];
    out->mps0 = d[7];
    if (len >= 18u)
    {
        out->vid = lat_get_u16(&d[8]);
        out->pid = lat_get_u16(&d[10]);
        out->bcd_device = lat_get_u16(&d[12]);
        out->i_manufacturer = d[14];
        out->i_product = d[15];
        out->i_serial = d[16];
        out->num_configs = d[17];
    }
    return true;
}

bool ud_parse_config(const uint8_t *d, uint16_t len, UdConfig *out)
{
    uint16_t pos;
    UdIface *cur = (UdIface *)0;

    lat_bytes_fill((uint8_t *)out, 0u, (uint32_t)sizeof(*out));
    if ((len < 9u) || (d[1] != DT_CONFIG) || (d[0] < 9u))
    {
        return false;
    }
    out->total_length = lat_get_u16(&d[2]);
    out->num_interfaces = d[4];
    out->config_value = d[5];
    pos = d[0];
    while ((uint32_t)pos + 2u <= len)
    {
        uint8_t bl = d[pos];
        uint8_t dt = d[pos + 1u];

        if ((bl < 2u) || ((uint32_t)pos + bl > len))
        {
            break; /* Malformed descriptor or end of received buffer */
        }
        if ((dt == DT_INTERFACE) && (bl >= 9u))
        {
            cur = (UdIface *)0;
            if ((d[pos + 3u] == 0u) && (out->iface_count < UD_IFACE_MAX))
            {
                cur = &out->iface[out->iface_count++];
                cur->number = d[pos + 2u];
                cur->alt = 0u;
                cur->cls = d[pos + 5u];
                cur->subcls = d[pos + 6u];
                cur->proto = d[pos + 7u];
            }
        }
        else if ((dt == DT_HID) && (bl >= 9u) && (cur != (UdIface *)0))
        {
            uint8_t n = d[pos + 5u];
            uint8_t k;

            cur->has_hid = 1u;
            /* Scan (bDescriptorType, wDescriptorLength) 3-byte pairs for report descriptor */
            for (k = 0u; (k < n) && ((uint32_t)k * 3u + 9u <= bl); k++)
            {
                if (d[pos + 6u + (k * 3u)] == DT_REPORT)
                {
                    cur->report_len = lat_get_u16(&d[pos + 7u + (k * 3u)]);
                    break;
                }
            }
        }
        else if ((dt == DT_ENDPOINT) && (bl >= 7u) && (cur != (UdIface *)0))
        {
            uint8_t ea = d[pos + 2u];
            uint8_t attr = d[pos + 3u];
            uint16_t mps = (uint16_t)(lat_get_u16(&d[pos + 4u]) & 0x07FFu);
            uint8_t iv = d[pos + 6u];

            if ((attr & 0x03u) == EP_XFER_INTR)
            {
                if ((ea & 0x80u) != 0u)
                {
                    if (cur->ep_in_count < UD_EP_IN_MAX)
                    {
                        uint8_t n = cur->ep_in_count++;

                        cur->ep_in[n] = ea;
                        cur->in_mps[n] = mps;
                        cur->in_interval[n] = iv;
                        cur->in_order[n] = cur->intr_count;
                    }
                    else if (cur->ep_in_dropped < 0xFFu)
                    {
                        cur->ep_in_dropped++;
                    }
                }
                else if (((ea & 0x80u) == 0u) && (cur->ep_out == 0u))
                {
                    cur->ep_out = ea;
                    cur->out_mps = mps;
                    cur->out_interval = iv;
                    cur->out_order = cur->intr_count;
                }
                cur->intr_count++;
            }
        }
        pos = (uint16_t)(pos + bl);
    }
    return true;
}

uint8_t ud_iface_index(const UdConfig *c, uint8_t number)
{
    uint8_t i;

    for (i = 0u; i < c->iface_count; i++)
    {
        if (c->iface[i].number == number)
        {
            return i;
        }
    }
    return 0xFFu;
}

uint16_t ud_bos_total(const uint8_t *d, uint16_t len)
{
    if ((len < 5u) || (d[1] != DT_BOS))
    {
        return 0u;
    }
    return lat_get_u16(&d[2]);
}

bool ud_bos_msos20(const uint8_t *d, uint16_t len, uint32_t win_version, uint8_t *vendor_code,
                   uint16_t *set_length)
{
    uint16_t pos;
    bool found = false;
    uint32_t best = 0u;

    if ((len < 5u) || (d[1] != DT_BOS))
    {
        return false;
    }
    pos = d[0];
    while ((uint32_t)pos + 3u <= len)
    {
        uint8_t bl = d[pos];

        if ((bl < 3u) || ((uint32_t)pos + bl > len))
        {
            break;
        }
        if ((d[pos + 1u] == DT_DEVCAP) && (d[pos + 2u] == CAP_PLATFORM) && (bl >= 20u) &&
            lat_bytes_equal(&d[pos + 4u], k_msos20_uuid, 16u))
        {
            uint16_t off;

            /* 8-byte capability records: dwWindowsVersion, wMSOSDescriptorSetTotalLength, bMS_VendorCode, bAltEnumCode */
            for (off = 20u; (uint32_t)off + 8u <= bl; off = (uint16_t)(off + 8u))
            {
                const uint8_t *e = &d[pos + off];
                uint32_t v = lat_get_u32(e);

                if ((v <= win_version) && (!found || (v >= best)))
                {
                    found = true;
                    best = v;
                    *set_length = lat_get_u16(&e[4]);
                    *vendor_code = e[6];
                }
            }
        }
        pos = (uint16_t)(pos + bl);
    }
    return found;
}

bool ud_msos10_string(const uint8_t *d, uint16_t len, uint8_t *vendor_code)
{
    if ((len < 18u) || (d[0] < 18u) || (d[1] != 0x03u) || !lat_bytes_equal(&d[2], k_msft100, 14u))
    {
        return false;
    }
    *vendor_code = d[16];
    return true;
}

uint32_t ud_msos10_length(const uint8_t *d, uint16_t len)
{
    if (len < 16u)
    {
        return 0u;
    }
    return lat_get_u32(d);
}

bool ud_msos10_has_compat(const uint8_t *d, uint16_t len, uint8_t iface, const char *id)
{
    uint16_t off;
    uint8_t want[8];
    uint8_t i;
    bool ended = false;

    for (i = 0u; i < 8u; i++)
    {
        if (!ended && (id[i] == '\0'))
        {
            ended = true;
        }
        want[i] = ended ? 0u : (uint8_t)id[i];
    }
    /* 16-byte header, followed by 24-byte function records (bFirstInterfaceNumber, reserved, compatibleID[8], ...) */
    for (off = 16u; (uint32_t)off + 24u <= len; off = (uint16_t)(off + 24u))
    {
        if ((d[off] == iface) && lat_bytes_equal(&d[off + 2u], want, 8u))
        {
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* HID report descriptor */
/* ------------------------------------------------------------------ */

#define HID_COLL_DEPTH   (8u)

void ud_parse_hid_report(const uint8_t *d, uint16_t len, UdHidInfo *out)
{
    uint16_t pos = 0u;
    uint16_t page = 0u;
    uint8_t report_id = 0u;
    uint32_t local_usage = 0u;     /* First local Usage (with page) */
    bool have_usage = false;
    uint8_t depth = 0u;
    uint8_t kbd_depth = 0u;        /* Depth inside keyboard collection (0 = outside) */

    lat_bytes_fill((uint8_t *)out, 0u, (uint32_t)sizeof(*out));
    while (pos < len)
    {
        uint8_t prefix = d[pos];
        uint8_t size;
        uint8_t type;
        uint8_t tag;
        uint32_t v = 0u;
        uint8_t k;

        if (prefix == 0xFEu)
        {
            /* Long item: bDataSize, bLongItemTag, data */
            if ((uint32_t)pos + 2u >= len)
            {
                break;
            }
            pos = (uint16_t)(pos + 3u + d[pos + 1u]);
            continue;
        }
        size = (uint8_t)(prefix & 0x03u);
        if (size == 3u)
        {
            size = 4u;
        }
        type = (uint8_t)((prefix >> 2) & 0x03u);
        tag = (uint8_t)(prefix >> 4);
        if ((uint32_t)pos + 1u + size > len)
        {
            break;
        }
        for (k = 0u; k < size; k++)
        {
            v |= (uint32_t)d[pos + 1u + k] << (8u * k);
        }
        pos = (uint16_t)(pos + 1u + size);

        if (type == 1u) /* Global */
        {
            if (tag == 0x0u)
            {
                page = (uint16_t)v;
            }
            else if (tag == 0x8u)
            {
                report_id = (uint8_t)v;
                out->uses_report_ids = 1u;
            }
        }
        else if (type == 2u) /* Local */
        {
            if ((tag == 0x0u) && !have_usage)
            {
                local_usage = (size == 4u) ? v : (((uint32_t)page << 16) | (v & 0xFFFFu));
                have_usage = true;
            }
        }
        else if (type == 0u) /* Main */
        {
            if (tag == 0xAu) /* Collection */
            {
                if (depth < HID_COLL_DEPTH)
                {
                    depth++;
                }
                if (have_usage && ((v & 0xFFu) == 0x01u)) /* Application */
                {
                    if ((local_usage == 0x00010006u) && (kbd_depth == 0u))
                    {
                        kbd_depth = depth;
                    }
                    if ((local_usage == 0x00010005u) || (local_usage == 0x00010004u))
                    {
                        out->gamepad = 1u;
                    }
                }
            }
            else if (tag == 0xCu) /* End Collection */
            {
                if ((kbd_depth != 0u) && (depth == kbd_depth))
                {
                    kbd_depth = 0u;
                }
                if (depth > 0u)
                {
                    depth--;
                }
            }
            else if (tag == 0x9u) /* Output */
            {
                uint16_t up = have_usage ? (uint16_t)(local_usage >> 16) : page;

                if ((kbd_depth != 0u) && (up == 0x0008u) && !out->kbd_led)
                {
                    out->kbd_led = 1u;
                    out->kbd_led_report_id = report_id;
                }
            }
            have_usage = false; /* Local items clear after each Main item */
        }
    }
}
