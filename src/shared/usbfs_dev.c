/*
 * usbfs_dev.c - Shared USBFS device driver (see usbfs_dev.h).
 *
 * EP0 control states:
 *   IDLE        Wait for SETUP (RX=ACK, TX=NAK).
 *   IN_DATA     Send IN packets; send ZLP if total < wLength and multiple of 64.
 *   OUT_DATA    Receive TOG_OK OUT packets.
 *   STATUS_IN   Zero-length DATA1 IN; apply SET_ADDRESS after completion (USB 2.0 9.4.6).
 *   STATUS_OUT  Wait for zero-length DATA1 OUT.
 *
 * INT_BUSY in BASE_CTRL keeps SIE NAKing until IRQ flags are cleared, so DMA
 * buffers can be safely accessed inside the ISR.
 */
#include "usbfs_dev.h"

#include "ch32h417.h"
#include "ch32h417_usb.h"

#include "lat_bytes.h"
#include "lat_fence.h"

/* ---- Endpoint registers (matches EVT USBFSD_UEP_*) ---- */
#define UEP_DMA(n)    (((volatile uint32_t *)(uintptr_t)&USBFSD->UEP0_DMA)[(n)])
#define UEP_TCTRL(n)  (*(volatile uint8_t *)((uintptr_t)&USBFSD->UEP0_TX_CTRL + (4u * (uint32_t)(n))))
#define UEP_RCTRL(n)  (*(volatile uint8_t *)((uintptr_t)&USBFSD->UEP0_RX_CTRL + (4u * (uint32_t)(n))))

#define T_TOG         USBFS_UEP_T_TOG
#define T_RES_MASK    USBFS_UEP_T_RES_MASK
#define T_RES_ACK     USBFS_UEP_T_RES_ACK
#define T_RES_NAK     USBFS_UEP_T_RES_NAK
#define T_RES_STALL   USBFS_UEP_T_RES_STALL
#define R_TOG         USBFS_UEP_R_TOG
#define R_RES_MASK    USBFS_UEP_R_RES_MASK
#define R_RES_ACK     USBFS_UEP_R_RES_ACK
#define R_RES_NAK     USBFS_UEP_R_RES_NAK
#define R_RES_STALL   USBFS_UEP_R_RES_STALL

/* Standard requests */
#define REQ_GET_STATUS        ((uint8_t)0x00u)
#define REQ_CLEAR_FEATURE     ((uint8_t)0x01u)
#define REQ_SET_FEATURE       ((uint8_t)0x03u)
#define REQ_SET_ADDRESS       ((uint8_t)0x05u)
#define REQ_GET_DESCRIPTOR    ((uint8_t)0x06u)
#define REQ_GET_CONFIGURATION ((uint8_t)0x08u)
#define REQ_SET_CONFIGURATION ((uint8_t)0x09u)
#define REQ_GET_INTERFACE     ((uint8_t)0x0Au)
#define REQ_SET_INTERFACE     ((uint8_t)0x0Bu)
#define FEATURE_ENDPOINT_HALT ((uint16_t)0x0000u)

typedef enum CtrlState
{
    CTRL_IDLE = 0,
    CTRL_IN_DATA,
    CTRL_OUT_DATA,
    CTRL_STATUS_IN,
    CTRL_STATUS_OUT
} CtrlState;

static uint8_t s_ep0_buf[USBFS_EP0_SIZE] __attribute__((aligned(4)));
static uint8_t s_ep_buf[USBFS_EP_NUM][USBFS_EP_PACKET_MAX] __attribute__((aligned(4)));

static const UsbfsDevClass *s_cls;
static volatile bool s_started;
static volatile uint8_t s_config;
static volatile bool s_in_busy[USBFS_EP_NUM];
static volatile bool s_out_paused[USBFS_EP_NUM];
static volatile bool s_in_halt[USBFS_EP_NUM];
static volatile bool s_out_halt[USBFS_EP_NUM];
static volatile uint32_t s_reset_count;
static volatile uint32_t s_ep_reset_count;
static UsbfsDiag s_diag;

static UsbfsSetup s_setup;
static CtrlState s_ctrl;
static const uint8_t *s_in_ptr;
static uint16_t s_in_remaining;
static bool s_in_zlp;
static uint8_t *s_out_ptr;
static uint16_t s_out_expected;
static uint16_t s_out_received;
static uint8_t s_pending_addr;
static bool s_addr_pending;
static uint8_t s_small[2];

/* ---- Helpers ---- */

/* Short busy-wait used only during init/stop (never in ISR). */
static void short_delay(void)
{
    volatile uint32_t i;

    for (i = 0u; i < 4000u; i++)
    {
    }
}

static void irq_lock(void)
{
    NVIC_DisableIRQ(USBFS_IRQn);
    /* Commit PFIC write before continuing. */
    (void)NVIC->ISR[((uint32_t)USBFS_IRQn) >> 5];
    LAT_FENCE_IORW();
}

static void irq_unlock(void)
{
    LAT_FENCE_IORW();
    if (s_started)
    {
        NVIC_EnableIRQ(USBFS_IRQn);
    }
}

static uint16_t min_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

static uint16_t desc_total_length(const uint8_t *desc)
{
    return lat_get_u16(&desc[2]);
}

static bool ep_is_in(uint8_t ep)
{
    return (ep >= 1u) && (ep < USBFS_EP_NUM) && ((s_cls->ep_in_mask & (1u << ep)) != 0u);
}

static bool ep_is_out(uint8_t ep)
{
    return (ep >= 1u) && (ep < USBFS_EP_NUM) && ((s_cls->ep_out_mask & (1u << ep)) != 0u);
}

/* TX/RX enable bits in UEPn_MOD for EPn. */
static void ep_mod_bits(uint8_t ep, volatile uint8_t **reg, uint8_t *rx_en, uint8_t *tx_en)
{
    volatile uint8_t *base = &USBFSD->UEP4_1_MOD;

    switch (ep)
    {
    case 1u: *reg = &base[0]; *rx_en = 0x80u; *tx_en = 0x40u; break;
    case 2u: *reg = &base[1]; *rx_en = 0x08u; *tx_en = 0x04u; break;
    case 3u: *reg = &base[1]; *rx_en = 0x80u; *tx_en = 0x40u; break;
    case 4u: *reg = &base[0]; *rx_en = 0x08u; *tx_en = 0x04u; break;
    case 5u: *reg = &base[2]; *rx_en = 0x08u; *tx_en = 0x04u; break;
    case 6u: *reg = &base[2]; *rx_en = 0x80u; *tx_en = 0x40u; break;
    default: *reg = &base[3]; *rx_en = 0x08u; *tx_en = 0x04u; break;
    }
}

/*
 * Write UEPn_TX_LEN (n = 0..7) directly via SDK struct fields; only UEP3 has
 * a contiguous uint16_t with TX_CTRL, while others have a reserved byte.
 */
static void uep_set_tlen(uint8_t ep, uint16_t len)
{
    switch (ep)
    {
    case 0u: USBFSD->UEP0_TX_LEN = (uint8_t)len; break;
    case 1u: USBFSD->UEP1_TX_LEN = (uint8_t)len; break;
    case 2u: USBFSD->UEP2_TX_LEN = (uint8_t)len; break;
    case 3u: USBFSD->UEP3_TX_LEN = len;          break;
    case 4u: USBFSD->UEP4_TX_LEN = (uint8_t)len; break;
    case 5u: USBFSD->UEP5_TX_LEN = (uint8_t)len; break;
    case 6u: USBFSD->UEP6_TX_LEN = (uint8_t)len; break;
    default: USBFSD->UEP7_TX_LEN = (uint8_t)len; break;
    }
}

/* Reset endpoint modes, DMA addresses, and responses to unconfigured state. */
static void endpoints_init(void)
{
    uint8_t ep;

    s_ep_reset_count++;
    USBFSD->UEP4_1_MOD = 0u;
    USBFSD->UEP2_3_MOD = 0u;
    USBFSD->UEP5_6_MOD = 0u;
    USBFSD->UEP7_MOD = 0u;

    UEP_DMA(0) = (uint32_t)(uintptr_t)s_ep0_buf;
    uep_set_tlen(0u, 0u);
    UEP_TCTRL(0) = T_RES_NAK;
    UEP_RCTRL(0) = R_RES_ACK;

    for (ep = 1u; ep < USBFS_EP_NUM; ep++)
    {
        volatile uint8_t *reg;
        uint8_t rx_en;
        uint8_t tx_en;

        s_in_busy[ep] = false;
        s_out_paused[ep] = false;
        s_in_halt[ep] = false;
        s_out_halt[ep] = false;
        if (!ep_is_in(ep) && !ep_is_out(ep))
        {
            continue;
        }
        ep_mod_bits(ep, &reg, &rx_en, &tx_en);
        *reg = (uint8_t)(*reg | (ep_is_out(ep) ? rx_en : tx_en));
        UEP_DMA(ep) = (uint32_t)(uintptr_t)s_ep_buf[ep];
        uep_set_tlen(ep, 0u);
        UEP_TCTRL(ep) = T_RES_NAK;
        UEP_RCTRL(ep) = R_RES_NAK;
    }
}

/* SET_CONFIGURATION: start from DATA0 and enable OUT if configured. */
static void endpoints_configure(bool on)
{
    uint8_t ep;

    s_ep_reset_count++;
    for (ep = 1u; ep < USBFS_EP_NUM; ep++)
    {
        s_in_halt[ep] = false;
        s_out_halt[ep] = false;
        if (ep_is_in(ep))
        {
            s_in_busy[ep] = false;
            uep_set_tlen(ep, 0u);
            UEP_TCTRL(ep) = T_RES_NAK;
        }
        if (ep_is_out(ep))
        {
            s_out_paused[ep] = false;
            UEP_RCTRL(ep) = on ? R_RES_ACK : R_RES_NAK;
        }
    }
}

/* ---- EP0 ---- */

static void ep0_stall(void)
{
    s_diag.stall++;
    UEP_TCTRL(0) = T_TOG | T_RES_STALL;
    UEP_RCTRL(0) = R_TOG | R_RES_STALL;
    s_ctrl = CTRL_IDLE;
}

static void ep0_status_in(void)
{
    uep_set_tlen(0u, 0u);
    UEP_TCTRL(0) = T_TOG | T_RES_ACK;
    s_ctrl = CTRL_STATUS_IN;
}

static void ep0_status_out(void)
{
    UEP_TCTRL(0) = T_RES_NAK;
    UEP_RCTRL(0) = R_TOG | R_RES_ACK;
    s_ctrl = CTRL_STATUS_OUT;
}

static void ep0_send_packet(bool first)
{
    uint16_t n = min_u16(s_in_remaining, (uint16_t)USBFS_EP0_SIZE);
    uint8_t tog;

    lat_bytes_copy(s_ep0_buf, s_in_ptr, n);
    s_in_ptr += n;
    s_in_remaining = (uint16_t)(s_in_remaining - n);
    uep_set_tlen(0u, n);
    if (first)
    {
        tog = T_TOG; /* First data-stage packet is DATA1 */
    }
    else
    {
        tog = (uint8_t)((UEP_TCTRL(0) & T_TOG) ^ T_TOG);
    }
    UEP_TCTRL(0) = (uint8_t)(tog | T_RES_ACK);
    s_ctrl = CTRL_IN_DATA;
    s_diag.ep0_in_sent++;
}

static void ep0_start_in(const uint8_t *data, uint16_t len)
{
    len = min_u16(len, s_setup.wLength);
    s_in_ptr = data;
    s_in_remaining = len;
    /* Send ZLP if response is shorter than wLength and a non-zero multiple of 64. */
    s_in_zlp = (len < s_setup.wLength) && (len != 0u) && ((len % USBFS_EP0_SIZE) == 0u);
    ep0_send_packet(true);
}

static void ep0_in_done(void)
{
    switch (s_ctrl)
    {
    case CTRL_IN_DATA:
        if (s_in_remaining != 0u)
        {
            ep0_send_packet(false);
        }
        else if (s_in_zlp)
        {
            s_in_zlp = false;
            ep0_send_packet(false); /* s_in_remaining == 0 -> ZLP */
        }
        else
        {
            ep0_status_out();
        }
        break;
    case CTRL_STATUS_IN:
        if (s_addr_pending)
        {
            USBFSD->DEV_ADDR = (uint8_t)((USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | (s_pending_addr & USBFS_USB_ADDR_MASK));
            s_addr_pending = false;
            s_diag.set_address++;
        }
        s_ctrl = CTRL_IDLE;
        UEP_TCTRL(0) = T_RES_NAK;
        UEP_RCTRL(0) = R_RES_ACK;
        break;
    default:
        UEP_TCTRL(0) = T_RES_NAK;
        break;
    }
}

static void ep0_out_done(uint8_t int_st, uint16_t len)
{
    if (s_ctrl == CTRL_OUT_DATA)
    {
        uint16_t room;
        uint16_t n;

        if ((int_st & USBFS_UIS_TOG_OK) == 0u)
        {
            return; /* Retransmission; drop and keep toggle */
        }
        room = (uint16_t)(s_out_expected - s_out_received);
        n = min_u16(len, room);
        lat_bytes_copy(&s_out_ptr[s_out_received], s_ep0_buf, n);
        s_out_received = (uint16_t)(s_out_received + n);
        UEP_RCTRL(0) = (uint8_t)(UEP_RCTRL(0) ^ R_TOG);
        if ((s_out_received >= s_out_expected) || (len < USBFS_EP0_SIZE))
        {
            if (s_cls->ctrl_out_done != 0)
            {
                s_cls->ctrl_out_done(&s_setup, s_out_ptr, s_out_received);
            }
            UEP_RCTRL(0) = R_RES_NAK;
            ep0_status_in();
        }
        return;
    }
    if ((s_ctrl == CTRL_STATUS_OUT) || (s_ctrl == CTRL_IN_DATA))
    {
        /* Status stage (or early host termination of IN): accept zero-length DATA1 only. */
        if ((int_st & USBFS_UIS_TOG_OK) == 0u)
        {
            return;
        }
        if (len != 0u)
        {
            ep0_stall();
            return;
        }
        s_ctrl = CTRL_IDLE;
        UEP_TCTRL(0) = T_RES_NAK;
        UEP_RCTRL(0) = R_RES_ACK;
    }
}

static bool std_get_descriptor(UsbfsCtrlReply *reply)
{
    uint8_t type = (uint8_t)(s_setup.wValue >> 8);
    uint8_t index = (uint8_t)s_setup.wValue;

    if ((s_setup.bmRequestType & USBFS_REQ_RECIP_MASK) != USBFS_REQ_RECIP_DEV)
    {
        return false;
    }
    switch (type)
    {
    case USBFS_DESC_DEVICE:
        reply->in_data = s_cls->device_desc;
        reply->in_len = s_cls->device_desc[0];
        return true;
    case USBFS_DESC_CONFIG:
        if (index != 0u)
        {
            return false;
        }
        reply->in_data = s_cls->config_desc;
        reply->in_len = desc_total_length(s_cls->config_desc);
        return true;
    case USBFS_DESC_STRING:
        if ((index >= s_cls->string_count) || (s_cls->strings[index] == 0))
        {
            return false;
        }
        reply->in_data = s_cls->strings[index];
        reply->in_len = s_cls->strings[index][0];
        return true;
    case USBFS_DESC_BOS:
        if (s_cls->bos_desc == 0)
        {
            return false;
        }
        reply->in_data = s_cls->bos_desc;
        reply->in_len = desc_total_length(s_cls->bos_desc);
        return true;
    default:
        /* Full-Speed-only devices STALL Device Qualifier and Other Speed Config (USB 2.0 9.6.2). */
        return false;
    }
}

static bool ep_address_valid(uint8_t addr, bool *is_in, uint8_t *ep)
{
    *ep = (uint8_t)(addr & 0x0Fu);
    *is_in = (addr & 0x80u) != 0u;
    if ((addr & 0x70u) != 0u)
    {
        return false;
    }
    if (*ep == 0u)
    {
        return true;
    }
    return *is_in ? ep_is_in(*ep) : ep_is_out(*ep);
}

static void ep_set_halt(uint8_t ep, bool is_in, bool halt)
{
    s_ep_reset_count++;
    if (is_in)
    {
        s_in_halt[ep] = halt;
        s_in_busy[ep] = false;
        uep_set_tlen(ep, 0u);
        UEP_TCTRL(ep) = halt ? T_RES_STALL : T_RES_NAK; /* Reset to DATA0 on clear */
    }
    else
    {
        s_out_halt[ep] = halt;
        if (halt)
        {
            UEP_RCTRL(ep) = R_RES_STALL;
        }
        else
        {
            UEP_RCTRL(ep) = s_out_paused[ep] ? R_RES_NAK : R_RES_ACK; /* From DATA0 */
        }
    }
}

/* Standard request handler; returns false to STALL. */
static bool std_request(UsbfsCtrlReply *reply)
{
    uint8_t recip = (uint8_t)(s_setup.bmRequestType & USBFS_REQ_RECIP_MASK);
    bool dir_in = (s_setup.bmRequestType & USBFS_REQ_DIR_IN) != 0u;
    uint8_t num_if = s_cls->config_desc[4];
    uint8_t cfg_value = s_cls->config_desc[5];

    /* Validate direction and wLength before any side effects. */
    switch (s_setup.bRequest)
    {
    case REQ_GET_DESCRIPTOR:
        if (!dir_in)
        {
            return false;
        }
        break;
    case REQ_GET_STATUS:
    case REQ_GET_CONFIGURATION:
    case REQ_GET_INTERFACE:
        /* wValue is 0; interface/endpoint index is in low byte of wIndex. */
        if (!dir_in || (s_setup.wValue != 0u) || ((s_setup.wIndex >> 8) != 0u))
        {
            return false;
        }
        break;
    case REQ_CLEAR_FEATURE:
    case REQ_SET_FEATURE:
    case REQ_SET_ADDRESS:
    case REQ_SET_CONFIGURATION:
    case REQ_SET_INTERFACE:
        if (dir_in || (s_setup.wLength != 0u) || ((s_setup.wIndex >> 8) != 0u))
        {
            return false;
        }
        break;
    default:
        return false;
    }

    switch (s_setup.bRequest)
    {
    case REQ_GET_STATUS:
        s_small[0] = 0u;
        s_small[1] = 0u;
        if (recip == USBFS_REQ_RECIP_DEV)
        {
            /* Bus-powered, no remote wakeup */
            if (s_setup.wIndex != 0u)
            {
                return false;
            }
        }
        else if (recip == USBFS_REQ_RECIP_IF)
        {
            if ((s_config == 0u) || ((uint8_t)s_setup.wIndex >= num_if))
            {
                return false;
            }
        }
        else if (recip == USBFS_REQ_RECIP_EP)
        {
            bool is_in;
            uint8_t ep;

            if (!ep_address_valid((uint8_t)s_setup.wIndex, &is_in, &ep))
            {
                return false;
            }
            if ((ep != 0u) && (is_in ? s_in_halt[ep] : s_out_halt[ep]))
            {
                s_small[0] = 1u;
            }
        }
        else
        {
            return false;
        }
        reply->in_data = s_small;
        reply->in_len = 2u;
        return true;

    case REQ_CLEAR_FEATURE:
    case REQ_SET_FEATURE:
    {
        bool is_in;
        uint8_t ep;

        if ((recip != USBFS_REQ_RECIP_EP) || (s_setup.wValue != FEATURE_ENDPOINT_HALT) ||
            !ep_address_valid((uint8_t)s_setup.wIndex, &is_in, &ep) || (ep == 0u) || (s_config == 0u))
        {
            return false;
        }
        ep_set_halt(ep, is_in, s_setup.bRequest == REQ_SET_FEATURE);
        return true;
    }

    case REQ_SET_ADDRESS:
        if ((recip != USBFS_REQ_RECIP_DEV) || (s_setup.wValue > 127u) || (s_setup.wIndex != 0u) ||
            (s_setup.wLength != 0u))
        {
            return false;
        }
        s_pending_addr = (uint8_t)s_setup.wValue;
        s_addr_pending = true;
        return true;

    case REQ_GET_DESCRIPTOR:
        return std_get_descriptor(reply);

    case REQ_GET_CONFIGURATION:
        if ((recip != USBFS_REQ_RECIP_DEV) || (s_setup.wIndex != 0u))
        {
            return false;
        }
        s_small[0] = s_config;
        reply->in_data = s_small;
        reply->in_len = 1u;
        return true;

    case REQ_SET_CONFIGURATION:
    {
        uint8_t value = (uint8_t)s_setup.wValue;

        if ((recip != USBFS_REQ_RECIP_DEV) || ((s_setup.wValue >> 8) != 0u) || (s_setup.wIndex != 0u) ||
            ((value != 0u) && (value != cfg_value)))
        {
            return false;
        }
        s_config = value;
        endpoints_configure(value != 0u);
        if (s_cls->configured != 0)
        {
            s_cls->configured(value);
        }
        return true;
    }

    case REQ_GET_INTERFACE:
        if ((recip != USBFS_REQ_RECIP_IF) || (s_config == 0u) || ((uint8_t)s_setup.wIndex >= num_if))
        {
            return false;
        }
        s_small[0] = 0u;
        reply->in_data = s_small;
        reply->in_len = 1u;
        return true;

    case REQ_SET_INTERFACE:
        /* Only alternate setting 0 is supported. */
        return (recip == USBFS_REQ_RECIP_IF) && (s_config != 0u) &&
               ((uint8_t)s_setup.wIndex < num_if) && (s_setup.wValue == 0u);

    default:
        return false;
    }
}

static void ep0_setup(void)
{
    UsbfsCtrlReply reply = {0};
    uint8_t type;
    bool ok;

    /* Read 8 bytes from EP0 buffer on SETUP without checking RX_LEN. */
    UEP_TCTRL(0) = T_TOG | T_RES_NAK;
    UEP_RCTRL(0) = R_TOG | R_RES_NAK;
    s_ctrl = CTRL_IDLE;
    s_in_zlp = false;
    s_addr_pending = false;

    s_setup.bmRequestType = s_ep0_buf[0];
    s_setup.bRequest = s_ep0_buf[1];
    s_setup.wValue = lat_get_u16(&s_ep0_buf[2]);
    s_setup.wIndex = lat_get_u16(&s_ep0_buf[4]);
    s_setup.wLength = lat_get_u16(&s_ep0_buf[6]);
    lat_bytes_copy(s_diag.last_setup, s_ep0_buf, 8u);

    type = (uint8_t)(s_setup.bmRequestType & USBFS_REQ_TYPE_MASK);
    if (type == USBFS_REQ_TYPE_STD)
    {
        ok = std_request(&reply);
    }
    else if ((type == USBFS_REQ_TYPE_CLASS) && (s_cls->class_setup != 0))
    {
        ok = s_cls->class_setup(&s_setup, &reply);
    }
    else if ((type == USBFS_REQ_TYPE_VENDOR) && (s_cls->vendor_setup != 0))
    {
        ok = s_cls->vendor_setup(&s_setup, &reply);
    }
    else
    {
        ok = false;
    }

    if (!ok)
    {
        s_addr_pending = false;
        ep0_stall();
        return;
    }
    if (s_setup.wLength == 0u)
    {
        ep0_status_in();
    }
    else if ((s_setup.bmRequestType & USBFS_REQ_DIR_IN) != 0u)
    {
        if (reply.in_data == 0)
        {
            reply.in_len = 0u;
            reply.in_data = s_small;
        }
        ep0_start_in(reply.in_data, reply.in_len);
    }
    else
    {
        if ((reply.out_buf == 0) || (reply.out_cap < s_setup.wLength))
        {
            ep0_stall();
            return;
        }
        s_out_ptr = reply.out_buf;
        s_out_expected = s_setup.wLength;
        s_out_received = 0u;
        UEP_RCTRL(0) = R_TOG | R_RES_ACK; /* First data-stage packet is DATA1 */
        s_ctrl = CTRL_OUT_DATA;
    }
}

/* ---- EP1-7 ---- */

static void ep_in_done(uint8_t ep)
{
    if (!ep_is_in(ep))
    {
        return;
    }
    UEP_TCTRL(ep) = (uint8_t)(((UEP_TCTRL(ep) & T_TOG) ^ T_TOG) | T_RES_NAK);
    s_in_busy[ep] = false;
    if (s_cls->ep_in_done != 0)
    {
        s_cls->ep_in_done(ep);
    }
}

static void ep_out_done(uint8_t ep, uint8_t int_st, uint16_t len)
{
    bool keep = true;

    if (!ep_is_out(ep))
    {
        return;
    }
    if ((int_st & USBFS_UIS_TOG_OK) == 0u)
    {
        return; /* Host retransmission (ACK lost); already processed */
    }
    UEP_RCTRL(ep) = (uint8_t)(UEP_RCTRL(ep) ^ R_TOG);
    if (len > USBFS_EP_PACKET_MAX)
    {
        len = USBFS_EP_PACKET_MAX;
    }
    if (s_cls->ep_out != 0)
    {
        keep = s_cls->ep_out(ep, s_ep_buf[ep], len);
    }
    if (!keep)
    {
        UEP_RCTRL(ep) = (uint8_t)((UEP_RCTRL(ep) & (uint8_t)~R_RES_MASK) | R_RES_NAK);
        s_out_paused[ep] = true;
    }
}

static void handle_bus_reset(void)
{
    s_reset_count++;
    s_diag.bus_reset++;
    s_config = 0u;
    s_ctrl = CTRL_IDLE;
    s_addr_pending = false;
    s_in_zlp = false;
    USBFSD->DEV_ADDR = 0u;
    endpoints_init();
    if (s_cls->configured != 0)
    {
        s_cls->configured(0u);
    }
}

void USBFS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBFS_IRQHandler(void)
{
    uint8_t flags = USBFSD->INT_FG;
    uint8_t st = USBFSD->INT_ST;

    s_diag.isr++;

    if ((flags & USBFS_UIF_TRANSFER) != 0u)
    {
        uint8_t token = (uint8_t)(st & USBFS_UIS_TOKEN_MASK);
        uint8_t ep = (uint8_t)(st & USBFS_UIS_ENDP_MASK);
        uint16_t rx_len = USBFSD->RX_LEN;

        s_diag.last_int_fg = flags;
        s_diag.last_int_st = st;
        s_diag.last_rx_len = rx_len;

        switch (token)
        {
        case USBFS_UIS_TOKEN_SETUP:
            s_diag.xfer_setup++;
            /* Ignore endpoint bits in INT_ST on SETUP (hardware may report non-zero). */
            ep0_setup();
            break;
        case USBFS_UIS_TOKEN_IN:
            s_diag.xfer_in++;
            if (ep == 0u)
            {
                ep0_in_done();
            }
            else
            {
                ep_in_done(ep);
            }
            break;
        case USBFS_UIS_TOKEN_OUT:
            s_diag.xfer_out++;
            if (ep == 0u)
            {
                ep0_out_done(st, rx_len);
            }
            else
            {
                ep_out_done(ep, st, rx_len);
            }
            break;
        default:
            s_diag.xfer_other++;
            break;
        }
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
    }
    else if ((flags & USBFS_UIF_BUS_RST) != 0u)
    {
        handle_bus_reset();
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    }
    else if ((flags & USBFS_UIF_SUSPEND) != 0u)
    {
        /* No-op on suspend/resume (do not use Delay_Us in ISR). */
        s_diag.suspend++;
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
    }
    else
    {
        USBFSD->INT_FG = flags;
    }
}

/* ---- Public API ---- */

bool usbfs_dev_clock_init(void)
{
    if (RCC_GetFlagStatus(RCC_FLAG_USBHSPLLRDY) == RESET)
    {
        uint32_t spins;

        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig(((RCC->CTLR & RCC_HSERDY) != 0u) ? RCC_USBHSPLLSource_HSE : RCC_USBHSPLLSource_HSI);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
        for (spins = 0u; spins < 2000000u; spins++)
        {
            if (RCC_GetFlagStatus(RCC_FLAG_USBHSPLLRDY) != RESET)
            {
                break;
            }
        }
        if (RCC_GetFlagStatus(RCC_FLAG_USBHSPLLRDY) == RESET)
        {
            return false;
        }
    }
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_USBHSPLL);
    RCC_USBFS48ClockSourceDivConfig(RCC_USBFS_Div10);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_OTG_FS, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
    return true;
}

void usbfs_dev_start(const UsbfsDevClass *cls, uint8_t irq_priority)
{
    NVIC_DisableIRQ(USBFS_IRQn);
    s_started = false;
    s_cls = cls;
    s_config = 0u;
    s_ctrl = CTRL_IDLE;
    s_addr_pending = false;
    s_in_zlp = false;
    s_reset_count = 0u;

    /*
     * Reset SIE first, then configure INT_EN -> BASE_CTRL -> EPs -> UDEV_CTRL
     * after releasing reset so register writes are not lost.
     */
    USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
    short_delay();
    USBFSH->BASE_CTRL = 0u;
    USBFSD->DEV_ADDR = 0u;
    USBFSD->INT_FG = 0xFFu;
    USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
    USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
    endpoints_init();
    USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;

    NVIC_SetPriority(USBFS_IRQn, irq_priority);
    NVIC_ClearPendingIRQ(USBFS_IRQn);
    s_started = true;
    LAT_FENCE_IORW();
    NVIC_EnableIRQ(USBFS_IRQn);
}

void usbfs_dev_stop(void)
{
    NVIC_DisableIRQ(USBFS_IRQn);
    (void)NVIC->ISR[((uint32_t)USBFS_IRQn) >> 5];
    s_started = false;
    s_config = 0u;
    /*
     * Reset SIE, release reset with BASE_CTRL=0 (clears DEV_PU_EN), then
     * clear INT_EN, UDEV_CTRL, and pending flags.
     */
    USBFSD->INT_EN = 0u;
    USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
    short_delay();
    USBFSH->BASE_CTRL = 0u;
    USBFSD->INT_EN = 0u;
    USBFSD->UDEV_CTRL = 0u;
    USBFSD->INT_FG = 0xFFu;
    NVIC_ClearPendingIRQ(USBFS_IRQn);
}

void usbfs_dev_lock(void)
{
    irq_lock();
}

void usbfs_dev_unlock(void)
{
    irq_unlock();
}

bool usbfs_dev_configured(void)
{
    return s_started && (s_config != 0u);
}

uint32_t usbfs_dev_reset_count(void)
{
    return s_reset_count;
}

uint32_t usbfs_dev_ep_reset_count(void)
{
    return s_ep_reset_count;
}

bool usbfs_ep_in_busy(uint8_t ep)
{
    return (ep < USBFS_EP_NUM) && s_in_busy[ep];
}

bool usbfs_ep_in_ready(uint8_t ep)
{
    return (s_cls != 0) && ep_is_in(ep) && s_started && (s_config != 0u) && !s_in_busy[ep] && !s_in_halt[ep];
}

bool usbfs_ep_in_send(uint8_t ep, const uint8_t *data, uint16_t len)
{
    bool ok = false;

    if ((s_cls == 0) || !ep_is_in(ep) || (len > USBFS_EP_PACKET_MAX))
    {
        return false;
    }
    irq_lock();
    if (s_started && (s_config != 0u) && !s_in_busy[ep] && !s_in_halt[ep])
    {
        lat_bytes_copy(s_ep_buf[ep], data, len);
        uep_set_tlen(ep, len);
        UEP_TCTRL(ep) = (uint8_t)((UEP_TCTRL(ep) & T_TOG) | T_RES_ACK);
        s_in_busy[ep] = true;
        ok = true;
    }
    irq_unlock();
    return ok;
}

void usbfs_ep_out_resume(uint8_t ep)
{
    if ((s_cls == 0) || !ep_is_out(ep))
    {
        return;
    }
    irq_lock();
    if (s_out_paused[ep])
    {
        s_out_paused[ep] = false;
        if (s_started && (s_config != 0u) && !s_out_halt[ep])
        {
            UEP_RCTRL(ep) = (uint8_t)((UEP_RCTRL(ep) & (uint8_t)~R_RES_MASK) | R_RES_ACK);
        }
    }
    irq_unlock();
}

bool usbfs_ep_out_paused(uint8_t ep)
{
    return (ep < USBFS_EP_NUM) && s_out_paused[ep];
}

void usbfs_dev_diag(UsbfsDiag *out)
{
    *out = s_diag;
    out->mis_st = USBFSD->MIS_ST;
    out->udev_ctrl = USBFSD->UDEV_CTRL;
    out->base_ctrl = USBFSD->BASE_CTRL;
    out->dev_addr = USBFSD->DEV_ADDR;
    out->ep0_tctrl = USBFSD->UEP0_TX_CTRL;
    out->ep0_rctrl = USBFSD->UEP0_RX_CTRL;
    out->ep0_tlen = USBFSD->UEP0_TX_LEN;
}

uint8_t usbfs_make_string_desc(uint8_t *out, uint32_t cap, const char *ascii)
{
    uint32_t len = 2u;

    if (cap < 2u)
    {
        return 0u;
    }
    while ((*ascii != '\0') && ((len + 2u) <= cap) && ((len + 2u) <= 254u))
    {
        out[len++] = (uint8_t)*ascii++;
        out[len++] = 0u;
    }
    out[0] = (uint8_t)len;
    out[1] = USBFS_DESC_STRING;
    return (uint8_t)len;
}

uint8_t usbfs_make_hex_string_desc(uint8_t *out, uint32_t cap, const uint8_t *bytes, uint32_t n)
{
    static const char hex[16] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    uint32_t len = 2u;
    uint32_t i;

    if (cap < 2u)
    {
        return 0u;
    }
    for (i = 0u; (i < n) && ((len + 4u) <= cap) && ((len + 4u) <= 254u); i++)
    {
        out[len++] = (uint8_t)hex[bytes[i] >> 4];
        out[len++] = 0u;
        out[len++] = (uint8_t)hex[bytes[i] & 0x0Fu];
        out[len++] = 0u;
    }
    out[0] = (uint8_t)len;
    out[1] = USBFS_DESC_STRING;
    return (uint8_t)len;
}
