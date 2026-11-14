/*
 * win_profile.h - Host timing profiles and step IDs
 *
 * All times are in TIM9 ticks (10ns). Hardware-independent.
 */
#ifndef LAT_WIN_PROFILE_H
#define LAT_WIN_PROFILE_H

#include <stdint.h>

#include "usbhs_host.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WP_US(x)                  ((uint32_t)((x) * 100.0 + 0.5))
#define WP_MS(x)                  ((uint32_t)((x) * 100000.0 + 0.5))

/* Host profile constants */
#define WP_APP_NONE               ((uint8_t)0u)
#define WP_APP_INPUT_MINIMAL      ((uint8_t)1u)
#define WP_APP_SDL_OPEN           ((uint8_t)2u)

typedef struct WinHostProfile
{
    uint8_t post_requests;    /* 1: send post-enum requests */
    uint8_t interval_mode;    /* 0: round down to power of 2, 1: raw descriptor value */
    uint8_t reset2_policy;    /* 0: second reset when bcdUSB < 0x0201 */
    uint8_t app_switch;       /* WP_APP_* */
    uint8_t app_sony;         /* WP_APP_NONE or WP_APP_SDL_OPEN */
} WinHostProfile;

typedef struct WinTiming
{
    uint32_t debounce;
    uint8_t  long_reset;      /* 1 = BUS_RST_LONG (~18ms), 0 = short (~16ms) */
    uint32_t first;
    uint32_t reset2_gap;
    uint32_t addr;
    uint32_t addr_noreset;
    uint32_t recovery;
    uint32_t cfg;
    uint32_t handover;
    uint32_t cfg_settle;
    uint32_t ctrl_timeout;
    uint8_t  hub_next_frame;  /* 1: align hub H7..H11 to next SOF + 7us */
    uint8_t  addr_fixed;      /* Non-zero: fixed SET_ADDRESS value; 0: increment */
    uint32_t slot_in_fs;
    uint32_t slot_out_fs;
    uint32_t slot_in_hs;
    uint32_t slot_out_hs;
} WinTiming;

/* High-Speed periodic slot offset from SOF (placed right after SOF like EHCI). */
#define WP_HS_PERIODIC_SLOT       WP_US(2)

#define WP_PCW_TIMING_INIT { \
    WP_MS(107.5), 0u, WP_MS(67.9), WP_US(316), WP_MS(64.5), WP_US(68), WP_MS(38.0), WP_US(1916), \
    WP_MS(30.78), WP_MS(10.42), WP_MS(5000), 0u, 0u, WP_US(625), WP_US(3.5), WP_HS_PERIODIC_SLOT, WP_HS_PERIODIC_SLOT }

#define WP_PCW_HUB_GAP            WP_US(60)
#define WP_PCW_NAK                WP_US(10)

#define WP_C1_CFG9_GAP            WP_US(130)
#define WP_C1_CFGALL_GAP          WP_US(96)
#define WP_C2_GAP                 WP_US(1672)
#define WP_C3_FULL_GAP            WP_US(77)
#define WP_C4_FIRST_GAP           WP_US(1103)
#define WP_C4_NEXT_IF_GAP         WP_US(3092)
#define WP_C4_SECOND_GAP          WP_US(105)
#define WP_C5_GAP                 WP_US(2066)
#define WP_X1_AT                  WP_US(17475)
#define WP_X2_AT                  WP_US(17957)
#define WP_X2B_GAP                WP_US(70)
#define WP_X3_AT                  WP_US(18313)
#define WP_X4_AT                  WP_US(18671)
#define WP_X5_AT                  WP_US(18794)
#define WP_X6_AT                  WP_US(19400)
#define WP_X7_AT                  WP_US(24300)
#define WP_ANCHOR_MIN_GAP         WP_US(60)
#define WP_D1_AFTER_X5_GAP        WP_US(66)
#define WP_D2_GAP                 WP_US(126)
#define WP_D3A_DELAY              WP_US(1350)
#define WP_D4_DELAY               WP_US(400)
#define WP_LED_OUT_AT             WP_US(60300)
#define WP_LED_SETREPORT_AT       WP_US(56500)
#define WP_APP_START_AT           WP_MS(45)
#define WP_G2_GAP                 WP_US(100)
#define WP_INTR_ERR_IMMEDIATE     (3u)
#define WP_INTR_ERR_PAUSE         WP_MS(5000)
#define WP_INTR_TRACE_BUDGET      (64u)
/* Windows version reported in H9 MS OS 2.0 request (NTDDI_WIN10+). */
#define WP_WIN_VERSION            ((uint32_t)0x0A000000u)

/*
 * Trace step ID encoding:
 *   high byte: layer ('H','C','D','K','X','G','S','A','P','R')
 *   low byte:  (step_num << 4) | sub_index
 */
#define WS(layer, num, sub)       ((uint16_t)((((uint16_t)(layer)) << 8) | ((((uint16_t)(num)) & 0x0Fu) << 4) | ((uint16_t)(sub) & 0x0Fu)))
#define WS_POST(row)              ((uint16_t)(((uint16_t)'P' << 8) | ((uint16_t)(row) & 0xFFu)))

#ifdef __cplusplus
}
#endif

#endif /* LAT_WIN_PROFILE_H */
