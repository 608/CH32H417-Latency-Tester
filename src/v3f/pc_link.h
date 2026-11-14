/*
 * pc_link.h - V3F side of the PC communication protocol.
 *
 * Hardware-independent framing, session/retransmission state, command dispatch,
 * and TX priority queue (responses > safe events > DROP_COUNTERS > lossy events).
 */
#ifndef LAT_PC_LINK_H
#define LAT_PC_LINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PcLinkStats
{
    uint32_t session_id;
    uint32_t rx_frames;       /* valid frames received */
    uint32_t rx_bad_cobs;
    uint32_t rx_bad_length;
    uint32_t rx_bad_crc;
    uint32_t rx_overrun;      /* frames exceeding 1171 bytes */
    uint32_t rx_dropped;      /* wrong session or command while busy */
    uint32_t tx_frames;
    uint32_t fwd_timeouts;    /* V5F forward timeouts */
    uint32_t v5f_stale;       /* stale V5F responses/events dropped */
    uint32_t drop_monitor;    /* dropped monitor reports in current session */
    uint32_t drop_log;        /* dropped log records in current session */
    bool overflow;            /* event overflow active */
    bool hello_pending;
} PcLinkStats;

/* Initialize all state on boot. */
void pc_link_init(void);

/* Feed received USB bytes from main loop. */
void pc_link_rx_bytes(const uint8_t *data, uint32_t len);

/* Poll V5F IPC, timeouts, session transitions, and bootloader handoff. */
void pc_link_poll(uint32_t now_ms);

/* Fill up to max TX bytes into out; returns bytes written (0 if idle). */
uint32_t pc_link_tx_fill(uint8_t *out, uint32_t max);

/* Notify that n bytes from pc_link_tx_fill() completed USB transfer. */
void pc_link_tx_done(uint32_t n);

/* Discard partial RX frame on USB bus reset or deconfigure. */
void pc_link_rx_reset(void);

/* True if TX data is pending (used for ZLP decision). */
bool pc_link_tx_pending(void);

void pc_link_get_stats(PcLinkStats *out);

/* Forward a V3F UART log line as LOG (0x40) if a session is active. */
void pc_link_push_log(const char *text, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* LAT_PC_LINK_H */
