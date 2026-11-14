/*
 * CH32H417 latency tester - V5F (real-time core) main
 *
 * Constraints:
 *   - Do not use SDK Delay_Us/Delay_Ms (overwrites SysTick1).
 *   - V3F owns RCC and IALLOCR; V5F must not write them.
 *   - ISRs must never block on IPC rings.
 */
#include <stdbool.h>
#include <stdint.h>

#include "ch32h417.h"

#include "lat_fmt.h"
#include "lat_bytes.h"
#include "lat_ipc.h"
#include "lat_pc_proto.h"
#include "tim9_time.h"
#include "usbhs_host.h"
#include "enum/win_enum.h"
#include "enum/enum_cmd.h"
#include "measure/measure.h"
#include "v5f_cmd.h"
#include "v5f_cmd_port.h"

#define V5F_TICKS_PER_MS      ((uint64_t)100000u)
#define V5F_ALIVE_LOG_MS      ((uint64_t)10000u)
#define V5F_LOG_LINE_MAX      (120u)
#define V5F_TRIGGER_PINS      ((uint32_t)0x00FFu)

typedef struct V5fState
{
    uint32_t heartbeat;
    uint32_t loop_count;
    uint32_t backsteps;
    uint64_t prev_now;
    uint64_t next_ms_tick;
    uint64_t next_alive_tick;
    uint32_t last_epoch_logged;
} V5fState;

static V5fState s_state;

static void v5f_log(const char *text)
{
    uint32_t len = 0u;

    while ((len < V5F_LOG_LINE_MAX) && (text[len] != '\0'))
    {
        len++;
    }
    (void)lat_ring_push_lossy(&g_ring_evt_lossy, LAT_EVT_LOG, 0u, text, len);
}

static void v5f_log_time(const char *prefix, uint64_t now)
{
    char buf[V5F_LOG_LINE_MAX + 1u];
    LatStr s;

    lat_str_init(&s, buf, sizeof(buf));
    lat_str_puts(&s, prefix);
    lat_str_puts(&s, " t=");
    lat_str_hex32(&s, (uint32_t)(now >> 32));
    lat_str_putc(&s, ':');
    lat_str_hex32(&s, (uint32_t)now);
    lat_str_puts(&s, " hb=");
    lat_str_u32(&s, s_state.heartbeat);
    lat_str_puts(&s, " isr=");
    lat_str_u32(&s, tim9_isr_count());
    lat_str_puts(&s, " upd=");
    lat_str_u32(&s, tim9_update_count());
    lat_str_puts(&s, " bk=");
    lat_str_u32(&s, s_state.backsteps);
    v5f_log(buf);
}

static void v5f_publish_status(void)
{
    LatV5fStatus *st = &g_ipc_header.v5f_status;

    lat_seqlock_write_begin(&st->seq);
    st->tim9_isr_count = tim9_isr_count();
    st->tim9_update_count = tim9_update_count();
    st->tim9_backsteps = s_state.backsteps;
    st->loop_count = s_state.loop_count;
    st->cmd_count = v5f_cmd_count();
    st->cmd_deferred = v5f_cmd_deferred();
    lat_seqlock_write_end(&st->seq);
}

/*
 * Drive open-drain triggers T0..T7 (PA0..PA7).
 * Release first (BSHR, Hi-Z), then assert low (BCR).
 */
void v5f_port_trigger_write(uint8_t release_mask, uint8_t assert_mask)
{
    if (release_mask != 0u)
    {
        GPIOA->BSHR = (uint32_t)release_mask & V5F_TRIGGER_PINS;
    }
    if (assert_mask != 0u)
    {
        GPIOA->BCR = (uint32_t)assert_mask & V5F_TRIGGER_PINS;
    }
}

void v5f_port_host_status(LatHostStatus *status)
{
    UsbhsPortInfo pi;
    UsbhsStats st;

    usbhs_port_info(&pi);
    usbhs_stats(&st);
    switch (pi.state)
    {
    case USBHS_PORT_DEBOUNCE:
        status->host_state = LAT_PC_HOST_DEBOUNCE;
        break;
    case USBHS_PORT_RESETTING:
        status->host_state = LAT_PC_HOST_RESET;
        break;
    case USBHS_PORT_ENABLED:
        status->host_state = pi.configured ? LAT_PC_HOST_CONFIGURED : LAT_PC_HOST_ENUMERATING;
        break;
    case USBHS_PORT_RECOVERING:
        status->host_state = LAT_PC_HOST_RECOVERING;
        break;
    case USBHS_PORT_HALTED:
        status->host_state = LAT_PC_HOST_ERROR_HALT;
        break;
    default:
        status->host_state = LAT_PC_HOST_DISCONNECTED;
        break;
    }
    status->speed = (pi.state == USBHS_PORT_DISCONNECTED) ? 0u : pi.speed;
    status->device_generation = pi.generation;
    status->enum_error = st.enum_error;
    status->sched_late_count = st.sched_late_count;
    status->sched_late_max = st.sched_late_max;
    status->txn_max_ticks = st.txn_max_ticks;
    status->recover_count = st.recover_count;
}

uint32_t v5f_port_enum_trace_export(uint8_t *destination, uint32_t capacity)
{
    return usbhs_record_export(destination, capacity);
}

void v5f_port_ping(const LatRingRecord *cmd)
{
    LatPingCmd ping = {0};
    LatPongEvt pong;
    uint64_t now;
    uint8_t *dst = (uint8_t *)&ping;
    uint32_t i;

    if (cmd->len < sizeof(ping))
    {
        return;
    }
    for (i = 0u; i < sizeof(ping); i++)
    {
        dst[i] = cmd->payload[i];
    }

    now = tim9_now();
    pong.ping_id = ping.ping_id;
    pong.v3f_ticks = ping.v3f_ticks;
    pong.tim9_lo = (uint32_t)now;
    pong.tim9_hi = (uint32_t)(now >> 32);
    pong.v5f_heartbeat = s_state.heartbeat;

    /* Space in safe ring was checked before popping. */
    (void)lat_ring_push_nowait(&g_ring_evt_safe, LAT_EVT_PONG, 0u, &pong, sizeof(pong));
}

int main(void)
{
    uint32_t diag;
    uint64_t now;

    /* V3F already configured clocks via SystemInit(); only refresh variables. */
    SystemAndCoreClockUpdate();

    /*
     * Do not touch other IPC header fields until validation passes.
     * Write v5f_diag only if the magic matched so V3F can report it.
     */
    diag = lat_ipc_v5f_validate();
    if (diag != LAT_V5F_DIAG_NONE)
    {
        if (diag != LAT_V5F_DIAG_MAGIC)
        {
            __atomic_store_n(&g_ipc_header.v5f_diag, diag, __ATOMIC_RELEASE);
        }
        for (;;)
        {
        }
    }

    tim9_time_init();
    measure_init();
    v5f_cmd_init();

    usbhs_host_init(v5f_log);
    enum_cmd_init(v5f_log);

    now = tim9_now();
    s_state.prev_now = now;
    s_state.next_ms_tick = now + V5F_TICKS_PER_MS;
    s_state.next_alive_tick = now + (V5F_ALIVE_LOG_MS * V5F_TICKS_PER_MS);
    s_state.last_epoch_logged = (uint32_t)(now >> 32);
    lat_ipc_time_publish(now);
    v5f_publish_status();

    lat_ipc_boot_state_store(LAT_BOOT_STATE_V5F_RUNNING);

    {
        char buf[V5F_LOG_LINE_MAX + 1u];
        LatStr s;

        lat_str_init(&s, buf, sizeof(buf));
        lat_str_puts(&s, "[v5f] running epoch=");
        lat_str_u32(&s, g_ipc_header.boot_epoch);
        lat_str_puts(&s, " core=");
        lat_str_u32(&s, SystemCoreClock);
        lat_str_puts(&s, " hclk=");
        lat_str_u32(&s, HCLKClock);
        lat_str_puts(&s, " tim9 psc=0 arr=FFFFFFFF");
        v5f_log(buf);
        if (HCLKClock != 100000000u)
        {
            v5f_log("[v5f] WARN hclk is not 100MHz - TIM9 tick is not 10ns");
        }
    }

    for (;;)
    {
        uint32_t epoch_now;

        now = tim9_now();
        if (now < s_state.prev_now)
        {
            s_state.backsteps++;
        }
        s_state.prev_now = now;
        s_state.loop_count++;

        v5f_cmd_poll();
        v5f_cmd_service();

        usbhs_host_poll(now);
        win_enum_poll(now);
        measure_poll(now);

        /* 1ms tick: heartbeat, time, and status publication. */
        if ((int64_t)(now - s_state.next_ms_tick) >= 0)
        {
            do
            {
                s_state.heartbeat++;
                s_state.next_ms_tick += V5F_TICKS_PER_MS;
            } while ((int64_t)(now - s_state.next_ms_tick) >= 0);

            lat_ipc_word_store(&g_ipc_header.v5f_heartbeat, s_state.heartbeat);
            lat_ipc_time_publish(now);
            v5f_publish_status();
            v5f_cmd_publish();
        }

        epoch_now = (uint32_t)(now >> 32);
        if (epoch_now != s_state.last_epoch_logged)
        {
            s_state.last_epoch_logged = epoch_now;
            v5f_log_time("[v5f] tim9 wrap", now);
        }

        if ((int64_t)(now - s_state.next_alive_tick) >= 0)
        {
            s_state.next_alive_tick += V5F_ALIVE_LOG_MS * V5F_TICKS_PER_MS;
            v5f_log_time("[v5f] alive", now);
        }
    }
}
