/*
 * enum_cmd.c - Bridge between enumeration and PC command handler (v5f_cmd.c)
 */
#include "enum_cmd.h"

#include "lat_pc_proto.h"
#include "v5f_cmd_port.h"

#include "enum_port.h"
#include "v5f_cmd.h"
#include "win_enum.h"
#include "usbhs_host.h"
#include "tim9_time.h"

static EnumLogFn s_log;

void enum_cmd_init(EnumLogFn log)
{
    s_log = log;
    win_enum_init();
}

/* ---- v5f_cmd_port.h ---- */

uint8_t v5f_port_device_info(uint8_t *resp, uint32_t cap, uint32_t *len)
{
    return win_enum_device_info(resp, cap, len);
}

uint8_t v5f_port_reenumerate(void)
{
    return win_enum_reenumerate();
}

uint8_t v5f_port_enum_object(uint8_t kind, uint8_t index, uint8_t *dst, uint32_t cap, uint32_t *len)
{
    return win_enum_object(kind, index, dst, cap, len);
}

uint32_t v5f_port_enum_trace_previous(uint8_t *destination, uint32_t capacity)
{
    return usbhs_record_previous_export(destination, capacity);
}

/* ---- enum_port.h ---- */

void enum_port_log(const char *line)
{
    if (s_log != (EnumLogFn)0)
    {
        s_log(line);
    }
}

bool enum_port_emit(uint8_t pc_type, const uint8_t *body, uint32_t len)
{
    return v5f_cmd_emit_event(pc_type, body, len);
}

uint32_t enum_port_session(void)
{
    return v5f_cmd_session();
}

uint64_t enum_port_now(void)
{
    return tim9_now();
}

void enum_port_profile(WinHostProfile *out)
{
    const uint8_t *p = v5f_cmd_host_profile();

    out->post_requests = p[1];
    out->interval_mode = p[2];
    out->reset2_policy = p[3];
    out->app_switch = p[4];
    out->app_sony = p[5];
}

bool enum_port_vendor_lookup(uint16_t vid, uint16_t pid, uint16_t bcd_device, uint8_t *state, uint8_t *code)
{
    return v5f_cmd_vendor_lookup(vid, pid, bcd_device, state, code);
}

void enum_port_vendor_learn(uint16_t vid, uint16_t pid, uint16_t bcd_device, uint8_t state, uint8_t code)
{
    v5f_cmd_vendor_learn(vid, pid, bcd_device, state, code);
}
