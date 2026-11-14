/*
 * bl_proto.c - Bootloader 64-byte fixed-length request/response (see bl_proto.h).
 */
#include "bl_proto.h"

#include "bl_flash_hal.h"
#include "lat_bytes.h"

static bool s_reset_request;

static void resp_header(uint8_t *resp, uint8_t cmd, BlStatus st, uint32_t offset, uint16_t length)
{
    lat_bytes_fill(resp, 0u, BL_FRAME_SIZE);
    resp[0] = cmd;
    resp[1] = (uint8_t)st;
    lat_put_u32(&resp[2], offset);
    lat_put_u16(&resp[6], length);
}

static void handle_info(uint8_t *resp)
{
    const uint8_t *m = bl_hal_read_ptr(BL_APP_BASE + APP_META_OFFSET);

    resp_header(resp, BL_CMD_INFO, BL_STATUS_OK, BL_APP_BASE, BL_INFO_LEN);
    lat_put_u32(&resp[8], BL_APP_BASE);
    lat_put_u32(&resp[12], APP_IMAGE_SIZE_MAX);
    lat_put_u32(&resp[16], BL_FLASH_PAGE_SIZE);
    if (lat_get_u32(m) == APP_META_MAGIC)
    {
        lat_bytes_copy(&resp[20], m, 32u);
    }
    lat_put_u32(&resp[52], BL_FLASH_SECTOR_SIZE);
    lat_put_u16(&resp[56], (uint16_t)BL_VERSION_MAJOR);
    lat_put_u16(&resp[58], (uint16_t)BL_VERSION_MINOR);
}

void bl_proto_init(void)
{
    s_reset_request = false;
}

void bl_proto_handle(const uint8_t *req, uint32_t len, uint8_t resp[BL_FRAME_SIZE])
{
    uint8_t cmd;
    uint32_t offset;
    uint16_t length;
    const uint8_t *data;
    BlStatus st;

    if (len != BL_FRAME_SIZE)
    {
        resp_header(resp, (len != 0u) ? req[0] : 0u, BL_STATUS_ALIGN, 0u, 0u);
        return;
    }
    cmd = req[0];
    offset = lat_get_u32(&req[2]);
    length = lat_get_u16(&req[6]);
    data = &req[8];

    if ((req[1] != BL_TARGET_CH32) || (length > BL_FRAME_DATA_MAX))
    {
        resp_header(resp, cmd, BL_STATUS_RANGE, offset, 0u);
        return;
    }

    switch (cmd)
    {
    case BL_CMD_INFO:
        handle_info(resp);
        break;

    case BL_CMD_ERASE:
        if ((length != 0u) && (length != 4u))
        {
            resp_header(resp, cmd, BL_STATUS_RANGE, offset, 0u);
            break;
        }
        st = bl_flash_erase(offset, (length == 4u) ? lat_get_u32(data) : 0u);
        resp_header(resp, cmd, st, offset, 0u);
        break;

    case BL_CMD_WRITE:
        st = bl_flash_write(offset, data, length);
        resp_header(resp, cmd, st, bl_flash_next_address(), (uint16_t)bl_flash_page_fill());
        break;

    case BL_CMD_FINALIZE:
    {
        uint32_t calc = 0u;

        if ((offset != BL_APP_BASE) || (length != 8u))
        {
            resp_header(resp, cmd, BL_STATUS_RANGE, offset, 0u);
            break;
        }
        st = bl_flash_finalize(lat_get_u32(&data[0]), lat_get_u32(&data[4]), &calc);
        resp_header(resp, cmd, st, offset, 4u);
        lat_put_u32(&resp[8], calc);
        break;
    }

    case BL_CMD_RESET_APP:
    {
        app_metadata_t meta;

        if ((offset != BL_APP_BASE) || (length != 0u))
        {
            resp_header(resp, cmd, BL_STATUS_RANGE, offset, 0u);
            break;
        }
        if (!bl_flash_boot_allowed() || (bl_flash_check_app(&meta) != 0))
        {
            resp_header(resp, cmd, BL_STATUS_NO_APP, BL_APP_BASE, 0u);
            break;
        }
        resp_header(resp, cmd, BL_STATUS_OK, BL_APP_BASE, 0u);
        s_reset_request = true;
        break;
    }

    case BL_CMD_READ_UUID:
        if ((offset != 0u) || (length != 0u))
        {
            resp_header(resp, cmd, BL_STATUS_RANGE, offset, 0u);
            break;
        }
        resp_header(resp, cmd, BL_STATUS_OK, 0u, 16u);
        bl_port_uuid(&resp[8]);
        break;

    default:
        resp_header(resp, cmd, BL_STATUS_UNKNOWN_COMMAND, offset, 0u);
        break;
    }
}

bool bl_proto_take_reset_request(void)
{
    bool r = s_reset_request;

    s_reset_request = false;
    return r;
}
