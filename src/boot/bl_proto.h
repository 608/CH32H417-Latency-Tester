/*
 * bl_proto.h - Bootloader 64-byte fixed-length request/response protocol.
 *
 * Frame layout:
 *   Req:  +0 u8 command, +1 u8 target(0), +2 u32 offset, +6 u16 length, +8 data[56]
 *   Resp: +0 u8 command, +1 u8 status(BlStatus), +2 u32 offset, +6 u16 length, +8 data[56]
 * Commands:
 *   INFO      0x01  offset 0x10000, len 52 (base, max_size, page_size, metadata[32], sector_size, ver)
 *   ERASE     0x02  offset 0x10000, data u32 image_size (len 4, or 0 to omit)
 *   WRITE     0x03  offset = next addr, len 1..56; resp offset = next addr, len = page fill
 *   FINALIZE  0x04  offset 0x10000, data u32 crc32, u32 image_size (len 8); resp data u32 calc_crc
 *   RESET_APP 0x05  offset 0x10000; if valid, returns OK and reboots after response completes
 *   READ_UUID 0x07  offset 0, len 0; resp data = 16-byte chip UUID
 */
#ifndef LAT_BL_PROTO_H
#define LAT_BL_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#include "bl_flash.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BL_FRAME_SIZE        (64u)
#define BL_FRAME_DATA_MAX    (56u)

#define BL_CMD_INFO          ((uint8_t)0x01u)
#define BL_CMD_ERASE         ((uint8_t)0x02u)
#define BL_CMD_WRITE         ((uint8_t)0x03u)
#define BL_CMD_FINALIZE      ((uint8_t)0x04u)
#define BL_CMD_RESET_APP     ((uint8_t)0x05u)
#define BL_CMD_READ_UUID     ((uint8_t)0x07u)

#define BL_TARGET_CH32       ((uint8_t)0x00u)
#define BL_INFO_LEN          (52u)
#define BL_VERSION_MAJOR     (1u)
#define BL_VERSION_MINOR     (0u)

/* Chip-unique ID (implemented outside bl_proto.c). */
void bl_port_uuid(uint8_t out[16]);

void bl_proto_init(void);

/* Process one request; len != 64 returns BL_STATUS_ALIGN. */
void bl_proto_handle(const uint8_t *req, uint32_t len, uint8_t resp[BL_FRAME_SIZE]);

/* Returns true once after RESET_APP succeeds (reboot after USB response completes). */
bool bl_proto_take_reset_request(void);

#ifdef __cplusplus
}
#endif

#endif /* LAT_BL_PROTO_H */
