/*
 * lat_pc_proto.h - PC communication protocol constants.
 *
 * Shared by V3F (framing, session, TX queue) and V5F (command handling).
 * Payloads must be accessed byte-by-byte via lat_bytes.h (no struct casts).
 */
#ifndef LAT_PC_PROTO_H
#define LAT_PC_PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Framing ---- */
#define LAT_PC_VERSION              ((uint8_t)1u)
#define LAT_PC_HEADER_LEN           (12u)
#define LAT_PC_CRC_LEN              (2u)
#define LAT_PC_PAYLOAD_MAX          (1152u)
#define LAT_PC_DECODED_MAX          (LAT_PC_HEADER_LEN + LAT_PC_PAYLOAD_MAX + LAT_PC_CRC_LEN)   /* 1166 */
#define LAT_PC_ENCODED_MAX          (1171u)
#define LAT_PC_WIRE_MAX             (1172u)   /* Includes 0x00 delimiter; RX buffer size */

/* Header offsets */
#define LAT_PC_OFF_VERSION          (0u)
#define LAT_PC_OFF_TYPE             (1u)
#define LAT_PC_OFF_FLAGS            (2u)
#define LAT_PC_OFF_STATUS           (3u)
#define LAT_PC_OFF_SEQ              (4u)
#define LAT_PC_OFF_LENGTH           (6u)
#define LAT_PC_OFF_SESSION          (8u)

/* ---- Frame types ---- */
#define LAT_PC_CMD_MIN              ((uint8_t)0x01u)
#define LAT_PC_CMD_MAX              ((uint8_t)0x3Fu)
#define LAT_PC_EVT_MIN              ((uint8_t)0x40u)
#define LAT_PC_EVT_MAX              ((uint8_t)0x7Fu)
#define LAT_PC_RESPONSE_BIT         ((uint8_t)0x80u)

/* ---- Result codes ---- */
#define LAT_PC_ST_OK                ((uint8_t)0x00u)
#define LAT_PC_ST_UNKNOWN_COMMAND   ((uint8_t)0x01u)
#define LAT_PC_ST_BAD_LENGTH        ((uint8_t)0x02u)
#define LAT_PC_ST_BAD_PARAM         ((uint8_t)0x03u)
#define LAT_PC_ST_BUSY              ((uint8_t)0x04u)
#define LAT_PC_ST_NO_DEVICE         ((uint8_t)0x05u)
#define LAT_PC_ST_NOT_READY         ((uint8_t)0x06u)
#define LAT_PC_ST_UNSUPPORTED       ((uint8_t)0x07u)
#define LAT_PC_ST_INTERNAL          ((uint8_t)0x08u)
#define LAT_PC_ST_SEQ_CONFLICT      ((uint8_t)0x09u)
#define LAT_PC_ST_STALE_OBJECT      ((uint8_t)0x0Au)

/* ---- Commands ---- */
#define LAT_PC_HELLO                ((uint8_t)0x01u)
#define LAT_PC_GET_STATUS           ((uint8_t)0x02u)
#define LAT_PC_GET_DEVICE_INFO      ((uint8_t)0x03u)
#define LAT_PC_GET_OBJECT           ((uint8_t)0x04u)
#define LAT_PC_SET_HOST_PROFILE     ((uint8_t)0x05u)
#define LAT_PC_GET_HOST_PROFILE     ((uint8_t)0x06u)
#define LAT_PC_SET_VENDOR_TABLE     ((uint8_t)0x07u)
#define LAT_PC_GET_VENDOR_TABLE     ((uint8_t)0x08u)
#define LAT_PC_REENUMERATE          ((uint8_t)0x09u)
#define LAT_PC_TRIGGER_MANUAL       ((uint8_t)0x0Au)
#define LAT_PC_MONITOR_START        ((uint8_t)0x0Bu)
#define LAT_PC_MONITOR_STOP         ((uint8_t)0x0Cu)
#define LAT_PC_SET_WATCH            ((uint8_t)0x0Du)
#define LAT_PC_TRIAL_START          ((uint8_t)0x0Eu)
#define LAT_PC_TRIAL_STOP           ((uint8_t)0x0Fu)
#define LAT_PC_POLL_TEST_START      ((uint8_t)0x10u)
#define LAT_PC_POLL_TEST_STOP       ((uint8_t)0x11u)
#define LAT_PC_SET_CALIBRATION      ((uint8_t)0x12u)
#define LAT_PC_GET_CALIBRATION      ((uint8_t)0x13u)
#define LAT_PC_CALIBRATION_MODE     ((uint8_t)0x14u)
#define LAT_PC_ENTER_BOOTLOADER     ((uint8_t)0x15u)
#define LAT_PC_READ_OBJECT          ((uint8_t)0x16u)
#define LAT_PC_CMD_LAST             LAT_PC_READ_OBJECT

/* Fixed request lengths (or LAT_PC_LEN_VARIABLE). */
#define LAT_PC_LEN_VARIABLE         (0xFFFFu)
#define LAT_PC_HELLO_REQ_LEN        (12u)
#define LAT_PC_HELLO_RESP_LEN       (62u)
#define LAT_PC_GET_STATUS_RESP_LEN  (52u)
#define LAT_PC_GET_OBJECT_REQ_LEN   (8u)
#define LAT_PC_GET_OBJECT_RESP_LEN  (16u)
#define LAT_PC_HOST_PROFILE_LEN     (8u)
#define LAT_PC_TRIGGER_MANUAL_LEN   (2u)
#define LAT_PC_MONITOR_START_LEN    (8u)
#define LAT_PC_SET_WATCH_LEN        (32u)
#define LAT_PC_TRIAL_START_LEN      (24u)
#define LAT_PC_RUN_STOP_LEN         (4u)
#define LAT_PC_POLL_TEST_START_LEN  (16u)
#define LAT_PC_CALIBRATION_LEN      (28u)
#define LAT_PC_CALIBRATION_MODE_LEN (8u)
#define LAT_PC_ENTER_BL_LEN         (4u)
#define LAT_PC_READ_OBJECT_REQ_LEN  (8u)
#define LAT_PC_READ_OBJECT_HDR_LEN  (16u)
#define LAT_PC_READ_OBJECT_MAX      (1024u)
#define LAT_PC_DEVICE_INFO_HDR_LEN  (24u)

/* HELLO */
#define LAT_PC_HELLO_MAGIC          ((uint32_t)0x5454414Cu) /* 'LATT' */
#define LAT_PC_CAP_MONITOR          ((uint32_t)0x01u)
#define LAT_PC_CAP_TRIAL            ((uint32_t)0x02u)
#define LAT_PC_CAP_POLL             ((uint32_t)0x04u)
#define LAT_PC_CAP_ENUM_RECORD      ((uint32_t)0x08u)
#define LAT_PC_CAP_CALIBRATION      ((uint32_t)0x10u)

/* ENTER_BOOTLOADER */
#define LAT_PC_BOOT_CONFIRM         ((uint32_t)0x424F4F54u) /* 'BOOT' */

/* GET_STATUS host_state */
#define LAT_PC_HOST_BOOTING         ((uint8_t)0u)
#define LAT_PC_HOST_DISCONNECTED    ((uint8_t)1u)
#define LAT_PC_HOST_DEBOUNCE        ((uint8_t)2u)
#define LAT_PC_HOST_RESET           ((uint8_t)3u)
#define LAT_PC_HOST_ENUMERATING     ((uint8_t)4u)
#define LAT_PC_HOST_CONFIGURED      ((uint8_t)5u)
#define LAT_PC_HOST_RECOVERING      ((uint8_t)6u)
#define LAT_PC_HOST_ERROR_HALT      ((uint8_t)7u)

/* run_kind */
#define LAT_PC_RUN_NONE             ((uint8_t)0u)
#define LAT_PC_RUN_TRIAL            ((uint8_t)1u)
#define LAT_PC_RUN_POLL             ((uint8_t)2u)
#define LAT_PC_RUN_CALIBRATION      ((uint8_t)3u)

/* Object types */
#define LAT_PC_OBJ_DEVICE_DESC      ((uint8_t)1u)
#define LAT_PC_OBJ_CONFIG_DESC      ((uint8_t)2u)
#define LAT_PC_OBJ_HID_REPORT_DESC  ((uint8_t)3u)
#define LAT_PC_OBJ_STRING_DESC      ((uint8_t)4u)
#define LAT_PC_OBJ_BOS_DESC         ((uint8_t)5u)
#define LAT_PC_OBJ_ENUM_RECORD      ((uint8_t)6u)
#define LAT_PC_OBJ_POLL_RAW         ((uint8_t)7u)
#define LAT_PC_OBJ_POLL_HISTOGRAM   ((uint8_t)8u)
#define LAT_PC_OBJ_POLL_LONG        ((uint8_t)9u)
/* Enumeration trace header */
#define LAT_PC_ENUM_RECORD_VERSION  ((uint16_t)2u)
#define LAT_PC_ENUM_RECORD_HDR_LEN  (16u)
#define LAT_PC_ENUM_RECORD_FIXED_LEN (53u)
#define LAT_PC_ENUM_RECORD_PREV_MAX (8192u)

/* Max raw polling samples (x4 <= OBJECT_BUF). */
#define LAT_PC_MAX_POLL_RAW_COUNT   ((uint32_t)16384u)

/* Vendor code table */
#define LAT_PC_VENDOR_TABLE_MAX     (64u)
#define LAT_PC_VENDOR_ENTRY_LEN     (8u)
#define LAT_PC_VENDOR_STATE_CODE    ((uint8_t)1u)
#define LAT_PC_VENDOR_STATE_NONE    ((uint8_t)2u)

/* ---- Events ---- */
#define LAT_PC_EVT_LOG              ((uint8_t)0x40u)
#define LAT_PC_EVT_DEVICE_ATTACHED  ((uint8_t)0x41u)
#define LAT_PC_EVT_DEVICE_DETACHED  ((uint8_t)0x42u)
#define LAT_PC_EVT_ENUM_PROGRESS    ((uint8_t)0x43u)
#define LAT_PC_EVT_DEVICE_READY     ((uint8_t)0x44u)
#define LAT_PC_EVT_MONITOR_REPORT   ((uint8_t)0x45u)
#define LAT_PC_EVT_TRIAL_RESULT     ((uint8_t)0x46u)
#define LAT_PC_EVT_RUN_DONE         ((uint8_t)0x47u)
#define LAT_PC_EVT_POLL_RESULT      ((uint8_t)0x49u)
#define LAT_PC_EVT_DROP_COUNTERS    ((uint8_t)0x4Au)
#define LAT_PC_EVT_RECOVERED        ((uint8_t)0x4Bu)
#define LAT_PC_EVT_ERROR            ((uint8_t)0x4Cu)

#define LAT_PC_RUN_DONE_LEN         (28u)
#define LAT_PC_POLL_RESULT_LEN      (88u)
#define LAT_PC_DROP_COUNTERS_LEN    (8u)
#define LAT_PC_ERROR_LEN            (8u)
#define LAT_PC_DEVICE_EVT_LEN       (8u)
#define LAT_PC_RECOVERED_LEN        (4u)

/* ERROR codes */
#define LAT_PC_ERR_EVENT_OVERFLOW   ((uint16_t)1u)
#define LAT_PC_ERR_IPC_ABI_MISMATCH ((uint16_t)2u)
#define LAT_PC_ERR_IRQ_ROUTE_FAILED ((uint16_t)3u)
#define LAT_PC_ERR_TIMER_CLOCK      ((uint16_t)4u)
#define LAT_PC_ERR_HUB_UNSUPPORTED  ((uint16_t)5u)
#define LAT_PC_ERR_ENUM_FAILED      ((uint16_t)6u)

/* completion_reason */
#define LAT_PC_DONE_NORMAL          ((uint8_t)0u)
#define LAT_PC_DONE_ABORTED         ((uint8_t)1u)
#define LAT_PC_DONE_DETACHED        ((uint8_t)2u)
#define LAT_PC_DONE_INTERNAL        ((uint8_t)3u)
#define LAT_PC_DONE_OVERFLOW        ((uint8_t)4u)

/* Timeout before declaring safe-event TX queue overflow (ms). */
#define LAT_PC_OVERFLOW_MS          (5000u)

#ifdef __cplusplus
}
#endif

#endif /* LAT_PC_PROTO_H */
