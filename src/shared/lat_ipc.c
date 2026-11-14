/*
 * lat_ipc.c - Shared SRAM IPC objects in NOLOAD sections (see lat_ipc.h).
 *
 * V3F zeros these sections at startup before starting V5F.
 */
#include "lat_ipc.h"

__attribute__((section(".usbhs_dma"), aligned(64), used)) LatUsbhsDma g_usbhs_dma;
__attribute__((section(".object_buf"), aligned(64), used)) LatObjectBuf g_object_buf;
__attribute__((section(".ipc_header"), aligned(64), used)) LatIpcHeader g_ipc_header;
__attribute__((section(".ipc_cmd"), aligned(64), used)) LatIpcCmdRing g_ipc_cmd;
__attribute__((section(".ipc_evt_safe"), aligned(64), used)) LatIpcEvtSafeRing g_ipc_evt_safe;
__attribute__((section(".ipc_evt_lossy"), aligned(64), used)) LatIpcEvtLossyRing g_ipc_evt_lossy;

const LatRing g_ring_cmd = {
    &g_ipc_cmd.head, g_ipc_cmd.data, LAT_IPC_CMD_DATA_BYTES / LAT_RING_UNIT
};
const LatRing g_ring_evt_safe = {
    &g_ipc_evt_safe.head, g_ipc_evt_safe.data, LAT_IPC_EVT_SAFE_DATA_BYTES / LAT_RING_UNIT
};
const LatRing g_ring_evt_lossy = {
    &g_ipc_evt_lossy.head, g_ipc_evt_lossy.data, LAT_IPC_EVT_LOSSY_DATA_BYTES / LAT_RING_UNIT
};
