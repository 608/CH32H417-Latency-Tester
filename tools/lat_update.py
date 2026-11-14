#!/usr/bin/env python3
"""
lat_update.py - Update the tester application (V3F+V5F) over USB.

write steps:
  1. Validate image (default .pio/app.bin) metadata and CRC before touching hardware
  2. Find the tester on CDC (16C0:05E1) and verify with HELLO
  3. Switch to bootloader via ENTER_BOOTLOADER
  4. Wait for bootloader (16C0:05DC) via libusb, then send INFO, ERASE, WRITE,
     FINALIZE, RESET_APP
  5. Wait for CDC port reappearance and verify the new build with HELLO

libusb is imported lazily inside Bootloader so image validation works without it.
"""
import argparse
import ctypes
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from pathlib import Path

import lat_proto as proto

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IMAGE = PROJECT_ROOT / ".pio" / "app.bin"

# Tester application (CDC-ACM)
APP_VID = 0x16C0
APP_PID = 0x05E1

# Bootloader (vendor class, WinUSB)
BL_VID = 0x16C0
BL_PID = 0x05DC
INTERFACE = 0
EP_OUT = 0x01
EP_IN = 0x82
USB_PACKET_SIZE = 64
USB_DATA_SIZE = 56
LIBUSB_ERROR_TIMEOUT = -7

BL_CMD_INFO = 0x01
BL_CMD_ERASE = 0x02
BL_CMD_WRITE = 0x03
BL_CMD_FINALIZE = 0x04
BL_CMD_RESET_APP = 0x05
BL_CMD_READ_UUID = 0x07

TARGET_CH32 = 0x00
APP_BASE = 0x00010000
PAGE_SIZE = 256
INFO_LENGTH = 52
UUID_LENGTH = 16

# Metadata (must match src/shared/app_metadata.h and tools/make_app_image.py)
APP_META_OFFSET = 0x400
APP_META_FORMAT = "<IHHII12sI"
APP_META_SIZE = 32
APP_META_MAGIC = 0x4D54414C  # 'LATM'
APP_META_FORMAT_VERSION = 1
APP_META_CRC_FIELD = APP_META_OFFSET + 8
IMAGE_SIZE_MIN_EXCL = 0x20000
IMAGE_SIZE_MAX = 0x60000

STATUS_NAMES = {
    0x00: "OK",
    0x01: "RANGE",
    0x02: "ALIGN",
    0x03: "SEQUENCE",
    0x04: "NO_APP",
    0x05: "FLASH_FAIL",
    0x06: "BUSY",
    0x07: "CRC_FAIL",
    0x08: "TIMEOUT",
    0x09: "UNKNOWN_COMMAND",
}

PROGRESS_STEP = 8192
POLL_INTERVAL_S = 0.2


class BootloaderError(RuntimeError):
    pass


@dataclass(frozen=True)
class AppImage:
    fw_major: int
    fw_minor: int
    crc32: int
    image_size: int
    build: str
    data: bytes  # Bytes to send (from start of file up to image_size)


@dataclass(frozen=True)
class DeviceInfo:
    base: int
    maximum_size: int
    page_size: int
    metadata: bytes
    sector_size: int
    bl_major: int
    bl_minor: int


def status_name(status: int) -> str:
    return STATUS_NAMES.get(status, f"0x{status:02X}")


def validate_image(image: bytes) -> AppImage:
    """Validate app image metadata, length, and CRC using the same rules as BL FINALIZE."""
    if len(image) < APP_META_OFFSET + APP_META_SIZE:
        raise BootloaderError("image file is too short to contain the metadata at +0x400")
    magic, major, minor, stored_crc, image_size, build, format_version = struct.unpack_from(
        APP_META_FORMAT, image, APP_META_OFFSET)
    if magic != APP_META_MAGIC:
        raise BootloaderError(
            f"metadata magic is 0x{magic:08X}, expected 0x{APP_META_MAGIC:08X} ('LATM')")
    if format_version != APP_META_FORMAT_VERSION:
        raise BootloaderError(
            f"metadata format_version is {format_version}, expected {APP_META_FORMAT_VERSION}")
    if not IMAGE_SIZE_MIN_EXCL < image_size <= IMAGE_SIZE_MAX:
        raise BootloaderError(
            f"metadata image_size 0x{image_size:X} is out of range "
            f"(must be > 0x{IMAGE_SIZE_MIN_EXCL:X} and <= 0x{IMAGE_SIZE_MAX:X})")
    if image_size > len(image):
        raise BootloaderError(
            f"image file is shorter than image_size: {len(image)} < {image_size}")

    data = bytes(image[:image_size])
    work = bytearray(data)
    work[APP_META_CRC_FIELD:APP_META_CRC_FIELD + 4] = b"\x00\x00\x00\x00"
    calculated_crc = zlib.crc32(work) & 0xFFFFFFFF
    if calculated_crc != stored_crc:
        raise BootloaderError(
            f"metadata CRC mismatch: stored=0x{stored_crc:08X}, "
            f"calculated=0x{calculated_crc:08X}")
    return AppImage(major, minor, stored_crc, image_size, proto.decode_build(build), data)


def _import_libusb():
    try:
        import libusb
    except (ImportError, OSError) as error:
        raise BootloaderError(
            f"the 'libusb' Python package is not available: {error}") from error
    return libusb


class Bootloader:
    """Open bootloader (16C0:05DC) via libusb and exchange 64-byte fixed requests/responses.

    Waits up to wait_s seconds for appearance (0 means a single probe).
    """

    def __init__(self, timeout_ms: int, wait_s: float = 0.0):
        self.usb = _import_libusb()
        self.timeout_ms = timeout_ms
        self.wait_s = wait_s
        self.context = ctypes.POINTER(self.usb.context)()
        self.handle = None

    def __enter__(self):
        try:
            result = self.usb.init_context(ctypes.byref(self.context), None, 0)
            if result != 0:
                raise BootloaderError(f"libusb initialization failed: {result}")
            self._open_bootloader()
            return self
        except Exception:
            self.__exit__(None, None, None)
            raise

    def _open_bootloader(self) -> None:
        # On Windows, WinUSB binding may not be ready immediately after enumeration;
        # retry open/claim within the timeout.
        deadline = time.monotonic() + self.wait_s
        last_error = "not found"
        while True:
            handle = self.usb.open_device_with_vid_pid(self.context, BL_VID, BL_PID)
            if handle:
                result = self.usb.claim_interface(handle, INTERFACE)
                if result == 0:
                    self.handle = handle
                    return
                self.usb.close(handle)
                last_error = f"USB interface claim failed: {result}"
            if time.monotonic() >= deadline:
                break
            time.sleep(POLL_INTERVAL_S)
        raise BootloaderError(f"bootloader {BL_VID:04X}:{BL_PID:04X}: {last_error}")

    def __exit__(self, exc_type, exc, traceback):
        if self.handle:
            self.usb.release_interface(self.handle, INTERFACE)
            self.usb.close(self.handle)
            self.handle = None
        if self.context:
            self.usb.exit(self.context)
            self.context = None

    def _bulk_out(self, packet: bytes) -> None:
        buffer = (ctypes.c_ubyte * len(packet)).from_buffer_copy(packet)
        transferred = ctypes.c_int()
        result = self.usb.bulk_transfer(
            self.handle, EP_OUT, buffer, len(packet),
            ctypes.byref(transferred), self.timeout_ms)
        if result != 0 or transferred.value != len(packet):
            raise BootloaderError(
                f"USB OUT failed: result={result}, transferred={transferred.value}")

    def _bulk_in(self) -> bytes:
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        last_result = 0
        while time.monotonic() < deadline:
            buffer = (ctypes.c_ubyte * USB_PACKET_SIZE)()
            transferred = ctypes.c_int()
            result = self.usb.bulk_transfer(
                self.handle, EP_IN, buffer, USB_PACKET_SIZE,
                ctypes.byref(transferred), min(250, self.timeout_ms))
            last_result = result
            if result == 0 and transferred.value == USB_PACKET_SIZE:
                return bytes(buffer)
            if result == 0 and transferred.value == 0:
                continue
            if result == LIBUSB_ERROR_TIMEOUT:
                continue
            raise BootloaderError(
                f"USB IN failed: result={result}, transferred={transferred.value}")
        raise BootloaderError(f"USB IN timeout: last_result={last_result}")

    def request(self, command: int, offset: int = 0, payload: bytes = b"",
                timeout_ms: int | None = None) -> tuple[int, int, int, bytes]:
        if len(payload) > USB_DATA_SIZE:
            raise ValueError("request payload is too large")

        previous_timeout = self.timeout_ms
        if timeout_ms is not None:
            self.timeout_ms = timeout_ms
        try:
            packet = struct.pack("<BBIH", command, TARGET_CH32, offset, len(payload)) + payload
            packet += bytes(USB_PACKET_SIZE - len(packet))
            self._bulk_out(packet)
            response = self._bulk_in()
        finally:
            self.timeout_ms = previous_timeout

        response_command, status, response_offset, length = struct.unpack_from(
            "<BBIH", response, 0)
        if response_command != command:
            raise BootloaderError(f"unexpected response command: 0x{response_command:02X}")
        data = response[8:8 + min(length, USB_DATA_SIZE)]
        return status, response_offset, length, data


def require_ok(action: str, status: int) -> None:
    if status != 0:
        raise BootloaderError(f"{action} failed: {status_name(status)}")


def read_info(device: Bootloader) -> DeviceInfo:
    status, response_offset, length, data = device.request(BL_CMD_INFO)
    require_ok("INFO", status)
    if length < INFO_LENGTH or len(data) < INFO_LENGTH:
        raise BootloaderError(f"INFO response is too short: {length}")
    base, maximum_size, page_size = struct.unpack_from("<III", data, 0)
    sector_size = struct.unpack_from("<I", data, 44)[0]
    bl_major, bl_minor = struct.unpack_from("<HH", data, 48)
    if response_offset != base or base != APP_BASE:
        raise BootloaderError(f"unexpected application base: 0x{base:08X}")
    if maximum_size == 0 or page_size != PAGE_SIZE:
        raise BootloaderError(
            f"invalid INFO geometry: max={maximum_size}, page={page_size}")
    return DeviceInfo(base, maximum_size, page_size, data[12:44], sector_size,
                      bl_major, bl_minor)


def print_info(info: DeviceInfo) -> None:
    print(f"BOOTLOADER version={info.bl_major}.{info.bl_minor} base=0x{info.base:08X} "
          f"max=0x{info.maximum_size:X} page={info.page_size} sector={info.sector_size}")
    magic, major, minor, crc, image_size, build, format_version = struct.unpack(
        APP_META_FORMAT, info.metadata)
    if magic != APP_META_MAGIC:
        print("  app metadata: none (no valid application)")
        return
    print(f"  app metadata: fw={major}.{minor} build={proto.decode_build(build)} "
          f"crc=0x{crc:08X} image_size=0x{image_size:X} format={format_version}")


def read_uuid(device: Bootloader) -> bytes:
    status, _offset, length, data = device.request(BL_CMD_READ_UUID, 0)
    require_ok("READ_UUID", status)
    if length != UUID_LENGTH or len(data) < UUID_LENGTH:
        raise BootloaderError(f"invalid READ_UUID response length: {length}")
    return data[:UUID_LENGTH]


def write_image_data(device: Bootloader, image: AppImage, label: str) -> DeviceInfo:
    info = read_info(device)
    print_info(info)
    if image.image_size > info.maximum_size:
        raise BootloaderError(
            f"image_size 0x{image.image_size:X} exceeds the device maximum "
            f"0x{info.maximum_size:X}")
    base = info.base
    print(f"WRITE source={label} base=0x{base:08X} size={image.image_size} "
          f"expected_crc=0x{image.crc32:08X}")

    status, _offset, _length, _data = device.request(
        BL_CMD_ERASE, base, struct.pack("<I", image.image_size), timeout_ms=10000)
    require_ok("ERASE", status)

    written = 0
    next_progress = PROGRESS_STEP
    while written < image.image_size:
        chunk = image.data[written:written + USB_DATA_SIZE]
        status, _offset, _length, _data = device.request(
            BL_CMD_WRITE, base + written, chunk, timeout_ms=10000)
        require_ok(f"WRITE @0x{base + written:08X}", status)
        written += len(chunk)
        if written >= next_progress or written == image.image_size:
            print(f"  written {written}/{image.image_size}")
            next_progress += PROGRESS_STEP

    status, _offset, length, data = device.request(
        BL_CMD_FINALIZE, base, struct.pack("<II", image.crc32, image.image_size),
        timeout_ms=30000)
    calculated_crc = struct.unpack_from("<I", data.ljust(4, b"\x00"), 0)[0]
    require_ok("FINALIZE", status)
    if length < 4 or calculated_crc != image.crc32:
        raise BootloaderError(
            f"FINALIZE CRC mismatch: device=0x{calculated_crc:08X}, "
            f"expected=0x{image.crc32:08X}")
    print(f"FINALIZE OK crc=0x{calculated_crc:08X}")
    return info


def reset_to_application(device: Bootloader, base: int) -> None:
    status, _offset, _length, _data = device.request(
        BL_CMD_RESET_APP, base, timeout_ms=10000)
    require_ok("RESET_APP", status)
    print("RESET_APP OK")


def show_bootloader(device: Bootloader) -> None:
    print_info(read_info(device))
    print(f"  uuid={read_uuid(device).hex().upper()}")


def find_tester_ports() -> list[str]:
    from serial.tools import list_ports
    return sorted(port.device for port in list_ports.comports()
                  if port.vid == APP_VID and port.pid == APP_PID)


def connect_tester(port_arg: str | None, wait_s: float
                   ) -> tuple[proto.TesterLink, proto.HelloInfo, str]:
    """Open tester CDC port and send HELLO, retrying up to wait_s seconds (0 = once).

    On Windows, COM port creation or PermissionError may fail transiently right
    after re-enumeration; retry within the timeout.
    """
    deadline = time.monotonic() + wait_s
    last_error = f"tester not found (CDC {APP_VID:04X}:{APP_PID:04X})"
    while True:
        # Probe candidates with HELLO because other devices (e.g. RP2040 trigger) share
        # the same VID:PID, and COM port numbers may change across re-enumeration.
        candidates = find_tester_ports()
        names = ([port_arg] + [n for n in candidates if n != port_arg]) if port_arg else candidates
        for name in names:
            try:
                link = proto.TesterLink.open(name)
            except OSError as error:
                last_error = f"cannot open {name}: {error}"
                continue
            try:
                info = link.hello()
            except (proto.ProtocolError, OSError) as error:
                link.close()
                last_error = f"HELLO on {name} failed: {error}"
                continue
            if port_arg and name != port_arg:
                print(f"NOTE: {port_arg} did not answer; using {name}")
            return link, info, name
        if time.monotonic() >= deadline:
            raise BootloaderError(last_error)
        time.sleep(POLL_INTERVAL_S)


def print_tester(port_name: str, info: proto.HelloInfo, detail: bool = False) -> None:
    print(f"TESTER port={port_name} fw={info.fw_major}.{info.fw_minor} "
          f"build={info.build} uuid={info.uuid.hex().upper()}")
    if detail:
        print(f"  session=0x{info.session_id:08X} capabilities=0x{info.capabilities:08X} "
              f"max_payload={info.max_payload} max_poll_raw_count={info.max_poll_raw_count}")


def enter_bootloader(link: proto.TesterLink) -> None:
    try:
        status, _payload = link.command(
            proto.TYPE_ENTER_BOOTLOADER, struct.pack("<I", proto.BOOT_CONFIRM))
    except (proto.ProtocolTimeout, OSError) as error:
        # Tester may reboot and drop the port before the response arrives;
        # check whether the bootloader appears.
        print(f"WARNING: no response to ENTER_BOOTLOADER ({error}); "
              f"waiting for the bootloader anyway")
        return
    finally:
        link.close()
    if status == proto.STATUS_BUSY:
        raise BootloaderError(
            "tester is BUSY: a trial or poll test is running or its events are still "
            "pending; stop it, let the PC tool receive all events, then retry")
    if status != proto.STATUS_OK:
        raise BootloaderError(f"ENTER_BOOTLOADER failed: {proto.status_name(status)}")
    print("ENTER_BOOTLOADER OK")


def command_write(args) -> None:
    image = validate_image(args.image_path.read_bytes())
    print(f"IMAGE {args.image_path} fw={image.fw_major}.{image.fw_minor} build={image.build} "
          f"size=0x{image.image_size:X} crc=0x{image.crc32:08X}")
    # Verify libusb works before switching the tester into bootloader mode.
    _import_libusb()

    if not args.bootloader_only:
        try:
            link, hello, port_name = connect_tester(args.port, 0)
        except BootloaderError as error:
            raise BootloaderError(
                f"{error} (if the device is already in the bootloader, "
                f"use --bootloader-only)") from error
        print_tester(port_name, hello)
        enter_bootloader(link)

    print(f"waiting for the bootloader {BL_VID:04X}:{BL_PID:04X} (up to {args.wait_s:g} s)")
    with Bootloader(args.timeout_ms, args.wait_s) as device:
        info = write_image_data(device, image, str(args.image_path))
        reset_to_application(device, info.base)

    if args.no_wait_app:
        return
    print(f"waiting for the tester {APP_VID:04X}:{APP_PID:04X} (up to {args.wait_s:g} s)")
    try:
        link, hello, port_name = connect_tester(args.port, args.wait_s)
    except BootloaderError as error:
        raise BootloaderError(
            f"the image was written, but the tester did not come back: {error}") from error
    link.close()
    print_tester(port_name, hello)
    if hello.build != image.build:
        print(f"WARNING: tester build {hello.build} differs from the image build {image.build}")
    else:
        print("UPDATE OK (build matches the image)")


def command_info(args) -> None:
    if args.bootloader_only:
        with Bootloader(args.timeout_ms) as device:
            show_bootloader(device)
        return

    if args.port or find_tester_ports():
        link, info, port_name = connect_tester(args.port, 0)
        link.close()
        print_tester(port_name, info, detail=True)
        return

    print(f"no tester CDC port {APP_VID:04X}:{APP_PID:04X} found, trying the bootloader")
    with Bootloader(args.timeout_ms) as device:
        show_bootloader(device)


def main(argv: list[str]) -> None:
    parser = argparse.ArgumentParser(
        description="CH32H417 latency tester USB updater (app image via the bootloader)")
    parser.add_argument("command", choices=("write", "info"))
    parser.add_argument("--bin", dest="image_path", type=Path, default=DEFAULT_IMAGE,
                        help="application image (default: .pio/app.bin in the project)")
    parser.add_argument("--port",
                        help=f"tester CDC port (default: auto-detect {APP_VID:04X}:{APP_PID:04X})")
    parser.add_argument("--bootloader-only", action="store_true",
                        help="skip the tester stage; the device is already in the bootloader")
    parser.add_argument("--wait-s", type=float, default=15.0,
                        help="seconds to wait for the bootloader and for the tester "
                             "after the update (default: 15)")
    parser.add_argument("--no-wait-app", action="store_true",
                        help="do not wait for the tester after RESET_APP")
    parser.add_argument("--timeout-ms", type=int, default=5000,
                        help="USB transfer timeout in ms (default: 5000)")
    args = parser.parse_args(argv)

    if args.command == "info":
        command_info(args)
    else:
        command_write(args)


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except (BootloaderError, proto.ProtocolError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
