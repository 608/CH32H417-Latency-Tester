#!/usr/bin/env python3
"""
make_app_image.py - Combine V3F and V5F binaries into a single application image
(app.bin) and populate metadata (src/shared/app_metadata.h).

Inputs:
  .pio/build/lat_v3f/firmware.bin   Placed at application start (Flash 0x10000)
  .pio/build/lat_v5f/firmware.bin   Placed at +0x20000 (Flash 0x30000)
Output:
  .pio/app.bin

Steps:
  1. Place V3F bin at +0 and V5F bin at +0x20000, filling the gap with 0xFF.
     image_size = 0x20000 + len(V5F bin); must be in (0x20000, 0x60000].
  2. Verify magic and format_version in metadata (32 bytes at +0x400).
  3. Write image_size, build ("YYYYMMDDHHMM", local time), and crc32.
     crc32 is CRC-32/ISO-HDLC (zlib.crc32) over [0, image_size) with the
     4-byte crc32 field zeroed (matching the bootloader check).
"""
import argparse
import struct
import sys
import zlib
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Must match src/shared/app_metadata.h.
V5F_OFFSET = 0x20000
IMAGE_SIZE_MIN_EXCL = 0x20000
IMAGE_SIZE_MAX = 0x60000
FILL_BYTE = 0xFF

APP_META_OFFSET = 0x400
APP_META_SIZE = 32
APP_META_MAGIC = 0x4D54414C  # 'LATM'
APP_META_FORMAT_VERSION = 1
CRC_FIELD = APP_META_OFFSET + 8
SIZE_FIELD = APP_META_OFFSET + 12
BUILD_FIELD = APP_META_OFFSET + 16
BUILD_LEN = 12
FORMAT_FIELD = APP_META_OFFSET + 28


class AppImageError(RuntimeError):
    pass


def crc32_with_field_zeroed(image: bytes) -> int:
    work = bytearray(image)
    work[CRC_FIELD:CRC_FIELD + 4] = b"\x00\x00\x00\x00"
    return zlib.crc32(bytes(work)) & 0xFFFFFFFF


def build_app_image(v3f_bin: bytes, v5f_bin: bytes, build_str: str) -> bytearray:
    if len(v3f_bin) > V5F_OFFSET:
        raise AppImageError(
            "V3F firmware.bin is %d bytes, larger than its 128KB window (0x%X)"
            % (len(v3f_bin), V5F_OFFSET))
    if len(v3f_bin) < APP_META_OFFSET + APP_META_SIZE:
        raise AppImageError("V3F firmware.bin (%d bytes) does not contain the metadata at +0x%X"
                            % (len(v3f_bin), APP_META_OFFSET))
    if len(v5f_bin) == 0:
        raise AppImageError("V5F firmware.bin is empty")

    total = V5F_OFFSET + len(v5f_bin)
    if not (IMAGE_SIZE_MIN_EXCL < total <= IMAGE_SIZE_MAX):
        raise AppImageError(
            "image size 0x%X is out of range (must be > 0x%X and <= 0x%X)"
            % (total, IMAGE_SIZE_MIN_EXCL, IMAGE_SIZE_MAX))

    image = bytearray([FILL_BYTE]) * total
    image[0:len(v3f_bin)] = v3f_bin
    image[V5F_OFFSET:V5F_OFFSET + len(v5f_bin)] = v5f_bin

    magic = struct.unpack_from("<I", image, APP_META_OFFSET)[0]
    if magic != APP_META_MAGIC:
        raise AppImageError(
            "metadata magic at +0x%X is 0x%08X, expected 0x%08X ('LATM') - check "
            "the .app_meta placement in ldscripts/Link_CH32H417_v3f.ld"
            % (APP_META_OFFSET, magic, APP_META_MAGIC))
    fmt = struct.unpack_from("<I", image, FORMAT_FIELD)[0]
    if fmt != APP_META_FORMAT_VERSION:
        raise AppImageError("metadata format_version is %d, expected %d"
                            % (fmt, APP_META_FORMAT_VERSION))
    if len(build_str) != BUILD_LEN or not build_str.isascii():
        raise AppImageError("build string must be %d ASCII characters" % BUILD_LEN)

    struct.pack_into("<I", image, SIZE_FIELD, total)
    image[BUILD_FIELD:BUILD_FIELD + BUILD_LEN] = build_str.encode("ascii")
    struct.pack_into("<I", image, CRC_FIELD, 0)
    crc = crc32_with_field_zeroed(bytes(image))
    struct.pack_into("<I", image, CRC_FIELD, crc)

    fw_major, fw_minor = struct.unpack_from("<HH", image, APP_META_OFFSET + 4)
    print("[make_app_image] v3f=%dB v5f=%dB image_size=0x%X (%d) fw=%d.%d build=%s crc32=0x%08X"
          % (len(v3f_bin), len(v5f_bin), total, total, fw_major, fw_minor, build_str, crc))
    return image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--v3f-bin", type=Path,
                        default=PROJECT_ROOT / ".pio" / "build" / "lat_v3f" / "firmware.bin")
    parser.add_argument("--v5f-bin", type=Path,
                        default=PROJECT_ROOT / ".pio" / "build" / "lat_v5f" / "firmware.bin")
    parser.add_argument("--out", type=Path, default=PROJECT_ROOT / ".pio" / "app.bin")
    parser.add_argument("--build", default=None,
                        help="12-character build string (default: local time YYYYMMDDHHMM)")
    args = parser.parse_args()

    for path in (args.v3f_bin, args.v5f_bin):
        if not path.exists():
            print("[make_app_image] ERROR: %s not found (run pio run first)" % path, file=sys.stderr)
            return 1

    build_str = args.build if args.build is not None else datetime.now().strftime("%Y%m%d%H%M")
    try:
        image = build_app_image(args.v3f_bin.read_bytes(), args.v5f_bin.read_bytes(), build_str)
    except AppImageError as exc:
        print("[make_app_image] ERROR: %s" % exc, file=sys.stderr)
        return 1

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(image)
    print("[make_app_image] wrote %s (%d bytes)" % (args.out, len(image)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
