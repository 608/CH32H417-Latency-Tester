#!/usr/bin/env python3
"""
check_placement.py - Verify ELF section and symbol placement for all 3 cores.

Default inputs:
  .pio/build/lat_bl/firmware.elf
  .pio/build/lat_v3f/firmware.elf
  .pio/build/lat_v5f/firmware.elf

Checks:
  1. All V5F executable code is in ITCM (0x200A0000-0x200BFFFF); only startup
     sections (.init, .text, .loadcode) and startup symbols may exist outside.
  2. All V3F and BL C functions are in RAM_CODE (0x20100000-0x2010FFFF) because
     BL programs Flash and must not execute from Flash.
  3. V3F RAM_CODE <= 64KB and V5F ITCM <= 128KB.
  4. Shared SRAM layout symbols (__lat_shm_*, shared object addresses/sizes,
     section bounds) match across all 3 ELFs and src/shared/lat_ipc.h.
  5. USBHS DMA buffers are in shared SRAM, not in V5F DTCM.

Exits with code 1 on any violation. Locates readelf under
~/.platformio/packages/toolchain-riscv*/bin/.
"""

import argparse
import glob
import os
import re
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LAT_IPC_H = PROJECT_ROOT / "src" / "shared" / "lat_ipc.h"

ITCM_START, ITCM_END = 0x200A0000, 0x200C0000          # V5F ITCM 128KB
DTCM_START, DTCM_END = 0x200C0000, 0x20100000          # V5F DTCM 256KB
RAM_CODE_START, RAM_CODE_END = 0x20100000, 0x20110000  # V3F/BL RAM_CODE 64KB
SHARED_SRAM_START, SHARED_SRAM_END = 0x20100000, 0x20180000

STARTUP_SECTIONS = {".init", ".text", ".loadcode"}
# Startup symbols placed in Flash and RAM_LOAD by startup_ch32h417_*.S.
STARTUP_SYMBOLS = {
    "_start", "_sinit", "_einit", "handle_reset", "_load_base",
    "_loadcode_vma_start", "_loadcode_vma_end", "BP",
}
# Size limits for startup sections to catch accidental C code placement.
STARTUP_SECTION_MAX = {".init": 0x10, ".text": 0x100, ".loadcode": 0x100}

SHM_REGIONS = ("usbhs_dma", "object_buf", "ipc_header", "ipc_cmd",
               "ipc_evt_safe", "ipc_evt_lossy", "spare", "boot_magic")
# Constant name suffixes in lat_ipc.h (LAT_SHM_<NAME>_ADDR / _SIZE).
SHM_HEADER_NAMES = {
    "usbhs_dma": "USBHS_DMA", "object_buf": "OBJECT_BUF",
    "ipc_header": "IPC_HEADER", "ipc_cmd": "IPC_CMD",
    "ipc_evt_safe": "IPC_EVT_SAFE", "ipc_evt_lossy": "IPC_EVT_LOSSY",
    "spare": "SPARE", "boot_magic": "BOOT_MAGIC",
}
# Shared objects and section boundary symbols linked by both V3F and V5F.
APP_SHARED_OBJECTS = ("g_usbhs_dma", "g_object_buf", "g_ipc_header",
                      "g_ipc_cmd", "g_ipc_evt_safe", "g_ipc_evt_lossy")
APP_SHARED_BOUNDS = tuple(
    "__%s_%s__" % (name, edge)
    for name in ("usbhs_dma", "object_buf", "ipc_header", "ipc_cmd",
                 "ipc_evt_safe", "ipc_evt_lossy")
    for edge in ("start", "end"))

SECTION_RE = re.compile(
    r"^\s*\[\s*(\d+)\]\s+(\S+)\s+(\S+)\s+([0-9a-f]{8})\s+([0-9a-f]{6,8})\s+"
    r"([0-9a-f]{6,8})\s+([0-9a-f]{2})\s*([A-Za-z]*)")
SYMBOL_RE = re.compile(
    r"^\s*\d+:\s+([0-9a-f]{8})\s+(\d+|0x[0-9a-f]+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)$")


class Report:
    def __init__(self):
        self.errors = []

    def fail(self, msg):
        self.errors.append(msg)
        print("  FAIL: " + msg)

    def ok(self, msg):
        print("  ok:   " + msg)


def find_readelf():
    patterns = ("~/.platformio/packages/toolchain-riscv/bin/*readelf*",
                "~/.platformio/packages/toolchain-riscv*/bin/*readelf*")
    for pattern in patterns:
        for path in sorted(glob.glob(os.path.expanduser(pattern))):
            if path.endswith((".exe", "readelf")):
                return path
    return None


def run_readelf(readelf, option, elf):
    result = subprocess.run([readelf, option, "-W", str(elf)],
                            capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        raise SystemExit("[check_placement] readelf %s failed for %s" % (option, elf))
    return result.stdout


class Elf:
    def __init__(self, readelf, path):
        self.path = path
        self.sections = []   # (name, addr, size, flags, index)
        self.symbols = []    # (name, value, size, type, ndx)
        for line in run_readelf(readelf, "-S", path).splitlines():
            m = SECTION_RE.match(line)
            if m:
                self.sections.append((m.group(2), int(m.group(4), 16),
                                      int(m.group(6), 16), m.group(8),
                                      int(m.group(1))))
        for line in run_readelf(readelf, "-s", path).splitlines():
            m = SYMBOL_RE.match(line)
            if m:
                size = int(m.group(2), 0)
                self.symbols.append((m.group(7), int(m.group(1), 16), size,
                                     m.group(3), m.group(6)))
        self.by_name = {}
        for sym in self.symbols:
            self.by_name.setdefault(sym[0], sym)

    def section_by_index(self, index):
        for sec in self.sections:
            if sec[4] == index:
                return sec
        return None

    def alloc_usage(self, start, end):
        """Total size of allocated sections placed in [start, end)."""
        total = 0
        for name, addr, size, flags, _ in self.sections:
            if "A" in flags and size > 0 and start <= addr < end:
                total += size
        return total

    def value(self, name):
        sym = self.by_name.get(name)
        return None if sym is None else sym[1]

    def size(self, name):
        sym = self.by_name.get(name)
        return None if sym is None else sym[2]


def in_range(addr, size, start, end):
    return start <= addr and addr + size <= end


def check_code_region(elf, label, start, end, report):
    """Check addresses of executable sections and C functions."""
    ok = True
    for name, addr, size, flags, index in elf.sections:
        if "X" not in flags or size == 0:
            continue
        if in_range(addr, size, start, end):
            continue
        if name in STARTUP_SECTIONS:
            limit = STARTUP_SECTION_MAX[name]
            if size > limit:
                report.fail("%s: startup section %s is 0x%X bytes (limit 0x%X) - "
                            "non-startup code may have been placed outside the "
                            "execution RAM" % (label, name, size, limit))
                ok = False
            for sym in elf.symbols:
                sname, value, ssize, stype, ndx = sym
                if ndx.isdigit() and int(ndx) == index and stype in ("FUNC", "NOTYPE") \
                        and sname and not sname.startswith("$") \
                        and sname not in STARTUP_SYMBOLS:
                    report.fail("%s: symbol %s at 0x%08X is in startup section %s "
                                "(outside the execution RAM)" % (label, sname, value, name))
                    ok = False
            continue
        report.fail("%s: executable section %s at 0x%08X (0x%X bytes) is outside "
                    "0x%08X-0x%08X" % (label, name, addr, size, start, end - 1))
        ok = False

    funcs = [s for s in elf.symbols if s[3] == "FUNC"]
    outside = [s for s in funcs if not in_range(s[1], max(s[2], 1), start, end)]
    for sname, value, ssize, _, _ in outside:
        report.fail("%s: function %s at 0x%08X (size %d) is outside 0x%08X-0x%08X"
                    % (label, sname, value, ssize, start, end - 1))
        ok = False
    if ok:
        report.ok("%s: all %d C functions and all executable sections are inside "
                  "0x%08X-0x%08X (startup code excepted)" % (label, len(funcs), start, end - 1))


def parse_header_constants(report):
    text = LAT_IPC_H.read_text(encoding="utf-8")
    consts = {}
    for m in re.finditer(r"#define\s+(LAT_SHM_\w+)\s+\(\(uint32_t\)(0x[0-9A-Fa-f]+)u\)", text):
        consts[m.group(1)] = int(m.group(2), 16)
    if not consts:
        report.fail("could not parse LAT_SHM_* constants from %s" % LAT_IPC_H)
    return consts


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bl", type=Path, default=PROJECT_ROOT / ".pio/build/lat_bl/firmware.elf")
    parser.add_argument("--v3f", type=Path, default=PROJECT_ROOT / ".pio/build/lat_v3f/firmware.elf")
    parser.add_argument("--v5f", type=Path, default=PROJECT_ROOT / ".pio/build/lat_v5f/firmware.elf")
    args = parser.parse_args()

    readelf = find_readelf()
    if readelf is None:
        print("[check_placement] ERROR: readelf not found under ~/.platformio/packages/toolchain-riscv*/bin")
        return 1
    for path in (args.bl, args.v3f, args.v5f):
        if not path.exists():
            print("[check_placement] ERROR: %s not found (run pio run first)" % path)
            return 1

    report = Report()
    elfs = {"bl": Elf(readelf, args.bl), "v3f": Elf(readelf, args.v3f), "v5f": Elf(readelf, args.v5f)}

    print("[check_placement] (1) V5F code placement (ITCM)")
    check_code_region(elfs["v5f"], "V5F", ITCM_START, ITCM_END, report)
    for name in ("main", "TIM9_IRQHandler"):
        value = elfs["v5f"].value(name)
        if value is None:
            report.fail("V5F: symbol %s not found" % name)
        elif not (ITCM_START <= value < ITCM_END):
            report.fail("V5F: %s at 0x%08X is not in ITCM" % (name, value))
        else:
            report.ok("V5F: %s at 0x%08X (ITCM)" % (name, value))

    print("[check_placement] (2) V3F / BL code placement (RAM_CODE)")
    check_code_region(elfs["v3f"], "V3F", RAM_CODE_START, RAM_CODE_END, report)
    check_code_region(elfs["bl"], "BL", RAM_CODE_START, RAM_CODE_END, report)

    print("[check_placement] (3) execution RAM usage")
    v3f_used = elfs["v3f"].alloc_usage(RAM_CODE_START, RAM_CODE_END)
    bl_used = elfs["bl"].alloc_usage(RAM_CODE_START, RAM_CODE_END)
    v5f_used = elfs["v5f"].alloc_usage(ITCM_START, ITCM_END)
    for label, used, limit in (("V3F RAM_CODE", v3f_used, RAM_CODE_END - RAM_CODE_START),
                               ("BL RAM_CODE", bl_used, RAM_CODE_END - RAM_CODE_START),
                               ("V5F ITCM", v5f_used, ITCM_END - ITCM_START)):
        msg = "%s used %d / %d bytes (%.1f%%)" % (label, used, limit, 100.0 * used / limit)
        if used > limit:
            report.fail(msg)
        else:
            report.ok(msg)

    print("[check_placement] V5F DTCM and stack")
    bss_end = elfs["v5f"].value("_ebss")
    stack_start = elfs["v5f"].value("_susrstack")
    stack_end = elfs["v5f"].value("_eusrstack")
    if bss_end is None or stack_start is None or stack_end is None:
        report.fail("V5F: DTCM / stack boundary symbols missing")
    elif not (DTCM_START <= bss_end <= stack_start < stack_end == DTCM_END):
        report.fail("V5F: DTCM data overlaps the stack or lies outside DTCM")
    else:
        report.ok("V5F DTCM free %d bytes before the %d-byte stack"
                  % (stack_start - bss_end, stack_end - stack_start))

    print("[check_placement] (4) shared SRAM layout identical across BL / V3F / V5F")
    consts = parse_header_constants(report)
    before = len(report.errors)
    for region in SHM_REGIONS:
        for kind, suffix in (("origin", "ADDR"), ("length", "SIZE")):
            sym = "__lat_shm_%s_%s" % (region, kind)
            values = {k: e.value(sym) for k, e in elfs.items()}
            if None in values.values():
                report.fail("%s missing in %s" % (sym, [k for k, v in values.items() if v is None]))
                continue
            if len(set(values.values())) != 1:
                report.fail("%s differs: %s" % (sym, {k: "0x%X" % v for k, v in values.items()}))
                continue
            const_name = "LAT_SHM_%s_%s" % (SHM_HEADER_NAMES[region], suffix)
            value = values["v3f"]
            if const_name not in consts:
                report.fail("%s not defined in lat_ipc.h" % const_name)
            elif consts[const_name] != value:
                report.fail("%s = 0x%X in the ELFs but %s = 0x%X in lat_ipc.h"
                            % (sym, value, const_name, consts[const_name]))
    if len(report.errors) == before:
        report.ok("all %d __lat_shm_* layout symbols match in BL / V3F / V5F and lat_ipc.h"
                  % (len(SHM_REGIONS) * 2))

    for sym in ("g_boot_magic", "__boot_magic_start__"):
        values = {k: (e.value(sym), e.size(sym)) for k, e in elfs.items()}
        if any(v[0] is None for v in values.values()) or len(set(values.values())) != 1:
            report.fail("%s differs or is missing: %s" % (sym, values))
        else:
            report.ok("%s = 0x%08X in BL / V3F / V5F" % (sym, values["v3f"][0]))

    before = len(report.errors)
    for sym in APP_SHARED_OBJECTS + APP_SHARED_BOUNDS:
        v3 = (elfs["v3f"].value(sym), elfs["v3f"].size(sym))
        v5 = (elfs["v5f"].value(sym), elfs["v5f"].size(sym))
        if v3[0] is None or v5[0] is None:
            report.fail("%s missing (v3f=%s v5f=%s)" % (sym, v3, v5))
        elif v3 != v5:
            report.fail("%s differs: v3f=0x%08X/%d v5f=0x%08X/%d" % (sym, v3[0], v3[1], v5[0], v5[1]))
    if len(report.errors) == before:
        report.ok("shared objects and section bounds identical in V3F / V5F: %s"
                  % ", ".join("%s=0x%08X" % (s, elfs["v3f"].value(s) or 0) for s in APP_SHARED_OBJECTS))

    print("[check_placement] (5) USBHS DMA buffers")
    dma = elfs["v5f"].value("g_usbhs_dma")
    dma_size = elfs["v5f"].size("g_usbhs_dma")
    if dma is None:
        report.fail("g_usbhs_dma not found in V5F")
    elif DTCM_START <= dma < DTCM_END:
        report.fail("g_usbhs_dma at 0x%08X is in V5F DTCM (USBHS DMA cannot reach DTCM)" % dma)
    elif not in_range(dma, dma_size, SHARED_SRAM_START, SHARED_SRAM_END) or dma % 4 != 0 or dma_size < 2048:
        report.fail("g_usbhs_dma 0x%08X/%d must be 4-byte aligned, >= 2048 bytes, in shared SRAM"
                    % (dma, dma_size))
    else:
        report.ok("g_usbhs_dma at 0x%08X, %d bytes, 4-byte aligned, shared SRAM" % (dma, dma_size))

    print()
    if report.errors:
        print("[check_placement] RESULT: FAIL (%d violation(s))" % len(report.errors))
        return 1
    print("[check_placement] RESULT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
