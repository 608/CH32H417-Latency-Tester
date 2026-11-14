# tools/pio_v5f_lto.py - Use single-partition LTO for V5F linking (PlatformIO extra_scripts)
#
# Prevents "lto-wrapper: warning: using serial compilation of N LTRANS jobs"
# when V5F code grows large; build_flags -f... are not passed to the linker.
Import("env")  # noqa: F821  (provided by PlatformIO)

env.Append(LINKFLAGS=["-flto-partition=one", "-Wl,-Map=$BUILD_DIR/firmware.map"])  # noqa: F821
