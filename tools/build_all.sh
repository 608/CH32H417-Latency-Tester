#!/usr/bin/env bash
# tools/build_all.sh - Build all 3 environments, check placement, and generate images.
# Usage: bash tools/build_all.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(dirname "$SCRIPT_DIR")"
cd "$PROJ" || exit 1

if [ "$#" -ne 0 ]; then
    echo "ERROR: expected no options" >&2
    exit 2
fi

if command -v python.exe >/dev/null 2>&1; then
    PYTHON=python.exe
elif command -v python >/dev/null 2>&1; then
    PYTHON=python
else
    PYTHON=python3
fi

if "$PYTHON" -c "import platformio" >/dev/null 2>&1; then
    PIO=("$PYTHON" -m platformio)
elif command -v pio >/dev/null 2>&1; then
    PIO=(pio)
else
    PIO=("$PYTHON" -m platformio)
fi

STEPS=()
RESULTS=()
FAILED=0

run_step() {
    local name="$1"
    shift
    if [ "$FAILED" -ne 0 ]; then
        STEPS+=("$name")
        RESULTS+=("SKIP")
        return
    fi
    echo
    echo "==== [build_all] $name"
    if "$@"; then
        STEPS+=("$name")
        RESULTS+=("OK")
    else
        STEPS+=("$name")
        RESULTS+=("FAIL")
        FAILED=1
    fi
}

rm -f "$PROJ/.pio/app.bin" "$PROJ/.pio/combined.bin"
# PlatformIO does not relink when only linker scripts change; remove ELF/bin
# so linker ASSERTs are always re-evaluated while keeping object files.
for env in lat_bl lat_v3f lat_v5f; do
    rm -f "$PROJ/.pio/build/$env/firmware.elf" "$PROJ/.pio/build/$env/firmware.bin"
done

run_step "pio run lat_bl lat_v3f lat_v5f" "${PIO[@]}" run -e lat_bl -e lat_v3f -e lat_v5f
run_step "check_placement.py" "$PYTHON" tools/check_placement.py
run_step "make_app_image.py -> .pio/app.bin" "$PYTHON" tools/make_app_image.py
run_step "make_combined_image.py -> .pio/combined.bin" "$PYTHON" tools/make_combined_image.py

echo
echo "==== [build_all] summary"
for i in "${!STEPS[@]}"; do
    printf '  %-5s %s\n' "${RESULTS[$i]}" "${STEPS[$i]}"
done
if [ "$FAILED" -ne 0 ]; then
    rm -f "$PROJ/.pio/app.bin" "$PROJ/.pio/combined.bin"
    echo "RESULT: FAIL (no image was produced)"
    exit 1
fi
echo "RESULT: SUCCESS  $PROJ/.pio/combined.bin"
exit 0