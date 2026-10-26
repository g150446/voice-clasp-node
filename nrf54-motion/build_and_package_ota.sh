#!/bin/bash
# Build nrf54-motion and refresh the signed OTA payload without USB flashing.
#
# Use this when the device already has OTA-capable firmware installed and you
# only need a new ota_update.bin for BLE OTA.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$SCRIPT_DIR"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$HOME/nrf54-motion-build}"
BOARD="xiao_nrf54l15/nrf54l15/cpuapp"
NCS_BASE="/opt/nordic/ncs/v2.9.2"
TOOLCHAIN_ROOT="/opt/nordic/ncs/toolchains"

WEST=""
if [ -e "$TOOLCHAIN_ROOT/b8efef2ad5/bin/west" ] &&
   "$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3" -m west --version >/dev/null 2>&1; then
    WEST="$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3 -m west"
    export PATH="$TOOLCHAIN_ROOT/b8efef2ad5/bin:$PATH"
fi
if [ -z "$WEST" ] && [ -d "$TOOLCHAIN_ROOT" ]; then
    for _bin_dir in "$TOOLCHAIN_ROOT"/*/bin; do
        if [ -x "$_bin_dir/python3" ] &&
           "$_bin_dir/python3" -m west --version >/dev/null 2>&1; then
            WEST="$_bin_dir/python3 -m west"
            export PATH="$_bin_dir:$PATH"
            break
        fi
    done
fi
if [ -z "$WEST" ] && command -v west >/dev/null 2>&1; then
    WEST="$(command -v west)"
fi
if [ -z "$WEST" ]; then
    echo "ERROR: west not found. Install NCS v2.9.2 from https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation.html" >&2
    exit 1
fi
echo "west: $WEST"

echo "Building $BOARD -> $BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$NCS_BASE"
eval "$WEST" build -p always --sysbuild -b "$BOARD" "$APP_DIR" \
    --build-dir "$BUILD_DIR" \
    -- -DBOARD_ROOT="$PROJECT_DIR"

OTA_BIN="$BUILD_DIR/nrf54-motion/zephyr/zephyr.signed.bin"
if [ -f "$OTA_BIN" ]; then
    cp "$OTA_BIN" "$APP_DIR/ota_update.bin"
    echo "OTA payload refreshed: $APP_DIR/ota_update.bin ($(wc -c < "$APP_DIR/ota_update.bin") bytes)"
else
    echo "ERROR: Signed OTA payload not found at $OTA_BIN" >&2
    exit 1
fi

echo ""
echo "Done: built firmware and refreshed the OTA payload only (no USB flash)."
