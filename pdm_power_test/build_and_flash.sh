#!/bin/bash
# Build and USB-flash the XIAO nRF54L15 Sense PDM power test.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$HOME/pdm-power-test-build}"
BOARD="xiao_nrf54l15/nrf54l15/cpuapp"
NCS_BASE="/opt/nordic/ncs/v2.9.2"
TOOLCHAIN_ROOT="/opt/nordic/ncs/toolchains"

WEST=()
if [ -x "$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3" ] &&
   "$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3" -m west --version >/dev/null 2>&1; then
	WEST=("$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3" -m west)
	export PATH="$TOOLCHAIN_ROOT/b8efef2ad5/bin:$PATH"
fi

if [ "${#WEST[@]}" -eq 0 ] && [ -d "$TOOLCHAIN_ROOT" ]; then
	for bin_dir in "$TOOLCHAIN_ROOT"/*/bin; do
		if [ -x "$bin_dir/python3" ] &&
		   "$bin_dir/python3" -m west --version >/dev/null 2>&1; then
			WEST=("$bin_dir/python3" -m west)
			export PATH="$bin_dir:$PATH"
			break
		fi
	done
fi

if [ "${#WEST[@]}" -eq 0 ] && command -v west >/dev/null 2>&1; then
	WEST=("$(command -v west)")
fi

if [ "${#WEST[@]}" -eq 0 ]; then
	echo "ERROR: west not found" >&2
	exit 1
fi

echo "Building $BOARD -> $BUILD_DIR"
cd "$NCS_BASE"
"${WEST[@]}" build -p always --sysbuild -b "$BOARD" "$SCRIPT_DIR" \
	--build-dir "$BUILD_DIR" -- -DBOARD_ROOT="$PROJECT_DIR"

HEX_FILE=""
for candidate in \
	"$BUILD_DIR/merged.hex" \
	"$BUILD_DIR/pdm_power_test/zephyr/zephyr.hex" \
	"$BUILD_DIR/zephyr/zephyr.hex"; do
	if [ -f "$candidate" ]; then
		HEX_FILE="$candidate"
		break
	fi
done
if [ -z "$HEX_FILE" ]; then
	echo "ERROR: expected hex not found under $BUILD_DIR" >&2
	exit 1
fi

PYOCD_CMD=""
if [ -n "${PYOCD:-}" ]; then
	PYOCD_CANDIDATES=("$PYOCD")
else
	PYOCD_CANDIDATES=(
		"$HOME/.pyenv/shims/pyocd"
		"/opt/homebrew/bin/pyocd"
		"/usr/local/bin/pyocd"
		"$(command -v pyocd 2>/dev/null || true)"
	)
fi

for candidate in "${PYOCD_CANDIDATES[@]}"; do
	if [ -x "$candidate" ] &&
	   "$candidate" list --targets 2>/dev/null |
		grep -qE '^[[:space:]]*nrf54l[[:space:]]'; then
		PYOCD_CMD="$candidate"
		break
	fi
done

if [ -z "$PYOCD_CMD" ]; then
	echo "ERROR: nRF54L対応のpyOCDが見つかりません。pyOCD 0.37.0以降をインストールしてください。" >&2
	echo "       python3 -m pip install 'pyocd>=0.37'" >&2
	exit 1
fi

echo "USB flashing $HEX_FILE with $PYOCD_CMD"
"$PYOCD_CMD" flash -t nrf54l "$HEX_FILE"

echo
echo "Done. Serial monitor at 115200 baud:"
echo '  screen $(ls /dev/tty.usbmodem* | head -1) 115200'
