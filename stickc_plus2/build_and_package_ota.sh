#!/usr/bin/env bash
# Build StickC Plus2 firmware and copy OTA payload for mac_client/ota_updater.py
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [[ -z "${IDF_PATH:-}" ]]; then
  if [[ -f "${HOME}/esp/esp-idf/export.sh" ]]; then
    # shellcheck source=/dev/null
    source "${HOME}/esp/esp-idf/export.sh"
  else
    echo "IDF_PATH not set; source esp-idf export.sh first" >&2
    exit 1
  fi
fi

VER="$(tr -d '[:space:]' < stickc_plus2/VERSION)"
echo "Building HarnessNode-Plus2 version ${VER}"

# Ensure esp_app_desc.version picks up VERSION (CMake cache can stale PROJECT_VER).
export PROJECT_VER="$VER"
idf.py -DHN_BOARD=stickc_plus2 reconfigure
idf.py -DHN_BOARD=stickc_plus2 build

BIN="build/voice_bridge_ble.bin"
if [[ ! -f "$BIN" ]]; then
  echo "Missing $BIN" >&2
  exit 1
fi

cp -f "$BIN" stickc_plus2/ota_update.bin
SIZE=$(wc -c < stickc_plus2/ota_update.bin | tr -d ' ')
echo "OTA package: stickc_plus2/ota_update.bin (${SIZE} bytes) ver=${VER}"
echo "Flash first-time (partition change): idf.py -DHN_BOARD=stickc_plus2 -p PORT flash"
echo "BLE OTA: python3 mac_client/ota_updater.py --device HarnessNode-Plus2 stickc_plus2/ota_update.bin"
