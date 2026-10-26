#!/usr/bin/env bash
# Build StickC Plus SE firmware and copy OTA payload for mac_client/ota_updater.py
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

VER="$(tr -d '[:space:]' < stickc_plus_se/VERSION)"
echo "Building HarnessNode-PlusSE version ${VER}"

export PROJECT_VER="$VER"
idf.py -DHN_BOARD=stickc_plus_se -B build-plus_se reconfigure
idf.py -DHN_BOARD=stickc_plus_se -B build-plus_se build

BIN="build-plus_se/voice_bridge_ble.bin"
if [[ ! -f "$BIN" ]]; then
  echo "Missing $BIN" >&2
  exit 1
fi

cp -f "$BIN" stickc_plus_se/ota_update.bin
SIZE=$(wc -c < stickc_plus_se/ota_update.bin | tr -d ' ')
echo "OTA package: stickc_plus_se/ota_update.bin (${SIZE} bytes) ver=${VER}"
echo "Flash first-time: idf.py -DHN_BOARD=stickc_plus_se -B build-plus_se -p PORT flash"
echo "BLE OTA: python3 mac_client/ota_updater.py --device HarnessNode-PlusSE stickc_plus_se/ota_update.bin"
