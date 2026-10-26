# nordic-main agent notes

Firmware for **Seeed XIAO nRF52840 Sense** (BLE name `ClaspNode`; `HarnessNode` before 0.0.117).

## Board (mandatory)

```
xiao_ble/nrf52840/sense
```

| Do | Do not |
|----|--------|
| `-b xiao_ble/nrf52840/sense` | `-b xiao_ble/nrf52840` |
| `--sysbuild` (MCUboot) | plain app-only west build for release/OTA |
| `./build_and_package_ota.sh` | invent board names from partial DTS paths |

### Why `/sense` matters

Zephyr maps board → DTS + **board-named overlay**:

- Board file stem: `xiao_ble_nrf52840_sense`
- Overlay in this app: `boards/xiao_ble_nrf52840_sense.overlay`

That overlay supplies:

- `aliases { imu0 = &lsm6ds3tr_c; }` — `main.c` uses `DT_ALIAS(imu0)`
- Mic enable regulator `msm261d3526hicpm_c_en`
- `zephyr,user` battery ADC + enable GPIO
- MCUboot slot partitions used by MCUmgr / img manager

If you build `xiao_ble/nrf52840` (no sense), the overlay is **not** applied.
You get floods of `devicetree.h` / `device.h` errors. **Fix the board string**;
do not “fix” those symbols in C.

Confirm identity:

```text
NCS: zephyr/boards/seeed/xiao_ble/xiao_ble_nrf52840_sense.yaml
  identifier: xiao_ble/nrf52840/sense
This tree: boards/xiao_ble_nrf52840_sense.overlay
Scripts: BOARD default in build_and_package_ota.sh
Good cache: build/CMakeCache.txt → BOARD=xiao_ble/nrf52840/sense
```

## Build

```bash
# Preferred (NCS discovery + west + sysbuild + board default)
./build_and_package_ota.sh

# Equivalent core
west build -p always --sysbuild -b xiao_ble/nrf52840/sense \
  "$PWD" --build-dir "$PWD/build"
```

- SDK: **nRF Connect SDK v2.9.2**
- Outputs: `build/merged.hex`, `build/nordic-main/zephyr/zephyr.signed.bin`,
  script copies OTA payload to `ota_update.bin`
- Repo-wide agent rules: `../AGENTS.md`
- OTA実行前に `../docs/ota_update_notes.md` を必ず読む。特に署名バージョンの繰り上げ、macOS CoreBluetoothのTerminal実行、slot 0のactive/confirmed検証を省略しない。

## Code orientation

- Gesture + BLE + motion: `src/main.c`
- DMIC: `src/audio_capture.c`
- Gesture design notes: `../docs/flex_pronation_gesture.md`
- Ops guide: `../docs/nordic_main_guide.md`
