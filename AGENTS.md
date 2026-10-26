# Repository agent instructions

## Terminal window lifecycle

- When an agent opens a macOS Terminal window or tab for a command that must run
  with Terminal permissions, record the exact window or tab identifier.
- Close that window or tab as soon as the command and its required verification
  have finished. Do not leave an idle Terminal session open for possible later
  reuse.
- Before the final response, close every Terminal window or tab opened by the
  agent during the task.
- Never close a Terminal window or tab that existed before the agent's task or
  was opened by the user. Resolve and close only the exact identifiers created
  by the agent.

## Interactive hardware tests

- Before starting a test that requires the user to perform a physical action,
  tell the user exactly what action will be required, how many trials will run,
  and what sound or message marks the start of each trial.
- Ask the user to confirm that they are ready, and do not launch the test or its
  countdown until the user explicitly confirms readiness.
- Run trials one at a time when the agent must interpret a result or give
  corrective guidance between trials. Ask for readiness again before resuming
  after an interruption or a materially changed test procedure.
- Run countdown-based interactive tests in a macOS Terminal window with the
  countdown, start cue (`GO` or equivalent), and final result visible to the
  user. Do not redirect all test output exclusively to a log file.
- When saving an interactive test log, use a display-preserving mechanism such
  as `tee` so the Terminal still shows live connection status, countdown, start
  cue, and results. The user should not need to request visible countdowns on
  subsequent tests.

## nordic-main (XIAO nRF52840 Sense) build — required reading

Primary firmware lives in `nordic-main/`. Full operational docs:
`docs/nordic_main_guide.md`. Local agent notes: `nordic-main/AGENTS.md`.
BLE OTAの必須注意事項は `docs/ota_update_notes.md` を参照すること。OTA実行時はこの文書の事前確認・macOS Terminal・完了検証を省略しない。

### Board target (do not guess)

| Correct | Wrong (common mistake) |
|---------|------------------------|
| `xiao_ble/nrf52840/sense` | `xiao_ble/nrf52840` |
| | `xiao_ble` alone |

- Overlay auto-applied only with the **sense** variant:
  `nordic-main/boards/xiao_ble_nrf52840_sense.overlay`
- That overlay defines `imu0`, mic power regulator, battery ADC (`zephyr,user`),
  and OTA flash partitions. Without it, the build fails with **DT** errors such as
  undeclared `DT_N_ALIAS_imu0_*`, `zephyr_user` io-channels, `msm261d3526hicpm_c_en`,
  or `slot0_partition` — these are **not** application logic bugs.
- Canonical name sources:
  - NCS: `zephyr/boards/seeed/xiao_ble/xiao_ble_nrf52840_sense.yaml`
    → `identifier: xiao_ble/nrf52840/sense`
  - Scripts: `nordic-main/build_and_package_ota.sh` (`BOARD` default)
  - Prior good build: `nordic-main/build/CMakeCache.txt` → `BOARD=xiao_ble/nrf52840/sense`

### Preferred build commands

Use the project scripts when possible (they pin NCS + board):

```bash
cd nordic-main
./build_and_package_ota.sh    # sysbuild + OTA bin → ota_update.bin (no flash)
./build_and_flash.sh          # UF2 flash path when present
```

Manual west (must match scripts):

```bash
# From NCS workspace (e.g. /opt/nordic/ncs/v2.9.2)
west build -p always --sysbuild -b xiao_ble/nrf52840/sense \
  /path/to/voice-clasp-node/nordic-main \
  --build-dir /path/to/voice-clasp-node/nordic-main/build
```

- **Always** `--sysbuild` for this app (`sysbuild.conf` enables MCUboot).
- Building **without** sysbuild breaks partition / flash_map / MCUmgr wiring.
- NCS version expected: **v2.9.2** (see `build_and_package_ota.sh`).

### Toolchain / PATH on this machine (typical)

`west` may not be on default PATH. Prefer the project script, or:

```bash
export ZEPHYR_SDK_INSTALL_DIR=/opt/nordic/ncs/toolchains/b8efef2ad5/opt/zephyr-sdk
export PATH="/opt/nordic/ncs/toolchains/b8efef2ad5/bin:\
/opt/nordic/ncs/toolchains/b8efef2ad5/Cellar/ninja/1.10.2/bin:\
/opt/nordic/ncs/toolchains/b8efef2ad5/Cellar/python@3.12/3.12.4/bin:$PATH"
cd /opt/nordic/ncs/v2.9.2
python3.12 -m west build ...
```

Or: `nrfutil sdk-manager toolchain launch --ncs-version v2.9.2 -- west build ...`

### Interpreting build failures

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `DT_N_ALIAS_imu0_*` / `zephyr_user` / mic GPIO undeclared | Board missing `/sense` | Use `xiao_ble/nrf52840/sense` |
| `slot0_partition` / flash_map errors with plain west | No sysbuild / wrong board | `--sysbuild` + sense board |
| `west: unknown command "build"` | Not inside NCS west workspace | `cd` to NCS root or use script |
| `ccache: command not found` | Toolchain bin not on PATH | Add NCS toolchain `bin/` to PATH |
| Kconfig mcuboot USB_CDC / FLASH_MAP noise on wrong board | Wrong board or stale build dir | Correct board; `-p always` clean rebuild |

### After code changes to nordic-main

1. Build with the correct board + sysbuild (script preferred).
2. Confirm `main.c` has no C errors (DT noise from wrong board is a red herring).
3. Do not flash or run hardware tests unless the user asks; follow interactive
   hardware test rules above when they do.
