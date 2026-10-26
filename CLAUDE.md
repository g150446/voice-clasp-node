# Claude / coding-agent notes for voice-clasp-node

Follow **`AGENTS.md`** at the repo root for all agent behavior (terminal lifecycle,
hardware tests, and nordic-main build rules).

## nordic-main quick facts (do not rediscover the hard way)

- **Board:** `xiao_ble/nrf52840/sense` only — never `xiao_ble/nrf52840`.
- **Build:** prefer `nordic-main/build_and_package_ota.sh` or
  `west build -p always --sysbuild -b xiao_ble/nrf52840/sense …`
- **Overlay:** `nordic-main/boards/xiao_ble_nrf52840_sense.overlay` applies only
  with the sense board; wrong board → DT errors that look like code bugs.
- **Details:** `AGENTS.md`, `nordic-main/AGENTS.md`, `docs/nordic_main_guide.md`
