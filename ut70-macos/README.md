# ALIENTEK UT70 macOS read-only monitor

This directory contains Apple Silicon-compatible Python tools that only read
UT70 HID Input Reports. They never call HID `write()` or
`send_feature_report()`.

## Setup

```bash
brew install hidapi
cd ut70-macos
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

On this Mac, `hidapi` 0.15.0 provides a native `macosx_11_0_arm64` wheel, so
Rosetta is not needed. The separate Homebrew library is useful for other HID
bindings but the Python wheel used here is self-contained.

Check USB and HID enumeration:

```bash
system_profiler SPUSBDataType
hidutil list
.venv/bin/python ut70_scan.py
```

The connected unit was observed as:

- VID:PID: `19F5:6666`
- product / serial: `ATK UT70` / `ATKUT70`
- HID Usage Page / Usage: `0x8C / 0x01`
- no Report ID
- 64-byte Input Report and 64-byte Output Report
- no Feature Report
- descriptor: `05 8c 09 01 a1 01 09 03 15 00 26 00 ff 75 08 95 40 81 02 09 04 15 00 26 00 ff 75 08 95 40 91 02 c0`

## Raw capture

```bash
.venv/bin/python ut70_raw.py --count 10
.venv/bin/python ut70_raw.py --duration 60 --csv logs/raw.csv
.venv/bin/python ut70_raw.py --count 100 --quiet --csv logs/raw.csv
```

Each report is printed as timestamp, length, hex dump, and decimal byte array.
CSV contains the same untouched report. Changing a load and comparing these
captures is the supported path for investigating currently unknown fields.

## Real-time measurements

```bash
.venv/bin/python ut70.py
.venv/bin/python ut70.py --csv logs/measurements.csv
```

For `pdm_power_test`, opening its USB UART resets the board and captures the
state messages. Settled samples (two seconds after each transition by default)
are summarized on exit:

```bash
.venv/bin/python ut70.py --duration 70 \
  --serial /dev/cu.usbmodem813707033 \
  --display-hz 1 --csv logs/pdm_states.csv
```

Keeping the UART open can itself raise nRF54 power. For the final low-noise
measurement, reset the board and immediately run without `--serial`:

```bash
.venv/bin/python ut70.py --duration 65 --cycle-seconds 20 \
  --settle 3 --display-hz 1 --csv logs/pdm_states_quiet_uart.csv
```

Here the process start is treated as the S0 epoch, so launch it immediately
after a debugger reset. The settling exclusion also absorbs small launch-time
uncertainty.

The monitor displays voltage, current in mA with three decimal places, computed
power, min/max/mean, a time-window moving average, observed report rate,
checksum status, and optional firmware state. It handles Ctrl+C and retries a
disconnected UT70 by default (`--no-reconnect` disables this).

## Empirically verified report fields

Offsets are zero-based:

| Bytes | Meaning | Encoding |
|---|---|---|
| 0 | header | constant `0xEE` in observed measurement reports |
| 1-2 | unknown | observed as `08 3C`; not assigned speculatively |
| 3-6 | voltage | little-endian IEEE-754 binary32, volts |
| 7-22 | unknown | not yet decoded |
| 23-26 | current | little-endian IEEE-754 binary32, amperes |
| 27-62 | unknown | not yet decoded |
| 63 | checksum | `sum(bytes[0:63]) & 0xFF` |

For example, current bytes `3C 68 18 3C` decode as approximately
`0.009302195 A` (`9.302195 mA`). The conversion is therefore:

```python
current_A = struct.unpack_from("<f", report, 23)[0]
current_mA = current_A * 1000
```

Power in `ut70.py` is explicitly calculated as `voltage_V * current_A`; an
independent transmitted power field has not been proven. Unknown bytes remain
unimplemented rather than being assigned from a single capture.

## 0.1 mA resolution and limitations

The Input Report carries current as a float with substantially finer numeric
steps than 0.1 mA. Whether those lower bits represent useful ADC accuracy or
noise must be judged from repeated measurements and a reference load. The
state capture log documents the actual differences seen during this firmware
test; it does not claim UT70 calibration accuracy beyond its specification.

### Results from the connected hardware (2026-09-03)

The nRF54 was reset through CMSIS-DAP, then its UART was kept closed. A 65 s
capture produced 3,250 valid reports at 50.0 Hz. Every report passed the
observed additive checksum. Samples during the first three seconds after each
state boundary were excluded:

| State | Samples | Mean | Median | 5th-95th percentile |
|---|---:|---:|---:|---:|
| S0: shared rail, IMU and microphone off | 951 | 9.377 mA | 9.364 mA | 9.214-9.517 mA |
| S1: IMU power-down, microphone sleep | 850 | 9.536 mA | 9.540 mA | 9.386-9.685 mA |
| S2: IMU power-down, microphone active | 850 | 9.897 mA | 9.878 mA | 9.736-10.037 mA |

Thus `S1-S0 = 0.159 mA`, `S2-S1 = 0.361 mA`, and `S2-S0 = 0.520 mA`
for this run. One S0 transient reached 10.651 mA, which is why percentiles and
medians are included rather than relying only on min/max.

The 0.1 mA result is positive. These actual Input Reports contained, for
example:

| Decoded current | Raw bytes 23-26 |
|---:|---|
| 9.300134 mA | `97 5F 18 3C` |
| 9.400635 mA | `1F 05 1A 3C` |
| 9.499472 mA | `AC A3 1B 3C` |
| 9.600014 mA | `61 49 1D 3C` |

Two consecutive reports also decoded as 9.516375 and 9.445669 mA (a
0.070706 mA difference), with distinct current bytes. Therefore the HID data
is **not rounded to 1 mA**; it contains enough information to distinguish
0.1 mA steps. This demonstrates the same property sought for a display reading
such as `0.011 A`, although the load during this low-noise capture was around
9-10 mA rather than exactly 11 mA.

As a control, holding the firmware UART open raised all state readings by about
2.4 mA. That run still gave nearly identical deltas (`S1-S0 = 0.156 mA`,
`S2-S1 = 0.365 mA`), but its absolute readings are not used above. This is why
the recommended state measurement uses reset-timed labels with UART closed.

Public material describes 16-bit acquisition, 0.0001 A display/measurement
resolution in the relevant range, and selectable 4/10/50/200/500/1000 Hz
sampling. No public UT70 packet specification, SDK, macOS/Linux implementation,
or reproducible packet dump was found in the searches performed for this task.
The sample-rate control command and bytes 1-2, 7-22, and 27-62 remain unknown.
No Output Report has been sent to discover them.

The connected device was operating at 50 Hz. The public documents list the
other acquisition rates, but the HID command that selects them is not publicly
documented in the sources located here. In accordance with the read-only safety
rule, this project does not guess or transmit a rate-change command.

## Sources

- [ALIENTEK official UT70 download index](https://www.alientek.com/download/p-10-10.html)
  (manual, firmware, FAQ, and Windows PC software listings)
- [UT70 manual PDF mirror](https://manuals.plus/alientek/ut70-high-performance-usb-tester-manual.pdf)
- [UT70 English manual mirror](https://manuals.plus/ae/1005006837234427)
- [UT70 manual/PC-software notes including VID/PID](https://microsin.net/adminstuff/hardware/ut70-usb-tester-user-manual.html)
- [Switch Science UT70 product/specification page](https://www.switch-science.com/products/9419)
- [Python `hid` package installation notes](https://pypi.org/project/hid/)
- [Python `hidapi` binding](https://pypi.org/project/hidapi/)
