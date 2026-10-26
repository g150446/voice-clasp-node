#!/usr/bin/env python3
"""tap_monitor.py — connect to HarnessNode and dump every TX notification.

Read-only: subscribes to the audio TX characteristic and prints raw hex plus a
decoded label for each event packet.  Exits after --duration seconds.

    venv/bin/python3 mac_client/tap_monitor.py --duration 60
"""

import argparse
import asyncio
import struct
import subprocess
import sys
from datetime import datetime

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "HarnessNode"
SHOW_GESTURE_DIAG = False
AUDIO_TX_UUID = "00000002-0000-1000-8000-00805f9b34fb"

NAMES = {
    0x01: "recording_start",
    0x02: "recording_stop",
    0x10: "motion_active",
    0x11: "motion_settled",
    0x12: "DOUBLE_TAP",
    0x14: "SINGLE_TAP",
    0x20: "sleep_enter",
    0x21: "sleep_wake",
    0x30: "gesture_diag",
    0x40: "driving_mode_ack",
    0xD0: "tap_diag",
    0xD1: "TAP_SRC_RAW",
    0xD2: "reg_ack",
}

AUDIO_RX_UUID = "00000003-0000-1000-8000-00805f9b34fb"

# 0xD0 payload byte order, after [ret]
DIAG_REGS = ["CTRL1_XL", "CTRL6_C", "TAP_CFG", "TAP_THS_6D",
             "INT_DUR2", "WAKE_UP_THS", "MD1_CFG", "INT1_CTRL"]
DIAG_EXPECT = {
    "CTRL1_XL": "ODR>=416Hz -> high nibble 0x6+",
    "TAP_CFG": "0x83 (INT_EN|TAP_Z|LIR)",
    "TAP_THS_6D": "0x08 (0.5g)",
    "INT_DUR2": "0x4a",
    "WAKE_UP_THS": "bit7 set (double enable)",
    "MD1_CFG": "bits 6+3 set (single+double)",
}


def cue(sound: str, phrase: str) -> None:
    """Audible marker so the tester knows exactly when a window opens/closes."""
    try:
        subprocess.run(["afplay", f"/System/Library/Sounds/{sound}.aiff"],
                       check=False, timeout=5)
        subprocess.run(["say", "-v", "Kyoko", phrase], check=False, timeout=10)
    except Exception:
        pass


def log(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}] {msg}", flush=True)


def on_notify(_sender, data: bytes) -> None:
    if len(data) < 3 or data[0] != 0x00 or data[1] != 0x55:
        # Audio frames and anything else: count only, do not spam.
        return

    code = data[2]
    name = NAMES.get(code, f"unknown_0x{code:02x}")
    extra = ""
    if code in (0x10, 0x11) and len(data) >= 15:
        x, y, z = struct.unpack_from("<fff", data, 3)
        extra = f"  x={x:+.2f} y={y:+.2f} z={z:+.2f}"
    elif code == 0x30 and len(data) >= 17:
        stage, reason = data[3], data[4]
        a, b, c = struct.unpack_from("<fff", data, 5)
        extra = f"  stage={stage} reason={reason} {a:+.2f} {b:+.2f} {c:+.2f}"
        if not SHOW_GESTURE_DIAG:
            return
    elif code == 0xD0 and len(data) >= 14:
        ret = struct.unpack_from("<b", data, 3)[0]
        regs = " ".join(f"{n}=0x{data[4 + i]:02x}"
                        for i, n in enumerate(DIAG_REGS))
        nonzero = data[12] | (data[13] << 8)
        int1 = struct.unpack_from("<b", data, 14)[0] if len(data) >= 15 else "?"
        extra = (f"  read_ret={ret} nonzero_taps={nonzero} INT1={int1}  {regs}")
    elif code == 0xD2 and len(data) >= 6:
        reg = data[3]
        val = data[4]
        ret = struct.unpack_from("<b", data, 5)[0]
        extra = f"  reg 0x{reg:02x} -> 0x{val:02x} (ret={ret})"
    elif code == 0xD1 and len(data) >= 5:
        src = data[3]
        ret = struct.unpack_from("<b", data, 4)[0]
        bits = []
        for bit, label in ((6, "TAP_IA"), (5, "SINGLE"), (4, "DOUBLE"),
                           (3, "SIGN_NEG"), (2, "X"), (1, "Y"), (0, "Z")):
            if src & (1 << bit):
                bits.append(label)
        extra = f"  TAP_SRC=0x{src:02x} [{'|'.join(bits) or 'none'}] ret={ret}"

    marker = " <<<<<" if code in (0x12, 0x14, 0xD1) else ""
    log(f"{name}{extra}  raw={data.hex()}{marker}")


async def main_async(duration: float, scan_timeout: float, writes) -> int:
    log(f"Scanning for {DEVICE_NAME!r} ({scan_timeout:.0f}s)...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=scan_timeout)
    if device is None:
        log(f"NOT FOUND: {DEVICE_NAME} is not advertising.")
        return 2

    log(f"Found {device.address} — connecting...")
    async with BleakClient(device) as client:
        log("Connected. Subscribing to TX notifications.")
        await client.start_notify(AUDIO_TX_UUID, on_notify)
        for spec in writes:
            reg_s, _, val_s = spec.partition(":")
            reg, val = int(reg_s, 0), int(val_s, 0)
            log(f"-> write IMU reg 0x{reg:02x} = 0x{val:02x}")
            await client.write_gatt_char(
                AUDIO_RX_UUID, bytes([0x50, reg, val]), response=True)
            await asyncio.sleep(0.3)
        log(f"Listening for {duration:.0f}s — tap the board now.")
        cue("Glass", "テスト開始。タップしてください")
        end = asyncio.get_event_loop().time() + duration
        while client.is_connected and asyncio.get_event_loop().time() < end:
            await asyncio.sleep(0.1)
        log("Done listening.")
        cue("Submarine", "テスト終了")
    log("Disconnected.")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor HarnessNode TX events")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--scan-timeout", type=float, default=15.0)
    parser.add_argument(
        "--write", action="append", default=[], metavar="REG:VAL",
        help="poke an IMU register after connect, e.g. --write 0x15:0x00")
    args = parser.parse_args()
    sys.exit(asyncio.run(main_async(args.duration, args.scan_timeout, args.write)))


if __name__ == "__main__":
    main()
