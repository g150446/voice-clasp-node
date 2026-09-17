#!/usr/bin/env python3
"""enter_bootloader.py — put HarnessNode into the Adafruit UF2 bootloader over BLE.

MCUboot lives at 0x27000 and MCUmgr only ever writes the app's secondary slot, so
changing MCUboot itself needs a UF2 write.  The documented way in was a double-tap
of the reset button, which is fiddly and needs the board in hand; this sends RX
`0x60 0xB0 0x07` instead and the firmware reboots with GPREGRET=0x57.

    venv/bin/python3 mac_client/enter_bootloader.py                 # just open it
    venv/bin/python3 mac_client/enter_bootloader.py flash.uf2       # open and write

Timing matters.  The nRF52840 watchdog has no STOP task and survives a soft
reset, so unless the bootloader feeds it the mass-storage volume may only live
for WDT_TIMEOUT_MS (12 s) after the reboot.  This script is therefore already
waiting on the mount point before it sends the command, and copies the moment the
volume appears.  It prints how long the copy took so you can see the margin.

Run it from a Terminal window: macOS only grants Bluetooth access to processes
that inherit Terminal's TCC permission.
"""

import argparse
import asyncio
import os
import shutil
import sys
import time
from datetime import datetime

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "HarnessNode"
AUDIO_TX_UUID = "00000002-0000-1000-8000-00805f9b34fb"
AUDIO_RX_UUID = "00000003-0000-1000-8000-00805f9b34fb"

CMD_REBOOT_BOOTLOADER = bytes([0x60, 0xB0, 0x07])
EVT_REBOOT_ACK = 0xD4

VOLUME = "/Volumes/XIAO-SENSE"
# The app needs ~0.5 s for the ack plus a reboot, then the bootloader has to
# enumerate as mass storage.
MOUNT_TIMEOUT_S = 30.0
POLL_S = 0.2


def log(msg: str) -> None:
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


def wait_for_volume(timeout: float) -> float | None:
    """Block until the bootloader volume mounts. Returns seconds waited."""
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        if os.path.isdir(VOLUME):
            return time.monotonic() - start
        time.sleep(POLL_S)
    return None


def copy_uf2(path: str) -> float:
    """Copy the image in. Returns seconds taken.

    The bootloader reboots the moment it has the whole file, which tears the USB
    volume out from under the copy -- so an I/O error here is the normal success
    path, not a failure.
    """
    start = time.monotonic()
    dest = os.path.join(VOLUME, os.path.basename(path))
    try:
        shutil.copyfile(path, dest)
    except OSError as exc:
        log(f"copy ended with {exc.__class__.__name__}: {exc} "
            f"(expected — the bootloader resets as soon as it has the image)")
    return time.monotonic() - start


async def main_async(uf2: str | None, scan_timeout: float) -> int:
    if uf2 and not os.path.isfile(uf2):
        log(f"ERROR: no such file: {uf2}")
        return 2

    if os.path.isdir(VOLUME):
        log(f"{VOLUME} is already mounted — the board is in the bootloader.")
        if uf2:
            took = copy_uf2(uf2)
            log(f"wrote {os.path.basename(uf2)} in {took:.1f}s")
        return 0

    log(f"Scanning for {DEVICE_NAME!r} ({scan_timeout:.0f}s)...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=scan_timeout)
    if device is None:
        log(f"NOT FOUND: {DEVICE_NAME} is not advertising.")
        log("If it is asleep in SYSTEMOFF, tap the board once to wake it.")
        return 2

    log(f"Found {device.address} — connecting...")
    acked: asyncio.Future = asyncio.get_running_loop().create_future()

    def on_notify(_, data: bytearray) -> None:
        b = bytes(data)
        if len(b) >= 5 and b[0] == 0x00 and b[1] == 0x55 and b[2] == EVT_REBOOT_ACK:
            if not acked.done():
                acked.set_result(bool(b[4]))

    async with BleakClient(device) as client:
        await client.start_notify(AUDIO_TX_UUID, on_notify)
        log("Sending reboot-to-bootloader command (RX 60 B0 07)...")
        await client.write_gatt_char(AUDIO_RX_UUID, CMD_REBOOT_BOOTLOADER,
                                     response=True)
        try:
            accepted = await asyncio.wait_for(acked, timeout=3.0)
        except asyncio.TimeoutError:
            log("No 0xD4 ack seen; the firmware may predate 0.0.106. "
                "Continuing to watch for the volume anyway.")
        else:
            if not accepted:
                log("REFUSED by firmware — a recording is in progress. "
                    "Stop it and retry.")
                return 3
            log("Accepted; the board is rebooting into the bootloader.")

    log(f"Waiting for {VOLUME} (up to {MOUNT_TIMEOUT_S:.0f}s)...")
    waited = wait_for_volume(MOUNT_TIMEOUT_S)
    if waited is None:
        log(f"TIMEOUT: {VOLUME} never appeared.")
        log("Check the USB cable (it must carry data, not just power) and that "
            "the bootloader accepted the GPREGRET magic.")
        return 4
    log(f"Bootloader mounted after {waited:.1f}s.")

    if not uf2:
        log("No .uf2 given — leaving the board in the bootloader.")
        log("NOTE: if the bootloader does not feed the watchdog it may reset "
            "itself ~12s from now. Watch whether the volume disappears.")
        return 0

    took = copy_uf2(uf2)
    log(f"Wrote {os.path.basename(uf2)} in {took:.1f}s "
        f"({waited + took:.1f}s total since the reboot).")
    return 0


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Reboot HarnessNode into the UF2 bootloader over BLE.")
    ap.add_argument("uf2", nargs="?",
                    help="optional .uf2 to copy as soon as the volume mounts")
    ap.add_argument("--scan-timeout", type=float, default=20.0,
                    help="BLE scan timeout in seconds (default: 20)")
    args = ap.parse_args()
    sys.exit(asyncio.run(main_async(args.uf2, args.scan_timeout)))


if __name__ == "__main__":
    main()
