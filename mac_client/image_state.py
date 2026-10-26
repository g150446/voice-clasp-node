#!/usr/bin/env python3
"""Read-only MCUmgr image-state query.  Prints slot versions/flags, uploads nothing.

    venv/bin/python3 mac_client/image_state.py --device HarnessNode
"""

import argparse
import asyncio
import sys

from bleak import BleakClient

import ota_updater as ota


async def main_async(name: str, timeout: float) -> int:
    print(f"Scanning for {name!r} ({timeout:.0f}s)...", flush=True)
    device = await ota.find_device(name, timeout)
    if device is None:
        print(f"NOT FOUND: {name} is not advertising.", flush=True)
        return 2

    print(f"Found {device.name} [{device.address}] — connecting...", flush=True)
    async with BleakClient(device) as client:
        smp = ota.SMPClient(client)
        await smp.start()
        state = await ota.query_image_state(smp)
        images = state.get("images", [])
        if not images:
            print(f"No images reported. Raw: {state}", flush=True)
            return 3
        for img in images:
            print(
                "slot={slot} version={version} active={active} confirmed={confirmed} "
                "pending={pending} hash={hash}".format(
                    slot=img.get("slot"),
                    version=img.get("version", "?"),
                    active=img.get("active", False),
                    confirmed=img.get("confirmed", False),
                    pending=img.get("pending", False),
                    hash=bytes(img.get("hash", b"")).hex()[:16],
                ),
                flush=True,
            )
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Read MCUmgr image state (read-only)")
    parser.add_argument("--device", default="HarnessNode")
    parser.add_argument("--scan-timeout", type=float, default=15.0)
    args = parser.parse_args()
    sys.exit(asyncio.run(main_async(args.device, args.scan_timeout)))


if __name__ == "__main__":
    main()
