#!/usr/bin/env python3
"""
Battery diagnostic for HarnessNode (nordic-main / StickC Plus SE).
Connects via BLE and reads Battery Service (0x180F) + dumps all services.
"""

import argparse
import asyncio
import sys
from bleak import BleakClient, BleakScanner

BATTERY_SERVICE_UUID = "0000180f-0000-1000-8000-00805f9b34fb"
BATTERY_LEVEL_UUID = "00002a19-0000-1000-8000-00805f9b34fb"


async def find_device(name: str, timeout: float):
    print(f"Scanning for '{name}'...")
    device = await BleakScanner.find_device_by_name(name, timeout=timeout)
    if device is not None:
        return device
    if name != "HarnessNode":
        return None
    print("Exact name missed; scanning HarnessNode* ...")
    devices = await BleakScanner.discover(timeout=timeout)
    for d in devices:
        if d.name and d.name.startswith("HarnessNode"):
            return d
    return None


async def main():
    parser = argparse.ArgumentParser(description="Read BLE Battery Service")
    parser.add_argument("--device", default="HarnessNode",
                        help="BLE advertised name (default: HarnessNode)")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    device = await find_device(args.device, args.timeout)
    if device is None:
        print(f"Device '{args.device}' not found.")
        sys.exit(1)

    print(f"Found: {device.name} {device.address}")

    async with BleakClient(device) as client:
        print(f"Connected. MTU={client.mtu_size}")
        print()

        print("=== Services & Characteristics ===")
        for svc in client.services:
            print(f"  Service: {svc.uuid}  ({svc.description})")
            for char in svc.characteristics:
                props = ",".join(char.properties)
                val_str = ""
                if "read" in char.properties:
                    try:
                        val = await client.read_gatt_char(char.uuid)
                        val_str = f"  → raw={val.hex()}  dec={list(val)}"
                    except Exception as e:
                        val_str = f"  → read error: {e}"
                print(f"    Char: {char.uuid}  [{props}]{val_str}")
        print()

        print("=== Battery Level ===")
        try:
            val = await client.read_gatt_char(BATTERY_LEVEL_UUID)
            pct = val[0]
            print(f"  Battery Level: {pct}%  (raw byte: 0x{val.hex()})")
            if pct == 0:
                print()
                print("  [!] 0% reported. Possible causes:")
                print("      1. USB給電のみ（バッテリー未接続）")
                print("      2. ADC / AXP 初期化失敗")
                print("      3. 電圧が LUT 下限（3000 mV）未満")
        except Exception as e:
            print(f"  Read failed: {e}")
            print("  Battery Service が見つからない")


asyncio.run(main())
