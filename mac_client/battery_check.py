#!/usr/bin/env python3
"""
Battery diagnostic for HarnessNode (nordic-main).
Connects via BLE and reads Battery Service (0x180F) + dumps all services.
"""

import asyncio
import sys
from bleak import BleakClient, BleakScanner
from node_names import find_node

DEVICE_NAME = "HarnessNode"
BATTERY_SERVICE_UUID  = "0000180f-0000-1000-8000-00805f9b34fb"
BATTERY_LEVEL_UUID    = "00002a19-0000-1000-8000-00805f9b34fb"


async def main():
    print(f"Scanning for '{DEVICE_NAME}'...")
    device = await find_node(DEVICE_NAME, timeout=10.0)
    if device is None:
        print(f"Device '{DEVICE_NAME}' not found.")
        sys.exit(1)

    print(f"Found: {device.address}")

    async with BleakClient(device) as client:
        print(f"Connected. MTU={client.mtu_size}")
        print()

        # --- dump all services & characteristics ---
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

        # --- Battery Level specific ---
        print("=== Battery Level ===")
        try:
            val = await client.read_gatt_char(BATTERY_LEVEL_UUID)
            pct = val[0]
            print(f"  Battery Level: {pct}%  (raw byte: 0x{val.hex()})")
            if pct == 0:
                print()
                print("  [!] 0% reported. Possible causes:")
                print("      1. USB給電のみ（バッテリー未接続）→ P0.14 が ~0V")
                print("      2. ADC初期化失敗 → battery_update() が早期リターン")
                print("      3. 分圧後の電圧が 3000mV 未満（LUT下限以下）")
                print()
                print("  → USB + LiPoバッテリーを接続して再試行してください。")
        except Exception as e:
            print(f"  Read failed: {e}")
            print("  Battery Service が見つからない → CONFIG_BT_BAS=y が効いていない可能性")


asyncio.run(main())
