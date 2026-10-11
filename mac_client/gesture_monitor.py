#!/usr/bin/env python3
"""
gesture_monitor.py — BLE接続して IMU イベントを表示する

イベントコード:
  0x10  motion_active    動き検出開始（+xyz値 f32）
  0x11  motion_settled   動き収束（+xyz値 f32）
  0x12  double_tap       リストバンド表側からのダブルタップ
  0x14  single_tap       同上のシングルタップ
  0x01  recording_start  録音開始
  0x02  recording_stop   録音停止
  0x20  sleep_enter      ライトスリープ開始（10秒無動作）
  0x21  sleep_wake       ライトスリープ復帰（動き検出）
  0xD0  imu_diag_report  IMUレジスタ診断（接続時送信）
  0xD1  imu_int_fired    GPIO割り込み発火（WAKE_UP_SRC）
"""

from device_debug import DebugControl
import asyncio
import struct
import sys
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME   = "HarnessNode"
AUDIO_TX_UUID = "00000002-0000-1000-8000-00805f9b34fb"


def on_notify(sender, data: bytes):
    if len(data) < 3 or data[0] != 0x00 or data[1] != 0x55:
        return

    code = data[2]
    ts   = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    if code == 0x10 and len(data) >= 15:
        x, y, z = struct.unpack_from('<fff', data, 3)
        print(f"[{ts}]  motion_active   x={x:+.2f} y={y:+.2f} z={z:+.2f}")
    elif code == 0x11 and len(data) >= 15:
        x, y, z = struct.unpack_from('<fff', data, 3)
        print(f"[{ts}]  motion_settled  x={x:+.2f} y={y:+.2f} z={z:+.2f}")
    elif code == 0x12:
        print(f"[{ts}]  double_tap ★")
    elif code == 0x14:
        print(f"[{ts}]  single_tap ★")
    elif code == 0x01:
        print(f"[{ts}]  recording_start ★")
    elif code == 0x02:
        print(f"[{ts}]  recording_stop")
    elif code == 0x20:
        print(f"[{ts}]  sleep_enter")
    elif code == 0x21:
        print(f"[{ts}]  sleep_wake")
    elif code == 0xD0 and len(data) >= 7:
        ctrl1, tap, md1, wusrc = data[3], data[4], data[5], data[6]
        ok = (ctrl1 == 0x20 and tap == 0x80 and md1 == 0x20)
        status = "OK" if ok else "NG"
        print(f"[{ts}]  [DIAG] IMU regs [{status}]"
              f"  CTRL1_XL=0x{ctrl1:02x}(exp:0x20)"
              f"  TAP_CFG=0x{tap:02x}(exp:0x80)"
              f"  MD1_CFG=0x{md1:02x}(exp:0x20)"
              f"  WAKE_UP_SRC=0x{wusrc:02x}")
    elif code == 0xD1 and len(data) >= 6:
        wusrc = data[3]
        count = data[4] | (data[5] << 8)
        sleep_bit = bool(wusrc & 0x10)
        event = "settle(sleep)" if sleep_bit else "wakeup(motion)"
        print(f"[{ts}]  [DIAG] GPIO INT #{count}  WAKE_UP_SRC=0x{wusrc:02x}  → {event}")
    else:
        print(f"[{ts}]  unknown 0x{code:02x}  raw={data.hex()}")


async def main():
    print(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
    if device is None:
        print(f"Device '{DEVICE_NAME}' not found.")
        sys.exit(1)

    print(f"Found: {device.address}. Connecting...")
    async with BleakClient(device) as client:
        print("Connected.\n")
        debug = DebugControl()
        await client.start_notify(AUDIO_TX_UUID, debug.wrap(on_notify))
        async with debug.session(client):
            try:
                while client.is_connected:
                    await asyncio.sleep(0.1)
            except KeyboardInterrupt:
                pass
    print("Disconnected.")


if __name__ == "__main__":
    asyncio.run(main())
