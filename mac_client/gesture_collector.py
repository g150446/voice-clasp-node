#!/usr/bin/env python3
"""
Gesture Data Collector for HarnessNode (nordic-main)

motion active から motion settle までの x/y/z 加速度時系列データを CSV に保存する。
録音開始ジェスチャーと、その他のジェスチャーを 5〜10 回ずつ収集する。

Usage:
    python3 gesture_collector.py
    python3 gesture_collector.py --output gesture_data.csv
"""

from device_debug import DebugControl
import asyncio
import struct
import time
import csv
import argparse
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME   = "HarnessNode"
AUDIO_TX_UUID = "00000002-0000-1000-8000-00805f9b34fb"

# Event packet header
SYNC0, SYNC1 = 0x00, 0x55
EVT_MOTION_ACTIVE  = 0x10
EVT_MOTION_SETTLED = 0x11
EVT_IMU_SAMPLE     = 0x13


class Collector:
    def __init__(self):
        self.current_label: str = ""
        self.collecting: bool = False

        self._gesture_id: int = -1
        self._in_gesture: bool = False
        self._samples: list = []   # [(gesture_id, label, elapsed_ms, x, y, z)]

    def on_notify(self, sender, data: bytes):
        if len(data) < 3:
            return

        # Event packet: [0x00][0x55][event_code][...]
        if data[0] == SYNC0 and data[1] == SYNC1:
            event = data[2]

            if event == EVT_MOTION_ACTIVE:
                if self.collecting and not self._in_gesture:
                    self._gesture_id += 1
                    self._in_gesture = True
                    t = time.strftime("%H:%M:%S")
                    print(f"  {t}  [ACTIVE ]  gesture #{self._gesture_id}  label={self.current_label}")

            elif event == EVT_MOTION_SETTLED:
                if self._in_gesture:
                    n = sum(1 for s in self._samples if s[0] == self._gesture_id)
                    t = time.strftime("%H:%M:%S")
                    print(f"  {t}  [SETTLED]  gesture #{self._gesture_id}  samples={n}")
                    self._in_gesture = False

            elif event == EVT_IMU_SAMPLE and len(data) >= 19:
                if self._in_gesture and self.collecting:
                    elapsed_ms = struct.unpack_from("<I", data, 3)[0]
                    x = struct.unpack_from("<f", data, 7)[0]
                    y = struct.unpack_from("<f", data, 11)[0]
                    z = struct.unpack_from("<f", data, 15)[0]
                    self._samples.append((self._gesture_id, self.current_label, elapsed_ms, x, y, z))

        # Audio packet [seq][0xAA][...]: ignore


async def run(output_path: str):
    print("=" * 60)
    print("Gesture Data Collector  (HarnessNode / nordic-main)")
    print("=" * 60)

    print("Scanning...")
    devices = await BleakScanner.discover(timeout=5.0)
    addr = next((d.address for d in devices if d.name == DEVICE_NAME), None)
    if not addr:
        print("ERROR: デバイスが見つかりません。電源とアドバタイズを確認してください。")
        return
    print(f"Found: {addr}")
    print()

    col = Collector()

    async with BleakClient(addr) as client:
        debug = DebugControl()
        await client.start_notify(AUDIO_TX_UUID, debug.wrap(col.on_notify))
        async with debug.session(client):
            loop = asyncio.get_event_loop()

            # ── Phase 1: 録音開始ジェスチャー ──────────────────────────
            print("=" * 60)
            print("  Phase 1: 録音開始ジェスチャー")
            print("  （腕を上げ → 握る → 手首を傾けるジェスチャー）")
            print("=" * 60)
            print("  5〜10 回繰り返してください。")
            print()
            print("  >>> 準備ができたら Enter を押してください（収集開始）")
            await loop.run_in_executor(None, input, "")

            col.current_label = "recording_start"
            col.collecting    = True
            print("  [収集中] ジェスチャーを行ってください...")
            print()
            print("  >>> 終わったら Enter を押してください（収集停止）")
            await loop.run_in_executor(None, input, "")
            col.collecting  = False
            col._in_gesture = False
            p1_count = col._gesture_id + 1
            print(f"  → {p1_count} ジェスチャー記録")
            print()

            await asyncio.sleep(1.0)

            # ── Phase 2: その他のジェスチャー ──────────────────────────
            print("=" * 60)
            print("  Phase 2: その他のジェスチャー")
            print("  （腕振り・歩行・ランダムな動作など）")
            print("=" * 60)
            print("  5〜10 回繰り返してください。")
            print()
            print("  >>> 準備ができたら Enter を押してください（収集開始）")
            await loop.run_in_executor(None, input, "")

            col.current_label = "other_motion"
            col.collecting    = True
            print("  [収集中] ジェスチャーを行ってください...")
            print()
            print("  >>> 終わったら Enter を押してください（収集停止）")
            await loop.run_in_executor(None, input, "")
            col.collecting  = False
            col._in_gesture = False
            p2_count = col._gesture_id + 1 - p1_count
            print(f"  → {p2_count} ジェスチャー記録")
            print()

            await client.stop_notify(AUDIO_TX_UUID)

    # ── 保存 ─────────────────────────────────────────────────────
    total_samples = len(col._samples)
    total_gestures = col._gesture_id + 1 if col._gesture_id >= 0 else 0

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["gesture_id", "label", "elapsed_ms", "x", "y", "z"])
        for row in col._samples:
            writer.writerow([row[0], row[1], row[2],
                             f"{row[3]:.6f}", f"{row[4]:.6f}", f"{row[5]:.6f}"])

    print("=" * 60)
    print(f"  完了: {total_gestures} ジェスチャー / {total_samples} サンプル")
    print(f"  保存: {output_path}")
    print()
    print(f"  グラフ表示:")
    print(f"    python3 gesture_visualizer.py {output_path}")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="gesture_data.csv",
                        help="出力CSVファイル名 (default: gesture_data.csv)")
    args = parser.parse_args()

    try:
        asyncio.run(run(args.output))
    except KeyboardInterrupt:
        print("\nInterrupted.")
