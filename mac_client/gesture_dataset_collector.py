#!/usr/bin/env python3
"""Collect one labelled six-axis gesture trial from HarnessNode."""

from __future__ import annotations

import argparse
from device_debug import DebugControl
import asyncio
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from gesture_validator import (
    AUDIO_RX_UUID,
    AUDIO_TX_UUID,
    DEFAULT_CUE_SOUND,
    DEVICE_NAME,
    EVT_DISCONNECTED,
    EVT_GESTURE_DIAG,
    EVT_RECORDING_START,
    EVT_RECORDING_STOP,
    diagnostic_record,
    parse_event_packet,
)
from imu_trajectory import (
    EVT_TRAJECTORY_BEGIN,
    EVT_TRAJECTORY_CHUNK,
    EVT_TRAJECTORY_END,
    TrajectoryAssembler,
    plot_trajectory,
    write_trajectory_csv,
)


CMD_STOP = 0x00
CMD_CLAIM_PRIMARY = 0x02
CMD_YIELD_PRIMARY = 0x03
CMD_COLLECT_IMU = 0x04
HOST_COLLECTION_RESULT = 3
CAPTURE_SECONDS = 6.0
PRE_GO_SECONDS = 0.15

INSTRUCTIONS = {
    "positive": "掌を上にして1秒静止し、挙上して掌を下にし、静止してください",
    "lift_only": "掌を上にして1秒静止し、掌を返さずに挙上してください",
    "flip_only": "掌を上にして1秒静止し、意図的に挙上せず掌だけを返してください",
    "daily": "スマートフォンや物を取る自然な腕動作を行い、元へ戻してください",
}

SAMPLE_PLAN = [
    (hand, motion, speed)
    for motion, speed in (
        ("positive", "natural"),
        ("positive", "slow"),
        ("positive", "fast"),
        ("lift_only", "na"),
        ("flip_only", "na"),
        ("daily", "na"),
    )
    for hand in ("right", "left")
]


class TrialCollector:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.client: Any = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.event_queue: asyncio.Queue[Any] = asyncio.Queue()
        self.trajectory_queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self.assembler = TrajectoryAssembler()
        self.diagnostics: list[dict[str, Any]] = []
        self.address = ""
        self.closing = False

    def on_notify(self, _sender: Any, data: bytearray) -> None:
        raw = bytes(data)
        if len(raw) >= 3 and raw[2] in (
            EVT_TRAJECTORY_BEGIN,
            EVT_TRAJECTORY_CHUNK,
            EVT_TRAJECTORY_END,
        ):
            completed = self.assembler.feed(raw)
            if completed is not None and self.loop is not None:
                self.loop.call_soon_threadsafe(
                    self.trajectory_queue.put_nowait, completed
                )
            return
        event = parse_event_packet(raw)
        if event is None or self.loop is None:
            return
        if event.code == EVT_GESTURE_DIAG:
            self.diagnostics.append(diagnostic_record(event))
        self.loop.call_soon_threadsafe(self.event_queue.put_nowait, event)

    def on_disconnected(self, _client: Any) -> None:
        if not self.closing and self.loop is not None:
            self.loop.call_soon_threadsafe(
                self.event_queue.put_nowait,
                type("DisconnectEvent", (), {"code": EVT_DISCONNECTED})(),
            )

    async def write_command(self, command: int) -> None:
        await self.client.write_gatt_char(
            AUDIO_RX_UUID, bytes([command]), response=True
        )

    async def connect(self) -> None:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as exc:
            raise RuntimeError("bleakをインストールしてください") from exc
        self.loop = asyncio.get_running_loop()
        if self.args.address:
            target: Any = self.args.address
            self.address = self.args.address
        else:
            print(f"BLE接続準備中: {self.args.device}", flush=True)
            target = await BleakScanner.find_device_by_name(
                self.args.device, timeout=self.args.scan_timeout
            )
            if target is None:
                raise RuntimeError(f"'{self.args.device}' が見つかりません")
            self.address = target.address
        self.client = BleakClient(
            target,
            timeout=self.args.connect_timeout,
            disconnected_callback=self.on_disconnected,
        )
        try:
            await self.client.connect()
            self.debug = DebugControl()
            await self.client.start_notify(AUDIO_TX_UUID, self.debug.wrap(self.on_notify))
            await self.debug.mode(self.client, 1)
            await self.write_command(CMD_CLAIM_PRIMARY)
            # Normalize a device left recording by another client or prior trial.
            await self.write_command(CMD_STOP)
            await asyncio.sleep(0.3)
            while not self.event_queue.empty():
                self.event_queue.get_nowait()
            print(f"BLE接続: OK ({self.address})", flush=True)

        except BaseException:
            await self.disconnect()
            raise

    async def disconnect(self) -> None:
        if self.client is None:
            return
        self.closing = True
        if self.client.is_connected:
            try:
                await self.write_command(CMD_YIELD_PRIMARY)
            except Exception:
                pass
            try:
                if self.client.is_connected and hasattr(self, 'debug'):
                    await self.debug.mode(self.client, 2)
            except Exception as exc:
                print(f'Debug cleanup failed; disconnect restores normal: {exc}', flush=True)
            finally:
                await self.client.disconnect()

    async def countdown(self) -> None:
        await self.debug.mode(self.client, 1)
        instruction = INSTRUCTIONS[self.args.motion]
        print(f"  手: {self.args.hand} / 動作: {self.args.motion}", flush=True)
        print(f"  速度: {self.args.speed}", flush=True)
        print(f"  指示: {instruction}", flush=True)
        print("  Ping音とGOのあとに動作してください。", flush=True)
        for remaining in range(self.args.countdown, 0, -1):
            print(f"  {remaining}...", flush=True)
            await asyncio.sleep(1.0)

    async def play_cue(self) -> None:
        if self.args.no_cue_sound:
            return
        sound = Path(self.args.cue_sound).expanduser()
        if not sound.is_file():
            raise RuntimeError(f"開始音がありません: {sound}")
        await asyncio.create_subprocess_exec(
            "/usr/bin/afplay",
            str(sound),
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )

    async def wait_for_collection(self, timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            trajectory = await asyncio.wait_for(
                self.trajectory_queue.get(), timeout=remaining
            )
            if trajectory.get("result") == HOST_COLLECTION_RESULT:
                return trajectory
        raise asyncio.TimeoutError

    async def run(self) -> int:
        await self.connect()
        matched = False
        stopped = False
        try:
            await self.countdown()
            while not self.event_queue.empty():
                self.event_queue.get_nowait()
            await self.write_command(CMD_COLLECT_IMU)
            await asyncio.sleep(PRE_GO_SECONDS)
            await self.play_cue()
            print("  >>> GO <<<", flush=True)

            deadline = time.monotonic() + CAPTURE_SECONDS + 0.4
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                try:
                    event = await asyncio.wait_for(
                        self.event_queue.get(), timeout=remaining
                    )
                except asyncio.TimeoutError:
                    break
                if event.code == EVT_DISCONNECTED:
                    raise ConnectionError("BLE接続が切断されました")
                if event.code == EVT_RECORDING_START:
                    matched = True
                elif event.code == EVT_RECORDING_STOP:
                    stopped = True

            if matched and not stopped:
                await self.write_command(CMD_STOP)
            try:
                trajectory = await self.wait_for_collection(timeout=6.0)
            except asyncio.TimeoutError as exc:
                raise RuntimeError(
                    "ホスト収集履歴を受信できません。"
                    "GESTURE_DEBUG_HISTORY=1の0.0.57以降が必要です"
                ) from exc
        finally:
            await self.disconnect()

        session_dir = Path(self.args.session_dir).expanduser()
        session_dir.mkdir(parents=True, exist_ok=True)
        existing = sorted(session_dir.glob("trial_*.json"))
        trial_number = len(existing) + 1
        stem = (
            f"trial_{trial_number:02d}_{self.args.hand}_"
            f"{self.args.motion}_{self.args.speed}"
        )
        csv_path = session_dir / f"{stem}.csv"
        png_path = session_dir / f"{stem}.png"
        json_path = session_dir / f"{stem}.json"
        write_trajectory_csv(csv_path, trajectory)
        plot_trajectory(
            trajectory,
            png_path,
            title=f"{self.args.hand} {self.args.motion} {self.args.speed}",
            markers=[(round(PRE_GO_SECONDS * 1000), "GO")],
            show=not self.args.no_plot,
        )
        report = {
            "trial": trial_number,
            "collected_at": datetime.now().isoformat(timespec="seconds"),
            "device": self.args.device,
            "address": self.address,
            "hand": self.args.hand,
            "motion": self.args.motion,
            "speed": self.args.speed,
            "instruction": INSTRUCTIONS[self.args.motion],
            "capture_seconds": CAPTURE_SECONDS,
            "pre_go_ms": round(PRE_GO_SECONDS * 1000),
            "classifier_matched": matched,
            "expected_match": self.args.motion == "positive",
            "usable": True,
            "trajectory_complete": trajectory.get("complete", False),
            "sample_count": len(trajectory.get("samples", [])),
            "csv": csv_path.name,
            "png": png_path.name,
            "diagnostics": self.diagnostics,
        }
        json_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(
            f"  保存完了: {len(trajectory.get('samples', []))} samples "
            f"({'complete' if trajectory.get('complete') else 'incomplete'})",
            flush=True,
        )
        print(f"  CSV: {csv_path}", flush=True)
        print(f"  PNG: {png_path}", flush=True)
        print(f"  JSON: {json_path}", flush=True)
        print(
            f"  現行判定: {'MATCH' if matched else 'NO MATCH'}",
            flush=True,
        )
        return 0


def default_session_dir() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return str(
        Path(__file__).resolve().parent / "output" / f"gesture_dataset_{stamp}"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="1試行の6軸データを収集")
    parser.add_argument("--hand", choices=("right", "left"))
    parser.add_argument(
        "--motion",
        choices=tuple(INSTRUCTIONS),
    )
    parser.add_argument(
        "--speed", choices=("natural", "slow", "fast", "na"), default="na"
    )
    parser.add_argument("--session-dir", default=default_session_dir())
    parser.add_argument("--device", default=DEVICE_NAME)
    parser.add_argument("--address")
    parser.add_argument("--countdown", type=int, default=3)
    parser.add_argument("--cue-sound", default=DEFAULT_CUE_SOUND)
    parser.add_argument("--no-cue-sound", action="store_true")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--scan-timeout", type=float, default=15.0)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument(
        "--list-plan", action="store_true", help="推奨12試行のコマンドを表示"
    )
    parser.add_argument("--self-test", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.self_test:
        assert len(SAMPLE_PLAN) == 12
        assert sum(motion == "positive" for _, motion, _ in SAMPLE_PLAN) == 6
        assert sum(motion != "positive" for _, motion, _ in SAMPLE_PLAN) == 6
        print("SELF_TEST: PASS")
        return 0
    if args.list_plan:
        session_dir = Path(args.session_dir).expanduser()
        for index, (hand, motion, speed) in enumerate(SAMPLE_PLAN, 1):
            print(
                f"{index:02d}. --hand {hand} --motion {motion} "
                f"--speed {speed} --session-dir {session_dir}"
            )
        return 0
    if args.hand is None or args.motion is None:
        raise SystemExit("--handと--motionを指定してください")
    if args.motion == "positive" and args.speed == "na":
        raise SystemExit("positiveでは--speed natural/slow/fastを指定してください")
    if args.motion != "positive":
        args.speed = "na"
    try:
        return asyncio.run(TrialCollector(args).run())
    except KeyboardInterrupt:
        print("\n中断しました", flush=True)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
