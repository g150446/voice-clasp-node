#!/usr/bin/env python3
"""Read-only real-time monitor for ALIENTEK UT70 measurements."""

from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import pathlib
import re
import statistics
import threading
import time

import serial

from ut70_common import UT70Error, decode_report, open_device, reports

STATE_RE = re.compile(r">>> STATE (S[012]):")


class SerialState:
    def __init__(self, port: str):
        self.port = port
        self.state = "unknown"
        self.changed_at = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial: serial.Serial | None = None

    def start(self) -> None:
        self._serial = serial.Serial(self.port, 115200, timeout=0.2)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        assert self._serial is not None
        while not self._stop.is_set():
            line = self._serial.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            match = STATE_RE.search(line)
            if match:
                self.state = match.group(1)
                self.changed_at = time.monotonic()
                print(f"\n[firmware] {line}")

    def snapshot(self) -> tuple[str, float]:
        return self.state, time.monotonic() - self.changed_at if self.changed_at else 0.0

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._serial:
            self._serial.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0, help="stop after N seconds")
    parser.add_argument("--csv", type=pathlib.Path)
    parser.add_argument("--moving-window", type=float, default=1.0, metavar="SECONDS")
    parser.add_argument("--display-hz", type=float, default=10.0)
    parser.add_argument("--serial", metavar="PORT", help="label samples using pdm_power_test UART")
    parser.add_argument(
        "--cycle-seconds", type=float, default=0,
        help="label S0/S1/S2 by elapsed time without opening UART (start just after board reset)",
    )
    parser.add_argument("--settle", type=float, default=2.0, help="exclude this many seconds after state changes")
    parser.add_argument("--reconnect", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    output = None
    writer = None
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        output = args.csv.open("w", newline="", encoding="utf-8")
        writer = csv.writer(output)
        writer.writerow([
            "timestamp", "state", "seconds_in_state", "voltage_V", "current_A",
            "current_mA", "power_W", "checksum_ok", "raw_hex",
        ])

    serial_state = SerialState(args.serial) if args.serial else None
    if serial_state:
        serial_state.start()

    all_values: list[float] = []
    by_state: dict[str, list[float]] = collections.defaultdict(list)
    window: collections.deque[tuple[float, float]] = collections.deque()
    rate_window: collections.deque[float] = collections.deque()
    started = time.monotonic()
    next_display = 0.0
    dev = None

    print("ALIENTEK UT70 (read-only)")
    try:
        while not args.duration or time.monotonic() - started < args.duration:
            try:
                if dev is None:
                    dev = open_device()
                    print("UT70 connected")
                for timestamp, raw in reports(dev):
                    now = time.monotonic()
                    measurement = decode_report(raw)
                    if serial_state:
                        state, state_age = serial_state.snapshot()
                    elif args.cycle_seconds:
                        cycle_elapsed = now - started
                        state_index = int(cycle_elapsed / args.cycle_seconds) % 3
                        state = f"S{state_index}"
                        state_age = cycle_elapsed % args.cycle_seconds
                    else:
                        state, state_age = "", 0.0
                    value = measurement.current_ma
                    all_values.append(value)
                    if state in {"S0", "S1", "S2"} and state_age >= args.settle:
                        by_state[state].append(value)

                    window.append((now, value))
                    while window and window[0][0] < now - args.moving_window:
                        window.popleft()
                    rate_window.append(now)
                    while rate_window and rate_window[0] < now - 2.0:
                        rate_window.popleft()
                    sample_rate = ((len(rate_window) - 1) /
                                   (rate_window[-1] - rate_window[0])) if len(rate_window) > 1 else 0.0

                    stamp = dt.datetime.fromtimestamp(timestamp).astimezone().isoformat(timespec="milliseconds")
                    if writer:
                        writer.writerow([
                            stamp, state, f"{state_age:.3f}", f"{measurement.voltage_v:.7f}",
                            f"{measurement.current_a:.9f}", f"{value:.6f}",
                            f"{measurement.power_w:.9f}", int(measurement.checksum_ok), raw.hex(),
                        ])

                    if now >= next_display:
                        moving = statistics.fmean(v for _, v in window)
                        label = f"  State: {state} ({state_age:5.1f}s)" if state else ""
                        print(
                            f"V={measurement.voltage_v:7.4f} V  "
                            f"I={value:7.3f} mA  P={measurement.power_w * 1000:7.3f} mW  "
                            f"min={min(all_values):7.3f} max={max(all_values):7.3f} "
                            f"avg={statistics.fmean(all_values):7.3f} moving={moving:7.3f} mA  "
                            f"rate={sample_rate:6.1f} Hz  checksum={'OK' if measurement.checksum_ok else 'BAD'}"
                            f"{label}"
                        )
                        next_display = now + 1.0 / max(args.display_hz, 0.1)
                    if output and len(all_values) % 100 == 0:
                        output.flush()
                    if args.duration and now - started >= args.duration:
                        raise StopIteration
            except StopIteration:
                break
            except (OSError, IOError, UT70Error) as exc:
                if dev:
                    dev.close()
                    dev = None
                if not args.reconnect:
                    raise
                print(f"UT70 disconnected/error: {exc}; retrying...")
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        if dev:
            dev.close()
        if serial_state:
            serial_state.close()
        if output:
            output.close()

    if by_state:
        print("\nSettled current by firmware state:")
        for state in ("S0", "S1", "S2"):
            values = by_state.get(state, [])
            if values:
                print(
                    f"{state}: n={len(values)} min={min(values):.3f} mA "
                    f"max={max(values):.3f} mA mean={statistics.fmean(values):.3f} mA"
                )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
