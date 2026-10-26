"""Decode, save, and plot HarnessNode gesture IMU trajectory batches."""

from __future__ import annotations

import csv
import math
import re
import struct
from pathlib import Path
from typing import Any, Optional


EVT_TRAJECTORY_BEGIN = 0x36
EVT_TRAJECTORY_CHUNK = 0x37
EVT_TRAJECTORY_END = 0x38
SAMPLE_SIZE = 27


class TrajectoryAssembler:
    def __init__(self) -> None:
        self.sessions: dict[int, dict[str, Any]] = {}

    def reset(self) -> None:
        self.sessions.clear()

    def feed(self, data: bytes) -> Optional[dict[str, Any]]:
        if len(data) < 3 or data[:2] != b"\x00\x55":
            return None
        code = data[2]
        if code == EVT_TRAJECTORY_BEGIN and len(data) >= 15:
            version, session, result, reason = data[3:7]
            expected_count, period_ms = struct.unpack_from("<HH", data, 7)
            gyro_y_bias_dps = struct.unpack_from("<f", data, 11)[0]
            self.sessions[session] = {
                "version": version,
                "session": session,
                "result": result,
                "reason": reason,
                "expected_count": expected_count,
                "period_ms": period_ms,
                "gyro_y_bias_dps": gyro_y_bias_dps,
                "samples_by_index": {},
            }
            return None
        if code == EVT_TRAJECTORY_CHUNK and len(data) >= 7:
            session = data[3]
            start_index = struct.unpack_from("<H", data, 4)[0]
            count = data[6]
            expected_len = 7 + count * SAMPLE_SIZE
            state = self.sessions.get(session)
            if state is None or len(data) != expected_len:
                return None
            offset = 7
            for item in range(count):
                elapsed_ms = struct.unpack_from("<H", data, offset)[0]
                flags = data[offset + 2]
                ax, ay, az, gx, gy, gz = struct.unpack_from(
                    "<ffffff", data, offset + 3
                )
                state["samples_by_index"][start_index + item] = {
                    "index": start_index + item,
                    "elapsed_ms": elapsed_ms,
                    "flags": flags,
                    "gyro_powered": bool(flags & 0x01),
                    "gyro_read_valid": bool(flags & 0x02),
                    "gyro_settled": bool(flags & 0x04),
                    "ax_ms2": ax,
                    "ay_ms2": ay,
                    "az_ms2": az,
                    "gx_dps": gx if flags & 0x02 else None,
                    "gy_dps": gy if flags & 0x02 else None,
                    "gz_dps": gz if flags & 0x02 else None,
                }
                offset += SAMPLE_SIZE
            return None
        if code == EVT_TRAJECTORY_END and len(data) >= 7:
            session = data[3]
            sent_count = struct.unpack_from("<H", data, 4)[0]
            status_flags = data[6]
            state = self.sessions.pop(session, None)
            if state is None:
                return None
            samples_by_index = state.pop("samples_by_index")
            state["samples"] = [samples_by_index[i] for i in sorted(samples_by_index)]
            state["sent_count"] = sent_count
            state["status_flags"] = status_flags
            state["overflow"] = bool(status_flags & 0x01)
            state["notify_error"] = bool(status_flags & 0x02)
            expected = state["expected_count"]
            state["missing_indices"] = [
                i for i in range(expected) if i not in samples_by_index
            ]
            state["complete"] = (
                len(samples_by_index) == expected
                and sent_count == expected
                and not state["missing_indices"]
                and not state["overflow"]
                and not state["notify_error"]
            )
            return state
        return None


def write_trajectory_csv(path: Path, trajectory: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "index", "elapsed_ms", "flags", "gyro_powered", "gyro_read_valid",
        "gyro_settled", "ax_ms2", "ay_ms2", "az_ms2", "gx_dps", "gy_dps",
        "gz_dps",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for sample in trajectory.get("samples", []):
            writer.writerow({field: sample.get(field) for field in fields})


def load_trajectory_csv(path: Path) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            sample: dict[str, Any] = {
                "index": int(row["index"]),
                "elapsed_ms": int(row["elapsed_ms"]),
                "flags": int(row["flags"]),
                "gyro_powered": row["gyro_powered"] == "True",
                "gyro_read_valid": row["gyro_read_valid"] == "True",
                "gyro_settled": row["gyro_settled"] == "True",
            }
            for key in ("ax_ms2", "ay_ms2", "az_ms2", "gx_dps", "gy_dps", "gz_dps"):
                sample[key] = float(row[key]) if row[key] else None
            samples.append(sample)
    return {"samples": samples, "complete": True}


def load_android_trajectory_csv(path: Path) -> dict[str, Any]:
    """Load the enriched CSV emitted by Android's GestureTrajectoryStore.

    Android prefixes the ordinary sample table with one ``# session`` record
    and optional ``# milestone`` / ``# live`` diagnostics.  Normalize the
    sample field names to the same shape as :func:`load_trajectory_csv` so
    offline evaluators can accept either source without special cases.
    """
    meta: dict[str, Any] = {}
    milestones: list[dict[str, Any]] = []
    live: list[dict[str, Any]] = []
    sample_lines: list[str] = []

    def parse_value(value: str) -> Any:
        if value.lower() in ("true", "false"):
            return value.lower() == "true"
        try:
            return int(value, 0)
        except ValueError:
            try:
                return float(value)
            except ValueError:
                return value

    def parse_diag(line: str, prefix: str) -> dict[str, Any]:
        payload, _, label = line[len(prefix):].strip().partition("  ")
        fields = next(csv.reader([payload]))
        if len(fields) != 6:
            raise ValueError(f"invalid {prefix.strip()} row in {path}: {line}")
        stage_name, _, reason_name = label.partition("/")
        return {
            "t_ms": int(fields[0]),
            "stage": int(fields[1], 0),
            "reason": int(fields[2], 0),
            "v1": float(fields[3]),
            "v2": float(fields[4]),
            "v3": float(fields[5]),
            "stage_name": stage_name,
            "reason_name": reason_name,
        }

    with path.open(encoding="utf-8") as handle:
        for raw in handle:
            line = raw.rstrip("\r\n")
            if line.startswith("# session="):
                for key, value in re.findall(r"(\w+)=([^\s]+)", line[2:]):
                    meta[key] = parse_value(value)
            elif line.startswith("# milestone "):
                milestones.append(parse_diag(line, "# milestone "))
            elif line.startswith("# live "):
                live.append(parse_diag(line, "# live "))
            elif line and not line.startswith("#"):
                sample_lines.append(line)

    if not sample_lines:
        raise ValueError(f"Android trajectory CSV has no sample table: {path}")
    samples: list[dict[str, Any]] = []
    for row in csv.DictReader(sample_lines):
        flags = int(row["flags"])
        samples.append({
            "index": len(samples),
            "elapsed_ms": int(row["t_ms"]),
            "flags": flags,
            "gyro_powered": bool(flags & 0x01),
            "gyro_read_valid": bool(flags & 0x02),
            "gyro_settled": bool(flags & 0x04),
            "ax_ms2": float(row["ax"]),
            "ay_ms2": float(row["ay"]),
            "az_ms2": float(row["az"]),
            "gx_dps": float(row["gx"]) if row["gx"] else None,
            "gy_dps": float(row["gy"]) if row["gy"] else None,
            "gz_dps": float(row["gz"]) if row["gz"] else None,
        })
    complete = (
        not bool(meta.get("overflow", False))
        and not bool(meta.get("notify_error", False))
        and int(meta.get("declared", len(samples))) == len(samples)
    )
    return {
        "samples": samples,
        "complete": complete,
        "meta": meta,
        "milestones": milestones,
        "live": live,
    }


def plot_trajectory(
    trajectory: dict[str, Any],
    png_path: Path,
    *,
    title: str,
    markers: Optional[list[tuple[int, str]]] = None,
    show: bool = True,
) -> None:
    import matplotlib.pyplot as plt

    samples = trajectory.get("samples", [])
    if not samples:
        raise ValueError("trajectory has no samples")
    times = [sample["elapsed_ms"] / 1000.0 for sample in samples]
    colors = {"X": "#d62728", "Y": "#2ca02c", "Z": "#1f77b4"}
    fig, (accel_ax, gyro_ax) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for axis, key in (("X", "ax_ms2"), ("Y", "ay_ms2"), ("Z", "az_ms2")):
        accel_ax.plot(times, [s[key] for s in samples], label=axis,
                      color=colors[axis], linewidth=1.2)
    for axis, key in (("X", "gx_dps"), ("Y", "gy_dps"), ("Z", "gz_dps")):
        values = [s[key] if s.get("gyro_read_valid") else math.nan for s in samples]
        gyro_ax.plot(times, values, label=axis, color=colors[axis], linewidth=1.2)

    invalid_times = [t for t, s in zip(times, samples) if not s.get("gyro_read_valid")]
    if invalid_times:
        gyro_ax.axvspan(min(invalid_times), max(invalid_times), color="0.8",
                        alpha=0.45, label="gyro unavailable")
    for marker_ms, label in markers or []:
        marker_s = marker_ms / 1000.0
        for axis in (accel_ax, gyro_ax):
            axis.axvline(marker_s, color="0.25", linestyle="--", linewidth=0.8)
        accel_ax.text(marker_s, 1.01, label, rotation=45, fontsize=8,
                      transform=accel_ax.get_xaxis_transform(), ha="left")

    accel_ax.set_ylabel("Acceleration (m/s²)")
    gyro_ax.set_ylabel("Angular velocity (dps)")
    gyro_ax.set_xlabel("Elapsed time (s)")
    for axis in (accel_ax, gyro_ax):
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper right", ncols=4)
    status = "complete" if trajectory.get("complete") else "incomplete"
    fig.suptitle(f"{title} — {status}")
    fig.tight_layout()
    png_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png_path, dpi=150)
    if show:
        plt.show(block=True)
    plt.close(fig)
