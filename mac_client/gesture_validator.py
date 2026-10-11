#!/usr/bin/env python3
"""HarnessNode の掌上静止→挙上→掌下静止をBLEで対話検証する。

人が画面のカウントダウンに合わせて動作し、ファームウェアから届く
``recording_start`` イベント (0x01) の有無を試行ごとに判定する。
録音終了は開始時の掌下姿勢からの緩い掌上反転であり、手を下ろすだけでは
``recording_stop`` しない。ホスト ``0x00`` による停止は従来どおり。

画面には試行ごとの判定条件だけを読みやすく表示し、生の診断イベントはJSON
レポートだけに保存する。標準入力は使わない。

Examples:
    venv/bin/python gesture_validator.py --trials 3
    venv/bin/python gesture_validator.py --start-only --trials 1
    venv/bin/python gesture_validator.py --expect no-match --trials 3 \
        --instruction "水平姿勢のまま手を動かしてください"
    venv/bin/python gesture_validator.py --self-test
"""

from __future__ import annotations

import argparse
from device_debug import DebugControl
import asyncio
import json
import math
import struct
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from imu_trajectory import (
    EVT_TRAJECTORY_BEGIN,
    EVT_TRAJECTORY_CHUNK,
    EVT_TRAJECTORY_END,
    TrajectoryAssembler,
    plot_trajectory,
    write_trajectory_csv,
)


DEVICE_NAME = "HarnessNode"
DEFAULT_CUE_SOUND = "/System/Library/Sounds/Ping.aiff"
DEFAULT_STOP_CUE_SOUND = "/System/Library/Sounds/Glass.aiff"
AUDIO_TX_UUID = "00000002-0000-1000-8000-00805f9b34fb"
AUDIO_RX_UUID = "00000003-0000-1000-8000-00805f9b34fb"

SYNC0 = 0x00
SYNC1 = 0x55
EVT_RECORDING_START = 0x01
EVT_RECORDING_STOP = 0x02
EVT_MOTION_ACTIVE = 0x10
EVT_MOTION_SETTLED = 0x11
EVT_SLEEP_ENTER = 0x20
EVT_SLEEP_WAKE = 0x21
EVT_GESTURE_DIAG = 0x30
EVT_GESTURE_HISTORY_BEGIN = 0x33
EVT_GESTURE_HISTORY_ENTRY = 0x34
EVT_GESTURE_HISTORY_END = 0x35
EVT_DISCONNECTED = -1

EVENT_NAMES = {
    EVT_RECORDING_START: "recording_start",
    EVT_RECORDING_STOP: "recording_stop",
    EVT_MOTION_ACTIVE: "motion_active",
    EVT_MOTION_SETTLED: "motion_settled",
    EVT_SLEEP_ENTER: "sleep_enter",
    EVT_SLEEP_WAKE: "sleep_wake",
    EVT_GESTURE_DIAG: "gesture_diag",
    EVT_GESTURE_HISTORY_BEGIN: "gesture_history_begin",
    EVT_GESTURE_HISTORY_ENTRY: "gesture_history_entry",
    EVT_GESTURE_HISTORY_END: "gesture_history_end",
    EVT_TRAJECTORY_BEGIN: "gesture_trajectory_begin",
    EVT_TRAJECTORY_CHUNK: "gesture_trajectory_chunk",
    EVT_TRAJECTORY_END: "gesture_trajectory_end",
    EVT_DISCONNECTED: "disconnected",
}

DIAG_STAGE_NAMES = {
    0x01: "outbound_start",
    0x02: "outbound_ready",
    0x03: "turnaround_ready",
    0x04: "return_start",
    0x05: "return_ready",
    0x06: "distance_ready",
    0x07: "final_hold_start",
    0x08: "final_ready",
    0x09: "match",
    0x0A: "match_detail",
    0x0B: "stop_near_miss",
    0x0C: "stop_hand_lower",
    0x0D: "gyro_enabled",
    0x0E: "gyro_disabled",
    0x0F: "outbound_gyro",
    0x20: "gyro_y_sample",
    0x21: "final_sample",
    0x22: "hold_sample",
    0x23: "motion_complete",
    0x24: "palm_down_gate",
    0x25: "lift_near_miss",
    0x10: "wait_reject",
    0x80: "reset",
}

DIAG_REASON_NAMES = {
    0x00: "none",
    0x01: "quiet_not_ready",
    0x02: "start_not_palm_up",
    0x03: "outbound_rate_low",
    0x11: "outbound_timeout",
    0x12: "incomplete_outbound",
    0x1A: "final_hold_interrupted",
    0x1B: "final_hold_timeout",
    0x1C: "sequence_timeout",
    0x1D: "final_accel_missing",
    0x1E: "final_brake_missing",
    0x1F: "final_brake_ratio_low",
    0x20: "final_tilt_unstable",
    0x21: "final_pulse_duration_invalid",
    0x22: "shake_not_oscillatory",
    0x23: "lift_palm_still_up",
    0x24: "motion_too_slow",
    0x25: "palm_down_gravity_low",
    0x26: "palm_down_gyro_angle_low",
    0x27: "palm_down_xy_ratio_low",
    0x28: "palm_down_gate_failed",
    0x29: "match_lift_impulse_low",
    0x2A: "match_pronation_low",
    0x2B: "match_gate_failed",
    0x2C: "stop_impulse_low",
    0x2D: "stop_peak_low",
    0x2E: "stop_pulse_short",
    0x2F: "stop_pulse_long",
    0x30: "lift_pulse_weak",
    0x31: "lift_start_timeout",
}

START_BOARD_FLAT_Z_MIN_RATIO = 0.75
START_PALM_UP_Z_MIN_RATIO = START_BOARD_FLAT_Z_MIN_RATIO
START_QUIET_HOLD_MS = 500
START_QUIET_ACCEL_MAX_MS2 = 4.0
SHAKE_PTP_MIN_MS2 = 5.0
SHAKE_MEAN_RATIO_MAX = 0.4
# Recording-stop hand-lower (0.0.69): reverse lift-axis pulse + settle.
# 0.0.73: time-decay soft thresholds + slow-path peak waiver.
PRONATION_ANGLE_MIN_DEG = 20.0
PRONATION_TILT_MIN_DEG = 30.0
PRONATION_START_DEG = 20.0
PRONATION_Z_RATIO_START = 0.50
PRONATION_Z_RATIO_DONE = 0.50
STOP_POST_START_INHIBIT_MS = 3000
STOP_OPP_ACCEL_MIN_MS2 = 0.25
STOP_OPP_ACCEL_SOFT_MS2 = 0.18
STOP_OPP_ACCEL_SOFT2_MS2 = 0.15
STOP_OPP_CONSECUTIVE_SAMPLES = 2
STOP_OPP_IMPULSE_MIN_MS = 0.10
STOP_OPP_IMPULSE_SOFT_MS = 0.08
STOP_OPP_IMPULSE_SOFT2_MS = 0.07
STOP_OPP_IMPULSE_LIFT_RATIO = 0.20
STOP_OPP_IMPULSE_LIFT_CAP_MS = 0.35
STOP_OPP_PULSE_MIN_MS = 60
STOP_OPP_PULSE_SLOW_MS = 180
STOP_OPP_PULSE_MAX_MS = 2000
STOP_PULSE_GAP_MS = 50
STOP_SETTLE_MS = 80
STOP_SETTLE_SOFT_MS = 50
STOP_SETTLE_SOFT2_MS = 40
STOP_SETTLE_LINEAR_MS2 = 4.0
STOP_SETTLE_LINEAR_SOFT_MS2 = 5.0
STOP_SOFTEN_AFTER_MS = 5000
STOP_SOFTEN2_AFTER_MS = 10000
# Legacy aliases kept for older notes / self-test migration.
STOP_HOLD_MS = STOP_SETTLE_MS
HOLD_PRONATION_ANGLE_MIN_DEG = 15.0
HOLD_PRONATION_Z_RATIO_DONE = 0.40
HOLD_GYRO_ANGLE_MIN_DEG = 30.0
HOLD_GYRO_INTEGRATE_RATE_DPS = 10.0
# Firmware waives xy only when lift is early (pre-flip) AND strong.
HOLD_GYRO_XY_PEAK_RATIO_MIN = 0.42
LIFT_PREFLIP_MAX_DEG = 50.0
LIFT_XY_WAIVER_IMPULSE_MIN_MS = 0.30
# Final: upward acceleration pulse, braking pulse, stable pose, then hold.
FINAL_POS_IMPULSE_MIN_MS = 0.30
MATCH_POS_IMPULSE_MIN_MS = 0.65
MATCH_PRONATION_MIN_DEG = 140.0
FINAL_NEG_IMPULSE_MIN_MS = 0.015
FINAL_BRAKE_RATIO_MIN = 0.05
FINAL_TILT_MAX_DEG = 15.0
FINAL_HOLD_MIN_MS = 500
LIFT_START_TIMEOUT_MS = 8000
MOTION_COMPLETE_MAX_MS = 4500
FINAL_STILL_RMS_MS2 = 3.0
FINAL_HOLD_RMS_EXIT_MS2 = 4.0
FINAL_HOLD_RMS_EXIT_SAMPLES = 3
FINAL_QUIET_RATE_DPS = 90.0
OUTBOUND_GYRO_INTEGRATE_RATE_DPS = 20.0
OUTBOUND_GYRO_ANGLE_MIN_DEG = 45.0
OUTBOUND_GYRO_ANGLE_PEAK_MIN_DPS = 30.0
OUTBOUND_GYRO_PEAK_DPS = 50.0
STOP_GYRO_INTEGRATE_RATE_DPS = 20.0
STOP_GYRO_ANGLE_MIN_DEG = 45.0
STOP_GYRO_ANGLE_PEAK_MIN_DPS = 30.0
STOP_GYRO_PEAK_DPS = 50.0
STOP_GYRO_PEAK_ONLY = False


@dataclass(frozen=True)
class GestureEvent:
    code: int
    name: str
    monotonic_s: float
    wall_time: str
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    elapsed_ms: Optional[int] = None
    diag_stage: Optional[int] = None
    diag_reason: Optional[int] = None
    value1: Optional[float] = None
    value2: Optional[float] = None
    value3: Optional[float] = None


@dataclass(frozen=True)
class ConditionResult:
    label: str
    status: str
    detail: str


@dataclass
class TrialResult:
    trial: int
    expected: str
    result: str
    matched: bool
    latency_ms: Optional[int]
    motion_active_seen: bool
    motion_settled_seen: bool
    sequence_reset_seen: bool
    reason: str
    diagnostics: list[dict[str, Any]]
    conditions: list[ConditionResult]
    trajectory: Optional[dict[str, Any]] = None
    artifacts: Optional[dict[str, str]] = None
    stop_cue_latency_ms: Optional[int] = None
    stop_latency_ms: Optional[int] = None
    stop_before_cue: bool = False


def gesture_gate_eligible(
    board_flat_z_ratio: float,
    palm_up_deg: float,
    positive_impulse_ms: float,
    negative_impulse_ms: float,
    final_tilt_deg: float,
    final_hold_ms: int,
    z_ratio_delta: float = 0.0,
    z_sign_flip: bool = False,
    shake_ptp_ms2: float = SHAKE_PTP_MIN_MS2,
    shake_mean_ms2: float = 0.0,
    palm_up_tilt_deg: float = 0.0,
) -> bool:
    """Mirror the firmware's shake, outbound palm-up, lift-pulse and hold gates.

    Outbound palm-up uses PRONATION_ANGLE_MIN_DEG / PRONATION_TILT_MIN_DEG /
    PRONATION_Z_RATIO_DONE. Hold flip in firmware remains
    HOLD_PRONATION_ANGLE_MIN_DEG / HOLD_PRONATION_Z_RATIO_DONE.
    """
    del negative_impulse_ms
    shake_ok = (
        shake_ptp_ms2 >= SHAKE_PTP_MIN_MS2
        and abs(shake_mean_ms2) < SHAKE_MEAN_RATIO_MAX * max(shake_ptp_ms2, 1e-6)
    )
    palm_up_ok = (
        abs(palm_up_deg) >= PRONATION_ANGLE_MIN_DEG
        or palm_up_tilt_deg >= PRONATION_TILT_MIN_DEG
        or z_ratio_delta >= PRONATION_Z_RATIO_DONE
        or z_sign_flip
    )
    return (
        board_flat_z_ratio >= START_BOARD_FLAT_Z_MIN_RATIO
        and shake_ok
        and palm_up_ok
        and positive_impulse_ms >= FINAL_POS_IMPULSE_MIN_MS
        and final_tilt_deg <= FINAL_TILT_MAX_DEG
        and final_hold_ms >= FINAL_HOLD_MIN_MS
    )


def recording_stop_impulse_need(lift_impulse_ms: float = 0.0) -> float:
    """Base impulse threshold before time-decay soften (0.0.71/0.0.73)."""
    return min(
        STOP_OPP_IMPULSE_LIFT_CAP_MS,
        max(
            STOP_OPP_IMPULSE_MIN_MS,
            lift_impulse_ms * STOP_OPP_IMPULSE_LIFT_RATIO,
        ),
    )


def recording_stop_active_thresholds(
    elapsed_ms: float = 0.0,
    lift_impulse_ms: float = 0.0,
) -> tuple[float, float, float, float]:
    """Return (need_imp, peak_min, settle_ms, settle_linear_max) for elapsed."""
    need = recording_stop_impulse_need(lift_impulse_ms)
    peak = STOP_OPP_ACCEL_MIN_MS2
    settle = float(STOP_SETTLE_MS)
    linear_max = STOP_SETTLE_LINEAR_MS2
    if elapsed_ms >= STOP_SOFTEN_AFTER_MS:
        need = min(need, STOP_OPP_IMPULSE_SOFT_MS)
        peak = STOP_OPP_ACCEL_SOFT_MS2
        settle = float(STOP_SETTLE_SOFT_MS)
        linear_max = STOP_SETTLE_LINEAR_SOFT_MS2
    if elapsed_ms >= STOP_SOFTEN2_AFTER_MS:
        need = min(need, STOP_OPP_IMPULSE_SOFT2_MS)
        peak = STOP_OPP_ACCEL_SOFT2_MS2
        settle = float(STOP_SETTLE_SOFT2_MS)
    return need, peak, settle, linear_max


def recording_stop_hand_lower_eligible(
    opp_impulse_ms: float,
    opp_peak_ms2: float = 0.0,
    pulse_ms: float = 0.0,
    lift_impulse_ms: float = 0.0,
    elapsed_ms: float = 0.0,
) -> bool:
    """Mirror firmware recording-stop hand-lower pulse (0.0.73)."""
    need_imp, peak_min, _settle, _linear = recording_stop_active_thresholds(
        elapsed_ms, lift_impulse_ms
    )
    if not (
        opp_impulse_ms >= need_imp
        and pulse_ms >= STOP_OPP_PULSE_MIN_MS
        and pulse_ms <= STOP_OPP_PULSE_MAX_MS
    ):
        return False
    if opp_peak_ms2 >= peak_min:
        return True
    # Slow-path: longer soft pulse may pass with peak >= soft2 floor.
    return (
        pulse_ms >= STOP_OPP_PULSE_SLOW_MS
        and opp_peak_ms2 >= STOP_OPP_ACCEL_SOFT2_MS2
    )


def recording_stop_palm_up_eligible(
    palm_up_deg: float,
    palm_up_tilt_deg: float = 0.0,
    z_ratio_delta: float = 0.0,
    z_sign_flip: bool = False,
    gyro_roll_deg: float = 0.0,
    gyro_peak_dps: float = 0.0,
) -> bool:
    """Deprecated palm-up stop (removed in 0.0.69). Always False."""
    del palm_up_deg, palm_up_tilt_deg, z_ratio_delta, z_sign_flip
    del gyro_roll_deg, gyro_peak_dps
    return False


def sequence_reset_seen(diagnostics: list[dict[str, Any]]) -> bool:
    """Return whether the one-action trial reset before its eventual match."""
    return any(record.get("stage") == "reset" for record in diagnostics)


def motion_completed_in_time(elapsed_ms: float) -> bool:
    """Firmware boundary: elapsed_ms >= MOTION_COMPLETE_MAX_MS is rejected."""
    return 0.0 <= elapsed_ms < MOTION_COMPLETE_MAX_MS


def gesture_match_final_gate_eligible(
    positive_impulse_ms: float, pronation_deg: float
) -> bool:
    """Mirror the 0.0.72 final match gate after palm-down hold."""
    return (
        positive_impulse_ms >= MATCH_POS_IMPULSE_MIN_MS
        and pronation_deg >= MATCH_PRONATION_MIN_DEG
    )


def stop_occurred_after_cue(
    stop_cue_latency_ms: Optional[int], stop_latency_ms: Optional[int]
) -> bool:
    """A physical recording-stop is valid only after the STOP GO cue."""
    return (
        stop_cue_latency_ms is not None
        and stop_latency_ms is not None
        and stop_latency_ms >= stop_cue_latency_ms
    )


def hold_rms_interrupts(samples: list[float]) -> bool:
    """Mirror the firmware's hold-only RMS hysteresis for diagnostics/tests."""
    consecutive = 0
    for rms in samples:
        if rms > FINAL_HOLD_RMS_EXIT_MS2:
            consecutive += 1
            if consecutive >= FINAL_HOLD_RMS_EXIT_SAMPLES:
                return True
        else:
            consecutive = 0
    return False


def _latest_diagnostic(
    diagnostics: list[dict[str, Any]],
    *,
    stage: Optional[str] = None,
    reasons: tuple[str, ...] = (),
) -> Optional[dict[str, Any]]:
    for record in reversed(diagnostics):
        if stage is not None and record.get("stage") != stage:
            continue
        if reasons and record.get("reason") not in reasons:
            continue
        return record
    return None


def build_condition_results(
    diagnostics: list[dict[str, Any]], matched: bool
) -> list[ConditionResult]:
    """Turn raw diagnostics into the firmware gate checklist shown per trial."""
    outbound_start = _latest_diagnostic(diagnostics, stage="outbound_start")
    outbound_ready = _latest_diagnostic(diagnostics, stage="outbound_ready")
    final_start = _latest_diagnostic(diagnostics, stage="final_hold_start")
    final_ready = _latest_diagnostic(diagnostics, stage="final_ready")
    motion_complete = _latest_diagnostic(diagnostics, stage="motion_complete")
    match_diag = _latest_diagnostic(diagnostics, stage="match")
    wait_quiet = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("quiet_not_ready",),
    )
    wait_pose = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("start_not_palm_up",),
    )
    outbound_reset = _latest_diagnostic(
        diagnostics,
        stage="reset",
        reasons=(
            "outbound_timeout",
            "incomplete_outbound",
        ),
    )
    final_reset = _latest_diagnostic(
        diagnostics,
        stage="reset",
        reasons=(
            "final_accel_missing",
            "final_brake_missing",
            "final_brake_ratio_low",
            "final_pulse_duration_invalid",
            "final_tilt_unstable",
            "final_hold_timeout",
            "lift_palm_still_up",
            "motion_too_slow",
            "palm_down_gate_failed",
            "match_gate_failed",
        ),
    )
    pulse_retry = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=(
            "final_brake_missing",
            "final_brake_ratio_low",
            "final_pulse_duration_invalid",
        ),
    )

    conditions: list[ConditionResult] = []

    # wait_reject encoding (firmware accel-only):
    #   start_not_palm_up: v1=z_ratio, v2=min_ratio, v3=linear_accel (board not flat)
    #   quiet_not_ready: v1=z_ratio, v2=min_ratio, v3=linear_accel
    pose_fail_detail: Optional[str] = None
    best_z = -1.0
    quiet_fail_detail: Optional[str] = None

    for rec in diagnostics:
        if rec.get("stage") != "wait_reject":
            continue
        reason = rec.get("reason")
        v1 = float(rec.get("value1") or 0.0)
        v2 = float(rec.get("value2") or 0.0)
        v3 = float(rec.get("value3") or 0.0)
        if reason == "start_not_palm_up":
            best_z = max(best_z, v1)
            thr = v2 if v2 > 0.1 else START_BOARD_FLAT_Z_MIN_RATIO
            pose_fail_detail = (
                f"Z絶対比 {v1:.2f} < {thr:.2f}"
                f"（線形加速度 {v3:.2f} m/s²）"
            )
        elif reason == "quiet_not_ready":
            best_z = max(best_z, v1)
            quiet_fail_detail = (
                f"Z絶対比 {v1:.2f}（要 ≥ {v2:.2f}）、"
                f"線形加速度 {v3:.2f} m/s²"
            )

    if outbound_start is not None:
        z_ratio = abs(float(outbound_start["value1"] or 0.0))
        linear = float(outbound_start["value3"] or 0.0)
        conditions.append(
            ConditionResult(
                "掌上候補の水平姿勢",
                "PASS" if z_ratio >= START_BOARD_FLAT_Z_MIN_RATIO else "FAIL",
                f"実測 Z比={z_ratio:.2f} 線形加速度={linear:.2f}m/s² | "
                f"閾値 Z≥{START_BOARD_FLAT_Z_MIN_RATIO:.2f}",
            )
        )
    elif wait_quiet is not None or wait_pose is not None:
        if best_z < 0.0 and wait_pose is not None:
            best_z = float(wait_pose.get("value1") or 0.0)
        if best_z < 0.0 and wait_quiet is not None:
            best_z = float(wait_quiet.get("value1") or 0.0)
        if pose_fail_detail:
            detail = pose_fail_detail
        elif quiet_fail_detail:
            detail = quiet_fail_detail
        else:
            detail = "水平・静止条件が未成立"
        conditions.append(
            ConditionResult(
                "掌上候補の水平姿勢",
                "FAIL",
                detail,
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "掌上候補の水平姿勢",
                "NOT_REACHED",
                "判定データなし",
            )
        )

    match_gate_reset = (
        final_reset
        if final_reset is not None
        and final_reset.get("reason") == "match_gate_failed"
        else None
    )
    if (
        match_diag is not None
        and match_diag.get("value1") is not None
        and match_diag.get("value2") is not None
    ):
        match_impulse = float(match_diag.get("value2") or 0.0)
        match_pronation = float(match_diag.get("value1") or 0.0)
    elif match_gate_reset is not None:
        match_impulse = float(match_gate_reset.get("value1") or 0.0)
        match_pronation = float(match_gate_reset.get("value2") or 0.0)
    else:
        match_impulse = None
        match_pronation = None

    if match_impulse is not None:
        conditions.append(
            ConditionResult(
                "最終挙上強度",
                "PASS" if match_impulse >= MATCH_POS_IMPULSE_MIN_MS else "FAIL",
                f"実測 {match_impulse:.3f} m/s | 閾値 ≥{MATCH_POS_IMPULSE_MIN_MS:.2f} m/s",
            )
        )
        conditions.append(
            ConditionResult(
                "掌上から掌下への反転",
                "PASS" if match_pronation >= MATCH_PRONATION_MIN_DEG else "FAIL",
                f"実測 {match_pronation:.1f}° | 閾値 ≥{MATCH_PRONATION_MIN_DEG:.0f}°",
            )
        )
    else:
        conditions.append(
            ConditionResult("最終挙上強度", "NOT_REACHED", "最終発火ゲート未到達")
        )
        conditions.append(
            ConditionResult("掌上から掌下への反転", "NOT_REACHED", "最終発火ゲート未到達")
        )

    if outbound_ready is not None:
        dwell_ms = float(outbound_ready["value1"] or 0.0)
        z_ratio = abs(float(outbound_ready["value2"] or 0.0))
        linear = float(outbound_ready.get("value3") or 0.0)
        dwell_ok = dwell_ms >= START_QUIET_HOLD_MS
        conditions.append(
            ConditionResult(
                "掌上で0.5秒静止",
                "PASS" if dwell_ok else "FAIL",
                f"実測 hold={dwell_ms:.0f}ms Z比={z_ratio:.2f} "
                f"線形加速度={linear:.2f}m/s² | 閾値 hold≥{START_QUIET_HOLD_MS}ms",
            )
        )
    elif outbound_reset is not None:
        conditions.append(
            ConditionResult(
                "掌上で0.5秒静止",
                "FAIL",
                f"phi {float(outbound_reset['value1']):+.1f}°、"
                f"3D {float(outbound_reset.get('value2') or 0.0):.1f}°、"
                f"Δz {float(outbound_reset.get('value3') or 0.0):.2f} "
                f"({outbound_reset['reason']})",
            )
        )
    elif outbound_start is not None:
        conditions.append(
            ConditionResult("掌上で0.5秒静止", "FAIL", "0.5秒静止まで到達せず")
        )
    else:
        conditions.append(
            ConditionResult("掌上で0.5秒静止", "NOT_REACHED", "掌上候補を検出せず")
        )

    # final_hold_start: v1=positive impulse, v2=negative impulse, v3=tilt
    # final_ready:      v1=positive impulse, v2=hold_ms, v3=tilt
    pulse_src = final_ready or final_start
    if pulse_src is not None:
        pos_impulse = float(pulse_src["value1"])
        neg_impulse = (
            float(final_start["value2"])
            if final_start is not None
            else FINAL_NEG_IMPULSE_MIN_MS
        )
        pulse_ok = pos_impulse >= FINAL_POS_IMPULSE_MIN_MS
        brake_ratio = neg_impulse / pos_impulse if pos_impulse > 0.0 else 0.0
        conditions.append(
            ConditionResult(
                "挙上",
                "PASS" if pulse_ok else "FAIL",
                f"実測 +imp={pos_impulse:.3f} m/s -imp={neg_impulse:.3f} m/s"
                f" 比={brake_ratio:.2f} | 閾値 +imp≥{FINAL_POS_IMPULSE_MIN_MS:.2f}",
            )
        )
    elif final_reset is not None and final_reset["reason"] in (
        "final_tilt_unstable",
        "final_hold_timeout",
        "lift_palm_still_up",
        "palm_down_gate_failed",
        "match_gate_failed",
    ):
        pos_impulse = float(final_reset.get("value1") or 0.0)
        neg_impulse = float(final_reset.get("value2") or 0.0)
        if final_reset["reason"] == "palm_down_gate_failed":
            detail = "挙上パルス成立後、掌下連動条件が未成立"
        elif final_reset["reason"] == "match_gate_failed":
            detail = (
                f"挙上パルス {pos_impulse:.3f} m/s成立後、"
                "最終発火ゲートで棄却"
            )
        else:
            detail = (
                f"加速 {pos_impulse:.3f} m/s、減速 {neg_impulse:.3f} m/s（成立済み）"
            )
        conditions.append(
            ConditionResult(
                "挙上",
                "PASS",
                detail,
            )
        )
    elif final_reset is not None:
        reason = str(final_reset["reason"])
        pos_impulse = float(final_reset.get("value1") or 0.0)
        neg_impulse = float(final_reset.get("value2") or 0.0)
        if reason == "motion_too_slow":
            detail = (
                f"動作開始から最終静止まで {pos_impulse:.0f} ms "
                f"（要 < {MOTION_COMPLETE_MAX_MS} ms）"
            )
        elif reason == "final_brake_ratio_low":
            detail = (
                f"減速/加速比 {float(final_reset.get('value3') or 0.0):.2f} "
                f"< {FINAL_BRAKE_RATIO_MIN:.2f}"
            )
        elif reason == "final_pulse_duration_invalid":
            detail = (
                f"パルス時間 {pos_impulse:.0f} ms "
                "（最小 150 ms）"
            )
        else:
            detail = (
                f"{reason}: 加速 {pos_impulse:.3f} m/s、"
                f"減速 {neg_impulse:.3f} m/s"
            )
        conditions.append(
            ConditionResult(
                "挙上",
                "FAIL",
                detail,
            )
        )
    elif pulse_retry is not None:
        reason = str(pulse_retry["reason"])
        pos_impulse = float(pulse_retry.get("value1") or 0.0)
        neg_impulse = float(pulse_retry.get("value2") or 0.0)
        if reason == "final_brake_ratio_low":
            detail = (
                f"減速/加速比 {float(pulse_retry.get('value3') or 0.0):.2f} "
                f"< {FINAL_BRAKE_RATIO_MIN:.2f}（パルス再試行）"
            )
        elif reason == "final_pulse_duration_invalid":
            detail = (
                f"パルス時間 {pos_impulse:.0f} ms "
                "（最小 150 ms、パルス再試行）"
            )
        else:
            detail = (
                f"{reason}: 加速 {pos_impulse:.3f} m/s、"
                f"減速 {neg_impulse:.3f} m/s（パルス再試行）"
            )
        conditions.append(
            ConditionResult(
                "挙上",
                "FAIL",
                detail,
            )
        )
    elif outbound_ready is not None:
        conditions.append(
            ConditionResult(
                "挙上",
                "FAIL",
                "掌上後に必要な挙上が成立せず",
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "挙上",
                "NOT_REACHED",
                "挙上判定まで到達せず",
            )
        )

    if motion_complete is not None:
        elapsed_ms = float(motion_complete.get("value1") or 0.0)
        conditions.append(
            ConditionResult(
                "3秒以内に完了",
                "PASS" if motion_completed_in_time(elapsed_ms) else "FAIL",
                f"実測 {elapsed_ms:.0f} ms | 閾値 < {MOTION_COMPLETE_MAX_MS} ms",
            )
        )
    elif final_reset is not None and final_reset["reason"] == "motion_too_slow":
        elapsed_ms = float(final_reset.get("value1") or 0.0)
        peak_y = float(final_reset.get("value2") or 0.0)
        roll = float(final_reset.get("value3") or 0.0)
        conditions.append(
            ConditionResult(
                "3秒以内に完了",
                "FAIL",
                f"実測 {elapsed_ms:.0f} ms | 閾値 < {MOTION_COMPLETE_MAX_MS} ms "
                f"(peak|gyro Y|={peak_y:.1f}dps, |積分角|={roll:.1f}°)",
            )
        )
    elif final_reset is not None and final_reset["reason"] == "palm_down_gate_failed":
        conditions.append(
            ConditionResult(
                "3秒以内に完了",
                "NOT_REACHED",
                "掌下連動条件が未成立のため、停止ジェスチャーを含む経過時間は評価対象外",
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "3秒以内に完了",
                "PASS" if matched else "NOT_REACHED",
                (
                    "録音開始成立（旧ファームのため完了時間diagなし）"
                    if matched else "最終静止未完了"
                ),
            )
        )

    hold_samples = [r for r in diagnostics if r.get("stage") == "hold_sample"]
    hold_interrupt = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("final_hold_interrupted",),
    )
    best_hold_rms = None
    best_hold_tilt = None
    best_hold_gy = None
    for rec in hold_samples:
        rms = float(rec.get("value1") or 0.0)
        tilt_h = float(rec.get("value2") or 0.0)
        gy_h = float(rec.get("value3") or 0.0)
        if best_hold_rms is None or rms < best_hold_rms:
            best_hold_rms = rms
            best_hold_tilt = tilt_h
            best_hold_gy = gy_h

    tilt_src = final_ready or final_start
    if final_reset is not None and final_reset["reason"] == "final_tilt_unstable":
        tilt_deg = float(final_reset.get("value3") or 0.0)
        conditions.append(
            ConditionResult(
                "上昇後の姿勢安定",
                "FAIL",
                f"実測 tilt={tilt_deg:.1f}° | 閾値 ≤{FINAL_TILT_MAX_DEG:.1f}°",
            )
        )
    elif tilt_src is not None:
        tilt_deg = abs(float(tilt_src["value3"]))
        conditions.append(
            ConditionResult(
                "上昇後の姿勢安定",
                "PASS" if tilt_deg <= FINAL_TILT_MAX_DEG else "FAIL",
                f"実測 tilt={tilt_deg:.1f}° | 閾値 ≤{FINAL_TILT_MAX_DEG:.1f}°",
            )
        )
    elif final_reset is not None and final_reset["reason"] == "final_hold_timeout":
        tilt_deg = float(final_reset.get("value3") or 0.0)
        conditions.append(
            ConditionResult(
                "上昇後の姿勢安定",
                "PASS" if abs(tilt_deg) <= FINAL_TILT_MAX_DEG else "FAIL",
                f"実測 max_tilt={tilt_deg:.1f}° | 閾値 ≤{FINAL_TILT_MAX_DEG:.1f}° "
                f"(hold timeout)",
            )
        )
    else:
        conditions.append(
            ConditionResult("上昇後の姿勢安定", "NOT_REACHED", "パルス成立前")
        )

    if final_ready is not None:
        hold_ms = float(final_ready["value2"])
        rms_s = (
            f" 最良RMS={best_hold_rms:.2f}(≤{FINAL_STILL_RMS_MS2})"
            if best_hold_rms is not None
            else ""
        )
        gy_s = (
            f" |gy|={best_hold_gy:.1f}dps"
            if best_hold_gy is not None
            else ""
        )
        conditions.append(
            ConditionResult(
                "掌下で静止",
                "PASS",
                f"実測 hold={hold_ms:.0f} ms{rms_s}{gy_s} | "
                f"閾値 ≥{FINAL_HOLD_MIN_MS} ms / 進入RMS≤{FINAL_STILL_RMS_MS2} / "
                f"保持中RMS>{FINAL_HOLD_RMS_EXIT_MS2}が"
                f"{FINAL_HOLD_RMS_EXIT_SAMPLES}サンプル連続で中断 / "
                f"進入時|ωy|≤{FINAL_QUIET_RATE_DPS:.0f}",
            )
        )
    elif final_reset is not None and final_reset["reason"] == "lift_palm_still_up":
        conditions.append(
            ConditionResult(
                "掌下で静止",
                "FAIL",
                "掌上基準から掌下へ回せなかった",
            )
        )
    elif final_reset is not None and final_reset["reason"] == "palm_down_gate_failed":
        gate = _latest_diagnostic(diagnostics, stage="palm_down_gate")
        if gate is None:
            detail = "掌下の重力＋gyro連動条件が未成立"
        elif gate["reason"] == "palm_down_gravity_low":
            detail = (
                f"重力反転不足: phi={float(gate.get('value1') or 0):.1f}° "
                f"ΔZ比={float(gate.get('value2') or 0):.2f} "
                f"符号反転={int(bool(gate.get('value3')))}"
            )
        elif gate["reason"] == "palm_down_gyro_angle_low":
            detail = (
                f"gyro Y積分角不足: {float(gate.get('value1') or 0):.1f}° "
                f"< {float(gate.get('value2') or HOLD_GYRO_ANGLE_MIN_DEG):.1f}°"
            )
        else:
            detail = (
                f"挙上/回内連動比不足: peak|gx|/peak|gy|="
                f"{float(gate.get('value1') or 0):.2f} "
                f"< {float(gate.get('value2') or HOLD_GYRO_XY_PEAK_RATIO_MIN):.2f}"
            )
        conditions.append(ConditionResult("掌下で静止", "FAIL", detail))
    elif final_start is not None or hold_interrupt is not None:
        detail_parts = [f"hold {FINAL_HOLD_MIN_MS} ms 未完了"]
        if hold_interrupt is not None:
            detail_parts.append(
                f"中断時 RMS={float(hold_interrupt.get('value1') or 0):.2f} "
                f"tilt={float(hold_interrupt.get('value2') or 0):.1f}° "
                f"|gy|={float(hold_interrupt.get('value3') or 0):.1f}dps "
                f"(進入RMS≤{FINAL_STILL_RMS_MS2}、保持中RMS>"
                f"{FINAL_HOLD_RMS_EXIT_MS2}が{FINAL_HOLD_RMS_EXIT_SAMPLES}"
                f"サンプル連続で中断、tilt≤{FINAL_TILT_MAX_DEG} "
                f"進入|ωy|≤{FINAL_QUIET_RATE_DPS:.0f})"
            )
        if best_hold_rms is not None:
            detail_parts.append(
                f"hold中最良 RMS={best_hold_rms:.2f} tilt={best_hold_tilt:.1f} "
                f"|gy|={best_hold_gy:.1f}"
            )
        conditions.append(
            ConditionResult(
                "掌下で静止",
                "FAIL",
                " / ".join(detail_parts),
            )
        )
    elif final_reset is not None and final_reset["reason"] == "final_hold_timeout":
        conditions.append(
            ConditionResult(
                "掌下で静止",
                "FAIL",
                f"{FINAL_HOLD_MIN_MS} ms静止を開始できずタイムアウト "
                f"max_tilt={float(final_reset.get('value3') or 0):.1f}°",
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "掌下で静止", "NOT_REACHED", "上昇が未成立"
            )
        )

    conditions.append(
        ConditionResult(
            "1回の動作で完結",
            "FAIL" if sequence_reset_seen(diagnostics) else "PASS",
            (
                "途中で判定がリセットされた"
                if sequence_reset_seen(diagnostics)
                else "途中の判定リセットなし"
            ),
        )
    )
    conditions.append(
        ConditionResult(
            "ジェスチャー発動",
            "PASS" if matched else "FAIL",
            "recording_startを受信" if matched else "recording_startなし",
        )
    )
    return conditions


def print_condition_results(conditions: list[ConditionResult]) -> None:
    markers = {
        "PASS": "[OK]",
        "FAIL": "[NG]",
        "NOT_REACHED": "[--]",
        "SKIP": "[--]",
    }
    print("  判定条件:", flush=True)
    for condition in conditions:
        print(
            f"    {markers[condition.status]} {condition.label}: {condition.detail}",
            flush=True,
        )


def summarize_gyro_y_samples(diagnostics: list[dict[str, Any]]) -> None:
    """Print a short summary of debug gyro_y_sample stream if present."""
    samples = [r for r in diagnostics if r.get("stage") == "gyro_y_sample"]
    if not samples:
        return
    ys = [float(r.get("value1") or 0.0) for r in samples]
    print(
        f"  gyro_y debug: n={len(samples)} "
        f"min={min(ys):+.1f} max={max(ys):+.1f} "
        f"last={ys[-1]:+.1f} dps",
        flush=True,
    )


def write_gyro_csv(path: str, diagnostics: list[dict[str, Any]]) -> None:
    samples = [r for r in diagnostics if r.get("stage") == "gyro_y_sample"]
    if not samples:
        return
    lines = ["elapsed_ms,gyro_y_dps,y_ratio,time"]
    for r in samples:
        lines.append(
            f"{float(r.get('value3') or 0.0):.0f},"
            f"{float(r.get('value1') or 0.0):.3f},"
            f"{float(r.get('value2') or 0.0):.4f},"
            f"{r.get('time') or ''}"
        )
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"  gyro CSV: {path} ({len(samples)} samples)", flush=True)


def parse_event_packet(data: bytes, now: Optional[float] = None) -> Optional[GestureEvent]:
    """BLE notificationからイベントだけを抽出する。音声パケットは無視する。"""
    if len(data) < 3 or data[0] != SYNC0 or data[1] != SYNC1:
        return None

    code = data[2]
    event = GestureEvent(
        code=code,
        name=EVENT_NAMES.get(code, f"unknown_0x{code:02x}"),
        monotonic_s=time.monotonic() if now is None else now,
        wall_time=datetime.now().isoformat(timespec="milliseconds"),
    )

    if code in (EVT_MOTION_ACTIVE, EVT_MOTION_SETTLED) and len(data) >= 15:
        x, y, z = struct.unpack_from("<fff", data, 3)
        elapsed_ms = None
        if code == EVT_MOTION_SETTLED and len(data) >= 19:
            elapsed_ms = struct.unpack_from("<I", data, 15)[0]
        event = GestureEvent(
            code=code,
            name=event.name,
            monotonic_s=event.monotonic_s,
            wall_time=event.wall_time,
            x=x,
            y=y,
            z=z,
            elapsed_ms=elapsed_ms,
        )
    elif code == EVT_GESTURE_DIAG and len(data) >= 17:
        stage = data[3]
        reason = data[4]
        value1, value2, value3 = struct.unpack_from("<fff", data, 5)
        event = GestureEvent(
            code=code,
            name=event.name,
            monotonic_s=event.monotonic_s,
            wall_time=event.wall_time,
            diag_stage=stage,
            diag_reason=reason,
            value1=value1,
            value2=value2,
            value3=value3,
        )
    elif code == EVT_GESTURE_HISTORY_BEGIN and len(data) >= 5:
        event = GestureEvent(
            code=code,
            name=event.name,
            monotonic_s=event.monotonic_s,
            wall_time=event.wall_time,
            value1=float(data[3]),
            value2=float(data[4]),
        )
    elif code == EVT_GESTURE_HISTORY_ENTRY and len(data) >= 19:
        t_ms = struct.unpack_from("<H", data, 3)[0]
        stage = data[5]
        reason = data[6]
        value1, value2, value3 = struct.unpack_from("<fff", data, 7)
        event = GestureEvent(
            code=code,
            name=event.name,
            monotonic_s=event.monotonic_s,
            wall_time=event.wall_time,
            elapsed_ms=t_ms,
            diag_stage=stage,
            diag_reason=reason,
            value1=value1,
            value2=value2,
            value3=value3,
        )
    elif code == EVT_GESTURE_HISTORY_END and len(data) >= 5:
        event = GestureEvent(
            code=code,
            name=event.name,
            monotonic_s=event.monotonic_s,
            wall_time=event.wall_time,
            value1=float(data[3]),
            value2=float(data[4]),
        )
    return event


def format_event(event: GestureEvent) -> str:
    ts = event.wall_time[11:23]
    if event.code == EVT_GESTURE_HISTORY_BEGIN:
        return (
            f"[{ts}] history_begin count={int(event.value1 or 0)} "
            f"session={int(event.value2 or 0)}"
        )
    if event.code == EVT_GESTURE_HISTORY_END:
        return (
            f"[{ts}] history_end count={int(event.value1 or 0)} "
            f"session={int(event.value2 or 0)}"
        )
    if event.code == EVT_GESTURE_HISTORY_ENTRY:
        stage = DIAG_STAGE_NAMES.get(
            event.diag_stage or 0, f"stage_0x{(event.diag_stage or 0):02x}"
        )
        reason = DIAG_REASON_NAMES.get(
            event.diag_reason or 0, f"reason_0x{(event.diag_reason or 0):02x}"
        )
        return (
            f"[{ts}] history_entry +{event.elapsed_ms or 0}ms "
            f"{stage}/{reason} "
            f"v1={event.value1 or 0.0:+.2f} "
            f"v2={event.value2 or 0.0:+.2f} "
            f"v3={event.value3 or 0.0:+.2f}"
        )
    if event.code == EVT_GESTURE_DIAG:
        stage = DIAG_STAGE_NAMES.get(
            event.diag_stage or 0, f"stage_0x{(event.diag_stage or 0):02x}"
        )
        reason = DIAG_REASON_NAMES.get(
            event.diag_reason or 0, f"reason_0x{(event.diag_reason or 0):02x}"
        )
        v1 = event.value1 or 0.0
        v2 = event.value2 or 0.0
        v3 = event.value3 or 0.0
        if stage == "outbound_start":
            suffix = (
                f" palm_candidate z={v1:.2f} elapsed={v2:.0f}ms "
                f"linear={v3:.2f}m/s^2"
            )
        elif stage == "outbound_ready":
            suffix = (
                f" dwell={v1:.0f}ms z={v2:.2f} linear={v3:.2f}m/s^2"
            )
        elif stage in ("stop_hand_lower", "stop_palm_up"):
            suffix = (
                f" opp_imp={v1:.3f}m/s peak={v2:.2f}m/s^2 "
                f"pulse={v3:.0f}ms"
            )
        elif stage == "stop_near_miss":
            if reason in ("stop_pulse_short", "stop_pulse_long"):
                suffix = (
                    f" opp_imp={v1:.3f}m/s peak={v2:.2f}m/s^2 "
                    f"pulse={v3:.0f}ms reason={reason}"
                )
            else:
                suffix = (
                    f" opp_imp={v1:.3f}m/s peak={v2:.2f}m/s^2 "
                    f"need={v3:.3f}m/s reason={reason}"
                )
        elif stage == "lift_near_miss":
            suffix = (
                f" peak_a_up={v1:.2f}m/s^2 imp={v2:.3f}m/s "
                f"elapsed={v3:.0f}ms reason={reason}"
            )
        elif stage == "gyro_enabled":
            suffix = f" odr={v1:.0f}Hz bias_y={v2:+.2f} valid={v3:.0f}"
        elif stage == "gyro_disabled":
            suffix = (
                f" last_roll={v1:+.1f}deg peak={v2:.1f}dps "
                f"stop_roll={v3:+.1f}deg"
            )
        elif stage == "turnaround_ready":
            suffix = (
                f" elapsed={v1:.0f}ms gyro_y={v2:+.1f}dps y={v3:+.2f}"
            )
        elif stage == "return_start":
            suffix = (
                f" roll_rate={v1:.1f}dps gyro_y={v2:+.1f}dps "
                f"outbound_roll={v3:+.1f}deg"
            )
        elif stage == "return_ready":
            suffix = (
                f" supination={v1:+.1f}deg peak={v2:.1f}dps "
                f"final_y={v3:+.2f}"
            )
        elif stage == "final_sample":
            suffix = (
                f" pulse_stage={v1:.0f} a_up={v2:+.2f}m/s^2 "
                f"net_impulse={v3:+.3f}m/s"
            )
        elif stage == "final_hold_start":
            suffix = (
                f" positive_impulse={v1:.3f}m/s "
                f"negative_impulse={v2:.3f}m/s tilt={v3:.1f}deg"
            )
        elif stage == "match":
            suffix = (
                f" pronation={v1:+.1f}deg positive_impulse={v2:.3f}m/s "
                f"hold={v3:.0f}ms"
            )
        elif stage == "motion_complete":
            suffix = (
                f" elapsed={v1:.0f}ms peak_y={v2:.1f}dps "
                f"roll={v3:.1f}deg"
            )
        elif stage == "palm_down_gate":
            if reason == "palm_down_gravity_low":
                suffix = (
                    f" reason={reason} phi={v1:.1f}deg "
                    f"z_delta={v2:.2f} sign_flip={int(bool(v3))}"
                )
            elif reason == "palm_down_gyro_angle_low":
                suffix = (
                    f" reason={reason} signed_roll={v1:.1f}deg "
                    f"required={v2:.1f}deg peak_y={v3:.1f}dps"
                )
            else:
                suffix = (
                    f" reason={reason} xy_ratio={v1:.2f} "
                    f"required={v2:.2f} peak_x={v3:.1f}dps"
                )
        elif stage == "distance_ready":
            suffix = (
                f" fused={v1:.1f}cm accel={v2:.1f}cm arc={v3:.1f}cm"
            )
        elif stage == "final_ready":
            suffix = (
                f" positive_impulse={v1:.3f}m/s hold={v2:.0f}ms "
                f"tilt={v3:.1f}deg"
            )
        elif stage == "gyro_bias_ready":
            suffix = (
                f" correction={v1:.1f}dps plane_bias={v2:.1f}dps "
                f"z_bias={v3:.1f}dps"
            )
        elif stage == "gyro_y_sample":
            suffix = (
                f" gyro_y={v1:+.1f}dps y_ratio={v2:+.2f} "
                f"elapsed={v3:.0f}ms"
            )
        elif stage == "wait_reject":
            if reason == "match_lift_impulse_low":
                suffix = (
                    f" reason={reason} positive_impulse={v1:.3f}m/s "
                    f"required={v2:.2f}m/s pronation={v3:.1f}deg"
                )
            elif reason == "match_pronation_low":
                suffix = (
                    f" reason={reason} pronation={v1:.1f}deg "
                    f"required={v2:.0f}deg positive_impulse={v3:.3f}m/s"
                )
            else:
                suffix = (
                    f" reason={reason} value1={v1:.2f} "
                    f"value2={v2:.2f} value3={v3:.1f}"
                )
        elif reason == "turnaround_timeout":
            suffix = (
                f" reason={reason} elapsed={v1:.0f}ms "
                f"max_pos_gy={v2:+.1f}dps min_neg_gy={v3:+.1f}dps"
            )
        elif reason == "distance_too_short":
            suffix = (
                f" reason={reason} fused={v1:.1f}cm "
                f"accel={v2:.1f}cm arc={v3:.1f}cm"
            )
        elif reason == "motion_too_slow":
            suffix = (
                f" reason={reason} elapsed={v1:.0f}ms "
                f"peak_y={v2:.1f}dps roll={v3:.1f}deg"
            )
        elif reason == "palm_down_gate_failed":
            suffix = (
                f" reason={reason} elapsed={v1:.0f}ms "
                f"signed_roll={v2:.1f}deg xy_ratio={v3:.2f}"
            )
        elif reason == "match_gate_failed":
            suffix = (
                f" reason={reason} positive_impulse={v1:.3f}m/s "
                f"pronation={v2:.1f}deg failure_bits={int(v3)}"
            )
        else:
            suffix = f" reason={reason} values={v1:.2f},{v2:.2f},{v3:.2f}"
        return f"[{ts}] gesture_{stage}{suffix}"
    if event.x is not None:
        suffix = f" x={event.x:+.2f} y={event.y:+.2f} z={event.z:+.2f}"
        if event.elapsed_ms is not None:
            suffix += f" elapsed={event.elapsed_ms}ms"
    else:
        suffix = ""
    marker = " ★" if event.code == EVT_RECORDING_START else ""
    return f"[{ts}] {event.name}{suffix}{marker}"


def diagnostic_record(event: GestureEvent) -> dict[str, Any]:
    record: dict[str, Any] = {
        "time": event.wall_time,
        "t": event.monotonic_s,
        "stage": DIAG_STAGE_NAMES.get(event.diag_stage or 0, "unknown"),
        "reason": DIAG_REASON_NAMES.get(event.diag_reason or 0, "unknown"),
        "value1": event.value1,
        "value2": event.value2,
        "value3": event.value3,
    }
    stage = record["stage"]
    reason = record["reason"]
    if stage == "distance_ready" or reason == "distance_too_short":
        record["metrics"] = {
            "fused_distance_cm": event.value1,
            "accel_distance_cm": event.value2,
            "arc_distance_cm": event.value3,
        }
    elif stage == "outbound_start":
        record["metrics"] = {
            "z_ratio": event.value1,
            "elapsed_ms": event.value2,
            "linear_accel_ms2": event.value3,
        }
    elif stage == "outbound_ready":
        record["metrics"] = {
            "dwell_ms": event.value1,
            "z_ratio": event.value2,
            "linear_accel_ms2": event.value3,
        }
    elif stage in ("stop_hand_lower", "stop_palm_up"):
        record["metrics"] = {
            "opp_impulse_ms": event.value1,
            "opp_peak_ms2": event.value2,
            "pulse_ms": event.value3,
            "pronation_angle_deg": event.value1,
            "tilt_3d_deg": event.value2,
            "z_ratio_delta": event.value3,
        }
    elif stage == "stop_near_miss":
        record["metrics"] = {
            "opp_impulse_ms": event.value1,
            "opp_peak_ms2": event.value2,
            "need_or_pulse": event.value3,
        }
    elif stage == "return_ready":
        record["metrics"] = {
            "supination_angle_deg": event.value1,
            "supination_peak_dps": event.value2,
            "final_y_ratio": event.value3,
        }
    elif stage == "gyro_y_sample":
        record["metrics"] = {
            "gyro_y_dps": event.value1,
            "y_ratio": event.value2,
            "elapsed_ms": event.value3,
        }
    elif stage == "reset" and reason == "turnaround_timeout":
        record["metrics"] = {
            "elapsed_ms": event.value1,
            "max_pos_gyro_y_dps": event.value2,
            "min_neg_gyro_y_dps": event.value3,
        }
    elif stage == "reset" and reason == "final_pulse_duration_invalid":
        record["metrics"] = {
            "pulse_duration_ms": event.value1,
            "positive_impulse_ms": event.value2,
            "negative_impulse_ms": event.value3,
        }
    elif stage == "reset" and reason == "final_brake_ratio_low":
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "negative_impulse_ms": event.value2,
            "brake_ratio": event.value3,
        }
    elif stage == "reset" and reason == "motion_too_slow":
        record["metrics"] = {
            "motion_elapsed_ms": event.value1,
            "gyro_y_peak_dps": event.value2,
            "gyro_y_integral_deg": event.value3,
        }
    elif stage == "reset" and reason == "match_gate_failed":
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "pronation_angle_deg": event.value2,
            "failure_bits": event.value3,
        }
    elif stage == "wait_reject" and reason == "match_lift_impulse_low":
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "required_impulse_ms": event.value2,
            "pronation_angle_deg": event.value3,
        }
    elif stage == "wait_reject" and reason == "match_pronation_low":
        record["metrics"] = {
            "pronation_angle_deg": event.value1,
            "required_pronation_deg": event.value2,
            "positive_impulse_ms": event.value3,
        }
    elif stage == "reset" and reason in (
        "final_accel_missing",
        "final_brake_missing",
        "final_tilt_unstable",
        "final_hold_timeout",
        "lift_palm_still_up",
    ):
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "negative_impulse_ms": event.value2,
            "tilt_deg": event.value3,
        }
    elif stage == "final_sample":
        record["metrics"] = {
            "pulse_stage": event.value1,
            "a_up_ms2": event.value2,
            "net_impulse_ms": event.value3,
        }
    elif stage == "final_hold_start":
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "negative_impulse_ms": event.value2,
            "tilt_deg": event.value3,
        }
    elif stage == "final_ready":
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "final_hold_ms": event.value2,
            "tilt_deg": event.value3,
        }
    elif stage == "match":
        record["metrics"] = {
            "pronation_angle_deg": event.value1,
            "positive_impulse_ms": event.value2,
            "final_hold_ms": event.value3,
        }
    elif stage == "motion_complete":
        record["metrics"] = {
            "motion_elapsed_ms": event.value1,
            "gyro_y_peak_dps": event.value2,
            "gyro_y_integral_deg": event.value3,
        }
    elif stage == "palm_down_gate":
        if reason == "palm_down_gravity_low":
            record["metrics"] = {
                "pronation_angle_deg": event.value1,
                "z_ratio_delta": event.value2,
                "z_sign_flipped": bool(event.value3),
            }
        elif reason == "palm_down_gyro_angle_low":
            record["metrics"] = {
                "signed_gyro_y_angle_deg": event.value1,
                "required_angle_deg": event.value2,
                "gyro_y_peak_dps": event.value3,
            }
        else:
            record["metrics"] = {
                "gyro_xy_peak_ratio": event.value1,
                "required_ratio": event.value2,
                "gyro_x_peak_dps": event.value3,
            }
    elif stage == "reset" and reason == "palm_down_gate_failed":
        record["metrics"] = {
            "elapsed_until_reset_ms": event.value1,
            "signed_gyro_y_angle_deg": event.value2,
            "gyro_xy_peak_ratio": event.value3,
        }
    elif stage == "gyro_bias_ready":
        record["metrics"] = {
            "correction_dps": event.value1,
            "plane_bias_dps": event.value2,
            "z_bias_dps": event.value3,
        }
    return record


def trajectory_markers(
    diagnostics: list[dict[str, Any]],
    duration_ms: Optional[int] = None,
    stop_cue_delay_ms: Optional[int] = None,
) -> list[tuple[int, str]]:
    """Convert diagnostic wall-clock times to trajectory-relative markers."""
    start = next((r for r in diagnostics if r.get("stage") == "outbound_start"), None)
    if start is None:
        return []
    try:
        start_time = datetime.fromisoformat(str(start["time"]))
    except (KeyError, ValueError):
        return []
    labels = {
        "outbound_start": "candidate",
        "gyro_enabled": "gyro on",
        "outbound_ready": "dwell ready",
        "final_hold_start": "hold",
        "motion_complete": "motion complete",
        "match": "START OK",
        "stop_hand_lower": "STOP OK",
        "stop_palm_up": "STOP OK",
    }
    markers: list[tuple[int, str]] = []
    for record in diagnostics:
        label = labels.get(str(record.get("stage")))
        if label is None:
            continue
        try:
            when = datetime.fromisoformat(str(record["time"]))
        except (KeyError, ValueError):
            continue
        elapsed_ms = max(0, round((when - start_time).total_seconds() * 1000))
        if duration_ms is None or elapsed_ms <= duration_ms:
            markers.append((elapsed_ms, label))
        if record.get("stage") == "match" and stop_cue_delay_ms is not None:
            stop_go_ms = elapsed_ms + stop_cue_delay_ms
            if duration_ms is None or stop_go_ms <= duration_ms:
                markers.append((stop_go_ms, "STOP GO"))
    return markers


class GestureValidator:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.client: Any = None
        self.queue: asyncio.Queue[GestureEvent] = asyncio.Queue()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.audio_seen = asyncio.Event()
        self.results: list[TrialResult] = []
        self.all_diagnostics: list[dict[str, Any]] = []
        self.trajectory_assembler = TrajectoryAssembler()
        self.trajectory_queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self.address = ""
        self.closing = False
        if args.json_output:
            self.output_dir = Path(args.json_output).expanduser().parent
        else:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_dir = Path(__file__).resolve().parent / "output" / f"gesture_debug_{stamp}"

    def _enqueue(self, event: GestureEvent) -> None:
        self.queue.put_nowait(event)

    def _on_notify(self, _sender: Any, data: bytearray) -> None:
        raw = bytes(data)
        if len(raw) >= 3 and raw[2] in (
            EVT_TRAJECTORY_BEGIN, EVT_TRAJECTORY_CHUNK, EVT_TRAJECTORY_END
        ):
            completed = self.trajectory_assembler.feed(raw)
            if completed is not None and self.loop is not None:
                self.loop.call_soon_threadsafe(
                    self.trajectory_queue.put_nowait, completed
                )
            return
        event = parse_event_packet(raw)
        if event is not None:
            if event.code == EVT_GESTURE_DIAG:
                record = diagnostic_record(event)
                self.all_diagnostics.append(record)
            if self.loop is not None:
                self.loop.call_soon_threadsafe(self._enqueue, event)
            return

        # Audio packet: [seq][0xAA][PCM...]. 既に録音中かの検出だけに使う。
        if len(raw) >= 2 and raw[1] == 0xAA and self.loop is not None:
            self.loop.call_soon_threadsafe(self.audio_seen.set)

    def _on_disconnected(self, _client: Any) -> None:
        if self.closing:
            return
        print("\n  BLE接続が切断されました。", flush=True)
        if self.loop is None:
            return
        event = GestureEvent(
            code=EVT_DISCONNECTED,
            name=EVENT_NAMES[EVT_DISCONNECTED],
            monotonic_s=time.monotonic(),
            wall_time=datetime.now().isoformat(timespec="milliseconds"),
        )
        self.loop.call_soon_threadsafe(self._enqueue, event)

    async def _write_command(self, command: int) -> None:
        if self.client is None or not self.client.is_connected:
            return
        await self.client.write_gatt_char(
            AUDIO_RX_UUID, bytes([command]), response=True
        )

    def _drain_events(self) -> None:
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    def _drain_trajectories(self) -> None:
        self.trajectory_assembler.reset()
        while not self.trajectory_queue.empty():
            try:
                self.trajectory_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    async def _wait_for_event(self, code: int, timeout: float) -> bool:
        deadline = self.loop.time() + timeout if self.loop is not None else 0.0
        while self.loop is not None:
            remaining = deadline - self.loop.time()
            if remaining <= 0:
                return False
            try:
                event = await asyncio.wait_for(self.queue.get(), timeout=remaining)
            except asyncio.TimeoutError:
                return False
            if event.code == EVT_DISCONNECTED:
                raise ConnectionError("BLE connection lost")
            if event.code == code:
                return True
        return False

    async def connect(self) -> None:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as exc:
            raise RuntimeError(
                "bleak がありません。mac_client/venv/bin/pip install -r "
                "mac_client/requirements.txt を実行してください。"
            ) from exc

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
                raise RuntimeError(
                    f"デバイス '{self.args.device}' が見つかりません。"
                )
            self.address = target.address

        self.client = BleakClient(
            target,
            timeout=self.args.connect_timeout,
            disconnected_callback=self._on_disconnected,
        )
        try:
            await self.client.connect()
            self.debug = DebugControl()
            await self.client.start_notify(AUDIO_TX_UUID, self.debug.wrap(self._on_notify))
            await self.debug.mode(self.client, 1)

            # イベントはprimary接続だけへ送られるため、検証中だけprimaryを引き受ける。
            await self._write_command(0x02)
            print("BLE接続: OK", flush=True)

            # 接続前から録音中なら音声パケットが届く。その場合だけ停止して初期化する。
            await asyncio.sleep(0.8)
            if self.audio_seen.is_set():
                await self._write_command(0x00)
                await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
            self._drain_events()

            if not self.args.skip_preflight:
                await self.preflight()

        except BaseException:
            await self.disconnect()
            raise

    async def preflight(self) -> None:
        """BLE writeとevent notifyの往復を実動作の前に検証する。"""
        self._drain_events()
        await self._write_command(0x01)
        started = await self._wait_for_event(EVT_RECORDING_START, timeout=3.0)
        if not started:
            raise RuntimeError("preflightでrecording_startを受信できません")

        await self._write_command(0x00)
        stopped = await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
        if not stopped:
            raise RuntimeError("preflightでrecording_stopを受信できません")

        self._drain_events()
        print("BLEイベント通信: OK", flush=True)
        await asyncio.sleep(0.5)

    async def disconnect(self) -> None:
        if self.client is None:
            return
        self.closing = True
        if self.client.is_connected:
            try:
                # 他の接続があればprimaryを返す。
                await self._write_command(0x03)
            except Exception:
                pass
            try:
                if self.client.is_connected and hasattr(self, 'debug'):
                    await self.debug.mode(self.client, 2)
            except Exception as exc:
                print(f'Debug cleanup failed; disconnect restores normal: {exc}', flush=True)
            finally:
                await self.client.disconnect()

    async def _countdown(self, trial: int) -> None:
        print("", flush=True)
        print(f"試行 {trial}/{self.args.trials}", flush=True)
        print(f"  指示: {self.args.instruction}", flush=True)
        print("  MacのPing音が鳴ってから動作を始めてください。", flush=True)
        for remaining in range(self.args.countdown, 0, -1):
            print(f"  {remaining}...", flush=True)
            await asyncio.sleep(1.0)

    async def _play_cue(self, sound: str) -> None:
        if self.args.no_cue_sound:
            return
        sound_path = Path(sound).expanduser()
        if not sound_path.is_file():
            raise RuntimeError(f"合図の音声ファイルがありません: {sound_path}")
        await asyncio.create_subprocess_exec(
            "/usr/bin/afplay",
            str(sound_path),
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )

    async def _announce_go(self, trial: int) -> None:
        await self._play_cue(self.args.cue_sound)
        print("  >>> GO <<<", flush=True)

    async def _announce_stop_go(self) -> None:
        await self._play_cue(self.args.stop_cue_sound)
        print("  >>> STOP GO <<< 掌は返さず腕を下ろしてください", flush=True)

    async def run_trial(self, trial: int) -> TrialResult:
        await self.debug.mode(self.client, 1)
        self._drain_events()
        self._drain_trajectories()
        await self._countdown(trial)
        assert self.loop is not None

        # GO前に届いたイベントを試行成立として数えない。早すぎる発動で録音中に
        # なっていた場合は停止し、この試行を明示的に失敗とする。
        early_match = False
        while not self.queue.empty():
            try:
                early_event = self.queue.get_nowait()
            except asyncio.QueueEmpty:
                break
            if early_event.code == EVT_RECORDING_START:
                early_match = True
        if early_match:
            await self._write_command(0x00)
            await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
            early_conditions = build_condition_results([], matched=False)
            early_conditions[-2] = ConditionResult(
                "1回の動作で完結", "FAIL", "GO表示前に発動したため試行無効"
            )
            early_conditions[-1] = ConditionResult(
                "ジェスチャー発動", "FAIL", "GO表示前の発動は判定対象外"
            )
            result = TrialResult(
                trial=trial,
                expected=self.args.expect,
                result="FAIL",
                matched=True,
                latency_ms=0,
                motion_active_seen=False,
                motion_settled_seen=False,
                sequence_reset_seen=False,
                reason="GO表示前にrecording_startを受信",
                diagnostics=[],
                conditions=early_conditions,
            )
            print_condition_results(result.conditions)
            print(f"  結果: FAIL — {result.reason}", flush=True)
            return result

        start = self.loop.time()
        await self._announce_go(trial)

        deadline = start + self.args.window
        matched = False
        gesture_stopped = False
        stop_cued = False
        stop_before_cue = False
        latency_ms: Optional[int] = None
        stop_cue_latency_ms: Optional[int] = None
        stop_latency_ms: Optional[int] = None
        motion_active_seen = False
        motion_settled_seen = False
        disconnected = False
        diagnostics: list[dict[str, Any]] = []

        while self.loop.time() < deadline:
            remaining = deadline - self.loop.time()
            try:
                event = await asyncio.wait_for(self.queue.get(), timeout=remaining)
            except asyncio.TimeoutError:
                break

            if event.code == EVT_DISCONNECTED:
                disconnected = True
                break
            if event.code == EVT_MOTION_ACTIVE:
                motion_active_seen = True
            elif event.code == EVT_MOTION_SETTLED:
                motion_settled_seen = True
            elif event.code == EVT_GESTURE_DIAG:
                diagnostics.append(diagnostic_record(event))
            elif event.code == EVT_RECORDING_START:
                if not matched:
                    matched = True
                    latency_ms = max(
                        0, round((event.monotonic_s - start) * 1000)
                    )
                    if self.args.start_only:
                        print(
                            f"  >>> START OK <<< {latency_ms} ms"
                            " — 開始のみ: ホスト停止します",
                            flush=True,
                        )
                        break
                    print(
                        f"  >>> START OK <<< {latency_ms} ms — 掌下を維持してください",
                        flush=True,
                    )
                    if self.args.expect == "match":
                        await asyncio.sleep(self.args.stop_cue_delay)
                        await self._announce_stop_go()
                        stop_cued = True
                        stop_cue_latency_ms = max(
                            0, round((self.loop.time() - start) * 1000)
                        )
                        # Keep enough time after STOP GO for the hand-lower.
                        deadline = max(deadline, self.loop.time() + 12.0)
            elif event.code == EVT_RECORDING_STOP:
                if matched and not self.args.start_only:
                    gesture_stopped = True
                    stop_latency_ms = max(
                        0, round((event.monotonic_s - start) * 1000)
                    )
                    stop_before_cue = not stop_cued or not stop_occurred_after_cue(
                        stop_cue_latency_ms, stop_latency_ms
                    )
                    break

        if matched and not gesture_stopped and not disconnected:
            await self._write_command(0x00)
            try:
                await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
            except ConnectionError:
                disconnected = True

        trajectory: Optional[dict[str, Any]] = None
        if not disconnected:
            try:
                trajectory = await asyncio.wait_for(
                    self.trajectory_queue.get(), timeout=3.0 if matched else 1.5
                )
            except asyncio.TimeoutError:
                trajectory = None

        reset_seen = sequence_reset_seen(diagnostics)

        if disconnected:
            result = "ERROR"
            reason = "BLE接続が切断されました"
        elif self.args.expect == "match":
            if matched and reset_seen:
                result = "FAIL"
                reason = (
                    "recording_start前にgesture resetを検出"
                    "（単一動作試行として無効）"
                )
            elif self.args.start_only:
                result = "PASS" if matched else "FAIL"
                if matched:
                    reason = (
                        f"recording_startを{latency_ms} msで受信"
                        "（開始のみ）"
                    )
                else:
                    reason = f"{self.args.window:g}秒以内にrecording_startなし"
            else:
                result = (
                    "PASS"
                    if matched and gesture_stopped and not stop_before_cue
                    else "FAIL"
                )
                if matched and gesture_stopped and stop_before_cue:
                    reason = "STOP GO前にrecording_stopを受信"
                elif matched and gesture_stopped:
                    reason = (
                        f"recording_startを{latency_ms} msで受信し、"
                        f"STOP GO後"
                        f"{(stop_latency_ms or 0) - (stop_cue_latency_ms or 0)} msで停止"
                    )
                elif matched:
                    reason = (
                        f"recording_startを{latency_ms} msで受信したが"
                        "手下ろし停止なし"
                    )
                else:
                    reason = f"{self.args.window:g}秒以内にrecording_startなし"
        else:
            result = "FAIL" if matched else "PASS"
            reason = (
                f"意図しないrecording_startを{latency_ms} msで受信"
                if matched
                else f"{self.args.window:g}秒間、誤発動なし"
            )

        conditions = build_condition_results(diagnostics, matched)
        if self.args.start_only:
            conditions.append(
                ConditionResult(
                    "手下ろしで録音終了",
                    "SKIP",
                    "開始のみモード（ホスト0x00で停止）",
                )
            )
            stop_diag = None
        else:
            stop_diag = _latest_diagnostic(
                diagnostics, stage="stop_hand_lower"
            ) or _latest_diagnostic(diagnostics, stage="stop_palm_up")
        if stop_diag is not None:
            opp_imp = float(stop_diag.get("value1") or 0.0)
            opp_peak = float(stop_diag.get("value2") or 0.0)
            pulse_ms = float(stop_diag.get("value3") or 0.0)
            match_diag = _latest_diagnostic(diagnostics, stage="match")
            lift_imp = float((match_diag or {}).get("value2") or 0.0)
            stop_t = float(stop_diag.get("t") or 0.0)
            match_t = float((match_diag or {}).get("t") or 0.0)
            elapsed_ms = max(0.0, (stop_t - match_t) * 1000.0) if match_t else 0.0
            stop_ok = recording_stop_hand_lower_eligible(
                opp_imp, opp_peak, pulse_ms, lift_imp, elapsed_ms
            )
            need_imp, peak_min, settle_ms, _lin = recording_stop_active_thresholds(
                elapsed_ms, lift_imp
            )
            conditions.append(
                ConditionResult(
                    "手下ろしで録音終了",
                    (
                        "PASS"
                        if stop_ok and gesture_stopped and not stop_before_cue
                        else "FAIL"
                    ),
                    f"実測 opp_imp={opp_imp:.3f} peak={opp_peak:.2f} "
                    f"pulse={pulse_ms:.0f}ms lift_imp={lift_imp:.3f} "
                    f"elapsed={elapsed_ms:.0f}ms | "
                    f"閾値 opp_imp≥{need_imp:.3f} peak≥{peak_min:.2f} "
                    f"pulse {STOP_OPP_PULSE_MIN_MS}-{STOP_OPP_PULSE_MAX_MS}ms "
                    f"+ settle {settle_ms:.0f}ms "
                    f"/ 開始後{STOP_POST_START_INHIBIT_MS}ms抑制"
                    + (" / STOP GO前に停止" if stop_before_cue else ""),
                )
            )
        elif not self.args.start_only and gesture_stopped:
            conditions.append(
                ConditionResult(
                    "手下ろしで録音終了",
                    "FAIL" if stop_before_cue else "PASS",
                    (
                        "STOP GO前にrecording_stopを受信"
                        if stop_before_cue
                        else "recording_stopを受信（stop_hand_lower diag なし）"
                    ),
                )
            )
        elif not self.args.start_only and matched:
            conditions.append(
                ConditionResult(
                    "手下ろしで録音終了",
                    "FAIL",
                    "判定時間内に手下ろし停止なし",
                )
            )
        elif not self.args.start_only:
            conditions.append(
                ConditionResult(
                    "手下ろしで録音終了",
                    "NOT_REACHED",
                    "録音開始前",
                )
            )
        trial_result = TrialResult(
            trial=trial,
            expected=self.args.expect,
            result=result,
            matched=matched,
            latency_ms=latency_ms,
            motion_active_seen=motion_active_seen,
            motion_settled_seen=motion_settled_seen,
            sequence_reset_seen=reset_seen,
            reason=reason,
            diagnostics=diagnostics,
            conditions=conditions,
            trajectory=trajectory,
            stop_cue_latency_ms=stop_cue_latency_ms,
            stop_latency_ms=stop_latency_ms,
            stop_before_cue=stop_before_cue,
        )
        print_condition_results(trial_result.conditions)
        if getattr(self.args, "show_gyro", False):
            summarize_gyro_y_samples(diagnostics)
        gyro_csv = getattr(self.args, "gyro_csv", None)
        if gyro_csv:
            stem = Path(gyro_csv)
            out = (
                stem
                if self.args.trials == 1
                else stem.with_name(f"{stem.stem}_t{trial}{stem.suffix}")
            )
            write_gyro_csv(str(out), diagnostics)
        # Always summarize briefly when debug samples arrived.
        if any(r.get("stage") == "gyro_y_sample" for r in diagnostics):
            if not getattr(self.args, "show_gyro", False):
                summarize_gyro_y_samples(diagnostics)
        if trajectory is not None:
            output_dir = self.output_dir
            csv_path = output_dir / f"trial_{trial:02d}.csv"
            png_path = output_dir / f"trial_{trial:02d}.png"
            write_trajectory_csv(csv_path, trajectory)
            trial_result.artifacts = {"csv": str(csv_path)}
            if not self.args.no_plot_files:
                samples = trajectory.get("samples", [])
                duration_ms = samples[-1]["elapsed_ms"] if samples else None
                markers = trajectory_markers(
                    diagnostics,
                    duration_ms,
                    round(self.args.stop_cue_delay * 1000),
                )
                try:
                    plot_trajectory(
                        trajectory,
                        png_path,
                        title=f"HarnessNode trial {trial:02d}",
                        markers=markers,
                        show=not self.args.no_plot,
                    )
                    trial_result.artifacts["png"] = str(png_path)
                except Exception as exc:
                    print(f"  グラフ生成失敗: {exc}", flush=True)
            print(
                f"  6軸履歴: {len(trajectory.get('samples', []))} samples "
                f"({'complete' if trajectory.get('complete') else 'incomplete'})",
                flush=True,
            )
            print(f"  CSV保存: {csv_path}", flush=True)
        print(f"  結果: {result} — {reason}", flush=True)
        return trial_result

    async def run(self) -> int:
        started_at = datetime.now().isoformat(timespec="seconds")
        await self.connect()
        print("", flush=True)
        print("=" * 64, flush=True)
        print("HarnessNode 掌上0.5秒静止→挙上→掌下静止 BLE検証", flush=True)
        print(f"期待結果: {self.args.expect}", flush=True)
        if self.args.start_only:
            print("モード: 開始のみ（停止ジェスチャは評価しない）", flush=True)
        print(f"試行回数: {self.args.trials} / 判定時間: {self.args.window:g}秒", flush=True)
        print("=" * 64, flush=True)

        try:
            for trial in range(1, self.args.trials + 1):
                result = await self.run_trial(trial)
                self.results.append(result)
                if result.result == "ERROR":
                    break
                if trial < self.args.trials:
                    await asyncio.sleep(self.args.interval)
        finally:
            await self.disconnect()

        passed = sum(result.result == "PASS" for result in self.results)
        failed = sum(result.result == "FAIL" for result in self.results)
        errors = sum(result.result == "ERROR" for result in self.results)
        total = len(self.results)
        overall = "PASS" if total == self.args.trials and failed == 0 and errors == 0 else "FAIL"

        print("", flush=True)
        print("=" * 64, flush=True)
        print(f"総合結果: {overall}  PASS={passed} FAIL={failed} ERROR={errors}", flush=True)
        print("=" * 64, flush=True)

        report = {
                "started_at": started_at,
                "finished_at": datetime.now().isoformat(timespec="seconds"),
                "device": self.args.device,
                "address": self.address,
                "expected": self.args.expect,
                "instruction": self.args.instruction,
                "window_s": self.args.window,
                "start_only": self.args.start_only,
                "stop_cue_delay_s": self.args.stop_cue_delay,
                "start_cue_sound": self.args.cue_sound,
                "stop_cue_sound": self.args.stop_cue_sound,
                "overall": overall,
                "results": [asdict(result) for result in self.results],
                "diagnostics": self.all_diagnostics,
        }
        output_path = (
            Path(self.args.json_output).expanduser()
            if self.args.json_output
            else self.output_dir / "report.json"
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"レポート保存: {output_path}", flush=True)

        return 0 if overall == "PASS" else 1


def run_self_test() -> int:
    start = parse_event_packet(bytes([0x00, 0x55, EVT_RECORDING_START]), now=12.5)
    assert start is not None
    assert start.code == EVT_RECORDING_START
    assert start.monotonic_s == 12.5

    xyz_packet = bytearray([0x00, 0x55, EVT_MOTION_ACTIVE])
    xyz_packet.extend(struct.pack("<fff", 1.25, -2.5, 9.75))
    motion = parse_event_packet(bytes(xyz_packet), now=13.0)
    assert motion is not None
    assert motion.name == "motion_active"
    assert motion.x is not None and math.isclose(motion.x, 1.25)
    assert motion.y is not None and math.isclose(motion.y, -2.5)
    assert motion.z is not None and math.isclose(motion.z, 9.75)

    assert parse_event_packet(bytes([0x01, 0xAA, 0x00, 0x00])) is None
    assert parse_event_packet(bytes([0x00, 0x55])) is None

    assembler = TrajectoryAssembler()
    begin = bytearray([0x00, 0x55, EVT_TRAJECTORY_BEGIN, 1, 7, 1, 0])
    begin.extend(struct.pack("<HHf", 2, 25, 1.5))
    assert assembler.feed(bytes(begin)) is None
    chunk = bytearray([0x00, 0x55, EVT_TRAJECTORY_CHUNK, 7])
    chunk.extend(struct.pack("<HB", 0, 2))
    chunk.extend(struct.pack("<HBffffff", 0, 0, 1, 2, 3, 0, 0, 0))
    chunk.extend(struct.pack("<HBffffff", 25, 7, 4, 5, 6, 7, 8, 9))
    assert assembler.feed(bytes(chunk)) is None
    end = bytes([0x00, 0x55, EVT_TRAJECTORY_END, 7, 2, 0, 0])
    trajectory = assembler.feed(end)
    assert trajectory is not None and trajectory["complete"]
    assert trajectory["samples"][0]["gx_dps"] is None
    assert math.isclose(trajectory["samples"][1]["gy_dps"], 8.0)
    assert motion_completed_in_time(0.0)
    assert motion_completed_in_time(2999.0)
    assert motion_completed_in_time(3000.0)
    assert motion_completed_in_time(4499.0)
    assert not motion_completed_in_time(4500.0)
    # 0.0.74: exit RMS 4.0, need 3 consecutive samples
    assert FINAL_HOLD_RMS_EXIT_MS2 == 4.0
    assert FINAL_HOLD_RMS_EXIT_SAMPLES == 3
    assert LIFT_START_TIMEOUT_MS == 8000
    assert not hold_rms_interrupts([3.08])
    assert not hold_rms_interrupts([4.1, 4.2])  # only 2 samples
    assert not hold_rms_interrupts([3.6, 3.5, 3.6])  # below 4.0
    assert hold_rms_interrupts([4.1, 4.2, 4.05])

    complete_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x23, 0x00]
    )
    complete_packet.extend(struct.pack("<fff", 2425.0, 573.4, 134.8))
    complete = parse_event_packet(bytes(complete_packet), now=13.5)
    assert complete is not None
    assert "elapsed=2425ms" in format_event(complete)
    assert math.isclose(
        diagnostic_record(complete)["metrics"]["motion_elapsed_ms"], 2425.0
    )

    slow_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x80, 0x24]
    )
    slow_packet.extend(struct.pack("<fff", 3000.0, 178.2, 149.3))
    slow = parse_event_packet(bytes(slow_packet), now=13.6)
    assert slow is not None
    slow_record = diagnostic_record(slow)
    assert slow_record["reason"] == "motion_too_slow"
    assert math.isclose(slow_record["metrics"]["motion_elapsed_ms"], 3000.0)

    gate_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x24, 0x27]
    )
    gate_packet.extend(struct.pack("<fff", 0.31, 0.42, 174.0))
    gate = parse_event_packet(bytes(gate_packet), now=13.7)
    assert gate is not None
    assert "palm_down_xy_ratio_low" in format_event(gate)
    gate_record = diagnostic_record(gate)
    assert math.isclose(
        gate_record["metrics"]["gyro_xy_peak_ratio"], 0.31, abs_tol=1e-6
    )

    gate_reset_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x80, 0x28]
    )
    gate_reset_packet.extend(struct.pack("<fff", 3010.0, 122.2, 0.31))
    gate_reset = parse_event_packet(bytes(gate_reset_packet), now=13.8)
    assert gate_reset is not None
    assert "palm_down_gate_failed" in format_event(gate_reset)
    assert not stop_occurred_after_cue(None, 4200)
    assert not stop_occurred_after_cue(4200, 4199)
    assert stop_occurred_after_cue(4200, 4200)
    gate_conditions = build_condition_results(
        [
            {
                "stage": "outbound_start", "reason": "none",
                "value1": 0.90, "value2": 0.0, "value3": 0.2,
            },
            {
                "stage": "outbound_ready", "reason": "none",
                "value1": 520.0, "value2": 0.88, "value3": 0.0,
            },
            {
                "stage": "palm_down_gate", "reason": "palm_down_xy_ratio_low",
                "value1": 0.31, "value2": 0.42, "value3": 174.0,
            },
            {
                "stage": "reset", "reason": "palm_down_gate_failed",
                "value1": 3010.0, "value2": 122.2, "value3": 0.31,
            },
        ],
        matched=False,
    )
    assert next(c for c in gate_conditions if c.label == "挙上").status == "PASS"
    assert (
        next(c for c in gate_conditions if c.label == "3秒以内に完了").status
        == "NOT_REACHED"
    )
    palm_down_condition = next(
        c for c in gate_conditions if c.label == "掌下で静止"
    )
    assert palm_down_condition.status == "FAIL"
    assert "0.31" in palm_down_condition.detail
    print("SELF_TEST: PASS", flush=True)
    return 0

    diag_packet = bytearray([0x00, 0x55, EVT_GESTURE_DIAG, 0x02, 0x00])
    diag_packet.extend(struct.pack("<fff", 1000.0, 0.92, 0.12))
    diag = parse_event_packet(bytes(diag_packet), now=14.0)
    assert diag is not None
    assert diag.diag_stage == 0x02
    assert diag.value1 is not None and math.isclose(diag.value1, 1000.0)
    assert "dwell=1000ms" in format_event(diag)
    assert "z=0.92" in format_event(diag)
    assert math.isclose(
        diagnostic_record(diag)["metrics"]["dwell_ms"], 1000.0
    )

    stop_packet = bytearray([0x00, 0x55, EVT_GESTURE_DIAG, 0x0C, 0x00])
    stop_packet.extend(struct.pack("<fff", 0.42, 2.5, 280.0))
    stop_diag = parse_event_packet(bytes(stop_packet), now=14.2)
    assert stop_diag is not None
    assert stop_diag.diag_stage == 0x0C
    assert "stop_hand_lower" in format_event(stop_diag)
    assert math.isclose(
        diagnostic_record(stop_diag)["metrics"]["opp_impulse_ms"],
        0.42,
        abs_tol=1e-6,
    )

    gyro_packet = bytearray([0x00, 0x55, EVT_GESTURE_DIAG, 0x20, 0x00])
    gyro_packet.extend(struct.pack("<fff", -12.5, 0.15, 250.0))
    gyro = parse_event_packet(bytes(gyro_packet), now=14.5)
    assert gyro is not None
    assert gyro.diag_stage == 0x20
    assert "gyro_y=-12.5dps" in format_event(gyro)
    assert math.isclose(
        diagnostic_record(gyro)["metrics"]["gyro_y_dps"], -12.5
    )

    final_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x08, 0x00]
    )
    final_packet.extend(struct.pack("<fff", 0.12, 525.0, 8.2))
    final = parse_event_packet(bytes(final_packet), now=16.0)
    assert final is not None
    assert final.diag_stage == 0x08
    assert "hold=525ms" in format_event(final)

    match_gate_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x10, 0x29]
    )
    match_gate_packet.extend(struct.pack("<fff", 0.638, 0.65, 153.6))
    match_gate = parse_event_packet(bytes(match_gate_packet), now=16.5)
    assert match_gate is not None
    assert "required=0.65m/s" in format_event(match_gate)
    match_gate_record = diagnostic_record(match_gate)
    assert math.isclose(
        match_gate_record["metrics"]["required_impulse_ms"],
        0.65,
        rel_tol=1e-6,
    )

    bias_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x0A, 0x00]
    )
    bias_packet.extend(struct.pack("<fff", 22.0, 21.5, 4.5))
    bias = parse_event_packet(bytes(bias_packet), now=17.0)
    assert bias is not None
    assert bias.diag_stage == 0x0A
    assert "correction=22.0dps" in format_event(bias)
    bias_record = diagnostic_record(bias)
    assert math.isclose(bias_record["metrics"]["correction_dps"], 22.0)

    # Firmware boundary: board-flat shake, palm-up flip, lift pulse, pose, hold.
    assert START_BOARD_FLAT_Z_MIN_RATIO == 0.75
    assert START_QUIET_HOLD_MS == 500
    assert START_QUIET_ACCEL_MAX_MS2 == 4.0
    assert HOLD_PRONATION_ANGLE_MIN_DEG == 15.0
    assert HOLD_PRONATION_Z_RATIO_DONE == 0.40
    assert HOLD_GYRO_ANGLE_MIN_DEG == 30.0
    assert HOLD_GYRO_INTEGRATE_RATE_DPS == 10.0
    assert MOTION_COMPLETE_MAX_MS == 4500
    assert FINAL_TILT_MAX_DEG == 15.0
    assert FINAL_HOLD_MIN_MS == 500
    assert MATCH_POS_IMPULSE_MIN_MS == 0.65
    assert MATCH_PRONATION_MIN_DEG == 140.0
    assert gesture_match_final_gate_eligible(0.65, 140.0)
    assert not gesture_match_final_gate_eligible(0.649, 140.0)
    assert not gesture_match_final_gate_eligible(0.65, 139.9)

    # De-identified (final positive impulse, palm-up reference angle) replay.
    # Android 2026-08-27 18:00+ labels all 45 records as false positives.
    android_false_positive_regression = [
        (0.453637, 94.341827),
        (0.380244, 147.922485),
        (0.606543, 124.721581),
        (0.541128, 120.005676),
        (0.619267, 140.790863),
        (0.515249, 89.626007),
        (0.316049, 100.010941),
        (0.521128, 35.383652),
        (0.391389, 72.995178),
        (0.454178, 72.241272),
        (0.523483, 41.429001),
        (0.422851, 101.257347),
        (0.706178, 42.493492),
        (0.352658, 81.310791),
        (0.379812, 92.696991),
        (0.514638, 76.439804),
        (0.307426, 99.555664),
        (0.317836, 52.513206),
        (0.924223, 120.045547),
        (0.326881, 61.969872),
        (0.454647, 173.499725),
        (0.392254, 47.517334),
        (0.545940, 41.548798),
        (0.375240, 141.261673),
        (0.508888, 111.016388),
        (0.307067, 113.648445),
        (0.333314, 103.727112),
        (0.638360, 153.558060),
        (0.420161, 137.856674),
        (0.778483, 137.790817),
        (0.478118, 88.654877),
        (0.579495, 85.624634),
        (0.302808, 97.471863),
        (0.782338, 71.117325),
        (0.633290, 90.213257),
        (0.629624, 163.651886),
        (0.422142, 117.385010),
        (0.335667, 69.190308),
        (0.657063, 64.689644),
        (0.690903, 69.442535),
        (1.178500, 92.087898),
        (0.438945, 42.783249),
        (0.467577, 52.722748),
        (0.403868, 59.182060),
        (0.558119, 91.195984),
    ]
    known_true_positive_regression = [
        (0.853, 179.0),
        (0.927, 179.1),
        (0.828, 147.3),
        (0.867, 177.9),
        (1.067, 179.6),
        (0.700, 170.8),
        (0.744, 177.3),
        (0.952, 173.7),
        (2.522, 163.5),
        (2.527, 158.4),
    ]
    assert not any(
        gesture_match_final_gate_eligible(*sample)
        for sample in android_false_positive_regression
    )
    assert all(
        gesture_match_final_gate_eligible(*sample)
        for sample in known_true_positive_regression
    )
    eligible = (0.90, 20.0, 0.30, 0.015, 15.0, 500)
    assert gesture_gate_eligible(*eligible)
    assert gesture_gate_eligible(0.75, *eligible[1:])
    assert not gesture_gate_eligible(0.74, *eligible[1:])
    assert not gesture_gate_eligible(0.90, 19.9, *eligible[2:])
    assert gesture_gate_eligible(0.90, 20.0, *eligible[2:])
    assert gesture_gate_eligible(0.90, 4.0, 0.04, 0.015, 15.0, 400, 0.50)
    assert not gesture_gate_eligible(0.90, 4.0, 0.04, 0.015, 15.0, 400, 0.49)
    assert gesture_gate_eligible(0.90, 4.0, 0.04, 0.015, 15.0, 400, 0.0, True)
    assert gesture_gate_eligible(
        0.90, 4.0, 0.04, 0.015, 15.0, 400, palm_up_tilt_deg=30.0
    )
    assert not gesture_gate_eligible(
        0.90, 4.0, 0.04, 0.015, 15.0, 400, palm_up_tilt_deg=29.9
    )
    # 0.0.69/0.0.73 stop: reverse lift-axis pulse + settle; soft time-decay.
    assert STOP_OPP_ACCEL_MIN_MS2 == 0.25
    assert STOP_OPP_IMPULSE_MIN_MS == 0.10
    assert STOP_OPP_IMPULSE_LIFT_RATIO == 0.20
    assert STOP_OPP_IMPULSE_LIFT_CAP_MS == 0.35
    assert STOP_OPP_PULSE_MIN_MS == 60
    assert STOP_OPP_PULSE_MAX_MS == 2000
    assert STOP_SETTLE_MS == 80
    assert STOP_POST_START_INHIBIT_MS == 3000
    assert STOP_SOFTEN_AFTER_MS == 5000
    assert STOP_SOFTEN2_AFTER_MS == 10000
    assert recording_stop_hand_lower_eligible(0.12, 1.0, 200.0, 0.50)
    assert recording_stop_hand_lower_eligible(0.10, 0.25, 60.0, 0.0)
    # リンゴ log shape should pass after 0.0.71
    assert recording_stop_hand_lower_eligible(0.189, 0.57, 732.0, 0.356)
    # 2026-08-28 リンゴ: soft stop that needed ~24s; eligible at t0 and soften.
    assert recording_stop_hand_lower_eligible(0.181, 1.75, 157.0, 0.337)
    assert recording_stop_hand_lower_eligible(
        0.181, 1.75, 157.0, 0.337, elapsed_ms=6000.0
    )
    assert not recording_stop_hand_lower_eligible(0.08, 1.0, 200.0, 0.0)
    # After soften2, impulse floor drops to 0.07 so 0.08 passes.
    assert recording_stop_hand_lower_eligible(
        0.08, 1.0, 200.0, 0.0, elapsed_ms=12000.0
    )
    # Short pulse still requires peak >= base min (no slow-path).
    assert not recording_stop_hand_lower_eligible(0.12, 0.20, 100.0, 0.0)
    # Slow-path: peak 0.16 with long pulse (soft2 floor 0.15).
    assert recording_stop_hand_lower_eligible(0.12, 0.16, 200.0, 0.0)
    assert not recording_stop_hand_lower_eligible(0.12, 0.14, 200.0, 0.0)
    assert not recording_stop_hand_lower_eligible(0.12, 1.0, 40.0, 0.0)
    assert not recording_stop_hand_lower_eligible(0.12, 1.0, 2100.0, 0.0)
    # Relative to lift, capped so strong lifts do not demand huge lowers
    assert recording_stop_hand_lower_eligible(0.35, 1.0, 200.0, 2.5)
    assert not recording_stop_hand_lower_eligible(0.30, 1.0, 200.0, 2.5)
    need5, peak5, settle5, lin5 = recording_stop_active_thresholds(6000.0, 0.50)
    assert math.isclose(need5, STOP_OPP_IMPULSE_SOFT_MS)
    assert math.isclose(peak5, STOP_OPP_ACCEL_SOFT_MS2)
    assert math.isclose(settle5, float(STOP_SETTLE_SOFT_MS))
    assert math.isclose(lin5, STOP_SETTLE_LINEAR_SOFT_MS2)
    # Palm-up / gyro stop path removed.
    assert not recording_stop_palm_up_eligible(20.0, z_ratio_delta=0.50)
    assert not recording_stop_palm_up_eligible(
        0.0, gyro_roll_deg=45.0, gyro_peak_dps=30.0
    )
    near_miss = bytearray([0x00, 0x55, EVT_GESTURE_DIAG, 0x0B, 0x2C])
    near_miss.extend(struct.pack("<fff", 0.09, 0.40, 0.10))
    near_evt = parse_event_packet(bytes(near_miss), now=20.0)
    assert near_evt is not None
    assert "stop_near_miss" in format_event(near_evt)
    assert "stop_impulse_low" in format_event(near_evt)

    def outbound_gyro_condition_status(roll_deg: float, peak_dps: float) -> str:
        conditions = build_condition_results(
            [
                {
                    "stage": "outbound_start",
                    "reason": "none",
                    "value1": 6.0,
                    "value2": 0.90,
                    "value3": 0.0,
                },
                {
                    "stage": "outbound_ready",
                    "reason": "none",
                    "value1": 0.0,
                    "value2": 0.0,
                    "value3": 0.0,
                },
                {
                    "stage": "outbound_gyro",
                    "reason": "none",
                    "value1": roll_deg,
                    "value2": peak_dps,
                    "value3": 1.0,
                },
            ],
            matched=False,
        )
        return next(
            condition.status
            for condition in conditions
            if condition.label == "掌上への反転"
        )

    assert outbound_gyro_condition_status(0.0, 49.9) == "FAIL"
    assert outbound_gyro_condition_status(0.0, 50.0) == "PASS"
    assert outbound_gyro_condition_status(44.9, 30.0) == "FAIL"
    assert outbound_gyro_condition_status(45.0, 29.9) == "FAIL"
    assert outbound_gyro_condition_status(45.0, 30.0) == "PASS"
    start_and_stop_gyro_conditions = build_condition_results(
        [
            {
                "stage": "outbound_start",
                "reason": "none",
                "value1": 6.0,
                "value2": 0.90,
                "value3": 0.0,
            },
            {
                "stage": "outbound_ready",
                "reason": "none",
                "value1": 0.0,
                "value2": 0.0,
                "value3": 0.0,
            },
            {
                "stage": "outbound_gyro",
                "reason": "none",
                "value1": 3.5,
                "value2": 66.0,
                "value3": 1.0,
            },
            {"stage": "match", "reason": "none"},
            {
                "stage": "stop_hand_lower",
                "reason": "none",
                "value1": 0.40,
                "value2": 2.5,
                "value3": 280.0,
            },
        ],
        matched=True,
    )
    start_condition = next(
        condition
        for condition in start_and_stop_gyro_conditions
        if condition.label == "掌上への反転"
    )
    assert start_condition.status == "PASS"
    assert "peak=66.0dps" in start_condition.detail
    assert not gesture_gate_eligible(0.90, 20.0, 0.29, *eligible[3:])
    assert not gesture_gate_eligible(0.90, 20.0, 0.30, 0.015, 15.1, 500)
    assert not gesture_gate_eligible(*eligible[:-1], 399)
    assert not gesture_gate_eligible(*eligible, shake_ptp_ms2=4.9)
    assert not gesture_gate_eligible(*eligible, shake_ptp_ms2=6.0, shake_mean_ms2=3.0)
    assert gesture_gate_eligible(*eligible, shake_ptp_ms2=6.0, shake_mean_ms2=2.0)
    assert not sequence_reset_seen([])
    assert not sequence_reset_seen([{"stage": "outbound_ready"}])
    assert sequence_reset_seen(
        [{"stage": "outbound_ready"}, {"stage": "reset"}]
    )
    retry_then_match = [
        {
            "stage": "outbound_start",
            "reason": "none",
            "value1": 6.2,
            "value2": 0.92,
            "value3": 0.20,
        },
        {
            "stage": "outbound_ready",
            "reason": "none",
            "value1": 35.0,
            "value2": 45.0,
            "value3": 0.55,
        },
        {
            "stage": "wait_reject",
            "reason": "final_brake_missing",
            "value1": 1.96,
            "value2": 0.0,
            "value3": 0.0,
        },
        {
            "stage": "final_hold_start",
            "reason": "none",
            "value1": 0.12,
            "value2": 0.10,
            "value3": 1.0,
        },
        {
            "stage": "final_ready",
            "reason": "none",
            "value1": 0.12,
            "value2": 525.0,
            "value3": 1.0,
        },
        {
            "stage": "match",
            "reason": "none",
            "value1": 35.0,
            "value2": 0.12,
            "value3": 525.0,
        },
    ]
    assert not sequence_reset_seen(retry_then_match)
    retry_conditions = build_condition_results(
        retry_then_match, matched=True
    )
    assert all(
        condition.status == "PASS" for condition in retry_conditions
    )

    passing_conditions = build_condition_results(
        [
            {
                "stage": "outbound_start",
                "reason": "none",
                "value1": 6.4,
                "value2": 0.92,
                "value3": 0.20,
            },
            {
                "stage": "outbound_ready",
                "reason": "none",
                "value1": 35.0,
                "value2": 45.0,
                "value3": 0.55,
            },
            {
                "stage": "final_hold_start",
                "reason": "none",
                "value1": 0.12,
                "value2": 0.10,
                "value3": 6.0,
            },
            {
                "stage": "final_ready",
                "reason": "none",
                "value1": 0.12,
                "value2": 525.0,
                "value3": 6.0,
            },
        ],
        matched=True,
    )
    assert all(condition.status == "PASS" for condition in passing_conditions)

    rejected_conditions = build_condition_results(
        [
            {
                "stage": "wait_reject",
                "reason": "start_not_palm_up",
                "value1": 0.40,
                "value2": 0.50,
                "value3": 5.0,
            }
        ],
        matched=False,
    )
    assert rejected_conditions[0].status == "FAIL"
    assert rejected_conditions[-1].status == "FAIL"
    for reason, values in (
        ("final_accel_missing", (0.02, 0.0, 2.0)),
        ("final_brake_missing", (0.10, 0.01, 2.0)),
        ("final_brake_ratio_low", (0.50, 0.06, 0.12)),
        ("final_pulse_duration_invalid", (100.0, 0.10, 0.08)),
        ("final_tilt_unstable", (0.12, 0.10, 18.0)),
        ("final_hold_timeout", (0.12, 0.10, 5.0)),
        ("lift_palm_still_up", (0.12, 0.10, 0.0)),
    ):
        conditions = build_condition_results(
            [
                {
                    "stage": "outbound_ready",
                    "reason": "none",
                    "value1": 35.0,
                    "value2": 35.0,
                    "value3": 0.55,
                },
                {
                    "stage": "reset",
                    "reason": reason,
                    "value1": values[0],
                    "value2": values[1],
                    "value3": values[2],
                },
            ],
            matched=False,
        )
        assert any(condition.status == "FAIL" for condition in conditions)
    z_only_conditions = build_condition_results(
        [
            {
                "stage": "outbound_start",
                "reason": "none",
                "value1": 6.0,
                "value2": 0.92,
                "value3": 0.18,
            },
            {
                "stage": "outbound_ready",
                "reason": "none",
                "value1": 12.0,
                "value2": 12.0,
                "value3": 0.55,
            },
            {
                "stage": "final_hold_start",
                "reason": "none",
                "value1": 0.12,
                "value2": 0.10,
                "value3": 6.0,
            },
            {
                "stage": "final_ready",
                "reason": "none",
                "value1": 0.12,
                "value2": 525.0,
                "value3": 6.0,
            },
        ],
        matched=True,
    )
    assert all(condition.status == "PASS" for condition in z_only_conditions)

    shake_fail_conditions = build_condition_results(
        [
            {
                "stage": "wait_reject",
                "reason": "shake_not_oscillatory",
                "value1": 6.0,
                "value2": 5.0,
                "value3": 0.83,
            }
        ],
        matched=False,
    )
    assert shake_fail_conditions[0].label == "掌下の短いシェイク"
    assert shake_fail_conditions[0].status == "FAIL"

    palm_up_lift_conditions = build_condition_results(
        [
            {
                "stage": "outbound_start",
                "reason": "none",
                "value1": 6.2,
                "value2": 0.92,
                "value3": 0.20,
            },
            {
                "stage": "outbound_ready",
                "reason": "none",
                "value1": 35.0,
                "value2": 45.0,
                "value3": 0.55,
            },
            {
                "stage": "wait_reject",
                "reason": "lift_palm_still_up",
                "value1": -0.80,
                "value2": 0.50,
                "value3": 0.12,
            },
            {
                "stage": "reset",
                "reason": "lift_palm_still_up",
                "value1": 0.12,
                "value2": 0.10,
                "value3": 0.0,
            },
        ],
        matched=False,
    )
    assert any(
        condition.label == "掌下で静止" and condition.status == "FAIL"
        for condition in palm_up_lift_conditions
    )
    assert any(
        condition.label == "挙上" and condition.status == "PASS"
        for condition in palm_up_lift_conditions
    )
    print("SELF_TEST: PASS", flush=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="HarnessNodeの掌上0.5秒静止→挙上→掌下静止をBLEで検証"
    )
    parser.add_argument("--device", default=DEVICE_NAME, help="BLEデバイス名")
    parser.add_argument("--address", help="スキャンせず接続するBLEアドレス/UUID")
    parser.add_argument("--trials", type=int, default=3, help="試行回数 (default: 3)")
    parser.add_argument(
        "--expect",
        choices=("match", "no-match"),
        default="match",
        help="各試行の期待結果 (default: match)",
    )
    parser.add_argument(
        "--instruction",
        help="試行時にユーザーへ表示する動作指示",
    )
    parser.add_argument(
        "--countdown", type=int, default=3, help="GOまでの秒数 (default: 3)"
    )
    parser.add_argument(
        "--window", type=float, default=15.0, help="GO後の判定秒数 (default: 15)"
    )
    parser.add_argument(
        "--interval", type=float, default=2.0, help="試行間隔秒 (default: 2)"
    )
    parser.add_argument(
        "--cue-sound",
        default=DEFAULT_CUE_SOUND,
        help=f"GO時に再生する音声ファイル (default: {DEFAULT_CUE_SOUND})",
    )
    parser.add_argument(
        "--stop-cue-sound",
        default=DEFAULT_STOP_CUE_SOUND,
        help=f"STOP GO時に再生する音声ファイル (default: {DEFAULT_STOP_CUE_SOUND})",
    )
    parser.add_argument(
        "--stop-cue-delay",
        type=float,
        default=1.3,
        help="START OKからSTOP GOまで掌下を維持する秒数 (default: 1.3)",
    )
    parser.add_argument(
        "--start-only",
        action="store_true",
        help="録音開始のみ判定（STOP GOなし。開始後はホスト0x00で停止）",
    )
    parser.add_argument(
        "--no-cue-sound",
        action="store_true",
        help="GO時のMacの合図音を無効化",
    )
    parser.add_argument(
        "--scan-timeout", type=float, default=15.0, help="スキャン秒数"
    )
    parser.add_argument(
        "--connect-timeout", type=float, default=10.0, help="接続タイムアウト秒"
    )
    parser.add_argument("--json-output", help="JSONレポートの保存先")
    parser.add_argument(
        "--no-plot", action="store_true",
        help="6軸PNGは保存するがグラフウィンドウは表示しない",
    )
    parser.add_argument(
        "--no-plot-files", action="store_true",
        help="6軸グラフPNGを生成しない",
    )
    parser.add_argument(
        "--gyro-csv",
        help="デバッグ用 gyro_y_sample をCSV保存（ファーム GESTURE_DEBUG_GYRO_Y=1 時）",
    )
    parser.add_argument(
        "--show-gyro",
        action="store_true",
        help="gyro_y_sample の min/max 要約を画面表示",
    )
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="BLE start/stopイベントの事前往復確認を省略",
    )
    parser.add_argument(
        "--self-test", action="store_true", help="BLE接続なしの内部テストを実行"
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    if args.trials < 1:
        parser.error("--trials は1以上にしてください")
    if args.countdown < 0:
        parser.error("--countdown は0以上にしてください")
    if args.window <= 0 or args.interval < 0 or args.stop_cue_delay < 0:
        parser.error(
            "--window は正、--intervalと--stop-cue-delayは0以上にしてください"
        )
    if args.instruction is None:
        if args.expect != "match":
            args.instruction = "指定された非対象動作を行ってください"
        elif args.start_only:
            args.instruction = (
                "手の甲側装着で掌を上にして0.5秒静止し、手を上げてから、"
                "掌を下にしてSTART OKまで維持してください。"
                "停止ジェスチャは不要です（開始のみ）"
            )
        else:
            args.instruction = (
                "手の甲側装着で掌を上にして0.5秒静止し、手を上げてから、"
                "掌を下にしてSTART OKまで維持してください。"
                "Glass音とSTOP GOのあとに掌は返さず腕を下ろしてください"
            )

    try:
        return asyncio.run(GestureValidator(args).run())
    except KeyboardInterrupt:
        print("\n中断しました。", flush=True)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
