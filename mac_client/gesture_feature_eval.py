#!/usr/bin/env python3
"""Evaluate candidate lift features against the firmware's current a_up gate.

The firmware estimates gravity with a plain 0.30 s low-pass and treats the
residual as linear acceleration (main.c:1678-1692).  That low-pass cannot track
the pronation flip, so part of the rotation leaks into the "lift" impulse the
final match gate reads (main.c:3008-3013).  This module reproduces the firmware
estimator, provides a gyro-propagated alternative, and measures how much of the
impulse each one attributes to a motion that contains no translation at all.

Stdlib only, matching gesture_dataset_analyzer.py.
"""

from __future__ import annotations

import argparse
import math
import random
from typing import Any, Callable, Iterable

G = 9.80665
DEG = 180.0 / math.pi

# --- firmware constants (nordic-main/src/main.c) ---
GRAVITY_LP_TAU_S = 0.30           # :291
SAMPLE_INTERVAL_MS = 25           # :87  MOTION_SAMPLE_INTERVAL_MS
LIFT_ACCEL_MIN_MS2 = 0.40         # :266
LIFT_BRAKE_MIN_MS2 = 0.15         # :267
LIFT_POS_IMPULSE_MIN_MS = 0.30    # :269
LIFT_NEG_IMPULSE_MIN_MS = 0.015   # :274
LIFT_CONSECUTIVE_SAMPLES = 2      # :277
MATCH_POS_IMPULSE_MIN_MS = 0.65   # :272

# --- gyro-propagated estimator ---
GYRO_CORRECT_TAU_S = 2.0
GYRO_CORRECT_ACCEL_BAND_MS2 = 0.5


def _cross(a: tuple, b: tuple) -> tuple:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: tuple) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


class LpfGravity:
    """Reproduces update_gravity_lp() at main.c:1678."""

    name = "lpf(tau=0.30s)"

    def __init__(self, tau_s: float = GRAVITY_LP_TAU_S) -> None:
        self.tau_s = tau_s
        self.g = None

    def update(self, accel: tuple, gyro_dps: tuple, dt_s: float) -> float:
        if self.g is None:
            self.g = accel
        alpha = dt_s / (self.tau_s + dt_s)
        self.g = tuple(
            self.g[i] + (accel[i] - self.g[i]) * alpha for i in range(3)
        )
        linear = tuple(accel[i] - self.g[i] for i in range(3))
        n = _norm(self.g)
        if n <= 0.1:
            return 0.0
        return sum(linear[i] * self.g[i] for i in range(3)) / n


class GyroGravity:
    """Propagates gravity with the gyro, correcting slowly from the accel.

    The gyro is already powered before the lift: it is enabled the moment the
    palm-up dwell is accepted and stays on through recording (main.c:190-193),
    so this needs no extra power.
    """

    name = "gyro-propagated"

    def __init__(
        self,
        tau_s: float = GYRO_CORRECT_TAU_S,
        band_ms2: float = GYRO_CORRECT_ACCEL_BAND_MS2,
    ) -> None:
        self.tau_s = tau_s
        self.band_ms2 = band_ms2
        self.g = None

    def update(self, accel: tuple, gyro_dps: tuple, dt_s: float) -> float:
        if self.g is None:
            self.g = accel
        omega = tuple(v / DEG for v in gyro_dps)
        # A world-fixed vector seen from the rotating body obeys dg/dt = -w x g,
        # i.e. g rotates about -w by |w|dt.  Euler stepping that inflates |g| by
        # O(|w dt|^2) -- 2% per sample at 470 dps -- which shows up as a large
        # spurious negative a_up, so integrate the rotation exactly.
        rate = _norm(omega)
        angle = rate * dt_s
        if angle > 1e-9:
            k = tuple(-omega[i] / rate for i in range(3))
            c = math.cos(angle)
            s = math.sin(angle)
            kg = _cross(k, self.g)
            kd = sum(k[i] * self.g[i] for i in range(3))
            self.g = tuple(
                self.g[i] * c + kg[i] * s + k[i] * kd * (1.0 - c)
                for i in range(3)
            )
        # Correct only while the accelerometer is plausibly reading gravity.
        if abs(_norm(accel) - G) < self.band_ms2:
            alpha = dt_s / (self.tau_s + dt_s)
            self.g = tuple(
                self.g[i] + (accel[i] - self.g[i]) * alpha for i in range(3)
            )
        linear = tuple(accel[i] - self.g[i] for i in range(3))
        n = _norm(self.g)
        if n <= 0.1:
            return 0.0
        return sum(linear[i] * self.g[i] for i in range(3)) / n


def perturb(samples: list, bias_dps: float, noise_dps: float, seed: int = 7) -> list:
    """Add a constant gyro bias and white noise, the errors the real IMU has.

    The firmware only captures a gyro bias while the wrist is still and rejects
    the capture above GESTURE_GYRO_BIAS_CAPTURE_MAX_DPS = 15 dps (main.c:219),
    so a few dps of uncorrected bias is the realistic residual.
    """
    rng = random.Random(seed)
    out = []
    for accel, gyro in samples:
        out.append(
            (
                accel,
                tuple(
                    gyro[i] + bias_dps + rng.gauss(0.0, noise_dps)
                    for i in range(3)
                ),
            )
        )
    return out


def run_lift_state_machine(a_ups: Iterable[float], dt_s: float) -> dict:
    """Reproduce the WAIT_ACCEL / WAIT_BRAKE impulse accounting.

    Mirrors main.c:2752-2851.  Returns the pre-flip latch value the firmware
    reports as pos_imp_at_lift and the running total the final match gate reads.
    """
    stage = "WAIT_ACCEL"
    pos_imp = 0.0
    neg_imp = 0.0
    accel_samples = 0
    brake_samples = 0
    event_started = False
    peak = 0.0
    imp_at_latch = None
    brake_ok = False

    for a_up in a_ups:
        peak = max(peak, a_up)
        if stage == "WAIT_ACCEL":
            if a_up >= LIFT_ACCEL_MIN_MS2:
                event_started = True
                accel_samples += 1
                pos_imp += a_up * dt_s
            elif event_started and a_up > 0.0:
                pos_imp += a_up * dt_s
                accel_samples = 0
            elif a_up <= 0.0:
                event_started = False
                accel_samples = 0
                pos_imp = 0.0
            else:
                accel_samples = 0
            if (
                accel_samples >= LIFT_CONSECUTIVE_SAMPLES
                and pos_imp >= LIFT_POS_IMPULSE_MIN_MS
            ):
                imp_at_latch = pos_imp
                stage = "WAIT_BRAKE"
                brake_samples = 0
        elif stage == "WAIT_BRAKE":
            if a_up > 0.0:
                pos_imp += a_up * dt_s
            else:
                neg_imp += -a_up * dt_s
            if a_up <= -LIFT_BRAKE_MIN_MS2:
                brake_samples = min(brake_samples + 1, LIFT_CONSECUTIVE_SAMPLES)
            elif brake_samples < LIFT_CONSECUTIVE_SAMPLES:
                brake_samples = 0
            if (
                brake_samples >= LIFT_CONSECUTIVE_SAMPLES
                and neg_imp >= LIFT_NEG_IMPULSE_MIN_MS
            ):
                brake_ok = True

    return {
        "latched": imp_at_latch is not None,
        "imp_at_latch": imp_at_latch,
        "pos_impulse_total": pos_imp,
        "neg_impulse": neg_imp,
        "peak_a_up": peak,
        "brake_ok": brake_ok,
        "match_lift_ok": pos_imp >= MATCH_POS_IMPULSE_MIN_MS,
    }


# ---------------------------------------------------------------- synthesis

def _smooth_angle(t: float, duration: float, total: float) -> tuple:
    """Minimum-acceleration angle profile; returns (theta, omega)."""
    if t <= 0.0:
        return 0.0, 0.0
    if t >= duration:
        return total, 0.0
    theta = total * (1.0 - math.cos(math.pi * t / duration)) / 2.0
    omega = total * math.pi / (2.0 * duration) * math.sin(math.pi * t / duration)
    return theta, omega


def synth_pure_rotation(
    flip_s: float,
    dt_s: float,
    still_s: float = 1.0,
    total_deg: float = 180.0,
) -> list:
    """Pronation flip with strictly zero translational acceleration.

    The board starts palm-up (gravity along +Z) and rotates about the forearm
    axis (Y).  The accelerometer therefore reads gravity and nothing else, so a
    perfect estimator must report a_up == 0 for the entire trace.
    """
    total = math.radians(total_deg)
    samples = []
    n_still = int(still_s / dt_s)
    for _ in range(n_still):
        samples.append(((0.0, 0.0, G), (0.0, 0.0, 0.0)))
    n_flip = int(flip_s / dt_s)
    for i in range(n_flip):
        t = (i + 1) * dt_s
        theta, omega = _smooth_angle(t, flip_s, total)
        accel = (G * math.sin(theta), 0.0, G * math.cos(theta))
        # omega_y = -dtheta/dt for this parameterisation (see module docstring)
        samples.append((accel, (0.0, -omega * DEG, 0.0)))
    for _ in range(n_still):
        samples.append(((0.0, 0.0, -G), (0.0, 0.0, 0.0)))
    return samples


def synth_pure_lift(
    lift_s: float,
    dt_s: float,
    height_m: float = 0.35,
    still_s: float = 1.0,
) -> list:
    """Vertical raise with no rotation: the motion the gate is meant to catch."""
    samples = []
    n_still = int(still_s / dt_s)
    for _ in range(n_still):
        samples.append(((0.0, 0.0, G), (0.0, 0.0, 0.0)))
    n = int(lift_s / dt_s)
    # Minimum-jerk displacement -> acceleration profile.
    for i in range(n):
        tau = (i + 0.5) * dt_s / lift_s
        acc = height_m / (lift_s * lift_s) * (
            60.0 * tau - 180.0 * tau * tau + 120.0 * tau ** 3
        )
        samples.append(((0.0, 0.0, G + acc), (0.0, 0.0, 0.0)))
    for _ in range(n_still):
        samples.append(((0.0, 0.0, G), (0.0, 0.0, 0.0)))
    return samples


def evaluate(samples: list, estimator, dt_s: float) -> dict:
    a_ups = [estimator.update(a, g, dt_s) for a, g in samples]
    out = run_lift_state_machine(a_ups, dt_s)
    out["raw_pos_integral"] = sum(max(v, 0.0) for v in a_ups) * dt_s
    return out


def _report(title: str, rows: list) -> None:
    print(f"\n{title}")
    print(
        "  %-18s %-16s %8s %8s %8s %6s"
        % ("case", "estimator", "peak", "latch", "total", "gate")
    )
    for label, est_name, res in rows:
        latch = "-" if res["imp_at_latch"] is None else "%.3f" % res["imp_at_latch"]
        print(
            "  %-18s %-16s %8.3f %8s %8.3f %6s"
            % (
                label,
                est_name,
                res["peak_a_up"],
                latch,
                res["pos_impulse_total"],
                "PASS" if res["match_lift_ok"] else "fail",
            )
        )


def cmd_synthetic_rotation(args) -> int:
    dt_s = args.interval_ms / 1000.0
    print(
        "Pure pronation flip, zero translation. A correct estimator must report\n"
        "a_up == 0 throughout, so any impulse below is rotation leaking into the\n"
        "lift measurement. Final match gate needs total >= %.2f m/s (main.c:%d)."
        % (MATCH_POS_IMPULSE_MIN_MS, 272)
    )

    rows = []
    contaminated = []
    for flip_s in args.flip_durations:
        peak_dps = math.radians(180.0) * math.pi / (2.0 * flip_s) * DEG
        samples = synth_pure_rotation(flip_s, dt_s)
        label = "flip %.1fs (%.0fdps)" % (flip_s, peak_dps)
        for est in (LpfGravity(), GyroGravity()):
            res = evaluate(samples, est, dt_s)
            rows.append((label, est.name, res))
            if isinstance(est, LpfGravity):
                contaminated.append((flip_s, res))
    _report("=== pure rotation (no translation) ===", rows)

    rows = []
    for lift_s in args.lift_durations:
        samples = synth_pure_lift(lift_s, dt_s)
        label = "lift %.1fs 35cm" % lift_s
        for est in (LpfGravity(), GyroGravity()):
            res = evaluate(samples, est, dt_s)
            rows.append((label, est.name, res))
    _report("=== pure vertical lift, 35 cm (no rotation) ===", rows)

    rows = []
    samples = synth_pure_rotation(0.6, dt_s)
    for bias_dps, noise_dps in args.gyro_error:
        dirty = perturb(samples, bias_dps, noise_dps)
        res = evaluate(dirty, GyroGravity(), dt_s)
        rows.append(
            ("bias %.0f noise %.0f" % (bias_dps, noise_dps), "gyro-propagated", res)
        )
    _report(
        "=== gyro-propagated under real sensor error (0.6 s flip) ===", rows
    )

    print()
    passing = [f for f, r in contaminated if r["match_lift_ok"]]
    if passing:
        print(
            "RESULT: rotation alone clears the 0.65 m/s match gate at flip "
            "durations %s." % ", ".join("%.1fs" % f for f in passing)
        )
        print(
            "        The gate does not measure the arm lift. This reproduces the\n"
            "        corr(total, roll_at_lift)=0.726 seen across 99 field entries."
        )
        return 0
    print(
        "RESULT: rotation alone did NOT clear the gate in this simulation.\n"
        "        The 0.726 field correlation has another cause - revisit the\n"
        "        hypothesis before building the offline feature evaluation."
    )
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser(
        "synthetic-rotation",
        help="prove or refute rotation contamination without hardware",
    )
    p.add_argument("--interval-ms", type=float, default=SAMPLE_INTERVAL_MS)
    p.add_argument(
        "--flip-durations",
        type=float,
        nargs="+",
        default=[0.4, 0.6, 0.8, 1.0, 1.5],
    )
    p.add_argument(
        "--lift-durations", type=float, nargs="+", default=[0.4, 0.8, 1.5]
    )
    p.add_argument(
        "--gyro-error",
        type=float,
        nargs=2,
        action="append",
        metavar=("BIAS_DPS", "NOISE_DPS"),
        default=None,
        help="repeatable: constant gyro bias and white noise sigma, in dps",
    )
    p.set_defaults(func=cmd_synthetic_rotation)

    args = parser.parse_args()
    if getattr(args, "gyro_error", None) is None:
        args.gyro_error = [(0.0, 0.0), (1.0, 2.0), (3.0, 5.0), (10.0, 10.0)]
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
