"""Shared, read-only helpers for the ALIENTEK UT70 USB HID interface."""

from __future__ import annotations

import dataclasses
import struct
import time
from typing import Iterator

import hid

VID = 0x19F5
PID = 0x6666
REPORT_LEN = 64
REPORT_HEADER = 0xEE


class UT70Error(RuntimeError):
    pass


@dataclasses.dataclass(frozen=True)
class Measurement:
    voltage_v: float
    current_a: float
    checksum_ok: bool
    raw: bytes

    @property
    def current_ma(self) -> float:
        return self.current_a * 1000.0

    @property
    def power_w(self) -> float:
        # No independent power field has been proven yet.
        return self.voltage_v * self.current_a


def find_devices(vid: int = VID, pid: int = PID) -> list[dict]:
    return list(hid.enumerate(vid, pid))


def open_device(path: bytes | None = None):
    devices = find_devices()
    if not devices:
        raise UT70Error(f"UT70 not found (VID:PID {VID:04X}:{PID:04X})")
    dev = hid.device()
    try:
        dev.open_path(path or devices[0]["path"])
        dev.set_nonblocking(False)
    except Exception:
        dev.close()
        raise
    return dev


def checksum(report: bytes) -> int:
    return sum(report[:-1]) & 0xFF


def decode_report(report: bytes) -> Measurement:
    if len(report) != REPORT_LEN:
        raise UT70Error(f"expected {REPORT_LEN} bytes, got {len(report)}")
    if report[0] != REPORT_HEADER:
        raise UT70Error(f"unexpected header 0x{report[0]:02X}")

    # These two offsets were established from captures on a real UT70. Other
    # bytes deliberately remain unnamed until independently verified.
    voltage_v = struct.unpack_from("<f", report, 3)[0]
    current_a = struct.unpack_from("<f", report, 23)[0]
    return Measurement(voltage_v, current_a, checksum(report) == report[-1], report)


def reports(dev, timeout_ms: int = 2000) -> Iterator[tuple[float, bytes]]:
    while True:
        data = bytes(dev.read(REPORT_LEN, timeout_ms))
        if data:
            yield time.time(), data
