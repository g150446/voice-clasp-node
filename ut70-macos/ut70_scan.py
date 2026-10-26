#!/usr/bin/env python3
"""List HID devices and identify an attached ALIENTEK UT70."""

from __future__ import annotations

import argparse

import hid

from ut70_common import PID, VID


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--all", action="store_true", help="show every HID device")
    args = parser.parse_args()

    devices = hid.enumerate() if args.all else hid.enumerate(VID, PID)
    if not devices:
        print(f"UT70 not found (expected VID:PID {VID:04X}:{PID:04X})")
        return 1

    for item in devices:
        marker = "  <-- UT70" if (
            item.get("vendor_id") == VID and item.get("product_id") == PID
        ) else ""
        print(
            f"{item.get('vendor_id', 0):04X}:{item.get('product_id', 0):04X} "
            f"usage={item.get('usage_page', 0):04X}:{item.get('usage', 0):04X} "
            f"product={item.get('product_string')!r} "
            f"serial={item.get('serial_number')!r}{marker}"
        )
        print(f"  path={item.get('path')!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
