#!/usr/bin/env python3
"""Print and optionally log untouched UT70 HID Input Reports.

This program never calls write() or send_feature_report().
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import pathlib
import time

from ut70_common import open_device, reports


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=0, help="stop after N reports")
    parser.add_argument("--duration", type=float, default=0, help="stop after N seconds")
    parser.add_argument("--timeout-ms", type=int, default=2000)
    parser.add_argument("--csv", type=pathlib.Path)
    parser.add_argument("--quiet", action="store_true", help="write CSV without console dumps")
    args = parser.parse_args()

    output = None
    writer = None
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        output = args.csv.open("w", newline="", encoding="utf-8")
        writer = csv.writer(output)
        writer.writerow(["timestamp", "length", "hex", "decimal"])

    started = time.monotonic()
    seen = 0
    try:
        dev = open_device()
        try:
            for timestamp, report in reports(dev, args.timeout_ms):
                stamp = dt.datetime.fromtimestamp(timestamp).astimezone().isoformat(timespec="milliseconds")
                hex_dump = report.hex(" ")
                decimal = " ".join(str(value) for value in report)
                if not args.quiet:
                    print(f"{stamp}  len={len(report)}")
                    print(hex_dump)
                    print(decimal)
                if writer:
                    writer.writerow([stamp, len(report), hex_dump, decimal])
                    output.flush()
                seen += 1
                if args.count and seen >= args.count:
                    break
                if args.duration and time.monotonic() - started >= args.duration:
                    break
        finally:
            dev.close()
    except KeyboardInterrupt:
        pass
    finally:
        if output:
            output.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
