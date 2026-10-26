#!/usr/bin/env python3
"""Capture pdm_power_test serial output with wall-clock timestamps.

The UT70 shows a waveform on its own screen with no PC link, so the only way to
say which plateau is which state is to line the waveform's time axis up against
these timestamps. Every state marker is stamped with local time.

usage: capture.py [seconds]     (default: run until Ctrl-C)
"""
import glob
import sys
import time

import serial

PORT = sorted(glob.glob("/dev/cu.usbmodem*"))[0]
OUT = "measure.log"
duration = float(sys.argv[1]) if len(sys.argv) > 1 else None

ser = serial.Serial(PORT, 115200, timeout=1)
end = time.time() + duration if duration else None
started = time.time()

print(f"capturing {PORT} -> {OUT}   (Ctrl-C to stop)")
with open(OUT, "a") as f:
    f.write(f"\n===== capture started {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
    f.flush()
    buf = b""
    try:
        while end is None or time.time() < end:
            buf += ser.read(4096)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").rstrip("\r")
                if not line:
                    continue
                stamp = time.strftime("%H:%M:%S")
                elapsed = time.time() - started
                out = f"[{stamp} +{elapsed:7.1f}s] {line}"
                f.write(out + "\n")
                f.flush()
                # Echo only the state markers so the console stays readable.
                if ">>> STATE" in line or ">>> CYCLE" in line or "!!!" in line:
                    print(out, flush=True)
    except KeyboardInterrupt:
        pass
ser.close()
print("stopped")
