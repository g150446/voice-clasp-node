#!/usr/bin/env python3
"""One user-approved tap trial, or an action-free software-tap preflight.

Run in macOS Terminal. The agent must obtain readiness before --kind
single/double. Each invocation runs ONE trial. Tap notifications are broadcast to all hosts.
"""
import argparse
from device_debug import DebugControl
import asyncio
import subprocess
import time

from bleak import BleakClient, BleakScanner
from node_names import find_node

TX = "00000002-0000-1000-8000-00805f9b34fb"
RX = "00000003-0000-1000-8000-00805f9b34fb"


async def run(kind):
    events = []
    registers = {}
    diagnostics = []
    stats_queue = asyncio.Queue()
    phase = "setup"

    def notify(_, data):
        if len(data) < 3 or data[:2] != b"\x00\x55":
            return
        code = data[2]
        if code in (0x12, 0x14):
            events.append((phase, code, time.monotonic()))
            print(f"{phase}: {'DOUBLE' if code == 0x12 else 'SINGLE'} raw={data.hex()}", flush=True)
        elif code == 0xD2 and len(data) >= 6:
            ret = int.from_bytes(data[5:6], "little", signed=True)
            registers[data[3]] = (data[4], ret)
            print(f"reg 0x{data[3]:02x}=0x{data[4]:02x} ret={ret}", flush=True)
        elif code == 0xD0:
            diagnostics.append(bytes(data))
            print(f"tap_diag={data.hex()}", flush=True)
        elif code == 0xD5 and len(data) == 43:
            import struct
            values = struct.unpack_from("<6I4f", data, 3)
            stats_queue.put_nowait(values)

    print("Scanning for the node (ClaspNode / HarnessNode)...", flush=True)
    device = await find_node(None, timeout=15)
    if not device:
        raise RuntimeError("node (ClaspNode / HarnessNode) not found")
    print(f"Found {device.name} [{device.address}]", flush=True)
    async with BleakClient(device) as client:
        debug = DebugControl()
        await client.start_notify(TX, debug.wrap(notify))
        async with debug.session(client):
            print("Connected; subscribed to tap notifications.", flush=True)
            await asyncio.sleep(1)
            # Reads only. FIFO status is not consumed by these register reads.
            expected = {0x10: 0x68, 0x15: 0x00, 0x58: 0x00, 0x5E: 0x00,
                        0x0D: 0x00, 0x08: 0x01, 0x0A: 0x36}
            for reg in expected:
                await client.write_gatt_char(RX, bytes([0x51, reg]), response=True)
                await asyncio.sleep(0.15)
            await asyncio.sleep(0.5)
            failed = {hex(r): (registers.get(r), v) for r, v in expected.items()
                      if registers.get(r) != (v, 0)}
            if failed:
                raise RuntimeError(f"Software tap configuration mismatch: {failed}")
            print("PASS: 416 Hz / +/-4g, accel-only FIFO, embedded tap routing disabled.", flush=True)
            if kind == "preflight":
                print("Preflight complete. No physical trial was started.", flush=True)
                return 0

            async def read_stats(clear=False):
                await client.write_gatt_char(RX, bytes([0x52, int(clear)]), response=True)
                try:
                    values = await asyncio.wait_for(stats_queue.get(), timeout=2)
                except asyncio.TimeoutError:
                    print("Tap statistics unavailable (requires FW 0.0.109+).", flush=True)
                    return
                print("Tap stats: samples=%d gaps=%d candidates=%d accepted=%d broad=%d lateral=%d "
                      "peak_g=(%.3f,%.3f,%.3f) max_z_edge_g=%.3f" % values, flush=True)

            phase = "countdown"
            print(f"Trial: {kind}. After GO: {'tap once' if kind == 'single' else 'tap twice, about 0.2 seconds apart'}.", flush=True)
            for remaining in range(5, 0, -1):
                print(remaining, flush=True)
                await asyncio.sleep(1)
            await read_stats(clear=True)
            phase = "trial"
            print("GO", flush=True)
            # Fire the cue without blocking notification processing.
            sound = await asyncio.create_subprocess_exec(
                "afplay", "/System/Library/Sounds/Glass.aiff",
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            await asyncio.sleep(5)
            await sound.wait()
            phase = "finished"
            if not client.is_connected:
                raise RuntimeError("BLE disconnected during trial; result is inconclusive")
            await read_stats()
            observed = [c for p, c, _ in events if p == "trial"]
            expected_code = 0x14 if kind == "single" else 0x12
            ok = observed == [expected_code] and not any(p == "countdown" for p, _, _ in events)
            print(f"{'PASS' if ok else 'FAIL'}: expected={[hex(expected_code)]} observed={[hex(c) for c in observed]}", flush=True)
            print("Trial finished. Wait for the agent before another trial.", flush=True)
            return 0 if ok else 1



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--kind", choices=["preflight", "single", "double"], required=True)
    args = parser.parse_args()
    raise SystemExit(asyncio.run(run(args.kind)))
