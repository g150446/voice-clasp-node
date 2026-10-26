#!/usr/bin/env python3
"""BLE OTA firmware updater using MCUmgr SMP over BLE.

Usage:
    venv/bin/pip install bleak cbor2
    venv/bin/python3 ota_updater.py --device HarnessNode ../nordic-main/ota_update.bin
"""

import asyncio
import hashlib
import struct
import sys
import time

import cbor2
from bleak import BleakClient, BleakScanner
from node_names import NODE_NAME, find_node

DEVICE_NAME = NODE_NAME   # default (either brand name is accepted); override with --device flag

# SMP BLE service / characteristic UUIDs
SMP_SERVICE_UUID   = "8D53DC1D-1DB7-4CD3-868B-8A527460AA84"
SMP_CHAR_UUID      = "DA2E7828-FBCE-4E01-AE9E-261174997C48"

# SMP header layout: op(1) flags(1) len(2BE) group(2BE) seq(1) id(1)
SMP_HEADER_SIZE = 8

# SMP ops
OP_READ  = 0
OP_WRITE = 2

# SMP groups
GRP_OS  = 0
GRP_IMG = 1

# SMP IDs
IMG_UPLOAD = 1
IMG_STATE  = 0
OS_RESET   = 5


def make_smp_frame(op: int, group: int, seq: int, cmd_id: int, payload: bytes) -> bytes:
    header = struct.pack(">BBHHBB", op, 0, len(payload), group, seq, cmd_id)
    return header + payload


class SMPClient:
    def __init__(self, client: BleakClient):
        self._client = client
        self._seq = 0
        self._response: asyncio.Future | None = None
        self._buf: bytes = b""
        self._loop = asyncio.get_event_loop()

    def _on_notify(self, _handle, data: bytearray) -> None:
        if not self._response or self._response.done():
            return
        self._buf += bytes(data)
        # Parse SMP header: len field (bytes 2:4 big-endian) = CBOR payload length
        if len(self._buf) >= SMP_HEADER_SIZE:
            cbor_len = struct.unpack(">H", self._buf[2:4])[0]
            total_expected = SMP_HEADER_SIZE + cbor_len
            if len(self._buf) >= total_expected:
                result = self._buf[:total_expected]
                fut = self._response

                def _set(res=result, f=fut):
                    if f is not None and not f.done():
                        f.set_result(res)

                # thread-safe set_result (bleak on macOS uses a separate CB thread)
                try:
                    self._loop.call_soon_threadsafe(_set)
                except Exception:
                    pass

    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()   # capture the actual running loop
        await self._client.start_notify(SMP_CHAR_UUID, self._on_notify)

    async def send(self, op: int, group: int, cmd_id: int, payload_dict: dict) -> dict:
        payload = cbor2.dumps(payload_dict)
        frame = make_smp_frame(op, group, self._seq, cmd_id, payload)
        self._seq = (self._seq + 1) & 0xFF

        self._buf = b""
        self._response = self._loop.create_future()
        await self._client.write_gatt_char(SMP_CHAR_UUID, frame, response=False)

        raw = await asyncio.wait_for(self._response, timeout=30.0)
        # strip 8-byte SMP response header
        return cbor2.loads(raw[SMP_HEADER_SIZE:])


async def find_device(name: str, timeout: float = 15.0):
    print(f"Scanning for '{name}'...", flush=True)
    device = await find_node(name, timeout=timeout)
    if device is None:
        raise RuntimeError(f"Device '{name}' not found. Is it powered on and advertising?")
    print(f"Found: {device.address}", flush=True)
    return device


async def query_image_state(smp: SMPClient) -> dict:
    state = await smp.send(OP_READ, GRP_IMG, IMG_STATE, {})
    images = state.get("images", [])
    printable = [
        {key: value.hex() if isinstance(value, bytes) else value
         for key, value in image.items()}
        for image in images
    ]
    print(f"Images: {printable}", flush=True)
    return state


async def verify_update(expected_hash: bytes, timeout: float) -> None:
    """Reconnect after reset and require the uploaded image to be active and confirmed."""
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None

    print("Waiting for the updated device to return...", flush=True)
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            device = await find_device(DEVICE_NAME, timeout=min(8.0, remaining))
            async with BleakClient(device) as client:
                smp = SMPClient(client)
                await smp.start()
                await asyncio.sleep(0.5)

                while time.monotonic() < deadline:
                    state = await query_image_state(smp)
                    matching = [
                        image for image in state.get("images", [])
                        if bytes(image.get("hash", b"")) == expected_hash
                    ]
                    if matching:
                        image = matching[0]
                        if (image.get("slot") == 0
                                and image.get("active") is True
                                and image.get("confirmed") is True):
                            print(
                                "OTA verified: uploaded image is active and confirmed "
                                f"in slot 0 (version={image.get('version', 'unknown')}).",
                                flush=True,
                            )
                            return
                        print(
                            "Updated image is present but not confirmed yet; retrying...",
                            flush=True,
                        )
                    else:
                        print("Updated image hash is not visible yet; retrying...", flush=True)
                    await asyncio.sleep(1.0)
        except Exception as exc:
            last_error = exc
            await asyncio.sleep(1.0)

    detail = f": {last_error}" if last_error else ""
    raise RuntimeError(
        f"OTA verification timed out after {timeout:.0f}s; "
        f"the uploaded image did not become active and confirmed{detail}"
    )


async def ota_update(bin_path: str, verify_timeout: float = 60.0) -> None:
    # Load firmware image
    with open(bin_path, "rb") as f:
        image_data = f.read()
    total = len(image_data)
    sha256 = hashlib.sha256(image_data).digest()
    print(f"Image: {bin_path} ({total} bytes)", flush=True)

    device = await find_device(DEVICE_NAME)

    async with BleakClient(device) as client:
        print(f"Connected. MTU={client.mtu_size}", flush=True)
        smp = SMPClient(client)
        await smp.start()
        await asyncio.sleep(0.5)  # wait for CCCD write to be acknowledged

        # ATT write-without-response payload = MTU - 3 (ATT header)
        # SMP frame header = 8 bytes
        # Available for CBOR = MTU - 3 - 8 = MTU - 11
        # First packet extra CBOR overhead (image, len, sha): ~70 bytes
        # Subsequent packets overhead (data key + off key): ~20 bytes
        att_payload = client.mtu_size - 3
        chunk_size_first = max(32, att_payload - SMP_HEADER_SIZE - 70)
        chunk_size_rest  = max(32, att_payload - SMP_HEADER_SIZE - 20)
        print(f"Chunk size: first={chunk_size_first}, rest={chunk_size_rest} bytes", flush=True)

        # Upload loop
        offset = 0
        start_time = time.monotonic()
        while offset < total:
            cs = chunk_size_first if offset == 0 else chunk_size_rest
            chunk = image_data[offset: offset + cs]

            if offset == 0:
                payload = {
                    "image": 0,
                    "data":  chunk,
                    "off":   0,
                    "len":   total,
                    "sha":   sha256,
                }
            else:
                payload = {
                    "data": chunk,
                    "off":  offset,
                }

            resp = await smp.send(OP_WRITE, GRP_IMG, IMG_UPLOAD, payload)
            rc = resp.get("rc", 0)  # rc absent = 0 = success in SMP image upload
            if rc != 0:
                raise RuntimeError(f"Upload error at offset {offset}: rc={rc} resp={resp}")

            # Server echoes the next expected offset; use it if available
            server_off = resp.get("off", offset + len(chunk))
            offset = server_off
            pct = offset * 100 // total
            elapsed = time.monotonic() - start_time
            rate_kbs = (offset / 1024) / max(elapsed, 0.001)
            print(f"\r  {offset}/{total} bytes  {pct}%  {rate_kbs:.1f} KB/s   ", end="", flush=True)

        print(f"\nUpload complete in {time.monotonic() - start_time:.1f}s", flush=True)

        # Query image list to get the hash MCUboot recorded for the uploaded image
        print("Querying image state...", flush=True)
        await asyncio.sleep(0.5)
        state_resp = await query_image_state(smp)
        images = state_resp.get("images", [])
        # Find slot 1 (secondary) image hash
        slot1_hash = None
        for img in images:
            if img.get("slot", -1) == 1:
                slot1_hash = bytes(img["hash"])
                break
        if slot1_hash is None:
            # Retry once after a short settle; SMP notify races can drop a reply.
            await asyncio.sleep(1.0)
            state_resp = await query_image_state(smp)
            images = state_resp.get("images", [])
            for img in images:
                if img.get("slot", -1) == 1:
                    slot1_hash = bytes(img["hash"])
                    break
        if slot1_hash is None:
            raise RuntimeError(f"No image in slot 1 found. State: {state_resp}")
        print(f"Slot 1 image hash: {slot1_hash.hex()}", flush=True)

        # Mark image for test (swap on next boot)
        print("Setting image test flag...", flush=True)
        resp = await smp.send(OP_WRITE, GRP_IMG, IMG_STATE, {"hash": slot1_hash, "confirm": False})
        rc = resp.get("rc", 0)
        if rc != 0:
            raise RuntimeError(f"Image test set failed: rc={rc} resp={resp}")
        print("Image test flag set.", flush=True)

        # Reset device
        print("Sending reset command...", flush=True)
        try:
            await smp.send(OP_WRITE, GRP_OS, OS_RESET, {})
        except asyncio.TimeoutError:
            pass  # device reboots immediately, disconnect is expected

        print("Device reset. MCUboot will swap slots on next boot.", flush=True)

    await verify_update(slot1_hash, verify_timeout)


def main() -> None:
    import argparse
    global DEVICE_NAME
    parser = argparse.ArgumentParser(description="BLE OTA firmware updater (SMP)")
    parser.add_argument("bin", help="Path to ota_update.bin")
    parser.add_argument("--device", default=DEVICE_NAME,
                        help=f"BLE device name to target (default: {DEVICE_NAME})")
    parser.add_argument(
        "--verify-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for the new image to become active and confirmed (default: 60)",
    )
    args = parser.parse_args()
    DEVICE_NAME = args.device
    asyncio.run(ota_update(args.bin, args.verify_timeout))


if __name__ == "__main__":
    main()
