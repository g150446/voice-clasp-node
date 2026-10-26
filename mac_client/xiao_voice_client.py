#!/usr/bin/env python3
"""
Harness Node - Mac Client for XIAO nRF52840 / nRF54L15 Sense

Connects to XIAO nRF52840 Sense or XIAO nRF54L15 Sense running
nordic-main / nrf54-handy firmware.
Receives audio and IMU event notifications via BLE.

Recording start/stop: controlled by gesture events from the device.
  0x01 recording_start → WAV recording begins automatically
  0x02 recording_stop  → WAV recording ends and file is saved
  0x10 motion_active   → displayed (z-value shown)
  0x11 motion_settled  → displayed (z-value shown)
  0x12 double_tap      → displayed (recording state unchanged)
  0x14 single_tap      → displayed (recording state unchanged)
  0x20 sleep_enter     → displayed
  0x21 sleep_wake      → displayed
"""

import asyncio
import logging
import os
import struct
import sys
import wave
import time
from typing import Optional
from datetime import datetime

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Please install bleak: pip install bleak")
    sys.exit(1)


# ============================================================================
# Configuration
# ============================================================================

PRIMARY_DEVICE_NAME = "ClaspNode"
LEGACY_DEVICE_NAMES = ("HarnessNode", "XIAOVoice")
DEVICE_NAMES = (PRIMARY_DEVICE_NAME, *LEGACY_DEVICE_NAMES)

# Audio Service UUIDs
AUDIO_TX_UUID = "00000002-0000-1000-8000-00805f9b34fb"  # Notify
AUDIO_RX_UUID = "00000003-0000-1000-8000-00805f9b34fb"  # Write

# Motion Service UUIDs
MOTION_TX_UUID      = "00000011-0000-1000-8000-00805f9b34fb"  # Notify
WAKEUP_TX_UUID      = "00000013-0000-1000-8000-00805f9b34fb"  # Notify
TILT_TX_UUID        = "00000014-0000-1000-8000-00805f9b34fb"  # Notify
POWER_STATE_TX_UUID = "00000015-0000-1000-8000-00805f9b34fb"  # Notify

BLE_MTU_SIZE = 512

# Audio config (must match firmware)
SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2   # 16-bit
CHANNELS = 1

PACKET_HEADER_SIZE = 2
PACKET_SYNC_BYTE = 0xAA
AUDIO_SILENCE_TIMEOUT = 1.5  # seconds


# ============================================================================
# Logging
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================================
# WAV Recorder
# ============================================================================

class WAVRecorder:
    def __init__(self, sample_rate=SAMPLE_RATE, sample_width=SAMPLE_WIDTH, channels=CHANNELS):
        self.sample_rate = sample_rate
        self.sample_width = sample_width
        self.channels = channels
        self.wav_file: Optional[wave.Wave_write] = None
        self.is_recording = False
        self.start_time: Optional[float] = None
        self.bytes_recorded = 0
        self.last_duration = 0.0
        self._filename: Optional[str] = None

    def start_recording(self, filename: Optional[str] = None) -> bool:
        if self.is_recording:
            logger.warning("Already recording")
            return False

        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, 'output')
        os.makedirs(output_dir, exist_ok=True)

        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(output_dir, f"xiao_recording_{ts}.wav")
        elif not os.path.isabs(filename):
            filename = os.path.join(output_dir, filename)

        try:
            self.wav_file = wave.open(filename, 'wb')
            self.wav_file.setnchannels(self.channels)
            self.wav_file.setsampwidth(self.sample_width)
            self.wav_file.setframerate(self.sample_rate)
            self.is_recording = True
            self.start_time = time.time()
            self.bytes_recorded = 0
            self.last_duration = 0.0
            self._filename = filename
            print(f"  [WAV] Recording: {os.path.basename(filename)}")
            return True
        except Exception as e:
            logger.error(f"Failed to start recording: {e}")
            return False

    def stop_recording(self):
        if not self.is_recording:
            return
        self.last_duration = time.time() - (self.start_time or time.time())
        if self.wav_file:
            self.wav_file.close()
            self.wav_file = None
        self.is_recording = False
        print(f"  [WAV] Saved {self.last_duration:.1f}s ({self.bytes_recorded} bytes) → {os.path.basename(self._filename or '')}")

    def write_samples(self, pcm_data: bytes):
        if not self.is_recording or not self.wav_file:
            return
        try:
            self.wav_file.writeframes(pcm_data)
            self.bytes_recorded += len(pcm_data)
        except Exception as e:
            logger.error(f"Error writing samples: {e}")

    def get_duration(self) -> float:
        if self.is_recording and self.start_time:
            return time.time() - self.start_time
        return self.last_duration


# ============================================================================
# BLE Client
# ============================================================================

class XiaoBleClient:
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.recorder = WAVRecorder()
        self.is_connected = False
        self._last_seq = -1
        self._packets_received = 0
        self._packet_loss = 0

        # Disconnect tracking
        self._connect_time: Optional[float] = None
        self._disconnect_time: Optional[float] = None
        self._address: Optional[str] = None

        # Device-initiated recording detection
        self._last_audio_time: float = 0.0
        self._audio_silence_task = None

    async def find_device(self) -> Optional[str]:
        accepted_names = ", ".join(repr(name) for name in DEVICE_NAMES)
        print(f"Scanning for {accepted_names} (8s)...")
        audio_svc_uuid = "00000001-0000-1000-8000-00805f9b34fb"
        candidates: list[tuple[str, str]] = []  # (address, label)
        seen: set[str] = set()

        def on_detection(device, adv):
            if device.address in seen:
                return
            local_name = adv.local_name or device.name or ""
            uuids = [str(u).lower() for u in adv.service_uuids]
            if local_name in DEVICE_NAMES or audio_svc_uuid in uuids:
                seen.add(device.address)
                label = f"{device.address}  (name={local_name!r})"
                candidates.append((device.address, label))

        async with BleakScanner(detection_callback=on_detection):
            await asyncio.sleep(8.0)

        if not candidates:
            print("  Device not found.")
            return None

        if len(candidates) == 1:
            addr, label = candidates[0]
            print(f"  Found: {label}")
            return addr

        # Multiple devices — ask the user to choose
        print(f"\n  Found {len(candidates)} compatible devices:")
        for i, (addr, label) in enumerate(candidates):
            print(f"    [{i + 1}] {label}")
        loop = asyncio.get_event_loop()
        while True:
            try:
                raw = await loop.run_in_executor(
                    None, input, f"  Select device [1-{len(candidates)}]: "
                )
                idx = int(raw.strip()) - 1
                if 0 <= idx < len(candidates):
                    return candidates[idx][0]
            except (ValueError, EOFError):
                pass
            print(f"  Please enter a number between 1 and {len(candidates)}.")

    def _on_disconnected(self, client: "BleakClient"):
        elapsed = time.time() - (self._connect_time or time.time())
        self._disconnect_time = time.time()
        self.is_connected = False
        print(f"\n  [DISCONNECT] After {elapsed:.0f}s connected")
        if self.recorder.is_recording:
            self.recorder.stop_recording()

    async def connect(self, address: str):
        print(f"Connecting to {address}...")
        self._address = address
        self.client = BleakClient(
            address,
            mtu_size=BLE_MTU_SIZE,
            timeout=10.0,
            disconnected_callback=self._on_disconnected,
        )
        await self.client.connect()
        self._connect_time = time.time()
        self.is_connected = True
        print(f"  Connected (MTU={self.client.mtu_size})")

        await self.client.start_notify(AUDIO_TX_UUID, self._on_audio)
        print("  Notifications enabled (audio)")
        self._audio_silence_task = asyncio.create_task(self._audio_silence_monitor())
        await asyncio.sleep(0.5)

    async def disconnect(self):
        if self.client:
            for uuid in [AUDIO_TX_UUID, MOTION_TX_UUID, WAKEUP_TX_UUID,
                         TILT_TX_UUID, POWER_STATE_TX_UUID]:
                try:
                    await self.client.stop_notify(uuid)
                except Exception:
                    pass
            await self.client.disconnect()
            self.is_connected = False
            print("Disconnected.")

    def _on_audio(self, sender, data: bytes):
        if len(data) < 2:
            return
        # イベントパケット: [0x00][0x55][event_code][optional f32 z]
        if data[0] == 0x00 and len(data) >= 3 and data[1] == 0x55:
            code = data[2]
            if code == 0x01:
                print("  [EVT] recording_start → auto-start WAV")
                self.recorder.start_recording()
            elif code == 0x02:
                print("  [EVT] recording_stop → save WAV")
                self.recorder.stop_recording()
                self._last_audio_time = 0.0
            elif code == 0x10:
                if len(data) >= 15:
                    x, y, z = struct.unpack_from('<fff', data, 3)
                    print(f"  [EVT] motion_active  x={x:+.2f} y={y:+.2f} z={z:+.2f}")
                elif len(data) >= 7:
                    z = struct.unpack_from('<f', data, 3)[0]
                    print(f"  [EVT] motion_active  z={z:+.2f}")
                else:
                    print("  [EVT] motion_active")
            elif code == 0x11:
                if len(data) >= 31:
                    x, y, z = struct.unpack_from('<fff', data, 3)
                    elapsed_ms = struct.unpack_from('<I', data, 15)[0]
                    avg_speed  = struct.unpack_from('<f', data, 19)[0]
                    peak_speed = struct.unpack_from('<f', data, 23)[0]
                    distance   = struct.unpack_from('<f', data, 27)[0]
                    print(f"  [EVT] motion_settled x={x:+.2f} y={y:+.2f} z={z:+.2f} "
                          f"elapsed={elapsed_ms}ms avg={avg_speed:.3f}m/s "
                          f"peak={peak_speed:.3f}m/s dist={distance:.3f}m")
                elif len(data) >= 23:
                    z = struct.unpack_from('<f', data, 3)[0]
                    elapsed_ms = struct.unpack_from('<I', data, 7)[0]
                    avg_speed  = struct.unpack_from('<f', data, 11)[0]
                    peak_speed = struct.unpack_from('<f', data, 15)[0]
                    distance   = struct.unpack_from('<f', data, 19)[0]
                    print(f"  [EVT] motion_settled z={z:+.2f} elapsed={elapsed_ms}ms "
                          f"avg={avg_speed:.3f}m/s peak={peak_speed:.3f}m/s dist={distance:.3f}m")
                else:
                    z = struct.unpack_from('<f', data, 3)[0] if len(data) >= 7 else float('nan')
                    print(f"  [EVT] motion_settled z={z:+.2f}")
            elif code == 0x12:
                print("  [EVT] double_tap")
            elif code == 0x14:
                print("  [EVT] single_tap")
            elif code == 0x20:
                print("  [EVT] sleep_enter")
            elif code == 0x21:
                print("  [EVT] sleep_wake")
            else:
                print(f"  [EVT] 0x{code:02x}")
            return
        if len(data) < PACKET_HEADER_SIZE:
            return
        seq = data[0]
        if data[1] != PACKET_SYNC_BYTE:
            return
        if self._last_seq >= 0:
            expected = (self._last_seq + 1) % 256
            if seq != expected:
                self._packet_loss += 1
        self._last_seq = seq
        self._packets_received += 1
        self._last_audio_time = time.time()

        # デバイス主導で録音が始まった場合、自動的にWAV録音を開始
        if not self.recorder.is_recording:
            print("  [REC] Device-initiated recording detected → auto-start WAV")
            self.recorder.start_recording()

        self.recorder.write_samples(data[PACKET_HEADER_SIZE:])

    def _on_motion(self, sender, data: bytes):
        if len(data) < 14:
            return
        event_type = data[0]
        count = data[1]
        activity = struct.unpack_from('<f', data, 2)[0]
        peak = struct.unpack_from('<f', data, 6)[0]
        elapsed_ms = struct.unpack_from('<I', data, 10)[0]
        z_excursion = struct.unpack_from('<f', data, 14)[0] if len(data) >= 18 else 0.0

        if event_type == 0x01:
            elapsed_label = f"since_last_settle={elapsed_ms}ms"
            state = "ACTIVE"
        else:
            elapsed_label = f"motion_duration={elapsed_ms}ms"
            state = "SETTLED"
        print(f"  [MOTION {state}] count={count} activity={activity:.2f} peak={peak:.2f} "
              f"z_exc={z_excursion:.2f} {elapsed_label}")
        if event_type == 0x00 and elapsed_ms <= 450:
            print(f"  [GESTURE] clench-like settle → possible READY state")

    def _on_wakeup(self, sender, data: bytes):
        if len(data) < 2:
            return
        axes = data[0]
        count = data[1]
        axis_str = "".join([
            "Z" if axes & 0x04 else "",
            "Y" if axes & 0x02 else "",
            "X" if axes & 0x01 else "",
        ]) or "?"
        print(f"  [WAKEUP] axes={axis_str} count={count}")

    def _on_tilt(self, sender, data: bytes):
        if len(data) < 2:
            return
        tilt_src = data[0]
        count = data[1]
        elapsed_ms = struct.unpack_from('<I', data, 2)[0] if len(data) >= 6 else 0
        active = "YES" if (tilt_src & 0x20) else "NO"
        print(f"  [TILT] active={active} src=0x{tilt_src:02x} count={count} since_motion_active={elapsed_ms}ms")
        if elapsed_ms <= 1500:
            print(f"  [GESTURE] Tilt within gesture window")

    def _on_power_state(self, sender, data: bytes):
        if len(data) < 1:
            return
        state = data[0]
        if state == 0x01:
            print("  [SLEEP] Device entering low-power mode")
        else:
            print("  [SLEEP] Device waking up from low-power mode")

    async def _audio_silence_monitor(self):
        """録音中にaudioパケットが途絶えたら自動でWAVを閉じる"""
        while True:
            await asyncio.sleep(0.5)
            if self.recorder.is_recording and self._last_audio_time > 0:
                if time.time() - self._last_audio_time > AUDIO_SILENCE_TIMEOUT:
                    print("  [REC] Audio silence detected → auto-stop WAV")
                    self.recorder.stop_recording()
                    self._last_audio_time = 0.0

    async def _send_ble_start(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.write_gatt_char(AUDIO_RX_UUID, bytes([0x01]))
        except Exception as e:
            logger.error(f"BLE start failed: {e}")

    async def _send_ble_stop(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.write_gatt_char(AUDIO_RX_UUID, bytes([0x00]))
        except Exception as e:
            logger.error(f"BLE stop failed: {e}")

    def start_recording(self) -> bool:
        if not self.recorder.start_recording():
            return False
        t = asyncio.create_task(self._send_ble_start())
        t.add_done_callback(lambda _: None)
        return True

    def stop_recording(self):
        """手動停止: WAV閉じ + BLE stop コマンド送信"""
        self.recorder.stop_recording()
        try:
            t = asyncio.create_task(self._send_ble_stop())
            t.add_done_callback(lambda _: None)
        except RuntimeError:
            pass

    def print_stats(self):
        print(f"  Packets received: {self._packets_received}")
        print(f"  Packets lost:     {self._packet_loss}")
        if self._packets_received > 0:
            rate = self._packet_loss / (self._packets_received + self._packet_loss) * 100
            print(f"  Loss rate:        {rate:.1f}%")
        print(f"  Recording:        {'YES ({:.1f}s)'.format(self.recorder.get_duration()) if self.recorder.is_recording else 'NO'}")


# ============================================================================
# Interactive Control
# ============================================================================

async def interactive_control(client: XiaoBleClient):
    print()
    print("=" * 60)
    print("HarnessNode / XIAOVoice - XIAO Recording Control")
    print("=" * 60)
    print("Commands:")
    print("  r  - Start recording")
    print("  s  - Stop recording")
    print("  t  - Show status")
    print("  q  - Quit")
    print()

    loop = asyncio.get_event_loop()

    while True:
        try:
            cmd = await loop.run_in_executor(None, input, "Command: ")
            cmd = cmd.strip().lower()

            if cmd in ('q', 'quit', 'exit'):
                break
            elif cmd == 'r':
                client.start_recording()
            elif cmd == 's':
                client.stop_recording()
            elif cmd == 't':
                client.print_stats()
            elif cmd == '':
                client.print_stats()
            else:
                print(f"  Unknown command: '{cmd}'")

        except EOFError:
            await asyncio.sleep(0.5)
        except Exception as e:
            logger.error(f"Error: {e}")
            await asyncio.sleep(0.5)


# ============================================================================
# Main
# ============================================================================

async def main():
    print("=" * 60)
    print("Harness Node - Mac Client")
    print("=" * 60)
    print()

    client = XiaoBleClient()

    try:
        address = await client.find_device()
        if not address:
            print(
                "Device not found. Make sure XIAO Sense is advertising "
                f"'{PRIMARY_DEVICE_NAME}' (legacy '{LEGACY_DEVICE_NAMES[0]}' also supported)."
            )
            return

        await client.connect(address)

        control_task = asyncio.create_task(interactive_control(client))

        while not control_task.done():
            if not client.is_connected:
                print("  [RECONNECT] Waiting for device to reappear...")
                control_task.cancel()
                try:
                    await control_task
                except asyncio.CancelledError:
                    pass

                reconnect_address = None
                while reconnect_address is None:
                    await asyncio.sleep(3.0)
                    print(f"  [RECONNECT] Scanning for '{PRIMARY_DEVICE_NAME}'...")
                    try:
                        reconnect_address = await client.find_device()
                    except Exception:
                        pass

                gap = time.time() - (client._disconnect_time or time.time())
                print(f"  [RECONNECT] Device found after {gap:.0f}s offline — reconnecting")
                await client.connect(reconnect_address)
                control_task = asyncio.create_task(interactive_control(client))

            await asyncio.sleep(0.5)

        await control_task

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if client.recorder.is_recording:
            client.stop_recording()
            await asyncio.sleep(0.5)
        client.print_stats()
        await client.disconnect()
        print("Done.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
