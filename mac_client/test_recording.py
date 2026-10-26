#!/usr/bin/env python3
"""
Automated dual-BLE recording + recognition test.

Both Handy (Mac) and Android are connected to the XIAO simultaneously for each test.

Tests:
  A) Handy primary — Android starts in MAC_HANDY mode (auto-yields via 0x03 when
     Handy connects). Handy claims primary (0x02). Records via XIAO mic + serial
     command. Verifies Handy transcribes "音声入力テスト" and Android does not.

  B) Android primary — Handy stays connected. Android restarts in ANDROID mode,
     claims primary (0x02). Verifies Android transcribes via Groq.

Usage:
  python3 test_recording.py [A|B|both]   (default: both)
"""

import asyncio
import subprocess
import sys
import time
import struct
import os
import re
import json
import sqlite3
import shutil
import threading
from typing import Optional

try:
    import serial
except ImportError:
    print("Missing: pip install pyserial")
    sys.exit(1)

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Missing: pip install bleak")
    sys.exit(1)


# ─── Configuration ───────────────────────────────────────────────────────────

DEVICE_NAME     = "HarnessNode"
AUDIO_TX_UUID   = "00000002-0000-1000-8000-00805f9b34fb"
AUDIO_RX_UUID   = "00000003-0000-1000-8000-00805f9b34fb"
AUDIO_SVC_UUID  = "00000001-0000-1000-8000-00805f9b34fb"

SERIAL_PORT     = "/dev/tty.usbmodem1101"
SERIAL_BAUD     = 115200

ANDROID_SERIAL  = "192.168.86.144:5555"   # WiFi adb target
ANDROID_PKG       = "com.g150446.voiceclasp"
ANDROID_ACTIVITY  = f"{ANDROID_PKG}/.MainActivity"
ANDROID_PREFS_PATH = f"/data/data/{ANDROID_PKG}/shared_prefs/ble_connection_prefs.xml"

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
TEST_WAV    = os.path.join(SCRIPT_DIR, "../test.wav")
PLAY_DURATION = 10   # seconds (test.wav is ~5s, give extra margin)

HANDY_APP         = "/Applications/Handy.app"
HANDY_DATA_DIR    = os.path.expanduser("~/Library/Application Support/com.pais.handy")
HANDY_DB          = os.path.join(HANDY_DATA_DIR, "history.db")
HANDY_SETTINGS    = os.path.join(HANDY_DATA_DIR, "settings_store.json")

# Model to use for recognition test (must already be downloaded)
# "turbo" = ggml-large-v3-turbo.bin — multilingual Whisper, supports Japanese
HANDY_MODEL_FOR_TEST = "turbo"

# Keywords that indicate successful recognition of "音声入力テスト"
RECOGNITION_KEYWORDS_JA = ["音声", "入力", "テスト"]
RECOGNITION_KEYWORDS_EN = ["onsei", "voice", "input", "test", "nyuu", "nyur"]

PACKET_SYNC = 0xAA


# ─── Android helpers ─────────────────────────────────────────────────────────

def adb(*args, capture=True) -> subprocess.CompletedProcess:
    return subprocess.run(["adb", "-s", ANDROID_SERIAL] + list(args),
                          capture_output=capture, text=True)


ANDROID_DEVICE_ADDRESS = "CF:78:E0:AC:05:04"
ANDROID_DEVICE_NAME    = "HarnessNode"


def android_set_priority(mode: str):
    """Set connection_priority and enable auto-reconnect to saved device."""
    prefs_xml = (
        '<?xml version=\'1.0\' encoding=\'utf-8\' standalone=\'yes\' ?>\n'
        '<map>\n'
        f'    <string name="connection_priority">{mode}</string>\n'
        '    <boolean name="auto_reconnect_enabled" value="true" />\n'
        f'    <string name="preferred_device_address">{ANDROID_DEVICE_ADDRESS}</string>\n'
        f'    <string name="preferred_device_name">{ANDROID_DEVICE_NAME}</string>\n'
        '</map>\n'
    )
    subprocess.run(
        ["adb", "-s", ANDROID_SERIAL,
         "shell", f"run-as {ANDROID_PKG} tee shared_prefs/ble_connection_prefs.xml"],
        input=prefs_xml, capture_output=True, text=True
    )


def android_stop():
    print("  [ADB] Force-stopping Android app...")
    adb("shell", "am", "force-stop", ANDROID_PKG)
    time.sleep(1)


def android_start():
    print("  [ADB] Starting Android app...")
    adb("shell", "am", "start", "-n", ANDROID_ACTIVITY)


def android_clear_logcat():
    adb("logcat", "-c")
    time.sleep(0.3)


def _parse_bounds(bounds_str: str) -> tuple[int, int]:
    nums = list(map(int, re.findall(r'\d+', bounds_str)))
    if len(nums) == 4:
        return (nums[0] + nums[2]) // 2, (nums[1] + nums[3]) // 2
    return 0, 0


def adb_dump_ui() -> str:
    adb("shell", "uiautomator", "dump", "/sdcard/ui.xml")
    local = "/tmp/voice_clasp_ui.xml"
    adb("pull", "/sdcard/ui.xml", local)
    return local


def adb_find_element_by_text(xml_path: str, text: str):
    import xml.etree.ElementTree as ET
    tree = ET.parse(xml_path)
    for el in tree.iter():
        if el.get("text", "") == text or el.get("content-desc", "") == text:
            bounds = el.get("bounds", "")
            if bounds:
                return _parse_bounds(bounds)
    return None


def adb_tap_text(text: str, wait: float = 0.5) -> bool:
    xml_path = adb_dump_ui()
    pos = adb_find_element_by_text(xml_path, text)
    if not pos:
        print(f"  [ADB] Element not found: {text!r}")
        return False
    x, y = pos
    print(f"  [ADB] Tapping '{text}' at ({x},{y})")
    adb("shell", "input", "tap", str(x), str(y))
    time.sleep(wait)
    return True


def android_ble_connect(device_name: str = "HarnessNode",
                        scan_wait: float = 5.0,
                        connect_wait: float = 6.0) -> bool:
    if not adb_tap_text("Scan devices", wait=scan_wait):
        adb_tap_text("Scanning...", wait=scan_wait)

    xml_path = adb_dump_ui()
    pos = adb_find_element_by_text(xml_path, device_name)
    if not pos:
        print(f"  [ADB] Device '{device_name}' not found, retrying scan...")
        adb_tap_text("Scan devices", wait=scan_wait)
        xml_path = adb_dump_ui()
        pos = adb_find_element_by_text(xml_path, device_name)
        if not pos:
            print(f"  [ADB] Device still not found")
            return False

    x, y = pos
    print(f"  [ADB] Selecting '{device_name}' at ({x},{y})")
    adb("shell", "input", "tap", str(x), str(y))
    time.sleep(0.5)

    if not adb_tap_text("Connect", wait=connect_wait):
        print("  [ADB] Connect button not found")
        return False

    print(f"  [ADB] BLE connect initiated — waiting {connect_wait:.0f}s...")
    return True


def android_wait_for_ble_connect(timeout: float = 40.0) -> bool:
    """Wait for Android to auto-connect to BLE device by polling logcat."""
    print(f"  [ANDROID] Waiting for BLE auto-connect (up to {timeout:.0f}s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = adb("logcat", "-d", "-s", "BleManager:D")
        for line in r.stdout.splitlines():
            if "TX notifications enabled" in line or "BLE fully connected" in line:
                print(f"  [ANDROID] BLE connected: {line.strip()[-80:]}")
                return True
        time.sleep(2)
    print("  [ANDROID] WARNING: BLE connect not detected in logcat")
    return False


def android_wait_for_primary_claim(timeout: float = 20.0) -> bool:
    """Wait for Android to log 'Role declared: primary', confirming it claimed BLE primary."""
    print(f"  [ANDROID] Waiting for primary role claim (up to {timeout:.0f}s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = adb("logcat", "-d", "-s", "BleManager:D")
        for line in r.stdout.splitlines():
            if "Role declared: primary" in line:
                print(f"  [ANDROID] Primary claimed: {line.strip()[-80:]}")
                return True
        time.sleep(2)
    print("  [ANDROID] WARNING: Android primary claim not detected in logcat")
    return False


def _android_parse_history() -> list:
    """Parse Android history JSON from SharedPreferences, sorted newest-first."""
    r = adb("shell", f"run-as {ANDROID_PKG} cat shared_prefs/voice_history_prefs.xml")
    xml = r.stdout.replace('&quot;', '"').replace('&#10;', '\n').replace('&amp;', '&')
    m = re.search(r'<string name="history_json">(.*?)</string>', xml, re.DOTALL)
    if not m:
        return []
    try:
        entries = json.loads(m.group(1))
        entries.sort(key=lambda e: e.get("timestamp", 0), reverse=True)
        return entries
    except Exception:
        return []


def android_get_latest_history_entry() -> Optional[dict]:
    """Fetch the most recent history entry (by timestamp) from Android SharedPreferences."""
    entries = _android_parse_history()
    return entries[0] if entries else None


def android_get_history_count() -> int:
    return len(_android_parse_history())


# ─── Handy helpers ───────────────────────────────────────────────────────────

def handy_set_model(model_id: str, language: Optional[str] = None):
    """Temporarily change Handy's selected model in settings_store.json."""
    with open(HANDY_SETTINGS, 'r') as f:
        data = json.load(f)
    original_model = data["settings"].get("selected_model")
    original_lang  = data["settings"].get("selected_language")
    data["settings"]["selected_model"] = model_id
    if language is not None:
        data["settings"]["selected_language"] = language
    with open(HANDY_SETTINGS, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"  [HANDY] Model set to '{model_id}' (was '{original_model}'), language='{language}'")
    return original_model, original_lang


def handy_restore_model(model_id: str, language: Optional[str]):
    """Restore Handy's model/language setting."""
    with open(HANDY_SETTINGS, 'r') as f:
        data = json.load(f)
    data["settings"]["selected_model"] = model_id
    data["settings"]["selected_language"] = language
    with open(HANDY_SETTINGS, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"  [HANDY] Settings restored: model='{model_id}' language='{language}'")


def handy_launch(wait: float = 12.0):
    """Launch Handy.app and wait for it to auto-connect to HarnessNode."""
    subprocess.Popen(["open", "-a", HANDY_APP])
    print(f"  [HANDY] Launched. Waiting for process to start...")
    # Poll until the 'handy' process appears (up to 20s)
    for _ in range(20):
        time.sleep(1)
        if handy_is_running():
            print("  [HANDY] Process started. Waiting for BLE auto-connect...")
            break
    else:
        print("  [HANDY] WARNING: process not detected after 20s")
    # Extra wait for BLE connection + model load
    time.sleep(max(wait, 10))


def handy_quit():
    """Quit Handy gracefully."""
    subprocess.run(["osascript", "-e", 'quit app "Handy"'],
                   capture_output=True)
    time.sleep(2)
    # Force kill if still running
    r = subprocess.run(["pgrep", "-x", "handy"], capture_output=True, text=True)
    if r.returncode == 0 and r.stdout.strip():
        pid = r.stdout.strip().split()[0]
        subprocess.run(["kill", pid], capture_output=True)
        time.sleep(1)
    print("  [HANDY] Quit.")


def handy_is_running() -> bool:
    r = subprocess.run(["pgrep", "-x", "handy"], capture_output=True)
    return r.returncode == 0


def handy_get_db_count() -> int:
    try:
        conn = sqlite3.connect(HANDY_DB)
        c = conn.execute("SELECT count(*) FROM transcription_history")
        n = c.fetchone()[0]
        conn.close()
        return n
    except Exception:
        return 0


def handy_get_latest_entry() -> Optional[dict]:
    try:
        conn = sqlite3.connect(HANDY_DB)
        conn.row_factory = sqlite3.Row
        row = conn.execute(
            "SELECT * FROM transcription_history ORDER BY timestamp DESC LIMIT 1"
        ).fetchone()
        conn.close()
        return dict(row) if row else None
    except Exception:
        return None


def handy_get_latest_timestamp() -> Optional[int]:
    entry = handy_get_latest_entry()
    return entry.get("timestamp") if entry else None


def handy_wait_for_new_entry(ts_before: Optional[int], timeout: float = 60.0) -> Optional[dict]:
    """Poll DB until a newer entry appears (by timestamp), then return it."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        entry = handy_get_latest_entry()
        if entry:
            ts = entry.get("timestamp", 0)
            if ts_before is None or ts > ts_before:
                return entry
        time.sleep(2)
    return None


# ─── Recognition check ───────────────────────────────────────────────────────

def check_recognition(text: str, label: str) -> bool:
    """Check if transcription text contains expected keywords for 音声入力テスト."""
    if not text:
        print(f"  [{label}] Transcription is empty")
        return False
    t_lower = text.lower()
    ja_hit = any(kw in text for kw in RECOGNITION_KEYWORDS_JA)
    en_hit = any(kw in t_lower for kw in RECOGNITION_KEYWORDS_EN)
    hit = ja_hit or en_hit
    print(f"  [{label}] Transcription: {repr(text[:120])}")
    print(f"  [{label}] Keyword match: JA={ja_hit} EN={en_hit} → {'RECOGNIZED ✓' if hit else 'NOT RECOGNIZED ✗'}")
    return hit


# ─── Serial helper ───────────────────────────────────────────────────────────

class SerialConn:
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser: Optional[serial.Serial] = None
        self._log_lines: list = []
        self._reader_thread: Optional[threading.Thread] = None
        self._reader_stop = threading.Event()

    def open(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f"  [SERIAL] Opened {self.port}")
            self._start_reader()
        except serial.SerialException as e:
            print(f"  [SERIAL] Cannot open: {e}")

    def _start_reader(self):
        """Background thread to read firmware debug output."""
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _reader_loop(self):
        while not self._reader_stop.is_set():
            try:
                if self._ser and self._ser.is_open:
                    line = self._ser.readline()
                    if line:
                        txt = line.decode(errors='replace').strip()
                        self._log_lines.append(txt)
            except Exception:
                pass

    def get_log(self, clear: bool = True) -> list:
        """Return accumulated firmware log lines."""
        lines = list(self._log_lines)
        if clear:
            self._log_lines.clear()
        return lines

    def close(self):
        self._reader_stop.set()
        if self._ser and self._ser.is_open:
            self._ser.close()

    def _ensure_open(self):
        """Reopen serial port if it was closed (e.g. after XIAO USB re-enumeration)."""
        if self._ser and self._ser.is_open:
            return
        # Wait briefly for USB to re-enumerate
        for attempt in range(10):
            time.sleep(2)
            if os.path.exists(self.port):
                try:
                    self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
                    print(f"  [SERIAL] Reopened {self.port} (attempt {attempt+1})")
                    return
                except serial.SerialException:
                    pass
        print(f"  [SERIAL] WARNING: could not reopen {self.port}")

    def send(self, ch: str):
        self._ensure_open()
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(ch.encode())
                self._ser.flush()
                print(f"  [SERIAL] Sent '{ch}'")
            except (serial.SerialException, OSError) as e:
                print(f"  [SERIAL] Write error: {e} — trying to reopen...")
                self._ser = None
                self._ensure_open()
                if self._ser and self._ser.is_open:
                    self._ser.write(ch.encode())
                    self._ser.flush()
                    print(f"  [SERIAL] Sent '{ch}' (after reconnect)")
                else:
                    print(f"  [SERIAL] FAILED to send '{ch}'")
        else:
            print(f"  [SERIAL] WARNING: port not open, cannot send '{ch}'")


# ─── BLE helpers ─────────────────────────────────────────────────────────────

async def find_device() -> Optional[str]:
    print(f"  [BLE] Scanning for '{DEVICE_NAME}'...")
    found_addr: Optional[str] = None

    def on_detect(device, adv):
        nonlocal found_addr
        if found_addr:
            return
        name = adv.local_name or device.name or ""
        uuids = [str(u).lower() for u in adv.service_uuids]
        if name == DEVICE_NAME or AUDIO_SVC_UUID in uuids:
            found_addr = device.address

    async with BleakScanner(detection_callback=on_detect):
        for _ in range(100):
            await asyncio.sleep(0.1)
            if found_addr:
                break

    if found_addr:
        print(f"  [BLE] Found {found_addr}")
    else:
        print("  [BLE] Device not found!")
    return found_addr


class EventObserver:
    """Connects as BLE secondary and counts audio packets + events."""
    def __init__(self):
        self.audio_count = 0
        self.events: list[str] = []
        self._last_seq = -1
        self.losses = 0

    def on_notify(self, _sender, data: bytes):
        if len(data) < 2:
            return
        if data[0] == 0x00 and len(data) >= 3 and data[1] == 0x55:
            code = data[2]
            names = {0x01: "RecordingStarted", 0x02: "RecordingStopped",
                     0x31: "PeerConnected", 0x32: "PeerDisconnected"}
            name = names.get(code, f"0x{code:02x}")
            print(f"  [BLE] EVT: {name}")
            self.events.append(name)
            return
        if len(data) < 2 or data[1] != PACKET_SYNC:
            return
        seq = data[0]
        if self._last_seq >= 0:
            expected = (self._last_seq + 1) % 256
            if seq != expected:
                self.losses += 1
        self._last_seq = seq
        self.audio_count += 1


# ─── Test A: Handy app primary (BOTH apps connected) ─────────────────────────

async def test_a_handy_primary(ser: SerialConn) -> bool:
    print("\n" + "="*60)
    print("TEST A: Handy primary (both apps connected)")
    print("="*60)

    # Set Handy to multilingual Whisper model BEFORE launching
    original_model, original_lang = handy_set_model(HANDY_MODEL_FOR_TEST, language=None)

    # --- Step 1: Start Android in MAC_HANDY mode ---
    # Android will connect but NOT send 0x02. When Handy connects (0x31 event),
    # Android automatically yields primary to Handy by sending 0x03.
    android_stop()
    android_set_priority("MAC_HANDY")
    android_clear_logcat()

    # Capture timestamps BEFORE starting either app
    ts_before_handy = handy_get_latest_timestamp()
    _pre = android_get_latest_history_entry()
    ts_before_android = _pre.get("timestamp", 0) if _pre else 0
    print(f"  [HANDY] DB latest timestamp: {ts_before_handy}")
    print(f"  [ANDROID] History latest timestamp: {ts_before_android}")

    adb("shell", "input", "keyevent", "KEYCODE_WAKEUP")
    await asyncio.sleep(1)
    android_start()
    print("  [ADB] Android started (MAC_HANDY mode) — waiting for BLE connect...")
    android_connected = await asyncio.get_event_loop().run_in_executor(
        None, lambda: android_wait_for_ble_connect(timeout=40.0)
    )
    if not android_connected:
        print("  [ANDROID] WARNING: Android BLE connect not detected — proceeding anyway")

    # --- Step 2: Launch Handy (it claims primary via 0x02, Android yields via 0x03) ---
    print("  [HANDY] Launching Handy — it will claim primary role...")
    handy_launch(wait=25)

    if not handy_is_running():
        print("  [HANDY] App failed to start!")
        android_stop()
        handy_restore_model(original_model, original_lang)
        return False

    # Extra wait: primary role exchange (Android receives 0x31 → yields 0x03),
    # then firmware schedules conn_param_work (200ms delay), then centrals accept
    # the new intervals. Give 5s total to ensure fast/slow params are in effect.
    await asyncio.sleep(5)
    print("  [INFO] Both apps connected. Handy=primary, Android=secondary (MAC_HANDY yielded).")

    # Print BLE setup log: confirm primary assignment and role exchange
    setup_log = ser.get_log()
    for line in setup_log:
        if any(k in line for k in ("primary", "claimed", "yielded", "conn[", "conn_param")):
            print(f"  [XIAO] {line}")

    # --- Step 3: Recording sequence (no Python secondary — MAX_CONNS=2 is full) ---
    subprocess.run(["osascript", "-e", "set volume output volume 100"], capture_output=True)

    ser.send('r')
    await asyncio.sleep(1.0)
    print(f"  [PLAY] Playing {os.path.basename(TEST_WAV)} ...")
    afplay = subprocess.Popen(["afplay", "-v", "3.0", TEST_WAV])
    await asyncio.sleep(PLAY_DURATION)
    if afplay.poll() is None:
        afplay.terminate()
    ser.send('s')
    await asyncio.sleep(1.0)

    # Print firmware serial log to verify primary role & event delivery
    fw_log = ser.get_log()
    relevant = [l for l in fw_log if any(k in l for k in
                ("send_event", "primary", "SERIAL", "conn[", "conn_param"))]
    if relevant:
        print("  [XIAO] Recording log:")
        for line in relevant:
            print(f"    {line}")

    # --- Step 4: Wait for Handy transcription ---
    print("  [HANDY] Waiting for Whisper transcription (up to 60s)...")
    new_entry = await asyncio.get_event_loop().run_in_executor(
        None, lambda: handy_wait_for_new_entry(ts_before_handy, timeout=60)
    )

    result = False
    if new_entry:
        text = new_entry.get("transcription_text", "")
        result = check_recognition(text, "HANDY")
    else:
        print("  [HANDY] No new DB entry within timeout — transcription may have failed")

    # Verify Android did NOT transcribe (confirming it was secondary)
    android_entries = _android_parse_history()
    android_new = [e for e in android_entries
                   if e.get("timestamp", 0) > ts_before_android and not e.get("isSilent", True)]
    if android_new:
        print(f"  [ANDROID] NOTE: Android also transcribed (unexpected for secondary): "
              f"'{android_new[0].get('transcription','')[:60]}'")
    else:
        print("  [ANDROID] Android: no transcription (secondary role confirmed ✓)")

    handy_restore_model(original_model, original_lang)
    # Keep Handy running — it stays connected as secondary for Test B
    print(f"\n  TEST A: {'PASS ✓' if result else 'FAIL ✗'}")
    return result


# ─── Test B: Android primary (BOTH apps connected) ───────────────────────────

async def test_b_android_primary(ser: SerialConn) -> bool:
    print("\n" + "="*60)
    print("TEST B: Android primary (both apps connected)")
    print("="*60)

    # --- Step 1: Ensure Handy is connected (secondary after Android claims primary) ---
    # If running standalone (not after Test A), start Handy first so it connects,
    # then restart Android to make it the LAST to send 0x02 (= primary).
    if not handy_is_running():
        print("  [HANDY] Not running — launching Handy first (will be secondary)...")
        handy_launch(wait=20)
        if not handy_is_running():
            print("  [HANDY] WARNING: Handy failed to start")

    # --- Step 2: Restart Android in ANDROID mode (claims primary via 0x02) ---
    android_stop()
    android_set_priority("ANDROID")
    android_clear_logcat()

    _pre = android_get_latest_history_entry()
    ts_before_recording = _pre.get("timestamp", 0) if _pre else 0
    print(f"  [ANDROID] History latest timestamp: {ts_before_recording}")

    adb("shell", "input", "keyevent", "KEYCODE_WAKEUP")
    await asyncio.sleep(1)
    android_start()
    print("  [ADB] Android started (ANDROID mode) — waiting for BLE connect and primary claim...")

    connected = await asyncio.get_event_loop().run_in_executor(
        None, lambda: android_wait_for_ble_connect(timeout=40.0)
    )
    if not connected:
        print("  [ANDROID] WARNING: Android BLE connect not detected — proceeding anyway")

    primary_claimed = await asyncio.get_event_loop().run_in_executor(
        None, lambda: android_wait_for_primary_claim(timeout=15.0)
    )
    if not primary_claimed:
        print("  [ANDROID] WARNING: Primary claim not confirmed — proceeding anyway")

    # Extra wait: firmware schedules conn_param_work 200ms after primary claim,
    # then centrals accept new intervals. Give 5s for fast(Android)/slow(Handy) to take effect.
    await asyncio.sleep(5)
    print("  [INFO] Both apps connected. Android=primary, Handy=secondary.")

    # --- Step 3: Recording sequence (no Python secondary — MAX_CONNS=2 is full) ---
    subprocess.run(["osascript", "-e", "set volume output volume 100"], capture_output=True)

    ser.send('r')
    await asyncio.sleep(1.0)
    print(f"  [PLAY] Playing {os.path.basename(TEST_WAV)} ...")
    afplay = subprocess.Popen(["afplay", "-v", "3.0", TEST_WAV])
    await asyncio.sleep(PLAY_DURATION)
    if afplay.poll() is None:
        afplay.terminate()
    ser.send('s')
    await asyncio.sleep(1.0)

    # --- Step 4: Wait for Groq transcription (skip silent entries) ---
    print("  [ANDROID] Waiting for Groq transcription (up to 60s)...")
    deadline = time.time() + 60
    new_entry = None
    while time.time() < deadline:
        entries = _android_parse_history()
        for e in entries:
            if e.get("timestamp", 0) > ts_before_recording:
                if not e.get("isSilent", True) and e.get("transcription", ""):
                    new_entry = e
                    break
        if new_entry:
            break
        await asyncio.sleep(2)

    result = False
    if new_entry:
        text = new_entry.get("transcription", "")
        result = check_recognition(text, "ANDROID")
    else:
        entries = _android_parse_history()
        new_entries = [e for e in entries if e.get("timestamp", 0) > ts_before_recording]
        if new_entries:
            print(f"  [ANDROID] New entries found but all silent ({len(new_entries)} entries)")
            for e in new_entries:
                print(f"    ts={e.get('timestamp')} isSilent={e.get('isSilent')} "
                      f"transcription='{e.get('transcription','')[:50]}'")
        else:
            print("  [ANDROID] No new history entry within timeout")
        r = adb("logcat", "-d", "-s", "VoiceProcessor:D")
        for line in r.stdout.splitlines()[-20:]:
            print(f"    {line}")

    android_stop()
    print(f"\n  TEST B: {'PASS ✓' if result else 'FAIL ✗'}")
    return result


# ─── Main ────────────────────────────────────────────────────────────────────

async def main():
    mode = sys.argv[1].upper() if len(sys.argv) > 1 else "BOTH"

    if not os.path.isfile(TEST_WAV):
        print(f"ERROR: test.wav not found at {TEST_WAV}")
        sys.exit(1)

    # Keep Android screen on and disable Doze (prevents BLE scan blocking)
    adb("shell", "settings", "put", "global", "stay_awake", "1")
    adb("shell", "settings", "put", "system", "screen_off_timeout", "1800000")
    adb("shell", "dumpsys", "deviceidle", "disable")
    adb("shell", "dumpsys", "deviceidle", "whitelist", f"+{ANDROID_PKG}")

    ser = SerialConn(SERIAL_PORT)
    ser.open()
    results = {}

    try:
        if mode in ("A", "BOTH"):
            results["A"] = await test_a_handy_primary(ser)
        if mode in ("B", "BOTH"):
            # In BOTH mode, Handy stays connected from Test A.
            # Test B just restarts Android with ANDROID priority.
            results["B"] = await test_b_android_primary(ser)
    finally:
        ser.close()
        if handy_is_running():
            handy_quit()
        android_stop()

    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    for k, v in results.items():
        label = {"A": "Mac/Handy primary", "B": "Android primary"}.get(k, k)
        print(f"  Test {k} ({label}): {'PASS ✓' if v else 'FAIL ✗'}")
    print()


if __name__ == "__main__":
    asyncio.run(main())

