# Harness Node

Bidirectional BLE Audio Streaming for XIAO Boards.

This repository was renamed from `voice-bridge-ble` to `harness-node`.
The current nRF52840 Sense firmware path is `nordic-main/`, and its BLE
device name is `HarnessNode`.

Supports:
- **XIAO ESP32S3 Sense** (ESP-IDF)
- **XIAO nRF52840 Sense** (Zephyr/NCS) — `nordic-main`（現行: ジェスチャー録音 + OTA + Battery Service）
- **XIAO nRF54L15 Sense** (Zephyr/NCS) - Latest!

## Project Structure

```
harness-node/
├── esp32s3/                   # ESP-IDF firmware (ESP32S3)
│   ├── main.c                 # Main application
│   ├── adpcm.c/h              # IMA ADPCM codec
│   └── CMakeLists.txt
├── nrf54l15/                  # Zephyr firmware (nRF54L15)
│   ├── src/
│   │   ├── main.c             # BLE service + audio streaming loop
│   │   ├── adpcm.c/h          # ADPCM codec (互換用)
│   │   └── audio_capture.c/h  # PDM microphone capture
│   ├── boards/
│   │   └── xiao_nrf54l15_sense.overlay
│   ├── README.md              # nRF54L15 implementation notes
│   ├── prj.conf               # Zephyr config with Audio
│   └── west.yml               # Zephyr manifest
├── nordic-main/               # Zephyr ジェスチャートリガー音声+OTA for XIAO nRF52840 Sense (HarnessNode) ← 現行
│   ├── src/
│   │   ├── main.c             # BLE サービス + ジェスチャー検出 + DMIC 制御
│   │   ├── audio_capture.c/h  # DMIC キャプチャ（16 kHz / 16-bit / モノラル）
│   │   └── adpcm.c/h          # ADPCM コーデック（互換用）
│   ├── boards/
│   │   └── xiao_ble_nrf52840_sense.overlay  # PDM + IMU + マイク電源 + フラッシュパーティション
│   ├── sysbuild/mcuboot/      # MCUboot 設定
│   ├── prj.conf               # Zephyr 設定（BLE, Audio, MCUmgr）
│   ├── sysbuild.conf          # SB_CONFIG_BOOTLOADER_MCUBOOT=y
│   ├── pm_static.yml          # フラッシュパーティション定義
│   ├── build_and_flash.sh     # 初回 UF2 フラッシュ
│   └── build_and_package_ota.sh  # OTA バイナリ生成（→ ota_update.bin）
├── nrf54-motion/              # Zephyr IMU motion detection test for XIAO nRF54L15 Sense
│   ├── src/
│   │   └── main.c             # IMU wake-up / motion interrupt test
│   ├── boards/
│   │   ├── xiao_nrf54l15_nrf54l15_cpuapp.overlay
│   │   └── xiao_nrf54l15_sense.overlay
│   ├── prj.conf               # Zephyr config for the onboard LSM6DS3TR-C
│   ├── build_and_flash.sh     # Build + USB flash + OTA payload helper
│   ├── build_and_package_ota.sh # Build + OTA payload helper (no USB flash)
│   └── flash.sh               # Compatibility wrapper to build_and_flash.sh
├── pdm_power_test/            # nRF54L15: IMU継続中のPDMマイク省電力実測
│   ├── README.md              # 目的、状態シーケンス、判定基準
│   ├── MEASURE.md             # 測定器別の接続・記録手順
│   └── build_and_flash.sh     # NCS v2.9.2 build + CMSIS-DAP flash
├── boards/seeed/xiao_nrf54l15/ # Custom XIAO nRF54L15 board definition
├── mac_client/                # Mac Python client (shared)
│   ├── nrf52_voice_client.py  # HarnessNode BLE クライアント（自動録音、ジェスチャーイベント表示）
│   ├── gesture_monitor.py     # ジェスチャーイベントモニター（タイムスタンプ付き、録音なし）
│   ├── gesture_validator.py   # BLE接続で掌上0.5秒静止→挙上→掌下静止を検証・6軸グラフ化
│   ├── gesture_dataset_collector.py # 左右・正例/負例を1試行ずつ6軸収集
│   ├── gesture_dataset_analyzer.py  # 特徴量比較・重ね合わせ・疑似横加速度解析
│   ├── imu_trajectory.py      # 6軸BLEバッチの復元・CSV・グラフ共通処理
│   ├── gesture_classifier.py  # オフライン CSV ジェスチャー分類器（検証用）
│   ├── gesture_collector.py   # ジェスチャーデータ収集ツール（ARM_LIFT / DOUBLE_CLENCH）
│   ├── ota_updater.py         # BLE OTA アップデータ（HarnessNode / nRF52840 用）
│   ├── nrf54_controller.py
│   ├── voice_bridge_client.py
│   └── requirements.txt
└── README.md
```

## Hardware Requirements

- **Seeed Studio XIAO ESP32S3 Sense**  or **XIAO nRF54L15 Sense**
- **Mac** with Bluetooth 5.0 support

## Quick Start

### For XIAO ESP32S3 Sense

See `esp32s3/` or the ESP-IDF section below.

### For XIAO nRF52840 Sense — HarnessNode (`nordic-main`)

`nordic-main/` is the primary nRF52840 Sense target in this repository.
It advertises as `HarnessNode` and is the firmware used by the current
Android app and Handy integration.

PDM マイク音声 + LSM6DS3TR-C ジェスチャー検出 + BLE OTA + **バッテリー残量通知** を統合したファームウェアです。XIAOをリストバンドの手の甲側に配置し、掌上で0.5秒静止、挙上、掌下で400 ms静止すると録音を自動開始します（加速度常時 + ジャイロは0.5秒静止成立〜録音終了のみ）。ジェスチャー判別は **ファームウェア側で完結** します。

LED は起動直後に白 1 秒のあと、**未接続アドバタイズ中は青の遅い点滅**（200 ms ON / 1800 ms OFF）、**BLE 接続済み待機中 / 単純なモーション検出中は消灯**、**録音中は赤**です。

バッテリー残量は **BLE Battery Service（UUID 0x180F）** で 1 分ごとに通知されます（録音停止直後にも即時更新）。nRF Connect アプリ等から Battery Level キャラクタリスティック（UUID 0x2A19）で読み取り可能です。

全体構成は `docs/nordic_main_guide.md`、軸の根拠、状態遷移、全閾値、制約、
実機テスト手順は `docs/flex_pronation_gesture.md` を参照してください。

#### ジェスチャー検出アルゴリズム（概要）

録音開始は、以下が順番に成立したとき:

1. **掌上候補で0.5秒静止**: 基板水平（Z絶対比 ≥ 0.75）、重力帯、線形加速度RMS ≤ 4.0 m/s²、姿勢差 ≤ 20°を確認 → ジャイロ ON（50 ms整定）
2. **挙上**: 上向き加速度パルス。掌向きは問わない（開始期限 8 s、0.0.74）
3. **掌下で静止**: 掌上基準からの重力＋gyro連動条件を試行中ラッチし、RMS≤3.0で進入。保持中はRMS>**4.0**が**3**サンプル連続で中断（0.0.74）、**500 ms**維持
4. **最終発火**: 挙上正インパルス ≥ 0.65 m/s **かつ** 掌上基準 phi ≥ 140°（0.0.72）

回外・最終仰角帯・甲上条件は要求しません。しきい値と状態遷移の正本は
`docs/flex_pronation_gesture.md`、保守上の注意は
`docs/lift_detection_maintenance.md`。

録音停止は、開始後3秒の抑制のあと、**挙上軸と逆向きの線形パルス（手下ろし）** + settle
（0.0.69+）。掌上反転・静止のみでは止まらない。失敗パルスは `stop_near_miss`、
録音経過で閾値を緩和（0.0.73）。詳細は `docs/flex_pronation_gesture.md`（現行 `0.0.74`）。

#### 省電力（FW `0.0.102+`）

- **接続中・非録音**: primary の BLE 間隔は 30–50 ms。録音中だけ 7.5 ms。supervision timeout 8 s
- **接続中のアドバタイズ**: 2 台目の枠が空いている間も 100–150 ms で回り続けていたのを、
  接続数が変わってから 30 秒だけ高速、以降 **1–1.2 s** に落とす（`0.0.102+`）
- **未接続で SYSTEMOFF**: コールドブート後は 3 分、**一度接続した後に落ちた場合は 30 分**（`0.0.102+`）。
  電池 15% 未満は常に 3 分。シングル／ダブルどちらのタップでも起床する
- **ライトスリープ**: 10 秒無モーションで IMU ポール 25→50 ms（BLE は維持）。効果は小さい

詳細は `docs/nordic_main_guide.md` の「省電力」。

#### アイドル切断とウォッチドッグ（`0.0.102` で修正）

`0.0.100` まで、接続したまま放置すると Android が切断され、そのままデバイスが
ディープスリープで見つからなくなることがあった。真因は BLE の切断ではなく
**ウォッチドッグリセット**で、`bt_gatt_notify()` が ATT バッファを `K_FOREVER` で
確保するため、相手が ACK を返さなくなるとメインループがそこで固まっていた。
`0.0.100` が supervision を 4 s → 8 s に上げたことで 5 s のウォッチドッグより長くなり、
必ず犬が先に吠えるようになっていた。`0.0.102` でウォッチドッグを 12 s にし、
制御系の通知をブロックしない実装に変えた。診断用に `>>> HB #n` ハートビート、
`>>> DISCONN ... reason=0x..`、`>>> CRASHBOX stage=...` をシリアルに出す。

#### ビルド時のボード名（重要）

ターゲットは **`xiao_ble/nrf52840/sense` のみ**です。
`xiao_ble/nrf52840`（`/sense` なし）でビルドすると
`boards/xiao_ble_nrf52840_sense.overlay` が適用されず、IMU / マイク / ADC /
パーティションの DT エラーになります。詳細は `docs/nordic_main_guide.md` の
「ビルド手順とよくある失敗」、エージェント向けは `AGENTS.md` /
`nordic-main/AGENTS.md`。

#### 初回フラッシュ（MCUboot + アプリを UF2 で書き込み）

```bash
cd nordic-main
./build_and_flash.sh
```

NCS v2.9.2 を使用します。標準インストール先以外にある場合は
`NCS_BASE=/path/to/ncs/v2.9.2` を指定してください。XIAO のリセットボタンを
ダブルタップして UF2 ブートローダに入ると（XIAO-SENSE ドライブが出現）、
スクリプトが MCUboot と署名済みアプリを含む merged UF2 を書き込み、USB の
再列挙まで確認します。

#### BLE OTA アップデート（2 回目以降）

```bash
cd nordic-main
./build_and_package_ota.sh           # OTA バイナリをビルド → ota_update.bin

cd mac_client
python3 -m venv venv
venv/bin/pip install bleak cbor2 pyserial
venv/bin/python ota_updater.py --device HarnessNode ../nordic-main/ota_update.bin
```

アップデータは再起動後に自動で再接続し、転送したイメージが slot 0 で
`active` かつ `confirmed` になったことまで検証します。

#### Mac クライアントで接続・録音

```bash
cd mac_client
source venv/bin/activate
python3 xiao_voice_client.py
```

`0x01` イベント受信で WAV 録音を自動開始、`0x02` 受信で自動停止します。motion_active / motion_settled / sleep_enter / sleep_wake イベントも表示されます。

録音ファイルは `mac_client/output/xiao_recording_YYYYMMDD_HHMMSS.wav` に保存されます（16 kHz / 16-bit / モノラル）。

#### ジェスチャーモニター（録音なし）

```bash
cd mac_client
source venv/bin/activate
python3 gesture_monitor.py
```

掌上0.5秒静止→挙上→掌下静止ジェスチャーを試行ごとに検証する場合:

```bash
cd mac_client
venv/bin/python gesture_validator.py --trials 3
# 開始のみ（停止ジェスチャは評価しない）
venv/bin/python gesture_validator.py --trials 1 --start-only
# 開始後 N 秒維持してから STOP GO（例: 10秒録音）
venv/bin/python gesture_validator.py --trials 1 --window 30 --stop-cue-delay 10
```

カウントダウン後にMacのスピーカーからPing音が鳴ります。Ping後に開始動作を行い、
`START OK`まで掌下を維持します。`recording_start`を受信すると開始判定が確定し、
既定1.3秒後（`--stop-cue-delay`）のGlass音と`STOP GO`のあとにだけ**腕を下ろします**。
STOP GOより前の停止は不成立として扱います。`--start-only` では STOP GO を
出さず、ホスト停止で開始のみを判定します。

掌上候補、0.5秒静止、挙上、掌下で静止、途中リセット、最終発動、
手下ろしでの録音終了を試行ごとに `[OK]` / `[NG]` / `[--]` で表示します。
GO後の判定時間は既定15秒。BLE診断の生ログは画面へ出さず、判定履歴をJSON、
6軸履歴をCSV・PNGへ保存し、試行後に時系列グラフを表示します（`--no-plot` で
ウィンドウのみ抑制）。詳細は `docs/flex_pronation_gesture.md`（ファーム `0.0.74`）を
参照してください。

グラフは上段が加速度（X/Y/Z、m/s²）、下段がジャイロ角速度（X/Y/Z、dps）です。
灰色部分はジャイロ未取得期間（掌上0.5秒静止成立前の正常状態）です。

FW `0.0.97+` では手首ジェスチャー検出は **既定オフ**（タップのみ）。
タップ閾値は `TAP_THS_6D=0x11`（≈1.0625 g）。詳細は `docs/nordic_main_guide.md`。
Android の「ジェスチャー録音」または RX `[0x07, 0x01]` で有効化する（RAM のみ、接続時に再送）。

閾値調整前のラベル付き収集は、FW `0.0.91+` の IMU収集スイッチ（RX `0x06`）を
ONにして行います（デバッグビルドの作り直しは不要）。ジェスチャー検出も ON が必要。
推奨する最小構成は左右それぞれの正例3回と負例3回、合計12回です。手順は
`docs/nordic_main_guide.md` の「ラベル付き6軸データ収集」を参照してください。

BLE イベントをタイムスタンプ付きで表示するだけのミニマルモニターです。ジェスチャー動作確認やデバッグに使用します。

---

### For XIAO nRF54L15 Sense (Latest - Recommended)

#### 自動フラッシュ

```bash
cd nrf54l15
./flash.sh
```

#### 手動フラッシュ

XIAO nRF54L15 は CMSIS-DAP インターフェースのため、pyocd でフラッシュします。

```bash
# 前提: NCS v2.9.2, nRF54L対応のpyOCD 0.37.0以降
python3 -m pip install 'pyocd>=0.37'  # 未インストールの場合

# ビルド
export PATH="/opt/nordic/ncs/toolchains/b8efef2ad5/bin:$PATH"
cd /opt/nordic/ncs/v2.9.2
west build -b xiao_nrf54l15/nrf54l15/cpuapp nrf/samples/voice_bridge_nrf54l15 -- -DBOARD_ROOT=<project_dir>

# フラッシュ (pyocd 経由, ターゲット名は nrf54l)
pyocd flash -t nrf54l build/merged.hex
```

**Key Features of nRF54L15:**
- Latest Nordic MCU (Cortex-M33 128MHz)
- Bluetooth 6.0 support
- Zephyr Audio Subsystem for PDM
- Better audio quality with hardware PDM interface

See `nrf54l15/README.md` for details.

### nRF54L15 implementation overview

The working nRF54L15 path now uses a custom XIAO board definition imported from newer Zephyr board support, because NCS v2.9.2 did not include the XIAO board files by default.

- `boards/seeed/xiao_nrf54l15/`
  - Defines the real XIAO nRF54L15 hardware: UART routing, fixed regulators, IMU alias, and the onboard PDM microphone pins.
  - Uses the correct microphone pins: `P1.12` for `PDM_CLK` and `P1.13` for `PDM_DIN`.
- `nrf54l15/src/audio_capture.c`
  - Initializes the DMIC device through `DT_ALIAS(dmic20)`.
  - Configures 16 kHz / 16-bit / mono capture.
  - Starts and stops the Zephyr DMIC stream and returns PCM buffers to the BLE thread.
- `nrf54l15/src/main.c`
  - Exposes the custom BLE service used by the Mac client.
  - Starts recording on BLE write `0x01`, stops on `0x00`.
  - Streams raw PCM frames in packets formatted as `[sequence][0xAA][PCM data...]`.
  - Falls back to a generated 440 Hz tone only if DMIC initialization or reading fails.
- `mac_client/nrf54_controller.py`
  - Connects over BLE, sends start/stop commands, and stores the received PCM stream as a 16 kHz WAV file.
  - This was updated to match the firmware sample rate, so saved files now have the correct timing.

### Known Hardware Limitation on nRF54L15: Shared PDM / IMU Power Regulator

The Seeed XIAO nRF54L15 Sense board shares a single fixed regulator (`pdm_imu_pwr`, GPIO0.1) between the onboard PDM microphone and the LSM6DS3TR-C IMU. Because of this design, **the microphone cannot be powered on or off independently**—whenever the IMU is needed for gesture detection, the microphone is also powered.

This is why the `nrf54-handy/` firmware keeps `pdm_imu_pwr` ON for the entire BLE session (enabling both IMU gesture detection and PDM recording) and turns it OFF only when BLE is disconnected. By contrast, the `nordic-main/` firmware for nRF52840 Sense can toggle microphone power individually via the dedicated `msm261d3526hicpm-c-en` regulator (P1.10), achieving better battery life for gesture-triggered recording use cases.

The standalone [`pdm_power_test/`](pdm_power_test/README.md) app measures whether
the shared IMU/microphone rail and an active PDM stream create measurable current
steps. It cycles every 20 seconds through both devices powered off, shared power
on with the IMU powered down and microphone asleep, and the same state with the
microphone active.

For details, see `docs/nrf54_power_management.md`.

### nRF52840 Sense の現行サポート

このリポジトリでサポートしている nRF52840 Sense 向けファームウェアは現在
`nordic-main/` のみです。旧 `nrf52-motion/` / `nrf52-voice/` は削除済みで、
BLE 名 `HarnessNode` を使う現行構成へ統一しました。

### IMU motion detection test app

The repository also includes `nrf54-motion/`, a standalone Zephyr app for the Seeed XIAO nRF54L15 Sense onboard LSM6DS3TR-C IMU.

```bash
cd nrf54-motion
./build_and_flash.sh
```

The app polls `imu0` at 26 Hz, calibrates a baseline for about 2.5 seconds after boot, and then prints `Motion detected!` messages on the serial console at 115200 baud when the board is moved. Once the board becomes still again, it logs `Motion settled` and updates the baseline to the new resting orientation.

### nRF54L15 BLE OTA update (`nrf54-motion`)

`nrf54-motion/` supports BLE OTA using MCUboot + MCUmgr/SMP. The OTA target device name is `MotionBridge`.

#### Prerequisites

- NCS v2.9.2
- Python environment with `bleak`
- `cbor2` for `mac_client/ota_updater.py`

```bash
cd mac_client
python3 -m venv ../venv  # if needed
../venv/bin/pip install -r requirements.txt
../venv/bin/pip install cbor2
```

#### Case 1: the device does not yet have OTA-capable firmware

Use USB once to install the OTA-capable firmware and generate the first OTA payload:

- Requires `pyocd`

```bash
cd nrf54-motion
./build_and_flash.sh
```

This command builds with sysbuild + MCUboot, flashes `merged.hex` over USB via `pyocd`, and refreshes `nrf54-motion/ota_update.bin`.

After that initial USB step, later updates can use BLE OTA only.

#### Case 2: the device already has OTA-capable firmware

1. Build a new OTA payload without USB flashing:

- `pyocd` is not required for this step

```bash
cd nrf54-motion
./build_and_package_ota.sh
```

This command only rebuilds the firmware and refreshes `nrf54-motion/ota_update.bin`. It does **not** flash over USB.

2. Make the firmware newer than the running image.

In practice, change the firmware, or rebuild after your intended modifications so the generated signed image differs from the version already running on the board. The firmware exposes a Build Info GATT characteristic, so the build timestamp can be used to verify the switch after reboot.

3. Run the OTA updater:

```bash
cd mac_client
../venv/bin/python3 ota_updater.py ../nrf54-motion/ota_update.bin
```

Expected successful tail output:

```text
Upload complete in ...
Querying image state...
Setting image test flag...
Image test flag set.
Sending reset command...
Device reset. MCUboot will swap slots on next boot.
```

4. Verify that the device came back on the new image.

The `Build Info` characteristic UUID is `00000012-0000-1000-8000-00805f9b34fb`. Read it before and after OTA to confirm that the build timestamp changed.

#### Notes

- OTA support described here is for `nrf54-motion/`, not `nrf54l15/`.
- The updater currently performs upload + test boot request + reset. It does not automatically reconnect after reboot to confirm the new image.
- Use `build_and_flash.sh` only for USB provisioning / recovery.
- Use `build_and_package_ota.sh` when the board already has OTA-capable firmware and you only need a new OTA payload.
- `nrf54-motion/flash.sh` still works, but it is now only a compatibility wrapper around `build_and_flash.sh`.
- For implementation details and maintenance notes, see `docs/nrf54l15_ota_guide.md` and `docs/nrf54l15_ota_status.md`.

## Firmware Setup (ESP-IDF)

### Prerequisites

- ESP-IDF v5.x installed
- XIAO ESP32S3 board support configured

### Build and Flash

```bash
# Set target to ESP32S3
idf.py set-target esp32s3

# Build the project
idf.py build

# Flash to device
idf.py -p /dev/ttyACM0 flash monitor
```

Replace `/dev/ttyACM0` with your actual serial port.

### Pin Configuration

- **PDM CLK**: GPIO 42
- **PDM DATA**: GPIO 41

## Mac Client Setup

### Install Dependencies

```bash
cd mac_client
pip3 install -r requirements.txt
```

### Recording Mode (High Quality)

Record audio to WAV file with control:

```bash
cd mac_client
python3 nrf54_controller.py
```

Then use commands:
- `r` - Start recording via BLE
- `s` - Stop recording via BLE
- `q` - Quit

Recorded files are saved to `mac_client/output/recording_YYYYMMDD_HHMMSS.wav`.

**Note**: The nRF54L15 path records and saves `16kHz / 16-bit / mono` PCM to match the firmware output.

### Real-time Playback Mode

Stream audio to Mac speakers:

```bash
python3 voice_bridge_client.py
```

### Serial Monitor

To view XIAO's serial output for debugging:

```bash
# Auto-detect XIAO and connect
python3 serial_monitor.py

# Specify port
python3 serial_monitor.py -p /dev/tty.usbmodem1101

# List available ports
python3 serial_monitor.py --list

# Reset device
python3 serial_monitor.py --reset
```

On the Atom Echo S3R firmware, the USB serial console at `115200` baud also accepts
simple control commands:

- `r` / `R`: request recording start
- `s` / `S`: request recording stop
- `c` / `C` / `1`: emulate a **single-click**
- `d` / `D` / `2`: emulate a **double-click**
- `h` / `H`: print command help

This is useful for reproducing Handy-side BLE button handling without physically
pressing the device button.

## Technical Specifications

### Audio Format
- Sample Rate: 16,000 Hz
- Bit Depth: 16-bit (PCM) / 4-bit (ADPCM compressed)
- Channels: Mono

### BLE Configuration
- MTU Size: 247 bytes on the current nRF54L15 path
- PHY: 2M PHY (for low latency)
- Connection Interval: 7.5ms - 15ms

### PCM Packet Format (nRF54L15)
```
Byte 0: Sequence number (0-255)
Byte 1: Sync byte (0xAA)
Bytes 2..N: 16-bit little-endian PCM samples
```

### ADPCM Frame Format
```
Byte 0: Sequence number (0-255)
Bytes 1-60: ADPCM encoded audio data (120 samples)
```

## Troubleshooting

### Firmware Issues

1. **No audio data on nRF54L15**: Confirm the custom XIAO board definition is being used and that the firmware is built for `xiao_nrf54l15/nrf54l15/cpuapp`
2. **BLE connection fails**: Reset the XIAO board
3. **Audio quality issues**: Verify sample rate configuration

### Mac Client Issues

1. **Device not found**: Ensure XIAO is advertising (check serial monitor)
2. **Audio playback fails**: Check PyAudio installation
   ```bash
   # macOS: Install portaudio
   brew install portaudio
   pip3 install pyaudio
   ```

## License

MIT License
