# HarnessNode / nordic-main ファームウェア運用ガイド

`voice-clasp-node` リポジトリで現在メインとなっている、XIAO nRF52840 Sense 向けのジェスチャートリガー式 BLE 音声ファームウェアです。腕の動きを IMU で検知し、録音の開始・停止を自動制御します。

---

## 概要

| 項目 | 内容 |
|------|------|
| ターゲットボード | `xiao_ble/nrf52840/sense`（Seeed XIAO nRF52840 Sense） |
| BLE デバイス名 | `HarnessNode` |
| ブートローダ | MCUboot（Adafruit UF2 ブートローダ経由で 0x27000 に配置） |
| OTA | MCUboot + BLE SMP（MCUmgr） |
| PDM マイク | DMIC（Zephyr DMIC API、RIGHT チャンネル、マイク電源制御あり） |
| IMU | LSM6DS3TR-C（加速度 ODR 416 Hz。ジャイロはオンデマンド 104 Hz） |
| 音声フォーマット | 16 kHz / 16-bit / モノラル PCM |
| LED 方針 | 起動後は待機時消灯。録音ジェスチャー成立後の録音中のみ赤 |
| 署名バージョン（目安） | `0.0.77+`（single_tap `0x14` とdouble回帰修正を含む） |
| OTA 手順 | [`ota_update_notes.md`](ota_update_notes.md) |
| Android single_tap | `0x14`を診断ログへ記録（Kindle/G2操作はdouble専用） |

---

## ファイル構成

```
nordic-main/
├── src/
│   ├── main.c                    # BLE サービス + ジェスチャー検出 + DMIC 制御
│   ├── audio_capture.c/h         # DMIC キャプチャ（16 kHz / 16-bit / モノラル）
│   └── adpcm.c/h                 # ADPCM コーデック（互換用）
├── boards/
│   └── xiao_ble_nrf52840_sense.overlay  # PDM + IMU + マイク電源 + フラッシュパーティション
├── sysbuild/mcuboot/             # MCUboot 設定
├── prj.conf                      # Zephyr 設定（BLE, Audio, MCUmgr）
├── sysbuild.conf                 # SB_CONFIG_BOOTLOADER_MCUBOOT=y
├── pm_static.yml                 # フラッシュパーティション定義
├── build_and_flash.sh            # 初回 UF2 フラッシュスクリプト
├── build_and_package_ota.sh      # OTA バイナリ生成スクリプト（→ ota_update.bin）
└── AGENTS.md                     # エージェント向け board / ビルド注意
```

---

## ビルド手順とよくある失敗

### 正しいボードターゲット

| 正しい | 誤り（よくある） |
|--------|------------------|
| **`xiao_ble/nrf52840/sense`** | `xiao_ble/nrf52840`（`/sense` なし） |
| | `xiao_ble` のみ |

- NCS 上の identifier: `zephyr/boards/seeed/xiao_ble/xiao_ble_nrf52840_sense.yaml`
- アプリ overlay は board 名に紐づく:
  `boards/xiao_ble_nrf52840_sense.overlay`
  → **`/sense` 付きターゲットのときだけ**自動適用される
- overlay が載らないと `imu0`・マイク電源・バッテリ ADC・slot パーティションが
  DT 上に存在せず、`main.c` や flash_map 周りで **未定義マクロの嵐**になる。
  これは C ロジックのバグではなく **ボード指定ミス**である

### 推奨コマンド

```bash
cd nordic-main
./build_and_package_ota.sh    # sysbuild + 署名 OTA → ota_update.bin（書込みなし）
./build_and_flash.sh          # 初回 UF2 向け（ある場合）
```

手動 west（スクリプトと同等）:

```bash
# NCS v2.9.2 の west ワークスペースで実行すること
west build -p always --sysbuild -b xiao_ble/nrf52840/sense \
  /path/to/voice-clasp-node/nordic-main \
  --build-dir /path/to/voice-clasp-node/nordic-main/build
```

- **必ず** `--sysbuild`（`sysbuild.conf` で MCUboot 有効）
- sysbuild なしのアプリ単体ビルドはパーティション / MCUmgr 前提が崩れやすい
- NCS: **v2.9.2**（`NCS_BASE` でパス上書き可）

### 失敗の切り分け

| 症状 | 原因 | 対処 |
|------|------|------|
| `DT_N_ALIAS_imu0_*` / `zephyr_user` / mic GPIO 未定義 | board に `/sense` がない | `xiao_ble/nrf52840/sense` |
| `slot0_partition` / flash_map 関連 | board 誤り or sysbuild なし | sense + `--sysbuild`、必要なら `-p always` |
| `west: unknown command "build"` | NCS 外で west を実行 | NCS ルートへ移動 or 付属スクリプト |
| `ccache: command not found` | ツールチェーン PATH 不足 | NCS toolchain の `bin` を PATH へ |

エージェント向けの短縮版はリポジトリ直下 `AGENTS.md` / `CLAUDE.md`、
および `nordic-main/AGENTS.md` を参照。

---

## フラッシュレイアウト

| パーティション | 開始アドレス | 備考 |
|--------------|------------|------|
| Adafruit UF2 ブートローダ | `0x000000` | 書き換え不要 |
| MCUboot | `0x027000` | Adafruit がジャンプするアドレス |
| slot0（稼働中アプリ） | `0x033000` | 署名済みアプリイメージ |
| slot1（OTA 受信バッファ） | `0x085000` | OTA 転送先、MCUboot がスワップ |

---

## BLE サービス仕様

### Battery Service（標準 UUID）

**サービス UUID**: `0000180f-0000-1000-8000-00805f9b34fb`（Bluetooth SIG 標準 Battery Service）

| キャラクタリスティック | UUID | プロパティ | 説明 |
|----------------------|------|-----------|------|
| Battery Level | `00002a19-0000-1000-8000-00805f9b34fb` | Read, Notify | バッテリー残量（0〜100%） |

- nRF Connect や iOS/Android の標準 API で直接読み取り可能
- 1 分ごとに更新。録音停止直後にも即時更新。変化があると Notify が発火する
- 詳細な実装・回路知見は `docs/nrf52_battery_guide.md` を参照

### Audio Service

**サービス UUID**: `00000001-0000-1000-8000-00805f9b34fb`

| キャラクタリスティック | UUID | プロパティ | 説明 |
|----------------------|------|-----------|------|
| TX（送信） | `00000002-0000-1000-8000-00805f9b34fb` | Notify | 音声 PCM パケット / イベントパケット |
| RX（受信） | `00000003-0000-1000-8000-00805f9b34fb` | Write | 制御コマンド |

### RX コマンド（ホスト → ファームウェア）

| バイト値 | 動作 |
|---------|------|
| `0x01` | 録音開始 |
| `0x00` | 録音停止 |

### TX パケット形式（ファームウェア → ホスト）

#### 音声 PCM パケット

```
[seq: 1 byte][0xAA: 1 byte][PCM data: 16-bit LE samples...]
```

- `seq`: シーケンス番号（0–255、ロールオーバー）
- `0xAA`: 音声パケット識別バイト
- `PCM data`: 16-bit リトルエンディアン PCM サンプル列

#### イベントパケット

```
[0x00: 1 byte][0x55: 1 byte][code: 1 byte][optional data: 4 bytes]
```

| コード | イベント名 | オプションデータ | 説明 |
|--------|-----------|----------------|------|
| `0x01` | `recording_start` | なし | 録音開始（ジェスチャートリガー後） |
| `0x02` | `recording_stop` | なし | 録音停止 |
| `0x10` | `motion_active` | x, y, z f32 LE（各 4 byte） | モーション検出開始、xyz 加速度値 |
| `0x11` | `motion_settled` | x, y, z f32 LE + elapsed_ms u32 + avg/peak_speed/distance f32 LE（計 28 bytes） | モーション静定、詳細メトリクス |
| `0x12` | `double_tap` | なし | リストバンド表側から基板面へ垂直に行ったダブルタップ |
| `0x14` | `single_tap` | なし | 同上のシングルタップ（間隔窓を超えた単独衝撃） |
| `0x40` | `operation_mode` | effective mode u8 + pending mode u8 | Android設定モードの同期状態（0=通常、1=運転、0xff=pendingなし） |
| `0x20` | `sleep_enter` | なし | ライトスリープ移行（10 秒無動作） |
| `0x21` | `sleep_wake` | なし | ライトスリープ復帰（モーション検出） |
| `0x30` | `gesture_diag` | stage/reason u8 + value1/2/3 f32 LE | USBなしのジェスチャー内部診断 |

---

## ジェスチャー検出アルゴリズム

この章は運用時の概要です。軸調査の根拠、判定式、状態遷移、全閾値、既知の
制約、実機テスト項目は [掌上0.5秒静止→挙上→掌下静止仕様](flex_pronation_gesture.md)
を参照してください。

### IMU 軸と取り付け方向

Seeed Studio 公式の XIAO nRF52840 Sense KiCad 基板データと ST の LSM6DS3TR-C 軸図を照合すると、基板上のセンサー軸は次の向きになります。

| 軸 | XIAO 基板上の向き | リストバンド装着時の用途 |
|----|------------------|------------------------|
| `+X` | 基板長手方向、USB 端子から離れる向き | 前腕を横切る方向 |
| `+Y` | 基板短い辺に平行（5V/GND/3V 側） | 前腕に沿う＝回内・回外軸（**gyro_y**） |
| `+Z` | 部品面から外向き | 基板水平（\|Z 比\|）と掌向きの相対判定 |

資料: [Seeed Studio XIAO nRF52840 Series](https://wiki.seeedstudio.com/XIAO_BLE/)、[LSM6DS3TR-C datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)

ジャイロは待機時 power-down。掌上候補の0.5秒静止成立で 104 Hz / ±500 dps を ONし、50 ms整定後に使用する。録音終了で OFFする。

### 録音開始トリガー（掌上0.5秒静止 → 挙上 → 掌下静止）

XIAOをリストバンドの手の甲側に置き、部品面を皮膚側、基板X軸を前腕と直交、Y軸を
前腕に沿わせる。右手・左手とUSB端子方向の違いは、掌上候補から掌下への相対姿勢変化で吸収する。

1. 甲側装着で掌を上にし、`|Z|/|a| ≥ 0.75`、`|a|` 8.5–11.5 m/s²、線形加速度RMS ≤ 4.0 m/s²、姿勢差20°以内を0.5秒維持する。成立時にジャイロON、50 ms整定後に使用する。
2. 成立時の重力方向を掌上基準とし、重力LPで姿勢成分を除いた線形加速度から上向き加速パルスを検出する（物理動作の目安は約5 cm上昇）。
3. holdは掌上基準からの重力反転とgyro連動条件が成立してから開始し、成立した掌下は試行中ラッチする。線形加速度RMS ≤ 3.0 m/s²で進入し、hold中はRMS > 3.5 m/s²が2サンプル連続した場合だけ中断する。姿勢差15°以内を500 ms維持すると録音を開始する。挙上はgyro起動後5秒以内に開始し、挙上開始から最終静止完了までは4.5秒未満とする。挙上 +imp≥0.30、XY免除は回内前liftかつ入口+imp≥0.30のときだけ（0.0.68）。

回外・最終仰角帯・甲上条件は要求しない。詳細は`docs/flex_pronation_gesture.md`を参照する。

### 録音停止トリガー

MATCH 時に挙上パルスの線形加速度方向を単位ベクトル `L` として固定する（弱ければ重力 LP）。
開始後 **3000 ms** は停止判定しない。その後:

1. `a_opp = -dot(linear, L)` が **≥ 0.25 m/s²** の短いパルス
2. 負インパルス ≥ min(0.35, max(0.10, 0.20×lift_imp))、長さ **60–2000 ms**
3. パルス後 **80 ms** の quiet settle（`|a|` 7.5–12.5、linear ≤ 4.0）

成立で `stop_requested = true` → DMIC 停止 → イベント `0x02` → ジャイロ OFF。
掌上反転・静止のみでは止めない（0.0.69+、感度は 0.0.71）。ホスト `0x00` と
シリアル `'s'` による停止は従来どおり。

### ライトスリープ

10 秒間 `motion_active` でなく録音中でもない場合、ライトスリープに移行します。

| 状態 | IMU ポーリング | BLE 送信 |
|------|-------------|---------|
| 通常（アクティブ） | 25 ms | — |
| ライトスリープ移行時 | → 50 ms | `0x20 sleep_enter` |
| ライトスリープ中 | 50 ms | — |
| ライトスリープ復帰時 | → 25 ms | `0x21 sleep_wake` |

BLE 接続はスリープ中も維持されます。録音停止後もタイマーはリセットされ、即座にスリープに入ることはありません。

### シングル／ダブルタップ

LSM6DS3TR-C のハードウェア判定を使用し、部品面を皮膚側にした装着状態で
リストバンド表側から基板面へ垂直に行うタップを Z 軸で検出する。
閾値は約 0.5 g、2 回の最大間隔は約 308 ms、認識後の共有クールダウンは 700 ms。
`DOUBLE_TAP_EN` 有効時、ファームウェアは1打目のsingle候補を約330ms保留し、窓内の2回目をdoubleとして確定する。
窓内に2回目がなければsingleを送信する。確定後の連続イベントには700msのクールダウンを適用する。

| モード | single (`0x14`) | double (`0x12`) |
|--------|-----------------|-----------------|
| 通常 | BLE 通知のみ。録音は変更しない。待機中の開始ジェスチャー候補を破棄し 3 s re-arm | 同左 |
| 運転 | BLE 通知のみ（録音トグルなし） | primary 接続中に録音開始／停止を交互。掌上ジェスチャーは無効 |

Android は RX characteristic に `[0x05, mode]`（`mode=0` 通常、`mode=1` 運転）を
書き込み、Node は `0x40`（`[0x00,0x55,0x40,effective,pending]`）で応答する。
録音中のモード変更は pending として保持し、録音停止後に適用する。
ライトスリープ中は復帰し、タップが掌上静止開始ジェスチャーとして
扱われないよう、待機中の開始候補を破棄する（通常モード）。

#### ハードウェア割り込み構成

この機能は、25 / 50 ms ごとに取得する加速度値からCPUがタップ波形を分類する
ソフトウェア検出ではない。LSM6DS3TR-C 内蔵のtap/double-tapエンジンと
**INT1ハードウェア割り込み**を使用する。

```
リストバンド表側への衝撃
  → LSM6DS3TR-C内蔵判定（Z軸、閾値・Shock・Quiet・Duration）
  → TAP_SRCに single-tap または double-tap / Z を記録
  → IMU INT1をassert
  → XIAO P0.11のGPIO割り込み
  → メインループでTAP_SRCを確認（double 優先）
  → BLE [0x00, 0x55, 0x12] または [0x00, 0x55, 0x14]
```

- 基板DTSの `irq-gpios = <&gpio0 11 GPIO_ACTIVE_HIGH>` により、IMU INT1は
  nRF52840のP0.11へ接続される。
- `INT1_CTRL` の加速度・ジャイロdata-ready割り込みを無効化し、`MD1_CFG` の
  single-tap と double-tap を INT1 へ割り当てる。割り込み処理はフラグを立てるだけで、
  I2C読み出しとBLE通知はメインループ側で行う。
- NCS v2.9.2のLSM6DSLドライバはこのセンサーの`SENSOR_TRIG_DOUBLE_TAP`を
  実装していない。そのためtap関連レジスタをI2Cで直接設定し、ドライバの
  `SENSOR_TRIG_DATA_READY`用INT1配送経路をGPIO通知手段として再利用する。
  実際の割り込み源はdata-readyではなく、IMUが判定したタップである。
- 初期化に失敗した場合はタップ検出だけを無効化してログへ理由を出し、既存の
  IMUポーリング、録音ジェスチャー、BLE接続は継続する。

#### 実機確認結果

2026-08-24、ファームウェア`0.0.55`をXIAO nRF52840 SenseへOTA適用し、
部品面を皮膚側にして装着した状態で確認した（当時は double のみ有効）。

| 試験 | 結果 |
|------|------|
| リストバンド表側をダブルタップ | 3 / 3 検出 |
| 表側を1回だけタップ | 2 / 2 誤検出なし（single 未実装時） |
| タップせず腕を自然に上げ下げ | 2 / 2 誤検出なし |
| 合計 | **7 / 7 PASS** |

single (`0x14`) はファーム `0.0.76` で実装・OTA 済み。検出系の実機再確認は未実施。  
Android 側の受信・UX は未実装（[`Android未実装_シングルタップイベント対応.md`](Android未実装_シングルタップイベント対応.md)）。

---

## モーション検出パラメータ

### サンプリング / キャリブレーション

| パラメータ | 値 | 説明 |
|-----------|---|------|
| `ACCEL_ODR_HZ` | 416 | 加速度センサ ODR（Hz） |
| `MOTION_SAMPLE_INTERVAL_MS` | 25 | ソフトウェアポーリング間隔（ms） |
| `CALIBRATION_SAMPLES` | 25 | 起動時ベースライン計測サンプル数 |
| `ACTIVITY_WINDOW_SAMPLES` | 4 | アクティビティ判定ウィンドウ（サンプル数） |

### モーション検出しきい値

| パラメータ | 値（m/s²） | 説明 |
|-----------|----------|------|
| `MOTION_ENTRY_ACTIVITY_MS2` | 8.0 | モーション開始判定：ウィンドウ内の活動量 |
| `MOTION_ENTRY_PEAK_MS2` | 2.4 | モーション開始判定：ピーク加速度 |
| `MOTION_SETTLE_ACTIVITY_MS2` | 4.0 | 静定判定：ウィンドウ内の活動量 |
| `MOTION_SETTLE_PEAK_MS2` | 1.4 | 静定判定：ピーク加速度 |
| `MOTION_START_WINDOWS` | 2 | モーション開始に必要な連続ウィンドウ数 |
| `MOTION_SETTLE_WINDOWS` | 2 | 静定判定に必要な連続ウィンドウ数 |
| `BASELINE_ALPHA` | 0.03 | ベースライン更新の指数移動平均係数 |
| `REPORT_COOLDOWN_MS` | 700 | 連続レポートのクールダウン（ms） |

### ジェスチャー判定しきい値

| パラメータ | 値 | 説明 |
|-----------|---|------|
| `GESTURE_START_PALM_UP_Z_MIN_RATIO` | 0.75 | 開始時の基板水平（\|Z比\|） |
| `GESTURE_PALM_UP_DWELL_MS` | 500 ms | 掌上候補の連続静止 |
| `GESTURE_PALM_UP_DWELL_TILT_MAX_DEG` | 20° | 候補開始姿勢からの許容差 |
| `GESTURE_START_QUIET_ACCEL_MS2` | 4.0 m/s² | 掌上候補の線形加速度/RMS上限 |
| `GESTURE_GYRO_SETTLE_MS` | 50 ms | ジャイロ起動後の整定待ち |
| `GESTURE_PRONATION_MIN_DEG` | 15° | hold 反転の重力phi |
| `GESTURE_PRONATION_Z_RATIO_DONE` | 0.40 | hold 反転のZ比変化 |
| `GESTURE_PRONATION_Z_SIGN_MIN_MS2` | 2.0 m/s² | Z符号反転と認める\|az\|下限 |
| `GESTURE_LIFT_ACCEL_MIN_MS2` | 0.40 m/s² | 上向き加速パルス下限 |
| `GESTURE_LIFT_BRAKE_MIN_MS2` | 0.15 m/s² | 逆向き減速パルス下限 |
| `GESTURE_LIFT_POS_IMPULSE_MIN_MS` | 0.30 m/s | 正インパルス下限（0.0.68） |
| `GESTURE_MATCH_POS_IMPULSE_MIN_MS` | 0.65 m/s | 最終発火の挙上全体インパルス下限（0.0.72） |
| `GESTURE_MATCH_PRONATION_MIN_DEG` | 140° | 最終発火の掌上基準phi下限（0.0.72） |
| `GESTURE_LIFT_NEG_IMPULSE_MIN_MS` | 0.015 m/s | 負インパルス下限 |
| `GESTURE_LIFT_BRAKE_RATIO_MIN` | 0.05 | 減速/加速インパルス比下限 |
| `GESTURE_LIFT_PULSE_MIN_MS` | 150 ms | 短すぎる加減速パルスの下限 |
| `GESTURE_STOP_OPP_ACCEL_MIN_MS2` | 0.25 | 録音停止の逆向き a 下限（0.0.71） |
| `GESTURE_STOP_OPP_ACCEL_SOFT` / `SOFT2` | 0.18 / 0.15 | 5 s / 10 s 後 peak（0.0.73） |
| `GESTURE_STOP_OPP_IMPULSE_MIN_MS` | 0.10 | 録音停止の負インパルス下限 |
| `GESTURE_STOP_OPP_IMPULSE_SOFT` / `SOFT2` | 0.08 / 0.07 | 5 s / 10 s 後 imp |
| `GESTURE_STOP_OPP_IMPULSE_LIFT_RATIO` | 0.20 | 負インパルス ≥ ratio×lift（上限あり） |
| `GESTURE_STOP_OPP_IMPULSE_LIFT_CAP_MS` | 0.35 | lift 相対閾値の上限 |
| `GESTURE_STOP_OPP_PULSE_MIN_MS` / `MAX` | 60 / 2000 | パルス時間窓 |
| `GESTURE_STOP_OPP_PULSE_SLOW_MS` | 180 | slow-path 最短パルス（0.0.73） |
| `GESTURE_STOP_PULSE_GAP_MS` | 50 | パルス終端ヒステリシス |
| `GESTURE_STOP_SETTLE_MS` / soft / soft2 | 80 / 50 / 40 | パルス後 quiet |
| `GESTURE_STOP_SOFTEN_AFTER_MS` / `2` | 5000 / 10000 | soft 閾値への経過 |
| `GESTURE_HOLD_GYRO_INTEGRATE_RATE_DPS` | 10 dps | hold 回内の積分対象レート |
| `GESTURE_HOLD_GYRO_ANGLE_MIN_DEG` | 30° | hold 回内の ∫ω_y 下限（0.0.70） |
| `GESTURE_HOLD_GYRO_XY_PEAK_RATIO_MIN` | 0.42 | peak \|ω_x\| / peak \|ω_y\| |
| `GESTURE_LIFT_PREFLIP_MAX_DEG` | 50° | 挙上時 \|∫ωy\| 上限（回内前判定） |
| `GESTURE_LIFT_XY_WAIVER_IMPULSE_MIN_MS` | 0.30 | XY 免除に必要な入口 +imp |
| `GESTURE_FINAL_QUIET_RATE_DPS` | 90 dps | hold 進入時のみ |
| `GESTURE_LIFT_FINAL_TILT_MAX_DEG` | 15° | 静止開始時の重力方向からの保持中姿勢差上限 |
| `GESTURE_FINAL_STILL_RMS_MS2` | 3.0 m/s² | 4サンプル静止RMS上限 |
| `GESTURE_FINAL_HOLD_RMS_EXIT_MS2` | 4.0 m/s² | hold中のRMS中断閾値（0.0.74） |
| `GESTURE_FINAL_HOLD_RMS_EXIT_SAMPLES` | 3 | hold中断に必要な連続超過数（0.0.74） |
| `GESTURE_FINAL_HOLD_MS` | 500 ms | 最終静止保持時間（0.0.68） |
| `GESTURE_GRAVITY_LP_TAU_S` | 0.30 s | 重力推定の低通時定数 |
| `GESTURE_LIFT_START_TIMEOUT_MS` | 8000 ms | gyro起動後に挙上を開始するまでの期限（0.0.74） |
| `GESTURE_MOTION_COMPLETE_MAX_MS` | 4500 ms | 挙上開始から500 ms最終静止完了までの上限 |
| `GESTURE_RETRIGGER_BLOCK_MS` | 3000 ms | 開始直後の停止抑制（基準再ロック）/ 停止後の再開始抑制 |
| `GYRO_ODR_HZ` | 104 | オンデマンドジャイロ ODR |
| `GYRO_FULL_SCALE_DPS` | 500 | ジャイロ FS |
| `SLEEP_IDLE_TIMEOUT_MS` | 10000 ms | ライトスリープ移行までの無動作時間 |
| `SLEEP_POLL_INTERVAL_MS` | 50 ms | スリープ中の IMU ポーリング間隔 |

---

## ビルドと OTA

NCS v2.9.2 を使用します。SDK が標準の `/opt/nordic/ncs/v2.9.2` または
`/opt/nordic/ncs/2.9.2` 以外にある場合は、SDK workspace を `NCS_BASE` で
指定してください。`west` が PATH にない場合は、実行可能な nRF Util を
`NRFUTIL` で指定すると SDK Manager のツールチェーン環境を使用できます。

### 初回フラッシュ（MCUboot + アプリを UF2 で書き込み）

```bash
cd nordic-main
./build_and_flash.sh
```

XIAO のリセットボタンをダブルタップして UF2 ブートローダに入ると
（XIAO-SENSE ドライブが出現）、スクリプトが merged UF2 を書き込み、
アプリのUSBシリアル再列挙まで確認します。

### OTA バイナリのビルド

```bash
cd nordic-main
./build_and_package_ota.sh
# → nordic-main/ota_update.bin が生成される
```

OTA バイナリのバージョンは稼働中ファームウェアより新しくする必要があります。`prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` を更新してからビルドしてください（例: `"0.0.2+0"`）。

### BLE OTA アップデート（2 回目以降）

```bash
cd mac_client
python3 -m venv venv
venv/bin/pip install bleak cbor2 pyserial
venv/bin/python ota_updater.py --device HarnessNode ../nordic-main/ota_update.bin
```

正常終了時の出力例:

```
Scanning for 'HarnessNode'...
Found: <address>
Connected. MTU=244
Upload complete in ~104s
Querying image state...
Setting image test flag...
Image test flag set.
Sending reset command...
Device reset. MCUboot will swap slots on next boot.
Waiting for the updated device to return...
OTA verified: uploaded image is active and confirmed in slot 0 (version=...).
```

---

## Mac クライアント

### xiao_voice_client.py — BLE 録音クライアント

HarnessNode に接続し、`0x01` 受信で WAV 録音を自動開始、`0x02` 受信で自動停止します。motion_active / motion_settled / double_tap / single_tap / sleep_enter / sleep_wake イベントも画面表示します。

```bash
cd mac_client
source venv/bin/activate
python3 xiao_voice_client.py
```

録音ファイルは `mac_client/output/xiao_recording_YYYYMMDD_HHMMSS.wav` に保存されます（16 kHz / 16-bit / モノラル）。

### gesture_monitor.py — ジェスチャーモニター

録音機能なし。BLE イベントをタイムスタンプ付きで表示するだけのミニマルモニターです。ジェスチャー動作確認やデバッグに使用します。

```bash
cd mac_client
source venv/bin/activate
python3 gesture_monitor.py
```

表示イベント: `motion_active`（x/y/z）、`motion_settled`（x/y/z + elapsed/peak/dist）、`double_tap`、`single_tap`、`recording_start`、`recording_stop`、`sleep_enter`、`sleep_wake`

### gesture_validator.py — 掌上0.5秒静止→挙上→掌下静止ジェスチャー検証

試行ごとにカウントダウンと Ping 音を合図に、掌上で0.5秒静止→挙上→掌下で0.4秒静止する。
`recording_start`受信時に`START OK`を表示し、1.3秒後のGlass音と`STOP GO`後にだけ掌上へ戻す。
これにより開始評価へ停止動作を混入させない。条件ごとの `[OK]` / `[NG]` / `[--]` を表示し、
生の診断ログは JSON へ保存する。GO後の判定時間は既定15秒。開始後はホスト `0x00` を送らず、
掌上の `recording_stop` を待つ。
履歴有効ファームでは試行ごとに6軸CSV・PNGを保存し、加速度XYZとジャイロXYZの
時系列グラフを表示する。`--no-plot`でPNG保存のみ、`--no-plot-files`で画像生成も無効にできる。

グラフ上では、gyro起動前の灰色区間は正常な未取得区間である。gyro Yは回内・回外の主軸、
gyro X/Zは挙上に伴う複合回転を含む。録音停止は手下ろし（線形加速度の逆向きパルス）で行い、
対応し得るため、`START OK` と `STOP GO` の前後を分けて評価する。

物理操作を伴う試験は内容と回数を事前に説明し、準備完了を確認してから1試行ずつ開始する。
カウントダウン、`GO`、接続状態、最終結果はmacOS Terminalへ表示する。ログも保存する場合は
`tee` を使い、Terminalのライブ表示を消さない。

```bash
cd mac_client
venv/bin/python gesture_validator.py --trials 1 --window 15 \
  --json-output /private/tmp/voice-clasp-node-volar-sequence.json
venv/bin/python gesture_validator.py --self-test
```

詳細は `docs/flex_pronation_gesture.md` を参照。

### gesture_classifier.py — オフライン分類器（検証用）

CSV ファイル（`gesture_data.csv`）を読み込み、2 特徴量ベースのジェスチャー分類を実行します。ジェスチャーしきい値のチューニング・検証に使用します。

```bash
cd mac_client
source venv/bin/activate
python3 gesture_classifier.py
```

---

## ジェスチャ履歴デバッグビルド

本番は `GESTURE_DEBUG_HISTORY=0`（既定）。判定履歴と6軸履歴バッチを有効にするには:

```bash
# sysbuild の west configure 引数に C フラグを足す
west build -p always --sysbuild -b xiao_ble/nrf52840/sense \
  /path/to/voice-clasp-node/nordic-main --build-dir /tmp/voice-clasp-node-debug \
  -- -DEXTRA_CFLAGS=-DGESTURE_DEBUG_HISTORY=1
```

または `nordic-main/src/main.c` の `#ifndef GESTURE_DEBUG_HISTORY` 既定を一時的に `1` にする。

履歴有効時は従来の判定履歴 `0x33–0x35` に加え、40 Hzの加速度XYZ・ジャイロXYZを
`0x36 trajectory_begin`、`0x37 trajectory_chunk`、`0x38 trajectory_end` でバッチ送信する。
最初の0.5秒はジャイロ未取得フラグ付きで、成功履歴は録音停止後、成立後の失敗履歴は即時送る。

### ラベル付き6軸データ収集

デバッグ版 `0.0.57` 以降では、ホストコマンド `0x04` が現行判定と独立した6秒収集を開始する。
終了時に `0x36–0x38` を `result=3` で送り、加速度XYZ・ジャイロXYZを約40 Hzで保存する。
収集モード自体は現行ジェスチャーの閾値や判定結果を変更しない。

まず推奨12試行の一覧を出し、同じ `session-dir` に1試行ずつ保存する。

```bash
cd mac_client
venv/bin/python gesture_dataset_collector.py --list-plan \
  --session-dir output/gesture_dataset_YYYYMMDD_HHMMSS

# 例: 右手・自然速度の正例を1回
venv/bin/python gesture_dataset_collector.py \
  --hand right --motion positive --speed natural \
  --session-dir output/gesture_dataset_YYYYMMDD_HHMMSS

# 12試行後
MPLBACKEND=Agg venv/bin/python gesture_dataset_analyzer.py \
  output/gesture_dataset_YYYYMMDD_HHMMSS
```

内訳は左右それぞれ、正例3回（自然・遅い・速い）と負例3回（挙上のみ・回転のみ・
日常動作）の合計12回。各試行はCSV、6軸PNG、ラベルJSONを生成する。解析は特徴量CSV、
左右を色分けした重ね合わせPNG、0.1–0.5 gの一定水平加速度と低周波ノイズを加えた
感度CSV、追加収集フラグJSONを生成する。正例の手別ばらつきが30%を超える、または
正例最小と負例最大の分離が25%未満の場合だけ該当条件を左右各1回追加し、最大16回とする。
疑似横加速度は車・電車を再現するものではなく、一定オフセットが基線相対特徴へ及ぼす
感度確認に限定する。

対話試験は物理動作を伴うため、内容・回数・Ping音/`GO`を事前に案内し、準備完了を
確認してから1試行ずつ開始する。

## LED 状態

| 状態 | LED 色 / パターン |
|------|----------------|
| 起動直後 | 白（1 秒点灯） |
| BLE アドバタイジング中 | 消灯 |
| BLE 接続済み（待機中） | 消灯 |
| 単純なモーション検出中 | 消灯 |
| 録音中 | 赤（常時点灯） |

省発光のため、BLE のみで待機している間や単純な `motion_active` 検出中は LED を点灯しません。LED が点灯するのは録音ジェスチャーが成立して `is_recording` に入ったときだけです。リモート未接続でも録音ジェスチャー成立後は赤点灯を維持し、停止ジェスチャーで消灯します。赤消灯は `is_recording == false` と一致し、意図的に録音継続のまま LED だけ消す経路はない。
