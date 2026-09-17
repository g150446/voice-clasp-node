# HarnessNode / nordic-main ファームウェア運用ガイド

`harness-node` リポジトリで現在メインとなっている、XIAO nRF52840 Sense 向けの BLE 音声ファームウェアです。
**既定はシングルタップ録音**（ホスト承認）。手首ジェスチャーは実行時スイッチ（RX `0x07`）で
opt-in したときだけ自律 start/stop する（FW `0.0.95+`）。

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
| LED 方針 | 未接続アドバタイズ中は青の遅い点滅。接続待機は消灯。録音中のみ赤 |
| 署名バージョン（目安） | `0.0.102+`（primary idle 30–50 ms、supervision 8 s、WDT 12 s。未接続 3 分／接続実績ありは 30 分で SYSTEMOFF） |
| OTA 手順 | [`ota_update_notes.md`](ota_update_notes.md) |
| 録音操作 | 既定: シングルタップ（ホスト RX）。ジェスチャーは RX `0x07=1` かつ通常モード時のみ |

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
  /path/to/harness-node/nordic-main \
  --build-dir /path/to/harness-node/nordic-main/build
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
| `0x01` | 録音開始。既に録音中なら停止→再開始（ホストの状態ずれ救済、`0.0.97+`） |
| `0x00` | 録音停止 |
| `0x50 [reg] [val]` | IMUレジスタ書き込み（タップ関連のみ許可。`0xD2`で応答） |
| `0x51 [reg]` | IMUレジスタ読み出し（`0xD2`で応答） |

`0x50` / `0x51` はタップ調整用の診断コマンド。書き込みは `CTRL1_XL` `CTRL6_C`
`CTRL10_C` `TAP_CFG` `TAP_THS_6D` `INT_DUR2` `WAKE_UP_THS` `MD1_CFG` に限定され、
それ以外は `-EPERM` で拒否する。OTAなしで閾値・軸・Duration を実機で振れるので、
タップ系の調査はまずこれを使う（`mac_client/tap_monitor.py --write 0x59:0x04` など）。

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
| `0x01` | `recording_start` | なし | 録音開始（ホスト RX またはジェスチャー検出 ON 時） |
| `0x02` | `recording_stop` | なし | 録音停止 |
| `0x10` | `motion_active` | x, y, z f32 LE（各 4 byte） | モーション検出開始、xyz 加速度値 |
| `0x11` | `motion_settled` | x, y, z f32 LE + elapsed_ms u32 + avg/peak_speed/distance f32 LE（計 28 bytes） | モーション静定、詳細メトリクス |
| `0x12` | `double_tap` | なし | リストバンド表側から基板面へ垂直に行ったダブルタップ |
| `0x14` | `single_tap` | なし | 同上のシングルタップ（間隔窓を超えた単独衝撃） |
| `0x39` | `gesture_capture` | enabled u8 | IMU 軌跡収集スイッチ ack（既定 OFF） |
| `0x3A` | `gesture_detect` | enabled u8 | ジェスチャー検出スイッチ ack（既定 OFF、`0.0.95+`） |
| `0x40` | `operation_mode` | effective mode u8 + pending mode u8 | Android設定モードの同期状態（0=通常、1=運転、0xff=pendingなし）。要求受理時と適用時の2回、**全接続へ**送る |
| `0x20` | `sleep_enter` | なし | ライトスリープ移行（10 秒無動作） |
| `0x21` | `sleep_wake` | なし | ライトスリープ復帰（モーション検出） |
| `0x30` | `gesture_diag` | stage/reason u8 + value1/2/3 f32 LE | USBなしのジェスチャー内部診断（検出 ON 時のみ） |
| `0xD0` | `tap_diag` | read_ret i8 + CTRL1_XL/CTRL6_C/TAP_CFG/TAP_THS_6D/INT_DUR2/WAKE_UP_THS/MD1_CFG/INT1_CTRL u8 + nonzero_count u16 LE + int1_level i8 | タップ関連レジスタのスナップショット（接続確立時に1回。任意タイミングの個別読み出しは RX `0x51` → `0xD2`） |
| `0xD1` | `tap_src_raw` | TAP_SRC u8 + read_ret i8 | 判定に使った生の `TAP_SRC`。タップ1回につき1パケット |
| `0xD2` | `imu_reg_ack` | reg u8 + 読み戻し値 u8 + ret i8 | RX `0x50` / `0x51` への応答 |

---

## ジェスチャー検出アルゴリズム

**FW `0.0.95+`:** ジェスチャー状態機械は `gesture_detect_enabled`（RX `[0x07,e]`、
TX `0x3A` ack）が ON **かつ** 通常モードのときだけ動く。既定 OFF＝タップのみ。
フラグは RAM のみで、接続時 greeting でも `0x3A` を送る。IMU 軌跡収集（`0x06`）とは独立。

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

### 省電力（FW `0.0.102+`）

接続中の消費は **BLE 間隔** が支配的。次いで、2 台目の枠が空いている間ずっと回っている**アドバタイズ**（`0.0.102` で低速化）。未接続の長時間放置は **SYSTEMOFF**（`0.0.98+`、`0.0.102` で猶予を 2 段化）。IMU ポールを 25→50 ms にするライトスリープは効果が小さい。

#### ライトスリープ

10 秒間 `motion_active` でなく録音中でもない場合、IMU ポーリング間隔だけ落とす。BLE 接続は維持。

| 状態 | IMU ポーリング | BLE 送信 |
|------|-------------|---------|
| 通常（アクティブ） | 25 ms | — |
| ライトスリープ移行時 | → 50 ms | `0x20 sleep_enter` |
| ライトスリープ中 | 50 ms | — |
| ライトスリープ復帰時 | → 25 ms | `0x21 sleep_wake` |

録音停止後もタイマーはリセットされ、即座には入らない。

#### 接続中の BLE 間隔

録音していない primary は **30–50 ms**。録音中だけ 7.5–15 ms。secondary は 100–150 ms。接続直後も fast は要求しない。supervision timeout と GAP PPCP は全セット **8 s**。Zephyr 既定 PPCP の 420 ms を使うと Android がリンクを落とす。

| 条件 | primary | secondary |
|------|---------|-----------|
| 非録音（接続直後含む） | 30–50 ms | 100–150 ms |
| 録音中 | 7.5–15 ms | 100–150 ms |

録音開始で fast に戻す。アイドル中のタップ通知は最大約 50 ms 遅れる。idle 中に間隔を再更新しない（タップ後 10 秒の fast 復帰は `0.0.99` まで。Android 切断の原因になった）。

#### 接続中のアドバタイズ間隔（`0.0.102+`）

`MAX_CONNS=2` なので、1 台だけ繋がっている間も 2 台目の枠を狙って
アドバタイズが回り続ける。`0.0.100` まではこれが素の `BT_LE_ADV_CONN`＝
**100–150 ms のまま永久に**走っていた。

| 接続数 | アドバタイズ |
|--------|-------------|
| 0 | 100–150 ms（青の遅い点滅） |
| 1、接続数が変わってから 30 秒以内 | 100–150 ms（Mac を 2 台目として繋ぐための窓） |
| 1、それ以降 | **1–1.2 s** |
| 2 | 停止 |

**落とし穴**: `BT_LE_ADV_CONN` は `BT_LE_ADV_OPT_ONE_TIME` を含まないので
`BT_ADV_PERSIST` が立ち、ホストが LE Connection Complete の中で
`bt_le_adv_resume()` を呼ぶ。その後 `bt_le_adv_start()` を呼んでも
**間隔を再設定せずに `-EALREADY`** を返す（`host/adv.c`）。
必ず `bt_le_adv_stop()` してから start し直すこと。

#### 未接続 SYSTEMOFF（`0.0.102` で猶予を 2 段化）

接続 0 が続き、かつ USB（VBUS）が無いとき、アドバタイズを止めて nRF SYSTEMOFF に入る。

| 状況 | 猶予 |
|------|------|
| コールドブート後／SYSTEMOFF 復帰後、一度も接続がない | 3 分 |
| 一度でも接続した後に落ちた | **30 分** |
| 電池 15% 未満 | 常に 3 分 |

`0.0.98`〜`0.0.100` は一律 3 分だった。Android は数秒で再接続してくるが、
Doze・圏外・アプリ再起動を挟むと 3 分は足りず、**一過性のリンク断が
「ダブルタップするまでデバイスが消えたまま」に変質していた**。

- IMU は**シングル／ダブルどちらのタップでも** INT1 に出す（`0.0.102+`。
  `0.0.100` まではダブルのみ）。誤起床しても 3 分アドバタイズして寝直すだけ
- 起床はリセット相当。通常ブートのあとアドバタイズと青点滅を再開する
- USB 給電中は入らない（デバッグ用）
- SYSTEMOFF 中の床は IMU accel（タップ用 416 Hz）が支配的。nRF 本体は µA

#### アイドル切断の真因とウォッチドッグ（`0.0.102` で修正）

`0.0.98`〜`0.0.100` で「接続したまま録音せず放置すると勝手に切断され、
そのままディープスリープで見つからなくなる」症状が出た。BLE の切断ではなく
**ウォッチドッグリセット**だった。

1. `bt_gatt_notify()` は ATT PDU を **`K_FOREVER`** で確保する
   （`zephyr/subsys/bluetooth/host/att.c`。タイムアウトが付くのは
   `ATT_RESPONSE` と `ATT_CONFIRMATION` だけ）
2. バッファは `CONFIG_BT_ATT_TX_COUNT` の共有プール。解放にはコントローラへの
   送出＝ACL クレジットが要り、クレジットは相手が LL ACK を返して初めて戻る
3. 相手が黙るとプールが尽き、次の `bt_gatt_notify()` が永久に返らない
4. `wdt_feed()` はメインループの 1 箇所だけ。そのメインループ自身が
   ライトスリープ通知（10 秒無モーションで必ず通る）、バッテリ BAS 通知（60 秒周期）、
   モーション／タップ通知、軌跡ダンプで `bt_gatt_notify()` を呼ぶ

`0.0.98`/`0.0.99` は supervision が 4 s で **5 s のウォッチドッグより短かった**ため、
先にコントローラがリンクを切ってバッファが解放され、餌やりが間に合っていた。
`0.0.100` が 8 s に上げたことで順序が逆転し、**必ずウォッチドッグが先に発火する**
ようになった。リセットは相手からは切断と区別できず、再起動後 3 分誰も繋がないので
SYSTEMOFF に落ちる。

`0.0.102` の対策：

- **`WDT_TIMEOUT_MS` 5000 → 12000。不変条件は「ウォッチドッグ > supervision timeout」**
- 制御系の通知を `notify_one()` 経由にし、専用セマフォ（`CTRL_NOTIFY_IN_FLIGHT=4`）を
  20 ms で取れなければ**捨てる**。犬に餌をやるスレッドは絶対に止めない
- `CONFIG_BT_ATT_TX_COUNT` 8 → 12（音声 6 ＋ 制御 4 ＋ MCUmgr 用 2）

#### 起動レポート `0xD3`（`0.0.103+`）— USB なしで原因を判別する

ウォッチドッグリセットと純粋なリンク断は、**相手からは区別できない**（どちらも
HCI `0x08` / Android `status=8`）。区別にはシリアルが要るが、装着運用中はまさに
その USB が無い。

Android アプリは**未知の TX イベントを hex のまま logcat に出す**ので、FW 側から
起動情報を流せば**アプリを一切変更せずに** logcat で読める。接続ごとに greeting の
先頭で 1 回送る。

```
[00][55][D3][cause u32 LE][crash_valid][stage][prev_up_s u16][hb u16][ndrop u16]
```

| フィールド | 意味 |
|---|---|
| `cause` | `hwinfo_get_reset_cause()` の値。`0x01`=PIN、`0x02`=SOFTWARE、`0x10`=WATCHDOG、`0x80`=LOW_POWER_WAKE（SYSTEMOFF 復帰） |
| `crash_valid` | 1 なら以下が前回の実行の値（`.noinit` のクラッシュボックスが生き残った） |
| `stage` | 停止時のメインループ位置 1=motion 2=tap 3=greet 4=tapdiag 5=sleepnfy 6=battery |
| `prev_up_s` | 死んだ実行の uptime 秒 |
| `hb` | 死んだ実行の最後のハートビート番号 |
| `ndrop` | 死んだ実行で捨てた制御通知の数 |

読み方：

- 再接続時のレポートが**前回と同じ `cause` で `crash_valid=0`** → FW は再起動していない。
  **リンクだけが落ちた**（RF / 圏外）
- `cause` に `0x10`（WATCHDOG）が立ち `crash_valid=1` → **ウォッチドッグリセット**。
  `stage` がどこでブロックしたかを示す
- `cause=0x80` → SYSTEMOFF から復帰した（タップで起こされた）

logcat では `TX event 0xD3 (00 55 D3 ...)` として現れる。

#### 診断ログ（`0.0.102+`）

シリアルだけで「リンクが落ちた」と「デバイスがリセットした」を判別できる。

| 行 | 意味 |
|----|------|
| `>>> HB #n up=... conns=... c0=間隔/latency/timeout` | 接続中 30 秒ごと。`#n` の直後に `#1` が来たら**リセット** |
| `>>> DISCONN idx=.. reason=0x.. NAME age=.. params=..(negotiated\|never-updated) ndrop=..` | 切断の記号化した理由。`never-updated` なら相手が接続パラメータ要求を全部黙殺している |
| `>>> STALL n ms at stage=..(名前)` | メインループが 500 ms 以上ブロックした場所 |
| `>>> CRASHBOX stage=..(名前) up=.. hb=..` | 起動時。直前のウォッチドッグ／ソフトリセットでどこにいたか（`.noinit` 保持） |
| `>>> Adv tier=... conns=..` | アドバタイズ間隔の切替 |

reason の読み方：`0x08 CONN_TIMEOUT` は本物の supervision timeout、
`0x13 REMOTE_USER_TERM` は**相手（Android アプリ）が切った**、
`0x22`/`0x3B` は接続パラメータ要求が原因。

`CONFIG_LOG_MODE_IMMEDIATE=y`（`0.0.102+`）。`prj.conf` に
`CONFIG_LOG_MODE_DEFERRED=n` と書いてあったが、**LOG_MODE は Kconfig choice なので
メンバに `=n` を書いても無視され**、実際は deferred のままだった。
別スレッドが 1 秒周期で吐く方式では、ハング直前の 1 行がリングに残ったまま失われる。

### シングル／ダブルタップ

LSM6DS3TR-C のハードウェア判定を使用し、部品面を皮膚側にした装着状態で
リストバンド表側から基板面へ垂直に行うタップを Z 軸で検出する。

| パラメータ | 既定（FW `0.0.97+`） | 備考 |
|------------|---------------------|------|
| 閾値 `TAP_THS_6D` | `0x11`（≈ **1.0625 g**） | FS=±2 g 時 `THS` は 0–31 の 32 段階（1 LSB ≈ 0.0625 g）。小さいほど敏感 |
| 2 打最大間隔 Duration | ≈ 308 ms（`INT_DUR2=0x4a`） | |
| 分類待ち | 350 ms（`TAP_SRC_SETTLE_MS`） | Duration 閉じ後に `TAP_SRC` を1回読む |
| クールダウン | 700 ms | single / double 別 |

実行時調整は RX `0x50` で `TAP_THS_6D`（`0x59`）などへ書き込み可（再起動で既定に戻る）。
OTA なしの試し書き例: `mac_client/tap_monitor.py --write 0x59:0x12`。

| モード | single (`0x14`) | double (`0x12`) |
|--------|-----------------|-----------------|
| 通常 | **notify-only**（録音はホスト RX）。待機中の開始ジェスチャー候補は double 時に破棄 | 通知のみ（録音不変）。開始ジェスチャー候補を破棄し 3 s re-arm |
| 運転 | 同上（ジェスチャー無効） | 通知のみ（録音不変） |

録音 start/stop はホストが RX `0x01` / `0x00` で指示する（パススルー中の single はページ送り専用で RX を送らない）。
手首ジェスチャーによる録音は、検出スイッチ ON かつ通常モードのときだけ Node 自律。

Android は RX characteristic に:

- `[0x05, mode]`（`mode=0` 通常、`mode=1` 運転）→ ack `0x40`
- `[0x07, e]`（ジェスチャー検出、既定 OFF）→ ack `0x3A`
- `[0x06, e]`（IMU 軌跡収集、既定 OFF）→ ack `0x39`

録音中の運転モード変更は pending として保持し、録音停止後に適用する。
ライトスリープ中は復帰し、タップが掌上静止開始ジェスチャーとして
扱われないよう、待機中の開始候補を破棄する（検出 ON かつ通常モード）。

`0x40` の送信タイミング（`0.0.88+`）:

- **要求を受理した時点**と**実際に適用した時点**の2回送る。受理時のackがないと、
  録音中に保留された切替をホストが観測できない（`apply_pending_operation_mode()` は
  録音中に早期returnするため、適用まで何も飛ばなかった）。
- `notify_all_conns()` で**全接続へ**送る。Mac Handy が primary を握っている間も
  Android がモードを追えるようにするため（`0.0.87` までは primary 1本にしか飛ばず、
  secondary の Android には届かなかった）。
- 接続確立時は primary / secondary のどちらでも現在のモードを1回送る。ただし送信は
  `ble_connected()` ではなく **メインループが `bt_gatt_is_subscribed()` を見て**行う。
  詳細は下の「接続時通知は購読完了を待つ」を参照。

#### ハードウェア割り込み構成

この機能は、25 / 50 ms ごとに取得する加速度値からCPUがタップ波形を分類する
ソフトウェア検出ではない。LSM6DS3TR-C 内蔵のtap/double-tapエンジンと
**INT1ハードウェア割り込み**を使用する。

```
リストバンド表側への衝撃
  → LSM6DS3TR-C内蔵判定（Z軸、閾値・Shock・Quiet・Duration）
  → 1打目の時点で IMU INT1 をassert（MD1_CFGにsingleも割り当てているため）
  → XIAO P0.11のGPIO割り込み／メインループでのINT1レベル監視
  → TAP_SRCには触れずに 350 ms 待つ（TAP_SRC_SETTLE_MS）
  → HWのDurationが閉じた後に TAP_SRC を1回だけ読む
  → TAP_IA|DOUBLE|Z なら double、それ以外は single
  → BLE [0x00, 0x55, 0x12] または [0x00, 0x55, 0x14]
```

- 基板DTSの `irq-gpios = <&gpio0 11 GPIO_ACTIVE_HIGH>` により、IMU INT1は
  nRF52840のP0.11へ接続される。
- `INT1_CTRL` の加速度・ジャイロdata-ready割り込みを無効化し、`MD1_CFG` の
  single-tap と double-tap を INT1 へ割り当てる。INT1に載るのはタップだけなので、
  **INT1のassertは常に「打撃があった」ことを意味する**。
- NCS v2.9.2のLSM6DSLドライバはこのセンサーの`SENSOR_TRIG_DOUBLE_TAP`を
  実装していない。そのためtap関連レジスタをI2Cで直接設定し、ドライバの
  `SENSOR_TRIG_DATA_READY`用INT1配送経路をGPIO通知手段として再利用する。
  実際の割り込み源はdata-readyではなく、IMUが判定したタップである。
- 同ドライバは初期化時に `CTRL6_C.XL_HM_MODE` を無条件でセットし、加速度計を
  high-performance モードから外す。ST のタップ手順（AN5040）はこれがONである前提
  なので、`configure_tap_detection()` で明示的にクリアする。
- ドライバの `lsm6dsl_thread_cb()` は INT1 がラッチでhighのまま
  `GPIO_INT_EDGE_TO_ACTIVE` を再武装し、`lsm6dsl_trigger_set` と違ってピンレベルを
  再確認しない。立ち上がりエッジを取りこぼすことがあるため、ファームウェアは
  **エッジではなくINT1のレベル**もメインループで見る。
- 初期化に失敗した場合はタップ検出だけを無効化してログへ理由を出し、既存の
  IMUポーリング、録音ジェスチャー、BLE接続は継続する。

#### TAP_SRC を1打目で読んではいけない（0.0.76〜0.0.86 回帰の真因）

`TAP_CFG` は LIR（latched interrupt request）を有効にしている。この状態で
**Duration の窓が閉じる前に `TAP_SRC` を読むと、ラッチが解除されて進行中の
ダブルタップ判定が中断される**。

`0.0.76` が `MD1_CFG` に `INT1_SINGLE_TAP`(BIT6) を追加したことで、INT1は
1打目の時点でassertされるようになった。従来どおりINT1を見て即座に`TAP_SRC`を
読むと、上記のとおり2打目の判定が壊れる。実測では、ダブルタップが
`TAP_SRC=0x21`（`SINGLE|Z`、`TAP_IA`は立たない）2発に分解され、
**`0x12` は一度も送信されなくなった**。`0.0.55`（double専用、`MD1_CFG=0x08`）が
7/7 PASS していたのは、INT1がダブル確定後にしか上がらず、読み出しが必ず
事後だったからである。

対策は「INT1が上がってから `TAP_SRC_SETTLE_MS`（350 ms＝HW Duration 308 ms＋
メインループ25 ms＋余裕）待ってから1回だけ読む」。窓が閉じた後なので、
レジスタは最終判定を持っている。

実測した `TAP_SRC` の値（FW `0.0.85`〜`0.0.87`、Z軸・0.5 g）:

| 操作 | 350 ms後の `TAP_SRC` | 判定 |
|------|---------------------|------|
| ダブルタップ | `0x51` / `0x59`（`TAP_IA\|DOUBLE\|Z`） | double → `0x12` |
| シングルタップ | `0x00` | single → `0x14` |

このセンサーは `SINGLE_DOUBLE_TAP=1` のとき **`TAP_IA` をダブル確定時にしか
立てない**。単打では窓が閉じた時点で `TAP_SRC` が `0x00` までクリアされるため、
シングルを `TAP_IA` でマッチさせることはできない。INT1にタップ以外が載っていない
ことを利用し、**「INT1が上がった、かつダブルが確定しなかった」＝シングル**と
推定する。

過去に修正した派生バグも記録しておく（いずれも実在したが、上記が本体）:

- **クールダウン共有（`0.0.76`〜`0.0.80`）**: single と double が1つの
  タイムスタンプを共有していたため、仮にsingleが先に出ると700 ms間doubleが
  無言で捨てられた。`0.0.81` で `tap_last_single_ms` / `tap_last_double_ms` に分離。
  確定タップはすべてsingleのクールダウンを刻むが、doubleのクールダウンを
  刻めるのはdoubleだけ。クールダウンで捨てたタップは必ず printk する。
- **毎ループの無条件 `TAP_SRC` ポーリング（`0.0.80`〜`0.0.83`、未リリース）**:
  ドライバのエッジ取りこぼし対策として入れたものだが、25 ms ごとにラッチを
  解除するため上記の中断を最悪化させていた。INT1レベル監視＋遅延読み出しに置換。

#### タップ系の診断

USBシリアルなしで実機を追えるよう、タップ経路はBLEへ計装済み。

- `0xD0`（接続確立時に1回、`0.0.88+`。実際の送信は購読完了後、`0.0.90+`）: `CTRL1_XL` `CTRL6_C` `TAP_CFG` `TAP_THS_6D` `INT_DUR2`
  `WAKE_UP_THS` `MD1_CFG` `INT1_CTRL` の実値、`TAP_SRC`読み出しの戻り値、
  非ゼロ検出回数、INT1のレベル。レジスタが期待どおりかを一目で確認できる。
- `0xD1`: 判定に使った生の `TAP_SRC`。どの軸が立ったか、`TAP_IA` / `DOUBLE` /
  `SINGLE` のどれが立ったかがそのまま読める。
- RX `0x50` / `0x51`: OTAなしで閾値・軸・Durationを実機で振る。個別レジスタは
  `0x51` → `0xD2` で任意のタイミングで読めるので、`0xD0` の定期配信は不要。

`0.0.87` までは `0xD0` を2秒周期で全接続へ流していた（真因特定用の計装）。
帯域は無視できるが、Android の `BleManager` が高レート系以外を全部hexダンプするため
logcat が2秒ごとに埋まり、Node も2秒ごとにI2Cを8回叩いていた。`0.0.88` で
接続時1回に変更。

#### 接続時通知は購読完了を待つ（`0.0.90` で修正）

**`ble_connected()` で `bt_gatt_notify()` を呼んでも、その接続した本人には届かない。**
接続確立からクライアントがTX characteristicのCCCDを書くまで約1秒あり、その間は
購読者がいないので通知は捨てられる。実測（Android、FW `0.0.88`）:

```
17:33:49.854  GATT connected            <- ble_connected() はここ
17:33:51.072  TX notifications enabled  <- CCCD有効化は1.2秒後
```

`0.0.88` ではこのせいで接続時の `0xD0` と `0x40` がAndroid自身には1件も届かず、
**別のクライアントが後から接続したときだけ**（既に購読済みなので）現れるという
紛らわしい挙動になっていた。`0.0.87` までは `0xD0` の2秒周期タイマーが再送していたため
表面化せず、`send_operation_mode_status()` の方は元から同じ欠陥を抱えていた。

**`BT_GATT_CCC` の `cfg_changed` はこの用途に使えない**（`0.0.89` で試して失敗した）。
Zephyr の `gatt_ccc_changed()` は全クライアントの**集約値が変化したときだけ**
コールバックを呼ぶ:

```c
for (i = 0; i < ARRAY_SIZE(ccc->cfg); i++)
    if (ccc->cfg[i].value > value) value = ccc->cfg[i].value;
if (value != ccc->value) { ccc->value = value; ccc->cfg_changed(attr, value); }
```

つまり既に誰かが購読している状態で2台目が購読しても発火しない。接続ごとの処理には
使えない。

`0.0.90` の実装: `ble_connected()` は `conn_needs_greeting[slot]` を立てるだけにし、
メインループ（25 ms周期）の `send_pending_greetings()` が
**接続ごとに `bt_gatt_is_subscribed(conn, &audio_svc.attrs[2], BT_GATT_CCC_NOTIFY)`**
を見て、trueになった接続にだけ送る。時間待ちではないので取りこぼしも早すぎる送信もない。

**接続直後に何かを通知したくなったら、`ble_connected()` でもCCCDコールバックでもなく、
`bt_gatt_is_subscribed()` を接続ごとにポーリングすること。**

正常時の期待値: `CTRL1_XL=0x60` `CTRL6_C=0x00` `TAP_CFG=0x83` `TAP_THS_6D=0x11`
`INT_DUR2=0x4a` `WAKE_UP_THS=0x80` `MD1_CFG=0x48` `INT1_CTRL=0x00`。

Mac から `mac_client/tap_monitor.py` で観測する（macOSのBluetooth権限の都合で
Terminal.appから実行すること。詳細は [`ota_update_notes.md`](ota_update_notes.md)）。

```bash
mac_client/venv/bin/python3 mac_client/tap_monitor.py --duration 60
mac_client/venv/bin/python3 mac_client/tap_monitor.py --duration 45 --write 0x59:0x04
```

#### 実機確認結果

2026-08-24、ファームウェア`0.0.55`をXIAO nRF52840 SenseへOTA適用し、
部品面を皮膚側にして装着した状態で確認した（当時は double のみ有効）。

| 試験 | 結果 |
|------|------|
| リストバンド表側をダブルタップ | 3 / 3 検出 |
| 表側を1回だけタップ | 2 / 2 誤検出なし（single 未実装時） |
| タップせず腕を自然に上げ下げ | 2 / 2 誤検出なし |
| 合計 | **7 / 7 PASS** |

2026-08-31、ファームウェア`0.0.87`で single / double 同時運用を再確認した
（Mac直結の `tap_monitor.py` で観測、その後 Android アプリでも受信を確認）。

| 試験 | 結果 |
|------|------|
| シングルタップ | 5 / 5 → `0x14` のみ（`TAP_SRC=0x00`） |
| ダブルタップ | 5 / 5 → `0x12` のみ（`TAP_SRC=0x51`/`0x59`） |
| 取り違え・誤検出 | 0 件 |
| 合計 | **10 / 10 PASS** |

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
| `SLEEP_IDLE_TIMEOUT_MS` | 10000 ms | ライトスリープ移行、および接続中 BLE 間隔を落とすまでの無タップ時間 |
| `SLEEP_POLL_INTERVAL_MS` | 50 ms | ライトスリープ中の IMU ポーリング間隔 |
| `ADV_NO_CONN_SLEEP_MS` | 180000 ms | 未接続アドバタイズ後に SYSTEMOFF するまでの時間 |
| `ADV_LED_ON_MS` / `ADV_LED_OFF_MS` | 200 / 1800 ms | 未接続アドバタイズ中の青点滅 |

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
  --json-output /private/tmp/harness-node-volar-sequence.json
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

## ジェスチャ履歴と IMU 軌跡の収集

`0.0.91` から `GESTURE_DEBUG_HISTORY` は**既定 1**（本番ビルドに含む）で、
実際の収集は**実行時スイッチ**で切り替える。以前のように収集のたびに
デバッグビルドを OTA する必要はない。

| | |
|---|---|
| RX | `CMD_SET_GESTURE_CAPTURE`（`0x06`）＋ `0x00`/`0x01` |
| TX ack | `EVT_GESTURE_CAPTURE`（`0x39`）、`notify_all_conns()` で全接続へ |
| 既定 | **OFF**。RAM のみに保持し、リセットで OFF に戻る |

**RAM にしか持たないので、接続時 greeting でも現在値を送る。**
アプリ側も接続のたびに再送する。UI にはアプリの意図ではなく Node の報告値を出すこと。

収集 OFF のときは `gesture_trajectory_clear()` が `active` を立てないため、
push / finish / flush がすべて no-op になる。分岐1つ以外のコストは掛からない。

痩せたビルドが要るときは `-DEXTRA_CFLAGS=-DGESTURE_DEBUG_HISTORY=0`。
このとき ack は**常に OFF を報告する**（`gesture_capture_effective()`）ので、
ホストが「オンにしたつもり」で空振りすることはない。

`0x36` の `period_ms` は `MOTION_SAMPLE_INTERVAL_MS`（25）を載せるだけの**公称値**で、
実サンプル間隔とは一致しない。IMUポーリングはライトスリープ中 `SLEEP_POLL_INTERVAL_MS`
（50 ms）に落ちるため（`imu_poll_ms`）、静止状態では約51 ms間隔になる。
解析側は各サンプルの `t_ms` を使い、一定レートを仮定してはいけない。

**軌跡 flush は2スレッドから呼ばれる。** `process_motion_sample()`（main スレッド）と
録音停止処理（`audio_thread()`）の両方が `flush_gesture_trajectory()` に到達する。
`flush_trajectory_batch()` はチャンク間で `k_msleep()` するため、`0.0.91` では
同一ウィンドウが2セッションとして交互に送信された。`0.0.92` で `atomic_cas` の
実行中ガードを入れた。ここに新しい呼び出し元を足すときは同じ罠に注意すること。

**録音成功時の flush は `send_event_packet(0x02)` の後**（`main.c` の停止処理）。
ホストは停止イベントを受けた時点ではまだ軌跡を持っていない。約250ms後に届く。

コスト: 静的バッファ 22.5 KB（`gesture_trajectory` / `gesture_host_collection`
各 10752 B、`gesture_history` 1536 B）。`0.0.91` の実測で RAM 171 KB / 256 KB、
署名イメージ 253799 B（slot 上限 335872 B）。

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
| BLE アドバタイジング中（接続 0） | 青 200 ms ON / 1800 ms OFF |
| BLE 接続済み（待機中） | 消灯 |
| 単純なモーション検出中 | 消灯 |
| 録音中 | 赤（常時点灯） |
| 未接続 3 分後 SYSTEMOFF | 消灯 |

接続 0 のアドバタイズ中だけ青を遅い点滅にします。接続が 1 本でもある間（2 本目待ちのアドバタイズ含む）は消灯です。録音中は赤。赤消灯は `is_recording == false` と一致し、意図的に録音継続のまま LED だけ消す経路はない。
