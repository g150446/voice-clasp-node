# M5StickC Plus2 / HarnessNode-Plus2

ESP-IDF ファーム: `stickc_plus2/`  
BLE 名: **`HarnessNode-Plus2`**  
Audio Service UUID: XIAO `HarnessNode` と同じ  
`00000001/0002/0003-0000-1000-8000-00805f9b34fb`

Handy / Android は `HarnessNode` 接頭辞で接続可能。

---

## ハードウェア

| 項目 | 値 |
|------|-----|
| SoC | ESP32-PICO-V3-02（**esp32s3 ではない**） |
| Flash / PSRAM | 8 MB / 2 MB |
| USB-UART | CH9102 → `/dev/cu.usbserial-…` |
| Mic | SPM1423 **PDM** CLK=**G0**, DIN=**G34** |
| BtnA | **G37** active-low（single / double / **long ≥1 s**） |
| POWER_HOLD | **G4 = HIGH 必須**（起動直後に保持。deep sleep 中も RTC hold で維持） |
| LCD | ST7789V2 135×240（MOSI=G15 CLK=G13 DC=G14 RST=G12 CS=G5 BL=G27） |
| Status LED | G19（IR と共有、active-high） |
| 電池 | 200 mAh（ADC G38、v1 では BAS 未実装） |

---

## ビルド / USB フラッシュ

```bash
source ~/esp/esp-idf/export.sh   # 例: ~/esp/esp-idf
cd harness-node

# ボード切替（デフォルトも stickc_plus2）
idf.py -DHN_BOARD=stickc_plus2 set-target esp32
idf.py -DHN_BOARD=stickc_plus2 build
idf.py -DHN_BOARD=stickc_plus2 -p /dev/cu.usbserial-XXXX flash monitor
```

| 変数 / ファイル | 意味 |
|-----------------|------|
| `-DHN_BOARD=stickc_plus2` | コンポーネント `stickc_plus2/` をリンク |
| `sdkconfig.defaults.esp32` | 共通 NimBLE / UART |
| `sdkconfig.defaults.stickc_plus2` | Flash 8MB、PSRAM、dual OTA、BLE 名 |
| `stickc_plus2/VERSION` | `esp_app_desc.version`（単一の真実。ビルド前に読む） |

**注意:** 以前 `esp32s3` でビルドした `build/` がある場合は退避してから `set-target esp32` する。

CH9102 が見えないとき: USB 抜き → 電源長押し（緑 LED）→ 再挿し。

---

## BLE OTA（nordic と同じ `ota_updater.py`）

パーティション: `ota_0` / `ota_1`（`stickc_plus2/partitions_ota.csv`）  
SMP UUID: nordic MCUmgr と同じ（`8D53DC1D-…` / `DA2E7828-…`）

```bash
# 1. VERSION を上げる
# 2. パッケージ
./stickc_plus2/build_and_package_ota.sh
# → stickc_plus2/ota_update.bin

# 3. macOS は Terminal で（Bluetooth TCC）
python3 mac_client/ota_updater.py --device HarnessNode-Plus2 stickc_plus2/ota_update.bin
```

- dual-OTA レイアウトに変えた**直後**は USB full flash が必要（bootloader + table + app）。
- OTA 中は Handy / Android など他 BLE クライアントを切る。
- 成功: `OTA verified: uploaded image is active and confirmed in slot 0`
- 起動約 3 秒後に auto-confirm（それ以前のクラッシュは rollback）
- 成果物 `ota_update.bin` は git 管理外（ビルドで生成）

詳細・nRF との共通注意: [`ota_update_notes.md`](ota_update_notes.md)

---

## プロトコル（v1）

| 入力 | 動作 |
|------|------|
| BtnA **single** / serial `c` | BLE 接続中に録音トグル。TX `0x14` のあと `0x01` or `0x02` |
| Voice Clasp が RX `0x08 0x01` を送った後 | **notify-only**。録音はトグルせず TX `0x14` のみ |
| BtnA **double** / serial `d` | TX `0x12` のみ（録音しない） |
| BtnA **≥1 s 長押し** / serial `l` | deep sleep 入眠（LCD 消灯・BLE 切断）。**同じ BtnA を押すだけで起床**（電源ボタン不要） |
| RX `0x01` / `0x00` | ホスト start / stop |
| RX `0x05 mode` | モード（0=NORMAL, 1=DRIVING）。TX `0x40` |
| RX `0x08 0/1` | タップ権限。0=ローカルトグル（既定・Handy）、1=notify-only。切断で 0 に戻る |
| 音声 | 16 kHz mono PCM `[seq][0xAA][i16 LE…]`、ペイロード最大 200 B |

### 起動表示 / スリープ

**LCD 状態**（中央寄せ・黒地、`stickc_plus2/display.c`）:

| 表示 | 色 | 条件 |
|------|-----|------|
| `not connected` | 白 | BLE 未接続 |
| `connected` | 青 | BLE 接続中・非録音 |
| `recording` | 赤 | 録音中（または開始要求中） |

**入眠**（`enter_deep_sleep()`, `stickc_plus2/main.c`）:

1. 録音停止 → LED off
2. BLE 切断（`ble_gap_terminate`）+ 広告停止。**NimBLE は落とさない**
3. **BtnA 解放待ち（無期限）** — 押したままだと ext0 が即発火して寝られない
4. `POWER_HOLD`(G4) を digital → RTC へグリッチなしで受け渡し、`rtc_gpio_hold_en()`
5. LCD 消灯 + G12 / G27 を pulldown で待避（`display_prepare_deep_sleep()`）
6. `esp_sleep_enable_ext0_wakeup(G37, 0)` → `esp_deep_sleep_start()`

**起床**: `rst:0x5 (DEEPSLEEP_RESET)` の cold boot 相当。`app_main` 冒頭で HOLD を
digital に戻してから hold 解除 → BtnA 解放待ち → 通常初期化 → 状態表示を再描画。

- **mic は触らない**。`i2s_channel_disable()` は `i2s_channel_read` 中のミューテックス
  待ちでブロックしうるうえ、deep sleep でどうせリセットされる。
- **MPU6886**: 本 FW では未使用のため sleep 入出でも触らない（常時通電のまま）。
- **完全電源オフ**（HOLD=0）は未使用。ほぼ 0 mA だが復帰は電源ボタン BtnC ≥2 s（公式仕様）。

#### 落とし穴: `nimble_port_stop()` は必ず panic する

`nimble_port_run()` は stop イベントを処理すると **return する**
（`components/bt/host/nimble/nimble/porting/nimble/src/nimble_port.c`）。
ESP-IDF の `vPortTaskWrapper` はタスク関数の return を `abort()` にするため、
ホストタスクが即クラッシュする:

```
E FreeRTOS: FreeRTOS Task "nimble_host" should not return, Aborting now!
```

deep sleep は BT コントローラごと電源を落とすので**解体は不要**。切断と広告停止だけ行う。
`ble_host_task()` は保険として `nimble_port_run()` の後に
`nimble_port_freertos_deinit()` を呼んでおくこと。

#### 落とし穴: `POWER_HOLD`(G4) を一瞬でも LOW にすると電源が落ちる

G4 LOW ＝ Plus2 の電源オフ操作そのもの。数 µs でも駆動すると board ごと落ちる。

- **入眠**: `rtc_gpio_set_level(1)` → `rtc_gpio_set_direction(OUTPUT_ONLY)` →
  `rtc_gpio_init()` → `rtc_gpio_hold_en()`。**この順序が必須**。
  ESP-IDF の例どおり `init → direction → level` にすると、RTC 出力データが初期値 0 の
  まま driver が有効化され G4 が LOW に駆動される。
- **起床**: `power_hold_on()`（digital で HIGH 出力）→ **その後に**
  `rtc_gpio_hold_dis()`。`gpio_config()` は内部で `rtc_gpio_deinit()` を呼んだ直後に
  output を有効化するので、hold を先に外すと `GPIO_OUT`=0 のまま G4 が LOW になる。
- `power_hold_on()` 自体も `gpio_config()` の前に `gpio_set_level(1)` で
  出力ラッチを先込めしてある。単体で呼んでも LOW を出さない。

#### 落とし穴: USB 給電中は HOLD のバグが見えない

USB 接続中は 5 V 側から給電され続けるため、G4 が LOW に落ちても board は動き続ける。
**HOLD 周りの検証は必ず USB を抜いて電池駆動で行うこと。**
シリアルログが取れないので LCD の点灯／消灯で判定する。

#### 落とし穴: 起床直後に寝直す

起こすのも寝かせるのも同じ BtnA なので、起床時に指が残っていると長押し判定に化ける。

- `wait_button_a_release()` は**タイムアウトなし**で離すまで待つ。
- `button_task` は開始時に押されているボタンを無視する（`press_active = false` で開始）。
  `press_start_tick = 0` のまま開始すると `now - 0 >= 1000 ms` が即成立して寝直す。

#### 落とし穴: LCD の色は数値から推測しない（必ず `p` で実測）

このパネルは **RGB565 の素直な読みと一致しない**。実測（シリアル `p` で
`0xF800` / `0x07E0` / `0x001F` / `0xFFFF` の 4 帯を上から描画）した結果:

| 送る値 | 実際に見える色 |
|--------|----------------|
| `0xF800` | **赤** |
| `0x07E0` | **青** |
| `0x001F` | 緑 |
| `0xFFFF` | 白 |

したがって `display.c` の `COLOR_BLUE` は `0x07E0`、`COLOR_RED` は `0xF800`。
**数値だけ見て「間違っている」と直すと逆に壊れる。** `rgb_ele_order` を
変えるとこの対応表も変わるので、いじったら必ず `p` で測り直すこと。

（経緯: 当初 `rgb_ele_order = RGB` で `connected`=`0x07E0` は緑、
`recording`=`0xF800` は青に見えていた。BGR に変えたところ赤は正しくなったが
緑と青の対応が入れ替わり、推測では追えないことが判明した。）

#### 落とし穴: LCD は 1 タスクからしか触らない

`display_set_status()` は **button_task / audio_stream_task / uart_task /
NimBLE host task / app_main** の 5 文脈から呼ばれる。1 回の再描画は
`esp_lcd_panel_draw_bitmap()` を約 240 回叩く重い処理なので、素で呼ぶと:

- **SPI デバイスを 2 タスクが同時に使って固まる**。`fill_rect()` が黒で塗った直後に
  止まるため、**画面は真っ黒・呼び出したタスクは永久ブロック**になる。
  button_task が巻き込まれると BtnA が完全に無反応になる。
- audio_stream_task が録音開始／停止のたびに数十 ms 止まり、**音声が途切れて
  STT が文字起こしできない**。

対策として `display.c` に描画専用タスクを置き、パネルへのアクセスは全て
mutex (`DISPLAY_LOCK`) 配下に集約した。`display_set_status()` は
深さ 1 の queue に `xQueueOverwrite()` するだけで**呼び出し側は一切ブロックしない**。
同じ状態への再描画は `s_drawn` で握り潰す。

再現テスト（BLE 不要）: シリアル `r` / `s` を 0.2 s 間隔で連打する。
`r` は uart_task から、録音開始は audio_stream_task から再描画を呼ぶため衝突する。
**修正前は 1 サイクル目で固まる**（最後のログが `Stop recording requested` で停止）。

#### 落とし穴: 入眠を中止したら LCD のパッドを戻す

`display_prepare_deep_sleep()` は G27 / G12 を **RTC mux に移す**ので、以降
`gpio_set_level()` 系の digital 書き込みはパッドに届かない。`ext0` の設定失敗などで
入眠を取りやめる場合は `display_resume()` で `rtc_gpio_deinit()` して digital に
戻すこと。ここで `display_init()` を呼んでも `s_ready` により即 return するだけで、
**バックライトが二度と点かない**（ファームは動いているのに画面だけ真っ黒になる）。

#### 落とし穴: G12（LCD RST）は MTDI ストラップ

deep sleep 中は Hi-Z になる。起床リセット時に HIGH で読まれると VDD_SDIO が 1.8 V に
なり内蔵 flash が起動しない。`display_prepare_deep_sleep()` で pulldown して LOW に固定する。
BL の G27 も同様に pulldown（浮かせるとバックライトが薄く光る）。

#### 検証手順

1. **USB 接続 + `idf.py monitor`**: シリアル `l` を送る →
   `entering deep sleep` → `deep sleep now (wake on BtnA)` の後**ログが止まり、
   panic も reset も出ない**こと。panic が出るなら NimBLE の解体が残っている。
2. BtnA を押す → `rst:0x5 (DEEPSLEEP_RESET)` と
   `Woke from deep sleep (BtnA); waiting for release` が出て通常起動すること。
3. 実機で BtnA を 1 秒以上長押し → `Button A: Long-press -> sleep`。
   単発クリック（録音トグル）が混ざらないこと。
4. **USB を抜いて電池駆動**（本命）: 長押し → LCD 消灯 → BtnA をもう一度押す →
   **電源ボタンを使わずに** LCD が復帰すること。ここが通れば HOLD のグリッチは無い。

### 消費電力（接続中・非録音を含む）

| 状態 | 目安 | 備考 |
|------|------|------|
| 未接続（LCD ON + BLE 広告） | 数十 mA | BL + RF が支配 |
| **BLE 接続・非録音** | 広告より低いが RF は生きる | **idle 接続間隔 50–100 ms**（下記） |
| BLE 接続・録音中 | 高い | mic ON + fast 7.5–15 ms + PCM notify |
| deep sleep（本実装） | **数十 µA〜1 mA 未満（要実測）** | ESP32 単体 ~10 µA だが Plus2 は AXP 無しで LDO / BM8563 / MPU6886 等が常時通電 |
| HOLD=0 完全オフ | ほぼ 0 | 今回の UX では不採用 |

実測は電池経路に直列電流計（0.1 mA 分解能以上）を入れ、**USB 非接続**で
(1) 未接続アイドル (2) 接続・非録音 (3) 録音 (4) deep sleep を比較すること。
USB 給電中の値は電池駆動と一致しない。

すでにゲート済み（接続中でも非録音ならオフ）:

- マイク / PDM（`set_microphone_enabled`）
- 録音 LED
- BLE 広告（接続中は出さない。1 本リンク前提）

#### 動的 BLE 接続間隔（省電力の本命）

Mac / Handy 等に繋いだまま長時間放置すると、**接続間隔が短いほどラジオが頻繁に起きる**。
16 kHz PCM には fast が必須だが、非録音では不要。`stickc_plus2/main.c` で切替する。

| モード | interval | latency | いつ |
|--------|----------|---------|------|
| **idle** | 40–80（**50–100 ms**） | 0 | 接続後・非録音 |
| **fast** | 6–12（**7.5–15 ms**） | 0 | 録音開始〜停止 |

ライフサイクル（`request_conn_params` / `deferred_idle_conn_params`）:

```
CONNECT ──(+500 ms timer)──► idle
     │
     ├─ recording start ──► fast（即時）
     │         │
     │         └─ PCM 前: interval ≤15 ms 待ち（最大 500 ms、タイムアウト後も録音続行）
     │
     └─ stop / cancel ──► idle
```

- **latency は常に 0**（START/STOP が 1 interval 以上遅れない）
- 接続直後タイマーは **idle**。録音中・開始要求中なら idle を出さない
- mic DC settle と fast ネゴは重ねる。`EVENT_RECORDING_STARTED` はゲート後
- **GAP コールバック内で `ble_gap_update_params` しない**（無視されやすい / ロック問題）

検証ログの目安:

1. 接続のみ → `conn param update requested (idle: 50.00-100.00 ms)` →
   `Conn params (conn update): interval=…` が数十 ms 帯
2. シリアル `r` → fast 要求 → `fast gate ok` または `fast gate timeout` →
   10 s 後 `s` で TX summary の nomem がほぼ 0、pkt/s が ~160 以上
3. 停止後 → 再び idle

ホスト（特に一部 Mac）がパラメータを拒否する場合がある。そのときは
ゲートがタイムアウトしても録音は止めない（品質は従来どおりホスト依存）。

LCD バックライト消灯・自動 deep sleep は未実装（接続表示を優先）。

### 音声帯域（Handy へのストリーミング）

16 kHz mono の生 PCM を `[seq][0xAA][i16 LE…]` で送る。必要帯域は
**32000 B/s**、パケットは 200 B なので **160 pkt/s** 必要。
実際は 256 サンプル読むごとに 100/100/56 に割るため 187 pkt/s 出る。

**ADPCM は使っていない。** `adpcm.c` は Plus2 にも `nordic-main` にも入っているが
どちらも未使用で、Handy は生 PCM を期待している。圧縮すれば帯域は 1/4 になるが
**Handy 側の対応が必要**なので勝手に変えないこと。

#### 落とし穴: 接続パラメータを要求しないと音声が 3 割落ちる

セントラルの既定接続間隔（多くは 30 ms）だと **131 pkt/s / 26.3 kB/s** しか出ず、
必要な 160 pkt/s に届かない。溢れた分は `ble_hs_mbuf_from_flat()` が NULL を返して
**送信前に捨てられる**（ログに `os_memblock_get failed`）。BLE の通知自体は
一度も失敗しないので `notify failed` は 0 のまま、音声だけが虫食いになり
STT が通らない。

録音中は上記 **fast（7.5–15 ms）** が必須。`nordic-main` も録音向けに同じ帯を使う
（`conn_param_work_handler`。ただし nordic は primary を常時 fast のまま — Plus2 のみ
非録音 idle 切替を入れている）。

実測（10 秒録音、シリアル `r` → `s`、fast 常時要求時代）:

| | 要求なし | 7.5–15 ms 要求 + プール 48 |
|---|---|---|
| 送信 | 1314 | **1863** |
| 欠落 `nomem` | **555（29.7%）** | **6（0.3%）** |
| `os_memblock_get failed` | 14 回 | **0 回** |
| 確定した接続間隔 | 未計測 | **15.00 ms** |

#### 落とし穴: `sdkconfig.defaults` は既存の `sdkconfig` を上書きしない

`CONFIG_BT_NIMBLE_MSYS_1_BLOCK_COUNT` を `sdkconfig.defaults.esp32` で
12 → 48 にしても、**生成済みの `sdkconfig`（git 管理外）が古い値を保持**していると
反映されない。`build/config/sdkconfig.h` を grep して実際の値を確認すること。
反映されていなければ `sdkconfig` も直すか消して `reconfigure` する。

#### 落とし穴: `pdMS_TO_TICKS(2)` は 0 tick に丸められる

`CONFIG_FREERTOS_HZ=100`（1 tick = 10 ms）なので `pdMS_TO_TICKS(1)` も
`pdMS_TO_TICKS(2)` も **0**。通知リトライの「短いバックオフ」は
まったく待っていなかった。10 ms 未満を待ちたいなら tick を上げるしかない。

#### 送信状況の計測方法

録音停止時に `TX summary` が出る。BLE 接続中にシリアル `r` → 10 秒 → `s` で取れる
（喋る必要はない。測るのは送信レート）。

```
TX summary: sent=1863 failed=0 nomem=6 over 9991 ms (187 pkt/s, ...)
Conn params (at stop): interval=15.00 ms latency=0 timeout=4000 ms
```

- `failed` … `ble_gattc_notify_custom()` が失敗した数
- `nomem` … mbuf を確保できず捨てた数 ← **帯域不足はここに出る**
- `B/s` 表示は全パケットを 200 B と仮定した概算で、末尾の端数パケットぶん過大。
  正確に見るなら `sent` と `nomem` の比を使うこと。

接続相手の BD アドレス（`Peer (connect): …`）と切断理由
（`BLE disconnected (reason=0x…)`）も出る。Stick が `connected` のままなのに
アプリ側が未接続、という状況では**誰がリンクを掴んでいるか**がこれで分かる。

#### 落とし穴: NimBLE の notify ログが音声を殺す

NimBLE は既定で `GATT procedure initiated: notify` を **INFO** で出す。
音声通知ごとに出るので短い録音 1 回で **約 600 行・55 kB**。`esp_log` は
コンソール UART へ同期書き込みするため、115200 baud では **4.8 秒ぶん**の
送信待ちが BLE ホストタスクに乗り、音声ストリームが枯れて STT が失敗する。

`app_main` で `esp_log_level_set("NimBLE", ESP_LOG_WARN)` を入れて抑止している。
警告・エラーは残る。切断テストのログで `procedure initiated: notify` が
0 件であることを確認できる。

### マイク処理（Handy STT 用）

内蔵 SPM1423（`pin_ws = G0`, `pin_data_in = G34`, M5Unified `M5Unified.cpp:2143`）。

**確定した設定**（`MIC_PDM_DEFAULT`, `stickc_plus2/main.c`）:

| 項目 | 値 |
|---|---|
| クロック経路 | `MIC_CLK_IDF`（`i2s_channel_init_pdm_rx_mode()` に任せる） |
| `dn_sample_mode` | `I2S_PDM_DSR_16S`（PDM CLK 2.048 MHz） |
| `slot_mask` | **`I2S_PDM_SLOT_LEFT`** |
| `slot_mode` | `I2S_SLOT_MODE_MONO` |
| gain | ×4（`MIC_GAIN_DEFAULT`、シリアル `g` で変更可） |

#### 真因は slot_mask だった（M5Unified の既定は移植できない）

M5Unified の `mic_config_t` 既定は `input_only_right`（`Mic_Class.hpp`）で、
Plus2 でも上書きしていない。しかしこれを IDF のクロック経路にそのまま持ち込むと
**マイクが載っていない側のスロットを読む**ことになる。

2026-09-02 のシリアル `m` スイープ実測（9 構成、発話しながら各 0.3 s 計測）:

| # | 構成 | verdict | rms | zcr |
|---|---|---|---|---|
| 1 | IDF DSR_8S **RIGHT** | `CONST`（-30935 固定） | 30935 | 0 Hz |
| 2 | IDF DSR_16S **RIGHT** | `NOISE` | 21414 | 3362 Hz |
| **3** | IDF DSR_8S **LEFT** | **`SIGNAL`** | 1108 | 472 Hz |
| **4** | IDF DSR_16S **LEFT** | **`SIGNAL`** | 1157 | 431 Hz |
| 5 | IDF DSR_8S STEREO | **L=`SIGNAL`** / R=`CONST` | 1022 | 234 Hz |
| 6 | IDF DSR_16S STEREO | **L=`SIGNAL`** / R=`NOISE` | 1312 | 293 Hz |
| 7 | M5-raw DSR_16S **RIGHT** | `SIGNAL` | 924 | 382 Hz |
| 8 | M5-raw DSR_16S STEREO | L=`CONST` / **R=`SIGNAL`** | 1231 | 378 Hz |
| 9 | M5-raw DSR_8S RIGHT | `SIGNAL`（ただし出力 32 kHz） | 970 | 336 Hz |

**IDF クロックでは LEFT、M5 生レジスタクロックでは RIGHT** に音がある。
`bclk_div` が違う（IDF は 8 以上を強制 / M5 は `div_m = 2`）ため WS の位相が
1 スロットずれるのが理由。M5Unified の `input_only_right` は
**M5 自身のクロックとセットでのみ正しく**、IDF クロックには成り立たない。

`#9`（M5 クロック + DSR_8S）は M5 のクロックが DSR128 前提で固定されているため
デシメーションだけが半分になり、出力が 32 kHz になる（同じ 0.3 s で n が 2 倍）。使わない。

`DSR_8S`（1.024 MHz）ではなく `DSR_16S`（2.048 MHz）を選ぶのは、SPM1423 の
normal mode 下限が 1.0 MHz で `DSR_8S` がその縁に乗るため。

#### 判定は peak ではなくゼロ交差レートで

16 kHz なのでナイキストは 8 kHz。白色雑音の zcr は約 4000 Hz に張り付く:

| verdict | 条件 | 意味 |
|---|---|---|
| `CONST` | `zcr = 0` / distinct 一桁 | PDM DOUT がアイドル。無音のスロットを読んでいる |
| `NOISE` | `zcr > 2500 Hz` | 広帯域雑音。マイクが載っていないスロット、またはデコード不成立 |
| `SIGNAL` | `zcr` 100–2500 Hz かつ distinct ≥ 100 | 実音声。目安は 300–1500 Hz |
| `WEAK` | 上記以外 | レベル不足 |

**注意**: この verdict は DC 除去**前**の生ストリームに対して計算する。
DC オフセットが振幅より大きいと符号が反転しなくなり、正常な音声でも
`zcr ≈ 0` → `WEAK` と出る（実測例: `mean=-1925 rms=2398 zcr=1Hz` だが
処理後は `peak=8972 rms=453 clip=0%` で正常）。スイープでは DC が小さいので有効、
録音時のログでは `Mic proc` 側と `head` ダンプを併せて見ること。

#### 実績値

| | 張り付き | rms | peak | zcr | distinct |
|---|---|---|---|---|---|
| XIAO `HarnessNode`（基準） | 0 % | 560–692 | — | 800–1004 Hz | — |
| Plus2 `RIGHT`（不良） | 84–90 % | 30995 | 32768 | 3810–3909 Hz | 2783 |
| **Plus2 `LEFT` + gain 4** | **0.00 %** | 1345 | 13236 | 1723 Hz | 3057 |

シリアル側: `Mic proc peak=8972 rms=453 gain=4 clip=0%`、
`Mic raw head: 4023 4033 4000 4017 3955 3987 3898 …`（滑らかな波形）。

#### PDM クロックの 2 系統

`mic_pdm_cfg_t.clk_path` で切り替える。既定は `MIC_CLK_IDF`。

| | `MIC_CLK_IDF` | `MIC_CLK_M5` |
|---|---|---|
| 実装 | `i2s_channel_reconfig_pdm_rx_clock()` | `apply_m5_mic_clock()`（生レジスタ） |
| bclk_div | **8 以上を強制**（`i2s_pdm.c` `I2S_PDM_RX_BCLK_DIV_MIN`） | `div_m = 2` |
| mclk | 16.384 MHz（160 MHz / 9.7656） | 4.096 MHz（160 MHz / 39.0625） |
| 音のあるスロット | **LEFT** | **RIGHT** |

M5Unified は `Mic_Class.cpp:317` で `sample_rate_hz = 48000; // dummy setting` と
書いたうえで生レジスタで上書きしており、**IDF のクロック計算を使っていない**。
`MIC_CLK_M5` はその ESP32-classic (I2S HW v1) 分岐（`Mic_Class.cpp:528-548`）の移植で、
`atom_echo_s3r/main.c:621` の同名関数は I2S HW v2 用なのでレジスタ配置が異なる。
`i2s_channel_enable()` の後に適用する必要がある（`set_microphone_enabled()` が自動で呼ぶ）。

両方 `SIGNAL` になるが、生レジスタを叩かず IDF 更新にも強い `MIC_CLK_IDF` を既定にしている。

#### ファーム処理

- 起動時 DC 収束待ち（`|x-offset|` 閾値、上限 ~400 ms）のあと `recording_started`
  （実測 63 ms で収束）
- DC IIR 除去 + デジタル gain ×4 + ハードクリップ
- STEREO 取り込み時は `mic_stereo_pick` のスロットだけを詰めてから DSP/BLE へ
- 録音開始後 ~1 s の serial:
  - `Mic raw n=… min=… max=… mean=… rms=… zcr=…Hz distinct=… CONST|NOISE|WEAK|SIGNAL`
  - `Mic raw head: …`（生サンプル先頭 32 個。波形か全振幅ジャンプかが一目で分かる）
  - `Mic proc peak=… rms=… gain=… clip=…%`
- **`Mic raw` / `Mic proc` は BLE 接続中しか出ない。** 録音ループは
  `audio_conn_handle == BLE_HS_CONN_HANDLE_NONE` のとき読み出しごとスキップするので、
  シリアル `r` だけでは統計が出ない。BLE クライアントを繋いだ状態で測ること。

#### 落とし穴: `i2s_channel_read()` のタイムアウトは「ミリ秒」

```c
i2s_channel_read(h, buf, bytes, &read, MIC_READ_TIMEOUT_MS);   /* 正 */
i2s_channel_read(h, buf, bytes, &read, pdMS_TO_TICKS(50));     /* 誤 */
```

第 5 引数は **ミリ秒**で、IDF が内部で `pdMS_TO_TICKS()` を適用する
（`esp_driver_i2s/i2s_common.c`）。二重に変換すると
`pdMS_TO_TICKS(50)` = 5 → 内部で `pdMS_TO_TICKS(5)` = **0 tick**
（`CONFIG_FREERTOS_HZ=100`）となり、ノンブロッキング読み出しになって
DMA にデータが揃っていなければ即 `ESP_ERR_TIMEOUT` を返す。

実害（0.1.2 で発生）:

- 診断スイープが全構成 `no samples` で 140 ms で終わる（測定が一切走らない）。
- **録音開始時の DC 収束ループが即 `break`** し、`mic_dc_offset = 0` のまま録音開始。

read の失敗は握りつぶさずログすること（`Sweep #n read failed:` / `DC settle read failed:`）。

#### 落とし穴: `VERSION` は configure 時にしか読まれない

`CMakeLists.txt` は `file(READ stickc_plus2/VERSION)` で読むため、`VERSION` を
書き換えただけの `idf.py build` では `esp_app_desc.version` が古いままになる
（OTA のバージョン確認が通らない）。`stickc_plus2/build_and_package_ota.sh` は
`PROJECT_VER` の export と `reconfigure` を行うので、**OTA は必ずこのスクリプト経由**で
パッケージすること。USB フラッシュ前に手で `idf.py build` する場合は
`idf.py -DHN_BOARD=stickc_plus2 reconfigure` を挟む。

#### 診断スイープ

シリアル `m` で 9 構成（クロック経路 × DSR × slot）を各 0.5 s 測る。約 5 秒間
**途切れず**喋り続けること（黙った構成は `CONST` と誤判定される）。
STEREO 構成は L / R を別行で出すので、**どちらのスロットが実マイクか**が直接分かる。
ゲインはシリアル `g` で 1→2→4→8→16 と巡回でき、較正に再フラッシュは不要。

BLE 側の確認には `mac_client/esp32_controller.py` を使うが、
`find_device()` はスキャンで**最初に見つかった既知デバイス**を掴むため、
XIAO `HarnessNode` が同時に起動しているとそちらに繋がる。
`VoiceBridgeClient(device_names=("HarnessNode-Plus2",))` で固定するか、
XIAO の電源を切ること。

---

## シリアル（115200）

| キー | 動作 |
|------|------|
| `r` | 録音開始 |
| `s` | 録音停止 |
| `c` / `1` | single-click 相当 |
| `d` / `2` | double-click 相当 |
| `l` | long-press 相当（deep sleep） |
| `m` | PDM 設定スイープ 9 構成（録音中は不可） |
| `g` | mic gain 巡回 1→2→4→8→16 |
| `p` | LCD カラーバー（色順の実測用、下記） |
| `h` | ヘルプ |

---

## ボード切替

Plus SE（4 MB / AXP192）は別コンポーネント。**`build/` を共有しない**:

```bash
idf.py -DHN_BOARD=stickc_plus_se -B build-plus_se set-target esp32 build
```

詳細: [`stickc_plus_se_guide.md`](stickc_plus_se_guide.md)

```bash
idf.py -DHN_BOARD=atom_echo_s3r set-target esp32s3 build
```

Echo は別コンポーネント `atom_echo_s3r/`（保留中のデスクノード）。  
詳細のたたき台: [`atom_echo_s3r_guide.md`](atom_echo_s3r_guide.md)

---

## ファイル一覧

| パス | 役割 |
|------|------|
| `stickc_plus2/main.c` | HOLD, PDM, BtnA, long-press sleep, NimBLE audio, LED |
| `sdkconfig.defaults.esp32` | 共通 NimBLE（**生成済み `sdkconfig` を上書きしない**） |
| `sdkconfig.defaults.stickc_plus2` | 8MB / PSRAM / BLE 名 |
| `stickc_plus2/display.c` | ST7789 状態表示 / deep sleep 前のパッド待避 |
| `stickc_plus2/smp_ota.c` | MCUmgr 互換 SMP OTA |
| `stickc_plus2/partitions_ota.csv` | dual OTA |
| `stickc_plus2/build_and_package_ota.sh` | OTA bin 生成 |
| `stickc_plus2/VERSION` | アプリバージョン |
| `CMakeLists.txt` | `HN_BOARD` で EXTRA_COMPONENT_DIRS 切替 |
