# M5StickC Plus SE / HarnessNode-PlusSE

ESP-IDF ファーム: `stickc_plus_se/`  
BLE 名: **`HarnessNode-PlusSE`**（現行 `VERSION` **0.1.9**）  
Audio Service UUID: XIAO `HarnessNode` / Plus2 と同じ  
`00000001/0002/0003-0000-1000-8000-00805f9b34fb`

Plus2 の音声・ボタン・LCD・SMP OTA を移植し、AXP192 の残量 LCD と BLE Battery Service を足した。Plus2 と **ピンも電源も Flash も違う**。Plus2 ファームを焼かないこと。

---

## ハードウェア

| 項目 | 値 |
|------|-----|
| SoC | ESP32-PICO-D4（**esp32s3 ではない**、Plus2 の V3-02 でもない） |
| Flash / PSRAM | **4 MB / なし** |
| USB-UART | FTDI → `/dev/cu.usbserial-…`（**115200**。460800 は失敗する） |
| Mic | SPM1423 **PDM** CLK=**G0**, DIN=**G34**（AXP GPIO0 LDO 給電） |
| BtnA | **G37** active-low（録音トグル／ADV／長押し睡眠） |
| BtnB | **G39** active-low（短押しで切断して ADV） |
| 電源 | **AXP192** I2C SDA=G21 SCL=G22（GPIO4 HOLD はない） |
| LCD | ST7789V2 135×240（MOSI=G15 CLK=G13 DC=**G23** RST=**G18** CS=G5 BL=**AXP LDO2**） |
| Status LED | **G10** active-low（録音中のみ点灯） |
| 電池 | 120 mAh、AXP ADC（0x78、1.1 mV/LSB） |
| IMU | **なし**（本 FW でも未使用） |

比較: [docs.m5stack.com StickC-Plus SE](https://docs.m5stack.com/en/core/StickC-Plus_SE)

---

## Handy / Android 接続

未接続は **deep sleep** か **`ADV`** だけ。`not connected` 画面は出さない。

| きっかけ | 次の状態 |
|----------|----------|
| 電池での電源投入 | deep sleep |
| USB（VBUS）起動 | `ADV` |
| BtnA で起床 | その一回で `ADV`（二度押し不要） |
| Handy 切断／音声購読解除 | すぐ切って `ADV` |
| OS だけ掴んで未購読 | 15 秒で蹴って deep sleep |
| BtnB 短押し / serial `a` | リンクを切って `ADV` |
| BtnA 長押し / serial `l` | deep sleep |

1. 起床または USB 起動で画面が **`ADV`**
2. Handy 設定で音声ソース **BLE** → **Scan** → `HarnessNode-PlusSE` を選ぶ
3. **Connect**
4. 画面が **`connected`**、状態の下に相手 MAC

`connected` は GAP 接続ではなく、**音声 TX Notify 購読**が付いたときだけ。OS が掴んだだけの接続は `LINK` のあと 15 秒で切断し睡眠する。

広告パケットにはローカル名 `HarnessNode-PlusSE` を入れる（UUID はスキャンレスポンス）。

---

## ビルド / USB フラッシュ

**必ず `-B build-plus_se`。** Plus2 の `build/`（8 MB + PSRAM）と混ぜない。sdkconfig は `build-plus_se/sdkconfig`。

```bash
source ~/esp/esp-idf/export.sh
cd voice-clasp-node

idf.py -DHN_BOARD=stickc_plus_se -B build-plus_se set-target esp32
idf.py -DHN_BOARD=stickc_plus_se -B build-plus_se build
python -m esptool --chip esp32 -p /dev/cu.usbserial-XXXX -b 115200 \
  --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 build-plus_se/bootloader/bootloader.bin \
  0x8000 build-plus_se/partition_table/partition-table.bin \
  0xf000 build-plus_se/ota_data_initial.bin \
  0x20000 build-plus_se/voice_bridge_ble.bin
```

| 変数 / ファイル | 意味 |
|-----------------|------|
| `-DHN_BOARD=stickc_plus_se` | コンポーネント `stickc_plus_se/` |
| `sdkconfig.defaults.stickc_plus_se` | Flash 4MB、PSRAM なし、dual OTA、BLE 名 |
| `stickc_plus_se/VERSION` | `esp_app_desc.version` |

---

## BLE OTA

広告中（`ADV`）であること。Handy は切る。

```bash
./stickc_plus_se/build_and_package_ota.sh
python3 mac_client/ota_updater.py --device HarnessNode-PlusSE stickc_plus_se/ota_update.bin
```

初回（パーティション）は USB flash。詳細は [`ota_update_notes.md`](ota_update_notes.md)。

---

## プロトコル（v1）

Plus2 と同じ音声 UUID / パケット。

| 入力 | 動作 |
|------|------|
| 未購読で BtnA **single** / serial `c` | 未接続なら `ADV`。GAP のみなら切って再広告 |
| Handy 接続中に BtnA **single** / serial `c` | 録音トグル。TX `0x14` のあと `0x01` or `0x02` |
| Voice Clasp が RX `0x08 0x01` を送った後 | **notify-only**。録音はトグルせず TX `0x14` のみ |
| BtnA **double** / serial `d` | TX `0x12` のみ |
| BtnB **short** / serial `a` | 切断して `ADV` |
| BtnA **≥1 s** / serial `l` | deep sleep。同じ BtnA で起床して `ADV` |
| RX `0x01` / `0x00` | ホスト start / stop |
| RX `0x05 mode` | モード（0=NORMAL, 1=DRIVING）。TX `0x40` |
| RX `0x08 0/1` | タップ権限。0=ローカルトグル（既定・Handy）、1=notify-only。切断で 0 に戻る |

### LCD

| 表示 | 色 | 条件 |
|------|-----|------|
| `ADV` | 白 | 広告中（未接続はこれか睡眠だけ） |
| `LINK` | 白 | GAP 接続済み・未購読（15 秒で蹴って睡眠） |
| `connected` | 青 | 音声 TX Notify 購読済み・非録音 |
| `recording` | 赤 | 録音中 |
| 上部アイコン + `N%` | 白（充電中は青、≤15% は赤） | AXP 残量、30 秒ごと |
| 状態の下の MAC | 白 | GAP 接続中の相手。OTA のみなら `OTA ` 接頭辞 |

色定数は Plus2 と同じ実測値。SE で違う場合は serial `p`。

### LED（G10）

本体左上の赤 LED。**G10 は active-low**（LOW で点灯）。0.1.5 以前は active-high として駆動していたため、コネクテッド／待機中も点灯した。

| 状態 | LED |
|------|-----|
| 起動 | 約 200 ms 点灯して消灯 |
| `ADV` / `LINK` / `connected` / 睡眠 | 消灯 |
| `recording`（`is_recording`） | 点灯 |
| 録音停止・開始キャンセル・切断・入眠 | 消灯 |

LCD バックライトと充電アイコン（上部、充電中は青）とは別。点灯は録音中だけ。

### バッテリー

- AXP192 0x78、LSB 1.1 mV。ADC 有効化後に非ゼロまで待つ。0 mV は未計測として捨て、0% にしない。`ADV` 開始時にも再読込
- nordic-main と同じ OCV LUT（4150 mV=100% … 3000 mV=0%）
- BLE BAS `0x180F` / `0x2A19` Read+Notify（Handy 購読後）
- `python3 mac_client/battery_check.py --device HarnessNode-PlusSE`
- serial `b`

USB 給電中は電圧が高め。充電中はアイコンが青。

### 入眠

1. 録音停止 → LED off → BLE 切断（NimBLE は落とさない）
2. BtnA 解放待ち
3. LCD 消灯、AXP `SetSleep`（DCDC1 のみ）
4. `esp_sleep_enable_ext0_wakeup(G37, 0)` → deep sleep

起床は cold boot。`axp192_init()` がレールを戻す。BtnA 起床はその押しで `ADV` を始める。

### シリアル（115200）

`r/s/c/d/a/l/m/g/f/p/h` に加え `b`=battery。`a` は BtnB 相当（切断して ADV）。

---

## マイク

初期値 **IDF クロック + LEFT + DSR_16S + gain ×4**。SE でスロットが違うなら serial `m`。

BGM 低音対策の **200 Hz 1次 HPF が既定 ON**（DC 除去 IIR ≈80 Hz の後段、gain の前段）。
serial `f` でトグル。状態は録音開始ログ（`Mic DC settle ... hpf=on/off`）と
1 秒統計（`Mic proc ... hpf=on/off`）に出る。

---

## ファイル一覧

| パス | 役割 |
|------|------|
| `stickc_plus_se/main.c` | AXP, PDM, BtnA/BtnB, ADV/睡眠, NimBLE audio, BAS |
| `stickc_plus_se/axp192.c` | AXP192 I2C |
| `stickc_plus_se/display.c` | ST7789 + 残量 + 相手 MAC |
| `stickc_plus_se/smp_ota.c` | MCUmgr 互換 SMP OTA |
| `stickc_plus_se/partitions_ota.csv` | dual OTA（4 MB に収まる） |
| `sdkconfig.defaults.stickc_plus_se` | 4MB / no PSRAM / BLE 名 |
| `stickc_plus_se/VERSION` | アプリバージョン |
