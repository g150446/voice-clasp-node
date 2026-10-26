# M5 Atom Echo S3R / HarnessNode-Echo（reserved）

ESP-IDF コンポーネント: `atom_echo_s3r/`  
BLE 名（現行コード）: **`HarnessNode-Echo`**  
UUID: HarnessNode と同じ `…0001/0002/0003`

StickC Plus2 が主線の M5 デスク／リストノード。Echo は **保留**（S3 ターゲット・ES8311 パス）。

## ハードウェア（要約）

| 項目 | 値 |
|------|-----|
| SoC | ESP32-S3-PICO 系 |
| Mic | ES8311 + I2S STD（M5Unified 相当ピン） |
| Button A | GPIO 41 |
| I2C SCL | **GPIO 0**（ストラップピン → monitor は `--no-reset` 推奨） |

## ビルド

```bash
source ~/esp/esp-idf/export.sh
cd voice-clasp-node
idf.py -DHN_BOARD=atom_echo_s3r set-target esp32s3
idf.py -DHN_BOARD=atom_echo_s3r build
idf.py -DHN_BOARD=atom_echo_s3r -p PORT flash
# monitor: idf.py -p PORT monitor --no-reset
```

## プロトコル

StickC Plus2 と同じ Handy 互換イベント:

- single click → 録音トグル + `0x14`
- double → `0x12` のみ
- PCM 16 kHz mono `[seq][0xAA]…`

## 既知の注意

- 実機が **ESP32-S3** として列挙されること（FTDI + ESP32-PICO-D4 は旧 Atom Echo）
- dual-OTA / SMP は Plus2 側で実装済み。Echo への横展開は未着手
