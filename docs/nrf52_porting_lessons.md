# nRF52840 移植で得られた知見

nrf54-motion（XIAO nRF54L15 Sense）を nRF52840 Sense に移植し、BLE OTA 対応を実装した際に判明した技術的な知見をまとめます。

---

## 1. NCS sysbuild + Adafruit UF2 ブートローダの共存

### 問題

NCS（Nordic Connect SDK）の sysbuild は Partition Manager (PM) を使い、アプリを VMA 0x0 にリンクする。Adafruit UF2 ブートローダは 0xF4000 にあり、アプリが 0x27000 から始まることを期待する。PM が VMA 0x0 のまま UF2 で 0x27000 に書き込むと、リセットベクタが 0x27000 未満を指すため起動時にハードフォルトする。

### 解決策

`pm_static.yml` で softdevice パーティション（0x0〜0x27000）を定義することで、PM が app を 0x27000 に配置する。

```yaml
softdevice:
  address: 0x0
  size: 0x27000
  region: flash_primary
app:
  address: 0x27000
  ...
```

さらに `CMakeLists.txt` で `find_package(Zephyr)` の**前**に PM_STATIC_YML_FILE を設定する必要がある：

```cmake
set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/pm_static.yml)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
```

**重要:** sysbuild モードでは、アプリディレクトリの `pm_static.yml` はデフォルトで無視される（`partition_manager.cmake` の `NOT SYSBUILD` 条件）。`PM_STATIC_YML_FILE` を CMakeLists.txt で明示的に指定することで sysbuild レベルの PM に渡される。

---

## 2. MCUboot をゼロでない番地に配置する（Adafruit UF2 との共存）

### 背景

標準的な NCS MCUboot は 0x0 に配置されるが、XIAO nRF52840 では 0x0〜0xF4000 が Adafruit ブートローダの管轄。MCUboot を 0x27000 に配置することで Adafruit が MCUboot に直接ジャンプする「ネスト構造」を実現する。

### 実装

`pm_static.yml` で MCUboot を 0x27000 に定義：

```yaml
mcuboot:
  address: 0x27000
  size: 0xc000        # 48 KB
  region: flash_primary
```

`slot0_partition` (DTS) の reg も 0x33000 から始まるように設定し、MCUboot がここの primary slot にジャンプする。

### UF2 での書き込み可否

Adafruit UF2 ブートローダは 0x1000〜0xF3FFF の範囲への書き込みを許可するため、MCUboot (0x27000) + アプリ (0x33200) の merged UF2 を初回プロビジョニングで書き込める。0x0 (MBR) は書き込み不可。

---

## 3. DTS partition ID と PM partition ID のミスマッチ（最重要）

### 問題

MCUmgr の `zephyr_img_mgmt.c` は `FIXED_PARTITION_ID(slot1_partition)` を使いスロット 1 のフラッシュエリアを特定する。`FIXED_PARTITION_ID` は DTS が自動割り当てする ID（アドレス順）を返す。一方、PM のカスタムフラッシュマップは PM の内部 ID を使う。

両者が一致しない場合：

- MCUmgr がアップロード先として誤ったフラッシュエリアを開く
- `img_mgmt_find_by_hash` が正しいスロットを見つけられない
- `Setting image test flag` が `rc=1` で失敗する

### 具体例（本プロジェクト）

| パーティション | DTS ID（アドレス順） | PM ID |
|----------------|---------------------|-------|
| slot0_partition@33000 | 2 | PM_MCUBOOT_PRIMARY_ID = **3** |
| slot1_partition@85000 | 3 | PM_MCUBOOT_SECONDARY_ID = **6** |

DTS slot1 の ID 3 が PM_MCUBOOT_PRIMARY (0x33000) を指してしまう。

### 解決策

オーバーレイにダミーパーティションを追加し、DTS の自動割り当て ID が PM ID と一致するよう順序を調整する：

```dts
&flash0 {
    partitions {
        // ID 0: sd_partition@0       (board DTS)
        // ID 1: code_partition@27000 (board DTS)
        partition@2f000 { reg = <0x2f000 0x200>; };    // ID 2 = PM_MCUBOOT_PAD_ID
        slot0_partition: partition@33000 { ... };        // ID 3 = PM_MCUBOOT_PRIMARY_ID ✓
        partition@33200 { reg = <0x33200 0x51e00>; };  // ID 4 = PM_APP_ID
        partition@34000 { reg = <0x34000 0x51000>; };  // ID 5 = PM_MCUBOOT_PRIMARY_APP_ID
        slot1_partition: partition@85000 { ... };        // ID 6 = PM_MCUBOOT_SECONDARY_ID ✓
        // ID 7: storage_partition@ec000 (board DTS) = PM_SETTINGS_STORAGE_ID
        // ID 8: boot_partition@f4000  (board DTS) = PM_ADAFRUIT_BOOT_ID
    };
};
```

同じオーバーレイを MCUboot のボード設定（`sysbuild/mcuboot/boards/`）にも適用する。

### 確認方法

ビルド後に生成された `devicetree_generated.h` で ID を確認：

```bash
grep "PARTITION_ID" build/nordic-main/zephyr/include/generated/zephyr/devicetree_generated.h \
  | grep "flash_0.*partition"
```

---

## 4. OTA バイナリのハッシュ衝突

### 問題

`img_mgmt_find_by_hash` はすべてのスロットをスキャンし、ハッシュが一致する最初のスロットを返す。OTA バイナリ（slot 1）のハッシュが稼働中のファームウェア（slot 0）と同一の場合、slot 0 が返され、アクティブスロットへの test フラグ設定が `IMG_MGMT_ERR_IMAGE_SETTING_TEST_TO_ACTIVE_DENIED` で失敗する。これが rc=1 (`MGMT_ERR_EUNKNOWN`) として返る。

### 解決策

OTA バイナリを毎回異なるバージョンでビルドする：

```
# prj.conf
CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="0.0.2+0"   # リリースごとにインクリメント
```

バージョンは MCUboot イメージヘッダに書き込まれ、バイナリの内容が変わるためハッシュも変わる。

---

## 5. NCS sysbuild モードにおける pm_static.yml の適用ルール

### 仕組み

sysbuild 使用時、`partition_manager.cmake` はアプリの `pm_static.yml` を見つけても **sysbuild モードでは無視**する（`if (NOT SYSBUILD)` 条件）。

sysbuild レベルの PM に渡すには `CMakeLists.txt` で明示指定が必要：

```cmake
set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/pm_static.yml)
```

また pm_static.yml では flash_primary 全体をギャップなくカバーする必要がある（PM は「動的パーティション用のギャップがちょうど 1 つ」を要求する）。

---

## 6. Adafruit UF2 ブートローダでの注意点

### 1200 baud トリガーは Zephyr では効かない

Arduino フレームワークでは USB CDC ACM ポートを 1200 baud でオープン→クローズするとブートローダに入る「magic baud」機能がある。Zephyr の USB CDC ACM ドライバはこの機能を実装していないため、この手では入れない。**代わりに下の GPREGRET 経路を使う**（`nordic-main 0.0.106+`）。ファーム側にコマンドが無い場合は物理的なダブルタップが唯一の手段。

### adafruit-nrfutil はシリアル DFU に使えない

`adafruit-nrfutil dfu serial` はアプリが DFU モード（ブートローダ実行中）である必要がある。Zephyr アプリは nRF5 SDK の DFU プロトコルを実装していないため、serial DFU は使えない。

### GPREGRET による DFU トリガー（`nordic-main 0.0.106+` で実装・実測済み）

`NRF_POWER->GPREGRET = 0x57` + `sys_reboot()` で Adafruit ブートローダの UF2 モードに入れる。
nordic-main では RX `0x60 0xB0 0x07` とシリアル `'b'` として実装済み
（`nordic_main_guide.md` の「UF2 ブートローダへ再起動する」）。
ブートローダ `0.6.2-12-g459adc9` / Seeed XIAO nRF52840 Sense での実測：

- **USB（VBUS）が必須。** 未接続だとブートローダは DFU モードに留まらずアプリへ素通りする。
  このとき `hwinfo` の reset cause は `SOFTWARE` のみで `WATCHDOG` は立たない
  ＝ ブートローダが待機して犬に殺されたのではなく、最初から素通りしている
- USB があればダブルタップ不要でマウントされ、**60 秒経っても維持される**。
  つまりブートローダは自分でウォッチドッグに給餌している
- nRF52840 の WDT は `WDT_TASKS_STOP` が無く**ソフトで停止できない**。止まるのは
  POR / ブラウンアウト / **ピンリセット** / SYSTEMOFF 復帰のみで、`sys_reboot()` は
  ソフトリセットなので WDT は armed のまま引き継がれる。ダブルタップ（ピンリセット）だと
  クリアされるという非対称性があるので、この経路はブートローダの給餌に依存している

---

## 7. nRF54L15 との主要な差分

| 項目 | nRF52840 Sense | nRF54L15 Sense |
|------|---------------|----------------|
| ボードターゲット | `xiao_ble/nrf52840/sense` | `xiao_nrf54l15/nrf54l15/cpuapp` |
| コンソール | USB CDC ACM（`usb_enable()` 必須） | UART (uart20) |
| I2C バス | `i2c0` | `i2c30` |
| MCUboot 配置 | 0x27000（Adafruit 内ネスト） | 0x0（標準） |
| 初回フラッシュ | UF2 ドライブ | pyocd (J-Link) |
| RRAMC/GRTC 設定 | 不要 | 必要 |
| OTA セカンダリ | 内部 flash 0x85000 | 内部 flash |
| slot サイズ | 328 KB | 324 KB |
| IMU DTS ノード | ボード DTS に定義済み（overlay で alias のみ） | overlay で `irq-gpios` も設定 |
| USB CDC ACM | ボード defconfig に USB_DEVICE_STACK=y あり | なし（UART のみ） |

---

## 8. USB CDC ACM コンソールの初期化

nRF52840 はボードデフォルトコンフィグで USB_DEVICE_STACK=y が設定され、chosen.zephyr,console が usb_cdc_acm_uart を指している。ただし、Zephyr の USB 自動初期化が完了する前に `LOG_INF` を呼ぶと出力が失われることがある。

```c
// main() の最初で明示的に USB を有効化
usb_enable(NULL);
k_msleep(500);   // ホスト側の USB 列挙を待つ

LOG_INF("起動ログ...");
```

---

## 9. MCUboot ビルドサイズの実測値

| 構成 | フラッシュ使用 | 割り当て |
|------|--------------|---------|
| MCUboot (nRF52840, SWAP_USING_MOVE) | 40 KB | 48 KB |
| アプリ（モーション+BLE+MCUmgr）| 217 KB | 328 KB |

MCUboot はコンソール・watchdog・USB を無効化すると 32 KB 未満に収まるが、余裕を持って 48 KB を確保しておくのが安全。
