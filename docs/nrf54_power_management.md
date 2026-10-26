# nrf54-handy: pdm_imu_pwr 電源管理

## 1. pdm_imu_pwr の役割

`pdm_imu_pwr` は GPIO0.1 で制御する固定レギュレータ（`regulator-fixed`）。
名称のとおり PDM マイク（MSM261D3526HIC3DPM）と IMU（LSM6DS3TR-C）の両方に
電源を供給していると考えられる。

## 2. 問題点（変更前）

ボード DTS（`xiao_nrf54l15_sense.dts`）に `regulator-boot-on` が設定されていたため、
nrf54-handy では起動直後から `pdm_imu_pwr` が常時 ON。録音していない間も
マイク+IMU が通電し続けるため、マイク電源を録音時のみ ON にしている
nordic-main よりバッテリー消費が大きかった。

## 3. 制約: IMU とマイクを個別に電源制御できない

`pdm_imu_pwr` が IMU と PDM マイクを共用しているため、マイクだけを
録音停止時に切ることができない。IMU が必要な間は電源を維持しなければならない。

## 4. 解決方針: BLE 接続状態に連動させる

| 状態 | pdm_imu_pwr | 理由 |
|------|-------------|------|
| BLE 非接続中 | OFF | ジェスチャー録音はクライアントがいないと意味がない |
| BLE 接続中 | ON | IMU ジェスチャー検出 + PDM 録音が必要 |

実装:
- BLE 切断時: `regulator_disable()` → 電源 OFF
- BLE 接続時: `regulator_enable()` → IMU キャリブレーション（~0.6 s）→ ジェスチャー検出開始

### overlay での変更

`boards/xiao_nrf54l15_nrf54l15_cpuapp.overlay` に以下を追加し起動時 OFF にする:

```dts
&pdm_imu_pwr {
    /delete-property/ regulator-boot-on;
};
```

### `prj.conf` の変更

```kconfig
CONFIG_REGULATOR=y
```

### `src/main.c` の変更

- `pdm_imu_power_on()`: `regulator_enable()` + IMU キャリブレーション状態をリセット
- `pdm_imu_power_off()`: `regulator_disable()` + キャリブレーション状態をリセット
- `ble_connected()` 内で `pdm_imu_power_on()` を呼ぶ
- `ble_disconnected()` 内で `pdm_imu_power_off()` を呼ぶ

## 5. IMU 再キャリブレーションの注意

BLE 接続のたびに 25 サンプル × 25 ms ≈ **0.6 秒のキャリブレーション**が必要。
この間はジェスチャー検出が無効（`baseline_valid = false`）。
接続直後 0.6 秒は録音トリガーが反応しない点をユーザーに伝えること。

## 6. 動作確認

```bash
# ビルド & フラッシュ
bash voice-clasp-node/nrf54-handy/build_and_flash.sh

# シリアルモニタ
screen $(ls /dev/tty.usbmodem* | head -1) 115200
```

期待ログ:

| タイミング | 出力 |
|-----------|------|
| 起動時 | （なし、電源 OFF のまま） |
| BLE 接続時 | `pdm_imu_pwr ON — IMU recalibrating` → `Baseline ready: accel=(...)` |
| BLE 切断時 | `pdm_imu_pwr OFF` |

## 7. 将来の調査事項

`pdm_imu_pwr` が IMU を本当に給電しているかは基板回路に依存する。
実装後に `Baseline ready` ログが接続ごとに出れば IMU は正常動作している。
万一 IMU が動作しなければ `pdm_imu_pwr` はマイク専用の可能性があり、
その場合は「接続中は常時 ON、録音停止で OFF」という細かい制御に変更する。
