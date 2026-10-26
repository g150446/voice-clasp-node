# XIAO nRF54L15 Sense PDM省電力検証

XIAO nRF54L15 Senseで、LSM6DS3TR-C加速度センサーを416 Hzで動かしたまま、
オンボードPDMマイクの消費電流をクロック停止によって下げられるか確認するための
単独ファームウェアです。既存の`nordic-main/`、`nrf54-handy/`は変更しません。

## 何を判定するテストか

IMUとマイクのVDDは`IMU&MIC_3V3`として共有されているため、IMUを動かしたまま
マイクの物理電源だけを切ることはできません。このテストで確認する「マイクOFF」は、
VDDを残したままPDMクロックを止め、マイクの消費電流がIMUのみの基準値まで戻る
**実質的なスリープ**を意味します。

判定には必ず電流値とシリアルの`IMU proof`を組み合わせます。電流が下がっても
IMU値が読めなければ成功ではありません。

## 状態シーケンス

各状態は遷移後1秒待ってから20秒間継続します。1周は約147秒です。

| 状態 | 内容 | 判定上の役割 |
| --- | --- | --- |
| S0 | 共有レールOFF | IMU・マイクともOFFの参考値 |
| S1 | 共有レールON、加速度416 Hz、ジャイロOFF | IMUのみの目標基準値。初周ではDMIC未設定 |
| S2 | S1 + PDM録音 | マイク動作時の比較値 |
| S3 | S2から`DMIC_TRIGGER_STOP` | STOPだけでスリープするか |
| S3D | S3 + sleep pinctrlでCLK/DIN切断 | pinctrl sleepの効果 |
| S4 | S3D + P1.12をGPIO Low固定 | CLKを確実にLowにした効果 |
| S5 | S4 + DMIC device suspend試行 | デバイスPMの追加効果 |

LEDとBLEは全状態で無効です。S1以降は各状態で一度、加速度値を表示します。

## ビルドと書き込み

前提はNCS v2.9.2とnRF54L対応のpyOCD 0.37.0以降です。

```bash
cd pdm_power_test
./build_and_flash.sh
```

pyOCDを明示する場合:

```bash
PYOCD=/path/to/pyocd ./build_and_flash.sh
```

ボードターゲットは`xiao_nrf54l15/nrf54l15/cpuapp`、ビルドはsysbuildを使用します。
シリアルは115200 baudです。

```bash
screen /dev/tty.usbmodemXXXXXXXXX 115200
```

## 判定

最初に`S2 > S1`であることを確認します。差が見えなければ測定器の分解能不足で、
以降の比較から結論を出せません。

| 観測 | 結論 | 本番コードへの反映候補 |
| --- | --- | --- |
| S3 ≈ S1、IMU proof成功 | STOPだけでマイクは実質スリープ | `DMIC_TRIGGER_STOP` |
| S3 > S1、S3D ≈ S1 | sleep pinctrlが必要 | sleep stateを適用 |
| S3D > S1、S4 ≈ S1 | CLKのLow固定が必要 | P1.12をGPIO Lowへ切り替え |
| S4 > S1、S5 ≈ S1 | device suspendが必要 | `CONFIG_PM_DEVICE` + suspend |
| S3〜S5がS2相当 | クロック停止で十分な低下を確認できない | ハードウェア構成を再検討 |
| 全状態差が100 µA未満 | 測定系が不十分 | 分解能の高い機材で再測定 |

結論は「マイクのVDDをOFFにできた」ではなく、次のどちらかで記録します。

- マイクVDDはONだが、消費電流はIMUのみのS1基準まで戻った。
- クロックを停止しても消費電流はS1基準まで戻らなかった。

接続方法、安価なUSB電流計を含む測定器別手順、記録表は
[MEASURE.md](MEASURE.md)を参照してください。

## 現在の確認状況

2026-08-31にNCS v2.9.2でビルドと実機書き込みを行い、S2からS3へ遷移しても
416 Hz設定のIMU値を取得できることを確認済みです。電流値は未測定のため、
マイクが実質スリープするかの最終結論はまだ出していません。

## マイク仕様の扱い

メーカーの[製品一覧](https://en.memsensing.com/product/176.html)には実装品
`MSM261DGT006`が掲載されていますが、公開ページにスリープ電流はありません。
関連品`MSM261DGT003`の
[メーカー作成データシート](https://dfimg.dfrobot.com/5d57611a3416442fa39bffca/wiki/d2c58ceeebf0ab6a527b0c37c0ef525e.pdf)
には、50 kHz以下でtyp. 1 µA、スリープ移行最大30 µsと記載されています。
これは`MSM261DGT006`の保証値ではなく、この実測で確認する仮説として扱います。
