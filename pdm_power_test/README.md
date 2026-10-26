# XIAO nRF54L15 Sense PDM省電力検証

XIAO nRF54L15 Senseの共有IMU/マイク電源と、PDMマイクをactiveにしたときの
消費電流を分離して測定する単独ファームウェアです。

IMUとマイクはTPS22916Cロードスイッチ配下の`IMU&MIC_3V3`を共有しています。
S0ではロードスイッチ自体をOFFにし、S1/S2では共有電源をONにしたままIMUの
加速度・ジャイロをpower-downへ設定します。

## 3つの状態

次の状態を20秒ごとに切り替えます。1周は60秒です。

| 状態 | 共有電源 | IMU | マイク | CPU |
| --- | --- | --- | --- | --- |
| S0 | OFF | 完全OFF | 完全OFF | System ON idle |
| S1 | ON | 加速度・ジャイロODR 0 | STOP、CLK Low | System ON idle |
| S2 | ON | 加速度・ジャイロODR 0 | 16 kHz active | PDM処理時だけ起床 |

比較の意味は次のとおりです。

- `S1 - S0`: 共有電源をONにしたときのロードスイッチ、power-down中のIMU、
  sleep中のマイクなどの合計
- `S2 - S1`: マイクactive、nRF54 PDM peripheral、クロック、DMA処理の増分

S0へ入るときはDMICを停止して残りのバッファを解放し、PDMピンを切断して
CLKをLowへ固定してからIMUをpower-downにし、最後に共有電源をOFFにします。
S1へ入るときは共有電源を再投入して10 ms待ってから、IMUの両ODRを0へ設定します。

各状態は遷移処理を含めて20秒です。絶対deadlineで管理するため、設定処理時間による
周期のずれは蓄積しません。

## CPUと状態表示

ここでのCPU sleepは復帰可能なSystem ON idleです。S0/S1では20秒タイマーまで
sleepし、S2ではPDMバッファ割り込み時だけ短時間起床します。System OFFはタイマー
復帰やPDM連続動作と両立しないため使用しません。

オンボードLEDは測定電流へ影響させないため全状態で消灯します。通常版ではUARTへ
次の状態表示を出します。

```text
>>> STATE S0: shared rail off; IMU off; microphone off; duration=20000 ms
>>> STATE S1: shared rail on; IMU power-down; microphone sleep; duration=20000 ms
>>> STATE S2: shared rail on; IMU power-down; microphone active; duration=20000 ms
```

## ビルドと書き込み

NCS v2.9.2とnRF54L対応のpyOCD 0.37.0以降を使用します。

```bash
cd pdm_power_test
./build_and_flash.sh
```

ボードは`xiao_nrf54l15/nrf54l15/cpuapp`、ビルドはsysbuildです。UARTは
115200 baudで、タイムスタンプ付きログは`python3 capture.py`で取得できます。

### 静音ビルド

```bash
./build_and_flash.sh --quiet
```

静音版はUARTを無効にし、LEDも消灯します。リセット後のS0=0〜20秒、
S1=20〜40秒、S2=40〜60秒という周期から状態を特定できる場合だけ使用します。

## 判定

UT70の表示が0.001 A刻みの場合、1 mAの差は1カウントしかないため、複数周で
同じ段差が再現するか確認します。これはUSB 5 V入力で見た基板全体の電流であり、
各デバイス単体の電流ではありません。

詳しい手順と記録表は[`MEASURE.md`](MEASURE.md)を参照してください。
