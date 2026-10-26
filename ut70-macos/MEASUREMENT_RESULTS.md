# UT70によるPDM省電力テスト実測結果

- 測定日: 2026-09-03
- 測定器: ALIENTEK UT70
- 測定対象: XIAO nRF54L15 Sense `pdm_power_test`

## 結論

UARTを閉じた低ノイズ条件で測定した基板全体の平均電流は、S0が
`9.377 mA`、S1が`9.536 mA`、S2が`9.897 mA`だった。

共有電源をONにしてIMUをpower-down、マイクをsleepにしたときの増分は
`0.159 mA`であり、マイクsleep状態だけで約10 mAを消費しているわけではない。
マイクをactiveにしたことによる追加増分は`0.361 mA`だった。

UT70のUSB HID reportには1 mAより細かい電流値が含まれており、0.1 mA単位の
違いをリアルタイムで取得できることも確認した。

## 測定状態

各状態を20秒ごとに切り替えた。LEDは全状態で消灯している。

| 状態 | 共有`IMU&MIC_3V3`電源 | IMU | マイク | CPU |
|---|---|---|---|---|
| S0 | OFF | OFF | OFF | System ON idle |
| S1 | ON | 加速度・ジャイロODR 0 | sleep、PDM停止、CLK Low | System ON idle |
| S2 | ON | 加速度・ジャイロODR 0 | 16 kHz active | PDM処理時のみ起床 |

ここでファームウェア上のdeep sleepは、20秒後にタイマー復帰できる
`System ON idle`を意味する。System OFFではない。S2ではPDM DMAバッファ処理時に
CPUが短時間起床する。

## 測定方法

1. UT70のHID端子をMacへ接続し、UT70の測定側からXIAOへ給電した。
2. CMSIS-DAP経由でnRF54をリセットし、直後をS0の開始時刻とした。
3. nRF54のUSB UARTは開かず、状態を20秒周期の経過時刻から判定した。
4. UT70の64-byte Input Reportを65秒間、読み取り専用で取得した。
5. 状態遷移後3秒間を過渡区間として集計から除外した。

取得結果は3,250 report、実測サンプルレートは`50.0 Hz`、チェックサム不良は
0件だった。

## 状態別電流

| 状態 | 有効サンプル数 | 平均 | 中央値 | 標準偏差 | 5–95パーセンタイル |
|---|---:|---:|---:|---:|---:|
| S0: 共有電源・IMU・マイクOFF | 951 | 9.377 mA | 9.364 mA | 0.107 mA | 9.214–9.517 mA |
| S1: IMU power-down・マイクsleep | 850 | 9.536 mA | 9.540 mA | 0.092 mA | 9.386–9.685 mA |
| S2: IMU power-down・マイクactive | 850 | 9.897 mA | 9.878 mA | 0.083 mA | 9.736–10.037 mA |

差分は次のとおり。

| 比較 | 電流差 | 主に含まれる負荷 |
|---|---:|---|
| S1 − S0 | 0.159 mA | 共有ロードスイッチ、power-down中のIMU、sleep中のマイク、I²Cプルアップ |
| S2 − S1 | 0.361 mA | マイクactive、PDM peripheral、PDMクロック、DMA起床 |
| S2 − S0 | 0.520 mA | 共有電源系とactiveマイクを合わせた増分 |

S0で一度`10.651 mA`の過渡値が記録されたため、min/maxだけでなく中央値と
5–95パーセンタイルも併記した。

## 0.1 mA分解能の確認

実機のHID reportから、次の値を取得できた。

| 復号した電流 | report内のbyte 23–26 |
|---:|---|
| 9.300134 mA | `97 5F 18 3C` |
| 9.400635 mA | `1F 05 1A 3C` |
| 9.499472 mA | `AC A3 1B 3C` |
| 9.600014 mA | `61 49 1D 3C` |

また、連続する2 reportから`9.516375 mA`と`9.445669 mA`を取得できた。
差は`0.070706 mA`であり、生byteも異なっていた。このため、HIDデータは
1 mA単位に丸められておらず、0.1 mAの差を識別できる。

電流値はbyte 23–26にlittle-endian IEEE-754 binary32のアンペア値として格納される。

```python
current_A = struct.unpack_from("<f", report, 23)[0]
current_mA = current_A * 1000
```

ただし、細かい数値がHIDに含まれることと、同じ桁まで絶対精度が保証されることは
別である。今回の状態内標準偏差は約`0.08–0.11 mA`だったため、消費電流の評価には
瞬時値ではなく安定区間の平均または中央値を使う。

## HID reportの確認済み構造

offsetは0始まり。

| byte | 内容 | 形式 |
|---|---|---|
| 0 | ヘッダー | 観測した測定reportでは`0xEE` |
| 1–2 | 未解析 | 観測値`08 3C` |
| 3–6 | 電圧 | little-endian IEEE-754 binary32、V |
| 7–22 | 未解析 | 推測で割り当てていない |
| 23–26 | 電流 | little-endian IEEE-754 binary32、A |
| 27–62 | 未解析 | 推測で割り当てていない |
| 63 | チェックサム | `sum(report[0:63]) & 0xFF` |

電力は独立したreport fieldとして未確定のため、モニタでは
`voltage_V * current_A`として計算している。UT70へのOutput ReportやFeature Reportは
送信していない。

## UARTによる測定への影響

状態確認のためnRF54のUARTを開いた測定では、全状態の絶対電流が約2.4 mA増加した。
この測定でも差分は`S1 − S0 = 0.156 mA`、`S2 − S1 = 0.365 mA`となり、UARTを
閉じた結果とほぼ一致した。ただし、UARTを開いた測定値は低消費電力時の絶対値には
採用していない。

今後も絶対電流を測る場合はUARTを閉じ、リセット時刻と20秒周期から状態を特定する。

## 関連ファイル

- [UT70ツールとプロトコル調査](README.md)
- [リアルタイムモニタ](ut70.py)
- [生HID表示ツール](ut70_raw.py)
- [HIDデバイス列挙ツール](ut70_scan.py)

測定時の生ログは次の名前で`logs/`に保存しているが、生成データとしてGit管理外に
している。必要な集計値と検証用byte列は本書に記録済みである。

- `logs/pdm_states_uart_closed.csv`: UARTを閉じた状態別実測
- `logs/pdm_states.csv`: UARTを開いた比較測定
- `logs/raw_100.csv`: 生HID report 100件
