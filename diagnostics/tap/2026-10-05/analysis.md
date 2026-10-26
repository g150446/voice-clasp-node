# ソフトウェアタップ判定の実機記録（2026-10-05）

ファームウェア0.0.111、416 Hz、±4 g、2496サンプル/試行。通常姿勢は既定40 ms。
斜め45度姿勢では40 ms、60 msを比較後、幅だけ100 msの一時設定を試した。
全JSONに生XYZ、姿勢ラベル、適用config、判定flag、通知、欠落/飽和情報を保存。

| 姿勢・設定 | 操作 | 結果 |
|---|---|---|
| 通常、40 ms | single | single検知 |
| 通常、40 ms | double（約200 ms間隔） | double検知、single誤分類なし |
| 通常、40 ms | 普段の腕の動き | 通知・候補なし |
| 斜め45度、40 ms | single | single検知 |
| 斜め45度、40 ms | double | 1打目を幅で棄却し、single誤分類 |
| 斜め45度、60 ms | 同じ保存波形をPC再生 | 1打目が再度棄却 |
| 斜め45度、100 ms | double | 2打をdouble検知 |
| 斜め45度、100 ms | 少し速めの手首の動き | 通知・候補なし |

100 ms候補の独立実機doubleは1試行成功。40 ms記録の未検知した1打目は約50 ms、
60 ms設定での別の未検知は約89 msの非静止区間を含む。double間隔は約197–313 ms。
すべての有効記録でFIFO欠落0・測定範囲飽和0、実機と本番CコードによるPC再生のflagsが
2496サンプル全て一致する。検知率・誤検知率の保証に足る反復数ではない。
手のひら上向き試行はユーザーの訂正により無効扱いとし、記録・集計から除外した。

## 設定の比較

`sweep`は8記録に対して156設定が期待分類を再現した。max_pulse_msの探索値は
40/60/80/120/160/200 ms（100 msはこのgridに含まれない）。選んだ100 msは保存8記録を
すべて期待分類し、24個のC合成シナリオと設定検証も通過。これを実機RAMにBLEで適用し、
readbackを確認した。40 ms既定をビルドへは反映していない。RAMの100 ms設定は再起動または
SYSTEMOFF復帰後に40 msへ戻る。100 msでのsingle実機再試行、反復信頼性、実際の運転振動耐性は未確認。
タップなし動作は運転振動の代用ではない。

## 再生

本番C分類器を使って各JSONを再生し、一つの設定候補を作る:

```bash
python3 mac_client/tap_lab.py replay diagnostics/tap/2026-10-05/harness_tap_0111_45deg_double_1.json
python3 mac_client/tap_lab.py sweep diagnostics/tap/2026-10-05/harness_tap_0111_single_1.json diagnostics/tap/2026-10-05/harness_tap_0111_double_2.json diagnostics/tap/2026-10-05/harness_tap_0111_motion_1.json diagnostics/tap/2026-10-05/harness_tap_0111_45deg_single_1.json diagnostics/tap/2026-10-05/harness_tap_0111_45deg_double_1.json diagnostics/tap/2026-10-05/harness_tap_0111_45deg_double_60ms_1.json diagnostics/tap/2026-10-05/harness_tap_0111_45deg_motion_100ms_1.json diagnostics/tap/2026-10-05/harness_tap_0111_45deg_double_100ms_1.json --output /tmp/tap_candidates.json
```

`candidate_width_100ms.json`は他の全ての検出器条件を保存時のままにし、最大衝撃幅だけを100 msに設定する。
