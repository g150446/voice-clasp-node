# 掌上0.5秒静止→挙上→掌下静止ジェスチャー仕様

対象デバイス: Seeed Studio XIAO nRF52840 Sense  
対象ファームウェア: HarnessNode `0.0.74` 以降（最終誤発火ゲートは`0.0.72`、停止 soft-lower は`0.0.73`、開始 hold 緩和は`0.0.74`）

## 検証時の二段階プロトコル

開始動作と停止動作を同じ時間窓で評価しないため、Mac validator は次の順で案内する。

1. Ping音の後、掌上候補の静止 → 挙上 → 掌下への回内を行う。
2. `START OK` が表示されるまで掌下を維持する。これは録音開始イベントを受信した合図である。
3. `START OK` から既定1.3秒後（開始抑制 3000 ms を超える試験ではそれ以上）、Glass音と `STOP GO` が出るまで掌下を維持する。
4. `STOP GO` の後に**掌は返さず腕を下ろし**、`recording_stop` を発生させる。

`STOP GO` 前の `recording_stop` は停止ジェスチャーの先行入力として記録し、試行を不成立にする。
`recording_start` が発生しなかった場合は停止合図も出ないため、停止動作の成否や所要時間は
開始判定とは別に扱う。

## 発動条件

- XIAO をリストバンドの**手の甲側**に置く
- 部品面を皮膚側にし、基板 X 軸を前腕と直交、Y 軸を前腕に沿わせる
- 掌を上にし、基板がほぼ水平（`|Z|/|a| ≥ 0.75`）のまま **0.5秒静止**する
  - `|a|` 8.5–11.5 m/s²、4サンプルの線形加速度RMS ≤ 4.0 m/s²、候補開始姿勢から20°以内を連続して満たす
  - 0.5秒未満で条件が崩れた候補は破棄し、静止を数え直す
  - 0.5秒成立時にジャイロを104 Hz / ±500 dpsでONにし、50 ms整定後、その重力方向を掌上基準とする
  - 加速度だけで水平な掌上/掌下を絶対判別できないため、水平静止は「掌上候補」として扱う
- 掌向きを問わず、口元へ上げる（物理目安 **約 5 cm**）
- 上げたあと、**掌上基準から手首を回して掌下** にし、**500 ms** 静止する
  - 掌下: 重力反転（phi≥15° または Δz≥0.40 または符号反転）と `|∫gyro_y dt| ≥ 30°`（0.0.70、積分対象 |ωy|≥10 dps）。peak `|gyro_x| / |gyro_y| ≥ 0.42` は、挙上未成立・回内先行・または挙上入口 impulse < 0.30 のときに要求。XY 免除は **回内前 lift かつ入口 +imp≥0.30** のときだけ。一度成立したらその試行中はラッチする
  - 挙上候補: `a_up` 正インパルス ≥ **0.30** m/s（0.0.68）
  - 静止進入: 線形加速度 RMS ≤ 3.0 m/s²。進入時のみ `|gyro_y| ≤ 90°/s`
  - hold中: RMS > **4.0** m/s²が**3**サンプル連続した場合だけ中断する（0.0.74）
  - 最終静止: **500** ms（0.0.68）。姿勢差 ≤ 15°
  - 最終発火: 挙上全体の正インパルス ≥ **0.65 m/s** **かつ**掌上基準からの重力角 phi ≥ **140°**（0.0.72）。30°のgyro条件は遅い回内をholdへ進める予備条件であり、単独では発火しない
  - gyro起動後の挙上開始期限は **8000** ms（0.0.74）。弱い挙上パルスは `lift_near_miss`（0x25）で通知
- 録音中に**挙上と逆向きの線形パルス（手下ろし）**で録音終了する。掌上反転・静止のみでは終了しない（0.0.69+）
  - 開始後 3000 ms 抑制のあと、opp パルス + settle。失敗パルスは `stop_near_miss`（0x0B）で通知（0.0.73）
  - 録音 5 s / 10 s 経過で閾値を段階緩和（soft / soft2）。パルス間 50 ms のヒステリシスと settle 中の単発スパイク無視
- ジャイロは録音終了（またはシーケンス失敗）で OFF

右手・左手および USB 向きの差は、掌上候補から最終掌下への相対姿勢変化で吸収する。
最終姿勢の仰角帯・甲上条件・回外は要求しない。

重力 LP（tau 0.30 s）で姿勢成分を除き、候補開始のRMS静止判定と挙上パルス判定に使う。

## 装着と軸

| 軸 | 基板上 | 役割 |
|---|---|---|
| `X` | 長手（USB から離れる） | 前腕を横切る |
| `Y` | 短い辺方向（5V/GND 側） | 前腕軸＝回内・回外軸（**gyro_y**） |
| `Z` | 部品面外向き | 基板水平（|Z 比|）の判定 |

実機検証では装着ごとに raw `z` の符号が反転したため、
水平姿勢は Z の絶対比、掌下 hold は0.5秒静止時の掌上基準からの
相対 phi / Z 変化で判定する。角速度は **gyro_y**。

```text
board_flat = abs(z / |a|) >= 0.75
palm_up_dwell = board_flat AND 8.5<=|a|<=11.5 AND rms<=4.0 AND tilt<=20° for 500 ms
lift_before_flip = (|∫ωy| at lift entry) < 50°
lift_strong = (pos_imp at lift entry) >= 0.30
palm_down_after_lift = latch(gravity flip AND gyro_y angle>=50° AND (peak_x/peak_y>=0.42 OR (lift_done AND lift_before_flip AND lift_strong)))
hold_entry = palm_down_latched AND rms<=3.0 AND tilt<=15° AND |ω_y|<=90
hold_continue = palm_down_latched AND NOT(rms>3.5 for 2 samples) AND tilt<=15°
motion_complete = lift_start_to_hold_done < 4500 ms
match_gate = positive_impulse>=0.65 AND palm_up_reference_phi>=140°
```

## 上昇動作の測り方（双極パルス）

掌上成立後:

1. 低通加速度を重力推定とし、rawとの差から線形加速度を得る
2. 線形加速度をその時点の低通重力方向へ投影し、上向き加速度 `a_up` を得る
3. 上向きの加速パルスを確認する。減速パルスがあれば早期に hold へ進む
4. 正インパルスを満たし、減速パルスを検出するか、掌下・gyro静定・4サンプルRMS静止が揃ったらholdへ進む。掌向きは加速パルス判定中には見ない
5. 掌上基準から手首が反転してから、短い加速度 RMS 窓が静止を示した時点の重力方向を保持基準とし、その後の角度差が 15° 以内か確認する

変位の二重積分は使わない。約 5 cm はユーザー動作の目安であり、判定閾値ではない。
加速インパルスと、掌上基準からの反転後の姿勢安定・500 ms 静止を満たす必要がある。減速は任意。

## 録音停止（挙上と逆向きの線形パルス＝手下ろし）

MATCH 時に挙上パルス中の線形加速度を積算して単位ベクトル `L` を固定する
（弱ければ重力 LP 単位で代用）。併せて lift 正インパルスを保存する。

開始後 **3000 ms** は停止判定しない。その後:

1. `a_opp = -dot(linear, L)` が **≥ 0.25 m/s²** の短いパルス
2. 負インパルス ≥ min(0.35, max(0.10, 0.20×lift_imp))、パルス長 **60–2000 ms**
   （長すぎる定常 G は車両向けに破棄。相対閾値に上限を付け強挙上後も止めやすく）
3. パルス後 **80 ms** の quiet settle（`|a|` 7.5–12.5、linear ≤ 4.0）

掌上反転・gyro_y 単独では止めない。静止のみでも止めない。
ホスト `0x00` とシリアル `'s'` は従来どおり。停止後にジャイロ OFF。

## 状態遷移

```text
WAITING(掌上候補0.5秒) [dwell accept → gyro ON]
        → HOLDING_FINAL(WAIT_ACCEL → WAIT_BRAKE → WAIT_HOLD)
WAIT_HOLD は掌上基準からの反転が取れるまで開始しない
WAIT_BRAKE では減速パルス、または掌下・gyro静定・4サンプルRMS静止で WAIT_HOLD へ
挙上開始から4500 ms時点で掌下連動が未成立なら`palm_down_gate_failed`、
掌下成立後も500 ms最終静止が未完了なら`motion_too_slow`
500 ms静止後に最終発火ゲート不足なら`match_gate_failed`
MATCH → recording (gyro stays ON) → stop hand-lower → gyro OFF
```

## しきい値

| 項目 | 値 |
|---|---:|
| 開始水平 | \|Z 比\| ≥ 0.75 |
| 掌上候補静止 | 500 ms、\|a\| 8.5–11.5 m/s²、RMS ≤ 4.0 m/s²、姿勢差 ≤ 20° |
| ジャイロ | 0.5秒静止成立で ON、録音終了で OFF。ODR 104 Hz、FS ±500 dps、整定 50 ms |
| 上向き加速度 | `a_up` ≥ 0.40 m/s² を2サンプル、正インパルス ≥ 0.30 m/s |
| 最終発火ゲート | 挙上全体の正インパルス ≥ **0.65 m/s** かつ掌上基準からの phi ≥ **140°** |
| 逆向き減速 | 任意。`a_up` ≤ -0.15 m/s² なら早期に hold へ |
| パルス形状 | 150 ms以上。固定の最大パルス時間は設けない |
| hold の掌 | 重力反転 **かつ** 逆符号 gyro_y（∫≥30° + XY連動または lift 免除） |
| 保持中の姿勢差 | 静止開始時の重力方向から ≤ **15°** |
| 静止進入 | 4サンプル RMS ≤ **3.0** m/s²。進入時のみ \|ω_y\|≤**90** dps |
| hold中RMS | **3.5** m/s²超過が **2サンプル連続**した場合に中断 |
| 最終静止 | **500** ms |
| 挙上開始待ち | gyro起動後 **5000** ms |
| 動作完了期限 | 挙上開始から500 ms最終静止完了まで **4500 ms未満** |
| 録音停止 | 挙上逆向き線形パルス（peak≥0.25、imp≥min(0.35,max(0.10,0.20×lift))、60–2000 ms）+ 80 ms settle |
| 録音停止 開始抑制 | 3000 ms |

## 診断イベント（抜粋）

| stage | 意味 |
|---|---|
| `outbound_start` | 掌上候補開始。v1=Z絶対比、v2=0、v3=線形加速度 |
| `outbound_ready` | 0.5秒静止成立。v1=dwell ms、v2=Z絶対比、v3=線形加速度 |
| `gyro_enabled` (0x0D) / `gyro_disabled` (0x0E) | ジャイロ電源 |
| `wait_reject` reason `start_not_palm_up` | 基板が水平でない。v1=Z比、v2=下限、v3=線形加速度 |
| `wait_reject` reason `quiet_not_ready` | 水平だが重力帯または静止条件を満たさない |
| `wait_reject` reason `final_hold_interrupted` | hold 中断。v1=RMS、v2=tilt、v3=\|gy\| |
| `final_sample` (0x21) | 挙上中 100 ms 周期: pulse_stage, a_up, net_impulse |
| `hold_sample` (0x22) | hold 中 100 ms 周期: rms, tilt, \|gy\| |
| `final_hold_start` / `final_ready` | 静止保持開始 / 完了（v1=pos imp、v2=neg imp or hold_ms、v3=tilt） |
| `motion_complete` (0x23) | 挙上開始から最終静止完了まで。v1=elapsed ms、v2=gyro Y peak、v3=積分角 |
| `palm_down_gate` (0x24) | 掌下未成立の理由。重力反転不足、gyro Y積分角不足、peak gyro X/Y比不足を個別送信 |
| `match` | 発動 |
| `stop_near_miss` (0x0B) | 手下ろしパルス不成立。reason=`stop_impulse_low`/`stop_peak_low`/`stop_pulse_short`/`stop_pulse_long`。v1=opp_imp、v2=peak、v3=need または pulse_ms（0.0.73） |
| `stop_hand_lower` (0x0C) | 録音中の手下ろし停止。v1=opp_imp、v2=peak a_opp、v3=pulse_ms |
| `lift_near_miss` (0x25) | 弱い挙上または挙上開始タイムアウト。v1=peak a_up、v2=pos_imp、v3=elapsed_ms（0.0.74） |
| `reset` reason `outbound_timeout` | 掌上不成 |
| `reset` reason `final_accel_missing` | gyro起動後 8000 ms 以内に上向き加速が不足（0.0.74）
| `reset` reason `motion_too_slow` | 挙上開始から最終静止完了まで4500 ms以上 |
| `reset` reason `palm_down_gate_failed` | 4500 ms時点で掌下の重力＋gyro連動条件が未成立。録音停止動作を開始時間へ含めず別分類する |
| `wait_reject` / `reset` reason `match_*` | 500 ms静止後の最終発火ゲート不足。実測インパルス・phiと閾値を通知する |

### 履歴バッチ（IMU収集スイッチがONのときのみ）

録音終了またはシーケンス失敗後にまとめて送信（録音中は送らない）:

| event | 内容 |
|---|---|
| `0x33` history_begin | count, session_id |
| `0x34` history_entry | u16 t_ms, stage, reason, f32×3（19 B） |
| `0x35` history_end | count, session_id |

40 Hzの6軸履歴は成功時に録音停止後、ジャイロON後の失敗時に即時送信する。

| event | 内容 |
|---|---|
| `0x36` trajectory_begin | version, session, result/reason, count, period, gyro Y bias |
| `0x37` trajectory_chunk | index付き最大8サンプル。`t_ms`, validity, accel XYZ, gyro XYZ |
| `0x38` trajectory_end | sent count, overflow/notify error flags |

デバッグ版ではホストコマンド `0x04` で、現行分類器を変更せずに6秒間の独立した
6軸収集を開始できる。終了時は同じ `0x36–0x38` を `result=3` として送る。
これにより不成立の負例も、ジェスチャー状態機械のリセット時刻に左右されず同じ長さで保存する。

閾値を追加する前に、右手・左手それぞれで正例3回（自然・遅い・速い）と負例3回
（挙上のみ・回転のみ・日常動作）、合計12回を収集する。左右の符号差を避けるため、
候補特徴は基線相対のXY合成値、各gyro軸の絶対peak、gyro Y絶対積分を用いる。
車・電車の実測は含めず、一定0.1–0.5 gと低周波ノイズを加えた感度解析は
ロバスト性の予備評価としてのみ扱う。

### 0.0.58–0.0.60 低速回内挙上の除外とhold安定化

収集した使用可能13試行では、自然・高速の正例5試行の peak `|gyro_y|` は
最小369.8 dps、左右の低速2試行は最大178.2 dpsだった。0.0.59では瞬間peakを
速度の必須下限にはせず、挙上開始から400 ms最終静止完了までが3秒以上なら
低速として除外する。回転のみの負例を除くため、挙上パルス未成立（WAIT_ACCEL）の間は
peak `|gyro_x| / |gyro_y|` 0.42以上を要求する。挙上パルス成立後は加速度側で
腕の動きが担保済みのため XY 比は必須にしない。`|∫gyro_y dt|` は80°以上を
必要とする。すべて絶対値または比率で判定するため右手・左手を同じ条件で扱う。
重力による掌下判定とのAND条件であり、gyro peak単独では成立しない。

減速パルスを検出できなくても、掌下、gyro静定、加速度RMSが成立すれば最終holdへ
進める。これにより実測1,625 msの自然動作を固定パルス上限だけで棄却せず、
明らかに遅い動作だけを完了時間で除外する。

0.0.60では、掌下の重力＋gyro連動条件を一度満たしたらシーケンス終了まで保持する。
回転停止時の逆方向角速度で積分角が80°未満へ戻っても、成立済みの掌下を取り消さない。
また、静止進入はRMS 3.0 m/s²以下のまま維持し、hold中は3.5 m/s²超過が
2サンプル連続した場合だけ中断する。単発の境界ノイズで400 msを数え直さないための
    ヒステリシスであり、進入条件自体は緩和しない。

### 0.0.63 挙上後の XY 比必須解除

実測で gyro_y 積分角≥80°かつ重力反転は足りるが peak|gx|/|gy| が 0.14 前後の
自然な回内が `palm_down_xy_ratio_low` で落ちた。挙上は加速度パルスで既に判定
しているため、0.0.63 では lift stage が WAIT_ACCEL を抜けたあとは XY 比を
掌下ラッチの必須条件から外す。挙上前の手首回しのみは従来どおり XY 比で抑止する。

### 0.0.64 録音停止の誤発火抑制

掌下録音中に 3D tilt だけが 20° を一瞬超えて早期停止する事例があった
（phi 未達・gyro ほぼゼロ）。0.0.64 では:

- 重力停止は **phi≥20° 必須** +（tilt≥25° または Δz≥0.35 または Z 符号反転）
- 判定は **現在姿勢の 250 ms 連続**（peak latch では止めない）
- gyro の peak 単独（50 dps）経路を廃止（∫≥45° かつ peak≥30 のみ）
- 開始直後の停止抑制を 1200 ms → 1500 ms

### 0.0.65 低速回内の開始感度

ゆっくり回内すると掌下ラッチ後に 400 ms hold が 3 秒期限を超え
`motion_too_slow` になる事例（∫107° / peak439 / 3013 ms）があった。0.0.65 では:

- 動作完了期限 3000 → **4500 ms**
- hold 回内 ∫ 80° → **50°**、積分対象レート 15 → **10 dps**
- 重力 phi 20° → **15°**、Δz 0.50 → **0.40**

### 0.0.66 録音停止のさらなる誤発火抑制

開始約1秒後に重力だけで停止する事例（phi差165° / gyroほぼ0、使用者は掌下静止
の認識）があった。0.0.66 では:

- 開始後抑制 1500 → **3000 ms**。抑制中は静かなら掌下基準を更新し終了時に固定
- 連続成立 250 → **500 ms**
- 重力は **LP の phi** +（**Δz≥0.50 または Z 符号反転**）。tilt 単独不可
- 開始条件・案B（挙上待ち延長）は変更しない

### 0.0.67 手首反転のみの開始誤検出抑制

挙上なしの掌返し（flip_only）が、回内中の偽 `a_up` で lift stage を抜け、
0.0.63 の XY 比免除により録音開始する事例があった（右手 2/2 再現、xy≈0.19–0.22）。
一方、正しい挙上後の低 xy 回内（xy≈0.20）は MATCH が必要。0.0.67 では:

- 挙上パルス成立時に `|∫ωy|` をスナップショットし、`lift_before_flip = (|∫ωy| < 50°)`
- XY 比免除は **`lift_done AND lift_before_flip`** のときだけ
- 回内が先に進んだあとの偽 lift や flip_only は xy≥0.42 が無い限り掌下ラッチしない
- 挙上と回内が重なり xy が高い正例は従来どおり xy 経路で成立

### 0.0.68 日常生活の誤開始抑制

Android 履歴の日常 FP（hold +imp 中央≈0.14–0.26）がフルシーケンスで MATCH していた。
0.0.68 では:

- 挙上正インパルス 0.04 → **0.30** m/s
- XY 免除は `lift_before_flip` **かつ** 入口 +imp≥**0.30**
- 最終静止 400 → **500** ms
- match 直前に `match_detail(0x0A)`: xy / lift_imp / roll_at_lift、reason bit=before_flip|xy_waived

### 0.0.69 録音停止を手下ろしへ

掌上反転停止を廃止し、MATCH 時に固定した挙上軸 `L` の逆向き線形パルス + settle のみで停止する。
静止のみ・掌上のみでは止めない。開始後 3000 ms は停止抑制。

### 0.0.70 掌下 hold の gyro 感度

自然な掌下で ∫ωy≈35° が 50° 閾値で落ちた。hold の `|∫gyro_y|` 下限を **50° → 30°**。

### 0.0.71 Android 実使用向け停止感度

Razr 履歴「リンゴ」では停止まで約 13 s、opp_imp≈0.189 / peak≈0.57 でぎりぎり成立。
Mac の意図的下ろし（opp≈1.4）より弱い。0.0.71 では:

- peak ≥ **0.25**、imp ≥ min(0.35, max(0.10, 0.20×lift))
- pulse **60–2000** ms、settle **80** ms、quiet linear ≤ **4.0**
- lift 相対閾値に上限を付け、強挙上後も止めやすくする

### 0.0.72 運転・料理中の開始誤発火抑制

2026-08-27 18:00以降のRazr履歴45件をすべて負例として分析した。最終正インパルスは
0.303–1.178 m/s、掌上基準からのphiは35.4–173.5°で、単独閾値では正例と重なった。
最終正インパルス≥0.65 m/sとphi≥140°をAND条件にすると負例45/45を棄却し、
保存済み意図的正例10/10を維持したため、500 ms hold完了後の最終発火ゲートに採用した。

### 0.0.73 実使用の弱い手下ろし（soft-lower）

2026-08-28 13:06 Razr「リンゴと一言つぶやいてリンゴ」は MATCH 後 **約 24 s** で
`stop_hand_lower`（opp≈0.181 / peak≈1.75 / pulse≈157 ms）。意図的下ろし（opp≈1.4）より弱く、
途中の失敗パルスは履歴に残らなかった。0.0.73 では:

- `stop_near_miss` (0x0B) を 400 ms 間隔で通知（impulse/peak/pulse 不足理由）
- MATCH から 5 s で soft、10 s で soft2（need / peak / settle / quiet linear を緩和）
- パルス終端に 50 ms ギャップ・ヒステリシス、settle 中の a_opp 単発スパイクは 2 サンプル連続まで無視
- 長めパルス（≥180 ms）は peak ≥ 0.15 の slow-path で latch 可

### 0.0.74 開始の迷い・hold 中断緩和

Mac 試行で dwell 後に挙上せず `final_accel_missing`（5 s）、Android 履歴では
`final_hold_interrupted` が最多。0.0.74 では:

- 挙上開始期限 5000 → **8000** ms
- hold 中断: RMS > **4.0** が **3** サンプル連続
- `lift_near_miss` (0x25): 弱い a_up パルス破棄時と start timeout 時
- 0.0.72 最終 match ゲート（imp≥0.65 ∧ phi≥140°）は変更しない

### 6軸グラフの読み方

IMU収集をONにすると（RX `0x06`、FW `0.0.91+`）、試行終了時に同じサンプル列からCSVとPNGを
生成する。上段は加速度XYZ、下段はgyro XYZで、横軸はGOからの経過時間である。

- 灰色のgyro未取得区間は、掌上0.5秒静止が成立する前の正常な期間。
- 上段の加速度は、重力方向と挙上による線形加速度が重なった値。手を持ち上げたときの山は
  `a_up` パルスの根拠になるが、軸の符号は右手・左手や装着方向で変わる。
- 下段のgyro Yは前腕軸まわりの回内・回外を主に表す。開始判定の回内と、録音中の手下ろし
  （加速度の逆向きパルス）を時刻で分離して読む。停止は gyro ではなく線形加速度側を見る。
- gyro X/Zは挙上時の手首姿勢や複合回転にも現れる。gyro Xの山だけでは回内成立とは判定しない。

判定失敗時は、まず `reset` の理由を確認する。`palm_down_gate_failed` は「3秒を超えた低速」
ではなく、掌下の重力反転・gyro Y積分角・gyro X/Y比のいずれかが成立しなかったことを示す。
一方、`motion_too_slow` は掌下連動が一度成立した後、最終500 ms静止までに期限超過した場合に限る。

## 検証

```bash
cd mac_client
venv/bin/python gesture_validator.py --trials 1 --window 18 \
  --json-output /tmp/gesture-lift.json
```

表示条件: 掌上候補、0.5秒静止、挙上、掌下で静止、姿勢安定、
単一動作、発動、手下ろしで録音終了。各行に **実測値と閾値** を出す。
停止は挙上軸と逆向きの手下ろしパルスで判定する。
`recording_start` のあとホスト `0x00` では止めず、手下ろしによる `recording_stop` を待つ。
判定時間内に停止がなければ後始末として `0x00` を送る。

対話式の実機試験では、カウントダウン、開始合図、BLE 接続状態、判定結果を
macOS Terminal に表示する。ログを保存するときは `tee` を使い、画面表示を維持する。
物理操作が必要なため、試行内容と回数を事前に説明し、準備完了の確認後に1試行ずつ開始する。

### 0.0.54 実機回帰（2026-08-22）

`0.0.54` を BLE OTA で更新し、slot0 が active + confirmed であることを確認した。
録音開始後、掌下のまま約5秒静止してから意図的に掌上へ返す試験を4回実施した。

| 試行 | GO→`recording_start` | 録音開始相当→停止 | 停止経路 | 結果 |
|---:|---:|---:|---|---|
| 1 | 4.611 s | 6.045 s | gyro peak 65.96 dps | PASS |
| 2 | 4.742 s | 5.895 s | 重力 phi 12.51° | PASS |
| 3 | 4.430 s | 4.905 s | 重力 phi 16.71° | PASS |
| 4 | 4.356 s | 7.875 s | 重力 phi 33.59° | PASS |

全4試行で静止中の早期停止は発生せず、最後の意図的な掌上で停止した。試行3は
操作による掌上が5秒より95 ms早かったため4.905 sだが、静止中の自動停止ではない。
録音開始相当時刻は、`match` 後に受信した `gyro_enabled` の時刻を使用した。

### 0.0.72 実機受入（2026-08-27）

`0.0.72` をBLE OTAで更新し、slot 0が`active=true`、`confirmed=true`、
version `0.0.72`であることを確認した。Mac validatorの現装着側正例は、ユーザー指示により
2試行で終了し、両方とも開始・手下ろし停止がPASSした。

| 試行 | 最終正インパルス | phi | 最終hold | 結果 |
|---:|---:|---:|---:|---|
| 1 | 2.911 m/s | 161.2° | 500 ms | PASS |
| 2 | 0.883 m/s | 175.5° | 500 ms | PASS |

### 0.0.73 / 0.0.74 実機受入（2026-08-28）

| 版 | 内容 | 確認 |
|----|------|------|
| `0.0.73` | soft-lower 停止 + `stop_near_miss` | OTA active+confirmed。Mac 弱め手下ろし PASS（opp≈0.27） |
| `0.0.74` | 挙上開始 8 s、hold RMS 4.0×3、`lift_near_miss` | OTA active+confirmed。hash `3af0f110…c366c2`。Mac 開始+停止 PASS（latency≈4.9 s、弱 lift near-miss 2 件後に MATCH） |

Android受入では次を確認する。

1. アプリがHarnessNodeへ接続し、意図的な正例で録音開始・手下ろし停止が成立する。
2. 運転・料理の安全な模擬動作では録音が開始しない（停車・火/刃物なし）。
3. 試行前後の`voice_history_prefs.xml`を比較し、負例による新規認識履歴がない。
4. 誤発火時は時刻・動作・認識文と `lift_near_miss` / `stop_near_miss` / match 診断を照合する。

## 挙上ゲートは挙上を測っていない（2026-09-01）

### 運転対策は分類器の中で無効

`main.c:2491` は運転モードかつ非録音のとき `process_gesture_sample()` を即 return する。
**運転中はジェスチャー状態機械が1サンプルも走らない**（録音は double tap 専用）。
それでも分類器には車両加速度対策が残っている:

| 箇所 | 内容 |
|---|---|
| `:1675-1677` | 重力推定 LPF `τ = 0.30 s`（"removes vehicle accelerations"） |
| `:227-228` | 停止パルス上限 2000 ms（"rejects long vehicle G"） |
| `:2378` | quiet accel LPF（"vehicle vibration is averaged out"） |

τ=0.30 s のハイパス（コーナー ≈0.53 Hz）が直流成分を落とすため、
**到達高度・保持高度に類する条件を構造的に持てない。**今後のチューニングで
この制約に縛られる必要はない。

### 純回転だけで最終ゲートを通る

`mac_client/gesture_feature_eval.py synthetic-rotation` は並進ゼロの回内を合成し、
理想値 `a_up = 0` に対して各推定器が何を返すか測る:

| 回内時間 | peak `|ωy|` | 現行LPFの `∫a_up⁺dt` | gyro伝播版 | 最終ゲート(0.65) |
|---|---|---|---|---|
| 0.4 s | 707 dps | 2.051 | 0.000 | LPFはPASS |
| 0.6 s | 471 dps | 1.684 | 0.000 | LPFはPASS |
| 0.8 s | 353 dps | 1.401 | 0.000 | LPFはPASS |
| 1.0 s | 283 dps | 1.166 | 0.000 | LPFはPASS |
| 1.5 s | 188 dps | 0.774 | 0.000 | LPFはPASS |

**並進が一切ない手首の回転だけで、全速度域で 0.65 の挙上ゲートを通過する。**
τ=0.30 s の重力推定が回内に追従できず `|g_LP|` が縮むため、残差の `ĝ_LP` 射影が
正のまま数 m/s² 出続けるのが原因。gyro で重力を伝播すれば 0 になる
（gyro は掌上 dwell 受理時点で既にONなので追加電力は不要、`:190-193`）。

逆に**35 cm を 1.5 s かけた本物のゆっくりした挙上は 0.149 で不成立**。
ハイパスが意図的な低速動作の成分を削り、素早い動作を通す — 判別方向が逆になっている。

### 実運用データでの裏付け

履歴100件（`voice_history_prefs.xml`）の `0x07 final_hold_start` v1 と
`0x0A match_detail` v2/v3:

```
corr(総インパルス, 挙上latch時のroll) = 0.726       n=99
  roll < 50°  : n=60  総インパルス中央値 0.631
  roll ≥ 100° : n=23  総インパルス中央値 2.845

pos_imp_at_lift  範囲 0.300-0.601  中央値 0.344  σ=0.056
                 81/99 件が 0.30-0.40 に集中
```

`pos_imp_at_lift` は閾値 0.30 を跨いだ瞬間に latch するため**打ち切り変数**で、
挙上の大きさを測っていない。合成試験の latch 値（0.302–0.541）が実測分布と
一致することが、状態機械再現の妥当性の裏付けになっている。

0.0.68 の 0.30 は打ち切られて情報がなく、0.0.72 の 0.65 は回転で汚染されている。
誤発火と正検出の分布が完全に重なった理由はこれで説明できる。

### 旧解析（撤回済み、2026-09-01 07:58、session=143）

軌跡236サンプルの構造:

| 軌跡 t_ms | 事象 |
|---|---|
| 0 | 掌上 dwell 開始（`flags`=0、gyro未給電） |
| 548 | gyro 給電（dwell 受理。`GESTURE_PALM_UP_DWELL_MS` 500 と整合） |
| 1362 | 回内ピーク `|ωy|` = 573 dps |
| 1441 | `az` 符号反転（掌下ラッチ） |

この窓で両推定器を回した結果:

| 区間 | 現行LPF | gyro伝播版 |
|---|---|---|
| 挙上＋回内（0→1441 ms） | 1.098 | 1.602 |
| 静止hold 500 ms込み（→1941 ms、FWのmatch判定点） | **2.562** (+1.464) | 1.626 (+0.024) |

**この「hold中に1.46積む」という結論は積分窓の誤りであり撤回する。**
表の2.562はFWの最終ゲート値ではない。FWが積分を止めた後までオフライン側だけが
計算を続けた値であり、LPFとgyro伝播版の比較にも使用できない。
（上記の数値は積分終端が誤っているため、候補特徴の根拠には使用しない。）

### 積分窓の訂正（2026-09-01、実測4件）

FWは `begin_gesture_final_hold()` で `WAIT_HOLD` に入った後、
`gesture_lift_pos_impulse_ms` を更新しない。したがって積分窓の終端は**hold進入時**であり、
500 ms holdは含まれない。Android CSVの生サンプルからFW状態機械を再生し、hold進入で
積分を止めた結果は次のとおり。

| 試行 | offline hold進入 | `final_hold_start` v1 | `match` v2 | 誤差 |
|---|---:|---:|---:|---:|
| 09-01 11:50 誤発火 | 0.8471 | 0.8471 | 0.8471 | 0.00% |
| 09-01 11:55 誤発火 | 0.7827 | 0.7827 | 0.7827 | 0.00% |
| 09-01 13:57 意図的 | 3.3211 | 3.3211 | 3.3211 | 0.00% |
| 09-01 13:58 意図的 | 0.9667 | 0.9667 | 0.9667 | 0.00% |

`mac_client/gesture_feature_eval.py verify-window <csv...>` がこの照合を自動化し、既定5%を
超えると非ゼロ終了する。候補特徴の数値はこの検証が通った窓からだけ報告する。

### palm-up dwellは第3の打ち切り変数

`outbound_ready.v1` は実際に静止した総時間ではなく、500 ms閾値に到達した瞬間の値である。
履歴では誤発火5件・意図的33件のどちらも520–523 ms、中央値521 msで完全に一致した。
`pos_imp_at_lift` と同じく、閾値通過後の余剰時間を観測しない打ち切り変数なので、特徴量に
使えない。

生サンプルへdwell継続条件（RMS≤4.0 m/s²、候補姿勢からの傾き≤20°）を後付けした結果:

| 要求 dwell | 誤発火 11:50 | 誤発火 11:55 | 意図 13:58 | 意図 13:57 | 意図 07:58 |
|---|---|---|---|---|---|
| 500 ms | PASS | PASS | PASS | PASS | PASS |
| 800 ms | PASS | PASS | PASS | BREAK 757 ms | PASS |
| 1000 ms | PASS | PASS | PASS | BREAK 757 ms | PASS |
| 1500 ms | PASS | PASS | PASS | BREAK 757 ms | BREAK 1126 ms |
| 2000 ms | PASS | PASS | BREAK 1751 ms | BREAK 757 ms | BREAK 1126 ms |

1000 msでも誤発火2件は残り、意図的動作が先に落ちる。外来の誤発火は聴診・カルテ保持・
安静など実際に腕が静止する場面で起きる一方、意図的ジェスチャーはユーザーが次の動作へ
移るためである。`gesture_feature_eval.py dwell-sweep --limits 500 800 1000 1500 2000
<csv...>` で再現できる。dwell延長は実装しない。

### peak `|ωy|` の分離と飽和

ラベル付き履歴では、peak `|ωy|` は誤発火 n=6・中央値324 dps、意図的 n=39・中央値
569 dpsだった。400 dpsを閾値にすると意図的維持90%、誤発火削減83%だが、採用基準の
維持95%には届かない。さらに最大値573.4 dpsへ多数が張り付いている。これは
`GYRO_FULL_SCALE_DPS=500` と感度17.5 mdps/LSBから、符号付き16 bitの上限
（17.5×32768=573.4 dps）に対応するセンサー飽和である。

FSを上げると分解能が半減し `accumulate_gyro_roll()` の精度へ影響する。標本も小さく、
意図的ラベルは書き起こしによる発見的分類を含むため、peakゲートとFS変更は保留する。

### マイルストーンは使えない。軌跡だけで足りる

同じ試行で:

```
Captured 24 gesture diags for history      ← ライブ 0x30 は 24 件
TX event 0x33 (00 55 33 03 A3)             ← Node バッチは 3 件
Attached ... (milestones=3 from node)      ← アプリは 3 件の方を採用
```

Node の履歴バッファは flush 時点で末尾3件（`gyro_enabled` / `stop_hand_lower` /
`gyro_disabled`）しか保持していない。`gesture_history_push()` は
`FINAL_SAMPLE` 以外を無条件に積む（`:3670`）ので、シーケンス途中で
バッファがクリアまたは flush されている。`gesture_history_clear()` は
`:2608`（新しい掌上候補）と `:3443`（収集OFF）のみ、`flush_gesture_history()` は
`:3280` の遅延 flush（`!is_recording` 条件）で `count = 0` にする。
`:2608` なら軌跡も同時にクリアされるはずだが軌跡は t=0 から連続しているため、
**静的解析では機序を特定できていない。**printk を入れないと確定しない。

実害として、マイルストーンの t_ms は `gesture_history_t0_ms`
（clear後の最初のpush時刻、`:1488`）基準で、軌跡の `gesture_trajectory_t0_ms`
（`:1517`）とは**この試行で正確に 548 ms ずれていた**。

ただし**オフライン解析にマイルストーンは不要**である。軌跡そのものに
dwell・gyro給電（`flags` bit0）・回内ピーク・掌下ラッチが全て含まれており、
FW の積分窓は生サンプルから再現できる。上表はそれで計算した。

Android 側には修正の余地がある: ライブ 24 件を捨てて Node バッチ 3 件を
採用しているため、履歴に残る診断が実際より遥かに乏しい。

### 参考: 元の未解決メモ

軌跡CSVに添付される `# milestone` は `gesture_history_t0_ms`（clear後の最初のpush時刻、
`main.c:1488`）基準で、軌跡の `gesture_trajectory_t0_ms`（`gesture_trajectory_clear()`、
`:1517`）とは別変数。両者は `:2607-2608` で同時にクリアされ、直後に
`outbound_start` が push されるので本来は一致するはずだが、実測3件の CSV では
**先頭マイルストーンが `outbound_start` ではなく `gyro_enabled` (t=0)** になっている。
開始側マイルストーン（lift pulse、palm_down、final_hold_start、match）が
1件も含まれないため、**オフラインで FW の積分窓を再現できない。**
Step 2 の整合チェックを通すにはここを先に解く必要がある。

なお `gesture_trajectory_push()` は CAP でリングせず**停止**する（`:1548`）ので、
`overflow=true` は末尾（録音区間）の切り捨てを意味し、先頭の開始シーケンスは保持される。

## ビルド・OTA

```bash
# 必要に応じて NCS / west を PATH に
export NCS_BASE=/opt/nordic/ncs/v2.9.2
export PATH="/opt/nordic/ncs/toolchains/<id>/bin:$PATH"

nordic-main/build_and_package_ota.sh
cd mac_client && venv/bin/python ota_updater.py --device HarnessNode \
  ../nordic-main/ota_update.bin
```

`prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` と `VERSION` を揃えてからビルドすること。  
更新後は対象versionがslot 0でactiveかつconfirmedであること。  
`0.0.74` 確認済み hash: `3af0f11006cb33140f4706f2c5ff277a73762483a4406584a15fc8cc5ac366c2`。
