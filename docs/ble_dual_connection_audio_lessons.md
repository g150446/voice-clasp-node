# BLE デュアル接続 + 音声ストリーミング 実装知見

nRF52840（XIAO nRF52840 Sense）に Mac アプリ（Handy）と Android アプリを同時接続した状態で音声認識を動作させる際に判明した技術的な知見をまとめます。

---

## 1. BLE デュアル接続アーキテクチャ

### 接続スロット

`prj.conf` の `CONFIG_BT_MAX_CONN=2` によりファームウェアは最大 2 本の BLE 接続を保持できます。`connections[0]` と `connections[1]` に格納し、`primary_idx` でどちらが「プライマリ」かを管理します。

```c
#define MAX_CONNS 2
static struct bt_conn *connections[MAX_CONNS];
static int primary_idx = -1;
```

**プライマリの役割**：音声 PCM パケット・録音開始/終了イベントは `get_primary_conn()` のみに送信されます。セカンダリには一切送られません。

### ロール制御コマンド（RX Characteristic 経由）

| バイト値 | 送信元 | 意味 |
|---|---|---|
| `0x02` | クライアント → ファームウェア | **プライマリ宣言**（last-write-wins） |
| `0x03` | クライアント → ファームウェア | **プライマリ譲渡**（自分以外の接続をプライマリに設定） |
| `0x01` | ファームウェア → プライマリ | 録音開始イベント |
| `0x02` | ファームウェア → プライマリ | 録音終了イベント |

> ⚠️ 値 `0x02` は「クライアントからの primary 宣言」と「ファームウェアからの録音終了イベント」で共用されています。区別はパケット形式（イベントパケットは `[0x00][0x55][code]` の 3 バイト）で行います。

### 接続時のイベント通知

```c
// 2 本目が接続した際、ファームウェアは既存のプライマリに 0x31 を通知
uint8_t pkt[3] = { 0x00, 0x55, 0x31 };
bt_gatt_notify(get_primary_conn(), ...);

// いずれかが切断した際、残った接続に 0x32 を通知
uint8_t pkt[3] = { 0x00, 0x55, 0x32 };
```

### Test A（Handy プライマリ）の接続シーケンス

```
Android(MAC_HANDYモード)   nRF52840              Handy
        │── connect ──────────▶│                     │
        │                   primary_idx=0             │
        │                      │──── connect ────────▶│
        │                   primary_idx=0             │
        │◀── 0x31(peer connected)──│                  │
        │── 0x03(yield) ─────────▶│                   │── 0x02(claim primary) ──▶│
        │                   primary_idx=1             │
        │                      │  conn_param_work      │
        │◀── slow(100ms) ────────│── idle(30ms) ──────▶│
```

### Test B（Android プライマリ）の接続シーケンス

```
Handy(Test Aから継続)    nRF52840              Android(ANDROIDモード)
        │── connected ──────────│                     │
        │                   primary_idx=1             │
        │                      │◀──── connect ────────│
        │                      │◀──── 0x02(claim) ───│
        │                   primary_idx=0             │
        │◀── 0x31(peer connected)──│                  │
        │  (stays primary, no re-claim)               │
        │                      │  conn_param_work      │
        │◀── slow(100ms) ────────│── idle(30ms) ───────▶│
```

---

## 2. 問題：デュアル接続時の BLE 音声スループット低下

### 症状

Handy と Android を同時接続した状態で Android をプライマリにして録音すると、Android 側が「無音」を報告し音声認識が失敗する。

### 根本原因

nRF52840 の BLE ラジオは **1 本の無線回路を複数接続でタイムシェア** します。2 接続が共に ~83ms の接続インターバルを持つ場合、各接続は約 **6 イベント/秒** しか得られません。

| 状況 | 接続インターバル | 音声スループット |
|---|---|---|
| Android のみ接続 | ~83ms → 12イベント/秒 | ~2.4 KB/秒（辛うじて認識可能） |
| Handy + Android 同時接続（修正前） | 各~83ms → 各6イベント/秒 | ~1.2 KB/秒 (**4% of 必要量**) |
| Handy + Android 同時接続（修正後） | Android:7.5ms / Handy:200ms | ~24 KB/秒（十分） |

**音声仕様**: 16 kHz / 16-bit / mono = 32 KB/秒、PCM パケットサイズ 200 バイト → **160 パケット/秒必要**

### 実測値（修正前）

```
VoiceProcessor: BLE recording stopped by firmware, 12160 bytes of PCM
Silero VAD: rmsAfterDC=0.0012   ← ほぼ無音
```

10 秒録音で 12 KB しか届かず（期待値 320 KB）。VAD が音声なしと判定し Groq API を呼ばずにスキップ。

---

## 3. 解決策：接続パラメータ最適化（`conn_param_work`）

### 設計方針

- **プライマリ接続**（音声データを受け取る側）: 録音中だけ 7.5 ms。それ以外は 30–50 ms（FW `0.0.100+`）
- **セカンダリ接続**（イベント通知のみ）: 常に 100–150 ms
- **supervision timeout / GAP PPCP**: すべて 8 s。idle 中に間隔を再更新しない
- **ウォッチドッグ 12 s**（FW `0.0.102+`）: supervision より必ず長くすること。理由は下の
  「supervision timeout をウォッチドッグより長くしてはいけない」

### 実装（`main.c`）

```c
/* conn_param_work: primary/secondary が決まった後に遅延実行 */
static struct k_work_delayable conn_param_work;

static void conn_param_work_handler(struct k_work *work)
{
    static const struct bt_le_conn_param fast_param = {
        .interval_min = 6,    /* 7.5 ms */
        .interval_max = 12,   /* 15 ms  */
        .latency = 0, .timeout = 800,
    };
    static const struct bt_le_conn_param idle_param = {
        .interval_min = 24,   /* 30 ms */
        .interval_max = 40,   /* 50 ms */
        .latency = 0, .timeout = 800,
    };
    static const struct bt_le_conn_param slow_param = {
        .interval_min = 80,   /* 100 ms */
        .interval_max = 120,  /* 150 ms */
        .latency = 0, .timeout = 800,
    };
    bool primary_fast = /* recording only */;
    for (int i = 0; i < MAX_CONNS; i++) {
        if (!connections[i]) continue;
        const struct bt_le_conn_param *p = (i != primary_idx) ? &slow_param
            : (primary_fast ? &fast_param : &idle_param);
        bt_conn_le_param_update(connections[i], p);
    }
}
```

### 発動タイミング

接続確立時、primary claim（RX `0x02`）/ yield（RX `0x03`）、録音開始・停止で **200 ms 後に発動**するようスケジュール：

```c
k_work_schedule(&conn_param_work, K_MSEC(200));
```

BT コールバック内から直接 `bt_conn_le_param_update()` を呼ぶと BT スタックのロックと競合する可能性があるため、**必ずワークキュー経由で遅延実行**してください。

### 注意：セントラル側が更新を受け入れるかどうか

`bt_conn_le_param_update()` はペリフェラルからセントラルへの **要求** であり、セントラルが拒否することもあります。

- **Android**: 7.5 ms–4000 ms は受け入れることが多い。GAP PPCP の timeout が短い（Zephyr 既定 420 ms）と、探索後にそれを採用して Connection Timeout（HCI 0x08）で切る。`prj.conf` で PPCP を idle（30–50 ms / 8 s）に合わせること
- **macOS (CoreBluetooth)**: 15 ms–2000 ms 程度は受け入れる
- **拒否された場合**: 既存のインターバルのまま動作する（切断にはならないとは限らない。timeout をセット間で変えない）

#### ペリフェラルでは要求は即座に送られない（`0.0.102` でログを訂正）

`bt_conn_le_param_update()` はペリフェラルでは **何も送信せず**、
`conn->interval_min/max` 等を保存して `BT_CONN_PERIPHERAL_PARAM_SET` を立てて
`0` を返すだけ（`host/conn.c`）。実際の送信は接続確立時に仕込まれた
`conn->deferred_work` が **T+5 s** に 1 回行う。

- `0.0.100` までのログ `>>> conn_param_update[i]: idle(30ms)` は
  **何も送っていない時点で出ていた**。`0.0.102` で
  `>>> conn_param[i]: ... requested (host sends at T+5s)` に改めた
- 受理されたかどうかの唯一の証拠は `>>> le_param_updated[i]`。
  これが一度も出なければ、相手は要求を全部黙殺してリンクを自分のパラメータで
  回している。`0.0.102` の `>>> DISCONN` は `params=...(never-updated)` でそれを示す
- `CONFIG_BT_CONN_PARAM_RETRY_COUNT` 系は `BT_CONN_PERIPHERAL_PARAM_AUTO_UPDATE` が
  立っているときだけ効く。アプリが自分で `bt_conn_le_param_update()` を呼ぶこの FW では
  そのフラグは立たないので、**再送設定は死にコード**

### supervision timeout をウォッチドッグより長くしてはいけない

`0.0.100` が supervision を 4 s → 8 s に上げたことで、5 s のウォッチドッグより
長くなり、「接続したまま放置すると切れてデバイスが消える」退行が出た。

`bt_gatt_notify()` は ATT PDU を `K_FOREVER` で確保する（`host/att.c`。
タイムアウトが付くのは response と confirmation だけ）。バッファは相手が
LL ACK を返して初めて戻るので、**相手が黙るとメインループがそこで固まる**。

- supervision < ウォッチドッグ … コントローラが先にリンクを切り、バッファが解放され、
  ブロックが解けて餌やりが間に合う（`0.0.98`/`0.0.99` が偶然生き延びていた理由）
- supervision > ウォッチドッグ … **必ずリセットする**。相手からは切断と区別できない

`0.0.102` の不変条件: **`WDT_TIMEOUT_MS`（12 s）> supervision timeout（8 s）**。
加えて制御系の通知は `notify_one()` の専用セマフォ経由にして、
20 ms で枠が取れなければ捨てる（犬に餌をやるスレッドを止めない）。

---

## 4. 自動テストスクリプト（`mac_client/test_recording.py`）

### 概要

XIAO のシリアルポートへのコマンド送信と、Mac/Android の両アプリへの自動接続・録音・認識検証を一括で行うスクリプトです。

```
python3 test_recording.py [A|B|both]   # デフォルト: both
```

### Test A（Handy プライマリ）の流れ

1. Android を **MAC_HANDY モード**（`ble_connection_prefs.xml`）で起動 → BLE 接続を待機
2. Handy を起動 → 800 ms 後に `0x02` 送信してプライマリ宣言
3. Android が `0x31`（peer connected）を受信 → `0x03` 送信してセカンダリに降格
4. `conn_param_work` 発動（5 秒待機して安定化）
5. シリアル `'r'` → `afplay -v 3.0 test.wav`（10 秒）→ シリアル `'s'`
6. Handy の `~/Library/Application Support/com.pais.handy/history.db` を監視して認識テキスト取得
7. 「音声」「入力」「テスト」いずれかが含まれれば PASS

### Test B（Android プライマリ）の流れ

1. Handy は Test A から接続維持したまま継続
2. Android を停止 → **ANDROID モード**で再起動 → `0x02` × 3 でプライマリ宣言
3. Handy が `0x31` を受信 → 何もしない（re-claim なし）
4. `conn_param_work` 発動（5 秒待機）
5. シリアル `'r'` → `afplay -v 3.0 test.wav`（10 秒）→ シリアル `'s'`
6. Android の `shared_prefs/voice_history_prefs.xml` を監視（`isSilent=True` エントリはスキップ）
7. 認識テキストに「音声」「入力」「テスト」が含まれれば PASS

### Android モード切替の仕組み

```python
# MAC_HANDY: 接続後 0x02 を送らず、0x31 受信時に 0x03 を送って降格
android_set_priority("MAC_HANDY")

# ANDROID: 接続後すぐに 0x02 × 3 を送ってプライマリ宣言
android_set_priority("ANDROID")
```

`shared_prefs/ble_connection_prefs.xml` の `connection_priority` キーを書き換えることで実現。`BleManager.kt` の接続コールバックがこの値を参照します。

### SerialConn のバックグラウンド読み取り

シリアルポートへの書き込みと同時に **バックグラウンドスレッドでファームウェアログを読み取り** ます。`ser.get_log()` で蓄積ログを取得でき、問題発生時の診断に利用できます。

```
[XIAO] >>> conn[0] is primary
[XIAO] >>> Primary yielded to conn[1]
[XIAO] >>> conn_param_update[1]: idle(30ms)
[XIAO] >>> send_event 0x01 -> primary[1]
```

---

## 5. 保守上の注意点

### 接続順序とプライマリ割り当て

- **最初に接続したデバイス**が `primary_idx = 0` として自動的にプライマリになります（`ble_connected` コールバック）
- 後から `0x02` を送ったデバイスが **last-write-wins** でプライマリを上書きします
- 両方が `0x02` を送り合うと無限ループになる可能性があるため、`MAC_HANDY` モードでは意図的に `0x02` を送りません

### macOS の AEC（音響エコーキャンセル）

Handy は Mac の内蔵マイクで録音します。`afplay` で同じ Mac のスピーカーから音声を再生すると、macOS の AEC が **スピーカー出力をマイク入力から除去** することがあります。

- 十分な音量（`output volume 100` + `afplay -v 3.0`）にすることで AEC をある程度オーバーライドできます
- 実際の使用環境（ヘッドセット、外部スピーカー）では問題になりません

### Android のドーズモードと BLE スキャン

Android の Doze モード（画面オフ）は BLE スキャンを抑制します。テスト実行前に必ず：

```python
adb("shell", "input", "keyevent", "KEYCODE_WAKEUP")
```

を送信してスクリーンをウェイクアップしてください。

### シリアルポートの再接続

BLE 切断/再接続のタイミングで XIAO の USB CDC ACM が一時的に切断されることがあります。`SerialConn._ensure_open()` が最大 20 秒間ポートの再出現を待機して自動再接続します。

```python
# ポートが消えた場合、最大 10 回 × 2 秒 = 20 秒リトライ
for attempt in range(10):
    time.sleep(2)
    if os.path.exists(self.port):
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        break
```

### PCM パケットサイズと帯域の関係

現在の設定：

| パラメータ | 値 |
|---|---|
| `PCM_PACKET_SIZE` | 200 バイト |
| 音声仕様 | 16 kHz / 16-bit / mono = 32 KB/秒 |
| 必要パケット数 | 160 パケット/秒 |
| `CONFIG_BT_L2CAP_TX_MTU` | 244（DLE 有効） |
| プライマリ接続インターバル | 7.5–15 ms → 67–133 イベント/秒 |

7.5 ms インターバル・200 バイトパケットで **133 × 200 = 26.6 KB/秒** となり、32 KB/秒の音声レートをわずかに下回りますが、実測では VAD・Groq API ともに十分なデータ量を確保できています。

さらに改善が必要な場合は `PCM_PACKET_SIZE` を最大 242 バイト（MTU 244 − ATT ヘッダ 2）まで増やすことができます。
