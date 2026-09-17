# BLE OTA update notes (HarnessNode)

この手順は、XIAO nRF52840 Sense の MCUboot + MCUmgr/SMP over BLE OTA を実行するエージェント向けの必須注意事項です。

## 事前確認

1. 対象は `HarnessNode` であることを確認する。別のBLEデバイスへ書き込まない。
2. `nordic-main/prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` を、実機のslot 0より新しい値へ更新する。同じバージョンや同じハッシュを再送しない。
3. `xiao_ble/nrf52840/sense` と `--sysbuild` を使用する。生成物は `nordic-main/ota_update.bin` とする。
4. OTA中はデバイスの電源を切らず、BLEクライアント（Androidアプリ等）を同時接続させない。
5. 他の BLE クライアント（Android / Handy）が同時接続していると SMP が **0.2 KB/s** まで落ちる。OTA 前に必ず切る。競合したまま張った接続はパラメータが劣化したままなので、切ったあと **OTA 自体をやり直す**。`0.0.100+` の primary idle は 30–50 ms（単独なら 7–12 KB/s）。

## ビルド時の既知問題

- NCSのZephyrキャッシュへ書き込めない場合、`Operation not permitted` や `ToolchainCapabilityDatabase` エラーになる。SDKを変更せず、書込み可能なユーザーキャッシュを使うか、必要な権限を得てから再実行する。ビルド成果物だけを信用してアップロードを続行しない。
- `build_and_package_ota.sh` はパッケージ生成のみで、デバイスを書き込まない。signed imageの存在とサイズ（OTA slot上限以内）を確認する。

## macOSでのOTA実行

- `mac_client/ota_updater.py` のBleak/CoreBluetoothスキャンは、非対話のバックグラウンド実行でmacOSプロセスが `SIGABRT` になることがある。原因はmacOSのTCC（プライバシー）で、`NSBluetoothAlwaysUsageDescription` を持たないプロセスからのBluetooth利用が拒否されるため。OTAはユーザーセッションのTerminalで実行する。同じ理由で `mac_client/tap_monitor.py` や `mac_client/image_state.py` などBLEを使うスクリプトはすべて同様に扱う。
- エージェントから実行する場合は、TerminalのBluetooth許可を継承させるためAppleScript経由で起動する。

  ```bash
  osascript -e 'tell application "Terminal" to do script "cd /path/to/harness-node && mac_client/venv/bin/python3 mac_client/ota_updater.py --device HarnessNode nordic-main/ota_update.bin > /tmp/ota.log 2>&1; exit"'
  ```
- Terminalをエージェントが開いた場合は、ウィンドウまたはタブIDを記録し、アップロードと検証が終わったら必ず閉じる。既存のユーザーTerminalは閉じない。
- 失敗時に同じイメージを無条件で再送しない。まず対象名、接続状態、slot 0/slot 1のversion・hashを読み直す。

## 完了条件

`ota_updater.py` が次の状態を報告するまで完了とみなさない。

- `OTA verified: uploaded image is active and confirmed in slot 0`
- versionが今回の署名バージョンと一致する
- slot 0 の `active=true`、`confirmed=true` を確認する

検証前にUSB抜去、電源断、別ファームウェアの上書きを行わない。

## 更新履歴

- 2026-08-30: 0.0.77（single候補遅延確定によるdouble回帰修正）をビルド・OTA。
  slot 0で`active=true`、`confirmed=true`、version=`0.0.77`を確認済み。
- 2026-08-31: 0.0.81（single/doubleのクールダウン分離）をビルド・OTA。confirmed確認済み。
  ただしタップは復旧せず、実機は0.0.80が載っていたことも判明した。
- 2026-08-31: 0.0.82〜0.0.84 をOTA。タップ経路の診断イベント（`0xD0`/`0xD1`/`0xD2`）と
  RXレジスタポーク（`0x50`/`0x51`）を追加し、実機のレジスタ値と生 `TAP_SRC` を可視化。
  これで真因（1打目での `TAP_SRC` 読み出しによるLIRラッチ解除）を特定した。
- 2026-08-31: 0.0.85〜0.0.87 をOTA。INT1レベル監視＋350ms遅延読み出しへ変更し、
  single 5/5・double 5/5・誤検出0でPASS。slot 0 `active=true` / `confirmed=true` /
  version=`0.0.87` を確認済み。詳細は
  [`nordic_main_guide.md`](nordic_main_guide.md) の「シングル／ダブルタップ」。
- 2026-08-31: 0.0.88（`0xD0` の2秒周期配信を接続時1回へ、`0x40` を受理時にも全接続へ）
  をビルド・OTA。251064 B を50.3秒でアップロード。slot 0 `active=true` /
  `confirmed=true` / version=`0.0.88` / hash=`c65303ff50c6a52e` を確認済み
  （直前のslot 0は`0.0.87` / `3553b743a35377e5`、これはslot 1へ退避）。
  実機検証: `0xD0` の2秒周期配信が消え（12秒で0件、`0.0.87`なら約6件）、`0x40` の
  受理時ack（`pending`付き）と適用時ackが primary / secondary の両方へ届くことを確認。
  ただし**接続時の `0xD0` / `0x40` が接続した本人に届かない**欠陥が判明（下記 `0.0.89`）。
- 2026-08-31: 0.0.89 をビルド・OTA。接続時通知の起点を `audio_tx_ccc_cfg_changed()` へ
  移したが**効果なし**。Zephyr の `gatt_ccc_changed()` は全クライアントの集約値が
  変化したときだけコールバックを呼ぶため、接続ごとの処理には使えない。
- 2026-08-31: 0.0.90 をビルド・OTA。`ble_connected()` は `conn_needs_greeting[slot]` を
  立てるだけにし、メインループが接続ごとに `bt_gatt_is_subscribed()` を見て送る方式へ。
  slot 0 `active=true` / `confirmed=true` / version=`0.0.90` /
  hash=`dc8741b31f8dfb0c` を確認。実機検証: 購読直後に `0x40`（`00554000ff`）と
  `0xD0`（レジスタ値は期待値と完全一致）が1件ずつ届き、以降 `0xD0` の再送なし。
  タップ非退行 single 5/5・double 5/5・誤分類0（10/10 PASS）、録音中のモード切替も
  保留→停止20ms後に適用を確認。

  **OTA中は他のBLEクライアントを必ず切ること。** Androidアプリは `ServiceWatchdog` で
  force-stop から自動復活してNodeへ再接続し、`0.0.90` の初回OTAは 0.2 KB/s まで落ちた
  （通常 7〜9 KB/s）。`pm disable-user` で復活を止め、OTAを張り直したら回復した。
  競合接続がある状態で確立したコネクションはパラメータが劣化したまま戻らないので、
  アプリを止めるだけでなくOTA自体をやり直すこと。

### 調査のコツ

タップ系の不具合は、`RX 0x50` のレジスタポークでOTAなしに実機で条件を振れる。
1回のOTAあたり約2分かかるので、まず `tap_monitor.py --write` で当たりを付けてから
ファームへ反映すると回数を大幅に減らせる。
- 2026-08-31: 0.0.91（IMU軌跡収集の実行時スイッチ）をビルド。`GESTURE_DEBUG_HISTORY` を
  既定 1 にして本番ビルドへ含め、収集は RX `0x06` / TX `0x39` で切り替える方式に変更。
  既定OFFかつRAM保持のみなので、接続時 greeting でも現在値を送る。
  署名イメージ 253799 B、RAM 171 KB / 256 KB（軌跡バッファ 22.5 KB 増）。
- 2026-08-31: 0.0.92（軌跡 flush の並行実行ガード）をビルド。`0.0.91` の実機検証で、
  `process_motion_sample()`（main）と録音停止処理（`audio_thread()`）の2スレッドが
  同時に `flush_trajectory_batch()` に入り、同一ウィンドウを2セッションとして
  交互送信していた（Android受信ログ: END が session=23→22 の LIFO で1ms差、
  チャンクの start が 288,288,296,296,... と重複）。`atomic_cas` の実行中ガードで解決。
  署名イメージ 253943 B。
- 2026-09-02: 0.0.94（single/double notify-only）。single の FW 録音トグルを廃止し、
  ホストが RX `0x01`/`0x00` で start/stop。パススルー中 single は G2 ページ送り専用。
  Android アプリとセットで更新すること。
- 2026-09-04: 0.0.95（ジェスチャー検出スイッチ）。RX `0x07` / TX `0x3A`、既定 OFF（タップのみ）。
  253887 B を 51.9 s でアップロード。slot 0 `active=true` / `confirmed=true` /
  version=`0.0.95` / hash=`e1d126db89819e5f` を確認済み（直前の 0.0.94 は slot 1 へ）。
- 2026-09-04: 0.0.96（タップ閾値 0.5 g → 1.0 g、`TAP_THS_6D=0x10`）。
  253943 B を 52.4 s でアップロード。slot 0 `active=true` / `confirmed=true` /
  version=`0.0.96` / hash=`c3c7ff2e558cdf88` を確認済み（直前の 0.0.95 は slot 1 へ）。
- 2026-09-04: 0.0.97（タップ閾値 +1 段階、`TAP_THS_6D=0x11` ≈ 1.0625 g）。
  253947 B を 1295 s でアップロード（他 BLE 接続競合で低速）。slot 0 `active=true` /
  `confirmed=true` / version=`0.0.97` / hash=`61cc3cd193bf9d82` を確認済み
  （直前の 0.0.96 は slot 1 へ）。
- 2026-09-05: 0.0.98（接続アイドルで BLE 間隔を落とす。未接続 3 分で SYSTEMOFF、
  アドバタイズ中は青の遅い点滅）。255354 B を 51.3 s でアップロード。
  slot 0 `active=true` / `confirmed=true` / version=`0.0.98` /
  hash=`74b078b20791d840` を確認済み（直前の 0.0.97 は slot 1 へ）。
- 2026-09-06: 0.0.99（primary idle を 200–500 ms から 30–50 ms へ。
  `0.0.98` ではタップ〜録音が約 2 秒遅れた）。255390 B を 1248.4 s で
  アップロード（当時の idle 200–500 ms のため 0.2 KB/s）。
  slot 0 `active=true` / `confirmed=true` / version=`0.0.99` /
  hash=`6bc5fe384518e7b2` を確認済み（直前の 0.0.98 は slot 1 へ）。
- 2026-09-08: 0.0.100（supervision 8 s、PPCP を idle と一致、fast は録音中のみ。
  Android の一定時間後切断を修正）。255412 B を 36.2 s でアップロード。
  slot 0 `active=true` / `confirmed=true` / version=`0.0.100` /
  hash=`fa8c4bdffba3b780` を確認済み（直前の 0.0.99 は slot 1 へ）。
  **この修正では直らず**、アイドル切断は `0.0.102` まで残った。
- 2026-09-16: 0.0.101（アイドル切断の真因＝ウォッチドッグリセットを修正）。
  257362 B を 88.1 s でアップロードし slot 1 に格納、test フラグ設定・リセットまでは成功。
  **しかしスワップ後の起動に失敗し、検証が 60 s でタイムアウトした。**
  原因は同梱した `CONFIG_LOG_MODE_IMMEDIATE=y`。`cdc_acm_poll_out()` は TX リングが満杯のとき
  `LOG_WRN_ONCE("Ring buffer full, discard data")` を出すが、即時モードではこれがログ出力経路の
  内部からログを再帰呼び出しする。起動直後はホストがまだポートを開いていないのでリングは必ず
  満杯になり、そこで停止した。当時 `configure_watchdog()` はアドバタイズ開始より後だったため
  犬も armed されず、USB は列挙されるのに無出力・非アドバタイズのまま固まった。
  **CDC ACM コンソールで `CONFIG_LOG_MODE_IMMEDIATE=y` を使ってはいけない。**

  復旧: リセットで Adafruit UF2 ブートローダに入ったので、`merged.hex`（MCUboot＋署名済みアプリ、
  0x27000–0x720d4）に両スロットのトレーラ消去（0xD7000 まで 0xFF）を足した UF2 を作って書き戻した。
  トレーラを消すのは 0.0.101 の test ブートが残した MCUboot のスワップ状態を一掃するため。
  Adafruit ブートローダ（0xF4000）と設定領域（0xD7000）は無変更。

- 2026-09-16: 0.0.102（0.0.101 と同内容 ＋ 事故の再発防止）。UF2 で直接書き戻したため OTA 経由ではない。
  署名イメージ 257596 B、FLASH 76.8%、RAM 67.2%。
  `image_state.py` で slot 0 `active=true` / `confirmed=true` / version=`0.0.102` /
  hash=`84be313f582201c7` を確認済み（セカンダリは消去済みのため一覧に出ない）。
  0.0.101 からの差分:
  - `CONFIG_LOG_MODE_IMMEDIATE=y` を撤回し deferred のまま、`CONFIG_LOG_PROCESS_THREAD_SLEEP_MS=100`
    （1 s → 100 ms）。ハング直前のログは `.noinit` のクラッシュボックスが担うので診断能力は落ちない
  - **`configure_watchdog()` を起動直後（`log_reset_cause()` の直後）へ前倒し。**
    以前は `boot_write_img_confirmed()` の後だったため、初期化中にハングすると犬が armed されず
    永久に停止した。前倒しにより同種の事故は 12 s で自動リセットされ、MCUboot が unconfirmed
    イメージを自分で巻き戻す。起動中の 2 つの待ちループに `wdt_feed()` を追加済み

  実機検証（Android 実接続、16:44〜18:21）:
  - 85 分連続接続、GATT 切断 0 回。メインループからの通知 116 回（ライトスリープ 72 ＋ バッテリ 44）
    がすべて正常に届いた。これは修正前にブロックしていた経路そのもの
  - 18:09 と 18:20 に `status=8`（supervision timeout）で 2 回切断したが、
    **76 秒 / 14 秒で自動再接続**。どちらも直前に連続モーションがあり、装着中の RF 低下と整合する。
    修正前（同日 15:36〜15:49）は切断後 12 分間スキャンし続けてノードを発見できず、
    SYSTEMOFF からタップで復帰させる必要があった
  - 切断が RF ロスかウォッチドッグリセットかは Android 側からは区別できない（どちらも status=8）。
    これを USB なしで判別するため 0.0.103 で起動レポートイベントを追加する

- 2026-09-17: 0.0.103（起動レポート TX `0xD3`）。署名イメージ 258008 B、FLASH 76.9%、RAM 67.2%。
  ウォッチドッグリセットと純粋なリンク断は相手からは区別できない（どちらも `status=8`）ため、
  reset cause と `.noinit` クラッシュボックスを接続ごとに greeting で送る。
  Android アプリは未知の TX イベントを hex で logcat に出すので**アプリ変更は不要**。
  読み方は `nordic_main_guide.md` の「起動レポート `0xD3`」。

- 2026-09-17: 0.0.104（`sysbuild/mcuboot/boards/xiao_ble_nrf52840_sense.conf` の
  `CONFIG_BOOT_WATCHDOG_FEED` を `n` → `y`）。0.0.103 の起動レポートで
  `reset cause: 0x00000012 SOFTWARE | WATCHDOG` が見えたことから発覚：
  nRF52 の WDT はソフトリセットでは止まらず（POR でしか止まらない）、MCUboot は
  スワップ中に犬へ給餌していなかった。256 KB のスワップは 30〜60 秒かかり
  `WDT_TIMEOUT_MS`（12 s）を必ず超えるため、**OTA のたびにスワップ中盤でリセットしていた**
  （スワップはレジューム可能なので更新自体は毎回成功していたが、無駄なリセットと
  スワップ再開が1回挟まっていた）。
  **MCUboot はアプリの OTA では配信できない**（MCUmgr が書くのはアプリのセカンダリスロットのみ、
  MCUboot は `0x27000` に常駐）。UF2 ブートローダ（ダブルタップリセットで起動、
  `XIAO-SENSE` としてマウント）から `merged.hex` の `0x27000`–`0x720d4` ＋ 両スロットの
  トレーラ消去（`0xD7000` まで 0xFF）を書き込んで適用した。
  Adafruit ブートローダ（`0xF4000`）と設定領域（`0xD7000`）は無変更。
- 2026-09-17: 0.0.105（バージョンのみ、0.0.104 の MCUboot 修正を実 OTA スワップで検証するため）。
  0.0.104 → 0.0.105 の OTA を実行し、アップロード完了後 `EVT_BOOT_REPORT`（`0xD3`）を確認：

  ```
  reset cause : 0x00000002  SOFTWARE
  crash_valid : 1
  stage       : 0 (idle)
  prev uptime : 62 s
  ```

  **`WATCHDOG` ビットが消えた。** 0.0.103 での同種の OTA（`0x00000012 SOFTWARE | WATCHDOG`）
  と比べ、スワップ中に犬が吠えなくなったことを実機で確認。OTA のスキャン再試行も
  1 回のみ（以前は複数回）に減った。
