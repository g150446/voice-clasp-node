# BLE OTA update notes (HarnessNode)

この手順は、XIAO nRF52840 Sense の MCUboot + MCUmgr/SMP over BLE OTA を実行するエージェント向けの必須注意事項です。

## M5StickC Plus2 (`HarnessNode-Plus2`)

ESP-IDF dual-OTA + **同じ SMP BLE UUID / `ota_updater.py`** で更新できる。

1. `stickc_plus2/VERSION` を上げる。
2. `./stickc_plus2/build_and_package_ota.sh` → `stickc_plus2/ota_update.bin`
3. パーティションを初めて dual-OTA にした直後は **USB `idf.py flash` 一回**が必要。
4. Terminal で:
   `python3 mac_client/ota_updater.py --device HarnessNode-Plus2 stickc_plus2/ota_update.bin`
5. 完了条件は nRF と同じ: slot 0 `active` + `confirmed`、version 一致。

イメージ形式は ESP app bin（MCUboot signed ではない）だが、SMP の upload/state/reset はホスト互換。

## M5StickC Plus SE (`HarnessNode-PlusSE`)

Plus2 と同じ SMP / `ota_updater.py`。ビルドは **`-B build-plus_se`**（4 MB、PSRAM なし）。Stick は **BtnA で `ADV`** にしてから OTA。Handy は切る。FTDI 初回 USB は **115200**。

1. `stickc_plus_se/VERSION` を上げる。
2. `./stickc_plus_se/build_and_package_ota.sh` → `stickc_plus_se/ota_update.bin`
3. 初回は USB flash（115200）。
4. Terminal で:
   `python3 mac_client/ota_updater.py --device HarnessNode-PlusSE stickc_plus_se/ota_update.bin`
5. 完了条件は Plus2 / nRF と同じ。

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
  osascript -e 'tell application "Terminal" to do script "cd /path/to/voice-clasp-node && mac_client/venv/bin/python3 mac_client/ota_updater.py --device HarnessNode nordic-main/ota_update.bin > /tmp/ota.log 2>&1; exit"'
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

- 2026-09-02: StickC Plus2 `0.1.3` をビルド・OTA（635728 B）。slot 0 `active=true` /
  `confirmed=true` / version=`0.1.3` / hash=`1c675e46d9df677b9a…` を確認済み。
  その後の実機スイープで **PDM の音は LEFT スロット**（IDF クロック使用時）と判明し、
  `MIC_PDM_DEFAULT` を `I2S_PDM_SLOT_LEFT` + gain ×4 に変更して USB フラッシュした。
  この LEFT 版は `VERSION` を上げないまま焼いたため、**実機は 0.1.4 相当のコードを
  `0.1.3` というラベルで載せている**。リポジトリ側は `0.1.4`。次回の OTA は
  `0.1.4` 以降で行うこと（同じ 0.1.3 に別バイナリが 2 つ存在する状態を解消する）。
  詳細は [`stickc_plus2_guide.md`](stickc_plus2_guide.md) の「マイク処理」。
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

- 2026-09-17: 0.0.106（UF2 ブートローダへ再起動するコマンド）。署名イメージ 258464 B、FLASH 77.1%。
  RX `0x60 0xB0 0x07` / シリアル `'b'` で `GPREGRET=0x57` を立てて `sys_reboot()` する。
  MCUboot の書き換えに毎回ダブルタップが要る問題への対処
  （直前の 0.0.104 で 30 分以上を空費したため）。ack は TX `0xD4`、録音中は拒否。
  ホスト側は `mac_client/enter_bootloader.py [file.uf2]` でコマンド送信からマウント検知・
  書き込みまで一括。

  実測（ブートローダ `0.6.2-12-g459adc9`）:
  - **USB 必須**。VBUS が無いとブートローダは DFU モードに留まらずアプリへ素通りする
    （`0xD3` の reset cause が `SOFTWARE` のみ ＝ 犬に殺されたのではなく素通り）
  - USB があればダブルタップ不要でマウント。**60 秒経っても維持**され、
    ブートローダが自分でウォッチドッグに給餌していることを確認
  - UF2 書き込みは 1.44 MB で 3.6〜16.5 秒。16.5 秒かかった回もあるので、
    もし給餌されていなければ 12 秒の WDT には間に合っていなかった
  - 誤爆防止: `0x60` 単体 / `0x60 0x00` / `0x60 0xB0 0x08` はいずれも無応答・無反応
  - 録音中に送ると `0xD4` の `accepted=0` で拒否され、再起動しない

- 2026-10-04: 0.0.107（加速度FIFOによるソフトウェアsingle/doubleタップ判定）をBLE OTA適用。
  署名イメージ259596 B。直前のslot 0は0.0.106。更新後slot 0の
  `version=0.0.107` / `active=true` / `confirmed=true` /
  hash=`626e8ff1482e71a0`を確認。ログ: `/tmp/harness_ota_0107.log`。
  合成波形17ケースとsense＋sysbuildのビルド検証済み。実装の実機タップ精度・電流は未測定。

- 2026-10-05: 0.0.108（短い衝撃の段階的な立ち上がりを見逃す問題を修正）をBLE OTA適用。
  署名イメージ259627 B。更新前slot 0は0.0.107。更新後slot 0の
  `version=0.0.108` / `active=true` / `confirmed=true` /
  hash=`df5266a8c33997c1`を確認。ログ: `/tmp/harness_ota_0108.log`。
  合成波形21ケースとsense＋sysbuildのビルド検証済み。実機タップ精度の再試験は別途行う。

- 2026-10-05: 0.0.109（タップ判定のサンプル数・候補・棄却理由・加速度ピークを読む診断コマンド）をBLE OTA適用。
  署名イメージ260268 B。更新前slot 0は0.0.108。更新後slot 0の
  `version=0.0.109` / `active=true` / `confirmed=true` /
  hash=`ea8bef4d365d1d93`を確認。ログ: `/tmp/harness_ota_0109.log`（終了コード0）。
  診断はRX `0x52` / TX `0xD5`で要求時のみ送信。実機タップ試験は準備確認後に行う。

- 2026-10-05: 0.0.110（Z方向の立ち上がり後に生じる横方向ピークによる棄却を廃止）をBLE OTA適用。
  署名イメージ260220 B。更新前slot 0は0.0.109。更新後slot 0の
  `version=0.0.110` / `active=true` / `confirmed=true` /
  hash=`bf0c49e842444bc7`を確認。ログ: `/tmp/harness_ota_0110.log`（終了コード0）。
  合成波形24ケースとsense＋sysbuildのビルド検証済み。実機single/double・振動耐性は再試験が必要。

- 2026-10-05: 0.0.111（最大8秒の加速度・検出器状態保存とBLE経由の一時設定変更）をBLE OTA適用。
  署名イメージ262563 B。更新前slot 0は0.0.110。更新後slot 0の
  `version=0.0.111` / `active=true` / `confirmed=true` /
  hash=`5faec2a57ab1a592`を確認。ログ: `/tmp/harness_ota_0111.log`（終了コード0）。
  操作不要の実機チェックで416サンプルを取得、FIFO欠落0・飽和0、全サンプルで
  本番CコードによるPC再生と実機判定状態が一致。ログ: `/tmp/harness_tap_lab_0111_check.log`。
  実機タップ波形の採取・条件調整は準備確認後に行う。既定条件は0.0.110と同じ。

- 2026-10-05: **0.0.112（通常／デバッグ分離とCPU待機処理の省電力化）をビルド。OTA未実行。**
  起動時は通常運用、RX `0x55` / TX `0xD8`で接続所有のデバッグを有効化。所有者切断・10分経過・
  明示解除・再起動で通常へ戻る。通常では416 Hzのタップ検出を維持し、診断集計・波形／履歴・詳細ログを停止。
  FPUとコンテキスト共有を有効化、音声停止中とUSB受付の周期ポーリングをイベント待ちへ変更。
  `xiao_ble/nrf52840/sense`＋sysbuildの最終ビルド成功。署名版`0.0.112+0`、
  署名イメージ264079 B（slot上限335872 B）、image hash=`5a72957697041df4`。
  アプリFLASH 263416 B、RAM 205440 B / 256 KB。最終ビルドのアプリC警告・エラーなし。
  MCUboot設定の既存USBログレベルchoice警告は継続する。
  静的スタックの単一関数フレーム: main 816 B、音声capture 680 B、tap pipeline／detector各96 B。
  FPU追加コンテキスト72 Bを考慮し、既存stack（main/audio 4096 B、capture 2048 B、serial 1024 B）を維持。
  数値は呼び出し全体の最大使用量ではなく、実機high-watermarkは未測定。
  FPU命令`vsub.f32`／`vmul.f32`等とhard-float ABIを確認。tap detectorオブジェクトに
  `__aeabi_f*`の未解決参照はない（double演算は従来どおり別途存在する）。
  判定24ケース、通常／デバッグ途中切替の内部状態・イベント一致、診断集計停止、セッション所有・期限・切断、
  PCツール10テスト、gesture validator self-testがPASS。保存済み8波形の全19968サンプルflagsが一致。
  本版の実機BLE操作・録音・FPUタップ・電流比較は未実施。デバイスのactive/confirmed確認も未実施で、
  最後に確認済みのOTA版は引き続き0.0.111。操作方法は`nordic_main_guide.md`の通常運用／デバッグ節。

- 2026-10-11: 0.0.113（**ダブルタップ精度改善: 判定を加速度ベクトルの大きさに統一**）をBLE OTA適用。
  署名イメージ264031 B、アプリFLASH 263368 B、RAM 205440 B / 256 KB。
  更新前slot 0は0.0.111。更新後slot 0の`version=0.0.113` / `active=true` / `confirmed=true` /
  hash=`f942c948ffac8023`を確認（セカンダリslot 1は0.0.111へ退避）。
  アップロード49.2 s（5.2 KB/s、競合BLEクライアント無し）。ログ: `/tmp/ota_113.log`（終了コード0）。
  真因と修正は`diagnostics/tap/2026-10-11/analysis.md`参照。旧判定はボードZ軸支配の
  `dominance`ゲートと`max(az,lateral)`幅で、衝撃が横軸に出る斜め姿勢の1打目を棄却し
  doubleをsingleへ誤分類していた。重力除去後の加速度**大きさ**でピーク・エッジ・quiet・幅を
  測る姿勢／方向不変モデルへ変更し、`dominance`は予約化（ワイヤ互換で保持、未使用）。
  既定は従来どおり`max_pulse_ms=40 ms`へ**ビルド反映**（従来のRAM限定100 ms回避を撤去）。
  検証: 保存済み8波形を本番Cリプレイで新既定40 ms全正解（45度double3本を含む）、
  C単体26ケース・PCツール5テストPASS。0.0.112の通常／デバッグ分離とFPU省電力化を含む。
  実機held-out（45度・5試行）: **45度 double 3/3 が double**（報告の本丸＝斜め姿勢の
  double→single誤分類は解消）、通常の腕の動きは誤検知なし。**1試行のみ激しい手首の
  jerk が double に化けた**。±4 g で本物のタップ（最大3.996 g）と jerk（3.997 g・clip 2サンプル）が
  ともに天井へ張り付き、飽和・幅・立ち上がり・極性のいずれでも分離不能（飽和棄却は本物の
  タップも巻き込み退行→撤回）。詳細 `diagnostics/tap/2026-10-11/analysis.md`。

- 2026-10-11: 0.0.114（**フルスケール ±8 g 化の実験**）をBLE OTA適用 → **却下・0.0.115で巻き戻し**。
  `main.c` `sensor_g_to_ms2(4→8)` と `TAP_RAW_TO_G 0.000122→0.000244`。実機held-out（45度）で
  本物のタップは ±8 g で **非クリップ 5.6–7.6 g**（0.0.113でクリップした原因が範囲不足と確定）、
  通常の動きは ≤0.9 g で誤検知なし。ただし**同じ加速度センサがジェスチャ経路を駆動**しており、
  `GESTURE_LIFT_ACCEL_MIN_MS2=0.40` m/s²（≈0.041 g）等が **±8 g のLSB 0.244 g 未満＝分解能外**で
  手のひら起こし／静止検出が壊れる。**peak_g を上げても激しい jerk（≥4 g・タップと同値域）は
  排除できず**、±8 g は破棄した。OTA verified（slot0 0.0.114 active/confirmed）後に 0.0.115 へ差し替え。

- 2026-10-11: 0.0.115（**±4 g へ巻き戻し＋tap判定は大きさモデル維持**）をBLE OTA適用。
  署名イメージ264032 B。更新前slot 0は0.0.114、更新後slot 0の`version=0.0.115` / `active=true` /
  `confirmed=true` / hash=`7c69940e391338ba`を確認（セカンダリslot 1は0.0.114）。
  挙動は 0.0.113 と同一（斜めdouble解消・ジェスチャは±4 g分解能を回復）。
  **既知の限界（許容）**: 限界まで振り抜いた手首jerkのみ誤検知しうる（単一IMUの物理限界、
  tapと区別不能）。通常の反復・運転振動の統計保証は未取得。

- 2026-10-11: 0.0.116（**省電力: 切断後のSYSTEMOFF入眠を30分→5分**）をBLE OTA適用。
  署名イメージ264031 B。更新前slot 0は0.0.115、更新後slot 0の`version=0.0.116` / `active=true` /
  `confirmed=true` / hash=`fbabb408bc068ea1` を確認（セカンダリは0.0.115）。
  動機: 0.0.107からソフトウェアtap判定が `MAIN_LOOP_INTERVAL_MS=25` msでIMU FIFOをread+分類し続け、
  **接続有無にかかわらず起床中は眠れない**。切断後30分は毎回このフル稼働窓で、試験直後の電池減りの
  主因だった。`ADV_AFTER_DROP_SLEEP_MS`（main.c）を5分に短縮（cold boot/タップ起床後は従来どおり3分、
  `ADV_NO_CONN_SLEEP_MS`）。トレードオフ: 5分超の切断はタップで起床が必要。
  USB接続中は入眠しない仕様（`unconnected_since_ms`固定）とawake中の25 ms poll床は変更せず、
  恒常的な床を下げるには未接続時INT1起床化（Phase 2）が必要。実機の入眠/起床検証は次段。
- 2026-10-26: 0.0.117（BLE広告名を `HarnessNode` から `ClaspNode` へ変更。機能の変更なし）をビルド・OTA。
  先にアプリと `mac_client` を新旧両名対応にしてから更新した（`ota_updater.py` の更新後の検証は
  どちらの名前でも探す）。264027 B を 50.9 s（5.1 KB/s）でアップロード。slot 0 `active=true` /
  `confirmed=true` / version=`0.0.117` / hash=`c10f047c536cc3ac` を確認済み（直前の 0.0.116 は
  slot 1 へ）。更新後、Android が**同じ MAC**（`CF:78:E0:AC:05:04`）で `ClaspNode` として再接続し、
  優先デバイスの照合も通った。
