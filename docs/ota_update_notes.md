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
