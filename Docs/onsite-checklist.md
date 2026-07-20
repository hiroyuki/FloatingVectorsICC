# 現地セットアップ・チェックリスト（体験フロー）

体験フローは **1 秒撮影シーケンス**（無人時は床グリッドのみの Idle →
Calibrate 大の字=骨長プロファイル計測 → FreeMove 自由時間 → カウントダウン →
0 から 1 秒収録 → v11s 変換 + capture → 結果表示 + 書き出し → QR）。
詳細は Plans/visitor-sequence-plan.md。
以下は**現地でしか検証・調整できない項目**。

## セットアップ

- [ ] LFKS token を `<persistentDataPath>/lfks-token.txt` に配置（`C:\Users\<user>\AppData\LocalLow\<company>\<product>\lfks-token.txt`。1 行、token のみ）。現地 PC には**本番環境 token**（開発機は開発・テスト token 配置済み）。期限 **2026-10-31 23:59:59**。サーバーは `https://ntticc.lfks.app`
- [ ] ExperienceConfig アセット作成（`Create > FloatingVectors > Experience Config`）: rigSerialOrder（**どのカメラを 1 番にするか = +X の向き**をここで確定）、**visitorRecordingRoot（来場者テイクの書き込み先 — 高速ディスク）**、`dryRunPublish=false`
- [ ] **v11s 変換の実行環境**: `eval/models/`（yolox-m + rtmpose-m ONNX）と `eval/body_profile.json` を本番機に配置。`conversionProvider` は CUDA 環境が入っていれば Cuda（自動で DirectML→CPU にフォールバック）
- [ ] **ライブは LiveFusedBodySource（CUDA）**: シーンで有効化（provider=Cuda、30Hz 実測済）。director が自動検出してライブ骨格 = 融合になる。無効なら k4abt にフォールバック
- [ ] **床高の実測 → `SensorManager.rebaseFloorY` と `SensorRecorder.rebaseFloorY`**（キャリブ座標系での床の y。前回リグは -0.9。rebase 常時 ON なので、これで y=0 = 物理床になる。ExperienceConfig.floorY は 0 のまま）
- [ ] シーンに ExperienceDirector GO を追加し config を割当（HUD の Experience Mode チェックボックスに自動で出る）
- [ ] CameraHealthMonitor の expectedSerials = 現地 4 台の serial
- [ ] （任意）ポーズガイド画像 / SE を差し替え（poseGuideTexture / banzaiGuideTexture / AudioClip 群。未設定ならシルエット + 無音で動く）

## カメラ設定（収録・本番前に必ず）

- [ ] **COLOR の露出/シャッターを上げる** — 速い腕振りの色ブレが RTMPose の stick→catch-up jump の根本原因（depth/ToF 露出は固定、調整できるのは color のみ）
- [ ] **`PointCloudRenderer.trigger2ImageDelayUs` を 160µs × n でスタガー**（N=0 / Z=160 / L=320 / EG=480）— 対向 ToF の IR プロジェクタ干渉（偽 depth のチリチリ幽霊点）対策。sync ケーブルはトリガー共有のみで干渉回避は開発者責務。`GetSyncConfig` で実機反映を確認
- [ ] 三脚などの機材を capture volume の外に出す

## 実機検証（本番機でしか出来ないもの）

- [ ] **大の字ポーズ検知 + 計測**（Calibrate）: 大人・子供で starHold/しきい値の反応、10 秒タイムアウト時のデフォルト続行
- [ ] **骨長プロファイル計測**（Calibrate）: 大の字ホールドで per-visitor プロファイルが適用されるか（ログ `per-visitor bone profile applied`）。タイムアウト時に既定プロファイルで続行するか
- [ ] **Shoot → Processing の体感**: カウントダウン中に収録が始まり、0 から 1 秒で止まるか。変換 + capture の待ち時間（プログレスバー）
- [ ] **Idle の presence**: 入場で Calibrate に進むか、誰もいないとき床グリッドのみで待機し続けるか
- [ ] **Processing の実測時間**（10 秒テイクの v11s 変換。CUDA なら ~15 秒目安、processingTimeoutSeconds=90 で k4abt フォールバック）と、変換中の live 描画のカクつき
- [ ] 再生中の presence（occupancy + live feed）: 途中離脱で Attract に戻るか、QR 表示中の離脱検知
- [ ] **USB 物理抜き** → 全画面赤アラート + Fault → 再接続 → 復帰
- [ ] live 4 ストリーム + playback 同時の USB/GPU 負荷実測
- [ ] 入場遷移（attract → Calibrate）の体感 <2 秒
- [ ] `detectIdenticalContent` を ON にして実データで誤検知が無いか確認
- [ ] `keepLiveRenderersOnLoad` で再列挙 15 秒が本当に消えていること

## チューニング（ExperienceConfig で現地調整）

- [ ] 文言・タイミング（calibrate/freeMove/qrShow/resultMin 秒数、shootCueSeconds、captureSeconds、カウントダウン秒数）
- [ ] 大の字しきい値（star* 系）と profileMinSamplesPerBone
- [ ] playbackRenderDelayFrames（fused 骨格と点群の時刻整合、~4 推奨）
- [ ] 占有閾値（occupancyThreshold / insetMeters / yBand — 絨毯反射・縁の保護者）
- [ ] Bloom: `DefaultVolumeProfile` intensity（暗環境で判断）
- [ ] QR に載せる URL 種別（qrUrlKind: Usdz / Glb / First — LFKS 側の見え方で決定）
- [ ] SkeletonMerger の crowd 文言/デバウンス（既定 0.5s/1.0s）

## 運用

- [ ] ソークテスト（半日回しっぱなし、メモリ/VRAM/FPS 監視。visitorRecordingRoot のディスク残量も — 1 テイク ≈ 数 GB、自動削除は未実装）
- [ ] 失敗アップロードの手動リカバリ: `~/Documents/FloatingVectorsPrints/exp_*.glb/.usdz` が残る → guide の upload.ps1 で手動アップロード
- [ ] upload.ps1 がサーバー側で更新された場合: 再ダウンロード → `sha256sum` → ExperienceConfig の uploadScriptSha256 を更新（不一致のままだと publish が安全側で失敗する）

## dev-sim（カメラ無しでの E2E）

`debugForcePresence` + timings.skipCalibrate + skipFreeMove + skipShoot
（devCannedTakeRoot）+ skipProcessing、`dryRunPublish=true` で**全自動**で 1 周する
（capture は Processing 内で自動 — debug hook 不要）。canned take への実変換は
`allowCannedTakeConversion`（**使い捨てコピーに向けてから**）。
