# 現地セットアップ・チェックリスト（体験フロー）

体験フローは **ポーズ駆動シーケンス**（Calibrate 大の字 → Explore 10 秒収録 →
Processing v11s 変換 → Watch/BanzaiWait ループ再生 → ばんざいで書き出し → QR）。
以下は**現地でしか検証・調整できない項目**。

## セットアップ

- [ ] LFKS token を `<persistentDataPath>/lfks-token.txt` に配置（`C:\Users\<user>\AppData\LocalLow\<company>\<product>\lfks-token.txt`。1 行、token のみ）。現地 PC には**本番環境 token**（開発機は開発・テスト token 配置済み）。期限 **2026-10-31 23:59:59**。サーバーは `https://ntticc.lfks.app`
- [ ] ExperienceConfig アセット作成（`Create > FloatingVectors > Experience Config`）: rigSerialOrder（**どのカメラを 1 番にするか = +X の向き**をここで確定）、attractRecordingRoot（現地収録のテイク群）、**visitorRecordingRoot（来場者テイクの書き込み先 — 高速ディスク、attract とは別フォルダ）**、`dryRunPublish=false`
- [ ] **v11s 変換の実行環境**: `eval/models/`（yolox-m + rtmpose-m ONNX）と `eval/body_profile.json` を本番機に配置。`conversionProvider` は CUDA 環境が入っていれば Cuda（自動で DirectML→CPU にフォールバック）
- [ ] **ライブは LiveFusedBodySource（CUDA）**: シーンで有効化（provider=Cuda、30Hz 実測済）。director が自動検出してライブ骨格 = 融合、再生中のばんざい検知も融合直読みになる。無効なら k4abt にフォールバック
- [ ] **アトラクト用テイクは事前に v11s 変換**（FloatingVectors > Eval BT > FusedBatchConvert か FusedTakeConverter）— `attractUseRecordedBodies=true`（既定）でゴーストは焼き込み bodies を再生する
- [ ] **床高の実測 → `SensorManager.rebaseFloorY` と `SensorRecorder.rebaseFloorY`**（キャリブ座標系での床の y。前回リグは -0.9。rebase 常時 ON なので、これで y=0 = 物理床になる。ExperienceConfig.floorY は 0 のまま）
- [ ] シーンに ExperienceDirector GO を追加し config を割当（HUD の Experience Mode チェックボックスに自動で出る）
- [ ] CameraHealthMonitor の expectedSerials = 現地 4 台の serial
- [ ] アトラクト用テイクを現地で収録（BT が乗る動きのある 1 人テイクを複数）
- [ ] （任意）ポーズガイド画像 / SE を差し替え（poseGuideTexture / banzaiGuideTexture / AudioClip 群。未設定なら棒人間 + 無音で動く）

## 実機検証（本番機でしか出来ないもの）

- [ ] **大の字ポーズ検知 + 計測**（Calibrate）: 大人・子供で starHold/しきい値の反応、10 秒タイムアウト時のデフォルト続行
- [ ] **再生中のばんざい検知**（BanzaiWait）: LiveFusedBodySource が liveFramesOnly でライブの来場者を融合し続けているか（`TryGetLatestFusedWorld` / HasRecentFused。k4abt 構成なら LiveSkeletonFeed の `TryGetBestSkeleton`）。3 ループフォールバックのランダム地点キャプチャも確認
- [ ] **アトラクト中の presence**（v11s ゴースト時は BT がゴーストを見るため、入場検知は occupancy + ライブ融合のみ）: 入場で Calibrate に進むか、誰もいないとき進まないか
- [ ] **Processing の実測時間**（10 秒テイクの v11s 変換。CUDA なら ~15 秒目安、processingTimeoutSeconds=90 で k4abt フォールバック）と、変換中の live 描画のカクつき
- [ ] 再生中の presence（occupancy + live feed）: 途中離脱で Attract に戻るか、QR 表示中の離脱検知
- [ ] **USB 物理抜き** → 全画面赤アラート + Fault → 再接続 → 復帰
- [ ] live 4 ストリーム + playback 同時の USB/GPU 負荷実測
- [ ] 入場遷移（attract → Calibrate）の体感 <2 秒
- [ ] `detectIdenticalContent` を ON にして実データで誤検知が無いか確認
- [ ] `keepLiveRenderersOnLoad` で再列挙 15 秒が本当に消えていること

## チューニング（ExperienceConfig で現地調整）

- [ ] 文言・タイミング（calibrate/explore/qrShow 秒数、banzaiFallbackLoops、カウントダウン秒数）
- [ ] ポーズ検知しきい値（star* / banzai* 系。banzaiMarginArmFraction が個人適応の効き）
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

`debugForcePresence` + timings.skipCalibrate + skipExplore（devCannedTakeRoot）+
skipProcessing、`dryRunPublish=true`。ばんざいは `DebugTriggerBanzai()`
（execute_code）か 3 ループ放置のフォールバック。canned take への実変換は
`allowCannedTakeConversion`（**使い捨てコピーに向けてから**）。
