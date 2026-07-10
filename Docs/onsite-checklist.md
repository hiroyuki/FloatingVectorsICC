# 現地セットアップ・チェックリスト（体験フロー）

Phase 1-7 実装済み。以下は**現地でしか検証・調整できない項目**（Codex レビューの deferred 含む）。

## セットアップ

- [ ] LFKS token を `<persistentDataPath>/lfks-token.txt` に配置（`C:\Users\<user>\AppData\LocalLow\<company>\<product>\lfks-token.txt`。1 行、token のみ）。期限 **2026-10-30**
- [ ] ExperienceConfig アセット作成（`Create > FloatingVectors > Experience Config`）: rigSerialOrder（**どのカメラを 1 番にするか = +X/選択球の並ぶ向き**をここで確定）、floorY（現地キャリブ後に実測 — 前回リグは -0.9）、attractRecordingRoot（現地収録のテイク群）、`dryRunPublish=false`
- [ ] シーンに ExperienceDirector GO を追加し config を割当（HUD の Experience Mode チェックボックスに自動で出る）
- [ ] CameraHealthMonitor の expectedSerials = 現地 4 台の serial
- [ ] アトラクト用テイクを現地で収録（BT が乗る動きのある 1 人テイクを複数）

## 実機検証（deferred 項目）

- [ ] **USB 物理抜き** → 全画面赤アラート + Fault → 再接続 → 復帰
- [ ] live 4 ストリーム + attract playback 同時の USB/GPU 負荷実測（不足なら attract 中の live を depth のみに落とすオプションを検討 — 未実装、必要になったら）
- [ ] 入場遷移（attract → Welcome）の体感 <2 秒（骨格空白は Welcome 文言で隠れる想定）
- [ ] `detectIdenticalContent` を ON にして実データで誤検知が無いか確認（フリーズ USB の再現ができれば検知も）
- [ ] `keepLiveRenderersOnLoad` で再列挙 15 秒が本当に消えていること
- [ ] TSDF 部分バッチ seam（ハンドオフ瞬間の 1 カメラフレーム publish、Codex non-blocking）が見た目に出るか — 出るなら valid<4 のバッチを drop する処理を追加

## チューニング（ExperienceConfig で現地調整）

- [ ] 文言・タイミング（welcome/freePlay/ready/prompt/qrShow 秒数、prompt バリアント）
- [ ] 球の配置（sphereXOffsets / sphereHeight / sphereZ / radius / dwellSeconds）とミニチュアの見た目（displayMiniatureScale）
- [ ] 占有閾値（occupancyThreshold / insetMeters / yBand — 絨毯反射・縁の保護者）
- [ ] Bloom: `DefaultVolumeProfile` intensity（DwellSphere グローは bloom 無しでも視認可。暗環境で判断）
- [ ] QR に載せる URL 種別（qrUrlKind: Usdz / Glb / First — LFKS 側の見え方で決定）
- [ ] SkeletonMerger の crowd 文言/デバウンス（既定 0.5s/1.0s）

## 運用

- [ ] ソークテスト（半日回しっぱなし、メモリ/VRAM/FPS 監視）
- [ ] 失敗アップロードの手動リカバリ: `~/Documents/FloatingVectorsPrints/exp_*.glb/.usdz` が残る → guide の upload.ps1 で手動アップロード
- [ ] upload.ps1 がサーバー側で更新された場合: 再ダウンロード → `sha256sum` → ExperienceConfig の uploadScriptSha256 を更新（不一致のままだと publish が安全側で失敗する）
