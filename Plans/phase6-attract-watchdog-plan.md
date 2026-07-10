# Phase 6 実装プラン: アトラクト共存 + カメラ watchdog

Parent plan: `Plans/experience-flow-plan.md`（Phase 1-5 実装済み・main マージ済み）。
**制約**: 現在ライブカメラ未接続 — 実装 + 再生で検証できる範囲を仕上げ、実機必須項目（USB 抜き・live 4 ストリーム + playback 同時負荷・同一データ検出の実データ）は「実機検証残」として明示し Phase 7 現地セットアップ時に実施。

## 設計の親プランからの明確化

親プランの `SetLiveVisualsVisible(bool)` は MeshRenderer.enabled と suppressAsSource を同時切替としていたが、**フェーズにより両者は独立に動く**:
- アトラクト中: live は 非表示 + **suppress ON**（ゴースト playback が彫刻ソース）
- 入場後: live は 非表示のまま + **suppress OFF**（live が TSDF/curves を駆動、TSDF メッシュが主役）
よって API を2つに分離: `SetLiveVisualsVisible(bool)`（MeshRenderer.enabled のみ）と `SetLiveSuppressedAsSource(bool)`（suppressAsSource のみ）。占有検知・BT worker 供給・フレーム取得はどちらにも影響されない（Phase 4 契約どおり）。

## 変更・新規

### 1. `PointCloud/PointCloudRenderer.cs`
- `public bool suppressAsSource`（既定 false）。意味 = 「彫刻系ソース（TSDF 統合・motion curves シード）から除外」。可視性・占有検知・BT には無関係

### 2. `TSDF/TSDFIntegrator.cs`
- `HandleLiveRawFrame`: 先頭で `if (r.suppressAsSource) return;`（dispatch 前 skip。playback 経路は不変）

### 3. `BodyTracking/PointCloudMotionCurves.cs`
- `ResolveSources()` の live renderer 分岐で `suppressAsSource` の renderer を除外（既定 false なので Dev 不変）

### 4. `PointCloud/SensorManager.cs`
- `SetLiveVisualsVisible(bool)` / `SetLiveSuppressedAsSource(bool)`（全 live renderer に適用、現在値を返す/引数のみのシンプル API）

### 5. `PointCloud/SensorRecorder.cs`
- `public bool keepLiveRenderersOnLoad`（既定 false）: `Load()` の `cameraManager.DestroyAllRenderers()` をスキップ（アトラクト⇔ライブの再列挙 ~15 秒を回避する親プラン確定事項）
- `public void StopAndUnload()`: 再生停止 + playback GO/トラック破棄（既存 private teardown の公開。実装時に既存メソッドを特定して委譲 — 新規実装しない）

### 6. `PointCloud/CameraHealthMonitor.cs`
- `public bool IsHealthy => _alerts.Count == 0;` + `public event Action<bool> OnHealthChanged;`（RunCheck の遷移時 fire）
- **suppression 修正 — truth table で確定**（Codex 指摘1対応）:

  | playbackOnly | recorder Playing/Paused | live 台数 | 判定 |
  |---|---|---|---|
  | true | — | — | 抑制（healthy 扱い、従来どおり） |
  | false | Playing/Paused | 0 | 抑制（Dev の playback-only 運用: live 破棄は意図的） |
  | false | Playing/Paused | ≥1 | **監視する**（アトラクト共存 — 本フェーズの目的） |
  | false | 停止 | 0 | **alert**（期待カメラ不在） |
  | false | 停止 | ≥1 | 監視する（従来どおり） |

  検証項目 4 もこの表に合わせて記述を統一（「playback 停止 + live 0 → alert」「playback 中 + live 0 → 抑制」）
- **子供向け文言 API**: `public string KidsAlertText`（最初の異常カメラを「カメラ（ID {n}）が　いじょうです」形式で。director が VisitorMessageUI.ShowAlert に渡す。オペレーター向け詳細文言は既存のまま）
- **同一データ検出**（既定 off: `detectIdenticalContent=false`, `identicalContentSeconds=10`）: live renderer の `OnRawFramesReady` を購読し、checkInterval に1回だけ depth payload をストライドサンプリング（64 点 XOR ハッシュ）。ハッシュが identicalContentSeconds 不変 → 異常（フリーズした USB デバイスは timestamp が進み続けることがある）。実データ検証は実機残
- **購読ライフサイクル**（Codex 指摘3対応）: 購読は Dictionary<renderer, handler> で管理し、`OnDisable/OnDestroy` で全解除、RunCheck ごとに「消えた renderer のエントリ削除 + 新 renderer の購読」を再同期（keepLiveRenderersOnLoad / 再接続で renderer が入れ替わっても重複購読・リークなし）
- 既存 IMGUI バナー: 先頭に `if (Shared.OperatorOverlayGate.AlertActive) return;`（全画面赤アラートと二重描画しない）

### 7. `BodyTracking/SkeletonMerger.cs`
- `public void RestartWorkers()`: 全 worker Stop + `_latestBySerial.Clear()` + gate/prior 状態クリア（次フレームから再 spawn）。**renderer のイベント購読は解除しない**（Codex 指摘4対応 — OnDisable の流用ではなく専用実装。bound renderer からのフレーム供給がそのまま新 worker を spawn する）。アトラクト→入場の録画クロック→ライブクロックジャンプで loop-seam ガードを踏まないため（親プラン確定）

### 8. `Experience/AttractPlaybackController.cs`（新規）
- fields: `sensorRecorder`, `attractRootPath`（テイクフォルダ群の親。空 = 無効）
- `PlayRandomTake()`: ルート直下のディレクトリ列挙（`dataset` サブフォルダ or depth_main を含むものだけ）→ ランダム選択（前回と同じテイクは可能なら回避）→ `recorder.playbackFolderPath = take; recorder.Load(); recorder.TogglePlay();`
- `recorder.OnPlaybackLooped` 購読 → **pending フラグを立てるだけ**（Codex 指摘2対応: このイベントは recorder の Update 内から同期発火するため、その場で StopAndUnload/Load/TogglePlay を呼ぶと `_tracks`/再生状態を再入的に破壊しうる）。テイク切替は AttractPlaybackController 自身の `Update()` で次フレームに実行。`Stop()` = 購読解除 + pending クリア + `StopAndUnload()`
- 列挙失敗/テイク 0 件は warning + 何もしない（アトラクトは文言のみで成立）

### 9. `Experience/ExperienceDirector.cs` / `ExperienceConfig.cs`
- config 追加: `attractRecordingRoot`（既定 ""）
- Enter 追加: `keepLiveRenderersOnLoad=true` を recorder にセット（スナップショット・復元）、`SetLiveVisualsVisible(false)`、PointCloudCumulative があれば noErase を off スナップショット（shadow/PointCloudView 類は Phase 7 仕上げで確認）
- **live 有無で分岐**（開発フォールバック）: `liveAvailable = sensorManager.Renderers に非 null がある`
  - liveAvailable: Attract enter → suppress ON + attract.PlayRandomTake() / Attract exit → attract.Stop() + suppress OFF + merger.RestartWorkers()（~1-2 秒の骨格空白は Welcome 文言が隠す）
  - !liveAvailable（現環境・Mac）: アトラクト playback を止めると彫刻ソースが消えるため **playback を体験中も継続**（Phase 5 E2E と同じ）、suppress 系は no-op + ログ 1 回
- Fault 入力: `debugForceFault || (healthMonitor != null && !healthMonitor.IsHealthy)`。Fault 時の ShowAlert 文言は `healthMonitor.KidsAlertText` 優先
- Exit 復元に上記スナップショット（keepLive フラグ、visuals、suppress、cumulative.noErase）を追加

## 検証（現環境で可能な範囲）

1. コンパイル + 全 46+ EditMode テスト green
2. AttractPlaybackController 単体: RecordingBase ルートでテイク列挙 → Load+Play 成功、ループ時に再抽選が走る（ログ）。テイク 1 件でも動作
3. Director E2E 再走（attractRecordingRoot 設定）: Attract で playback が自動開始 → 入場（forced presence）→ !liveAvailable フォールバックで playback 継続のまま全ループ完走 → Exit で keepLive/visuals/suppress/cumulative 含め全復元
4. HealthMonitor — truth table の 3 ケースを個別に assert:
   - `playbackOnly=true` + live 0 → 抑制（healthy のまま）
   - `playbackOnly=false` + **recorder Playing** + live 0 → 抑制（Dev playback-only 運用）
   - `playbackOnly=false` + **recorder 停止** + live 0 → **alerts 発火** + `IsHealthy=false` + `OnHealthChanged` fire + director が Fault へ + KidsAlertText が赤アラートに出る → 条件解除（playback 再開）で Attract 復帰
5. IMGUI バナーがアラートゲート中に停止すること
6. **実機検証残（Phase 7 現地）**: live 4 + playback 同時の USB/GPU 負荷実測（必要なら attract 中の live を depth のみに落とすオプション検討）、USB 物理抜き → 全画面赤 + Fault、入場遷移 <2 秒、同一データ検出の実データ確認、keepLiveRenderersOnLoad の再列挙回避

## リスク
- 実機依存の未検証項目は上記 6 に集約（コードパスはレビュー + 可能な範囲の再生検証で担保）
- attract テイクの品質（BT が乗る録画か）は運用で選ぶ — コントローラは列挙するだけ
