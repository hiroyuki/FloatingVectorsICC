# 引き継ぎ: コード全体リファクタリング（冗長性・抽象化）

最終更新 2026-07-08。マスタープラン: [plan-code-refactoring.md](./plan-code-refactoring.md)（Codex 承認済み・全5フェーズ）。
本書はその**実施状況**の記録。次に着手する人はここを起点に、マスタープランの該当 Phase を参照して進める。

## 進捗サマリ

| Phase | 内容 | 状態 | マージコミット |
|-------|------|------|---------------|
| 0 | テスト整備 + doc/コメント修正 | ✅ 完了 | `2fcf5f4` |
| 1 | 死コード削除・実験終了トグル整理 | ✅ 完了 | `2fcf5f4`（Phase 0 と同一コミット） |
| 2 | 共有ヘルパー導入（機械的置換） | ✅ 完了 | `75a944d` |
| 3 | サブシステム内の中規模 dedup | ⬜ 未着手 | — |
| 4 | 構造リファクタリング | ⬜ 未着手 | — |

いずれも `main` にマージ済み。**まだ `origin/main` へ push していない場合はこの Phase 群を最初に push すること**
（本書コミット時点では push 済みのはず — `git log origin/main` で確認）。

## 完了済みの内容

### Phase 0（`refactor/phase0-1-deadcode` → `2fcf5f4`）
- **テスト新設**: `Assets/Tests/Editor/RecordedBodySerializerTests.cs` — bodies_main の
  round-trip + **1032バイト golden-bytes テスト**（8本）。これが Phase 4-6（body record codec 一本化）の安全網。
- CLAUDE.md のリネーム反映（PointCloudRecorder→SensorRecorder 等）、stale コメント4箇所修正。

### Phase 1（同上コミット）
- 死コード削除: CentroidCalibrationMath（+ テストの centroid 分）、legacy pointcloud_main、
  K4ACaptureBridge 未使用2メソッド、BodyVisual 死診断、SkeletonMerger Dump*3本、
  実験終了トグル `dropLowWhenHigherAvailable` / `WeightStrategy`、TSDF bandSteps。
- `OnFrameUploaded` を `[Obsolete]` 化（**次のマイルストーンで別コミット削除予定** — 要対応）。
- **SlotPool 遅延確保**: GPU モードで約26.5MB/カメラ×4台を回収（`EnsureBuffer()` on first use）。

### Phase 2（`refactor/phase2-helpers` → `75a944d`）
新規共有ヘルパーとコールサイト置換（4班並列、実質620行削減）:
- `Shared/GpuBuf.cs` — ComputeBuffer Ensure/Release（14 alloc + 31 release サイト）
- `Shared/Units.cs` — mm↔m 定数 + スケール行列
- `Orbbec/ObExtrinsicExtensions.cs` — `ToMatrixMm()`
- `Orbbec/OrbbecHandle.cs` — 6ラッパーの IDisposable 基底
- `TSDF/TSDFComputeUtil.cs` — TryLoad ×12 + DispatchLinear ×16
- `PointCloud/PointCloudUtil.cs` / `PointCloudMeshUtil.cs` — TailSerial/DestroySafe/unlit-MR/点群Mesh生成
- `Shared/Editor/WindowsBuildCopy.cs` — build postprocessor 共通化
- `BodyTrackingShared.UnityToK4Amm`、`BodySnapshot.CopyFrom`、BonePoseHistory リング走査、
  K4abtWorkerHost の毎フレーム `new string[]` 除去、joint数定数の一本化。
- **副産物のバグ修正**: RecordingGapAnalyzer のパス解決 drift（mac override + デフォルトフォルダ名欠落）。

## 残タスク（次に着手する Phase 3〜4）

マスタープランの該当節を参照。リスク順（低→高）に並んでいる。

### Phase 3（サブシステム内 M 級 dedup — 各項目独立、着手しやすい順）
第1トランシェ完了（2026-07-08、branch `refactor/phase3-dedup`）:
- ✅ **3-11 IPanelActions 削除**（`5bd03b6`）
- ✅ **3-8 SmoothUnion Copy カーネル削除**（`d1e259e` — `TSDFVolume.CopyBuffers` 共有化）
- ✅ **3-9 interval-gate 抽出**（`5da842d` — `TSDF/BatchIntervalGate.cs`）

第2トランシェ完了（2026-07-09、同ブランチ）:
- ✅ **3-7 capsule-bake 統一**（`8db0784` — `TSDF/TrailBakeOps.cs`: TrailSeg/SegStride/BindBakeTargets/BakeSegments）
- ✅ **3-2 TSDFPrintExporter 分割**（PrintExport/ に MeshOps・StlWriter・PlyWriter・CurveTubeBuilder、1221→864行）
  - 未了のオプション: bounds/縮退/winding の 4-5 近似重複（GlbWriter/UsdzWriter/MeshDecimator 側）の MeshOps 吸収は
    byte 同一でないため見送り。やるなら per-case 検証つきで別途。

第3トランシェ完了（2026-07-09、同ブランチ）:
- ✅ **3-6 ワイヤ可視化 twin 統合**（`cdf7d7d` — `PointCloud/WireVisualizationBehaviour.cs` 基底。
  serialized フィールドはサブクラス残置でシーン移行リスクゼロ。runtime 生成/破棄を実機確認済み）

残り（着手しやすい順）:
- 3-3 **Calibration 450行丸コピー解消**（CalibrationWindow ⇔ CalibrationRuntimeUI → CalibrationSession）
  ※注意: Mac 上では ChArUco ボード入り録画が無く Capture/Solve の機能 A/B ができない。
  やるなら「機械的抽出 + トークン等価レビュー」のみで進めるか、Windows 実機で A/B するか要判断。
- 3-1 SensorRecorder 内部 dedup、3-4 publish 状態のフィーダー移動
- 3-10 GPU raw アップロードヘルパー、3-5 **Primary-body 抽象（⚠️ 唯一の意図的挙動変更・オーナー承認必須）**

※関連バグ修正（このブランチ外・main 直行済み）: `8a663d3` — playback path 分離（92c6b6b）が
Mac 再生をサイレントに壊していた件。ResolvePlaybackRoot に macOS フォールバック追加 + シーン設定。

**2026-07-08 前提再検証済み**（上流8コミット: SkeletonMerger continuity/foot-snap、Recorder F9/take分割 後）:
Phase 3 全項目のアンカーは現存・有効。訂正2点 — **3-6 の CameraPoseMarker は `Calibration/` ではなく
`PointCloud/CameraPoseMarker.cs`**。SkeletonMerger は 1906 行・SensorRecorder は 2367 行に成長しており
プラン中の行番号参照はズレている（アンカーはコメント/シンボル名で探すこと）。

### Phase 4（構造リファクタリング — 1継ぎ目ずつ・録画 playback 回帰必須）
- 4-2 SkeletonMerger 分割（CrowdAlertOverlay → デバッグ → SkeletonMergeCore）
- 4-1 SensorRecorder 分割、4-3 TSDF 統合モード API、4-4 TSDFDebugSession 分割
- 4-5 Recorder 状態ルーティング一元化、4-6 body record codec 一本化（golden-bytes テスト活用）
- 4-7 RawFrameSourceHub、4-8 共有 HLSL（**作品出力に直撃・決定的 A/B 必須**）、4-9 TSDFMeshExtractor

## 未解決の宿題（Phase と独立）
- **`OnFrameUploaded` の完全削除**: Phase 1 で `[Obsolete]` 化しただけ。次マイルストーンで別コミット削除。
- **CPU 再構成経路（D1）は維持決定**: SDK-D2C 検証用リファレンスとして残す。削除しないこと。
- **`ignoreRecordedBodies`（D2）は維持決定**: bodies_main を持たない録画の Windows 再生に必要。
- **`requireMinWorkerCount` は現状維持（D5b）**: main.unity が実運用値 `2` を設定中。畳むなら別タスク（policy change）。

## 作業プロトコル（厳守）

1. **1 Phase = 1 feature ブランチ**（`refactor/phaseN-...`）。main から切る。
2. 各ステップ後: Unity MCP でコンパイル0エラー確認（CLAUDE.md ルール）→ EditMode テスト（現状 27本）。
3. **挙動不変が原則**。トークン単位の等価置換のみ。例外は 3-5 Primary-body（オーナー承認必須）。
4. hot path を触ったら既存診断行（`cb[alloc=…]` 等）でアロケーション非増加を確認。
5. shader/HLSL を触ったら決定的 A/B（固定フレーム readback + ハッシュ/画素差分）。
   スクショ保存先: `/Users/horihiroyuki/Dropbox/projects/ICC/capture`。
6. serialized field 削除は事前に scene/prefab YAML を grep（非デフォルト値なら中止）。
7. 完了時: `/codex-review`（code mode）→ APPROVED で main へ `--no-ff` マージ。**push はユーザー明示指示のみ**。
8. 並列 dedup は subagent にファイルスコープを割って投げると速い（Phase 2 実績: 4班並列）。
   その際「読んでから編集・grep で検証・挙動不変」を各エージェントに徹底させること。

## 環境メモ
- Unity Editor が**非フォーカスだと MCP セッションが落ちる**ことがある。テスト実行前にウィンドウをクリックさせる。
- 動作検証の詳細は CLAUDE.md「動作検証はAI自身がMCP経由で進める」節を参照。
- worker 関連（Workers/K4abtWorker/）は Windows でのビルド + 実カメラ/録画検証が必要。Mac 単独では完結しない。
