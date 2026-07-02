# プラン: アーキテクチャ/命名/GO 階層の整理（Phase 2〜3）

作成 2026-07-02（Phase 1 実施済み・Codex レビュー Round 1 反映で改訂）

## ゴール（why）
コンポーネントの**命名・配置・共有関係**を実態に合わせ、「PointCloud 専用に見えて実は
全系統（PointCloud / Mesh(TSDF) / BodyTracking）の共有インフラ」という違和感を解消する。
併せて系統ごとにバラバラな **Cumulative（累積）系の UI** を統一する。

## 調査で確定した現状認識
- **入力層は既に共有されている（が、誤配置・誤命名だった）**:
  `RawFrameData` + `OnRawFramesReady`/`OnPlaybackRawFrame` イベントを
  PointCloudRenderer / TSDFIntegrator / MeshCumulative / SkeletonMerger /
  TSDFDebugSession / Calibration が消費。
- **ただし SensorManager / SensorRecorder は「純粋な共有インフラ」ではない**（Codex 指摘）:
  SensorManager は `PointCloudRenderer` を spawn し PointCloudCumulative/View を配線
  （SensorManager.cs:207,226）。SensorRecorder は playback の点群再構成・フィルタ・
  playback mesh・cumulative feed を所有（SensorRecorder.cs:1859,1869）。
  → **asmdef 分離（旧 Phase 4）は現構造では成立しない**ため本プランから除外（下記）。
- **機能層は共通化されていない**: 3 系統をまたぐ interface / 抽象基底は皆無。
  - Cumulative: `PointCloudCumulative`（スナップショット積層）と `MeshCumulative`
    （TSDF 時間累積）は**同名だが別の仕組み**。BT 相当は `TSDFTrailBaker` の capture。
  - Clear の意味も三者三様（Codex 指摘）: PointCloudCumulative.Clear は snapshot 削除のみ /
    MeshCumulative.Release は clear+live 復帰 / TSDFTrailBaker.ResumeLive は彫刻破棄+
    double-buffer 復帰+integration 再開+playback 再開（多副作用）。

## ✅ Phase 1 — リネーム（実施済み, commit `ab35a26`）
- `PointCloudRecorder`→`SensorRecorder`, `PointCloudBoundingBox`→`BoundingVolume`,
  `PointCloudCameraManager`→`SensorManager`（+Editor 追随）。GUID 維持で参照無傷。
- 検証済み: error CS 0 / missing script 0 / 相互参照生存 / Play 回帰なし。

---

## Phase 2 — GO 階層再編（共有インフラを `Capture` ノードへ）
シーン `main` の再編。**コード変更なし・シーンのみ**。

```
Capture/                      (新設 root)
  SensorManager               (旧 GO "PointCloudManager")
  SensorRecorder              (旧 GO "Playback&Record")
  BoundingVolume              (旧 GO "BoundingRect")
PointCloud/                   (PC 専用のみ残す)
  PointCloudDecimater / PointCloudView / Cumulative(PointCloudCumulative)
Mesh/
  Volume / Integrator / View / Cumulative / Debug
  TSDFTrailBaker              (root から移動 — D4 決定: TSDFVolume を変異させ TSDF リソースを
                               使うので所有関係を階層で表す)
BodyTracking/, Environment/ …変更なし
```

手順:
1. 事前 grep: `GameObject.Find(` / `transform.Find(` / 文字列 GO 名参照が対象 GO に
   無いことを確認（あれば追随修正）。
2. **親 GO ごと移動する（サブツリー丸ごと）**。ランタイム生成子を個別に動かさない
   （Codex 指摘）: live renderer は SensorManager の**子**として spawn され
   （SensorManager.cs:209）、`_Playback_*` は SensorRecorder の**子**で cleanup が
   **直下走査**（SensorRecorder.cs:1873,2125）。親付け替えのみなら無傷。
3. MCP RunCommand で GO rename + 再親付け → `EditorSceneManager.SaveOpenScenes()`。
4. 検証: missing 0 / 参照生存（TSDFVolume.boundingBox 等）/ Play で auto-play・点群・
   TSDF mesh・BT skeleton・**capture Start/Stop/Resume live** の従来挙動。

## Phase 3 — Cumulative UI 統一（共通 interface + 共通 Editor ヘルパ）
実装は統合しない（積層 vs 融合は別物のまま）。**操作 UI の規約だけ**を揃える。

### 3-1. `IAccumulationController` interface（D2 決定: 状態を持つシーン操作なので
"Accumulator"（数値累算器）より Controller が適切 — Codex 推奨）
新規 asmdef **`Shared`**（参照ゼロ・interface のみ）:
```csharp
public interface IAccumulationController
{
    bool   IsAccumulating { get; }
    string StatusText     { get; }
    bool   CanStart       { get; }   // 既存の Start ゲート差を保持（Codex R2 指摘）
    string StartLabel     { get; }   // 例: MeshCumulative は蓄積中 "Restart"
    void   StartAccumulate();
    bool   CanStop        { get; }
    void   StopAccumulate();
    bool   CanClear       { get; }   // capability: Clear 相当を共通行に出すか
    string ClearLabel     { get; }   // 表示名（意味差を隠さない — D3）
    void   ClearAccumulated();
}
```
実装マッピング（事前に足す API プロパティ — Codex "Missing steps" 反映）:
- `PointCloudCumulative`: `IsAccumulating => noErase` / `StatusText => $"{SnapshotCount} snapshots"` /
  `CanStart => !noErase`, `CanStop => noErase`（トグル動作を分解） /
  Clear = 既存 Clear（`ClearLabel="Clear snapshots"`。**snapshot 子 GO を破壊するので
  Undo は FullHierarchy — 3-2 参照**）。
- `MeshCumulative`: `IsAccumulating => State==Accumulating` / StatusText は
  `State` + `Elapsed`/`Remaining` から生成 /
  **`CanStart => true`（蓄積中も Restart 可 — 既存挙動）**、
  `StartLabel => IsAccumulating ? "Restart" : "Start"` / `CanStop => IsAccumulating` /
  Clear = Release（`ClearLabel="Clear & resume live"`）。
- `TSDFTrailBaker`: `IsAccumulating => IsCapturing` / `StatusText => LastStatus` /
  `CanStart => !IsCapturing`（既存: capture 中は Start 無効）/ `CanStop => IsCapturing` /
  Start/Stop = StartCapture/StopCapture /
  **`CanClear => false` — 共通 Clear は出さず、既存の専用 "Resume live" ボタンのみ残す**
  （Codex R2 指摘: 同一副作用のボタン二重化を回避。D3 の「意味差を隠さない」も
  専用ボタン維持が最も明確）。

### 3-2. 共通 Inspector ヘルパ（`Shared.Editor` asmdef, `Shared` を参照）
interface だけでは Undo/dirty/play-mode ゲートを扱えない（Codex 指摘）ので、
**Object コンテキスト + オプション**を受ける薄いレイアウトヘルパに留める:
```csharp
public static class AccumulationControllerGUI
{
    public enum UndoMode { None, RecordObject, FullHierarchy }
    public struct Options { public bool requirePlayMode; public UndoMode clearUndo; }
    // target は Undo/SetDirty 用。ボタン列（CanStart/CanStop/CanClear ゲート +
    // StartLabel/ClearLabel）+ 状態 HelpBox を描くだけ。
    public static void Draw(UnityEngine.Object target, IAccumulationController c, Options opts);
}
```
- `requirePlayMode`: MeshCumulative/TSDFTrailBaker は true（既存挙動維持）、
  PointCloudCumulative は false（edit 時 Clear を許す既存挙動を維持）。
- `clearUndo`: PointCloudCumulative は **FullHierarchy**（既存 Editor が
  `Undo.RegisterFullObjectHierarchyUndo` を使う — snapshot 子 GO の破壊を戻すため。
  Codex R2 指摘: `RecordObject` では不足）。MeshCumulative/TrailBaker は None
  （playmode 限定のランタイム操作で Undo 対象外 — 既存挙動どおり）。
- 各 Editor（PointCloudCumulativeEditor / MeshCumulativeEditor / TSDFTrailBakerEditor）が
  ヘルパを呼び、固有 UI（手動 bake ボタン・Resume live 等）はその下に残す。

### 3-3. asmdef 配線（Codex "Missing steps" 反映）
- 新規: `Shared`（runtime, 参照ゼロ）/ `Shared.Editor`（`Shared` 参照, Editor only）。
- 追加参照: `PointCloud` と `TSDF` → `Shared`。
  `PointCloud.Editor` と `TSDF.Editor` → `Shared.Editor` + `Shared`。
- BodyTracking は今回対象外（BT の累積は TSDFTrailBaker が担うため）。

検証: 3 つの Inspector が同一 UI 規約で表示 / 各 Start/Stop/Clear が従来挙動
（特に TSDFTrailBaker: capture 中の pause 連動と Resume live 復帰）/
error CS 0 / Play 回帰なし。

---

## スコープ外（旧 Phase 4 の扱い — D1 決定: 今回は実施しない）
namespace/asmdef 分離は**現構造では不成立**（SensorManager/SensorRecorder が PC 描画系を
内包、TSDFIntegrator/SkeletonMerger が PointCloudRenderer に依存）。将来やる場合は
**別プランで設計パスから**（Codex 提案の 2 段階）:
1. 純データ/契約のみ先に抽出（`RawFrameData`・録画フォーマット DTO → `Capture.Core`）。
   その際 **Calibration/RuntimeUI の `PointCloudRecording` 依存も棚卸し**（今回は漏れていた）。
2. `IRawFrameSource` 抽象を導入して TSDF/BT の `PointCloudRenderer` 直依存を切る。
- D6 決定: `PointCloudRecording` の rename は上記実施時に同一コミットで行う。今回は触らない。
- D5 決定: 可視化トグル統一（showPointClouds/showMesh/showBones）は隣接 UX 作業で
  スコープ膨張するため今回やらない。

## 決定事項（Codex Round 1 推奨を採用）
- D1: Phase 4（asmdef 分離）除外 ✅ / D2: `IAccumulationController` ✅ /
  D3: TSDFTrailBaker は共通 Clear を出さず（CanClear=false）専用 Resume live ボタンのみ ✅ /
  D4: TSDFTrailBaker GO は `Mesh/` 配下 ✅ / D5: 可視化トグル統一しない ✅ /
  D6: `PointCloudRecording` 触らない ✅

## 実行順序と運用
Phase 2 → 3。**各フェーズ完了ごとにコンパイル + Play 検証 → コミット**。
着手は本プランの Codex 承認後。

## リスク / 未確定
- GO 名の文字列検索（Find 系）残存 → Phase 2 手順 1 で潰す。
- 新 asmdef 追加によるコンパイル順変化 — Shared は参照ゼロで最小に保つ。
- TSDFTrailBaker は共通行に Clear を出さない（CanClear=false）ため二重ボタンは無いが、
  他 2 コンポーネントと Clear の意味（snapshot 削除 vs clear+live 復帰）が異なる点は
  ClearLabel の文言で明示し続けること。
- 挙動非変更が原則。各フェーズで Rec2_jump 再生の回帰確認（点群 / TSDF mesh / BT / capture）。

## 検証（AI が MCP で実施）
各フェーズ: `RequestScriptCompilation` → console error CS 0 →
missing script 0 / 参照生存チェック → Play（auto-play）で従来挙動 → コミット。
