# プラン: コード全体リファクタリング（冗長性・抽象化）

作成 2026-07-07。Assets/Scripts 全 92 ファイル・約 27,600 行を 4 系統
（PointCloud / BodyTracking / TSDF / Orbbec+Calibration+横断）で並列調査した結果の統合。

関連: [plan-architecture-cleanup.md](./plan-architecture-cleanup.md)（命名・GO 階層。Phase 1 実施済み）
とは補完関係。本プランは**コードレベル**の重複排除と抽象化を扱う。

## ゴール（why）

- **重複の解消**: 同じロジックが 2〜4 箇所に手書きコピーされ、既に drift 実績がある
  （例: 点群フィルタの `>=`/`<` 不一致は Codex レビューで再整合済み、
  RecordingGapAnalyzer のパス解決は mac override 欠落中）
- **抽象の適正化**: god class 4 本（SensorRecorder 2333 行 / SkeletonMerger 1802 /
  TSDFPrintExporter 1230 / CalibrationRuntimeUI 1150≒CalibrationWindow 859 の丸コピー）を
  自然な継ぎ目で分割。逆に「統一すべきでない並行構造」も明示する（下記 非ゴール）
- **死コードの削除**: 廃止済み経路・実験終了トグル・参照ゼロの API を落とす

## 大原則

- **動いているインスタレーションの実行時挙動を変えない**。全ステップ「純粋なコード移動 +
  トークン単位の置換」。移動ついでの「改善」は禁止
  （唯一の例外は 3-5 Primary-body — 意図的 policy change として別扱い・オーナー承認必須）
- 1 ステップ = 1 コミット単位。各ステップ後に Unity コンパイル 0 エラー確認
  （Unity MCP）→ 録画データ playback で目視回帰（点群 / TSDF mesh / BT skeleton /
  capture Start/Stop/Resume）
- ホットパス（毎フレーム経路）はアロケーション追加禁止。ヘルパー化は static /
  struct / ref 引数で

---

## Phase 0 — 事前整備（enabler）

**0-1. ラウンドトリップテスト追加**（S・リスクなし）
Assets/Tests/Editor に現状 calibration 系しかない。以降の Phase の安全網として先に書く:
- `RecordedBodySerializer` encode/decode ラウンドトリップ
- body record 1032 バイトレイアウトの golden-bytes テスト（Phase 4-6 の前提）
- 可能なら BuildClusters の合成候補テスト（Phase 4-2 の前提。抽出前は reflection 経由で可）

**0-2. ドキュメント・コメントの実態合わせ**（S・リスクなし）
- CLAUDE.md: `PointCloudRecorder`→`SensorRecorder`, `PointCloudBoundingBox`→`BoundingVolume`,
  `BodyTrackingMultiLive`→`SkeletonMerger` のリネーム未反映を修正
- stale コメント: SkeletonMerger.cs:24-26（Phase 5b 完了済みなのに「これからやる」）、
  K4abtWorkerHost.cs:4-5（multi-cam 済み）、MotionLineRenderer.cs:62（存在しないフィールド参照）、
  PointCloudRecording.cs:10-11（pointcloud_main はもう出力しない）

---

## Phase 1 — 死コード削除・実験終了トグル整理（S 中心・低リスク）

**参照ゼロ判定のスコープ（全項目共通の前提）**: `Assets/Scripts` だけでなく
`Assets/Tests` / `Workers/` / asmdef / scene・prefab の YAML（serialized field 名 grep）
まで確認してから削除する。**scene/prefab で対象フィールドが非デフォルト値なら
その項目は中止**し、オーナーに差し戻す（監査対象: `bandSteps`, `weightStrategy`,
`requireMinWorkerCount`, `dropLowWhenHigherAvailable`, `ignoreRecordedBodies`）。

削除のみ・挙動変化なしと確認済みのもの:

| # | 対象 | 場所 |
|---|------|------|
| 1-1 | `CentroidCalibrationMath.cs` 全体（issue #6 の旧ソルバー、PairwiseCalibrationMath に置換済み）+ `PairwiseCalibrationMath.RecenterOnCameraCentroid`（未参照確認の上）。**注意: `Assets/Tests/Editor/CalibrationMathTests.cs:60 以降が直接参照** — 旧 solver のテストも同時に削除（or Pairwise ベースへ移植）しないとコンパイルが壊れる | Calibration/ + Assets/Tests/Editor/ |
| 1-2 | `PointCloudRecording` の legacy `pointcloud_main` 対応一式（PointcloudSensorName / PointCloudVertexStride / Frame.PointCount / BuildPointcloudHeaderYaml / RcsvStreamWriter.BytesWritten） | PointCloudRecording.cs:41,43,62,638 |
| 1-3 | `K4ACaptureBridge.CreateCaptureWithDepth` / `CreateCaptureFromDepthY16`（worker 化前の残骸） | K4ACaptureBridge.cs:84,151 |
| 1-4 | `BodyVisual` の死コード — **2 段階で実施**。(a) 真に未参照: `_prevPosForJump` / `SetVisible` / `GetSamplePosition`。(b) 診断 window 系（VisibilityToggles / MaxJumpThisWindow 等）は **Dump* が読んでいるため 1-10 の Dump* 削除を先に**行い、その後 re-grep して消す。`LastConfidence` / `LastFreshFrame` / `JointValid` / `JointPosition` は publish 系が使用中 — **削除対象外** | BodyVisual.cs:31,356,371,396-432 |
| 1-5 | SkeletonMerger の実験終了トグル: `dropLowWhenHigherAvailable`（tooltip 自ら「悪化した。One-Euro が本修正」と明記）+ MergeJoint 前処理パス | SkeletonMerger.cs:117-123,1487-1498 |
| 1-6 | `OnFrameUploaded` イベント（リポジトリ内購読者ゼロ、ただし public API）— 即削除せず **`[Obsolete]` 化 → 1 マイルストーン置いて別コミットで削除**（シーン外の operator script / ローカル実験コードからの購読可能性に配慮） | PointCloudRenderer.cs:197 |
| 1-7 | TSDFIntegrator の不要修飾: `unsafe`（ポインタ未使用）/ 未使用 using / `_lastBlockCount` の誤解を招く命名リネーム | TSDFIntegrator.cs:547,10 / TSDFView.cs:488 |
| 1-8 | `debugExtraClearPairs`（計測専用フィールド）を `#if UNITY_EDITOR` 化 or TSDFDebugSession へ | TSDFIntegrator.cs:664-671 |

**オーナー判断済みで削除確定**（2026-07-07 決定 → 下記「判断事項」参照）:

| # | 対象 | 場所 | 注意 |
|---|------|------|------|
| 1-9 | `bandSteps` Inspector フィールド + `_BandSteps` uniform（compute 側が「unused」と明記） | TSDFIntegrator.cs:85-89 | scene が値をシリアライズしていても Unity は silent drop で安全 |
| 1-10 | SkeletonMerger の `DumpJointJumps` / `DumpBoneFreshness` / `DumpPipelineState`（flying bones 調査完結済み、コード呼び出しゼロ） | SkeletonMerger.cs:1102-1202 | 削除。必要になれば git 履歴から復元 |
| 1-11 | `WeightStrategy` enum → linear 固定に畳む。**注意: Squared は tooltip の「placeholder」記載に反して実装済み**（WeightFor, SkeletonMerger.cs:1606）— scene が Squared を選択していたら挙動変更になるため中止ゲート対象 | SkeletonMerger.cs:192-195,1606 | scene/prefab 監査必須 |

**1-12 は削除対象から除外（レビューで棄却）**: `requireMinWorkerCount` は
**main.unity が実際に `2` を設定しており**（Assets/main.unity:1780 付近）、
クラスタ出力条件（SkeletonMerger.cs:1043）と `RequiresApplyExtrinsics()`（:674）の
両方に効いている生きた設定。削除・1 固定は実行時挙動の変更になる。
**現状維持**とし、将来畳みたければ「2 固定への policy change」として
録画 A/B + オーナー承認付きの別タスクに切り出す（→ 判断事項 D5）。

`ignoreRecordedBodies` エスケープハッチは**維持**（→ Phase 4-5 / 判断事項 D2）

**1-13. SlotPool の遅延確保**（S・低リスク・即効メモリ削減）:
GPU モード（デフォルト）では `Slot.Buffer` の NativeArray が一切書かれないのに
3 スロット × maxPoints × 24B ≒ **26.5MB/カメラ**（×4 台）常駐。
確保タイミングをコンストラクタから**初回アクセス時の遅延確保**に変える
（`useGpuReconstruction` を Play 中に toggle しても CPU モード切替時に確保される
経路を保証する。フラグを見て確保をスキップする実装は runtime toggle で壊れるので不可）。
PointCloudRenderer.cs:868,1090-1107

---

## Phase 2 — 共有ヘルパー導入（機械的置換パス・低リスク）

小さな static ユーティリティを作り、既存呼び出しをトークン単位で置換する。
数値・挙動は construction で同一。

**2-1. `GpuBuf`**（Ensure/Release）: `Ensure(ref ComputeBuffer, count, stride, type)` +
`Release(ref ComputeBuffer)`。50 allocation / 53 Release サイト（TSDF 5 ファイル +
BonePoseHistory + PointCloudMotionCurves + PointCloudCumulative + Reconstructor 他）。
「Release 忘れ realloc」リーク class を構造的に防ぐ。TSDFVolume.cs:808-842 の
手書き 13 バッファリストが最大の受益者。（M・低）

**2-2. `TSDFComputeUtil`**: `Resources.Load<ComputeShader>` + FindKernel + エラーログの
定型 11 コピー（約 130 行）を `TryLoad(name, ctx, out cs)` に。（S・低）

**2-3. `TSDFDispatch.Linear`**: 線形化ディスパッチ（65535 制限の gx/gy 分割 +
`_DispatchWidth`）の 5 コピーを統一。TSDFVolume は自前 private 版を 3 メソッドで
無視している状態。（S・低）

**2-4. `Units` / `SensorSpace`** static: `MmToM=0.001f`, `MmToUnity(v)`, `UnityToMm(v)`,
mm↔m スケール行列。現状 `K4AmmToUnity` だけ共有され、**逆変換**が
SkeletonMerger.cs:1587,1763 に手書き ×2、SkeletonWorldTransform.cs:52 に mirror コピー、
ExtrinsicsApply / ToObExtrinsicMm / TSDFIntegrator の Scale(1000) ×3。
符号ミスが致命傷になる領域なので**トークン単位置換のみ・代数変形禁止**。（S-M・中）

**2-5. `ObExtrinsic.ToMatrixMm()`拡張**: Rot[9]+Trans[3]→Matrix4x4 の手組みが
SkeletonMerger.cs:829 / TSDFIntegrator.cs:839 / ExtrinsicsApply.cs:37 の 3 様。
ExtrinsicsApply の手書き Shepperd quaternion は float 丸めが変わるため**現状維持**。（S・低）

**2-6. `OrbbecHandle` 基底クラス**: Context/Device/Pipeline/Config/Frame/Filter の 6 クラスが
Handle + _disposed + Dispose + finalizer + ThrowIfDisposed を verbatim 反復（約 130 行）。
`protected abstract void ReleaseHandle(IntPtr)` + Pipeline 用 `OnBeforeRelease` virtual。
SafeHandle 化は P/Invoke シグネチャが変わるので**今回はやらない**。（S-M・低）

**2-7. `PointCloudUtil`**: `TailSerial`（4 実装）/ `DestroySafe`（Destroy/DestroyImmediate
ガード 15+ 箇所）/ `ConfigureUnlitRenderer`（MeshRenderer 4 行設定 6 箇所）/
`ResolveRoot`（パス解決 3 実装 — GapAnalyzer 版は mac override 欠落の drift バグ持ち）。（S・低）

**2-8. `PointCloudMeshUtil.CreatePointMesh`**: 点群 Mesh 生成定型
（UInt32 index / 100f bounds / 24B 頂点レイアウト / identity index / Points topology）4 コピー。
Unity 6 の「未コミットバッファで compute 書き込みが silent fail」gotcha を 1 箇所に封じる。（M・低-中）

**2-9. `SensorManager.FindRendererBySerial` を public 化**して 3 コピー
（CalibrationWindow / CalibrationRuntimeUI / CameraHealthMonitor）を削除。（S・低）

**2-10. Build postprocessor 共通化**: Orbbec / BodyTracking の 2 postprocessor の
platform-gate + パス解決 + CopyTreeSkippingMeta を共有 Editor ヘルパーに。（S・低）

**2-11. 小粒 dedup**（各 S・低）: BodySnapshot.CopyFrom（2 箇所の pooled-buffer コピー）/
BonePoseHistory リングバッファ walk（×2）/ K4abtWorkerHost の毎フレーム `new string[]`
キースナップショット→再利用 List / joint 数定数の一本化（K4ABTConsts ↔
K4abtWorkerSharedLayout を片方参照に）/ Dictionary GetOrAdd 拡張

---

## Phase 3 — サブシステム内の中規模 dedup（M 中心・中リスク）

**3-1. SensorRecorder 内部 dedup（分割の前準備）**（各 S-M）
- ステップ再生と自然再生のフレーム emit 連鎖統一（SetCursorAndEmit ⇔ Update L1719-1742 の
  ~20 行コピー。コメントが約束する「visually identical」を construction で真にする）
- `ResolveShared<T>` ジェネリック化（5 つの copy-paste Resolve* → 75 行→15 行）
- HandleRawFrame の depth/color/IR 書き込みブロック 3 連 + RecordBodies の 4 つ目を
  `WriteSensorFrame` に（**hot callback path、アロケーション追加禁止**。既存の
  `cb[alloc=…]` 診断行で検証）
- 「timestamp ≤ playhead の最新フレーム」スキャン 4 実装を `AdvanceCursorTo` に
- extrinsics 精度ロジック（save L974-1032 / load L1259-1290）の機構部分を
  `ReadUsableGlobalExtrinsics` + merge 関数に。`IsIdentityExtrinsic` は
  PointCloudRecording へ移動。**実データで前後 YAML 等値チェック必須**
- RCSV preamble パース 3 重化解消 + `ReadRcsv`（全量 materialize、caller 1 件）廃止 →
  GapAnalyzer を `RcsvFrameStream.TimestampNsAt` に移行

**3-2. TSDFPrintExporter 分割**（M・低）: 既に static な純幾何関数群を機械的切り出し —
`MeshOps.cs`（Weld/Taubin/Normals/Simplify + bounds/縮退三角形/winding の 4-5 重複も吸収）、
`StlWriter`/`PlyWriter`（GlbWriter/UsdzWriter の既存慣習に合わせる）、`CurveTubeBuilder`。
Exporter 本体は orchestration + GPU 操作の ~500 行に。オフライン経路なので手動検証可能。

**3-3. Calibration の 450-500 行丸コピー解消**（L・低-中）:
CalibrationWindow ⇔ CalibrationRuntimeUI（RuntimeUI:772 に「ported from
CalibrationWindow」と自白コメント）。UI 非依存の `CalibrationSession` クラスに
capture/solve/reset/dump/estimator/status callback を抽出し、両者を薄い殻に。
オペレータツールなので Capture/Solve 1 回の A/B で検証可。

**3-4. Publish API と per-consumer 状態をフィーダーへ移動**（M・低-中）:
SkeletonMerger 内の velocity-EMA 状態機械（110 行、JointMotionState 一式）は
BodyJointMotionFeeder の仕事。カプセル列挙は BodyTubeCapsuleFeeder へ。
merger から PointCloud 出力型への依存が消え、~150 行減。
その後 2 フィーダーの共通骨格（OnEnable resolve / LateUpdate publish-or-clear /
OnDisable clear）を小基底に（optional）。

**3-5. Primary-body 抽象の導入**（S・中）— **⚠️ 本プランで唯一の意図的挙動変更
（policy change）。「挙動不変」原則の例外として扱い、実施前にオーナー明示承認を取る**:
「single-person」不変条件が「dictionary の最初のエントリ」として 4 箇所で
**互いに異なるセマンティクス**で再実装されている（TryGetJointWorld は IsActive 無視 /
TryReadBonePosesWorld は IsActive 必須 / Publish× 2 は全 body 列挙）。
`BodyVisualPool.Primary`（最新 active、tie-break 最小 id）に一本化。
挙動が変わるのは id フラップで複数 visual が並存する過渡期のみ（＝現状は未定義動作）。
検証: id フラップを含む録画での A/B 再生 + オーナー目視承認を必須とする。

**3-6. ワイヤ可視化 twin の統合**（M・低)>: BoundingVolume ⇔ CameraPoseMarker の
~80% 同一機構（CameraPoseMarker.cs:52 に「Same pattern as BoundingVolume」と自白）を
`WireVisualizationBehaviour` 基底に。~120 行減。デバッグ形状のみ。

**3-7. TSDF capsule-bake dispatch 統一**（S-M・低）: TrailBaker.BakeCore ⇔
PrintExporter.BakeSegs（「Same dispatch pattern as…」コメント）+ 48B TrailSeg 構造体の
二重定義を共有 `BakeSegments` + 単一 struct に。

**3-8. TSDFSmoothUnion の Copy カーネル削除**（S・低）: TSDFCopy と byte 同一ロジック。
`TSDFVolume.CopyBuffers` ヘルパー経由に。

**3-9. interval-gate 抽出**（S・低）: TrailBaker ⇔ MeshCumulative で byte 同一の
バッチ境界ゲート機構（~60 行、同一コメントまで複製）を `BatchIntervalGate` に。

**3-10. GPU raw 画像アップロードヘルパー**（M・中）: scratch-uint + BlockCopy + SetData
イディオムが TSDFIntegrator（コメントで Reconstructor を「参照実装」と明記）+
PointCloudReconstructor + TSDFDebugSession に計 5 変種。`GpuRawImageBuffer` に。
**hot path、既存と同じ realloc 条件・allocation-free を厳守**。

**3-11. IPanelActions の削除**（S・低）: 実装者ゼロの死抽象（D4 で削除決定）。
`Shared/IPanelActions.cs` + FloatingVectorsControlPanel の `_actions` plumbing
（L33, 51, 276）を削除。将来 TSDFPrintExporter にパネルアクションが欲しくなったら
git 履歴から復元して実装すればよい。

---

## Phase 4 — 構造リファクタリング（L・要慎重検証。価値最大・リスク最大）

show-critical 経路。1 継ぎ目ずつ、録画 playback での回帰確認を挟む。
big-bang 分割は**やらない**。

**4-1. SensorRecorder 分割**: 薄い MonoBehaviour ファサード（状態機械 + Inspector/
IRecorderTransport 面）+ 所有 plain class 群: `RecordingSession`（購読 + StreamWriters +
HandleRawFrame）/ `PlaybackEngine`（tracks/cursors/playhead/step/seek — L1747-1769 の
loop-wrap タイミングコメントは移動先でも死守）/ `RecordingMetadataStore` /
`RecorderDiagnostics`（3 つの per-serial 診断 dict を 1 DiagSlot に統合）。
イベント・公開シグネチャ不変。Phase 3-1 完了後に着手。

**4-2. SkeletonMerger 分割**（継ぎ目順に独立着地）:
1. `CrowdAlertOverlay` 分離（`MergedPersonCount` プロパティ追加のみで疎結合化）（S・低）
2. デバッグ可視化 + HUD + BigJump + Dump* を別コンポーネント or partial へ（M・低）
3. マージ中核（Candidate/Cluster/BuildClusters/MergeJoint）を pure class
   `SkeletonMergeCore` に → Phase 0-1 のテストが初めて書ける形になる（M・中）

**4-3. TSDF 統合モード API**: `clearVolumeOnNewBatch` / `integrationEnabled` /
`doubleBuffered` の 3 フラグを 4 クラス（Integrator/TrailBaker/MeshCumulative/
DebugSession）が直接つつき、「Freeze は per-batch clear を復元してはいけない」不変条件が
**3 箇所に同文コメント**で複製されている。TSDFIntegrator に
`EnterLiveFollow()/EnterAccumulate(...)/Freeze()/Resume()` を生やし、
prior-state 退避/復元を内部化。4 caller は intent 呼び出しに縮む。
文書化された freeze/resume シーケンス全てを手動検証。（L・中）

**4-4. TSDFDebugSession 分割 + accumulate 重複解消**: frame cache+replay /
compare bench+smooth-union / accumulate の 4 機能同居。accumulate は
MeshCumulative（or 4-3 の API）への委譲で削除。**production ノブ
`accumulateSmoothK` の駆動をデバッグファイルから accumulation 所有者へ移す**。（L・中）

**4-5. Recorder 状態ルーティングの一元化**: 「この serial の skeleton feed は今どの
ソースか」を 5 箇所が異なる述語（HasTrack / HasRecordedBodies / IgnoreRecordedActive）で
再回答。`SourceMode ResolveSource(serial)` → {LiveWorker, PlaybackBodies,
PlaybackLiveBt, Blocked} に。真理値表を変えない述語抽出のみ。
record→play→loop を実データ検証。（S-M・中）
※ `ignoreRecordedBodies` エスケープハッチは**維持で決定**（D2: bodies_main を持たない
録画を Windows でライブ k4abt 再計算しながら再生する経路として必要）。
代わりに loop-seam blackout + playhead-ahead ガードを `PlaybackLiveBtGuard`
ヘルパーに抽出し、SkeletonMerger のingest 側は `if (_guard.ShouldDrop(tsNs)) return;`
に縮める（S・低）

**4-6. body record codec 一本化**: 同一 1032B レイアウトの field-by-field marshal が
SkeletonOutputWriter（worker 側, Workers/K4abtWorker/）/ K4abtWorkerHost（読み）/
RecordedBodySerializer（disk）の 3 箇所。BodyTracking/Shared に Unity 非依存
`BodyRecordCodec` を新設。**IPC + on-disk 契約なので Phase 0-1 の golden-bytes
テスト必須・byte-identical 出力を assert**。（M・中）

worker 境界の手順（Unity と別ビルドのため必須）:
1. 新ファイルを K4abtWorker.csproj の既存 `<Compile Include>` 機構に追加
2. **codec 本体は unsafe なしの `Span<byte>` ベースで実装する**
   （Editor test asmdef は unsafe 不許可のため、golden-bytes テストから直接呼べる
   形にする）。shared-memory 側の呼び出し元（K4abtWorkerHost / SkeletonOutputWriter —
   どちらも unsafe 許可済み）が `new Span<byte>(ptr, len)` で橋渡しする。
   float bit-cast は `BitConverter.SingleToInt32Bits` 系で unsafe 回避
   （現 RecordedBodySerializer.cs:129 の unsafe cast を置換）
3. little-endian 前提を codec のヘッダコメントに明記（現行実装と同じ前提を明文化するだけ）
4. Windows 側で worker exe を再ビルド → 実カメラ or 録画で
   ライブ BT 1 セッション回して frame 受信を確認してからマージ
   （worker ビルドは Windows 環境が必要 — Mac 単独では完結しない点を作業順に織り込む）

**4-7. RawFrameSourceHub（live/playback 購読の一元化）**: 購読/解除/late-bind +
handler-dict 簿記を 7+ コンポーネントが自前実装（FindObjectsByType 系は
24 ファイル 68 呼び出し）。SensorManager 拡張 or 専用 hub で統一イベント +
renderer 出現/消滅通知（per-serial dict の eviction ヒューリスティック自作問題も解消）。
**イベント順序注意**（K4abtWorkerHost は ExecutionOrder(-100)、pooled buffer 即コピー
規約）。旧イベントを生かしたまま consumer を 1 つずつ移行。（M-L・中）

**4-8. 共有 HLSL 化**:
- 点群フィルタ関数（PassObb/PassCaps/decim hash/sanity）を共有 .hlsl include にして
  PointCloudUnlit.shader / PointCloudShadow.shader / PointCloudCumulativeFilter.compute で
  消費（既に drift 実績あり。CPU 版 FilterForCapture には source-of-truth 参照コメント
  or parity テスト）。FloorOrigin.BuildShadowMpb の PointCloudShaderFilters 手 mirror も
  合わせて composable 分割で解消（「Edit one place」の嘘を直す）。
  **ビット同一が必須 — 見た目が作品出力**（M・中）
- TSDFIntegrate ⇔ TSDFIntegrateDepth の共通 uniform 束縛 ~30 呼び出しを C# 側
  `BindCameraUniforms` に（**C# 半分だけ先行**。HLSL include 統合は useDepthBasis の
  A/B 決着後 — 現在フラグ OFF がデフォルトで意図的並存中）（M・中）

**4-9. MC 抽出の共有オーナー `TSDFMeshExtractor`**: TSDFView と PrintExporter が同一
shader asset を共有し、`_CellZBase` の stale-uniform で**既に実バグを踏んで**双方に
防御リセットが入っている。抽出器がディスパッチ周りで正しく設定し、iso/minWeight も
一元化（exporter の `FindFirstObjectByType<TSDFView>` 逆依存も解消）。
per-frame mesh 経路 — dispatch shape 完全維持、A/B 検証。（M・中）

---

## 非ゴール（意図的にやらない）

- **PointCloudCumulative ⇔ MeshCumulative / PointCloudView ⇔ TSDFView の実装統一**:
  「同じユーザー意図・別のアーティファクト」と header 自身が明文化。共有すべき面は
  既に Shared/ interface 層で共有済み（この層は健全: IViewToggle 4 実装 /
  IAccumulationController 3 実装 / IRecorderTransport は asmdef 分離のため単一実装で正当）
- **OrbbecFrame のプロパティ定型 9 連**: delegate ヘルパー化は hot path アロケーション
  リスク + 「P/Invoke シグネチャはヘッダで確認」ルールを不透明化。現状維持
- **CPU/GPU 連結成分の統一**（TSDF）: 目的が異なる（サイズ閾値 vs 最大成分 + レポート）。
  weight-gate 定数 0.5f の一元化と相互参照コメントのみ
- **SafeHandle 化 / 中央ロガー**: ROI が見合わない。ログは per-class `const string Tag` まで
- **OrbbecException.ThrowIfNotEmpty 57 サイト**: これ自体が共有ヘルパー。それ以上抽象化しない

## 判断事項（2026-07-07 オーナー決定済み）

| # | 問い | 決定 |
|---|------|------|
| D1 | PointCloudRenderer の CPU 再構成経路（SDK 純正 D2C アライン + PointCloud フィルタ経路）は要る？ | **維持**。自作 GPU 経路の検証用リファレンス（SDK が適用するレンズ歪み補正を GPU 経路が落としていた過去バグはこの照合で発見）。メモリ削減は 1-13 の遅延確保のみ実施 |
| D2 | `ignoreRecordedBodies`（live-BT-on-playback） | **維持**。bodies_main を持たない録画を Windows で再生する際に必要。ガードは `PlaybackLiveBtGuard` へ抽出（4-5） |
| D3 | SkeletonMerger の Dump* 3 メソッド | **削除**（1-10） |
| D4 | `IPanelActions` | **削除**（3-11） |
| D5a | `WeightStrategy` / `bandSteps` | **削除・固定化**（1-9/1-11。scene 側の有効化なし確認を先行） |
| D5b | `requireMinWorkerCount` | **現状維持**（Codex レビューで棄却: main.unity が `2` を実運用中 — 削除は挙動変更。畳むなら policy change として別タスク） |

## 実施順序まとめ

```
Phase 0 (テスト+doc) → Phase 1 (死コード) → Phase 2 (機械的ヘルパー置換)
  → Phase 3 (サブシステム内 M 級) → Phase 4 (構造・1 継ぎ目ずつ)
```

- Phase 1+2 だけで概算 **900 行超の削減**、リスクほぼゼロ。まずここまでやる価値が確実
- Phase 3 は互いに独立 — 触る予定のあるサブシステムから順に
- Phase 4 は各項目が独立して着地可能。4-1/4-2 は Phase 3-1/3-4/3-5 完了後、
  4-8 後半（HLSL）は useDepthBasis A/B 決着後
- 各フェーズ完了時に /codex-review でレビュー → main マージ（コンパイル 0 エラー確認必須）

## 検証プロトコル（全 Phase 共通）

1. Unity MCP でコンパイル 0 エラー確認（CLAUDE.md ルール）
2. 録画データ playback: 点群 / TSDF mesh / BT skeleton / capture Start/Stop/Resume の目視回帰
3. hot path を触ったステップは既存診断行（`cb[alloc=…]` 等）でアロケーション非増加を確認
4. shader/HLSL を触ったステップ（特に 4-8 — 作品出力に直撃）は目視スクリーンショットに
   加えて**決定的 A/B**: 代表録画の固定フレームを RenderTexture readback（or
   `AsyncGPUReadback` で頂点バッファ）して before/after のハッシュ or 画素差分を比較。
   スクリーンショット保存先: /Users/horihiroyuki/Dropbox/projects/ICC/capture
5. extrinsics / serialization を触ったステップは実データで byte/YAML 等値チェック
6. serialized field を削除・畳むステップは事前に scene/prefab YAML を grep して
   非デフォルト値がないことを確認（あれば中止 — Phase 1 冒頭のゲート参照）
7. 公開 API（public event / method）の削除は `[Obsolete]` → 別コミット削除の 2 段階
