# Plan: multi-camera skeleton merge (issue #11)

## ゴール

複数 worker (#10) からの skeleton ストリームを世界座標に変換し、同一人物判定 + joint 信頼度重み付け加重平均で 1 体に統合する。`BodyTrackingMultiLive` (新 MonoBehaviour) で集約し、既存の BodyVisual / Trail / MotionLine をそのまま流用する。

完了条件は GitHub issue #11 を参照。

## ブランチ

```bash
git checkout main
git checkout -b feature/issue-11-multicam-skeleton-merge
```

worktree は `.worktree/issue-11-multicam-skeleton-merge/` に置く（プロジェクト memo）。

## スコープ（v1）

- **N カメラ × 1 worker × M 人物まで**。N は静的（Live 中にカメラ追加・削除はしない）
- 同一人物判定は **pelvis world 距離 < 0.3 m**（issue メモの既定値）+ **前フレーム pelvis proximity による継続性ヒント**（軽い persistent ID stabilizer。Hungarian / Kalman は v2）
- joint 統合は **`K4ABT_JOINT_CONFIDENCE_LEVEL` を重み**として加重平均（HIGH=3, MEDIUM=2, LOW=1, NONE=0）
- joint orientation の加重平均は **hemisphere-aligned 加重和 → normalize**（v1）。Markley eigen 法は v2 検討
- **時間整合**: snapshot per worker の `tsNs` と現在時刻の差が `maxSkewMs` (default 50 ms) を超えたら **drop**。これによりゴースト肢を防止
- 1 worker のみ存在する場合は BodyTrackingLive と同じ振る舞い（fallback）
- per-joint Trail / MotionLine は merged 結果に対してそのまま機能（既存 BodyVisual 流用）
- **BodyTrackingLive と MultiLive はシーン上に同時存在禁止**（worker session 競合を防ぐ）。OnEnable で互いを検出して片方を hard-disable + Inspector 警告。fallback として in-process tracker を立てない（コード上、BodyTrackingLive.useWorker=true のときは `TryEnsureTracker` 経路を呼ばないように既に分離済み — MultiLive 側でも同様）

## 前提コード（最初に読むファイル）

| パス | 役割 | 用途 |
|---|---|---|
| `Assets/Scripts/BodyTracking/BodyTrackingLive.cs` | 既存の単一 worker 経路 | UpdateFromSkeleton / EvictIfFull / BodyVisual をそのまま利用 |
| `Assets/Scripts/BodyTracking/K4abtWorkerHost.cs` | 既存の per-serial worker spawn / IPC | `_sessions` Dictionary は per-serial 設計済 → そのまま使う |
| `Assets/Scripts/BodyTracking/BodySnapshot.cs` | OnSkeletonsReady の DTO | id + 32 joints、buffer は host 所有なので必要なら copy |
| `Assets/Scripts/PointCloud/PointCloudCameraManager.cs` | 複数 Renderer の spawn / extrinsics 適用 | `Renderers` リストと `applyExtrinsics` を読む |
| `Assets/Scripts/PointCloud/PointCloudRenderer.cs` | per-camera transform、`OnRawFramesReady` | renderer.transform が world-from-camera-local の代理 |
| `Assets/Scripts/Calibration/ExtrinsicsApply.cs` | OpenCV → Unity basis change | 同じ規約で skeleton 変換に使う |
| `Plans/issue-9-multicam-extrinsic-calibration.md` の「Transform 規約」節 | 座標系の正 | ここから外れない |
| `CLAUDE.md` | プロジェクト規約 | ヘッダ参照必須・MCP 検証等 |

## アーキテクチャ概要

```
PointCloudCameraManager
  ├── PointCloudRenderer A (renderer.transform = world ← cameraA local)
  ├── PointCloudRenderer B
  └── ...

K4abtWorkerHost  (既に per-serial 対応済み)
  ├── session A → k4abt_worker.exe A → OnSkeletonsReady("A", bodies, n)
  ├── session B → k4abt_worker.exe B → OnSkeletonsReady("B", ...)
  └── ...

BodyTrackingMultiLive (新規 MonoBehaviour)
  - cameraManager 経由で N renderers を購読
  - 各 renderer ごとに workerHost.StartWorker(serial, calib, ...)
  - workerHost.OnSkeletonsReady を購読し、per-serial に latest snapshot を貯める
  - Update でクラスタリング + 信頼度加重平均 → BodyVisual 群に投影
```

## 座標系（**Transform 規約に従う**）

`Plans/issue-9-multicam-extrinsic-calibration.md` の Transform 規約節を正とする。要点:

- skeleton joint は worker から **K4A camera-local mm (OpenCV: +x right, +y down, +z forward)** で返る
- 既存 `BodyTrackingLive.K4AmmToUnity(in k4a_float3_t)` で **mm/OpenCV → m/Unity-camera-local** に Y 反転 + /1000 する（再利用）
- 各 worker の serial に対応する renderer の transform が **world ← camera-local** を表現している（#9 で applyExtrinsics 済みなら localPosition/Rotation がセット、`ExtrinsicsApply.ToUnityLocal` 出力と等価）
- **重要**: `PointCloudRenderer` は `transform.localScale.y = -|y|` を Mesh 内点群の OpenCV→Unity 基底変換として使っている (`PointCloudRenderer.cs:595` 周辺)。一方、skeleton 側は `K4AmmToUnity` が既に Y 反転を済ませている。したがって **`renderer.transform.TransformPoint(...)` を skeleton joint に使うと Y が二重反転 → X 鏡像化** する。
  - **正しいやり方**: `renderer.transform` のうち scale を **無視**して位置と回転だけを使う。具体的には:
    1. `Vector3 localPos = renderer.transform.localPosition`
    2. `Quaternion localRot = renderer.transform.localRotation`
    3. `Matrix4x4 worldFromUnityLocal = parent.localToWorldMatrix * Matrix4x4.TRS(localPos, localRot, Vector3.one)`
       - parent が無ければ `Matrix4x4.TRS(localPos, localRot, Vector3.one)`
    4. `worldJoint = worldFromUnityLocal.MultiplyPoint3x4(K4AmmToUnity(jointMm))`
  - これは `ExtrinsicsApply.ApplyToTransform` が renderer に書いた pos/rot を取り戻す動作と等価
- 1 worker のみ・extrinsics 未適用の場合 (`renderer.transform.localPosition == 0`, `localRotation == identity`) は world 変換が identity となり、結果的に `BodyTrackingLive` と同じになる

新ヘルパー:

```csharp
// In a new utility (e.g. BodyTracking/MultiCam/SkeletonWorldTransform.cs).
public static Vector3 ToWorld(in k4a_float3_t jointMm, Transform rendererTransform);
public static Quaternion ToWorldRotation(in k4a_quaternion_t orient, Transform rendererTransform);
```
- jointMm: worker からの値（mm, OpenCV）
- rendererTransform: 対応するカメラの PointCloudRenderer.transform
- 内部で K4AmmToUnity → `worldFromUnityLocal.MultiplyPoint3x4` を実施。**scale 経路は通さない**

EditMode テスト（Phase 1 で必須）:
- (a) identity transform で K4AmmToUnity 同等
- (b) +1m 並進 extrinsic で joint world が +1m シフト
- (c) **localScale.y=-1 を持たせた renderer に対して通常の TransformPoint と比較し、X が鏡像にならないこと**（regression check）
- (d) parent transform 階層がある場合の合成チェック

## 段階的実装

### Phase 1: 座標変換ユーティリティ（半日）

1. `Assets/Scripts/BodyTracking/MultiCam/` ディレクトリを作る（meta も）
2. `SkeletonWorldTransform.cs`:
   - `ToWorld(k4a_float3_t jointMm, Transform rendererTransform) → Vector3`
   - `ToWorldQuat(k4a_quaternion_t orient, Transform rendererTransform) → Quaternion`（後述: joint orientation も world で保持しておくと、将来回転 IK 用途に役立つ）
   - 単体テスト用 EditMode テスト（Assets/Tests/Editor 配下に追加）:
     - identity transform で K4AmmToUnity 同等であること
     - 既知の extrinsic + 既知 joint で世界座標が想定どおりであること（X 鏡像にならない）
     - 1 m 並進した extrinsic で joint world 座標が +1 m シフトすること

### Phase 2: BodyTrackingMultiLive scaffold（1 日）

1. `Assets/Scripts/BodyTracking/BodyTrackingMultiLive.cs` (MonoBehaviour) を新設
2. Inspector フィールド:
   - `PointCloudCameraManager cameraManager`
   - `K4abtWorkerHost workerHost`
   - 既存 BodyTrackingLive と同じ visual 系（jointRadius / showAnatomicalBones / trail 設定 / maxBodies / unseen frames など）。共通設定は **`BodyVisualConfig` 構造体に切り出して** Live と Multi で共有する（diff 最小化）
3. ライフサイクル:
   - `OnEnable`: cameraManager 経由で renderers を取得（Play 開始直後は空かもしれないので、binding は Update で再試行）
   - **renderer ごとに**: `workerHost.StartWorker(serial, calib, ...)` を初回 raw frame で呼ぶ。各 renderer の `OnRawFramesReady` を購読
   - `workerHost.OnSkeletonsReady += OnWorkerSkeletons` を **1 度だけ**購読（per-serial に振り分けは handler 側）
   - `OnDisable`: 各 renderer の StopWorker、購読解除、BodyVisual 破棄
4. Per-serial latest snapshot 保持:
   - `Dictionary<string, LatestSnapshot>` で `{ frameId, bodies[], count, capturedAtFrame }` を持つ
   - `OnSkeletonsReady` は host 側 buffer の使い回しなので handler 内で **コピー**（id + 32 joints × { pos, quat, conf } を pre-allocated 構造へ）
   - 1 worker あたり MaxBodies (=6) ぶんの再利用 buffer を保持

### Phase 3: per-frame マージ（1 日）

`BodyTrackingMultiLive.Update` で:

1. **時間整合フィルタ**: 全 worker の latest snapshot を集めるとき、`now - snapshot.tsNs > maxSkewNs` (default 50 ms) の snapshot は **drop**。残らなかった worker は今フレーム参加しない
2. **Person association**:
   - 各 worker の各 body について pelvis (= JOINT_PELVIS) の world 座標を計算（`SkeletonWorldTransform.ToWorld`）
   - **Greedy clustering with previous-frame continuity**:
     1. 前フレームの各 merged person の pelvis world 位置を保持しておく
     2. **継続性パス（1-to-1）**: 前フレームの person を **持続信頼度（前フレームの cluster 内最大 confidence）降順**にソート。各 prior person について、まだどの cluster にも入っていない worker-bodies の中から最も近い 1 個（< `continuityRadiusMeters`、default 0.5 m）を採用して seed とする。さらに同じ条件を満たす他 worker の bodies を全て 1 cluster に取り込む。**取り込まれた worker-body は consumed フラグを立て、別 prior や新規パスで二重消費されないようにする**
     3. **新規パス**: 残った worker-bodies（consumed=false）の中で信頼度最高の body を seed にして greedy で `mergeRadiusMeters` (0.3 m) 内を 1 cluster。これも consumed フラグで重複防止
   - cluster 同士は重ならない（1 worker の同じ body は 1 cluster だけに入る、consumed フラグで保証）
   - merged body の **ID 型と振り方**:
     - 既存 `BodyVisual` / `BodySnapshot` / `ApplyBodySkeleton` API は `uint`。**v1 でも `uint` のままで一貫**（k4abt の body id も uint なので合わせる）。wrap 周回（2^32）は 1秒 30 ID 採番でも 4.5 年掛かるので運用上 irrelevant
     - 継続性パスで引き継いだ person はその `uint` ID を維持
     - 新規パスは `_nextMergedId++` で小さい単調増加 uint を採番
3. **Joint 統合（per joint）**:
   - cluster 内の各 worker × body の同 joint について、`(level == NONE ? 0 : (int)level)` を重みに加重平均（v1 Linear、Inspector で Squared に切替可）
   - 全 worker の confidence == NONE の joint は無効扱い（visual で hide / 直前 frame の値を保持）
   - **position**: weighted sum / total_weight
   - **orientation (quaternion)**: hemisphere-aligned weighted sum:
     1. cluster 内で **最高信頼度の sample を reference quaternion** にする
     2. 他 sample について `dot(q, qRef) < 0` なら `q = -q`（同半球に揃える）
     3. weighted component-wise sum → normalize
     4. v1 はこれで充分。fast rotation で破綻が見えたら Markley eigen 法 (v2)
   - merged confidence は cluster 内最大値（visual 的にも妥当）
4. 結果を `k4abt_skeleton_t`（既存型を流用）に詰めて、既存 `ApplyBodySkeleton(uint mergedId, in k4abt_skeleton_t)` 経路を呼ぶ
   - `ApplyBodySkeleton` は `BodyTrackingLive` から **共通 helper として移動 / 抽出** する（Phase 5 で実施）。それまでは BodyTrackingMultiLive 内に複製

### Phase 4: 信頼度モデルと閾値（半日）

issue メモを最終ルール化:

```csharp
public enum JointConfidence
{
    NONE = 0, LOW = 1, MEDIUM = 2, HIGH = 3,
}

// 重みは `(int)level`。NONE=0 は加重平均から除外。
// Phase 6 検証で「信頼度を非線形にした方が安定」が分かったらここを pow(2,level) などへ調整。
```

Inspector で公開:
- `mergeRadiusMeters` (default 0.3) — 新規 cluster 形成半径
- `continuityRadiusMeters` (default 0.5) — 前フレーム person との同一視半径
- `maxSkewMs` (default 50) — snapshot のタイムスタンプ skew 上限
- `requireMinWorkerCount` (default 1; 2 にすると単独 worker での検出を捨てる)
- `weightStrategy` (Linear / Squared) — まず Linear だけ実装

`stalenessFrames` は廃止（`maxSkewMs` が代替）。

### Phase 5: BodyVisual 共有 + diagnostic（1 日、**2 段階に分ける**）

BodyVisual の抽出は 1 ステップでやると trail clearing / SetActive 制御 / confidence gating の挙動が変わる罠が多い。**コミット 2 段階**で進める:

**Phase 5a — copy-only extraction（behavior change ZERO）**
1. `Assets/Scripts/BodyTracking/BodyVisual.cs` を新ファイルとして作り、`BodyTrackingLive.BodyVisual` の中身を **逐字コピー**（namespace は同じ `BodyTracking`、internal sealed class）
2. `BodyTrackingLive` 側は nested class 削除、新 top-level type を参照するだけ
3. **Phase 5a のテスト**: BodyTrackingLive を 1 cam in-process / 1 cam worker 双方の経路で 30 秒走らせ、Editor.log の `[BodyTrackingLive]` per-second 診断ログのカウンタが Phase 1 前と統計上同等であること（pelvis_jump 統計が突然壊れていない）
4. このコミット段階では `BodyVisual` の API は private/internal のまま、外部クラスは追加しない

**Phase 5b — BodyVisualPool 抽象**
1. `BodyVisualPool` を新設し、以下を集約: `_bodies` dict, `_lastSeenFrame`, `_seenThisFrame`, `EvictIfFull`, GC pass、共通 `ApplyBodySkeleton(uint id, in k4abt_skeleton_t skel, BodyVisualConfig cfg)`（**id は uint で統一** — Phase 3 の merged ID も uint なので型不整合なし）
2. `BodyTrackingLive` が `BodyVisualPool` を内部で持つ形に refactor
3. `BodyTrackingMultiLive` も同 Pool を持って merge 結果を投げる
4. **Phase 5b のテスト**: 同上の regression テストに加え、merge 結果が Live 単独と pelvis_jump 統計上等価（or 良化）であること

**Diagnostic ログ (per-second)**
- per-worker (host 側既存ログを流用): snapshots received, bodies seen
- merge レイヤー (新規):
  - `clusters_formed/s`, `persons_output/s`, `avg_cluster_size`
  - `dropped_stale_snapshots/s`（`maxSkewMs` 超過分）
  - `merge_time_ms_p50 / p99`（per-frame 計測 → 1 秒窓で集計）
  - `continuity_carry_over_count/s`（前フレーム ID 引き継ぎ件数）

既存 `BodyTrackingLive` の挙動は無変更（regression 無し）。

### Phase 6: 検証（1 日）

CLAUDE.md ルール: **動作確認は AI 自身が MCP 経由で進める**。下記をすべて Unity MCP コマンドで完結する。

**A. コンパイル / static チェック（gating: 失敗で先に進まない）**
- `UnityEditor.Compilation.CompilationPipeline.RequestScriptCompilation()` + `AssetDatabase.Refresh(ImportAssetOptions.ForceUpdate)` 後に `Unity_GetConsoleLogs(Error)` が 0 件
- Phase 1 の EditMode テスト一式が green（X 鏡像 regression test 含む）

**B. ランタイム検証（Play モード）**
1. **1 worker fallback**: BodyTrackingMultiLive を 1 カメラだけで動かして、BodyTrackingLive と同じ位置に skeleton が出ることを確認
   - 同シーンに両者居ないことを OnEnable ガードが報告
   - per-second 診断ログで pelvis_jump 統計が Live 単独と統計同等
2. **2 worker normal case**:
   - 1 人 / 2 cam で merged skeleton が生成され、体がブレなくなる（pelvis_jump_avg < per-cam pelvis_jump_avg / 1.4 を期待）
   - 2 人 / 2 cam で 2 person 検出され、入れ替わりが起きないこと（continuity_carry_over_count/s が `persons_output/s` に近い）
3. **Occlusion case**:
   - 1 人 / 2 cam の片方だけが見える状態にして、見えてる側だけで skeleton が継続すること（dropped_stale_snapshots/s が増えるが persons_output/s は維持）
4. **calibration がズレてる場合**:
   - わざと extrinsic.yaml を 30 cm ずらして読み込ませ、merge radius (0.3 m) で別人扱いされる挙動を確認（persons_output/s ≈ 2 × per-cam bodies/s）
5. **時間スキューフィルタ**:
   - 片 worker を 100 ms ストール（GPU stress 等）させ、`dropped_stale_snapshots/s` が立つこと、merged skeleton がゴーストにならないこと
6. **パフォーマンス**:
   - `merge_time_ms_p99` < 2 ms (Unity main thread)
   - 全 visuals 更新後の Update 時間が in-process / 単独 worker と統計同等
7. **Editor 強制終了** で全 worker が parent watchdog (#10) で自殺すること（複数 worker でも同じ）
8. **Domain reload (script edit during edit-mode)** で全 worker が綺麗に終了→ Play で再 spawn

**C. CLAUDE.md 準拠チェックリスト**
- [ ] `Unity_GetConsoleLogs` で Error / Warning 新規追加なし（既存の OnValidate 警告等は除く）
- [ ] スクリプト変更後 ExitPlaymode → recompile → EnterPlaymode を MCP で完遂
- [ ] Editor.log を `tail | grep` で per-second 診断ログを確認（MCP Console は古いセッション混在のリスクがあるため、定量数値は Editor.log を正とする）
- [ ] worker exe ビルドが `dotnet publish -c Release -r win-x64` で警告 0
- [ ] 入力 sanitize: serial を MMF / event 名へ渡す前に `K4abtWorkerSharedLayout.Sanitize` 経由（既存実装）であることを確認、新規 path で素通しが無いか grep

## オープン課題

1. **Persistent ID 強化**: v1 は前フレーム pelvis proximity による弱い継続性のみ。2 人がクロスする等の難ケースは Hungarian / Kalman を v2 で。Trail Reset しきい値 (0.4 m) も併せて見直し
2. **Quaternion mean の精度**: hemisphere-aligned 加重和は近接姿勢では十分だが、worker 間で姿勢推定が大きくズレる時は Markley eigen 法に上げる必要あり。Phase 6 でジッタが目立てば switch
3. **`requireMinWorkerCount=1`** のときに BodyTrackingLive を完全代替できるか確認（自動 disable はしない、ユーザーがどちらか 1 つを enable する運用）
4. **applyExtrinsics の必要性は条件付き**: `Renderers.Count == 1`（および `requireMinWorkerCount == 1`）のときは renderer.transform が identity でも問題なし（K4AmmToUnity 経由で BodyTrackingLive と等価動作）。**複数カメラ時 (`Renderers.Count > 1` or `requireMinWorkerCount > 1`) のときのみ** `cameraManager.applyExtrinsics == true` を要求する条件付きガードを OnEnable に置く。違反時はエラー報告 + disable
5. **Worker arg sanitize**: serial を MMF / event 名に渡す経路で、新規追加コードが既存の `K4abtWorkerSharedLayout.Sanitize` を素通りしないか実装中に確認

## 完了の目安

issue #11 Done 条件すべてチェック:
- 各 worker skeleton が world に変換される ✓
- 同一人物 merge ✓
- joint 信頼度加重平均 ✓
- BodyTrackingMultiLive 動作 ✓
- Trail / MotionLine も merged 上で動く ✓

加えて:
- 1 worker fallback で BodyTrackingLive と等価
- パフォーマンス劣化なし
- CLAUDE.md ルール準拠（MCP 自動検証 / コンパイルエラー無し / k4a / k4abt 経路ルール）

## 後続

- issue #12 (仮): persistent body ID トラッキング、Hungarian / Kalman で人物識別を frame-to-frame で安定化
- issue #13 (仮): 統合 skeleton をネットワーク配信（OSC / VMC など外部出力）
