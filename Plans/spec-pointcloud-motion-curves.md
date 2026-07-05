# 実装仕様書: 点群 × BTボーンによる「動きの曲線」(Point-Cloud Motion Curves)

> as-built 仕様。設計意図・不採用案の議論は [[plan-pointcloud-motion-curves]] を参照。
> 本書は **実装済みコードが実際に何をしているか** を記述する（2026-07-05 時点 / main `7fb1cf3`）。
> 対象コミット: `9843d2e`（CurveSamples 再設計）までを反映。

---

## 0. 概要

点群の各頂点（seed）から、その点が属する身体部位（BTボーン）が **直近 K フレームに描いた軌跡（曲線）** を伸ばし、群として「動いている感じ」＝**動きの彫刻**を作るライブエフェクト。

- 完全 GPU 経路・リアルタイム。CPU への読み戻し無し。
- 精度は演出用途（ビタビタ不要）。1人インスタレーション前提（first body のみ）。
- 既存の `TSDFTrailBaker`（Start/Stop で焼く静的スカルプチャ）とは別物。こちらは動きに追従して各点から曲線が伸縮するライブエフェクト。

### データフロー

```
SkeletonMerger (Update: スケルトン更新, world 座標)
   │  TryReadBonePosesWorld(freshWindowFrames)
   ▼
BonePoseHistory (LateUpdate: 端点ペア+安定フレームをリング蓄積 → GPU 公開)
   │  HistoryBuffer / CountBuffer / RingLength / CurveSamples
   ▼
PointCloudMotionCurves (次フレーム Update: 1フレーム遅延で消費)
   │  ① CSCollect: 点群頂点 → bbox内・有効点を compact
   │  ② CSBuild:   seed分類→パラメータ化→履歴再投影→Catmull-Rom→line strip
   ▼
MotionCurves.shader (DrawProceduralIndirect: 各セグメントを幅付きリボンへ)
```

座標空間は全て **Unity world (m)** で統一。seed はメッシュローカル→`localToWorldMatrix` で world 化。ボーンは `SkeletonMerger` が既に world。

---

## 1. 構成要素

| ファイル | 種別 | 役割 |
|---|---|---|
| `Assets/Scripts/BodyTracking/BonePoseHistory.cs` | MonoBehaviour | 全ボーンの端点ペア `(A,B)` ＋安定フレーム `(U,V,W)` をリングバッファ保持し GPU 公開。`Shared.IPanelTunable`（Control Panel に "Motion history" を露出）。 |
| `Assets/Scripts/BodyTracking/PointCloudMotionCurves.cs` | MonoBehaviour | seed 収集、compute dispatch、indirect 描画、バッファ生存管理、Inspector パラメータ。`Shared.IViewToggle`（Views パネルに "Motion lines"）。 |
| `Assets/Scripts/BodyTracking/Resources/MotionCurvesBuild.compute` | ComputeShader | `CSCollect`（点群 compact 前処理）＋ `CSBuild`（分類＋パラメータ化＋履歴再投影＋Catmull-Rom→line strip）。 |
| `Assets/Scripts/BodyTracking/Resources/MotionCurves.shader` | Shader (Built-in RP) | line strip の各セグメントをカメラ向きリボン（2三角/6頂点, 幅 `_Width`）へ展開。`Orbbec/MotionCurves`。 |

いずれも **BodyTracking アセンブリ**に置く（`BonePoseHistory`(BT) と点群メッシュ(PointCloud) の両方に依存し、BodyTracking→PointCloud の参照は既存・逆は循環になるため）。

---

## 2. BonePoseHistory — ボーン姿勢履歴

### 2.1 なぜクォータニオンを使わないか
`SkeletonMerger.TryGetJointWorld` は関節の **world 位置のみ**を返す（orientation/confidence 非公開）。端点方向だけからボーン回転を作ると **軸まわりの roll がフレーム毎に不定**で曲線がねじれる。→ 姿勢は使わず、**端点ペア＋明示的に安定化したフレーム**で曲線を再構成。

### 2.2 Sample（1フレーム分のボーン記録）
```
struct Sample { Vector3 A, B;      // world 端点 (a→b)
                Vector3 U, V, W; }  // 安定直交フレーム; U = normalize(B - A)
```
- `U` = 主軸（端点方向）
- `V` = 副軸 = 参照軸（親ボーン方向, 無ければ torso-up）を `U` 垂直面へ射影・正規化
- `W` = `U × V`
- **副軸の degenerate 対策**（`PickPerp` が `U` から ~20° 以内の参照を拒否）: 参照軸 → torsoRight → torsoFwd → prevV → world up → world right の順に fallback。脊椎ボーン（torso-up と平行）が roll する問題を、体幹の左右軸を優先することで回避。
- `torsoUp` = `neck - pelvis`（無ければ world up）。`torsoRight` = 肩span（無ければ腰span）を torsoUp 垂直へ射影。

### 2.3 リングバッファと CurveSamples 再設計 ★重要
**リング長は常に `MaxK = 32` 固定**。`historySamples` はリングをサイズせず、**描画時に「最新の何フレームをカーブに使うか」を選ぶ** draw-time パラメータ。

```csharp
private const int MaxK = 32;   // リング長 = compute の MAXK = per-curve 制御点上限
public int CurveSamples => Mathf.Clamp(historySamples, 2, MaxK);
```

- `EnsureBuffers` は `boneCount` 変化時のみ再確保（`historySamples` 変化では再確保しない）。
- 効果: 一時停止中でも `historySamples` を上下すると、**既に蓄積済みの32フレームから即座にカーブが伸縮**する（再確保・クリア・再充填なし）。
- 旧仕様（`historySamples` でリングをサイズし、K 変化時に resize-in-place で newest を保存）は廃止。約60行の resize ロジックが不要になった。

CPU リング: `_ring[bone]`(長さ MaxK), `_head[bone]`(newest スロット), `_count[bone]`(有効数 ≤ MaxK)。oldest = `(_head - _count + 1)` wrap。

### 2.4 取り込みタイミング（LateUpdate）
`SkeletonMerger.PoseVersion` を監視し、**新フレーム到来時のみ** サンプルを push：

- **新フレーム**（play / frame-step）: `UpdateHistory()` で全ボーン更新（invalid/stale はリセット）→ `PublishGpu()`
- **静止 + 非 pause**（body が停止/退場）: `ClearStale()` で freshness 切れボーンをリセット（重複サンプル追加なし。ゴースト残留を防ぐ）→ `PublishGpu()`
- **静止 + 意図的 pause**（`recorder.IsPaused`）: リングを hold（frozen sculpture を維持、関節が古くなっても collapse/premature clear しない）

`freshWindowFrames`（既定12）: 端点の最終 fresh BT フレームがこれより古いと stale 扱いでボーン履歴をリセット（held/predicted 関節のドラッグを防止）。

### 2.5 GPU 公開
| バッファ | 型 | 長さ | 内容 |
|---|---|---|---|
| `HistoryBuffer` | `StructuredBuffer<Sample>` (60B) | `boneCount * MaxK` | per bone `[b*K + i]`, **oldest→newest**, 有効分のみ書き込み |
| `CountBuffer` | `StructuredBuffer<uint>` | `boneCount` | ボーンごとの有効サンプル数 |
| `RingLength` | int | — | K (= MaxK = 32) |

`PublishGpu` は毎 LateUpdate（reset フレーム含む）でリング→scratch→`SetData`。新規 `GraphicsBuffer` は未定義値なので、確保直後に count=0 で zero-init（consumer が初回 publish 前に dispatch しても out-of-bounds を防ぐ）。

### 2.6 その他 API
- `SampleCurve(bone, t, cv, cw, dst)`: CPU 側再投影（Scene gizmo 用）。**注意**: 現状 `_count[bone]`(最大32) 分を返し `CurveSamples` を尊重しない → gizmo は GPU 描画より長く出ることがある（描画本体には影響なし。§7 参照）。
- `TryGetWorldBounds`: 全ボーン newest 端点の world AABB。seed を身体周辺へ集中させるのに使用。
- `SampleCount(bone)`, `BoneCount`。

---

## 3. PointCloudMotionCurves — seed・dispatch・描画

### 3.1 seed source
- `sourceMeshes` を明示指定、空なら `autoSourcePrefix`（既定 `_Playback_`）で始まる MeshFilter を毎フレーム自動収集（playback 経路）。
- **使用可能条件** (`IsUsableSource`): 頂点バッファ 0 が `GraphicsBuffer.Target.Raw` で作られ、stride が **24B**（`ObColorPoint` = pos12 + col12）であること。満たさないメッシュ（CPU/live mesh 等）は GPU バインドエラー回避のため除外。

### 3.2 ① CSCollect（前処理・compact）
1スレッド/source頂点。world 化 → sanity（`|coord| > sanityRange` は無効深度 dummy として drop）→ bbox（bone AABB + `bboxPadding`）外を drop → 生存点の `pos + 元RGB` を `InterlockedAdd` で `_CollectOut` に append。source メッシュごとに1回 dispatch し1バッファに累積。
→ seed 予算全体を（大半が背景の全点群でなく）**身体上に集中**させる。

### 3.3 ② CSBuild（分類→再投影→line strip）
1スレッド/seedスロット。

1. **seed 取得**: compact 済み点を `srcIdx = (seedLocal/_SeedCount) * collectCount` で等間隔ストライド。
2. **分類（soft-assign）**: 各ボーンの newest frame で中心線距離 `d`（point-to-segment）と半径 `radius = lerp(radiusA, radiusB, t) * radiusScale` から表面距離 `surf = d - radius`。ソートキー `key = max(surf,0)*1000 + d`（rank 優先・中心線 tie-break）の昇順 top-N を選択（N = `boneBlendCount`, 1..4）。
3. **背景カリング**: 最近傍ボーンの `surf > surfaceMargin` なら degenerate（不可視）。
4. **重み付け**: `w[j] = exp(-max(surf,0) / blendSigma)`。各選択ボーンの newest frame で `t, cv=dot(rel,V), cw=dot(rel,W)` を算出。
5. **カーブ長**: `m = min(minCnt, _CurveSamples, MAXK)`。最新 m サンプルを使用（各ボーンで `(cnt - m) + k` オフセット）。
6. **再投影**: `x_k = Σ_j (w_j/wsum) * (lerp(A_k,B_k,t_j) + cv_j*V_k + cw_j*W_k)` を `pts[0..m-1]` に。ボーンが回転すれば `U,V,W` が変化 → `x_k` が弧を描く。
7. **Catmull-Rom**: centripetal(α=0.5) で制御点間を `_Subdiv` 分割 → line strip（endpoint ペア）を固定スロット `slotBase` に書き込み。未使用尾は zero-fill。

### 3.4 出力バッファと描画
- `_outBuf`: `StructuredBuffer<LineVert>` (pos+col, 24B), 長さ `seedCount * (K-1) * subdiv * 2`。
- `_argsBuf`: indirect args。vertex 数 = `segments * 6`（リボン: 1セグメント→quad→2三角/6頂点）。
- `MotionCurves.shader` が `SV_VertexID` からセグメント／コーナーを復元し、`axis = b-a` とカメラ方向から幅 `_Width` のカメラ向き quad を生成。degenerate（zero-length）セグメントは画面外へクリップ。
- 描画: `Graphics.DrawProceduralIndirect(MeshTopology.Triangles)`, shadow off。

---

## 4. 標準半径テーブル

毎フレームの体格キャリブは廃止。`DefaultRadius` が joint ペアから **絶対値 m** の半径（parent側 `ra`→child側 `rb` テーパー）を静的に決め、`radiusScale`（Inspector, 0.2–4）で来場者の体格差を手動吸収。

| 部位 | ra | rb |
|---|---|---|
| 体幹/脊椎 | 0.18 | 0.16 |
| 鎖骨 | 0.09 | 0.07 |
| 上腕 | 0.07 | 0.06 |
| 肘→前腕 | 0.06 | 0.05 |
| 前腕→手首 | 0.05 | 0.04 |
| 頭クラスタ | 0.09 | 0.08 |
| 腰(pelvis→hip) | 0.12 | 0.10 |
| 太もも→膝 | 0.10 | 0.08 |
| すね→足首 | 0.07 | 0.05 |
| 足 | 0.05 | 0.04 |
| その他 | 0.06 | 0.05 |

---

## 5. パラメータ（Inspector / Control Panel）

| 名前 | 場所 | 既定 | 意味 |
|---|---|---|---|
| `historySamples` | BonePoseHistory / Panel "Motion history" | 12 | カーブが張る最新フレーム数 K (2..32)。draw-time 選択、pause 中も即反映 |
| `freshWindowFrames` | BonePoseHistory | 12 | stale 判定窓 |
| `seedCount` | MotionCurves | 20000 | seed 総数（バッファ容量） |
| `radiusScale` | MotionCurves | 1.0 | 半径全体倍率 |
| `surfaceMargin` | MotionCurves | 0.05 | 背景カリング閾値 (m) |
| `bboxPadding` | MotionCurves | 0.25 | seed 収集 bbox の余白 (m) |
| `boneBlendCount` | MotionCurves | 3 | ブレンドする最近傍ボーン数 (1=hard) |
| `blendSharpness` | MotionCurves | 0.02 | ブレンド falloff σ (m) |
| `smoothSubdiv` | MotionCurves | 4 | Catmull-Rom 分割数 (1..8, 頂点数に比例) |
| `brightness` | MotionCurves | 1.0 | 色の明度倍率 |
| `ribbonWidth` | MotionCurves | 0.006 | リボン幅 (m, 0=細線) |
| `freeze` | MotionCurves | false | ハード hold（再構築せず最後のカーブを描き続ける） |
| `sanityRange` | MotionCurves | 100 | 無効深度判定 (m) |
| `visible` | MotionCurves / Views "Motion lines" | true | 表示 on/off |

**freeze vs pause の違い**: playback を pause すると BonePoseHistory がポーズを hold するので、**再構築は続くがポーズは固定** → paused フレーム上で blend 系パラメータを比較調整できる。`freeze` はパラメータ変更も無視して最後のカーブを凍結（ライブ本番でスカルプチャを固定する用途）。

---

## 6. ライフサイクル / 生存管理

- `BonePoseHistory`: `HistoryBuffer`/`CountBuffer` は `OnDisable`/`OnDestroy` で `Release`。`boneCount` 変化でのみ再確保。
- `PointCloudMotionCurves`: `_outBuf`/`_argsBuf`/`_radiusA/B`/`_collectBuf`/`_collectCounter` を保持。`seedCount`/`ringLen`/`boneCount`/`subdiv` 変化で seed 系を再確保、collect は source 総頂点数へ grow-only。`mesh.GetVertexBuffer(0)` は取得毎に `Dispose`。source/history 不在時は draw せず（エラーにしない）。

---

## 7. 既知の制約 / follow-up

1. **`SampleCurve` gizmo が CurveSamples を尊重しない**（Codex NON_BLOCKING）: Scene ビューの gizmo は最大32フレーム分を描き、`historySamples` の短縮が gizmo に反映されない。**GPU 描画パスには影響なし**（gizmo は `drawGizmos` で off 可）。
2. **stale コメント**: `BonePoseHistory.cs` の `IPanelTunable` 節（55–63行付近）に旧仕様の記述「EnsureBuffers reallocates the ring when this changes」が残存。CurveSamples 再設計後は不正確（リングは常に MaxK で再確保しない）。コメントのみの軽微な不整合。
3. 多人数: first body のみ採用。2体以上検出時は既存 Alert 運用に委ねる。

## 8. 将来拡張（スコープ外）

- 体格オートスケール・キャリブ（起動時1回、肩幅/背骨長で `radiusScale` 自動決定）
- 動きの出どころ B: オプティカルフロー＋速度場 advection（現状は A = ボーン剛体運動のみ）
- Phase 4 の look 決定後の不要モード畳み込み

---

## 検証（AI が MCP で実施）

- 先頭で `Application.runInBackground = true`。
- 録画（Rec2_jump 等）を Play 保持のまま再生 → ライブ曲線確認 → コンソールエラー無し → 頂点数/整合を数値確認（点群は GPU 頂点バッファで読む）→ 1フレーム遅延を織り込む。
- スクショはリポジトリに保存しない（[[visual-eval-on-editor-no-captures]]）。
