# プラン: 点群 × BTボーンによる「動きの曲線」彫刻

作成 2026-07-02 / 改訂 2026-07-02 (Codex review round 1–3 反映, Codex approved)

## ゴール（why）
点群を**エフェクト素材**として使い、各点からその点が属する身体部位（BTボーン）の
**最近の動きの軌跡（曲線）**を伸ばして、「動いている感じ」＝**動きの彫刻**を作る。

- 精度はビタビタでなくてよい（演出用途）。肩などジャンクションの数%の誤分類は許容。
- 直線（瞬間速度）ではなく**曲線**（部位が描いた弧）を持つ。
- ライブ・リアルタイム志向（100ms 予算に対し桁違いに余裕がある設計）。

既存の [[plan-bt-trail-motion-viz]]（TSDF に焼く Start/Stop の静的彫刻、
`TSDFTrailBaker`）とは別物。あちらは「止めて焼く静的スカルプチャ」、
本件は「動きに追従して各点から曲線が伸び縮みするライブエフェクト」。

## 前提（決定事項）
過去の設計相談で確定した方針:

- **粒度 = ボーン単位**。`BodyTrackingShared.Bones` を使う。**本数はハードコードせず
  `BodyTrackingShared.Bones.Length` を参照**（現状は頭/鼻/目/耳を含む 25 エントリ。
  手先6関節は `IsDrawnJoint` で除外済み）。バッファ長・radii 配列・カーネル・色表は
  すべて `_BoneCount` 駆動にする（Codex 指摘1）。
- **分類 = 最近傍ボーン（点→線分距離）**。太さ差はボーン別半径の**正規化距離
  `d/radius`** で吸収。精度不要なので**異方性カプセル（関節 orientation 断面）は不採用**、
  等方半径＋端点テーパーで十分とする。
- **動きの出どころ = A（ボーンの剛体運動）**。オプティカルフロー（B）は将来。
- **曲線 = ボーン端点ペア履歴＋安定フレームの再投影**（下記「曲線の作り方」。当初案の
  クォータニオン姿勢履歴は roll 不定問題のため不採用、Codex 指摘2/代替案）。

---

## 曲線の作り方（本プランの核・改訂）

### なぜ姿勢クォータニオンを使わないか（Codex 指摘2）
`SkeletonMerger.TryGetJointWorld`（`SkeletonMerger.cs:404`）は**関節の world 位置しか
返さない**（orientation も confidence も出さない）。ボーン回転を端点方向だけから作ると、
**軸まわりの roll がフレーム毎に不定**になり曲線がねじれる。k4abt の生 orientation も
merge 経路を通ると保証できない。→ **姿勢は使わず、端点ペア＋明示的に定義した安定
フレームで曲線を再構成**する。

### データ表現
ボーンごとにリングバッファで**両端点の world 位置ペア** `(a_k, b_k)` を保持
（直近 N フレーム / `trailDuration` 秒）。加えて各時刻の**安定ボーンフレーム**を定義:

- 原点 = 線分中点 `m_k = (a_k+b_k)/2`
- 主軸 `u_k = normalize(b_k - a_k)`
- 副軸 `v_k` = **参照軸**（親ボーン方向、無ければ torso-up）を `u_k` 垂直面へ射影して正規化。
  **degenerate（ほぼ平行）時**は前フレームの `v_{k-1}` を **`u_k` 垂直面へ射影→正規化**して
  fallback（`v_{k-1}` は新 `u_k` に直交とは限らないため必ず再射影, Codex R2-2）。それでも
  degenerate なら**決定的な world/torso 軸**（例 world up）へ最終 fallback
- 第3軸 `w_k = u_k × v_k`
- **参照軸テーブル（明示）**: `BonePoseHistory` に全 `Bones.Length` エントリ分の参照軸を
  定義（親ボーン方向。頭/顔ボーンなど親が曖昧なものも表で固定）。**torso-up は一意に
  `neck - pelvis`（無ければ `spineChest - pelvis`）で定義**し fallback とする（Codex R2-3）

### 分類とオフセット
ある seed 点 `x`（world）を最近傍ボーン `b` に分類し、**現フレームの安定フレームでの
パラメータ化**を取る:

- 線分パラメータ `t = clamp(dot(x - a_now, u_now)/|b_now-a_now|, 0,1)`
- 垂直オフセット `off = x - lerp(a_now,b_now,t)` を `(v_now,w_now)` 基底で係数化 `(cv,cw)`

### 再投影（曲線の制御点）
履歴各時刻で同じパラメータを復元:

`x_k = lerp(a_k, b_k, t) + cv·v_k + cw·w_k`

ボーンが回転すれば `u_k,v_k,w_k` が変化するので `x_k` は**弧を描く＝曲線**。
「もしこの点がこのボーンに剛体接続されていたら窓の間どこを通ったか」を、
roll 不定を避けつつ描く。

### 時間コヒーレンス
- **seed は毎フレーム使い捨て**でよい。履歴はボーンが持つので per-particle 状態不要。
- ランダム seed はちらつくので**決定的 seed 選択**（頂点 index の stride/hash）＋
  **validity/cutoff/sanity フィルタ**（下記 Phase2）を併用してちらつきを抑える
  （Codex 指摘6：生頂点バッファには decimater/capsule フィルタは未適用前提）。

### 平滑化（Phase3 で追加）
履歴が疎（K が小さい）ならカクつくので、制御点間を **Catmull-Rom** で細分。
**まず Phase2 では細線のみで座標系・履歴意味論を実証し**、ribbon と Catmull-Rom は
Phase3 で足す（Codex 代替案：早期リスク低減）。

---

## 構成要素（新規/既存）

| 要素 | 新規/既存 | 役割 |
|---|---|---|
| `BonePoseHistory`（新規 MonoBehaviour） | 新規 | 全ボーンの端点ペア `(a_k,b_k)` ＋安定フレーム＋**validity/freshness** をリングバッファ保持。GPU `StructuredBuffer` に公開。`SkeletonMerger` から `Bones` 表で両端関節を引く。**ロスト時は該当ボーン履歴をリセット** |
| `SkeletonMerger.PublishBonePosesWorld(...)`（新規 API） | 新規 | 端点 world 位置＋**各端点の valid/fresh** を返す。既存 `TryGetJointWorld` は valid/invalid のみで confidence 非公開のため、held/stale 関節混入を防ぐ用に freshness を出す（Codex 指摘3） |
| seed source | 既存流用 | 点群頂点の decimated 集合（world）。GPU 経路: `MeshFilter.sharedMesh.GetVertexBuffer(0)`（`_reconstructor` は private なので mesh 経由）。CPU 経路: `PointCloudRenderer.OnFrameUploaded`(NativeArray) |
| 分類 + パラメータ化 | 新規 compute | seed→最近傍ボーン（`d/radius` 正規化, `PointCloudCapsuleFilter` の point-to-segment 数式流用）→ `(t,cv,cw)` |
| 曲線ビルダー | 新規 compute | seed ごとに履歴再投影 → 制御点列 → line/(Phase3) ribbon 頂点。**出力バッファ戦略は下記** |
| 描画マテリアル | 新規 shader | line/ribbon 描画。色: per-bone hue（`TSDFTrailBaker.HueFor` 流用）/ 時間フェード / 速度 |

座標空間は**全て Unity world（m）**で統一。seed はメッシュローカル→
`renderer.transform.localToWorldMatrix`（scale.y=-1 反転込み）で world へ。
ボーンは `SkeletonMerger` が既に world。

### 多人数ポリシー（Codex 指摘）
1人インスタレーション前提。`TryGetJointWorld` は first body を返す一方 capsule/motion
publish は全 active body を回すので不整合になりうる。→ **本機能は first body のみ採用**し、
2体以上検出時は warn（既存の Alert 運用に合わせる）。

---

## Phase 0 — 座標空間・パラメータ設計（紙の上）
- 空間チェーン確認: seed(mesh-local m) →[localToWorldMatrix]→ world、bone(world) の整合。
- パラメータ定義: `seedCount`（例 2k–20k）、`trailDuration`（例 0.4–1.5s）、
  `historySamples K`（例 8–16）、`boneRadius[_BoneCount]` + テーパー、`normalizedCutoff`
  （例 1.5、超過は seed 破棄）、`smoothSubdiv`（Phase3）。

## Phase 1 — BonePoseHistory と「素の曲線」検証（CPU/Gizmo）
点群抜きで、**ボーン中点の履歴軌跡を数本 Gizmo/LineRenderer で描く**だけを作り、
安定フレームで弧が正しく出るか・`trailDuration`/`K` の感触・roll 連続性を確認。
CPU/デバッグ経路を先に通すことで座標・フレームのバグをシェーダ前に潰す（Codex 指摘）。

- `BonePoseHistory` 実装（端点ペア＋安定フレーム＋validity/freshness、ロスト時リセット）。
- `SkeletonMerger.PublishBonePosesWorld` 追加。**publish は LateUpdate、消費は次 Update**
  ＝**1フレーム遅延**前提で検証（Codex 指摘7、`BodyJointMotionFeeder.cs:8` 準拠）。
- 手（ライブ）or 録画（Rec2_jump 等）を再生し腕振りで検証。

**評価軸**: 弧が正しく出るか。ねじれ無いか。窓長感。破綻しないか。

## Phase 2 — seed 点群 → 分類 → 曲線（細線・ライブ）
- seed source（GPU 経路）: `mesh.GetVertexBuffer(0)` を取得 → dispatch 後 **必ず Dispose**、
  `ByteAddressBuffer` として **24byte stride**（`ObColorPoint` = pos12+col12）で読み、
  `localToWorldMatrix` を渡す（cumulative パターン踏襲, Codex 指摘4）。
- **無効深度カリング**: GPU 再構成は無効点を `(1e10,1e10,1e10)` で出す
  （`PointCloudCumulativeFilter.compute:93`）。**分類前に sanity-range で除外**、
  さもないと NaN/オーバーフロー/巨大 bounds/誤分類（Codex 指摘5）。
- seed 選択: index stride/hash ＋ validity ＋ `normalizedCutoff` 超過破棄 ＋ sanity。
- 分類 compute: seed→最近傍ボーン（`d/radius`）→ `(t,cv,cw)`。
- ビルダー compute: 履歴再投影 → **細線（line list：2頂点/セグメント, `MeshTopology.Lines`）**。
  ribbon/Catmull-Rom はまだ。

**評価軸**: 各点から部位の動きの曲線が伸び、群として「動いている感じ」が出るか。
seed 密度と可読性のトレードオフ。ちらつき。1フレーム遅延の許容感。

## Phase 3 — 見た目の強度（アート耐性）
- **ribbon 化 ＋ Catmull-Rom 平滑**（Phase2 で座標・履歴が実証された後に追加）。
- **色モード**: per-bone hue（`HueFor`）/ 時間グラデ（新しい端ほど明るい・古い端フェード）
  / 速度ヒートマップ（履歴差分速度）。
- **太さ/窓長**: `radius`（リボン幅）, `trailDuration`, taper（先細り）をアート方向へ。
- **seed 密度・分布**: 全身均一 / 主要部位重点。
- （比較用・任意）**B の味見**: 速度場 advection の streak look を別モードで少し試す
  （将来のオプティカルフロー B への入口）。

**評価軸**: 静止画・動画で「作品」として立つか。

## Phase 4 — 評価と決定
候補を撮って（`/Users/horihiroyuki/Dropbox/projects/ICC/capture`）並べ、残す look を1つ決める。
決めたら不要モード/コードを畳む。

---

## 出力バッファ戦略（Codex 指摘・実装前に確定）
- **形式**: Phase2=line list（seed×(K-1) セグメント, 2頂点/セグメント）、Phase3=ribbon 三角（seed×
  セグメント×2三角）。ribbon は `20k×16×subdiv` で容易に数百万頂点 → **上限設計必須**。
- **確保方式**: 決定的固定 index（seed→固定スロット）＋ **`_Alive` フラグ**で無効 seed を
  縮退（degenerate 三角）。または append/counter＋indirect draw args。
- **クランプ**: `maxVertices`/`maxSeeds` で dispatch/draw を必ず上限。超過は log（無音打ち切り禁止）。
- **bounds**: 描画用 bounds を明示（body 周辺 + trail 余白）。
- **indirect 描画は既存プロジェクト踏襲で `Graphics.DrawProceduralIndirect`**（`TSDFView.cs:400`
  で使用中の作法）+ indirect args buffer。`RenderPrimitivesIndirect` は不採用（Codex R2-5）。

## ライフサイクル（Codex 指摘）
- `GraphicsBuffer`/`ComputeBuffer` は disable/OnDestroy で解放。`mesh.GetVertexBuffer` は
  取得毎に Dispose。
- `seedCount`/`K`/`subdiv`/**source mesh 容量**変化で再確保（永続 buffer 再利用、不足時のみ伸長）。
- body/history/source mesh が無い時は draw args を 0 クリア（何も描かない・エラーにしない）。

---

## 実装で触るファイル（想定）
- `Assets/Scripts/PointCloud/PointCloudMotionCurves.cs`（新規）— seed 収集
  （`sharedMesh.GetVertexBuffer(0)`）、compute dispatch、indirect 描画、ライフサイクル、
  first-body ポリシー、Inspector パラメータ（`_BoneCount` 駆動）。
- `Assets/Scripts/BodyTracking/BonePoseHistory.cs`（新規）— 端点ペア＋安定フレーム＋
  validity/freshness リングバッファ、ロスト時リセット、GPU 公開。
- `Assets/Scripts/BodyTracking/SkeletonMerger.cs` — `PublishBonePosesWorld(...)`
  追加（端点 world ＋ valid/fresh）。
- `Assets/Scripts/PointCloud/Resources/MotionCurvesBuild.compute`（新規）— sanity カリング＋
  分類（`d/radius`）＋パラメータ化＋履歴再投影＋（Phase3）Catmull-Rom→頂点生成。
- `Assets/Scripts/PointCloud/Resources/MotionCurves.shader`（新規）— line/ribbon 描画・色モード。
- 参照/流用: `PointCloudCapsuleFilter`（point-to-segment）, `PointCloudReconstructor`/
  `PointCloudCumulativeFilter.compute`（GPU 頂点バッファ作法・無効点 sanity）,
  `BodyTrackingShared`（`Bones`/`Bones.Length`/`IsDrawnJoint`/`K4AmmToUnity`）,
  `TSDFTrailBaker.HueFor`（per-bone 色）, `BodyJointMotionFeeder`（LateUpdate publish 作法）。

## 検証（AI が MCP で実施）
- `Application.runInBackground=true` を先頭で叩く（非フォーカスでも tick）。
- 録画（Rec2_jump 等）を Play 状態を保ったまま再生 → ライブ曲線を確認 → コンソール
  エラー無し（`Unity_GetConsoleLogs`）→ 頂点数・整合を数値確認（**点群は GPU 頂点
  バッファで読む**）→ **1フレーム遅延を織り込み同フレーム断定を避ける** → Dropbox にキャプチャ。
- スクショはリポジトリに保存しない（[[visual-eval-on-editor-no-captures]]）。

---

## 未確定 / 要確認（実装前に握る）
1. **seed source**: GPU 経路（`sharedMesh.GetVertexBuffer(0)`, `useGpuReconstruction=true`
   維持・完全リアルタイム）を推奨。CPU 経路（`OnFrameUploaded`）は試作は楽。→ GPU でよいか。
2. **seed 使い捨て（推奨・per-particle 状態なし）** ＋決定的 seed でよいか。
3. **曲線 = 端点ペア＋安定フレーム再投影（roll 不定回避）**でよいか。
4. 描画: Phase2 細線 → Phase3 ribbon/Catmull-Rom の段階でよいか。
5. `seedCount`/`trailDuration`/`K` の初期レンジ。
6. `BonePoseHistory` は独立コンポーネント（履歴は他機能でも使える）で確定。

## リスク
- GPU 頂点バッファ（reconstructor Mesh）を compute から読む配線が新規（stride/フラグ要確認）。
- seed × K × subdiv の頂点数上限。上限とカリング・indirect args を設計。
- 履歴欠損（confidence 低下・一時ロスト）時の曲線破綻 → freshness で drop/reset。
- 安定フレーム副軸の degenerate（親と平行）→ 前フレーム fallback で連続性確保。
- 「作品耐性」は主観評価。Phase 4 でユーザー判断。
