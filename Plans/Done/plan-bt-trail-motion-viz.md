# プラン: BT Trail × TSDF によるモーション・ビジュアライゼーション

作成 2026-07-02

## ゴール（why）
**作品として耐える「動きが見えるビジュアライゼーション」**を作る。
BT の trail（関節の時系列）を SDF に焼き、点群由来の body メッシュと合わせて、
動きが時間として立ち上がる映像表現を得る。手法を素早く検証しながら決める。

**今アンブロックされた前提**: depth→color 補正（commit `c156e5b`）で trail が
点群 body に正しく整合するようになった。これまで両者がズレて「合わさった絵」を
評価できなかったが、**今は fuse した見た目を初めて評価できる**。

## 現状（what we have）
- `TSDFTrailBaker`（コミット済み）: **手動一発 bake**。windowed trail(SkeletonMerger)
  or 録画全体(BodyTrackingPlayback) の per-joint 軌跡を capsule SDF に変換し、
  volume に min-union → Publish → 既存 MC が再メッシュ。
  - `clearVolumeFirst`: true=trail単体 / false=fuse（volume の既存内容に min-union）
  - `ribbonJoints`（既定 手首/手/足/頭）, `radius`, `sampleStride`, `perJointHue`
- 問題: `TSDFIntegrator`/`TSDFDebugSession` が毎フレーム volume を clear+publish
  するので、手動 bake が即上書きされる。プロトタイプでは手でドライバを止めていた。

## スコープ外
- ray-march 直描画（今回はやらない）

---

## Phase 1 — fuse の確認（今すぐの山）
整合済みの状態で **body TSDF ＋ trail を1枚の MC メッシュ**として見る。

### ⚠️ 二重バッファの罠（Codex 指摘・要対応）
integrator が publish した後は **body は `FrontBuffer`、`WriteBuffer` は古いスクラッチ**
（`TSDFVolume.Publish()` が swap）。現 `TSDFTrailBaker` は `WriteBuffer` に書いて
publish するので、`clearVolumeFirst=false` の fuse は**表示中の body ではなく古い
内容に融合**してしまう（プロトタイプで見た「1フレ古い body に fuse」の正体）。
- 対策: **`FuseTrailIntoDisplayedBody()` helper を追加** — `FrontBuffer → WriteBuffer`
  へコピー → trail を min-union → Publish（表示中の body に確実に重なる）。
  - `TSDFVolume` に `CopyFrontToWrite()` を追加。**SDF と color の両方**
    （`FrontBuffer→WriteBuffer` かつ `FrontColorBuffer→WriteColorBuffer`）をコピー。
    幾何だけコピーすると body の色が古い WriteColorBuffer のまま残る（Codex 指摘）。
- 代替: 単一バッファ(`doubleBuffered=false`)＋新鮮な body volume で手動検証。

### 手順
recorder Read(Rec2_jump) → 数秒再生で trail 窓を貯める → Pause →
`FuseTrailIntoDisplayedBody()` で fuse bake。**trail単体 / fuse / body単体**を並べ判断。

**評価軸**: trail が body に乗って見えるか。有機的か / ノイズか。色・太さの初期感触。

## Phase 2 — ライブ化（動きを見せる = 作品性の核）
静止 bake ではなく、**再生に追従して trail が伸び縮みする**動的メッシュにする。

### 設計の肝: volume 所有権（毎フレームの合成順序）— Codex 推奨案を採用
**publish は必ず integrator が1回だけ**行い、baker はその**直前に WriteBuffer を
書き足すだけ（no-op 可）**。baker は publisher にしない（`deferPublish` は不採用）。

- `TSDFIntegrator` に**「complete batch の publish 直前」フックを1つ**追加:
  ```csharp
  public event System.Action<TSDFIntegrator, TSDFVolume> BeforePublishCompleteBatch;
  ```
  - **発火は publish 直前に1回だけ**。`IntegrateOne`(=カメラ毎)ではなく、
    **全カメラ揃って publish する箇所**（`_batchSerials.Count==expectedCamCount` の
    `volume.Publish()` 直前, TSDFIntegrator.cs:281-284）に置く。
    → 毎 IntegrateOne で焚くと trail を複数回・全カメラ未integrate で焼く事故になる（回避）。
  - accumulate 経路も publish するなら、`FoldInstanceIntoAccumulation()` 後・publish 前に
    同フックを発火（対応する場合）。
  - **例外分離（Codex 指摘）**: 素の event 呼び出しは購読者が throw すると `volume.Publish()`
    をスキップしてしまう。`InvokeBeforePublishCompleteBatch()` ヘルパで **try/catch＋log
    してから必ず Publish** する（trail の失敗が body の publish を絶対に止めない）。
- `TSDFTrailBaker.LiveFuse` はこのフックを購読し、**WriteBuffer に min-union するだけで
  publish しない**。integrator が続けて Publish → TSDFView が MC。
- callback 契約: **`Action<TSDFIntegrator, TSDFVolume>`（戻り値なし）。「WriteBuffer を
  mutate、または no-op。publish は制御しない」**（成否は返さない＝signature と整合）。
  **trail 0本でも body の publish を止めない**（baker は何もせず返る）。

### モード整理（Inspector で切替）
- `Off`（従来の手動 bake ボタンのまま）
- `LiveTrailOnly`（毎フレーム trail 単体）
- `LiveFuse`（毎フレーム body＋trail、上記フック）

### LiveTrailOnly の競合ガード（Codex 指摘）
integrator は毎 Update で `volume.doubleBuffered = clearVolumeOnNewBatch` を強制し、
有効な限り clear/publish し続ける（TSDFIntegrator.cs:494）。
→ **LiveTrailOnly は integrator を無効化**（`integrationEnabled=false` or 「有効な
integrator が無い」ことを要求）した上で、baker が clear→bake→publish を所有する。

### `TSDFDebugSession`
bench（compare/accumulate）は**ライブ時 idle 前提**。競合しないようガード
（enabled チェック or モード排他）。

### 毎フレーム実行のアロケーション（Codex 指摘・必須）
現 `BakeTrailIntoVolume()` は bake 毎に `List<TrailSeg>` と `ComputeBuffer` を新規確保
（TSDFTrailBaker.cs:139,166）。ライブでは GC/バッファ churn が性能評価を歪めるので:
- **永続 `List<TrailSeg>`（Clear して再利用）＋ 再利用/伸長する `ComputeBuffer`**
  （容量不足時のみ再確保）。**disable/OnDestroy で解放**。

### コスト/安全
- 毎フレーム bake の GPU コスト（capsule×voxel）。TDR 回避のバッチは既存。
  対象 joint 数・radius・sampleStride で制御。`meshMaxTriangles` 上限に注意（truncate ログ）。
- 巨大 volume(32M voxel)での毎フレーム全走査コストを計測し、必要なら
  trail 近傍のみ dispatch する最適化を検討（まずは素直に、計測して判断）。

### 手動 bake と live のセマンティクス分離（Codex 指摘）
既存 `BakeTrailIntoVolume()` は「segments 0 なら Fail をログ」する手動向け挙動。
これを毎フレーム再利用するとライブで log-spam/無駄処理になる。
- **live 経路は empty trail = 静かな no-op**（ログ無し・失敗扱いしない）。
- **手動/ContextMenu bake は従来どおり失敗を報告**。
- 内部 bake ルーチンを `silent`/`live` フラグ付きに分け、両者を明確に分離。

### エッジケース（Codex 指摘）
- trail 0本 → body publish を止めない（baker no-op、ライブは静か）。
- `expectedCamCount` 過大 → complete batch が来ず publish されない＝ライブ trail も出ない。
  ライブ前に expectedCamCount と実カメラ数の整合をログ/検出。
- **volume/grid 再構築**（`RebuildIfNeeded`）でバッファ無効化 → baker のリソース/状態を
  refresh（PublishVersion or grid 変化を検知）。
- **同一 integrator に複数 baker** が購読しないようガード（単一想定を明示）。

**評価軸**: 動きが時間として読めるか。残像窓(trailDuration)の長さ感。破綻しないか。

## Phase 3 — 見た目の強度（アート耐性）
- **色モード**: per-joint hue / 速度ヒートマップ（JointMotionField が既にある）/
  時間グラデ（新しいほど明るい等）。SkeletonMerger 側の色資産を流用できるか確認。
- **交差部の色（Codex 指摘）**: 現 `TSDFTrailBake.compute` は `sdf < cur.x` の時だけ
  色を書くので、body 内部/近傍では body TSDF が幾何で勝ち **trail 色が消える**。
  「色付きモーションを body に埋め込む」なら、**幾何 min-union とは独立に色を
  ブレンド/上書きする方針**を用意（例: trail が bind 範囲内なら weight/tint を別途反映）。
  fuse で trail の存在感を出す肝なので Phase 3 の必須項目に含める。
- **smin 有機結合**: 既存 `TSDFSmoothUnion` の smin で capsule 同士・body との
  接合を有機的に（metaball 感）。`accumulateSmoothK` との棲み分け。
- **太さ/窓長**: radius, trailDuration をアート方向に振る。
- **対象 joint**: 全身 / 主要数本の比較（密度と可読性のトレードオフ）。

**評価軸**: 静止画・動画で「作品」として立つか。

## Phase 4 — 評価と決定
Phase 1–3 の候補を撮って（`/Users/horihiroyuki/Dropbox/projects/ICC/capture`）並べ、
**残す手法（look）を1つ決める**。決めたら不要モード/コードを畳む。

---

## 実装で触るファイル（想定）
- `Assets/Scripts/TSDF/TSDFTrailBaker.cs` — モード(Off/LiveTrailOnly/LiveFuse)、
  per-frame 実行、`BeforePublishCompleteBatch` 購読、永続 seg list＋再利用 ComputeBuffer、
  `FuseTrailIntoDisplayedBody()`、grid 再構築時の refresh、単一 baker ガード
- `Assets/Scripts/TSDF/TSDFIntegrator.cs` — `BeforePublishCompleteBatch` イベント追加
  （complete-batch の `volume.Publish()` 直前に1回発火。**baker は publish しない**）
- `Assets/Scripts/TSDF/TSDFVolume.cs` — `CopyFrontToWrite()`（Phase 1 の fuse 用）
- `Assets/Scripts/TSDF/TSDFView.cs` — 変更不要見込み（PublishVersion 駆動のまま）
- `Assets/Scripts/TSDF/Resources/TSDFTrailBake.compute` — 交差部の色ブレンド/上書き方針
- 色モード次第で `PointCloudJointMotionField` 参照

## 検証（AI が MCP で実施）
- Rec2_jump / cursor 中盤で再生 → ライブ bake → コンソールでエラー無し確認 →
  MC 頂点数・整合を数値確認（**点群は GPU 頂点バッファで読む**）→ Dropbox にキャプチャ。
- 競合ドライバ（integrator/debugsession）の状態を明示的に管理して再現性を担保。

## 実装メモ（Codex approved 時の残ノート）
- hook は **2箇所の publish 直前**に置く: complete-batch (`TSDFIntegrator.cs:281`) と
  accumulate-fold (`:302`)。
- 例外分離は multicast event を直接 Invoke せず **`BeforePublishCompleteBatch.GetInvocationList()`
  を回して各購読者を個別 try/catch**（一人が throw しても他＋publish を守る）。
- `CopyFrontToWrite()` は **単一バッファ時(`FrontBuffer==WriteBuffer`)は no-op**。
  distinct な時のみ SDF＋色を両方コピー。

## リスク / 未確定
- 毎フレーム bake の性能（32M voxel 全走査）。まず計測、ダメなら近傍 dispatch。
- `BeforePublishCompleteBatch` フック追加が既存の accumulate/compare/単発 replay 経路に
  回帰を出さないか（フックは「publish 直前・購読者無ければ完全 no-op」に限定して回帰回避）。
- 二重バッファ: Phase 1/フック双方で「表示される buffer に書けているか」を常に確認
  （Front/Write の取り違えが最大の落とし穴）。
- 「作品耐性」は主観評価。Phase 4 でユーザー判断。
