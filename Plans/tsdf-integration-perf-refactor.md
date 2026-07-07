# TSDF 統合パフォーマンス改造プラン（新セッション handoff）

2026-07-07 作成。**このドキュメント単体で cold start できるよう自己完結で書いてある。**
別セッションで着手する前提。前提コンテキストは全てここに畳んである。

## 0. ゴール

リアルタイム表示のフレームレートを上げる。**実測でボトルネックは TSDF 統合（`TSDFIntegrator`）**と判明済み。
この統合を、品質（特に motion trail と thin-limb 修正）を壊さずに軽量化する。

---

## 1. 背景 — 実測で判明したこと（2026-07-07、Unity MCP で計測）

`main.unity` の playback（Femto Bolt RCSV、4カメラ）で計測。**完全に GPU バウンド**（CPU メインスレッド ~6ms）。

### 1.1 フレームの内訳（確定・交絡なし）
同一 playback 位置で live → 即 `Time.timeScale=0`（freeze）して比較（下記 §6 参照）:

| | ms/frame | 意味 |
|---|---|---|
| LIVE 全ON | 78.2 | 実際の1フレーム |
| FROZEN（描画のみ） | 7.95 | ラスタ（既存メッシュの描画）だけ |
| **差 = 毎フレーム GPU コンピュート** | **70.3 = 90%** | ← ここが問題 |

**フレーム時間の 90% は毎フレームのコンピュート、ラスタ描画は 8% だけ。**
→ 三角形削減（`smoothSubdiv`, `meshMaxTriangles`）はこの 8% しか触れず、ほぼ無意味。
（旧 `Plans/realtime-performance-tuning.md` の「三角形を減らす」前提は実測で否定された。）

### 1.2 コンポーネント別内訳（interleaved A/B、§6.2）

| 処理 | コスト/frame | 判定 |
|---|---|---|
| **`TSDFIntegrator`（TSDF 統合 compute）** | **~25 ms** | 🔴 **最大の元凶** |
| `PointCloudMotionCurves`（カーブ build+描画） | ~5〜7 ms | 🟡 次点 |
| `TSDFView`（Marching Cubes + メッシュ描画） | ~0.8 ms | ✅ 安い（`useFullGridMC=false` が効いている） |
| 点群生成 / playback デコード / Decimater / Cumulative | ~3 ms 合計 | ✅ 安い |
| `PointCloudView` / `TSDFDebugSession` | ~0 / 0.4 ms | ✅ 無視可 |

注意: 絶対 fps は被写体依存で激しく変動（近い/大きいと 12.8fps、軽い区間で 27〜58fps）。
統合コストが被写体の深度点数に比例するため。interleaved A/B は各成分の**差**を取るのでドリフトが相殺され頑健。

---

## 2. 根本原因

`Assets/Scripts/TSDF/TSDFIntegrator.cs` の `IntegrateOne`（末尾 line ~508-516）:

```csharp
int total = volume.Dim.x * volume.Dim.y * volume.Dim.z;  // 479×411×654 = 1.29億 voxel
int groups = Mathf.CeilToInt(total / 64f);
_shader.Dispatch(_kernel, gx, gy, 1);   // フラスタム制限なし・全 voxel
```

- **統合カーネル（`Resources/TSDFIntegrate.compute`）は 1スレッド=1voxel で全グリッドを無条件 dispatch。**
- しかも **カメラ台数ぶん（4回）別々に dispatch**（`DispatchIntegrate` が per-camera raw-frame イベントで呼ばれる）。
- → 毎フレーム **4カメラ × 1.29億 = 5.15億 voxel 評価**。
- 各 voxel スレッドは自分の中心を各カメラの深度画像に投影（`_DepthFromWorld`）して TSDF 更新。
  大半の voxel は空間で early-out（カメラ背後 / 画角外 / 無効深度）するが、**スレッド起動と投影計算のコストは全 voxel ぶん毎回発生**。
- 表面付近の voxel は追加で重い: `edgeRejectRadius=2` → **5×5=25 depth ロード/voxel**（内容依存費の主因）。

### これは仕様どおりの設計（事故ではない）
`Plans/FloatingVectorsICC_v2_TSDF_spec.md §3`:
> 「1 スレッド = 1 ボクセル。各ボクセルが 4 台ループを回し、1 個のボリュームへ合成。」

voxel 基準を選んだ機能上の理由:
1. **per-voxel の trail 蓄積**（§3.2-A RetainGhost: `if abs(sdf) < abs(voxel.tsdf): voxel.tsdf = sdf`）。
   「その voxel がこれまでで最も表面に近かった観測を残す」= この作品の主眼「軌跡（残像）を立体として蓄積」。
2. **手前の空間を +tau で"空き"記録**（§3.1: `sdf = clamp(sdf, -tau, +tau)`）。MC が綺麗な等値面を出すのに必要。

**ただし仕様は統合スイープの計算コストを一度も問題視していない**（§10 の性能懸念はメモリと sub-frame 枚数だけ）。
= voxel 総なめの重さは未検討。depth 基準への移行は「仕様が禁じていない、未検討の最適化」。

---

## 3. 壊してはいけないもの（改造の制約）

1. **motion trail（残像）** = §3.2-A RetainGhost の per-voxel `|sdf|`-min 蓄積。これが作品の主眼。
2. **thin-limb 修正（実装済み・重要）** = `TSDFIntegrator.separateCameraFusion`。
   その瞬間の4カメラを instance バッファに **Average で合成**（denoise、細い肢の連続化）→ 4台揃ったら
   `Resources/TSDFFold.compute` の Fold で **`|sdf|`-min union を accumulation バッファへ**（時間方向の trail は保持）。
   詳細はメモリ `tsdf-thin-limb-collapse-retainghost.md`。**depth 基準に書き換えるならこのパイプラインを正しく移植必須。**
3. **free-space +tau 記録** = MC が符号場の空間側を必要とする。depth 基準は帯（±tau）しか書かないので、
   MC が破綻しないよう free-space の扱いを別途設計する必要がある。
4. **バッチ完結モデル** = `clearVolumeOnNewBatch` / `expectedCamCount=4`。4台揃って1スナップショット publish。
   → **round-robin（毎フレーム1台）は不可**（部分再構成しか出ない）。検証済みで却下。
5. MC 側（`TSDFView`, active-block MC, `useFullGridMC=false`）は既に安い（0.8ms）ので触らない。

---

## 4. 最適化オプション（効果 / 労力 / リスク）

| # | 方向 | 効果 | 労力 | 品質リスク |
|---|---|---|---|---|
| **①** | `edgeRejectRadius` 2→1/0（既存ノブ、実行時変更可） | 内容依存費（25近傍ロード）を直撃 | **ゼロ**（recompile 不要、今すぐ測れる） | 端の色にじみ処理が甘くなる程度 |
| **②** | **カメラ別フラスタム/有効深度 AABB で dispatch 制限** | 固定費（5.15億投影）を直撃。frustum 外 voxel はそのカメラで絶対更新されない=**結果は数学的に同一** | 中（CPU で AABB 算出 + シェーダに index オフセット） | **なし** |
| ③ | `voxelSize` 5.2→7mm 等を粗く | voxel 数 ∝1/size³ で ~2.4×減、①②両方に効く | 小（ノブ、ボリューム再構築、VRAM も減る） | 面解像度低下 |
| ④ | **depth 基準 splat に全面移行**（voxel→深度画素駆動、5.15億→~150万スレッド） | 最大（桁違い） | 大（カーネル全書換 + §3 の制約を全部移植） | 大（trail/free-space/thin-limb 再発の恐れ） |

### 推奨スタンス
- **④（depth 基準）は"本命の大改造"だが、いきなり賭けない。** §3 の制約（特に thin-limb 修正と free-space）を
  全部正しく移植しないと、解決済みの品質問題が再発する。
- まず**①で内容依存費を実測**（タダ）→ **②で固定費を品質不変のまま削る**（安全な本命）。
- ②で目標に届かない、または将来的にもっと軽くしたいなら ④ を本気で設計する。
- ③はいつでも併用可能な直交ノブ。

---

## 5. 推奨実装順序

1. **計測ハーネスを再セットアップ**（§6）。まず現状ベースラインの数字を取り直す（被写体依存なので毎回取る）。
2. **① `edgeRejectRadius` 2→0 を interleaved A/B で測定**（recompile 不要）。内容依存費がどれだけか確定。
   効くなら作品として許せる範囲でデフォルトを下げる。
3. **② フラスタム AABB 制限を実装**:
   - `IntegrateOne` で、そのカメラの視錐台（near/far は有効深度レンジ）をワールド→voxel 空間に投影し、
     voxel-index の AABB `[imin, imax]` を算出。
   - `_shader.Dispatch` を全グリッドではなく AABB サイズぶんに縮小。シェーダ側は thread id → voxel index を
     AABB オフセット付きで復元（`_DispatchOrigin` / `_DispatchDim` を追加）。
   - **検証**: 制限あり/なしで MC メッシュの頂点数・見た目が一致することを確認（数学的に同一のはず）。
   - interleaved A/B で統合コストの削減量を実測。
4. まだ足りなければ ③ `voxelSize` を作品として許せる範囲で粗く。
5. （将来）④ depth 基準を設計するなら §3 の制約を全て満たす設計書を別途起こす。
6. 完了後、`Plans/realtime-performance-tuning.md` を実測に基づいて改訂（「三角形削減」前提を撤回、
   「統合が支配的、フラスタム制限が本命」に差し替え）。

---

## 6. 計測手法（再現用・ハーネスごと）

**重要な教訓**: エディタ内では以下が**信用できない**ので使うな。
- 単発 `FrameTimingManager.gpu_frame_time`（Scene view / present 待ちを含み 150〜350ms と誤誘導。真値は 12〜27fps だった）。
- `EditorApplication.update` で GPU 時間をサンプリング（実レンダーフレームより高頻度で回り重複カウント）。

**信用できる指標**: `FPS = ΔframeCount / Δ実時間`。
**交絡に注意**: playback 内容は非定常、かつ **TSDF は時間積分で蓄積** → 逐次 A/B（別の時間窓の比較）は無効
（後の窓ほど TSDF が育って重い、という単調ドリフト）。有効な方法は次の2つだけ。

### 6.0 準備（Unity MCP `execute_code`、CodeDom C#6。`var` 可だが型は明示的に）
```csharp
Application.runInBackground = true;   // エディタ非フォーカスでも tick させる（必須）
// 型は完全修飾: UnityEngine.Object.FindObjectsByType<T>(FindObjectsSortMode.None)
// 名前空間: BodyTracking.PointCloudMotionCurves, TSDF.TSDFView / TSDF.TSDFIntegrator / TSDF.TSDFVolume
```
Unity MCP が未接続なら REPL で `/mcp`（HTTP ブリッジ 127.0.0.1:8080）。playback は Recorder Inspector の Play を要再押下。

### 6.1 live→freeze（同一位置でラスタ vs コンピュートを分離）
`Time.realtimeSinceStartup` と `Time.frameCount` をマーカー保存 → 数秒待つ（別ツールで sleep）→ 読んで fps。
その後 `Time.timeScale=0` で同位置を freeze（geometry hash = `Σ MeshFilter.vertexCount` が一定になることを確認）
→ もう一度 fps。`compute_ms = live_ms - frozen_ms`。**§1.1 はこれで取った。**

### 6.2 interleaved A/B（成分ごとのコストをドリフト相殺して測る）
対象コンポーネントの `.enabled` を 18 フレームごとに ON/OFF 交互切替、ON 状態と OFF 状態で
実時間・フレームを別々に累積。単調ドリフトが両者に均等に乗って相殺される。
`cost_ms = 1000*tOn/fOn - 1000*tOff/fOff`。**§1.2 はこれで取った。**

実装スケッチ（`EditorApplication.update` に自己退役ラムダを刺す。状態は EditorPrefs に持たせて別 execute_code 呼び出しで読む）:
```csharp
int token = UnityEditor.EditorPrefs.GetInt("il_token",0)+1;
UnityEditor.EditorPrefs.SetInt("il_token",token);
UnityEditor.EditorPrefs.SetFloat("il_tOn",0f); UnityEditor.EditorPrefs.SetFloat("il_tOff",0f);
UnityEditor.EditorPrefs.SetInt("il_fOn",0); UnityEditor.EditorPrefs.SetInt("il_fOff",0);
string targetName = "TSDFIntegrator";   // ← 測りたい型名
var targets = new System.Collections.Generic.List<MonoBehaviour>();
foreach (var mb in UnityEngine.Object.FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
  if(mb!=null && mb.GetType().Name==targetName) targets.Add(mb);
float[] lastT = { Time.realtimeSinceStartup }; int[] lastF = { Time.frameCount };
bool[] state = { true }; int[] sinceFlip = { 0 }; int flipEvery = 18;
UnityEditor.EditorApplication.CallbackFunction cb = null;
cb = () => {
  if (UnityEditor.EditorPrefs.GetInt("il_token",0)!=token){ UnityEditor.EditorApplication.update-=cb; return; }
  float now=Time.realtimeSinceStartup; int nf=Time.frameCount;
  float dt=now-lastT[0]; int df=nf-lastF[0]; lastT[0]=now; lastF[0]=nf; if(df<0) df=0;
  if(state[0]){ UnityEditor.EditorPrefs.SetFloat("il_tOn",UnityEditor.EditorPrefs.GetFloat("il_tOn")+dt); UnityEditor.EditorPrefs.SetInt("il_fOn",UnityEditor.EditorPrefs.GetInt("il_fOn")+df); }
  else       { UnityEditor.EditorPrefs.SetFloat("il_tOff",UnityEditor.EditorPrefs.GetFloat("il_tOff")+dt); UnityEditor.EditorPrefs.SetInt("il_fOff",UnityEditor.EditorPrefs.GetInt("il_fOff")+df); }
  sinceFlip[0]+=df;
  if(sinceFlip[0]>=flipEvery){ state[0]=!state[0]; foreach(var t in targets) t.enabled=state[0]; sinceFlip[0]=0; }
};
UnityEditor.EditorApplication.update += cb;
```
読み出し（~40秒後、別 execute_code）: `il_token` を別値にしてラムダ退役 → 対象を `.enabled=true` に復帰 →
`1000*tOn/fOn` と `1000*tOff/fOff` の差を出す。**測定後は必ず全コンポーネント ON・timeScale=1・EditorPrefs 掃除で原状復帰。**

---

## 7. 検証計画

- 各改造の前後で **interleaved A/B（§6.2）で `TSDFIntegrator` のコスト**を測り、削減 ms を数字で示す。
- **品質不変の担保**（②フラスタム制限で必須）: 制限あり/なしで
  `TSDFView` の MC メッシュ頂点数（`MeshFilter.sharedMesh.vertexCount`）と見た目が一致すること。
- **thin-limb 非再発**（④に着手する場合）: `tsdf-thin-limb-collapse-retainghost.md` の検証手順
  （`recorder.SeekAllTracksTo(cursor)` で決定論的に1インスタンスずつ fold、腕を水平に速く動かすポーズで
  連続した太い肢 vs 二重シェル/トゲ破片を A/B）。
- コンパイル確認: `mcp__UnityMCP__read_console` でエラー0を確認してから main マージ（CLAUDE.md ルール）。

---

## 8. 関連ファイル・参照

- 統合本体: `Assets/Scripts/TSDF/TSDFIntegrator.cs`（`DispatchIntegrate` / `IntegrateOne` / 全グリッド Dispatch）
- 統合カーネル: `Assets/Scripts/TSDF/Resources/TSDFIntegrate.compute`（1スレッド=1voxel、edge reject 内ループ）
- ボリューム: `Assets/Scripts/TSDF/TSDFVolume.cs`（`Dim` / `voxelSize` / `WorldFromVoxel` / instance・accumulation バッファ）
- thin-limb 修正: `Resources/TSDFFold.compute` + `separateCameraFusion`
- MC（既に安い、触らない）: `Assets/Scripts/TSDF/TSDFView.cs`（`useFullGridMC=false` = active-block MC）
- 元仕様: `Plans/FloatingVectorsICC_v2_TSDF_spec.md`（§2 ボリューム, §3 統合, §10 既知の限界）
- 旧・要改訂: `Plans/realtime-performance-tuning.md`（三角形削減前提は実測で否定済み）
- メモリ: `realtime-compute-bound-not-raster.md`, `tsdf-thin-limb-collapse-retainghost.md`,
  `gpu-tsdf-dropped-lens-distortion.md`
- 現状スペック実測値: 4カメラ（serial CL8F253004L/004N/004Z/300EG）、voxelSize 5.2mm、
  Dim 479×411×654 ≈ 1.29億 voxel、箱 ≈ 2.5×2.1×3.4m。
