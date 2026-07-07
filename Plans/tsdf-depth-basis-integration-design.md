# TSDF 統合 depth 基準リファクタ 設計書（オプション④）

2026-07-07 作成。`Plans/tsdf-integration-perf-refactor.md` のオプション④「depth 基準 splat に全面移行」の
設計を詰めるためのドキュメント。**このドキュメント単体で cold start できるよう自己完結で書く。**

親プランのスタンス:「④は本命の大改造。§3 の制約を全部正しく移植しないと、解決済みの品質問題が再発する。
着手するなら §3 の制約を全て満たす設計書を別途起こす」= **これがその設計書。**

---

## 0. ゴール / スコープ

TSDF 統合カーネルを **voxel 基準（1スレッド=1voxel、全グリッド無条件 dispatch）** から
**depth 基準（1スレッド=1深度画素、観測画素だけが surface 近傍の voxel を更新）** に置き換え、
毎フレームのコンピュートを桁違いに軽くする。品質（motion trail / thin-limb / MC 等値面）は不変を保つ。

### スコープ外
- Marching Cubes / TSDFView（既に安い 0.8ms、`useFullGridMC=false`）は触らない。
- BoundingBox / voxelSize / tau などの調整ノブ（①②③）はこの設計とは直交。

---

## 1. 確定事実（2026-07-07、Unity MCP で live playback から実測）

| 項目 | 値 | 出典 |
|---|---|---|
| 深度解像度 | **320×288 / camera**（binned NFOV） | 実機 reflect（4台とも同一）|
| カメラ台数 | 4（CL8F253004L/004N/004Z/300EG） | 〃 |
| depth 画素総数 | 320×288×4 = **368,640** | 計算 |
| voxel 総数 | 479×411×654 = **1.29 億**（×4 cam dispatch = 5.15億 thread/frame） | 実測 Dim |
| voxelSize / Tau | 0.0052 m / 0.0208 m（tau = 4 voxel） | 実測 |
| 稼働モード | **live-follow + RetainGhost**（`clearVolumeOnNewBatch=True`、バッチ毎 clear） | 実測 |
| MC ゲート | セルの **8隅すべて weight>0** でないと三角形を出さない | `TSDFMarchingCubes.compute:113` |

### 1.1 スレッド数の桁

- 現行 voxel 基準: **5.15 億 thread/frame**（早期 return が大半だが起動＋投影費は全 voxel ぶん発生）。
- depth 基準: **36.9 万 pixel-thread/frame**。各画素が ±tau バンド（≈ 2·tau/voxelSize = 8 voxel）を更新
  → voxel 書き込みは ≈ 37万 × 8 ≈ **300 万 write/frame**。
- → thread 起動 ≈ **1400× 減**、voxel touch ≈ **170× 減**。これが④の狙い。

### 1.2 現行アルゴリズム（voxel 基準、`TSDFIntegrate.compute`）

各 voxel v について:
1. v 中心 → world → depth-camera mm（`_DepthFromWorld`）に投影。z<=0 なら return（背後）。
2. forward lens distortion をかけて画素 (u,v) を得る。画角外なら return。
3. `d = depth[u,v]`。0 なら return。edge-reject（(2r+1)² 近傍）で silhouette 除外。
4. `sdf = d - v.z`（camera Z 方向）。`sdf < -tau` なら return、`sdf = clamp(sdf, ±tau)`。
5. RetainGhost（`|sdf|`-min が勝つ）で `_Voxels[v]` / `_Colors[v]` を更新。**レース無し**（1 voxel = 1 owner）。

**性質**: 表面手前の voxel は sdf=+tau（free-space）として書かれ weight が付く。表面をまたぐセルの
8隅は必ず「手前=+tau・奥=-sdf」で全部 weight>0 → MC ゲート通過。これが free-space carving の役割。

---

## 2. depth 基準アルゴリズム設計

### 2.1 スレッド = 1 深度画素

各深度画素 (u,v) について（1台ぶんの dispatch = 320×288 thread）:
1. `d = depth[u,v]`。0 なら return。edge-reject をここで**1回だけ**評価（現行は surface 近傍 voxel ごとに
   評価していた＝重複。depth 基準なら画素あたり1回で済む → edge-reject 費も激減）。
2. 画素 (u,v) + d を **undistort して逆投影** → depth-camera mm の 3D 表面点 P。
   → **`PointCloudReconstruct.compute` と同一の逆投影（`DepthUndistortLut`）を再利用**。
   これで TSDF の表面点が点群と厳密一致（座標系ズレの再発を原理的に防ぐ、cf. メモリ
   `gpu-tsdf-dropped-lens-distortion`, `skeleton-pointcloud-vertical-offset`）。
3. P を world → voxel index 空間へ（`_VoxelFromWorld`）。
4. P 周りの **surfel スタンプ**（±tau × 横 footprint）の各 voxel に sdf を書く（§3-B）。
   sdf は camera Z 方向の `d - sampleZ` を維持（MC 等値面が現行と一致するため）。

### 2.2 なぜ「バンドだけ」で MC が壊れないか（free-space 不要の論拠）

MC が三角形を出すのは **iso=0 をまたぐセルだけ**。そのセルの8隅は表面から高々 √3·voxel ≈ 9mm。
tau = 20.8mm なので **ゼロ交差セルの8隅は必ず ±tau バンド内 = 書かれている** → MC ゲート通過。
手前の free-space(+tau) voxel は全部 positive でゼロ交差を作らない → **三角形に寄与しない**。
∴ **単一観測では band-write と voxel-basis の抽出メッシュは一致する。** free-space の明示 carving は
単一観測の等値面には不要。**例外**（§3-C）: マルチカメラ Average 融合と、境界セルの隅欠け。

---

## 3. 品質ランドマイン と 対策（★要合意）

### 3-A. 書き込みレース（最重要）

depth 基準は複数画素（隣接画素のバンド重複＋4カメラ）が**同一 voxel に read-modify-write** → データレース。
voxel 基準にはこれが無かった（1 voxel=1 owner）。RetainGhost（`|sdf|`-min）を正しく出すには対策必須。

**推奨: 2-pass InterlockedMin scatter（幾何は厳密・レース無し）**
- 追加バッファ `_MinKey`（uint/voxel）。key = `quantize(|sdf|/tau, 16bit)`。小さいほど表面に近い。
- Pass1（scatter）: 各画素が band voxel に `InterlockedMin(_MinKey[v], key)`。→ 各 voxel の最小 `|sdf|` が確定。
- Pass2（scatter）: 各画素が再走査し、自分の `key == _MinKey[v]` の voxel にだけ sdf/weight/color を書く
  （非 atomic だが「勝者だけが書く」ので結果は決定的。同 key のタイは同 sdf、色だけ last-writer=軽微）。
- Clear 時に `_MinKey` を 0xFFFFFFFF リセット。コスト ≈ 2× scatter（それでも ~600万 write、安い）。

代替案（合意で選択可）:
- (b) **良性レース許容**（各成分 last-writer-wins）: 実装は最も簡単。`|sdf|`-min 不変条件がまれに破れ
  表面に微ノイズ。4カメラ融合では視覚的に無視できる可能性が高いが**保証は無い**。まず (b) を試して
  A/B で破綻を見てから (a) に上げる、という段階戦略も可。
- (c) カメラ別バッファに書いて後段でレース無し merge。VRAM 4× + merge dispatch 増、非推奨。

### 3-B. 横方向アンダーサンプリング → 穴 / MC ゲート破綻

voxelSize(5.2mm) ≈ 画素 footprint（2m/320≈6mm @1.5m）。表面が正対する所では **1画素≈1voxel** となり、
1画素=1本のレイ線だけ書くと voxel 格子の間に**網戸状の穴**が空きうる。穴が空くとゼロ交差セルの隅が
weight0 → MC ゲート却下 → メッシュ欠け。voxel 基準は全 voxel gather なのでこの穴が無かった。

**推奨: surfel スタンプで band を面的に埋める**
- 各画素は「点 P を中心に、レイ方向 ±tau・接方向 ±(footprint/2 + 1voxel)」の小 voxel 近傍を塗る。
- 法線が無いので簡易に **3×3×N の軸並行スタンプ**（N ≈ 2·tau/voxelSize）で近似し、live で穴が消える
  最小サイズに詰める。書き込み増は ~9× だが総量は数千万で安い。
- **検証**: voxel 基準と depth 基準で MC 頂点数・見た目（特に正対面と細い肢）を A/B 一致させる。

### 3-C. マルチカメラ融合 / free-space（thin-limb 修正の移植）

- **現行 live-follow は RetainGhost で4カメラを `|sdf|`-min 融合**（Average ではない）。§3-A の 2-pass min が
  そのまま「4カメラ横断の `|sdf|`-min」を実装する（batch 内で clear されず accumulate、batch 完了で Publish）。
- **accumulate モード（thin-limb 修正の本体、`separateCameraFusion` = Average→Fold）は現行 perf シーンでは
  非稼働**。ここは Average 融合が要る（メモリ `tsdf-thin-limb-collapse-retainghost`）。depth 基準の Average は
  atomic 加算が必要 → §3-A の (a)/(b) より難物。**フェーズ2 に回す**（§5）。
- free-space +tau の明示 carving は単一観測の等値面には不要（§2.2）。ただし Average 経路では「手前は空き
  =+tau」の票が Average に効いていたので、accumulate 移植時は band 内 free-space の扱いを別途検証する。

★**要確認**: 作品の本番は live-follow RetainGhost と accumulate Average+Fold のどちらが主か。
perf 実測は live-follow。trail の主眼は accumulate 寄り。→ フェーズ分割（§5）で両対応する。

---

## 4. 影響ファイル

- 新規: `Resources/TSDFIntegrateDepth.compute`（1スレッド=1画素の scatter カーネル、pass1/pass2）。
- 改修: `TSDFIntegrator.cs` — `IntegrateOne` の dispatch を「全グリッド」から「depth W×H」へ。
  depth 逆投影ユニフォーム（undistort LUT / 逆投影行列）を `PointCloudReconstruct` から流用してバインド。
  **フラグ `useDepthBasis`（bool）で voxel 基準↔depth 基準を切替**（ロールバック & A/B 用に両実装を残す）。
- 改修: `TSDFVolume.cs` — `_MinKey` バッファの alloc/clear（2-pass min 用）を追加。
- 参照（変更なし・整合確認のみ）: `TSDFMarchingCubes.compute`（8隅 weight ゲート）, `TSDFFold.compute`。

---

## 5. 段階実装 & 検証（フェーズ分割）

**フェーズ1: live-follow RetainGhost の depth 基準化（perf の本命、現行稼働モード）**
1. `TSDFIntegrateDepth.compute` を pass1/pass2 で実装。逆投影は点群と共有。
2. `useDepthBasis` フラグで切替。既定 OFF（現行維持）でマージ、A/B は Inspector で。
3. **検証**: 同一 playback 位置で voxel 基準↔depth 基準を切替、
   - MC 頂点数（`MeshFilter.sharedMesh.vertexCount`）と見た目一致（特に正対面・細い肢に穴が無い）。
   - interleaved A/B（親プラン §6.2）で `TSDFIntegrator` コスト削減 ms を実測。
4. 穴が出たら §3-B のスタンプサイズを詰める。レース由来ノイズが出たら §3-A を (b)→(a) に上げる。

**フェーズ2: accumulate Average+Fold の depth 基準化（thin-limb 修正の移植）**
5. instance バッファへの Average を depth 基準で（atomic 加算 or 別戦略）。Fold は据置（既に voxel 基準で安い？
   → Fold も全グリッド dispatch なので、必要なら active-block 限定で別途軽量化）。
6. **検証**: `tsdf-thin-limb-collapse-retainghost` の手順（`recorder.SeekAllTracksTo` で決定論再生、腕を水平に
   速く動かすポーズで「連続した太い肢 vs 二重シェル/トゲ」を A/B）。非再発を確認。

**共通**: コンパイルエラー0（`read_console`）を確認してから main マージ（CLAUDE.md ルール）。

---

## 6. ロールバック

`useDepthBasis` フラグで即座に旧 voxel 基準へ復帰可能。両カーネルを共存させ、品質が確認できるまで
既定は voxel 基準。フェーズ1が本番品質で通ってから既定を depth 基準へ切替。

---

## 7. 決定事項（2026-07-07 ユーザー合意）

1. **レース戦略**: **2-pass InterlockedMin（厳密）** で行く。
2. **本番モード**: 未確定だが現状の最有力は「**curved line + curve 始点/終点の body mesh**」。
   その body mesh は live-follow の瞬間スナップショット。→ **フェーズ1（live-follow RetainGhost）から着手**
   （perf 実測モードとも一致）。accumulate はフェーズ2。
3. **surfel スタンプ形状**: 軸並行 3×3×N 近似で始め、live で穴が消える最小サイズに詰める（法線推定なし）。

## 8. フェーズ1 実装メモ（2-pass InterlockedMin の具体化）

- **4カメラをバッチ単位でまとめて処理**する。現行は camera 到着イベントごとに integrate していたが、
  `_states[serial]` は4台ぶんの depth/color を保持し続けるので、到着時は**アップロードのみ**、4台目到着
  （batch 完了）で pass1×4 → pass2×4 → Publish をまとめて回す。→ `_VoxelKey` の clear が **batch 毎1回**で済む。
- **`_VoxelKey`（uint/voxel）**: `key = (quant|sdf|<<16) | (signBit<<15) | quantSignedLow`。
  InterlockedMin で「表面に最も近い観測」が勝ち、**signed sdf も key から復元可能**（別 sdf バッファ不要）。
  weight は wObs 一定なので「触られた voxel か否か」= key!=sentinel で判定でき、MC ゲート用 weight は pass2 で wObs 書込。
- **pass1（scatter）**: 各画素が surfel band voxel に `InterlockedMin(_VoxelKey[v], key)`。4カメラ横断で最小 |sdf| に収束。
- **pass2（scatter）**: 各画素が band voxel を再走査、`key==_VoxelKey[v]` の voxel に `_Voxels[v]=(unpack(key), wObs)`
  と `_Colors[v]=rgb` を書く（勝者だけ書くので決定的、タイは同 sdf で色 last-writer=軽微）。
- **clear**: batch 開始で `_Voxels`（weight=0, sdf=+tau, 既存 ClearWrite）+ `_VoxelKey=0xFFFFFFFF`。
  full-grid clear が残るので、これが新しい支配費になる。**フォロー最適化**: active-block 限定 clear か
  generation-counter 方式で全 grid clear を消せる（フェーズ1.5、まず full clear で正しさ優先）。
- **逆投影**: `PointCloudReconstruct.compute` の undistort 逆投影（`DepthUndistortLut`）を流用し表面点を点群と一致させる。

## 9. フェーズ1a 実装結果（2026-07-07、live playback で実測）

**実装済み**（branch `feat/tsdf-depth-basis-integration`、`useDepthBasis` 既定 OFF で main 影響なし）:
- `Resources/TSDFIntegrateDepth.compute`（ScatterMin / ScatterWrite の 2-pass）。
- `TSDFVolume._keyBuf` + `ClearWriteKey()`（0xFFFFFFFF）、`TSDFClear.ClearUint` に `_UintClearValue` 追加。
- `TSDFIntegrator`: `useDepthBasis` フラグ、バッチ単位アップロード→`RunDepthBatch`、ray LUT キャッシュ、
  `PrepareCamState` 抽出。

**スタンプ設計の変遷（landmine B の実測解決）**:
1. ray-march + ±1 面近傍 → **穴だらけ**（vert ratio 0.32、網戸状）。
2. 実体 AABB carve（pad=1〜2、z 基準 |sdf| ゲート）→ **正対面で穴は消えるが斜面で肥大**（ratio 2.9〜3.3、ブロブ）。
3. capsule（ray からの垂直距離ゲート）→ 正対と斜面の要求半径が両立せず（tube1.2 で ratio 0.38）。
4. **投影ゲート（採用）**: 各候補 voxel を歪み込みで depth 画素へ forward-project し、`floor(u,v)` が
   その画素と一致する voxel だけ書く。= voxel 基準の gather セマンティクスを表面 voxel だけで再現。
   → **肥大も穴も原理的に無い**。

**確定した数字（投影ゲート、interleaved A/B でドリフト相殺）**:

| | ms/frame | fps | verts |
|---|---|---|---|
| voxel 基準（現行） | 75.4 | 13 | 625,719 |
| **depth 基準（投影ゲート）** | **27.3** | **37** | 478,519 |
| 差 | **−48.1 ms（2.75×）** | | vert ratio **0.76** |

- **座標一致**: depth 基準メッシュは人物が正位置・正スケール・正色で再構成（screenshot 検証済み、`capture/cmp3_*`）。
- **視覚パリティ**: 同一 instant で voxel 基準とほぼ区別不能。ratio 0.76 の差は free-space 境界セル + シルエット rim
  由来で、**可視の穴は無い**。
- **VRAM**: `_keyBuf` +516MB（129M×4B）。5.9GB→6.4GB、OOM 無し。

**残タスク / フォロー最適化**:
- **フェーズ1b**: full-grid clear（`_Voxels` + `_VoxelKey` = 258M/batch）が新しい支配費。active-block 限定 clear か
  generation-counter で消せば 27ms からさらに下がる余地。
- ratio を 1.0 に寄せるなら pad 微調整 or free-space 境界の扱い（視覚上は不要）。
- **フェーズ2**: accumulate（Average+Fold）経路の depth 基準化（thin-limb 修正の移植、atomic 加算）。
- 既定を depth 基準に切替えるのは本番モードでの品質承認後（現状は `useDepthBasis` 既定 OFF）。
