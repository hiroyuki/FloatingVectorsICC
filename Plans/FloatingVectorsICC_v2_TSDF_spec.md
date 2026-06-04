# FloatingVectorsICC v2 — Motion-to-Mesh 実装仕様書

> Claude Code 向け実装仕様。既存の FloatingVectorsICC (Unity 6 / Windows / OrbbecSDK v2 / 4× Femto Bolt) プロジェクトに組み込む前提。
> 目的: ダンスなど速い動きの人体を 4 台 RGB-D で捉え、**動きの軌跡を蓄積した 1 つの立体メッシュ**をリアルタイム生成・表示する。

---

## 0. 全体像

```
4× デプスフレーム (ワールド座標統合済み)
   │
   │  ← body tracking スケルトン (統合済み・関節 confidence あり)
   ▼
[速度適応 中割り (sub-frame) 生成]  ← 関節補間で点をwarp
   │
   ▼
[TSDF 積算 Compute Shader]  ← 残像保持ルール / 1 個のボリュームに 4 台を合成
   │
   ▼
[Marching Cubes Compute Shader]  ← 毎フレーム
   │
   ▼
[Unity Mesh / DrawProcedural で表示]
```

中核となる設計判断:
- メッシュの**形状精度はデプス (TSDF) 由来**。スケルトンは**中割りの対応付けガイド**としてのみ使用 (スケルトン誤差が直接メッシュ誤差にならない)。
- TSDF ボリュームは**カメラ台数によらず 1 個**。4 台は同じボリュームに合成する。
- 蓄積は**残像保持ルール** (重み付き平均ではなく min 系)。動きの軌跡を消さず積み上げる。

---

## 1. 入力前提 (既存プロジェクトで充足済み)

- [x] 4 台のデプスフレームが **ワールド座標で統合可能な状態**で取得できている。
- [x] 各カメラの内部パラメータ (intrinsics) と外部パラメータ (extrinsics: world→camera 4x4) がキャリブ済みで利用可能。
- [x] body tracking スケルトンが**統合済み**で取得できる。**関節ごとの confidence / tracking state を持つ**。
- [x] Compute Shader からデプスフレームに Texture としてアクセスできる。
- [x] **Bounding Volume が既に定義済み**で、その内部のみ可視化する仕組みがある → そのまま TSDF の対象領域として使う。

> 既知の精度問題: 統合スケルトンは 4 台由来のズレ・トラッキング誤差を含む。**特に足元 (脚交差・床近傍・自己遮蔽) でズレやすい**。下記 confidence ベースの濃淡処理で対処する。

---

## 2. TSDF ボリューム定義

### 2.1 等方ボクセル (重要)
固定分割数 (256³ 等) は使わない。Bounding Volume の各辺長が異なると直方体ボクセルになり、Marching Cubes の面と距離計算が軸方向に歪むため。**立方体ボクセルを敷き詰める。**

```
voxelSize = 0.01 m   // 初期 1cm。調整可パラメータ。
dimX = ceil(boundingVolume.sizeX / voxelSize)
dimY = ceil(boundingVolume.sizeY / voxelSize)
dimZ = ceil(boundingVolume.sizeZ / voxelSize)
```

- ボクセルは常に 1cm³ の立方体。各軸の分割数は辺長に応じて可変。
- 例: 3m×2m×4m → 300×200×400 = 2,400 万ボクセル。

### 2.2 ストレージ
- `RWStructuredBuffer<float2>` または `RWTexture3D<float2>` (x = tsdf 値, y = weight)。
  - 推奨: `RWStructuredBuffer<float2>` (非立方次元で扱いやすい)。インデックス = `x + dimX*(y + dimY*z)`。
- メモリ概算: 2,400 万 × float2 (8byte) ≈ 192 MB。1cm で現実的。
- 初期化カーネルで tsdf = +τ (または無効値), weight = 0 にクリア (キャプチャ開始時)。

### 2.3 Truncation
```
tau = 4 × voxelSize   // 初期 4cm 程度。調整可。
```

---

## 3. TSDF 積算 Compute Shader

**1 スレッド = 1 ボクセル。** 各ボクセルが 4 台ループを回し、1 個のボリュームへ合成。

### 3.1 1 ボクセルあたりの処理
```
voxelWorldPos = boundingVolume.min + (voxelIndex + 0.5) * voxelSize

for cam in [0..3]:
    p_cam   = extrinsics[cam] * voxelWorldPos        // world→camera
    if p_cam.z <= 0: continue                        // カメラ後方は無視
    uv      = project(p_cam, intrinsics[cam])        // 画像へ射影 (=視線方向の決定)
    if uv が画角外: continue
    d       = depth[cam].Sample(uv)                  // その視線方向の表面までの実距離
    if d 無効 (0 や NaN): continue

    sdf = d - length(p_cam)                          // 符号付き距離 (視線方向)
    // 符号: 表面より手前(外)=+, 奥(内)=-

    // --- Truncation (非対称) ---
    if sdf < -tau: continue                          // 表面の奥すぎ → このカメラは無効 (weight 0)
    sdf = clamp(sdf, -tau, +tau)                     // 手前の遠方は +tau に丸めて空きとして記録

    // --- 観測重み (任意で精度向上) ---
    w_obs = 1.0
    // オプション: 視線と表面法線の角度で重み付け (正面を優遇、斜めを減衰)
    // オプション: デプス距離が遠いほど w_obs を下げる

    integrate(voxel, sdf, w_obs)   // ← 蓄積ルール (3.2)
```

### 3.2 蓄積ルール (差し替え可能に設計)
2 モードを用意し、フラグで切替可能にする。**初期は残像保持。**

**(A) 残像保持 (初期・本命) — 動きの軌跡を残す**
「これまでで最も表面に近かった記録」を保持する。
```
// 符号付き距離の絶対値が小さい = より表面に近い観測を優先して残す
if abs(sdf) < abs(voxel.tsdf) || voxel.weight == 0:
    voxel.tsdf   = sdf
    voxel.weight = max(voxel.weight, w_obs)
```
> これにより、手を突き出して引っ込めても「通過した瞬間」の表面記録が残り、軌跡が立体として蓄積される。

**(B) 標準 TSDF (debug / 静止スキャン用) — 重み付き平均**
```
voxel.tsdf   = (voxel.tsdf*voxel.weight + sdf*w_obs) / (voxel.weight + w_obs)
voxel.weight = voxel.weight + w_obs
```
> 動く人では残像がならされて消えるため本番では使わない。検証用。

### 3.3 床クリップ
足が床に溶けるのを防ぐ。床平面 (キャリブで既知, 例 world y = floorY) より下/近傍のデプス寄与を積算前にクリップ。
```
if voxelWorldPos.y < floorY + floorMargin: skip 積算 (または weight 抑制)
```
`floorMargin` 初期 1〜2cm。

---

## 4. 速度適応 中割り (Sub-frame Interpolation)

30fps のフレーム間ギャップを、body tracking ガイドで埋める。**速度に応じて枚数可変。**

### 4.1 中割り枚数の決定
```
// confidence の高い関節だけで最大移動量を測る (足元の誤差で暴れさせない)
maxDisp = max over { joints with confidence >= confThreshold } ( |pos(t+1) - pos(t)| )

stepThreshold = 3 * voxelSize        // 初期 3cm。調整可。
maxSubframes  = 8                    // 負荷上限。調整可。
N = clamp( ceil(maxDisp / stepThreshold) - 1, 0, maxSubframes )
```
- 止まっている局面: N = 0 (中割り不要、軽い)
- 高速局面: N 増加 (軌跡が途切れない)
- 天井超過の超高速部は tau 拡大で吸収 (完全連結でなくても破綻させない)
- **debug 用に N 固定 (=3) モードのフラグも残す。**

### 4.2 補間と warp (スキニング)
- 各補間時刻 k/(N+1) (k=1..N) で**関節姿勢を補間** → 補間スケルトンを得る。
  - 補間: **線形 (初期)**。将来 **Catmull-Rom スプライン**に差し替え可能な設計に (急な方向転換でのショートカットを防ぐ)。
- **中割り対象 = 各カメラのデプス由来の 3D 点** (point cloud。ボクセルではない)。
- **スキニング: 各点を最寄り関節 1 個に割り当て** (初期の単純法)。補間スケルトンの該当関節の剛体変換に従って点を移動。
  - 将来: 複数関節の加重スキニングに拡張可能に。
- **confidence の低い関節に紐づく点は中割りしない** (実フレームのみ積算)。嘘の中割りで足元が暴れるのを防ぐ。
- warp した中割り点群 (= 中割りデプス相当) を、実フレームと同じく §3 で TSDF へ積算。

> 実装簡略化の選択肢: point cloud を明示的に作らず、デプス画像をスケルトン補間に従って直接 warp する方式も可。どちらでも可、まず実装が軽い方で。

### 4.3 関節軌跡の時間平滑化
足元等のフレーム間ジッタ低減のため、ガイドに使う前に関節軌跡へ **One Euro Filter** をかける (速い動きを鈍らせずジッタのみ除去)。

---

## 5. Marching Cubes Compute Shader

- 入力: §3 で更新された TSDF ボリューム。**標準 Marching Cubes (改造不要)。**
- 等値面 = 0 (tsdf 符号が +→- に変わる境界)。weight == 0 のボクセルは無効として扱う。
- 出力: `AppendStructuredBuffer<Triangle>` に頂点を追記。
- **実行頻度: 毎フレーム。** 重い場合は数フレームおきに間引くフォールバックフラグを用意。
- 標準の 256 パターン (15 ベースケース) エッジテーブル / トライテーブルを使用。

---

## 6. 表示

- MC 出力頂点を **`DrawProcedural`** で直接描画 (毎フレーム更新なので Mesh 再構築より効率的)、または Unity `Mesh` へ書き戻し。**初期は表示のみ。**
- 既存の Bounding Volume 可視化と整合させる。

---

## 7. セッション / ライフサイクル

```
captureDuration = 10.0 s   // 初期。可変パラメータ。
```
- キャプチャ開始: TSDF ボリュームをクリア。
- キャプチャ中: 毎フレーム [速度適応中割り → TSDF 積算 → MC → 表示]。塊が育つ。
- captureDuration 経過 or 停止トリガー: 積算停止、最終メッシュを保持表示。
- 残像減衰: 初期は**なし (完全保持)**。蓄積ルールが差し替え可能なので、将来「経時減衰」「無限蓄積」へ切替可能に設計。

---

## 8. パラメータ一覧 (Inspector 公開推奨)

| パラメータ | 初期値 | 説明 |
|---|---|---|
| `voxelSize` | 0.01 m | 等方ボクセル 1 辺 |
| `tau` (truncation) | 4 × voxelSize | 切り詰め幅 |
| `accumulationMode` | RetainGhost | 残像保持 / 標準TSDF(debug) |
| `stepThreshold` | 3 × voxelSize | 中割り 1 ステップ最大移動量 |
| `maxSubframes` | 8 | 中割り枚数上限 |
| `subframeMode` | Adaptive | 速度適応 / 固定(=3, debug) |
| `interpMode` | Linear | 線形 / Catmull-Rom |
| `confThreshold` | (要調整) | 中割りに使う関節 confidence 下限 |
| `floorY` / `floorMargin` | キャリブ値 / 0.02 m | 床クリップ |
| `captureDuration` | 10.0 s | セッション長 |
| `mcEveryNFrames` | 1 | MC 実行間引き (1=毎フレーム) |

---

## 9. 実装順序 (推奨)

1. **TSDF 積算 (標準TSDF・静止物) + Marching Cubes + 表示** を先に通す。
   - まず静止した人/物で「デプス → TSDF → メッシュ」が出ることを確認 (中割りなし、accumulationMode=標準)。
2. **蓄積ルールを残像保持に切替**。動く対象で軌跡が残ることを確認 (まだ中割りなし → 飛び石になる)。
3. **固定 N=3 の中割り**を追加。飛び石が繋がることを確認。
4. **速度適応 N + One Euro 平滑化 + 床クリップ**を追加。ダンスで検証。
5. 補間を Catmull-Rom、スキニングを加重に強化 (必要なら)。

---

## 10. 既知の限界・注意 (実装時に踏みやすい点)

- **薄い物体に弱い (TSDF の既知の限界)**: 突き出した手のひら・指など薄い部位を表裏のカメラが見ると符号が打ち消し合い、欠け/穴が出る。tau を小さめにする・視線角度で重み付けすると緩和。
- **足元ズレ**: §3.3 床クリップ + §4.1 confidence 棄却 + §4.3 平滑化 の 3 点セットで対処。
- **超高速部の連結漏れ**: maxSubframes の天井を超える動きは tau で吸収。完全連結を狙うと負荷が爆発するのでガード必須。
- **4 台フレームの時刻同期**: ズレると速い動きで二重像。Femto Bolt のハード同期 (sync ケーブル) を効かせると改善。OrbbecSDK v2 のマルチデバイス同期設定を確認。
- **メモリ**: voxelSize を小さくすると急増 (1cm→192MB, 0.5cm→1.5GB)。GPU メモリと相談。
- **エクスポート**: OBJ/PLY 書き出しは将来用に口だけ用意 (今回は未実装で可)。
