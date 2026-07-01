# 引き継ぎ: スケルトン(bodies_main)が点群に対して縦にズレる件

作成 2026-07-01 (Mac セッションからの引き継ぎ / 続きは Windows でデバッグ)

## 症状（一言で）
再生時、**k4abt スケルトン(骨)が点群に対して縦方向にズレて浮く**。
単純な平行移動ではなく、**スケルトンが縦に約12%圧縮されて上寄りに配置**されている。

- 立位フレームで、点群の足は床(≈ y −0.98)についているのに、**スケルトンの足は y ≈ −0.55〜−0.62 で ~37〜45cm 上に浮く**
- 頭は ~15cm 上、足は ~40cm 上（**下ほどズレが拡大**）
- スケルトン身長 1.55m vs 点群人物 ~1.76m（≈ 12% 縦圧縮）
- **奥行き z はほぼ一致**（骨頭 z=3.635 vs 点群 3.68、差 ~5cm）→ 奥行きスケール異常ではなく、縦(Y)方向固有

## 検証データ
- 録画: **Rec2_jump**（214フレーム）
  - Mac: `/Users/horihiroyuki/Dropbox/projects/ICC/Recordings/Rec2_jump`
  - Win: `PointCloudRecorder.folderPath = D:\FloatingVectorsICC\Recordings_jump2`（※ Mac override が Rec2_jump を指してる。Windows では folderPath 側を Rec2_jump に合わせること）
- フレーム: **cursor 100**、原点カメラ **CL8F253004N**（extrinsics で identity＝world=カメラローカル。切り分けが一番綺麗）
- 人物は**立位**（点群の足元Yヒストグラムが −1.0→−0.2 連続、空中ギャップ無し）

### 実測値（cursor 100, camera N, identity）
| joint | スケルトン world | 点群(同XZ) | 差(骨−点群) | 生 k4a mmY |
|---|---|---|---|---|
| HEAD | (−0.157, 0.929, 3.635) | 頭頂 0.782 | +0.15 | −929 |
| PELVIS | (0.006, 0.286, 3.679) | — | ~0(柱状で不定) | −286 |
| FOOT_L | (−0.016, −0.617, 3.966) | 足元/床 −0.986 | **+0.37** | 617 |
| FOOT_R | (0.277, −0.530, 3.978) | 足元/床 −0.980 | **+0.45** | 530 |

## 確定している事実（切り分け済み）
1. **スケルトン数値 = bodies_main の生値そのまま**。描画値(BodyVisual.JointPosition)と生値(ToWorld)を全関節照合し diff=0.0000m。静止フレームなので One-Euro フィルタ/マージも実質 no-op。変換は `K4AmmToUnity`（mm→m + Yのみ反転 `(x,−y,z)*0.001`）＋原点カメラは identity のみ。**倍率・オフセットの後付けは無し**。
2. **深度 intrinsic は点群と recompute で完全一致**：
   - 点群 = 録画埋め込み `CameraParam.DepthIntrinsic`
   - recompute = `extrinsics.yaml` の `depth_intrinsic`
   - 両者 fx=252.4085, fy=252.3994, cx=167.7483, cy=169.3915, 320×288（一致）
3. **C# の recompute / calibration / playback 経路は正しい**（バグ見当たらず）:
   - `OfflineBodyRecomputeWindow.OnSkeletons`：worker の関節を無変換コピー
   - `K4ACalibration.WriteCamera`：intrinsic を k4a 期待順 **[cx, cy, fx, fy, k1..k6, codx, cody, p2, p1, metric_radius]**、count=14、BROWN_CONRADY、深度 extrinsics=identity
   - `PickDepthMode(320,288)=NFOV_2X2BINNED`（正しい）
4. 点群の床 y≈−0.98 は**カメラ位置的にも妥当**（ユーザー確認）。

## 除外した仮説
- **depth→color 外部パラメータ**：T = [−32.3, **−0.8**, 2.5] mm。縦成分ほぼ0 → 縦15〜40cm は説明不能。**除外**。
- **時間ずれ(bodyLeadFrames)**：0/+1/+2/+3/−2 で頭-頭頂差は 0.13〜0.17 とほぼ不動 → **除外**。
- **座標/transform バグ(C#側)**：`SkeletonMerger.transform`・`_root` とも identity、描画スフィア world = JointPosition 一致 → 描画経路のズレ無し。**除外**。
- **マージ(4カメラ blend)**：原点カメラ単体に絞っても出る → merge 起因ではない。**除外**。
- **計測アーティファクト（重要な落とし穴）**：playback 点群は GPU 再構成で、**`mesh.vertices`(CPU) は STALE/NaN**。正しくは **`mesh.GetVertexBuffer(0)` を GetData**（stride 24 = pos3+col3 float）。序盤の「4mズレ/NaN」は全部 CPU 頂点を読んでたこちらのバグ。**GPU バッファで読めばクリーン**。

## 本命の疑い（Windows で見るべき所）
**k4abt ワーカー(`k4abt_worker.exe`)内部の calibration/深度モード解釈**が、縦方向に圧縮された3D関節を出している可能性。
- ワーカー = `Workers/K4abtWorker/.../k4abt_worker.exe`（.NET8 win-x64、別プロセス、MMF IPC）。実際に k4abt 骨格推定を走らせて関節座標を出す張本人。**Windows 専用**（Mac では走らない＝録画 bodies_main を読むだけ）。
- 症状の性質（z一致・Y下ほど拡大・縦12%圧縮）は、fy/cy 相当の縦スケール食い違い、または k4abt が受け取る calibration blob の解釈差に整合的。intrinsic 値自体は一致しているので、**blob のバイナリレイアウト/深度モード/depthの単位や format をワーカーがどう k4abt に渡しているか**が焦点。

## 次の手（Windows）
1. **ライブ k4abt と比較**：Femto Bolt 実機でライブ骨格 vs ライブ点群。**ライブでも縦圧縮/浮きが出るか**を見る。
   - 出る → recompute 固有ではなく k4abt calibration/経路の問題
   - 出ない → recompute 固有（offline 経路 or 供給データの差）
2. **ワーカーを instrument**：受け取った k4a calibration blob をダンプ、ある関節の z と「その画素の深度実測値」を突き合わせ、縦方向のズレを定量化。
3. **calibration blob レイアウト検証**：`K4ACalibration.cs` の offset 群（`SizeofCalibration=1032`, `OffsetDepthMode=1024` 等）を、インストール済み K4A Wrapper の実 `k4a_calibration_t` 構造体と再照合（SDK バージョン差でズレると全滅級だが、今回は z 一致なので致命ではない…要確認）。
4. **深度 format/単位**：ワーカーに渡す depth (Y16 mm, 320×288) が k4abt の期待どおりか。depth mode と実キャプチャモードの一致。

## 関連ファイル
- `Assets/Scripts/BodyTracking/Editor/OfflineBodyRecomputeWindow.cs` — offline recompute 本体（メニュー: `Window > Body Tracking > Recompute Bodies (offline)`）
- `Assets/Scripts/BodyTracking/K4abtWorkerHost.cs` — ワーカー spawn + MMF IPC
- `Assets/Scripts/BodyTracking/K4ACalibration.cs` — k4a calibration blob 構築（← 要精査）
- `Workers/K4abtWorker/` — ワーカー本体ソース（Windows）
- `Assets/Scripts/BodyTracking/MultiCam/SkeletonWorldTransform.cs` — `ToWorld`(K4AmmToUnity + renderer transform, localScale skip)
- `Assets/Scripts/PointCloud/Resources/PointCloudReconstruct.compute` — 点群(color空間, LUT undistort)。`PointCloudReconstructor.ForcePinhole` で歪み補正 on/off
- `Assets/Scripts/PointCloud/FloorOrigin.cs` — 床グリッド = bounding box 底面中心（`TransformPoint(0,−0.5,0)`）。**実床検出ではない**

## MCP で再現する時の手順メモ
- 再生は暴走しがち＆セッション状態が脆い。`TogglePlay(); PausePlayback();` を1コール内で → その後 `StepForward()` をループで目的 cursor まで（Update を跨がないので暴走しない）。`SeekAllTracksTo` は Idle 時は効かない。
- 点群を見えなくする犯人: `PointCloudBoundingBox`(KeepInside/Outside) と `PointCloudDecimater`。素の点群を見るなら両方 `enabled=false`。
- カメラ単体骨: `SkeletonMerger._latestBySerial` の他カメラ slot の `BodyCount=0` にする（ただし StepForward で再 emit されると復活）。
- **点群座標は必ず GPU 頂点バッファから**（`mesh.vertexBufferTarget |= Raw; mesh.GetVertexBuffer(0)`）。

## このセッションでコミット済み（本件とは別）
- `fix/playback-object-leak` → main（push 済み）: `_Playback_<serial>` GO がシーンに漏れ蓄積するバグ修正（`ClearTracks` に名前掃除追加 + `OnDisable` で `ClearTracks`）。
- `feat/bt-trail-sdf-bake` → main（push 済み）: BT trail を SDF に焼いて MC 再メッシュする機能（本件とは無関係）。
