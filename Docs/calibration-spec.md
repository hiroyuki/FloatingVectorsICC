# キャリブレーション仕様（マルチカメラ extrinsic + 床）

2026-07-20 のセッションで確定・実装した仕様。実装は `CalibrationRuntimeUI`
(`Assets/Scripts/Calibration/RuntimeUI/`) と `PairwiseCalibrationMath` /
`WorldFrameRebase` / `PointCloudRecording`。

現地手順は [onsite-setup-procedure.md](onsite-setup-procedure.md) を参照
（このファイルは「何がどう決まるか」の仕様、あちらは「どの順にやるか」の手順）。

## 全体の流れ

```
① カメラID割当 (I)  →  ② マーカー収集 (C) ×N  →  ③ ソルブ (S)
                                                    ├ カメラ相対位置 (pairwise)
                                                    ├ 世界の水平化（床アンカー）
                                                    ├ rig 周回順の自動導出
                                                    └ sensing area / 床グリッド / frustum
                            ④ アプリ再起動  →  ⑤ 床高微調整 (G + ↑↓)
```

③ と ⑤ を**別セッションに分ける**のが確定運用。理由は「既知の問題」参照。

## 生成される 4 ファイル（すべてマシンローカル）

`<persistentDataPath>/Recordings/recording/calibration/` に置かれる。
**git 管理外・シーン外**。展示は同一構成セットが 2 つ（4070 / 5080）あり、
シリアルも部屋も違うため、セット間で持ち回ってはいけない値だけをここに置く。

| ファイル | 内容 | 書く人 |
|---|---|---|
| `extrinsics.yaml` | 各カメラの intrinsic / distortion / `global_tr_colorCamera` | ソルブ (S) |
| `cameras.yaml` | camera-id ↔ serial の対応、`origin_serial` | assign mode (Enter) / ソルブの自動導出 |
| `floor.yaml` | `rebase_floor_y`（床高）+ `floor_leveling_*`（傾き補正） | 床チューン (G → Enter で全点フィット、↑↓ は 0.75 秒デバウンス保存) |
| `sensing_area.yaml` | BoundingVolume の `center` / `size` | `ExperienceSpaceBuilder.Apply()`（ソルブ / `B` / メニュー） |

`floor.yaml` は `SensorManager.Start` が読んで `rebaseFloorY` を上書きする
(`loadFloorYFromCalibration` で無効化可)。Inspector 値は Play 停止で消えるため、
床高だけはファイル側を正とする。

## キー一覧（キャリブモード = F1）

| キー | 動作 |
|---|---|
| **F1** | キャリブモード ON/OFF（BT / TSDF 自動サスペンド、playback 停止、カラー解像度ブースト） |
| **I** | camera-id 割当モード（矢印で選択、0-9 で id、**O** で origin、**Enter** で保存、Esc/I で終了） |
| **C** | サンプル収集（全カメラ同時 + skew ゲート） |
| **F** | 床サンプル（ボードを床に平置き。BoardSample モード時のみ意味を持つ） |
| **S** | ソルブ |
| **G** | **床チューンモード（キャリブモード非依存 — 単独で使える）** |
| **B** | sensing area をカメラ配置から再フィット → `sensing_area.yaml` 保存 |
| **Enter** | 床チューン中: 見えている床の点を全部使って平面フィット → 傾き＋高さを適用・保存（**無人で実行**） |
| **↑/↓** | 床チューン中: 床を 1cm 移動（Shift で 1mm）。保存は 0.75 秒デバウンス＋退出時 |
| **R** / **D** / **Backspace** / **H** / **0-9** | リセット / フレームダンプ / サンプル消去 / UI 隠す / ソロ表示 |

## ① camera-id 割当

`cameras.yaml` のリスト順がそのまま id（先頭 = id 0）。`origin_serial` は
ソルブで world 原点に固定されるカメラ（id 順とは独立）。

- 外部ディスプレイは **id 0/1 → display2、id 2/3 → display3**。
  別セットの serial が id 0-3 を占有していると、接続中カメラが id 4+ に押し出されて
  **どの外部ディスプレイにも映らなくなる**（実際に発生した）。新しいマシンでは
  まず他セットの serial を消すこと
- 切断中のカメラも `cameras.yaml` に残る（再接続で id が変わらないように）

## ② サンプル収集

- 全カメラのカラーフレームを同時スナップショットし、**タイムスタンプの skew が
  `maxSkewMs` 以内**のときだけ採用（超えたサンプルも記録は残るがソルブでは無視）
- ボードは静止させてから C を押す（動きながらだと skew が 30ms 超になる）
- **2 台に同時に映れば十分**。4 隅配置ではまず無理なので、各辺の中間にボードを置いて
  その辺の両端 2 台に見せる。4 辺すべてで撮ってペアのグラフを輪にする
- 特定カメラのサンプルが薄いと**そのカメラだけ位置が壊れる**（CA–C6 が 4.6m のはずが
  1.31m になる事例が出た）。各辺 5〜10 枚が目安

### カラー解像度ブースト

キャリブモード入場時に `calibColorWidth/Height`（既定 1920×1080）へ切り替え、
退場時に本番解像度へ戻す（`boostColorResolution` で無効化可）。
マーカーの検出可能距離は画像上のピクセル数にほぼ線形なので、1280×960 → 1920×1080 で
検出レンジが約 1.3〜1.5 倍になる。切替のたびにパイプライン再起動（約 15 秒）。

## ③ ソルブ

### 相対位置（pairwise）

同一サンプル内で同じボードを見た 2 台の間に相対姿勢のエッジを張り、
`origin_serial` のカメラを world 原点（identity）として BFS で全体へ伝播する。
cam0 に到達できないカメラがあるとソルブは中断される（そのカメラ用のサンプル不足）。

### 床アンカー（世界の水平化）— `floorAnchor`

pairwise の結果は「cam0 のレンズ基準」なので、そのままでは世界が傾いている。
2 方式あり、**既定は CameraPlaneAssumeLevel**。

| モード | 決め方 | 精度 |
|---|---|---|
| `CameraPlaneAssumeLevel`（既定） | 4 台のカメラ位置のベストフィット平面を水平とみなし、`cameraHeightMeters`（既定 1.0m）の高さに置く。下向きはカメラの画像下方向（`global_tr_cam` の回転列 1）の平均で判定 | **良好**。4.6m スパンで平面を決めるので条件が良い |
| `BoardSample` | F で撮った「床に平置きしたボード」の平面を y=0 にする | **要注意**。遠くから浅い角度で撮ると平面法線が出ず、世界が丸ごと傾く（約 25° 傾いた事例あり）。使うならカメラの 1.5〜2m 前に置く |

物理的にカメラが水平・同一高さに設置されていることが CameraPlaneAssumeLevel の前提。
ソフトで直すのではなく**リグ側を水平に組む**のが正しい。

### rig 周回順の自動導出

`WorldFrameRebase` は `rigSerialOrder` に「+X = cam1→cam2、+Z ≈ cam2→cam3（許容 10°）」
という**周回順**を要求する。ソルブは水平化後の座標から 6 通りの並びを試し、
このゲートを通る順序を見つけて `cameras.yaml` に保存する。
以後、手入力の順序合わせは不要。

### 後処理（自動）

`applySensingAids` が ON なら、ソルブ成功時に:
- `ExperienceSpaceBuilder.Apply()` — 4 カメラ位置から 80cm 内側に寄せた矩形で
  BoundingVolume をリシェイプ（sensing area）。高さは `areaHeight` = 2.5m、
  rotation は必ず identity（床は世界側で水平化済みなので箱を傾けない）
- 床グリッド ON（`FloorOrigin.showGrid` / `fitToBoundingBox`）
- 全カメラの frustum マーカー表示

シーンに `ExperienceSpaceBuilder` が無ければ `ExperienceDirector` の参照を借りて
ランタイム生成する（本番で director がやるのと同じ組み方）。

### sensing area はマシンローカル（`sensing_area.yaml`）

箱はカメラ位置から算出されるので **セットごとに違う値**になる。シーンは 2 セットで
共有しているため、`main.unity` に箱を焼き込むと**もう一方のセットで静かに間違った箱**
になる（見た目が「それらしい」ので気づきにくい）。よって `cameras.yaml` / `floor.yaml`
と同じ扱いにする。

```
persistentDataPath/Recordings/recording/calibration/sensing_area.yaml
  center: [x, y, z]      # ワールド m
  size:   [x, y, z]      # ワールド m
  inset_meters: 0.8      # 算出に使った値（再現用）
  area_height: 2.5
```

- `ExperienceSpaceBuilder.Apply()` が算出後に保存
- `SensorManager.Start()` が読み込んで BoundingVolume に適用
  （`loadSensingAreaFromCalibration`、既定 ON）
- **yaml が無いマシンではシーンの値がそのまま使われる**。シーンの箱は
  原点中心 3 x 2.5 x 3 m の中立なフォールバックで、どちらのセットの実測値でもない
- rotation は常に identity（床は世界側で水平化済み）

**更新方法は 3 つ**、どれも同じ yaml を書く:

| 方法 | いつ使うか |
|---|---|
| ソルブ（`S`）に自動で付随 | キャリブをやり直したとき |
| キャリブ UI の `rebuildSensingAreaKey`（既定 `B`） | カメラを動かしたが再ソルブまでは不要なとき |
| メニュー `Window > Calibration > Rebuild Sensing Area` | Play していないとき / キャリブセッション外 |

## rigSerialOrder の解決順（重要）

`PointCloudRecording.ResolveRigSerialOrder` が一元管理する:

1. **`cameras.yaml`**（4 台ぶんあれば採用）
2. シーンの `rigSerialOrder`（フォールバック）

`SensorManager` は 1 が rebase のゲートを通らなかった場合に 2 で再試行する
（`cameras.yaml` は id マップであって周回順とは限らないため）。

再生時の `SensorRecorder` は**再生フォルダ側の `cameras.yaml`** を見る。
録画時に `Save` がリグの `cameras.yaml` を録画フォルダへコピーするので、
別セットで撮ったテイクを再生しても録画元のリグ順で正しく rebase される。

> なぜこうしたか: `rigSerialOrder` はシーンにシリアライズされるため、git で
> 4070/5080 間を往復すると必ずどちらかで他セットの serial になる。ID マップは
> マシンローカルなので、そちらを正とした。

## ④→⑤ 床高の微調整（G）

キャリブモードとは独立に動く。入ると:
- `PointCloudView.showPointClouds` を強制 ON、キャリブが隠したメッシュも復帰
- BoundingVolume を `KeepInside` に切り替え、**箱の内側だけ**表示
- 外部ディスプレイの黒背景キャンバスを一時的に畳む（**これが無いと 3D が全部隠れる** —
  display0 は黒塗りカメラで、3D は display1/2 に描かれ、その上にキャリブの
  全画面黒背景が sortingOrder 100 で乗っているため）

↑↓ で `rebaseFloorY` を動かし、`ApplyExtrinsicsToLive()` で即反映 →
床の点群がギリギリ消える位置が y=0。押すたび `floor.yaml` に保存される。

> Inspector で `rebaseFloorY` を直接書き換えても**再適用されない**。
> 反映するには ↑↓ を押すか `ApplyExtrinsicsToLive()` を呼ぶ。

## 検出オーバーレイ（display2 / display3）

外部ディスプレイは生カラーをフレームレートで流しつつ、検出したマーカー枠（シアン）と
ChArUco コーナー（黄）を**ベクター描画で重ねる**（`DetectionOverlay`、UGUI Graphic）。
更新は検出パスと同じラウンドロビン周期（`detectIntervalSec` 既定 0.12s/台）。
H で UI を隠すと検出自体が止まるため、オーバーレイも消える（古い枠が残らないように）。

ラベルは `[ID n ★origin] SERIAL  ● M=3 C=2  smp 5/8 (skew-ok 7)` 形式
（M=マーカー数、C=コーナー数、smp=そのカメラが検出できたサンプル数/全体）。

## 実測値（5080 セット、2026-07-20）

正常なソルブの目安:

| 項目 | 値 |
|---|---|
| 4 辺 | 4.55 / 4.56 / 4.59 / 4.61 m |
| 対角 | 6.39 / 6.56 m（理論 6.51） |
| カメラ高さ | 4 台とも同一（ばらつき 0.00 m） |
| camera-id 順 | CA → C6 → F0 → HJ（origin = CA） |

**壊れているソルブの見分け方**: 辺のはずのペアが 1〜2m しかない、対角が 6.5m 付近に
2 本無い、高さが 1 台だけ外れる（2.51m / -0.58m など）。この場合は該当カメラの
サンプルを撮り直す。

## 既知の問題

- **レンダラー再生成後に点群の更新が止まる**（未修正）。
  `DestroyAllRenderers()` + `StartLive()` を通ると `PointCloudRenderer` のメッシュが
  最後のフレームで固まり、NaN/Inf を含む「バッファの残骸」が表示され続ける。
  カラー解像度の切替がこの経路を通るため、**床微調整では解像度を切り替えない**
  仕様にして回避している（キャリブ後にアプリを再起動してから床調整する運用）。
  ライブ/再生の切替も同じ経路なので、いずれ本修正が要る
- `BoardSample` 床アンカーは浅い角度に弱い（上記）
- 床微調整中は `KeepInside` で箱の外が見えないため、床より下の点は判断材料にならない
