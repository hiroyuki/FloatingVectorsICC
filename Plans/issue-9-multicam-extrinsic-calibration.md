# Plan: マルチカメラ extrinsic calibration (issue #9 / Phase E)

## ゴール

複数 Femto Bolt が同じワールド空間に揃うよう、各カメラの `global_tr_colorCamera`（4x4 transform）を求める Editor ツールを実装。
結果を `<root>/calibration/extrinsics.yaml` に書き出し、`PointCloudRecorder.Read` で読み戻して点群が世界座標で重なるようにする。

完了条件は GitHub issue #9 を参照。

## ブランチ

```bash
git checkout main
git checkout -b feature/issue-9-extrinsic-calibration
```

## 前提コード（最初に読むファイル）

| パス | 役割 |
|---|---|
| `Assets/Scripts/PointCloud/PointCloudRecording.cs` | `WriteExtrinsicsYaml` の scaffold あり、`DeviceCalibration` 構造もここ。`global_tr_colorCamera` フィールドが null = identity の placeholder |
| `Assets/Scripts/PointCloud/PointCloudRecorder.cs` | Save / Load の本体。Read 時に extrinsics.yaml を読み戻すコードを足す（現状は書き出すだけ） |
| `Assets/Scripts/PointCloud/PointCloudCameraManager.cs` | 複数 Renderer を spawn。calibration セッションのトリガはここに置くのが妥当 |
| `Assets/Scripts/PointCloud/PointCloudRenderer.cs` | `OnRawFramesReady` で color (RGB8) 取得可能。calibration はカラーフレームベース |
| `Assets/Scripts/Orbbec/OrbbecNative.cs` | `ObCameraParam`, `ObCameraIntrinsic`, `ObExtrinsic` 等 v2 構造定義 |
| `CLAUDE.md` | v2 直叩き経路、ヘッダ参照必須等の規約 |

## アーキテクチャ概要

```
[ChArUco / AprilTag ボードを物理空間に置く]
   ↓
各 PointCloudRenderer の color フレームから marker を検出
   ↓
各カメラ視点で marker pose (cam_tr_marker) を求める
   ↓
基準カメラ（index 0）を world とする
world_tr_camN = (cam0_tr_marker)^-1 * camN_tr_marker  ※基準は marker 経由
   ↓
extrinsics.yaml の各 device エントリに `global_tr_colorCamera` として書き込み
   ↓
Read 時に読み戻して、各 PointCloudRenderer の transform に反映
```

## 設計判断

### Marker フォーマット
- **ChArUco** (Charuco = Chessboard + ArUco) を推奨：チェッカーで sub-pixel 精度、ArUco で個別マーカー識別
- A4 〜 A2 印刷で十分（精度は紙のフラットさに依存するので硬い板に貼ると良い）
- 参考: `cv::aruco::CharucoBoard` の dictionary は `DICT_4X4_50` あたりで十分

### Marker 検出ライブラリ
3 つ選択肢：

1. **OpenCV for Unity** (Asset Store 有料): 最も簡単、Unity ネイティブ
2. **OpenCVSharp** (NuGet): Windows ネイティブ、P/Invoke 経由で Unity から使える
3. **自前 ArUco 実装**: 実装コスト大、推奨しない

最初は 2. OpenCVSharp が依存最小・ライセンス自由・Windows x64 で問題なし。

### 撮影タイミング
- ユーザーが「Capture」ボタンを押した瞬間に全カメラの最新カラーフレームをスナップ
- `PointCloudCameraManager` 経由で全 `Renderer.OnRawFramesReady` の最新カラーバッファを保持
- マーカーが全カメラで同時に見える必要があるので、撮影位置を動かしながら複数枚撮ってベストショットを採用する設計

### 結果のシリアライズ
既存の `extrinsics.yaml` フォーマットにそのまま乗せる：

```yaml
devices:
  - serial: CL8F253004Z
    color_intrinsic: { fx: ..., fy: ..., cx: ..., cy: ..., width: 1280, height: 720 }
    depth_intrinsic: { ... }
    color_distortion: { ... }
    depth_distortion: { ... }
    depth_to_color: { rotation: [...], translation: [...] }
    global_tr_color_camera:
      rotation: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
      translation: [tx, ty, tz]   # meters
```

`global_tr_color_camera` は新規追加。基準カメラは identity を入れる。

## 段階的実装

### Phase 1: OpenCVSharp 依存追加（半日）
1. NuGet `OpenCvSharp4.runtime.win` を取得（NuGetForUnity プラグイン経由 or 手動 DLL 配置）
2. 配置先: `Assets/Plugins/OpenCvSharp/` (Plugins folder で Unity が自動 import)
3. ビルドターゲット: Windows x64 onlyOK
4. 動作確認: ダミーで `Cv2.GetVersionString()` を呼んで Console に出力

### Phase 2: 1 カメラの marker pose 推定（1 日）
1. `Assets/Scripts/Calibration/CharucoBoardSpec.cs` に board 仕様（squares X/Y, square length, marker length, dictionary）を ScriptableObject で持つ
2. `Assets/Scripts/Calibration/MarkerPoseEstimator.cs`:
   - 入力: RGB8 byte[], width, height, intrinsic, distortion
   - OpenCvSharp で `Mat` を作って ArUco 検出 → `EstimatePoseCharucoBoard`
   - 出力: rvec, tvec の 4x4 transform（color camera frame → marker frame）
3. テスト: 1 台の Femto Bolt の color フレームを Editor 上で取得して board 検出できることを確認

### Phase 3: 複数カメラの pose を world 統合（1 日）
1. `Assets/Scripts/Calibration/MultiCameraCalibrator.cs`:
   - 各 Renderer から最新カラーフレームを取って MarkerPoseEstimator を回す
   - 全カメラで marker 検出に成功したサンプルを採用
   - 基準カメラ（serial 順 or Inspector で指定）の pose を world とする
   - 各 camN について `world_tr_camN = world_tr_marker * marker_tr_camN`（marker 経由のチェイン）
2. 複数サンプル取得で平均 / 外れ値除外したい場合は次フェーズで対応、まずは 1 サンプルで動くこと

### Phase 4: extrinsics.yaml への保存と読み込み（半日）
1. `PointCloudRecording.WriteExtrinsicsYaml` を改修して `global_tr_color_camera` フィールドを書く
2. `PointCloudRecording.ReadExtrinsicsYaml` を新設（YAML パーサが必要）。簡易: `YamlDotNet` を NuGet から、または手書きパーサ
3. `PointCloudRecorder.Load` で `extrinsics.yaml` をパースして各 track の `CameraParam` 関連フィールドを埋める

### Phase 5: ランタイム反映（半日）
1. `PointCloudCameraManager` に「extrinsics 適用モード」のトグル追加
2. ON のとき各 Renderer の transform を `global_tr_color_camera` で世界座標化
3. 点群が物理空間で重なるかを目視確認

### Phase 6: Editor UI（半日 〜 1 日）
1. `Assets/Scripts/Calibration/Editor/CalibrationWindow.cs` を新設
2. メニュー: `Window > Calibration > Multi-Camera Extrinsic`
3. UI:
   - Board spec 選択（ScriptableObject）
   - 「Capture」ボタン: 全カメラの最新フレームを取って pose 推定 → サンプル蓄積
   - サンプルリスト表示（カメラ N が見えた / 見えなかった）
   - 「Solve」ボタン: 蓄積サンプルから best pose を計算 → extrinsics.yaml に書き出し
   - 「Reset」ボタン: identity に戻す（書き出し済みも上書き）

## 既知の制約 / 注意

- **同時撮影**: 全カメラで同時にマーカーが見える必要がある。HW sync 構成（issue #6 で Phase A 完了）の下では同時取得しやすい
- **物理セットアップ**: マーカー板はフラット（曲がると歪む）、十分な照明、各カメラから 1〜3 m 程度の距離
- **対称性**: ChArUco は対称デザインだと姿勢推定が反転することがある。非対称 dictionary 推奨
- **CLAUDE.md ルール**:
  - v2 ヘッダ参照必須 — `C:\dev\OrbbecSDK_v2\include\libobsensor\h\` の `ObTypes.h` 等
  - K4A Wrapper 経路は使わない（calibration は color フレームベース、k4abt は無関係）
  - 動作検証は AI 自身が MCP + Editor.log で進める（CLAUDE.md L29 以降）
- **依存追加**: OpenCvSharp4 + OpenCvSharp4.runtime.win の DLL を `Assets/Plugins/` に配置する必要があるので、`.gitignore` のチェック / 配置場所を CLAUDE.md に追記する

## オープン課題（着手中に決める）

1. OpenCvSharp 配置: Asset Store / NuGetForUnity / 手動 DLL コピーのどれか
2. YAML パーサ: YamlDotNet（NuGet 依存追加）、自前トークナイザ、JSON 変換のいずれか
3. board 仕様の標準化: 7x10 squares / 25mm square length あたりが定番だが、印刷サイズに合わせる
4. キャリブ精度の評価指標: re-projection error を Editor UI に出すと quality check しやすい

## 完了の目安

issue #9 の Done 条件すべてチェック。具体的には：
- Editor から calibration 実行できる
- 結果が `extrinsics.yaml` に書き出される
- Read で読み戻されて点群がワールドで重なる（目視確認）

## 後続

`Plans/issue-10-k4abt-worker-process.md` (#10) と組み合わせて `issue-11` (skeleton merge) に進む。
issue #11 では各カメラ worker が出した skeleton を `global_tr_colorCamera` で世界座標へ変換 → joint 信頼度ベースで合成、という流れになる。
