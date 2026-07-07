# Femto Bolt Point Cloud Visualizer

## 共通ルール

[claude-basics/claude_basics.md](./claude-basics/claude_basics.md) を参照。

## プロジェクト概要
Orbbec Femto Bolt（RGB-Dカメラ）から取得した深度+カラーデータを
Unity上でリアルタイムにポイントクラウドとして描画する。
将来的にはオプティカルフロー+深度からリボン状の3Dオブジェクトを
生成するインタラクティブインスタレーションに発展させる。

### 想定ユースケース: 1人のみ
**この作品/装置は 1 人を対象としたインスタレーション**。同時に 2 人以上を相手にするユースケースは想定外。

- ボディトラッキング系（issue #7 / #10 / #11）は **常に「単一人物」前提**で設計する
- 複数 worker からの skeleton merge も **同一人物の多視点統合**であって、複数人物の同時 ID 管理は対象外
- 2 人以上が検出された場合の振る舞い: **画面に Alert を出す**（来場者への運用フィードバック）。人数カウント機能自体は Alert 用途のため "nice to have" として実装する
- 入れ替わり耐性 / Hungarian-style 多人数追跡は実装しない

## 技術スタック
- Unity 6 LTS (URP - Universal 3D)
- OrbbecSDK v2（C/C++ネイティブSDK、直接P/Invoke経由で利用）
- 点群・カメラ I/O は K4A Wrapper を使わず v2 API を直接叩く
- ボディトラッキングのみ例外: Microsoft Azure Kinect Body Tracking SDK 1.1.2
  + Orbbec K4A Wrapper を併用（Femto Bolt で BT を動かす公式経路はこれしか無いため）
- 言語: C#
- 対象プラットフォーム: Windows x64

## ファイル構成
- OrbbecSDK v2ヘッダ: C:\dev\OrbbecSDK_v2\include\libobsensor\h\
- OrbbecSDK v2 Cサンプル: C:\dev\OrbbecSDK_v2\examples\c\
- DLL配置済み: Assets\Plugins\OrbbecSDK\
- ラッパーC#: Assets\Scripts\Orbbec\
- 描画スクリプト: Assets\Scripts\PointCloud\
- キャリブスクリプト: Assets\Scripts\Calibration\

### キャリブ用依存
- **OpenCV for Unity** (Enox Software, Asset Store): `Assets\OpenCVForUnity\` に展開 (ユーザー手動インポート)
  - 用途: マルチカメラ extrinsic calibration (issue #9)。ChArUco 検出 (`Aruco.detectMarkers` / `Aruco.interpolateCornersCharuco` / `Aruco.estimatePoseCharucoBoard`) と solvePnP
  - リポジトリには含めない (Asset Store ライセンス) — `.gitignore` で除外
- **YamlDotNet** 16.3.0: `Assets\Plugins\YamlDotNet\YamlDotNet.dll` (managed, netstandard2.1)
  - 取得元 / SHA-256 は `Assets\Plugins\YamlDotNet\CHECKSUMS.txt`
  - 用途: extrinsics.yaml の安全パース (`YamlStream` ノード走査、型自動 deserialization 禁止)
- 動作確認: Unity Editor メニュー `Window > Calibration > Check Dependencies` で OpenCV for Unity と YamlDotNet ロードを確認

### 外部 SDK（リポジトリには含めない）
- Orbbec K4A Wrapper:
  D:\OrbbecSDK_K4A_Wrapper_v2.0.11_windows_202510221441\
  - bin\: k4a.dll, k4arecord.dll, OrbbecSDK.dll, depthengine_2_0.dll
  - include\: k4a/, k4arecord/ ヘッダ
- Microsoft Azure Kinect Body Tracking SDK 1.1.2:
  C:\Program Files\Azure Kinect Body Tracking SDK\
  - sdk\windows-desktop\amd64\release\bin\: k4abt.dll, onnxruntime.dll, DirectML.dll,
    onnxruntime_providers_*.dll, dnn_model_2_0_*.onnx
  - sdk\include\: k4abt.h, k4abttypes.h
- UVCメタデータ登録スクリプト（Femto Bolt 接続初回に管理者で実行済み）:
  D:\OrbbecSDK_K4A_Wrapper_v2.0.11_windows_202510221441\scripts\obsensor_metadata_win10.ps1

## プロジェクト固有ルール

- AIは対応完了後 main にマージする前に、Unity Editor のコンソールでコンパイルエラーがないことを必ず確認する（Unity MCP の Unity_GetConsoleLogs などで取得）。コンパイルエラーが残っている場合はマージせず、原因を特定して修正する。

### 動作検証はAI自身がMCP経由で進める

ユーザーに「Playしてログを貼ってください」と都度依頼しない。Unity の動作検証・ログ確認は AI 側で完結させる：

- **Play / Stop**: `mcp__unity-mcp__Unity_RunCommand` で `EditorApplication.EnterPlaymode()` / `ExitPlaymode()` を呼ぶ
- **コンパイル**: `UnityEditor.Compilation.CompilationPipeline.RequestScriptCompilation()` を呼んで `AssetDatabase.Refresh(ImportAssetOptions.ForceUpdate)` で待つ。Play 中は再コンパイルしないので、スクリプト変更後は必ず一旦 Stop する
- **エラー / 警告**: `mcp__unity-mcp__Unity_GetConsoleLogs` で取得
- **ランタイムログ全体**: `C:\Users\hori\AppData\Local\Unity\Editor\Editor.log` を Bash で `tail` / `grep` する。MCP の Console は前のセッション分が混じることがあるので、行番号や時刻と照らし合わせて新しいエントリを判別する
- **シーン状態**: `mcp__unity-mcp__Unity_RunCommand` で `Object.FindObjectsByType<>` 経由で実行時状態を読む（Inspector の値、`MeshFilter.sharedMesh.vertexCount`、`MeshRenderer.sharedMaterial.shader.name` など）
- **シーン編集**: 同 RunCommand で `EditorUtility.SetDirty()` + `EditorSceneManager.SaveOpenScenes()` で永続化する。エディタ操作の代行は Inspector 値変更などに限り、ヒエラルキー大改修は事前にユーザー確認

ユーザーへの報告は「自分で確認した結果」を提示する形にする。コピペでログを貼ってもらうのは最終手段。

カメラデバイスが応答しなくなった等の物理層の問題（USB再接続など）はユーザーにしか対処できないので、その場合のみ依頼する。

#### Editor が非フォーカスだと play loop が止まる

Editor ウィンドウがフォーカス外（ユーザーが Claude チャットを操作している間など）だと、デフォルトで `Application.runInBackground = false` のため Play 中でも Update が呼ばれず `Time.frameCount` も進まない。`EditorApplication.isPlaying=True` のまま「Play してるのに動いてない」状態になる。

対処: 動作確認系の RunCommand スクリプトの先頭で必ず `Application.runInBackground = true;` を実行する。これだけで Editor が裏でも tick する。

#### Playback ベースのボディトラッキング（SkeletonMerger）

ライブの Femto Bolt 接続が無い／録画データを使う場合、シーン構成は以下：

- `SensorRecorder` (例: `CameraRecorder`) が RCSV を読み込む
- Inspector の **Play ボタン**で playback を開始すると `OnPlaybackRawFrame` イベントが発火
- `SkeletonMerger`（旧称 BodyTrackingMultiLive）が `K4abtWorkerHost` 経由で k4abt worker を spawn し、playback フレームを worker に流す
- ライブ時の `PointCloudRenderer` の代わりに `_Playback_<serial>` GO が source transform になる

ハマりどころ:

- **Recorder.Play 状態は EnterPlaymode / ExitPlaymode で消える** — Stop してから Play し直すと、再生は自動再開しない。Recorder Inspector の Play ボタンを再度押す必要がある。Stop/Recompile/Play サイクルを繰り返すと毎回 user に再生再押下を依頼するハメになるので、極力 Play を保ったまま検証する
- 既存の playback セッションが流れ終わると `_latestBySerial.Count` が 0 になり worker が消える。snapsRecv は累積カウンタなので増え続けるが、現在の worker 数は別途確認すること
- `SkeletonMerger.enabled` が false になると `OnDisable` で `_latestBySerial.Clear()` + `workerHost.StopWorker(...)` が走り、playback を再開するだけでは復活しない（component を enable し直す必要がある）

### P/Invokeラッパー（Assets\Scripts\Orbbec\）
- OrbbecNative.cs: 全P/Invoke宣言をまとめる
  - DllImportの対象は "OrbbecSDK"（拡張子なし）
  - CallingConvention は Cdecl
- OrbbecContext.cs: ob_context のIDisposableラッパー
- OrbbecDevice.cs: ob_device のIDisposableラッパー
- OrbbecPipeline.cs: ob_pipeline のIDisposableラッパー
- OrbbecFrame.cs: ob_frame のIDisposableラッパー
- OrbbecFilter.cs: ob_filter のIDisposableラッパー（ポイントクラウド変換用）
- OrbbecException.cs: エラーハンドリング用例外クラス

### 描画（Assets\Scripts\PointCloud\）
- PointCloudRenderer.cs: MonoBehaviour
  - Start: デバイスオープン、パイプライン開始
  - Update: フレーム取得、ポイントクラウドフィルタで3D点群変換、Mesh更新
  - OnDestroy: リソース解放
- BoundingVolume.cs（旧称 PointCloudBoundingBox）: MonoBehaviour
  - Transform（位置・回転・localScale）で OBB を定義し、KeepInside/KeepOutside で点群をフィルタ
  - showVisualization で Scene/Game 両方にワイヤーキューブを表示（Gizmo + runtime mesh）
- PointCloudDecimater.cs: MonoBehaviour
  - reductionPercent (0-100%) で毎フレーム各点を独立にランダムドロップ（Bernoulliサンプリング）
- PointCloudCumulative.cs: MonoBehaviour
  - noErase が ON の間、interval フレームごとに現在の点群をスナップショット Mesh として子 GO に保存・蓄積
  - Editor/PointCloudCumulativeEditor.cs が Inspector に Clear ボタンを追加（全スナップショットを破棄）
- SensorRecorder.cs（旧称 PointCloudRecorder）: MonoBehaviour
  - Rec / Play / Save / Read の 4 操作で raw センサーデータの収録・再生・保存・読み込みを行う
  - 収録は PointCloudRenderer.OnRawFramesReady イベント経由で depth / color / IR をメモリに蓄積
  - 保存形式は scanned-reality.com の RCSV 互換、デバイス毎に
    `<root>/dataset/<host>/FemtoBolt_<serial>/{depth_main,color_main,ir_main,bodies_main}` を書き出す
  - **BT 結果も bodies_main として保存**: K4abtWorkerHost.OnSkeletonsReady 経由で
    SkeletonMerger が `RecordedBodySerializer.Encode` で encode し、
    `SensorRecorder.RecordBodies` で書き込む。**再生時は k4abt を一切走らせない**
    （`bodies_main` から `OnPlaybackBodies` event 経由で復元 → Mac でも playback 可能）
  - 旧仕様 (depth + IR から k4abt 再計算) は廃止。`ir_main` は今もデバッグ用に保存しているが
    再生時は使わない
  - 再生はデバイスごとに子 GO を生成し、タイムスタンプに合わせて Mesh を更新
  - Editor/SensorRecorderEditor.cs が Inspector に Rec / Play / Save / Read ボタンと状態表示を追加
- PointCloudRecording.cs: static ユーティリティ
  - RCSV (variable-size records) 形式の書き込み / 読み込みと、デバイス毎ディレクトリ構成の組み立て
  - センサー名: `DepthSensorName`=depth_main, `ColorSensorName`=color_main, `IRSensorName`=ir_main,
    `BodiesSensorName`=bodies_main
- BodyTracking/RecordedBodySerializer.cs: static
  - `BodySnapshot[]` ↔ bodies_main payload バイト列の encode / decode。
    レイアウトは `K4abtWorkerSharedLayout` の per-body 領域と同じ (`u32 bodyCount` + bodies)

## コーディング規約
- ネイティブリソースは必ずIDisposableパターン + ファイナライザで管理
- P/Invoke宣言は推測で書かず、必ず以下のヘッダを参照して書く:
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\ObTypes.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Context.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Device.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Pipeline.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Frame.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\Filter.h
  - C:\dev\OrbbecSDK_v2\include\libobsensor\h\StreamProfile.h
- C/C++サンプルの参考実装:
  - C:\dev\OrbbecSDK_v2\examples\c\point_cloud\point_cloud.c
  - 上記サンプルと同等の処理をUnity C#で再現する
- 関数名や型名を「思い出し」で書かない。ヘッダで確定してから書く
- OrbbecSDK v1のAPIとv2は名前が違う関数がある。v2のヘッダのみを参照
- string marshalingはUTF-8（UnmanagedType.LPUTF8Str）
- 座標系: OrbbecSDKはmm単位。Unity側でm単位（1/1000スケール）に変換
- 深度+カラーの両方のストリームを有効にする
- D2C（Depth to Color）アライメントを有効にして、カラーが点群に正しくマッピングされるようにする

## 禁止事項
- OrbbecSDK v1のAPIを使わない（v2専用）
- 点群・カメラ I/O では Azure Kinect SDK / K4A API に依存しない（v2直接経路）
  - 例外: ボディトラッキング機能（k4abt）は K4A 経路を許可
- ヘッダを読まずにAPIシグネチャを推測しない
  - BT 用 P/Invoke も同様に `k4a.h` / `k4abt.h` のヘッダ参照を必須とする
