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

### 展示構成: 同一セット×2（呼称「4070」「5080」）
実際の展示では **「PC 1台 + Femto Bolt 4台」の同一構成セットが 2 セット**存在する。
呼称は搭載 GPU から **「4070（セット）」「5080（セット）」**。コード・シーンは共通だが、
**カメラのシリアル番号はセットごとに異なる**。

**2026-07-22 にカメラ本体を 2 セット間で入れ替えた（テレコ）。以下は入れ替え後の対応。**
セットの呼称は GPU に固定で、カメラ側が動く — 過去の録画や calib backup を読むときは
その時点の対応を見ること（入れ替え前は下表の serial が逆だった）。

| セット | GPU | カメラ serial |
|---|---|---|
| 4070 | RTX 4070 | CL8F25300CA / CL8F25300HJ / CL8F25300C6 / CL8F25300F0 |
| 5080 | RTX 5080 | CL8F253004N / CL8F253004Z / CL8F253004L / CL8F25300EG |

- **両セットとも 2026-07-22 に実機確認・再キャリブ済み**。上表の serial 対応は
  両マシンで実機照合済み（4070 機の GPU は `Get-CimInstance Win32_VideoController` で
  RTX 4070、接続 serial は CA / HJ / C6 / F0 を確認）
  - 5080 セット: extrinsics / floor まで再キャリブ。カメラ位置は入れ替え前の calib と
    10 cm 以内で一致
  - 4070 セット: `cameras.yaml` / `extrinsics.yaml` / `floor.yaml` / `sensing_area.yaml` を
    新規作成。id 順は **0=CA / 1=C6 / 2=F0 / 3=HJ**（origin=CA）。実測 4.6 m 四辺に対して
    4.514〜4.624 m、対角 6.374 / 6.540 m、カメラ高さのばらつき 12 mm、床傾き 0.19°
- カメラを挿し替えたら **USB のリンク速度を必ず確認する**。`Log/OrbbecSDK.log.txt` の
  `DeviceManager.cpp` 行に `Connection: USB3.1` / `USB2.1` が出る。USB2 で繋がった台は
  depth+color の帯域が足りず「フレーム停止」で異常判定される（入れ替え直後に 2 台が
  USB2 になっていた）
- **キャリブ前に sync 配線と `SensorManager.applySyncConfig` を必ず確認する**。
  2026-07-22 の 4070 機で、sync ケーブル未接続 + シーンの `applySyncConfig=false`
  （配線が無い状態で Secondary にすると従属機が止まるため、誰かが落としていた）のまま
  キャリブし、台間 skew 40〜90 ms で solve が破綻した（四辺 3.4/3.5/4.6/6.4 m、
  カメラ高さが −0.35〜1.88 m に散る）。配線 + `applySyncConfig=true` で再キャリブして解決
  - 台間 skew は `PointCloudRenderer.LastTimestampUs` を 4 台で比較すれば即わかる。
    **sync が効いていればサブ ms**（4070 機の実測: Primary の CA が最速、残り 3 台が
    1〜2 ms 以内）。数十 ms 出ていたら配線か設定を疑う
  - **skew が丁度 33 ms（30fps の 1 フレーム）なら Play し直す**。台が 2 グループに
    分かれ、グループ内はサブ ms なのにグループ間だけ 1 フレームずれることがある
    （ストリーム開始時のトリガ捕捉タイミング）。2026-07-22 の 4070 機では
    Play し直しだけで 33.5 ms → 1.5 ms に解消した
  - **Primary は USB 列挙 index 0 の台**（`syncTopology` が SyncHubPro でも DaisyChain でも
    同じ。`SensorManager.cs:545`）。これは `cameras.yaml` の camera-id とは**別の順番**で、
    USB ポートを挿し替えると別の台に移る。**配線の起点（ハブ入力 / 鎖の先頭）を
    index 0 の台に合わせる**こと。index 0 は GO 名 `PointCloud[0] ...` で確認できる。
    4070 機は index 0 = CL8F25300CA（id 0 もたまたま CA だが決まり方は無関係）
  - `maxSkewMs` は 150 ms に緩めてある（commit 171dbe9 の暫定処置）ので、
    **粗悪なキャプチャが reject されずに通る**。skew の値はログで自分で見ること
  - solve 後の検証は距離で行う: 四辺が実測値どおり、対角 2 本が一致、4 台の高さが揃うこと。
    `extrinsics.yaml` の `global_tr_colorCamera.trans_m` がカメラ位置（OpenCV 系、
    Unity の y は符号反転）

- カメラ ID 割当（`cameras.yaml`）はマシンローカル（persistentDataPath 配下）。
  **セットをまたいで serial を持ち込まない** — 別セットの serial が id 0-3 を占有すると、
  接続中カメラが id 4+ に追いやられ、外部カラーディスプレイ（id 0-3 のみ表示）に映らなくなる
- 新しいセット/マシンで初回キャリブする際は、キャリブ UI の assign mode（I キー）で
  そのセットの 4 台に id 0-3 を割当て直して保存する

#### カメラ ID マッピングやり直し手順（各セット共通）
1. Play → キャリブモード ON（F1、または AI が `CalibrationRuntimeUI.SetActive(true)` を reflection で呼ぶ。
   シーンが playbackOnly の場合は先に `SensorManager.StartLive()` でライブ起動）
2. `cameras.yaml`（`persistentDataPath/Recordings/recording/calibration/`）に
   **別セットの serial が残っていたら削除**（`_camOrder` から非 present を除去して `SaveCameraMap`、
   または yaml を直接消してから Reconcile させる）。CLAUDE.md の serial 対応表で判別
3. Game ビューで **I** → assign mode。カメラの前で手を振って個体を特定し、
   矢印で選択 → 数字キーで id 割当 → **O** で origin 指定 → **Enter** で保存
4. 外部ディスプレイ（display2 に cam0/1、display3 に cam2/3）で並びを確認
- **git 管理下（シーン / asset / スクリプト既定値）に serial を書かない**。セット間で
  同期され、片方のマシンでは必ず別セットの serial になる。リグ順は
  `PointCloudRecording.ResolveRigSerialOrder` が `cameras.yaml`（マシンローカル）から
  解決する — `SensorManager` / `SensorRecorder` / `ExperienceSpaceBuilder` /
  `BtFrameInspectorWindow` はすべてこれを通る。
  各コンポーネントの `rigSerialOrder` は**空のまま**が正しい状態（cameras.yaml の id 順が
  perimeter walk になっていない場合に限り、そのマシンでだけ手で埋めるフォールバック。
  埋めたらコミットしない）
- serial を持つマシンローカルファイルは
  `persistentDataPath/Recordings/recording/calibration/` に集約: `cameras.yaml`（id 割当）、
  `extrinsics.yaml`（外部パラメータ）、`floor.yaml`（床高さ）。いずれも git 管理外

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
- 依存が欠けている場合はコンパイルエラーになるのでそれで判別する（専用チェックツールは 2026-07-23 に削除済み）

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
    `SensorRecorder.RecordBodies` で書き込む
  - **再生時の BT はプラットフォームで分岐**（`SkeletonMerger.ignoreRecordedBodies`, Windows 既定 true）:
    - **Windows**: 再生 depth を k4abt worker にリアルタイムで流して解析（ライブと同一パイプライン）。
      bodies_main は無視する
    - **Mac 等 BT SDK 無し環境**: `bodies_main` から `OnPlaybackBodies` event 経由で復元
      （フラグは inert、k4abt は走らない）
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
