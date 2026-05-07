# Plan: k4abt worker process (issue #10)

## ゴール

カメラごとに独立した k4abt 用 worker .exe を立て、Unity 本体は IPC（共有メモリ + イベント）経由で depth+IR を送り skeleton を受け取る構造を作る。
これにより k4abt の「1 プロセス 1 tracker」制約を回避し、複数カメラ並列 BT の前段階を整える。

完了条件は GitHub issue #10 を参照。

## ブランチ

```bash
git checkout main
git checkout -b feature/issue-10-k4abt-worker
```

## 前提コード（最初に読むファイル）

| パス | 役割 | 用途 |
|---|---|---|
| `Assets/Scripts/BodyTracking/K4ANative.cs` | k4a の P/Invoke surface | worker でも同じものを使う（リンク共有） |
| `Assets/Scripts/BodyTracking/K4ABTNative.cs` | k4abt の P/Invoke surface | 同上 |
| `Assets/Scripts/BodyTracking/K4ACaptureBridge.cs` | depth+IR から `k4a_capture_t` を合成 | worker 内で使い回す |
| `Assets/Scripts/BodyTracking/K4ACalibration.cs` | `ObCameraParam` から `k4a_calibration_t` を構築 | worker 起動時に read する |
| `Assets/Scripts/BodyTracking/BodyTrackingLive.cs` | 現状の in-process Live BT | 切替先の挙動リファレンス |
| `Assets/Scripts/BodyTracking/BodyTrackingBootstrap.cs` | DLL 検索パス設定 | worker でも必要（再利用 or 同等処理） |
| `Assets/Scripts/PointCloud/PointCloudRenderer.cs` | `OnRawFramesReady` を発火 | Unity 側ホストはここから depth+IR を取る |
| `CLAUDE.md` | プロジェクト規約 | 特に「ヘッダ参照必須」「v2 直叩き経路」のルール |

## アーキテクチャ概要

```
Unity (本体プロセス)
  PointCloudRenderer ─OnRawFramesReady─→ K4abtWorkerHost
                                              │
                                              ├─ Worker A: shared memory (depth+IR / skeleton)
                                              │             ↑↓ (Win32 Event signaled)
                                              │           [k4abt_worker.exe --serial=A ...]
                                              │
                                              └─ Worker B: ...

各 worker はループで:
  WaitForSingleObject(input_event)
  → MMF から depth+IR 読む
  → k4abt_tracker_enqueue_capture / pop_result
  → MMF へ skeleton 書き込む
  → SetEvent(output_event)
```

## MMF レイアウト（カメラごとに 1 個、固定サイズ）

```
offset  size      field
0       8         frame_id (u64)
8       8         ts_ns (u64)
16      4         depth_w (u32) — typically 640
20      4         depth_h (u32) — typically 576
24      4         ir_w (u32)
28      4         ir_h (u32)
32      4         flags (u32; bit0 = depth_valid, bit1 = ir_valid)
36      28        reserved (zero)
64      720*1024  depth Y16 (640*576*2 = ~720 KB ぴったり、計算は実コードで合わせる)
~       720*1024  ir Y16
~       4096      skeleton output:
                    body_count (u32)
                    bodies[body_count] {
                      id (u32),
                      reserved (u32),
                      joints[32] {
                        pos (float3 = 12 B),
                        orientation (quat float4 = 16 B),
                        confidence (u32),
                      }
                    }
```

総サイズ: `64 + depth_size + ir_size + 4096`。Unity と worker で同じレイアウト定数を共有する必要があるので、`Assets/Scripts/BodyTracking/Shared/K4abtWorkerSharedLayout.cs` のような形で 1 箇所に切り出す。

## 同期

`EventWaitHandle` を 2 つ作る：
- `<mmf_name>_input` — Unity が SetEvent、worker が WaitOne
- `<mmf_name>_output` — worker が SetEvent、Unity が WaitOne (timeout 0 で poll)

`EventResetMode.AutoReset` でシグナル後に自動リセット。

## 段階的実装

### Phase 1: 共有コード切り出し（1〜2 時間）
1. `Assets/Scripts/BodyTracking/Shared/` を作る（meta も）
2. MMF レイアウト定数 `K4abtWorkerSharedLayout.cs` を切り出し（offset / size / SkeletonOutput 構造）
3. `K4ANative.cs` / `K4ABTNative.cs` / `K4ACaptureBridge.cs` / `K4ACalibration.cs` がそのまま再利用可能か確認（Unity API への依存があれば剥がす — `UnityEngine.Debug.Log` を `Console.WriteLine` 等に置換する条件コンパイル）

### Phase 2: worker .csproj セットアップ（半日）
1. `Workers/K4abtWorker/K4abtWorker.csproj` を作る（リポジトリ直下、Unity の Assets/ 外）
2. .NET 8 console app、`<PublishSingleFile>true</PublishSingleFile>`
3. `Assets/Scripts/BodyTracking/Shared/*.cs` と `K4ANative.cs` 等を `<Compile Include>` で相対参照
4. ビルド出力先を `Workers/K4abtWorker/bin/Release/net8.0-windows/win-x64/publish/k4abt_worker.exe` に固定
5. `dotnet publish -c Release -r win-x64` で single-file exe が出ることを確認

### Phase 3: worker main loop（1 日）
1. 引数パース: `--serial`, `--mmf`, `--calib`
2. `MemoryMappedFile.OpenExisting(mmfName)` + 2 つの `EventWaitHandle.OpenExisting`
3. calibration バイナリ読み出し（1032 B を `Marshal.AllocHGlobal` + `Marshal.Copy`）
4. `K4ABTNative.k4abt_tracker_create`
5. ループ:
   - `inputEvt.WaitOne(1000)` で 1 秒タイムアウト付き wait（shutdown チェックを挟む）
   - MMF view から depth + IR + ts を読む
   - `K4ACaptureBridge.CreateCaptureFromDepthAndIR` で capture 合成
   - `k4abt_tracker_enqueue_capture` (timeout 200ms) → `pop_result` (timeout 500ms)
   - body_frame から skeleton 取り出し → MMF の output 領域に書き込み
   - `outputEvt.Set()`
6. shutdown シグナル（`<mmf_name>_shutdown` event を別途用意）でループ離脱
7. tracker / calibration / MMF 解放

DLL 検索パスは `BodyTrackingBootstrap.Initialize` 同等の処理を最初に流す（K4A Wrapper bin と BT SDK bin を PATH に prepend）。

### Phase 4: Unity 側 WorkerHost（1 日）
1. `Assets/Scripts/BodyTracking/K4abtWorkerHost.cs` (MonoBehaviour) を新設
2. API 案:
   ```csharp
   public class K4abtWorkerHost : MonoBehaviour {
       public string workerExePath = "Workers/K4abtWorker/.../k4abt_worker.exe"; // path
       public bool useWorker = false; // toggle
       public event Action<string /*serial*/, BodySnapshot[]> OnSkeletonsReady;
       public void StartWorker(string serial, ObCameraParam calib);
       public void EnqueueFrame(string serial, byte[] depth, byte[] ir, int dW, int dH, int irW, int irH, ulong tsNs);
       public void StopWorker(string serial);
   }
   ```
3. `MemoryMappedFile.CreateNew` + `EventWaitHandle(false, AutoReset, name)` を Unity 側で先に作ってから `Process.Start` で worker を spawn（worker は `OpenExisting` 側）
4. calibration を一時バイナリに書き出して worker に渡す（`Marshal.StructureToPtr` ではなく `K4ACalibration.Build` の出力 1032B を `File.WriteAllBytes`）
5. Update で各 worker の output event を `WaitOne(0)` で polling、新規 skeleton があれば parse して event 発火
6. OnDestroy / OnApplicationQuit で shutdown event → worker 終了確認 → `Process.Kill` フォールバック

### Phase 5: BodyTrackingLive 切替パス（半日）
1. `BodyTrackingLive` に `bool useWorker` Inspector フィールド追加
2. `useWorker == true` の時は in-process tracker create を skip し、`K4abtWorkerHost.StartWorker(serial, cameraParam)` を呼ぶ
3. `OnRawFramesReady` のハンドラで `WorkerHost.EnqueueFrame` を呼ぶ
4. `WorkerHost.OnSkeletonsReady` を購読して body visual を更新（既存の BodyVisual ロジックを流用）
5. shutdown 時に StopWorker

### Phase 6: 動作確認（1 日）
1. 既存 in-process パスと worker パスを Inspector トグルで切り替えて、両方で 1 カメラ Live BT が出ることを確認
2. 数値で比較: BT 推論 fps、skeleton position の差、Unity main thread の負荷
3. クラッシュ時 (`Process.Kill` をテスト)、Unity 側で再起動できることを確認
4. Editor 終了時に worker プロセスが残らないことを確認（タスクマネージャーで確認 or `Get-Process k4abt_worker`）

## 既知の制約 / 注意

- **GPU メモリ**: worker ごとに ONNX runtime セッションが立つ。lite モデルで ~150MB / process。3 worker で ~500MB。GPU 容量を超えないこと
- **DirectML マルチプロセス**: 同じアダプタを複数プロセスで使うのは可能だが、コンテキスト切替コストが乗る。最初は 1 worker で確認、複数化は別 issue
- **k4abt model load**: tracker_create は ~5〜10 秒かかる。worker 起動を Unity Play 開始時に行うと初回 BT 反応が遅延する。バックグラウンドで spawn して ready を待つ設計
- **MMF サイズ固定**: depth/IR の解像度を変えたら MMF を作り直す必要あり。Live で resolution が変わらない前提
- **CLAUDE.md ルール**:
  - ヘッダ参照必須 — `D:\OrbbecSDK_K4A_Wrapper_v2.0.11_windows_202510221441\include\k4a\` と `C:\Program Files\Azure Kinect Body Tracking SDK\sdk\include\` を見ながら書く
  - 推測で書かない
  - 動作確認はAI 自身が MCP + Editor.log で進める（CLAUDE.md L29 以降）
- **動作検証フロー**: BodyTracking 既存の `BodyTrackingLive.diagnosticLogging` のような per-second 診断ログを worker にも入れる（worker stdout でも、Unity 側で受信して Debug.Log でも）

## オープン課題（着手中に決める）

1. worker exe のビルドを Unity Editor 内ボタンから自動実行できるか（`dotnet publish` を `EditorBuildPostprocessor` から呼ぶ）
2. calibration を file ではなく MMF header に乗せる方が起動高速だが、最初は file で素直に
3. MMF の `_input` シグナル粒度: フレームごとに毎回 set or 64ms ごとに最新フレーム
4. Worker stdout/stderr を Unity の Console に流したい（パイプリダイレクト + Debug.Log にバウンス）

## 完了の目安

issue #10 の Done 条件すべてチェック。具体的には：
- 1 カメラで worker 経由 Live BT が動く
- Inspector トグルで in-process と worker パスを切り替えられる
- worker のクラッシュからの復旧動作確認済み
- Editor 終了で worker も終了することを確認済み

## 後続

`Plans/issue-9-multicam-extrinsic-calibration.md` (#9) と組み合わせて `issue-11` (skeleton merge) に進む。
