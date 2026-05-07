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

## スコープ（初版）

- **1 Unity プロセス × 1 Femto Bolt × 1 worker.exe** に限定する。複数カメラ並列は別 issue。
- 「latest-wins」backpressure。Unity 側で frame をドロップし最新だけを worker に渡す。worker が遅れた古いフレームは捨てる。
- 解像度は Live 中固定（NFOV 320×288 / 640×576 などモード変更は worker 再起動で対応）。

## アーキテクチャ概要

```
Unity (本体プロセス)                                Worker プロセス
  PointCloudRenderer                                k4abt_worker.exe
        │                                           --session=<GUID>
   OnRawFramesReady                                 --parent-pid=<PID>
        ▼                                           --serial=<S>
  K4abtWorkerHost ──────────── shared memory ──────►│
        ▲                      (depth+IR / skel)    │
        │   ┌─ ready_event ◄──────────────────────  │  (model load 完了)
        │   ├─ input_event  ──────────────────────► │
        │   ├─ output_event ◄──────────────────────  │
        │   └─ shutdown_event ────────────────────► │
        │                                           ▼
        │   (parent PID watchdog: 1Hz で OpenProcess。失敗で worker 自殺)
```

`K4abtWorkerHost` がカメラごとに 1 worker を spawn する設計に拡張可能だが、初版は 1 カメラ限定。

## IPC オブジェクトの命名 / セキュリティ

すべての名前付きオブジェクト（MMF / EventWaitHandle）は session-scoped にして、他 Unity プロセス・stale worker・他ユーザープロセスとの衝突を避ける：

```
prefix      = $"FB_K4ABT_{sessionGuid:N}_{serial}"
mmfName     = $"Local\\{prefix}_mmf"
inputEvt    = $"Local\\{prefix}_in"
outputEvt   = $"Local\\{prefix}_out"
readyEvt    = $"Local\\{prefix}_ready"
shutdownEvt = $"Local\\{prefix}_shutdown"
```

- `Local\` 名前空間で同一 logon session に閉じる（他ユーザーから見えない）
- `sessionGuid` は Unity 起動ごとに新規生成（`Guid.NewGuid()`）。worker exe には `--session` で渡す
- `MemoryMappedFileSecurity` / `EventWaitHandleSecurity` で `WellKnownSidType.WorldSid` を deny、現在ユーザーに full control のみ許可
- Stale worker / handle の自動掃除は v1 では non-goal（Phase 4 の "Stale worker 掃除" 節を参照。代わりに parent PID watchdog で worker 側自殺を担保）

## MMF レイアウト（per camera, double-buffer + seq-lock）

入力（depth+IR）と出力（skeleton）はそれぞれ **2 スロット**を持つリングにし、per-slot に `seq` を置く（**seq-lock**: 書き込み中は奇数、コミット時に偶数。reader は前後で seq を読み一致+偶数なら採用、不一致なら別スロット or skip）。

```
[GLOBAL HEADER]   offset  size  field
                  0       4     magic ('K4BT' = 0x54 0x42 0x34 0x4B)
                  4       4     layout_version (u32, = 1)
                  8       4     depth_w (u32)
                  12      4     depth_h (u32)
                  16      4     ir_w (u32)
                  20      4     ir_h (u32)
                  24      4     depth_bpp (u32, =2)
                  28      4     ir_bpp (u32, =2)
                  32      4     input_slot_count (u32, =2)
                  36      4     output_slot_count (u32, =2)
                  40      8     produced (u64, host が atomic increment)
                  48      8     consumed (u64, worker が atomic increment)
                  56      8     dropped (u64, host が enqueue 失敗時 inc)
                  64    1032    calibration blob (k4a_calibration_t を Marshal — 実サイズはヘッダで確定後、定数化)
                  ~      ...    align to 256

[INPUT SLOT 0..1] (per slot, slot_size = ceil((depth+ir+meta) / 256) * 256)
                  +0      8     seq (u64, 奇数=writing / 偶数=committed)
                  +8      8     frame_id (u64)
                  +16     8     ts_ns (u64)
                  +24     4     flags (u32; bit0=depth_valid, bit1=ir_valid)
                  +28     4     reserved
                  +32     N     depth Y16 (depth_w * depth_h * 2)
                  +32+N   M     ir Y16 (ir_w * ir_h * 2)

[OUTPUT SLOT 0..1] (per slot)
                  +0      8     seq (u64)
                  +8      8     frame_id (u64)  -- 入力 frame_id とリンク
                  +16     8     ts_ns (u64)
                  +24     4     body_count (u32)
                  +28     4     reserved
                  +32   ~6 KB   bodies[K4ABT_MAX_BODIES=6] {
                                  id (u32), reserved (u32),
                                  joints[32] { pos float3, quat float4, confidence u32 }
                                } -- 1 body = 8 + 32 * (12+16+4) = 1032 B
```

**サイズ計算**: 全部実コードで `width * height * bpp` から計算する。`720 * 1024` のような magic number は使わない。共有定数は `K4abtWorkerSharedLayout.cs` に集約。

## 同期 / コミットプロトコル

`EventWaitHandle`（AutoReset）4 つ:
- `input_event` — host が新スロットを書き終わって `Set`、worker が `WaitOne` で起床
- `output_event` — worker が新 skeleton を書き終わって `Set`、host が `Update()` で `WaitOne(0)` poll
- `ready_event` — worker が `k4abt_tracker_create` 完了後 1 度だけ `Set`。host は ready 前に `EnqueueFrame` を呼ばない
- `shutdown_event` — host から worker に終了要求

**書き込み手順（host / worker 共通）**

1. 相手が読んでいない方のスロット index `s` を選ぶ（`(latest_committed_slot + 1) & 1`）
2. `Volatile.Write(ref slot[s].seq, oldSeq | 1)`（奇数化 = writing 開始）
3. payload を書く（depth/IR or skeleton）
4. `Volatile.Write(ref slot[s].seq, oldSeq + 2)`（偶数 = committed、奇数→偶数で +1 進む）
5. global header の `latest_committed_slot` (u32) に `s` を書く
6. event を `Set`

**読み取り手順**

1. event を `WaitOne`
2. `latest_committed_slot` から slot index を取る
3. `seq_before = slot[s].seq`（偶数を期待。奇数なら writer が書いてる最中 → もう一方を試す or 1ms 待ち再読）
4. payload をコピー
5. `seq_after = slot[s].seq`
6. `seq_before == seq_after && (seq_before & 1) == 0` なら採用、違えば破棄

**latest-wins**: host は `produced++` し続け、worker が読まないスロットは上書きされる。worker 側で `frame_id` の連続性が崩れたら `dropped` に counted（diagnostic 用）。

`output_event` は hint。host は frame_id で重複・古さを判定する（同じ frame_id を 2 度処理しない、新しい frame_id しか採らない）。

## 段階的実装

### Phase 1: 共有コード切り出し（1〜2 時間）
1. `Assets/Scripts/BodyTracking/Shared/` を作る（meta も）。**ここに置く .cs は Unity 依存禁止**（`UnityEngine.*`, `AOT.*` 不可）。Unity 側ロギングは `Shared/IWorkerLogger.cs` のような interface 経由にし、Unity 側だけ `UnityEngine.Debug.Log` 実装をバインドする
2. MMF レイアウト定数 `K4abtWorkerSharedLayout.cs` を切り出し:
   - magic / layout_version / 計算ヘルパー（`InputSlotSize(dW,dH,irW,irH)`, `OutputSlotSize()`, `TotalMmfSize()`）
   - seq-lock 読み書きヘルパー（`Volatile.Write`/`Volatile.Read` ベース）
   - calibration blob 長は `K4ACalibration` 既存実装の native layout 契約に合わせて固定値 `CalibrationBlobBytes = 1032`（`Assets/Scripts/BodyTracking/K4ACalibration.cs` のオフセット直書きと同じ source of truth）。**managed `k4a_calibration_t` の定義は禁止**（既存ポリシーに合わせる）
3. `K4ANative.cs` / `K4ABTNative.cs` / `K4ACaptureBridge.cs` / `K4ACalibration.cs` を「Unity 依存剥がし版」として `Shared/` に移すか、`#if UNITY_2017_1_OR_NEWER` で Unity 側ロガーを切替えて両ビルドで使い回せる形にする
4. CLAUDE.md ルールチェック: ヘッダ参照（k4a.h / k4abt.h / k4abttypes.h）必須。推測でシグネチャを書かない

### Phase 2: worker .csproj セットアップ（半日）
1. `Workers/K4abtWorker/K4abtWorker.csproj` を作る（リポジトリ直下、Unity の Assets/ 外）
2. .NET 8 console app、`<PublishSingleFile>true</PublishSingleFile>`
3. `Assets/Scripts/BodyTracking/Shared/*.cs` と `K4ANative.cs` 等を `<Compile Include>` で相対参照
4. ビルド出力先を `Workers/K4abtWorker/bin/Release/net8.0-windows/win-x64/publish/k4abt_worker.exe` に固定
5. `dotnet publish -c Release -r win-x64` で single-file exe が出ることを確認

### Phase 3: worker main loop（1 日）

**引数（全部 required）**
- `--session=<GUID>` （IPC 名 prefix 用）
- `--serial=<S>` （Femto Bolt serial）
- `--parent-pid=<PID>` （Unity の PID）
- `--depth-w/h`, `--ir-w/h` （MMF サイズ確定）
- 任意: `--log-file=<path>`（worker stdout を直接ファイルに）

**起動フロー**
1. DLL 検索パス設定（`BodyTrackingBootstrap.Initialize` 相当を `Shared/WorkerBootstrap.cs` に切り出して呼ぶ。K4A Wrapper bin と BT SDK bin を `SetDllDirectory` / PATH prepend）
2. `MemoryMappedFile.OpenExisting(mmfName)` + 4 event を `OpenExisting`
3. global header の magic / layout_version / 解像度を verify（不一致なら exit 2）
4. calibration を **MMF header の calibration blob (raw 1032 B)** から `MemoryMappedViewAccessor.ReadArray<byte>` で取り出し、`Marshal.AllocHGlobal(1032)` した unmanaged buffer に `Marshal.Copy` する。**managed struct 化はしない**（既存 `K4ACalibration.cs` がオフセット直書きで運用しているのと同じポリシー）。`IntPtr` をそのまま `k4abt_tracker_create` に渡す
5. `K4ABTNative.k4abt_tracker_create`
6. **`readyEvt.Set()` で host に ready を通知**
7. parent watchdog スレッド起動: 1Hz で `Process.GetProcessById(parentPid)` チェック（失敗 → shutdown_event 自己 Set）

**メインループ**

```
while (true) {
  int idx = WaitHandle.WaitAny(new[]{ shutdownEvt, inputEvt }, 1000);
  if (idx == 0 || shutdownRequested) break;
  if (idx == WaitHandle.WaitTimeout) continue;

  // 1) input slot を seq-lock で安全に読む（latest_committed_slot から）
  if (!TryReadInputSlot(out var slot)) { continue; } // torn or stale → skip
  if (slot.frame_id <= lastSeenFrameId) continue;    // 古いフレームは捨てる
  lastSeenFrameId = slot.frame_id;

  // 2) k4a capture 合成
  using var capture = K4ACaptureBridge.CreateCaptureFromDepthAndIR(...);

  // 3) BT 推論
  // NOTE: k4abt API takes timeout_in_ms (int) — ms 単位なので 200/500 を直接渡す
  var qres = K4ABTNative.k4abt_tracker_enqueue_capture(tracker, capture, EnqueueTimeoutMs); // 200
  if (qres == K4A_WAIT_TIMEOUT) { droppedAtEnqueue++; continue; }
  if (qres != K4A_SUCCEEDED) { errorCounter++; continue; }

  var pres = K4ABTNative.k4abt_tracker_pop_result(tracker, out var bodyFrame, PopTimeoutMs); // 500
  if (pres != K4A_SUCCEEDED) { droppedAtPop++; continue; }

  // 4) skeleton を output slot に seq-lock で書く（frame_id を入力からコピー）
  WriteOutputSlot(bodyFrame, slot.frame_id, slot.ts_ns);
  K4ABTNative.k4abt_frame_release(bodyFrame);
  outputEvt.Set();

  // 5) per-second 診断ログ stdout
}
```

**Teardown（finally で必ず通す）**
1. `tracker_shutdown` → `tracker_destroy`
2. capture / calibration unmanaged buffer の `Marshal.FreeHGlobal`
3. event handle / MMF view / MMF 解放
4. exit code: 0 = clean shutdown, 1 = parent died, 2 = layout mismatch, 3 = tracker_create failed

**タイムアウト定数（魔数禁止）**

`K4abtWorkerSharedLayout.cs` 等で名前付き定数化:
```csharp
public const int EnqueueTimeoutMs = 200;
public const int PopTimeoutMs     = 500;
public const int InputWaitTimeoutMs = 1000; // shutdown チェック粒度
public const int ReadyWaitTimeoutMs = 15_000; // tracker_create + model load
public const int TeardownWaitMs   = 3_000; // host 側 WaitForExit
public const int ParentWatchdogIntervalMs = 1_000;
```
k4abt API は `int timeout_in_ms`（`K4ABTNative.cs` 参照）。秒換算と取り違えない。

### Phase 4: Unity 側 WorkerHost（1 日）
1. `Assets/Scripts/BodyTracking/K4abtWorkerHost.cs` (MonoBehaviour) を新設
2. API 案:
   ```csharp
   public class K4abtWorkerHost : MonoBehaviour {
       public string workerExePath = "Workers/K4abtWorker/.../k4abt_worker.exe";
       public bool useWorker = false;
       public event Action<string /*serial*/, BodySnapshot[]> OnSkeletonsReady;
       public bool IsReady(string serial);                  // ready_event 立った後 true
       public void StartWorker(string serial, ObCameraParam calib, int dW, int dH, int irW, int irH);
       public bool EnqueueFrame(string serial, byte[] depth, byte[] ir, ulong tsNs); // ready 前は false
       public void StopWorker(string serial);
   }
   ```

**Startup 手順（race 対策込み）**
1. `sessionGuid = Guid.NewGuid()`、prefix を `FB_K4ABT_{guid:N}_{serial}` で組む
2. `EventWaitHandleSecurity` / `MemoryMappedFileSecurity` を current user のみ full control で構築
3. `MemoryMappedFile.CreateNew(name, totalSize, MemoryMappedFileAccess.ReadWrite, MemoryMappedFileOptions.None, security, HandleInheritability.None)`（Windows 限定）
4. global header に magic / layout_version / 解像度を書き、**calibration blob (1032 B) を `K4ACalibration.Build` が返す `IntPtr` の中身から `MemoryMappedViewAccessor` の `WriteArray<byte>` で raw bytes コピー**（managed struct 化はしない）
5. 4 event を `EventWaitHandle(false, AutoReset, name, out createdNew, security)` で `createdNew == true` 確認（false なら衝突 → error）
6. `Process.Start` で worker 起動。`StartInfo.UseShellExecute=false` / `RedirectStandardOutput=true` / `RedirectStandardError=true` で stdout/stderr を Unity Debug.Log にバウンス（`Process.OutputDataReceived`）
7. 引数: `--session={guid:N} --serial=... --parent-pid={Process.GetCurrentProcess().Id} --depth-w/h ... --ir-w/h ...`
8. **Ready 待ち**: 別スレッドで `readyEvent.WaitOne(15_000)` を待ち、立ったら `IsReady=true`。タイムアウトしたら worker を Kill して error 報告

**Stale worker 掃除（v1 では non-goal）**

Per-session GUID で命名しているので、前回 crash で残った IPC オブジェクト名は今回の名前と一致せず、`TryOpenExisting` 経由の自動掃除はできない。これは意図的なトレードオフ:
- ハンドル類は最後に開いてるプロセスが死ねば OS が回収するため、長期残骸にはならない
- worker.exe 本体プロセスの残骸は parent watchdog（Phase 3 の parent PID 1Hz チェック）で worker 側が自殺するのでカバーされる
- どうしても発見が必要になったら別 issue で stable discovery key（serial だけ + ACL）を導入する

**Update ループ（Unity main thread）**
- 各 worker の `output_event` を `WaitOne(0)` で poll → 立ってたら output slot を seq-lock 読み → `frame_id` 重複は無視 → 新しければ `BodySnapshot[]` に parse → `OnSkeletonsReady` 発火（main thread）

**EnqueueFrame**
- `IsReady` でなければ即 return false
- input slot を seq-lock で書き、`produced++`、`input_event.Set()`
- 古いスロットを上書きしたら `dropped++`（latest-wins）

**Teardown（OnDisable / OnDestroy / OnApplicationQuit）**
1. host 側 enqueue を止める
2. `shutdown_event.Set()`
3. `Process.WaitForExit(3000)`
4. timeout したら `Process.Kill()`
5. event / MMF を Dispose
6. **冪等**にする（domain reload で OnDisable→OnEnable が連続しても安全）

### Phase 5: BodyTrackingLive 切替パス（半日）
1. `BodyTrackingLive` に `bool useWorker` Inspector フィールド追加
2. `useWorker == true` の時は in-process tracker create を skip し、`K4abtWorkerHost.StartWorker(serial, cameraParam)` を呼ぶ
3. `OnRawFramesReady` のハンドラで `WorkerHost.EnqueueFrame` を呼ぶ
4. `WorkerHost.OnSkeletonsReady` を購読して body visual を更新（既存の BodyVisual ロジックを流用）
5. shutdown 時に StopWorker

### Phase 6: 動作確認（1 日）
1. 既存 in-process パスと worker パスを Inspector トグルで切り替えて、両方で 1 カメラ Live BT が出ることを確認
2. 数値で比較: BT 推論 fps、skeleton position の差、Unity main thread の負荷、`produced/consumed/dropped` カウンタ
3. クラッシュ時 (`Process.Kill` をテスト)、Unity 側で worker 再起動できることを確認
4. **Editor を強制終了**して worker が parent watchdog で 1〜2 秒以内に自殺することを確認（タスクマネージャー or `Get-Process k4abt_worker`）
5. **Domain reload**（スクリプト保存）で worker が一旦終了→再 spawn することを確認、stale handle が残っていないこと
6. **多重起動**: Unity Editor を 2 つ立ち上げて両方で worker 起動 → IPC 名衝突せず両方動くことを確認（session GUID で分離されている確認）
7. seq-lock 書き込み中の torn read が起きないこと: 100k フレーム回して `seq_before != seq_after` の retry 回数を統計化

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
2. ~~calibration を file ではなく MMF header に乗せる~~ → **MMF header に乗せる方針に確定**（Round 1 review 反映）
3. MMF の `_input` シグナル粒度: 「フレームごとに毎回 set」で確定。worker が遅れたら latest-wins で古いスロットを上書き（dropped++ で計測）
4. Worker stdout/stderr を Unity の Console に流す: `Process.OutputDataReceived` 経由で `Debug.Log("[worker] ...")` にバウンスする方針で確定
5. seq-lock の memory ordering: .NET 上は `Volatile.Read/Write` で十分か、`Interlocked` まで上げる必要があるか（x64 では `Volatile` で OK の見込み。実装中に再確認）

## 完了の目安

issue #10 の Done 条件すべてチェック。具体的には：
- 1 カメラで worker 経由 Live BT が動く
- Inspector トグルで in-process と worker パスを切り替えられる
- worker のクラッシュからの復旧動作確認済み
- Editor 終了で worker も終了することを確認済み

## 後続

`Plans/issue-9-multicam-extrinsic-calibration.md` (#9) と組み合わせて `issue-11` (skeleton merge) に進む。
