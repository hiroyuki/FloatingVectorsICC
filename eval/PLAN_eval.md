# Body-Tracking 代替手法 評価タスク — PLAN

k4abt の関節精度不満を受け、2 代替手法を同一ハーネスで比較する。

- **Track A: Nuitrack AI**（商用ミドルウェア, RGB+Depth DL, Unity ネイティブ）
- **Track B: RTMPose + 深度逆投影**（RTMPose ONNX を **ONNX Runtime ネイティブ / CUDA EP** で実行 → D2C 深度で 3D 化）

本番: Unity 6.0 (6000.4.9f1) / Windows / Femto Bolt ×4 HW 同期 / OrbbecSDK v2 P/Invoke。

## 大原則
- **main には一切触れない。** 全作業は git worktree 上。
- 2 トラックは独立。共有するのは `Assets/Scripts/Eval/` の評価ハーネスのみ。
- 不明点（ライセンスキー入力・SDK DL 認証・有償登録・外部送信）は**必ず停止して確認**。

## Worktree topology
```
main
└─ eval/harness-base   (../FloatingVectorsICC-eval-harness)  ← 共有ハーネス
   ├─ eval/nuitrack-ai (../FloatingVectorsICC-eval-nuitrack) ← Track A
   └─ eval/rtmpose-depth (../FloatingVectorsICC-eval-rtmpose)← Track B
```
harness-base を共通祖先にし、ハーネス修正は両トラックへ merge で伝播。

## 確定した決定（ユーザー承認済み）
- RTMPose ランタイム = **2. ONNX Runtime ネイティブ (CUDA EP)**。詰まったら Sentis / worker プロセスに退避。差し替え可能な `ITrackerAdapter` 実装にする。
- 順序 = **1. 共有ハーネス + k4abt ベースライン先行 → 各トラック**。
- 評価データ = `D:\Dropbox\projects\ICC\Recordings\RecordingBase` の RCSV 録画（本プロジェクト形式）。

## 評価データの現状（要注意）
このマシンで実体があるのは 3 セッションのみ、いずれも短い：

| セッション | 台数 | 中身 | 尺 | 人数 |
|---|---|---|---|---|
| `2026-07-14_11-29-59` | 2 (N,Z) | depth+color+ir+**bodies** | ~2s | 1 |
| `2026-07-14_13-18-39` | 3 (N,Z,EG) | depth+color+ir+bodies | ~1s | 1 |
| `2026-07-14_13-02-45` | 3 (L,N,EG) | depth+color+ir（bodies 空） | ~1.3s | 1 |

- **全て 1〜2 秒と短く、2 人シーン無し、extrinsics.yaml も無い。**
- → 静止ジッター（数秒静止が必要）/ 2 人 ID スワップ / 4 台融合は**このデータでは測定不能**。
- `2026-07-14_15-50-24` 等の長尺は Dropbox 未同期（フォルダのみ）。同期後に採用予定。
- **配管の実装・動作実証には `11-29-59` で十分**。最終数値は長尺録画に差し替える。

## ハーネス構成 `Assets/Scripts/Eval/`（asmdef: BodyTracking.Eval）
| ファイル | 役割 |
|---|---|
| `EvalSkeleton.cs` | 共通スケルトン DTO。15 関節サブセット（`EvalJointId`）。k4abt(32)/Nuitrack/RTMPose(COCO-17) を正規化。座標=カメラ空間 mm, OpenCV frame（k4abt と同一で直接比較可） |
| `ITrackerAdapter.cs` | トラッカー抽象。`SubmitFrame` push → `OnSkeletons` event。差し替え式 |
| `EvalReplayDriver.cs` | RCSV 再生基盤。既存 `PointCloudRecording.RcsvFrameStream` を流用し depth+color+ir(+bodies) を実時間ペースで供給。ライブと同一 IF |
| `K4abtBaselineAdapter.cs` | ベースライン。録画 `bodies_main` を `RecordedBodySerializer` で decode → Eval に正規化 |
| `EvalMetrics.cs` | ジッター(mm σ)/フレーム間スパイク/継続率/レイテンシ/ID スワップ。CSV 出力 |
| `EvalRunner.cs` | オーケストレータ。driver→adapters→metrics 配線、レイテンシ計測、結果書き出し |
| `Editor/EvalRunnerEditor.cs` | Inspector に Load/Run/Export ボタン |

各トラック側でスタブ（`NuitrackAdapter` / `RtmPoseAdapter`）を実装し `EvalRunner` に追加登録する。

## メトリクス定義
- **ジッター**: 静止区間 [staticStart,staticEnd] での主要人物の各関節位置 3D RMS 標準偏差 (mm)。
- **時間安定性**: フレーム間変位 > 閾値(既定 50mm) のスパイク頻度。**フィルタ前の生値**（One Euro OFF）。
- **オクルージョン耐性**: 遮蔽区間での追跡継続率・関節 valid 率・confidence 推移。
- **レイテンシ**: `SubmitFrame`→`OnSkeletons` の実測 ms（Stopwatch）。※録画 bodies passthrough は同期 decode ゆえ N/A、実レイテンシはライブ worker モードで別途。
- **複数人**: 2 人以上での ID スワップ回数。※現データに 2 人シーン無し → 要長尺録画。

## Session refresh checkpoints
### CP2 (現在) — harness-base 実装＋検証 完了、コミット待ち
- (a) 完了:
  - 共有ハーネス全ファイル実装（`Assets/Scripts/Eval/*.cs` + Editor）。
  - **Unity batchmode で隔離コンパイル green**（`error CS`=0、`BodyTracking.Eval.dll`/`.Editor.dll` 生成）。
    - 注: worktree は `OpenCVForUnity`（gitignore）が無く Calibration が壊れるため、main の同フォルダへ **junction** を張って解決（`Assets/OpenCVForUnity`）。worktree 破棄時はこの junction を消すこと。
  - **ヘッドレス batch 実行基盤**（`EvalBatchRun` + `EvalReplayDriver.RunToEndSync`）で end-to-end 実証。
  - **実データでベースライン計測成功**（`13-05-21`、bodies-only の実人物 73/74f）: per-camera で continuity 100% / validJoints 15 / idSwaps 0 / conf 0.667(=MEDIUM)。metric 演算の正しさを確認。
  - バグ修正 2 件: (1) metrics を **(tracker, serial) 単位**に（カメラ跨ぎ time-series 汚染を除去）、(2) truncated frame / depth 欠落 / bodies-only 録画に耐性。
- (b) 次に読む: `Assets/Scripts/Eval/EvalMetrics.cs`, `EvalReplayDriver.cs`, `Editor/EvalBatchRun.cs`。
- (c) 残:
  1. harness-base を **commit**（レビュー方針を確認してから）。
  2. `eval/nuitrack-ai` / `eval/rtmpose-depth` を harness-base から分岐作成。
  3. 可視化オーバーレイ（`EvalSkeleton` → spheres/lines、既存 `BodyVisual` は internal ゆえ簡易版を新規）。※未実装。
  4. ライブ k4abt worker モード（実レイテンシ計測用）。※未実装、現状 baseline latency は N/A。
  5. **本計測は長尺録画待ち**（静止保持/遮蔽/2人区間・4台・extrinsics 付き）。現ローカルデータは画像＋人物が両立するセッション無し。

### CP1 — 済
- worktree 作成、既存資産マップ確定、設計確定。

## 参照（既存資産の要点）
- 共通 DTO 候補: `BodySnapshot` + `k4abt_joint_t[32]`（mm, OpenCV frame）。`BodyTrackingShared.K4AmmToUnity` で Unity 変換。
- フレーム供給: live=`PointCloudRenderer.OnRawFramesReady`, 録画=`SensorRecorder.OnPlaybackRawFrame`（本ハーネスは独自 `EvalReplayDriver` で RCSV を直接読む=UI 状態非依存で再現性確保）。
- 録画 reader: `PointCloudRecording.RcsvFrameStream(path)` / `EnumerateDevices(root)` / `SensorFilePath(root,host,serial,sensor)` / `ReadRcsvHeaderDimensions(path)`。root=セッションdir（`dataset/`,`calibration/` を含む）。
- bodies_main codec: `RecordedBodySerializer.Decode(bytes,count,BodySnapshot[])`。
- オーバーレイ数学: `MultiCam/SkeletonWorldTransform.ToWorld(jointMm, depthToColorMm, rendererTransform)`。
- 外部キャリブ: `PointCloudRecording.ReadExtrinsicsYaml(root)` → `DeviceCalibration`（intrinsics/D2C/GlobalTrColorCamera）。
