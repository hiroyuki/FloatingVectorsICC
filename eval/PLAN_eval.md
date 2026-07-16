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
### CP5 (2026-07-16 夕) — 融合品質の反復改善 v7 まで（目的: 本番採用判断 → main マージ → 印刷STL）
- ユーザーの位置づけ: **トラッキング精度を本番レベルに上げるのが主目的**。合格なら main にマージし、
  その後 curved line 付きモデルの STL を作成して 3D プリンタへ。
- (a) v1→v7 の改善記録（全編スパイク>200mm / L トラック基準）:
  v1=230(暴走直線) → v2=107(速度キャップ+ジャンプゲート) → v3=78(median-3)
  → v4(ボーン長ハード拘束: 髪振り頭めり込み 13→0) → v5=80(オクルージョン z-テスト)
  → v6=43(median-5, 遅延66ms) → **v7: Heartbeat で emission gap 15→0**(収録フレーム落ち対策)
- (b) 主要な学び:
  1. confidence は深度リフト Z 誤差を見ない(592mm誤差でconf0.64) → クロスカメラ偏差が判別信号
  2. 深度マップ = オクルージョンの幾何学的判定器(z-テスト)。「隠れてる関節の捏造」をペア裁定で破棄
     (ユーザー提案のロジック、v5 実装)
  3. ボーン長キャリブは3役: 門前払い/レイ×長さの深度復元/出力ハード拘束
  4. 残る~43スパイクは**モーションブラー帯(45s/54s)の3フレーム以上逸脱** = センサーレベルの情報消失。
     フィルタでは回収不能 → **次回収録はシャッター速度を上げる**(+表現側でconf連動の曲線減衰も検討)
  5. 収録はHW同期ゆえフレーム落ちも4台同時(66-299ms穴) → Heartbeat(保持ポーズの心拍emit)で充填。
     ライブでもセンサーヒカップ対策としてそのまま有効
- (c) 再生A/B: `FloatingVectors > Eval BT > Use RTMPose-fused bodies` → `D:/FVICC_eval/15-50-24-rtmfused`
  (streams hardlink済み、bodies_main=v7)。書き出し: FusedBodiesExport.Start/Step(400)/Finish (~8分,
  MCPタイムアウトしても処理継続、小Stepで進捗確認)
- (d) 次セッション:
  1. ユーザーの目視判定(特に 23s のカクつき解消 / 45s/54s ブラー帯の許容判断)
  2. 合格なら: main マージ計画 — 残ギャップは**ライブ推論**(4cam RTMPose の GPU 予算、live capture 統合、
     CUDA/TensorRT EP 導入=人手) と本番 SkeletonMerger との接続設計
  3. STL: 再生 → Print Export パネル(Window > Print Export) → Pause → Fuse→Close→Export STL
     (出力 ~/Documents/FloatingVectorsPrints/)。凍結レシピは CP4 参照
  4. 表現側オプション: conf 連動の curved line 減衰(ブラー帯対策)

### CP4 (2026-07-16 午後) — 融合パイプライン初版まで完了
- (a) 完了:
  - **本番修正済**: SkeletonMerger volume ゲート（main e8a0e6f → cherry-pick 4c2d498）
  - **frame 788 デバッグ環境**: シーク→凍結レシピ（seek → EditorApplication.Step ×N →
    timeScale=0 + editor unpause で Game view オービット可 → TSDF RequestFullClearNextBatch+再emit）
  - **オクルージョン分析**: confidence では深度リフト Z 誤差を検出不能（実測: 592mm誤差で conf 0.64）。
    クロスカメラ偏差が判別信号。表面視差フロア（胴体 ~150-250mm）あり → 関節別ゲート必須
  - **身体キャリブ**: GoodFrameScan（全セッション走査）→ ユーザー目視承認（45/84/369/993/1056）→
    BodyProfileBuilder → `eval/body_profile.json`（左右対称性で検証済: 前腕 223/223mm）
  - **FusedRtmposeAdapter**（3段融合: ボーン長サニティ→median コンセンサス→レイ×ボーン長 relift
    +時間保持）+ FusedCompareChunked。初回 900f/cam: fused は 15.0/15 関節、whole-range
    jitter 305mm（per-cam RTMPose 318-414 / k4abt 297-336）。f788 で完全骨格・ボーン長整合
  - **クラッシュ罠**: RtmPoseAdapter.Dispose は渡された backend を破棄する —
    BtFrameInspectorWindow.SharedBackend() 利用時は adapter を Dispose しないこと（Editor 2回クラッシュ済み。
    OrtRtmposeBackend に破棄後ガード追加済み）
- (b) 既知の課題（次セッション）:
  1. fused の spikeRate 17.7% — カメラ集合の入れ替わりで視差オフセット分の跳び。
     候補: 全カメラ揃いでのみ融合 / カメラ×関節の系統バイアス学習 / 出力 One-Euro
  2. fused continuity 29% 表示は分母バグ（camera-frame 毎に AddSubmitted。実カバレッジは ~1.16 emissions/frame-set）
  3. 静止区間つき録画での純ジッター測定は未
- (c) 判断材料: 融合でオクルージョン破綻は修復可能と実証。残るは spike 源の除去と
  ライブ性能（4カメラ×RTMPose の GPU 予算）確認 → k4abt 置き換え判断

### CP3 (2026-07-16, セッション終了時点) — Track B 完了・ビジュアル A/B 稼働・本番修正 1 件持ち越し
- (a) 完了（このセッション）:
  - **Track B (RTMPose) 一式**: ORT DirectML バックエンド、捕捉ボリューム人物選別
    （box中心1点→**5x5グリッド中央値深度**修正済）、detect-once-then-track（**~44ms/frame**）、
    メトリクス比較（`eval/results/compare/`）、`eval/COMPARISON_REPORT.md`。
    codex-review 承認済（`codex review --base eval/harness-base` を worktree で直接実行）。
  - **本番ビジュアル A/B**（本番コード無変更）: メニュー `FloatingVectors > Eval BT >
    Use k4abt|RTMPose bodies`。k4abt=元セッション、RTMPose=`D:\FVICC_eval\15-50-24-rtmpose`
    （depth/color/ir はハードリンク、bodies_main は RtmposeBodiesExport 生成・全編 ~1540f/cam）。
    再生は SkeletonMerger.ignoreRecordedBodies=false 経由。開発ルック（TSDF+curves）も再生で動作
    （Views: Point cloud OFF / TSDF mesh ON / Motion lines ON — Window > Control Panel）。
  - **BT Frame Inspector**（`FloatingVectors > Eval BT > Frame Inspector`）:
    再生を観て → **Grab & Freeze**（Editor ごと凍結＋フレーム取得）→ カメラボタンで
    1台ずつ点群+k4abt(シアン)+RTMPose(オレンジ)+ボーンを Scene に表示。
    フレームは**タイムスタンプ照合**（カメラ毎に index が最大56f ずれる問題を修正）。
    配置は常に本番同一（extrinsics+world rebase）。viz は Play 遷移で自動削除。
  - **frame 788 の個別検証**: cam L で k4abt がほぼロスト(5関節) vs RTMPose 14関節、
    cam N で k4abt が歪んだポーズ。→ ユーザーの「k4abt 関節精度への不満」を具体化。
- (b) 次セッションで最初に読む: この CP3、`eval/COMPARISON_REPORT.md`、
  memory の `eval-bodytracking-task.md` / `codex-review-setup.md`。
  Editor は worktree `F:\FloatingVectorsICC-eval-rtmpose` を開く（main とは別インスタンス）。
- (c) 残タスク:
  1. **【持ち越し・別タスク】本番修正: SkeletonMerger に bounding volume ゲート追加**
     — 再生で box 外の見学者 (Body_3786, x≈-3.5m) を追跡・描画していた。
     「volume 外は捨てる」仕様に反する（誤作動・リソース圧迫）。本番コードなので要承認+計画。
  2. ユーザーのビジュアル A/B 継続（k4abt vs RTMPose の絵の所感 → レポートに追記）
  3. RTMPose さらなる高速化: CUDA/TensorRT EP（要 CUDA12.x+cuDNN9 導入=人手、現状 11.6/cuDNN8）
  4. Nuitrack ライブ試用（デバイス接続時、A-1: ライブ+同時録画）
  5. 静止保持区間つき録画で純ジッター測定（任意）
- 注意事項:
  - Frame Inspector の自動掃除ガードは**次回コンパイルから有効**（コミット済 6c4b619）
  - worktree には OpenCV junction (`Assets/OpenCVForUnity` → main) と
    Workers/K4abtWorker の publish 済 exe がある（このマシンに BT SDK 無し→ライブ k4abt 不可）
  - MCP: `set_active_instance` で worktree インスタンスを選ぶ。GPU 系（RTMPose/DirectML）は
    batchmode -nographics では動かない → 対話 Editor + execute_code（>60s は MCP タイムアウト
    するが処理は継続、chunked Start/Step/Finish パターンを使う）


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
