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
### CP7b (2026-07-16 深夜) — v11s: ブラー帯カクつき対策 + プロジェクタマスク完成
- ユーザー判定: 体の欠け解消 / チリチリ「たまに出るが影響小」/ 新指摘 = 高速手振り箇所でカーブがカクつく
- カクつきの実体 = **固まり→追いつき**（ブラーで 2D 姿勢が数フレーム停滞 [ステップ 40-60mm]、
  ~400mm 一発で追いつく）。融合時フィルタでは不可（センサー情報損失）だが**オフラインなら完全復元可能**:
  `FusedCatchupSmooth.Run(root)`（44a7887）が停滞区間へジャンプを時間比例で再配分
  （方向継続するジャンプのみ、往復スパイク除外、全トラック同一適用）。
  **結果: 全編スパイク 16 → 2**（v8 比 -95%）、55s 帯の手首は等速列に。原本は bodies_main.prekink
- A/B ルート = v11s（v11 + catch-up smooth）。ライブの根治は次回収録のシャッター速度（TODO 済）

### CP7 (2026-07-16 夜) — v11: ミラーペア対策（z-テスト降格・時間連続性タイブレーク・忠誠ルール）
- (a) ユーザー第2指摘: 24.1s（静止中）右手首が 378mm 跳んで 5 フレーム→329mm 復帰。
  v9/v10 でも残存（v10 = タイブレーク単体では無効だった）
- (b) 新設の**デバッグプローブ**で確定（FusedRtmposeAdapter.dbgJoint/dbgFromNs/dbgToNs/DbgSb、
  FusedBodiesExport.ActiveAdapter 経由で設定 → 候補・occdrop・クラスタ選択・ゲートを時系列ダンプ）:
  後ろ向き姿勢で N+EG が右手首を**左手（実在面）にミラー反転**。クラスタ前に走っていた
  オクルージョン z-テストが**正しい Z を落としていた**（Z は体の反対側の左手位置が本当に見えない
  =「盲目」判定、一方ミラーペアは輪郭エッジの背景深度で「見えている」判定になる）。
  残った {N,EG} が唯一のクラスタとして strong 勝利
- (c) v11 修正（commit 1b2e8dd）:
  1. **z-テストをクラスタ前段から weak パスへ降格**、かつ fresh history が無い時のみ実行
     （コールドスタートの 1対1 裁定 = v5 本来の用途）。「他人の位置が見えない事は自分の主張の
     反証にならない」
  2. 同数クラスタのタイブレーク = **クラスタ重心の予測距離**（confidence は 3D 誤差を見ない）
  3. **忠誠ルール**: ペア（≥3 は対象外）が予測から jumpGate 超に跳び、クラスタ外に
     予測near+骨長妥当な候補が居る場合 strong 扱いしない → weak パスが忠実候補を採用
- (d) 結果: 全編スパイク **40(v8) → 28(v9) → 16(v11)**、頭スパイク 0。16 中 13 は
  53.7-56.5s ブラー帯（センサー起因）。帯外は 9.4s/29.9s(j7), 33.1s(j14) の 3 件のみ。
  24.1s は {L,Z} が全バースト predD≈10mm で勝ち連続トラック化
- (e) A/B ルートは v11 反映済み（tmp: D:/FVICC_eval/tmp-v11-bodies が原本）。
  v9-v11 の codex review はまとめて未実施
- (f) 教訓: 「静止中に関節が数百 mm 跳ぶ」系は全て**複数カメラの構造的合意汚染**
  （背景抜け深度 / ミラー反転）で、単一カメラのノイズではない。時間連続性が最強の裁定者、
  ただし count≥3 の合意には勝たせない（本物の高速移動を殺さないため）
- (g) **チリチリノイズ調査（ユーザー指摘・原因確定）**: 舞台中央の宙に点滅する幽霊点群 = cam L の
  深度画像で、真正対の cam Z（6.5m 先）の IR プロジェクタが写るピクセル (u=171,v=129) の周辺
  (u187-192, v99-139) が**飽和/干渉で 6.5m を 3.0m と誤測距** → L→Z 視線の中点（舞台中央）に浮く。
  全セッションの 97% のフレームで発生。IR/カラー画像で確認済（Assets/Screenshots/L_ir_mid.png 等）。
  **影響: BT fusion は無事だが、PointCloudMotionCurves の curve seeding が点群に引かれてカーブが
  いびつになる（ユーザー実観測）+ TSDFJointSnap（関節→TSDFシェル snap, 半径0.2m）経由でも汚染し得る**
- (h) **TODO（live-capture 統合 / main マージ計画に含める）**:
  1. **IR 干渉スタガ**: `PointCloudRenderer.trigger2ImageDelayUs` をカメラ毎に 160µs×n で設定
     （例 N=0, Z=160, L=320, EG=480）。現状デフォルト 0 = 全カメラ同時発光。設定はキャプチャシーンの
     Inspector、投入後 `GetSyncConfig` で実機確認。sync ケーブルはトリガー共有のみで干渉回避は
     開発者責務（Azure Kinect 公式ドキュメントの指示事項）
  2. **プロジェクタ写り込みマスク — 実装済み（0f35788 + 修正 2722274, ユーザー承認済）**: `ProjectorMask`
     （新規）が extrinsics から他カメラ位置を各カメラ深度画像へ投影し、輝点中心の IR 輝度ゲート
     （可視 ~29000 / 遮蔽 ~50 → 閾値 2000）が開いた時のみ、半径 45px ディスク内の**閉じた小連結成分
     だけを消す**（flood fill、リンク 300mm、境界到達 or ≥500px = 実物で保持）。
     初版のディスク全消しは**体がディスクに重なりつつ輝点が脇から見える頻出ケースで体に穴を開けた**
     （ユーザー発見）。ピクセル単位 IR も不可（フレア偽深度画素は IR 暗、中央値 5 vs 体 90 — 位相破壊
     でありブルームではない）。**検証**: 体横断フレームで cutFromSurface=0（面切り取りゼロ）、
     空舞台ゴースト 63→0、壁も残存。SensorRecorder.EmitFrameAt で全消費者より前に適用、
     Inspector トグル maskRivalProjectors（既定 ON）。
     残 TODO: ライブ経路（PointCloudRenderer.OnRawFramesReady）への配線は main マージ時

### CP6 (2026-07-16 夜) — v9: クラスタ合意（ユーザー指摘「静止中に頭のラインが振れすぎ」への対応）
- (a) ユーザー指摘: 静止区間で頭のモーションラインが見た目以上に振れる → 実測で確認:
  28.26s に頭が首を挟んで 344mm サイドフリップ→5フレーム滞留→340mm 復帰。pelvis はほぼ静止
  （= CP5 の「残スパイクはブラー帯」の説明の外にある未対策モード）
- (b) 根本原因（カメラ別 raw を world 変換して特定）: その瞬間 4 台中 2 台が大外れ
  （N=深度リフトが背景壁 z≈7m / EG=約3.5m 転位）。成分 median が両クラスタの混合点になり、
  **一致している L/Z がゲート外に弾かれ** consensus 不成立 → ray×bone relift が狂ったカメラの
  視線で頭を再構成（z=3175、fused 出力と一致）→ ジャンプゲートは「同位置の繰り返し」で自己確証
- (c) v9 修正（FusedRtmposeAdapter, commit 0e2edaa）:
  1. **クラスタ合意**: ≤4台なので部分集合全列挙、ペア距離が全て関節ゲート内の最大クラスタ
     （タイは重み合計）を採用。旧 two-sample 特例も包含
  2. 合意なしフォールバックは**時間予測への近さ順**で single-accept / relift（辞書順廃止）
  3. **ジャンプゲートの証拠階層**: 方向継続と 1 確認受理は strong（クラスタ）限定。
     weak（単眼/relift）は weakJumpConfirms=3 フレーム必要（遅延受理の残渣 ≤1 フレームは
     median-5 が構造的に除去）
- (d) 結果: 全編スパイク>200mm **40 → 28**（残りはほぼ 45-56s ブラー帯の手首）。
  18.5s / 28.3s の静止帯頭フリップ消滅。27.4-30.5s の頭軌跡は滑らかな連続アーク
  （最大ステップ 344→63mm）。**静止窓の頭 P2P 300-500mm は残るが k4abt でも同量 → 実際の頭の動き**
  （この区間ダンサーは頭を大きく動かしている）。なお Head=COCO Nose（RtmposePipeline.cs:167）
  なので「頭ライン=鼻の軌跡」。頭の回転で鼻は空間を掃引する — 見た目とラインの乖離が気になる場合の
  次の一手は Head アンカーの再定義（両耳中点ベース等）だが、k4abt 比較では過剰でない
- (e) A/B ルート `D:/FVICC_eval/15-50-24-rtmfused` は v9 に更新済み（tmp: D:/FVICC_eval/tmp-v9-bodies）。
  再生・開発ルック・fused 選択は復旧済み。ExitPlaymode すると A/B 選択と Recorder.Play は消える →
  メニュー `FloatingVectors > Eval BT > Use RTMPose-fused bodies` を再実行（recorder 再生も再開してくれる）
- (f) 次: ユーザー目視判定の続き（23s カクつき / 45s/54s ブラー帯 / 頭ライン v9 の印象）→
  合格なら CP5(d)2 以降（main マージ計画 → STL）。v9 の codex review は未実施（マージ計画時に一括で）

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
