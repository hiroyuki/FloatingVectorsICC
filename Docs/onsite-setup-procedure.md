# 現地セットアップ手順（ハードウェアから通しまで）

新規会場でセンサー設置からシーケンス通しまでを**この順番で**進める。
チェック項目の詳細・チューニング値は [onsite-checklist.md](onsite-checklist.md) を併読。

## 0. 持ち物・PC 前提確認（出発前）

- [ ] Femto Bolt ×4 + 電源、**sync ケーブル**（デイジーチェーン or Sync Hub Pro）、USB3 ケーブル（長尺）、マウント/三脚
- [ ] **ChArUco ボード印刷物**（プロジェクトの CharucoBoardSpec asset と同一仕様のもの）
- [ ] 本番 PC に以下が入っていること:
  - プロジェクト一式 + Unity 6
  - K4A Wrapper（`D:\OrbbecSDK_K4A_Wrapper_v2.0.11_...`）と BT SDK 1.1.2（フォールバック用 k4abt。`Workers/K4abtWorker` は publish 済みか）
  - **CUDA/cuDNN + `eval/models/`（yolox-m, rtmpose-m）+ `eval/body_profile.json`**（ライブ融合と v11s 変換）
  - USDZ 用 python（`~/.venvs/usd`）
  - **LFKS 本番 token** → `<persistentDataPath>/lfks-token.txt`（期限 2026-10-31、サーバー ntticc.lfks.app）

## カメラの「順番」は 4 種類ある（混同注意）

| 順番 | 決まり方 | 何に効くか | どこで設定 |
|---|---|---|---|
| **① USB 列挙順（index）** | `QueryDevices()` の順。**シリアル指定不可**、USB ポートで変わる | **sync Primary（index 0）** と IR スタガー（160µs×index） | 設定不可 → **配線側を合わせる**（下記手順 2-3） |
| **② camera-id（cameras.yaml）** | キャリブ UI の assign モードで手動 | キャリブの world origin（cam0）と id 表示 | F1 → **I** → 矢印 + **O** |
| **③ rigSerialOrder** | ExperienceConfig に手入力 | **world rebase の +X 軸 = cam1→cam2 の向き**（QR や体験の向きに直結） | `ExperienceConfig.rigSerialOrder`（**現物は開発機のシリアルのまま — 必ず現地の 4 シリアルに差し替え**） |
| **④ expectedSerials** | 手入力（順不同） | カメラ死活監視（Fault） | `CameraHealthMonitor.expectedSerials` |

## 1. ハードウェア設置

1. カメラ 4 台をステージ四隅に設置（中心向き、高さ ~1m 目安 — 前回リグは床がキャリブ座標 y≈-0.9）
2. USB3 を PC へ — 可能なら**カメラごとに別の USB コントローラ**（ハブ共有は帯域切れの元）
3. **sync ケーブルはまだ確定配線しない**（Primary が USB 列挙順で決まるため、手順 2 で index 0 のカメラを確認してから、そのカメラを起点に VSYNC OUT → 次の IN と数珠つなぎ。`SensorManager.syncTopology` を配線方式に合わせる。Hub Pro でも「index 0 = Primary」は同じ）
4. **三脚・機材を capture volume（約 3.2×3.2m）の外へ**

## 2. PC 初回接続（カメラ認識）

1. **初回のみ**: 管理者 PowerShell で `D:\OrbbecSDK_K4A_Wrapper_...\scripts\obsensor_metadata_win10.ps1` を実行（UVC メタデータ登録。実行済み PC なら不要）
2. Unity で Play（live モード: `SensorManager.playbackOnly` OFF）→ 4 台のシリアルが列挙されるか確認、**index↔シリアル対応を控える**（ヒエラルキーの GO 名 `PointCloud[i] ... (シリアル)`、または verboseLogging のログ）
3. **index 0 のカメラを sync チェーンの起点にして配線**（①の表参照）。配線後に Play し直し、GetSyncConfig で Primary/Secondary の反映を確認
4. `CameraHealthMonitor.expectedSerials` に 4 シリアルを設定（④）
5. **カメラ設定の確認**:
   - IR 干渉スタガーは既定で有効（`SensorManager.applySyncConfig=ON`, `trigger2ImageDelayStepUs=160` → index×160µs）。**実機で GetSyncConfig の反映を確認**し、中空のチリチリ幽霊点が出ないこと
   - **COLOR の露出/シャッターを上げる**（アプリ内制御は無いので OrbbecViewer で各カメラに設定）— 色ブレ = 追跡の catch-up jump の根本原因

## 3. キャリブレーション（extrinsics）

Game 内 UI（`CalibrationRuntimeUI`）で実施。Editor 派なら `Window > Calibration > Multi-Camera Extrinsic` でも同じパイプライン。

1. **F1** でキャリブモード ON（BT/TSDF は自動サスペンド）
2. **I** で camera-id 割当モード（②）→ 矢印キーで並び順を決め、**O** で origin(cam0) 指定 → Esc。**この並びをそのまま ③ rigSerialOrder にも使う**と一貫する（rigSerialOrder の cam1→cam2 の向きが **+X 軸** = 体験の向きに直結）
3. ChArUco ボードを全カメラから見える位置で構え、**C** で capture（全カメラ同時 + skew ゲート）。**位置・向きを変えて複数回**
4. **S** で solve → `calibration/extrinsics.yaml` 保存（R=リセット、D=ダンプ、H=UI 隠す、F1 で通常表示へ復帰）
5. **検証**: 人が立って 4 台の点群が 1 体に重なること
6. **③ `ExperienceConfig.rigSerialOrder` を現地の 4 シリアル・確定した並びに差し替え**（既定値は開発機のシリアル! director が SensorManager/SensorRecorder に push する）

## 4. 床高（rebase）

1. rebase OFF の状態で点群の**床面の y をシーンビューで実測**（キャリブ座標系。前回リグは -0.9）
2. その値を `SensorManager.rebaseFloorY` と `SensorRecorder.rebaseFloorY` に**同値で**設定 → 以後 y=0 = 物理床（`ExperienceConfig.floorY` は 0 のまま）

## 5. ライブ動作確認

- [ ] `LiveFusedBodySource` 有効（provider=Cuda）→ **FusedHz/FreshFusedHz ≈ 30** を確認（開発機 CUDA で 30Hz 実測済み。出なければ DirectML フォールバックのログを疑う）
- [ ] 骨格が実人物に追従・カーブの見た目確認（render delay 相当は本番未実装 — playback 側のみ）
- [ ] presence: 入退場で occupancy が反応するか（閾値は checklist のチューニング項）
- [ ] freeze(Space)+Rec(F9) の同時動作（コードパス未実機検証 → dancer-session.md 参照）

## 6. Experience 用データ・設定

1. **アトラクト用テイク収録**（F9。BT が乗る動きのある 1 人テイクを複数）
2. 収録テイクを **v11s 変換**（`FloatingVectors > Eval BT` の FusedBatchConvert、または FusedTakeConverter。10 秒テイク ≈ 十数秒/本）
3. `ExperienceConfig`（Assets/Settings/ExperienceConfig.asset）:
   - `attractRecordingRoot` = 変換済みテイク群のフォルダ（`attractUseRecordedBodies=ON` 既定）
   - `visitorRecordingRoot` = 来場者テイク書き込み先（**高速ディスク**・attract とは別フォルダ・容量監視）
   - `dryRunPublish=false`、`qrUrlKind`（LFKS の見え方で決定）
4. シーンに **ExperienceDirector GO** を追加し config 割当（HUD に Experience Mode チェックボックスが出る）
5. （任意）ポーズガイド画像/SE 差し替え（`poseGuideTexture` / `banzaiGuideTexture` / AudioClip 群。未設定はシルエット+無音で動作）

## 7. 通し（E2E）

1. HUD の **Experience Mode ON** → 実人物で 1 サイクル:
   大の字（`per-visitor bone profile applied` ログ確認）→ 自由時間 → カウントダウン → **0 から 1 秒撮影** → 変換プログレス → 結果表示（凍結彫刻）→ QR をスマホでスキャン → モデルが開くこと
2. 大の字タイムアウト（既定プロファイル続行）、途中離脱 → Attract 復帰、USB 抜き → Fault → 復帰も一度ずつ
3. しきい値・文言・タイミング調整 → ソークテスト（checklist の該当項目へ）

## トラブル時の分岐

- 大の字/ばんざいが反応しない → star*/banzai* しきい値（checklist）、`director.CurrentMetrics` / `MetricsMeasured` をログで確認
- 変換が遅い/失敗 → k4abt bodies で自動フォールバックして進行は続く（`processingTimeoutSeconds=90`）。ログに `[FusedTakeConverter]`
- LFBS が死んだ → k4abt フィードが自動スポーン（ログ `live fused source inactive`）。復帰も自動
- アップロード失敗 → `~/Documents/FloatingVectorsPrints/` に残るファイルを lfks-upload-guide.md の手順で手動アップロード
