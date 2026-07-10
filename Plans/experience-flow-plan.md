# 体験フロー（Experience Flow）実装プラン

## Context

子供向けインスタレーションとして、来場者1人の体験フロー（アトラクト → ようこそ → 練習 → 3回キャプチャ → 選択 → glb/usdz 書き出し → LFKS アップロード → QR 表示）を仕上げる。既存の開発ワークフロー（メッシュ調整・curved line 確認・手動 Rec/Play/Export）は「Dev モード」としてそのまま残す。

**確定済みの仕様判断**（ユーザー回答）:
- タッチ判定 = 手（HAND_LEFT=8 / HAND_RIGHT=15、どちらでも）
- 「mesh+curved line は15フレーム」= 軌跡長15フレームの**静的**スナップショット
- 物理3画面: 画面A（Unity Display 0）= オペレーター（HUD ⇔ 4カメラカラーをキー切替）、画面B/C（Display 1,2）= 来場者向け
- カメラ異常の赤文字は**全画面**に表示し体験を停止（Fault 状態）
- 選択球は体験者の前に横並び（X=-1/0/+1m、高さ~1.2m）
- 球選択成立時: ワイヤーフレームがグロー + SE 再生（AudioClip スロットのみ用意、音素材は後日）

**コード調査で確認済みの土台**:
- `CameraHealthMonitor.cs` が既存（stall 検出 + IMGUI バナー）→ 拡張で対応
- `TSDFPrintExporter.ExportWeb()`（:498）は `RunMarchingCubes`(:511) + `WriteWebFiles`(:594) で volume 非破壊（`EnsureSnapshot` 不要）→ 3回連続キャプチャは安全
- main.unity に既に3カメラ: `Display1 Black`(display0) / `Main Camera`(display1) / `Display2 Camera`(display2)。シーンの `MultiDisplayActivator.displayCount` は**既に 3**（main.unity:2112）→ シーン変更不要。スクリプト既定値(2)を 3 に合わせ、各カメラの targetDisplay を検証するのみ
- `SkeletonMerger.HandleRawFrame` は playback 再生中、同 serial の live フレームを worker に流さない → アトラクト中の在場検知は BT 不可、点群占有カウントで行う
- `SensorRecorder.Load()` が live renderer を破棄する（~:1286）→ フラグでスキップ可能にする（再列挙 ~15秒はアトラクト→体験遷移で許容不可）
- 軌跡ウィンドウ = `BonePoseHistory.historySamples`（HUD の History Samples と同じノブ）→ キャプチャ直前に 15 へ
- Bloom は `DefaultVolumeProfile.asset` に intensity 0 で存在 → グローは unlit 線マテリアルの HDR 輝度ランプを既定、Bloom 有効化はチューニング項目

## 新規ファイル

`Assets/Scripts/Experience/`（新 `Experience.asmdef`、参照: Shared, Orbbec, PointCloud, BodyTracking, TSDF, Calibration, UnityEngine.UI）:

| ファイル | 責務 |
|---|---|
| `ExperienceDirector.cs` | ショーの単一オーナー。モードスイッチ（`IViewToggle` 実装 "Experience mode" で Control Panel に自動出現 + Inspector + HUD トグル）。Enter 時に既存コンポーネントの値をスナップショットし Exit 時に全復元（非破壊） |
| `ExperienceStateMachine.cs` | 純C#。状態: Attract / Welcome / FreePlay / Ready / PromptAnimal / PromptMantis / PromptFree / Select / Exporting / QrShow / Fault |
| `ExperienceConfig.cs` (ScriptableObject) | serial順(カメラ1-4)、各タイミング、文言バリアント配列(4,5用)、球の半径/位置、dryRun フラグ、アトラクト収録ルート、占有閾値。**token はアセットに含めない**（下記 Publishing 参照） |
| `VisitorMessageUI.cs` | Display1,2 それぞれに Canvas(targetDisplay)。API: ShowMessage / ShowCountdown / ShowAlert(赤) / ShowQr / ClearAll。フォント = `Font.CreateDynamicFontFromOSFont("Yu Gothic Medium", ...)`（uGUI Text、白。Mac では LegacyRuntime.ttf フォールバック + ログ） |
| `PresenceDetector.cs` + `PresenceOccupancy.compute` | `PersonCount` / `IsPersonInside` / `TryGetHands`。体験中 = BT（SkeletonMerger 経由、骨盤が OBB 内、デバウンス付き）。アトラクト中 = live 点群のOBB内占有カウント（compute + AsyncGPUReadback、`PointCloudMotionCurves` の vertex buffer bind パターン流用）。`debugForcePresence` で Mac 検証 |
| `AttractPlaybackController.cs` | 収録フォルダ列挙→ランダム選択→ `recorder.Load()` + 再生。ループ時に別収録へ再抽選 |
| `DwellSphere.cs` | ワイヤーフレーム球（3直交円の Lines メッシュ、`BoundingVolume.BuildWireCubeMesh` パターン）。手が半径内→円弧が1秒で円に(progress で部分円再構築)、離れるとリセット。成立で `OnSelected` + グロー(HDR輝度ランプ ~×6/0.4s) + `if(selectedClip) PlayOneShot` |
| `ExperienceSpaceBuilder.cs` | rebase 後のカメラ XZ 位置から対角1m内側の4点→既存 `BoundingVolume` の transform を設定（(0,1.25,0), scale=(sx,2.5,sz)）+ `FloorOrigin.boundingBox` 接続（グリッドは自動追従）。Exit 時に元 transform 復元 |
| `Publishing/ISculptureResultPublisher.cs` / `LfksUploadPublisher.cs` | `Task<string> PublishAsync(glb, usdz, ct)`。`powershell.exe -NoProfile -ExecutionPolicy Bypass -File <ローカル固定パスの upload.ps1> ... -Json` を `Process` + `Task.Run` で実行、stdout JSON から `downloadUrl` を取得。timeout 60s + 1リトライ。dryRun はフェイクURL。**token の扱い**: gitignore 済みローカりファイル `<persistentDataPath>/lfks-token.txt`（または環境変数 `LFKS_TOKEN`）から読み、**環境変数経由で子プロセスに渡す**（ps1 側は `-Token` 未指定時に `$env:LFKS_TOKEN` を読む薄いローカルラッパを用意。コマンドライン引数に token を載せない＝プロセス一覧露出回避）。upload.ps1 は事前に手動ダウンロードしたローカルコピーのみ実行（毎回の curl 実行はしない）。**スクリプトの信頼境界**: パスは StreamingAssets 等の固定位置にピン留めし、実行前に SHA-256 を既知値（config に平文で持ってよい、秘密ではない）と照合、不一致なら実行せず Fault ログ |
| `Publishing/IUrlPresenter.cs` / `QrUrlPresenter.cs` | QR 表示（後日 `OscUrlPresenter` に差し替え可能）。QR 生成は Nayuki qrcodegen C# 移植（単一ファイル MIT）を `ThirdParty/QrCodeGen.cs` にベンダリング → Texture2D(Point filter) |
| `Shaders/SnapshotVertexColor.shader` | スナップショット表示用の頂点カラー unlit URP シェーダ（~20行） |

その他新規:
- `Assets/Scripts/TSDF/PrintExport/TSDFSnapshotBuilder.cs` — キャプチャの核（下記）
- `Assets/Scripts/Calibration/WorldFrameRebase.cs` — ワールド再基準化の純数学（+ EditMode テスト）

## 既存ファイルの変更（すべてフラグゲートで Dev モードは現行動作のまま）

- `TSDF/PrintExport/TSDFPrintExporter.cs` — `RunMarchingCubes` と `WriteWebFiles` の前半(weld/smooth/decimate、:623 の recentre より前)・後半(recentre/mirror/書き出し)を `TSDFSnapshotBuilder` へ抽出。`ExportWeb()` は Capture+ExportFiles への委譲に書き換え（出力はビット同等を検証）
- `TSDF/PrintExport/CurveTubeBuilder.cs` — `exportSpace` パラメータ追加（false でミラーせずワールド空間チューブ生成）
- `PointCloud/SensorRecorder.cs` — `keepLiveRenderersOnLoad`（Load 時の `DestroyAllRenderers` スキップ）、`StopAndUnload()`（既存 teardown ~:2231 の公開）、rebase フック
- `PointCloud/SensorManager.cs` — `applyWorldRebase` + `rigSerialOrder`、`SetLiveVisualsVisible(bool)`（各 live renderer の MeshRenderer.enabled と `suppressAsSource` を同時に切替。フレーム取得・BT worker への供給・占有検知は継続）
- `PointCloud/PointCloudRenderer.cs` — `public bool suppressAsSource`（既定 false）。**意味は「彫刻系ソース（TSDF統合・motion curves シード）から除外」であり、live カメラ全体の無効化ではない** — PresenceDetector の占有カウントはこのフラグに関係なく live renderer のメッシュを読む
- `PointCloud/CameraHealthMonitor.cs` — `IsHealthy` + `OnHealthChanged` イベント、子供向け文言「カメラ（ID {n}）が　いじょうです」、**同一データ検出**（depth payload のストライドサンプル XOR ハッシュが `identicalContentSeconds` 不変で異常、既定 off）、suppression 修正（`IsPlaying` での無条件 return → live renderer 0 台のときのみ抑制）
- `PointCloud/MultiCameraDebugView.cs` — `Visible` プロパティ公開
- `BodyTracking/SkeletonMerger.cs` — `PersonCount` / `CrowdActive` / `TryGetPersonPelvis` / 手ジョイントの indexed 取得
- `CameraControl/Display1OperatorHud.cs` — Experience Mode トグル、HUD⇔4カメラビュー切替キー（Tab 等）
- `CameraControl/MultiDisplayActivator.cs` — スクリプト既定値 2→3（シーンは既に 3。各表示カメラの targetDisplay は検証のみ）

## ワールド再基準化（原点=4台重心、X=カメラ1→2、Z=カメラ2→3）

**extrinsics 適用時にインメモリで rebase**（親GO移動でも yaml 書き換えでもない）。live は SensorManager 配下、playback は SensorRecorder 配下で単一親が無く、両経路とも `ExtrinsicsApply.ApplyToTransform` を通るため、そこが一点フックで可逆。

**合成空間の契約**: rebase は **`ToUnityLocal()` の後、Unity 空間の Pose として合成**する（`ApplyToTransform` が返す localPosition/localRotation に `unityRebase` Pose を左から適用）。ObExtrinsic（OpenCV/mm、Y-flip 前）空間では合成しない — `ApplyToTransform` は localScale の Y-flip を保持するため（ExtrinsicsApply.cs:88）、OpenCV 空間合成は flip と干渉する。`WorldFrameRebase.TryCompute` も入力を `ToUnityLocal()` 変換済みのカメラ位置（Unity 空間）で受ける。playback（Y-flip 付き `_Playback_*`）と live の両方で同一結果になることをテストで保証。

`WorldFrameRebase.TryCompute`: serial順の4カメラ位置 → 重心を床平面に投影して原点（Y-up と床高はキャリブのまま、yaw+XZ のみ再基準化）、X̂=projXZ(p2-p1)、Ẑ=Ŷ×X̂（p3-p2 と>10°ズレたら警告）。

## キャプチャ・パイプライン（核心）

`TSDFSnapshotBuilder`（static、TSDF asmdef）:
1. `Capture(volume, curves, options) → SnapshotData` — MC slabs（GPU同期読み戻し）+ `curves.TryReadCurvePolylines` + weld/smooth/decimate（WriteWebFiles 前半の verbatim 移動）。**SnapshotData の座標契約を明示**: メッシュ頂点・curve polyline とも「ワールド空間（Unity LH）」で保持し、export 空間メタデータ（`center`, `minY` — WriteWebFiles:623 と同じ算出、サーフェスメッシュから**1回だけ**計算）を SnapshotData に格納。mesh と curve は必ず同じメタデータで export 変換する（現行実装と同一の整合を保証）
2. `BuildDisplayMeshes(SnapshotData) → (Mesh surface, Mesh tubes)` — シーン内表示用（選択画面の3体）。ワールド空間そのまま、ミラー無し
3. `ExportFiles(SnapshotData, glbPath, usdzPath)` — SnapshotData 内の center/minY を使って recentre/mirror/winding修正/GlbWriter/UsdzWriter（後半の verbatim 移動）。**アトミック書き込み**: `<path>.tmp` に書いて成功時に `File.Move`（rename）。キャンセル/失敗時は .tmp を削除 — 手動再アップロード用に残す正規ファイルと部分ファイルが混ざらない

キャプチャ手順（state 4/5/6 末尾）: 残り~1秒で `historySamples` を保存→15 に設定 → **readiness 待ち**: `BonePoseHistory.CurveSamples == 15` かつ curve 再構築が設定変更後に完了したこと（PoseVersion/rebuild カウンタの前進を確認、または強制 rebuild API を追加して await）を確認してから `Capture()` 同期実行 → 復元 → 表示GO を非表示で生成。`TryReadCurvePolylines` は既ビルドの `_outBuf` を読むため（PointCloudMotionCurves.cs:610）、この待ちが無いと**旧ウィンドウのバッファを書き出す**。volume は非破壊なので3回連続で問題なし。**MC 読み戻しのヒッチが1.5秒超なら** CPU 後半を `Task.Run` に逃がす（Phase 1 で実測）。

注意: `TryReadCurvePolylines` は `curves.visible=true` が前提 → state 2以降 visible 維持。静止しすぎで polyline が潰れたら mesh のみでキャプチャ成立（ログは残す）。

## アトラクト⇔ライブ遷移

- アトラクト: live renderer は残したまま（`keepLiveRenderersOnLoad`）visual だけ隠し、playback が TSDF/curves/BT を駆動。在場検知は点群占有
- **live+playback 共存時の下流ゲート（renderer 破棄スキップだけでは不十分）**: live renderer のフレームを購読する各コンシューマを列挙してアトラクト中の挙動を明示する —
  - `SkeletonMerger`: 既存の same-serial suppression で playback 優先（変更不要）
  - **TSDF 統合**: `TSDFIntegrator` は live の `OnRawFramesReady` を直接購読し renderer 可視性を見ない（TSDFIntegrator.cs:294, :330）→ **明示的な変更**: `PointCloudRenderer` に `public bool suppressAsSource` を追加し、TSDFIntegrator は dispatch 前にこのフラグで live フレームを skip する。`SensorManager.SetLiveVisualsVisible(false)` が MeshRenderer.enabled と `suppressAsSource` を同時に立てる
  - **`PointCloudMotionCurves` のソース収集**: `IsUsableSource`（PointCloudMotionCurves.cs:673, :689）も可視性を見ない → **明示的な変更**: `IsUsableSource` に `suppressAsSource` チェックを追加して隠し live メッシュのシード混入を防ぐ（既定 false なので Dev モード挙動不変）
  - `MultiCameraDebugView`: live タイルは出続けてよい（オペレーター監視用、むしろ望ましい）
  - `CameraHealthMonitor`: live rig を監視し続ける（suppression 修正の目的そのもの）
  - `PointCloudCumulative` / shadow / `PointCloudView`: 体験モード中は director が off をスナップショット管理（Exit で復元）
  - USB/GPU 負荷は live 4 ストリーム+playback デコードの合算になる — Phase 6 で実測し、必要なら attract 中の live を depth のみ（color 停止）に落とすオプションを検討
- 入場: playback `StopAndUnload()` → live visual は非表示のまま（TSDF メッシュが主役）→ k4abt worker を restart（録画クロック→ライブクロックのジャンプで loop-seam ガードを踏まないため）。~1-2秒の skeleton 空白は Welcome メッセージで隠れる
- 2人以上（体験中）: `PersonCount>=2` で「じゅうたんのうえは　ひとりだけ　にしてね」を白文字オーバーレイ（体験は継続、既存デバウンス値流用）。アトラクト中の人数カウントは BT 不可のため対象外（占有ブロブ分割は保留）

## 失敗・キャンセル経路（Exporting / QrShow）

- **アップロード失敗/タイムアウト（60s+1リトライ後）**: 来場者向けには「うまくいかなかったみたい　ごめんね」を5秒表示して Attract へ。オペレーター HUD にエラー詳細ログ。書き出し済み glb/usdz ローカルファイルは**保持**（後日手動アップロード可能に、`~/Documents/FloatingVectorsPrints` に日時名で残す）。書き込み途中で失敗した部分ファイルは削除
- **Exporting 中に体験モード off / Fault**: `CancellationTokenSource.Cancel()` → Process kill → 部分ファイル削除 → 通常の Exit 復元処理。upload タスクは fire-and-forget にせず director が保持して必ず observe（UnobservedTaskException 回避）
- **QR 表示中の Fault**: 赤アラートが QR に優先

## 実装フェーズと検証（Unity MCP で AI 自身が確認、CLAUDE.md 準拠）

1. **Snapshot builder 抽出（最高リスク先行）** — 検証: 録画再生を一時停止し新旧 export の tri/vert 数とファイルサイズ一致、`Capture`×3 + 表示GO 3体のレンダリング、live volume 継続を RunCommand で assert。MC ヒッチ実測
2. **World rebase + センシングエリア** — EditMode テスト + 実 extrinsics.yaml で `_Playback_*` 重心≈原点、X軸方向、BoundingVolume/グリッド整列を assert
3. **UI + ディスプレイ + HUD 切替** — 各 display のスクリーンショット、フォントロードログ確認
4. **Presence + DwellSphere** — 録画 body の手の軌道上に球を置いて dwell 成立を assert、占有カウンタ readback > 0
5. **ステートマシン E2E（dry）** — `debugForcePresence` で全遷移、3キャプチャ、dryRun publisher、QR テクスチャ非null。モードを途中で off → 全スナップショット値の復元を assert
6. **アトラクト共存 + watchdog（Windows 実機）** — playback 中に live 4本ストリーム維持、USB 抜きで全画面赤アラート + Fault、入場遷移 <2秒
7. **アップロード + 仕上げ（Windows）** — 実 upload.ps1（token は config、コミットしない）、Bloom 判断、文言・タイミング調整、ソークテスト

## メモリ更新（プラン承認後すぐ）

`~/.claude/projects/.../memory/` に以下を保存し MEMORY.md に索引追加:
- `experience-flow-spec.md`（project）— 確定した体験フロー仕様・画面構成・確定済み判断（15フレーム=静的軌跡長、手タッチ、3画面構成、全画面赤アラート等）
- `lfks-upload.md`（reference）— LFKS アップロードの叩き方・downloadUrl 契約・token 期限(2026-10-30)・guide の所在

## リスク / 未決事項

- **QR にどの URL を載せるか**（usdz? glb? ランディングページ?）— config enum `{Usdz, Glb, First}` で保留、LFKS 側の契約確認待ち
- Exporting 中の表示文言は仕様に無い →「いま　じゅんびしているよ　まってね」を仮置き
- Yu Gothic Medium のフォント名マッチ（"Yu Gothic Medium" vs "游ゴシック Medium"）— 起動時に `Font.GetOSInstalledFontNames()` を probe してログ
- キャリブのグローバル座標が床基準 Y-up でない場合、rebase に床平面入力の追加が必要（実機 extrinsics で確認）
- 占有閾値のロバスト性（絨毯反射、縁で覗き込む保護者）— マージン inset + y バンドを config 化して現地調整
