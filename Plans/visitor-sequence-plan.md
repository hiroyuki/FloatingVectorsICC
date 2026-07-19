# 1 秒撮影ビジターシーケンス（2026-07-18 改訂 = Phase G）

「ばんざいで良い瞬間を選ぶ」方式を廃止し、**カウントダウン → 0 から 1 秒間の
動きがそのまま最終作品**になるフローに改訂。ここは運用者向け要約。

## シーケンス

1. **Idle** — 無人時は**床グリッドのみ**（ゴースト再生・誘導テキストは廃止）。
   ライブがソースなので、人が入ればその場で彫刻が現れる。presence 検知で開始
2. **Calibrate**（10 秒）— 開始 SE + 大の字シルエット「この ポーズを とってね」。
   大の字を 0.5 秒ホールドすると **per-camera 生骨格から骨長プロファイルを計測**
   （= **BT 精度向上**。ライブ融合に即適用 + 後段の v11s 変換にも使用）。
   タイムアウト時は既定プロファイルで続行
3. **FreeMove**（25 秒）—「すきに うごいてみよう！」。個人適応済みライブ彫刻で自由に動く
4. **Shoot** —「これから うごきを さつえいするよ」（2.5 秒）→
   **カウントダウン 3・2・1**（開始と同時に裏で収録開始 = 融合ウォームアップ）→
   **0（シャッター SE）→ 1 秒間が本番**「さつえいちゅう！」→ 収録停止（計 ~4 秒テイク）
5. **Processing** — v11s 変換（per-visitor プロファイル、数秒〜十数秒）→
   テイクを 1 回だけ再生（loop OFF、終端で自動停止）→ **最後の 1 秒窓
   （historySamples=30）で curved line を capture**。プログレスバー表示
6. **ResultShow** —「できたよ！」。画面は capture した瞬間で凍結
   （= 書き出される形そのもの）。裏で glb/usdz 書き出し → LFKS アップロード。
   最低 5 秒表示
7. **QrShow** —「いりぐちの にじげんコードを スキャンしてね」+ QR（30 秒 or 離脱）
8. → Idle（次の人へ）

## 主要コンポーネント

| 役割 | クラス |
|---|---|
| 状態遷移 | `ExperienceStateMachine`（EditMode テスト有） |
| 進行役 | `ExperienceDirector` |
| 骨長プロファイル計測 | `LiveFusedBodySource.Begin/EndBodyProfileSampling`（per-camera 生骨格、融合の長さ矯正を通る前の値） |
| ポーズ判定 | `PoseClassifiers`（大の字）+ `PoseHoldDetector`（純関数、テスト有） |
| v11s 変換 | `FusedTakeConverter`（runtime、worker スレッド、`ProfileOverride` 対応） |
| 変換共通部 | `FusedSnapshotEncoder`（export/live/converter の 3 者で共用） |
| UI | `VisitorMessageUI`（ShowPoseGuide / ShowProgress）+ `StickFigureTexture`（シルエット） |
| k4abt フォールバック | `LiveSkeletonFeed`（LFBS 不在時のみ自動スポーン） |

## ライブパイプラインとボディソース切替

ライブ骨格は **LiveFusedBodySource（RTMPose CUDA 融合、実測 30Hz）** が一次。
director が enabled な instance を自動検出し、コンテキストごとに切り替える
（`ApplyBodySource`）:

| コンテキスト | merger のソース | LFBS |
|---|---|---|
| Live（Idle/Calibrate/FreeMove/Shoot） | 融合 bodies（useExternalBodies） | submit ON |
| VisitorPlayback（Processing の再生〜QrShow） | テイクの v11s bodies | submit OFF・liveFramesOnly |

- 大の字検知は `SkeletonMerger.TryGetPrimarySkeleton`（ソース非依存）
- LFBS が死んだら k4abt 経路（LiveSkeletonFeed）に自動フォールバック、復活で自動退役
- Shoot 収録は external mode だと bodies_main を書かないが、v11s 変換は
  streams から生成するので問題ない
- `muteWorkerIngest` は「recorded bodies が merge を持つ」全期間 ON:
  worker 出力の ingest と、**merger 側の worker への供給/spawn の両方**を止める
  （凍結 ResultShow 中に live-idle 経路が worker を立ち上げるのを防ぐ）

## フォールバック連鎖

大の字不検知 → 既定プロファイル / 変換失敗・タイムアウト → テイク自身の bodies で
再生（無ければ k4abt 再解析）/ capture 失敗 → 謝罪表示 → Idle / アップロード失敗 →
ローカル保存 + 謝罪。どの経路でも来場者が dead-end しない。

## dev-sim（カメラ無し E2E）

skips（skipCalibrate/skipFreeMove/skipShoot + devCannedTakeRoot）+
`debugForcePresence` + `dryRunPublish` で**全自動**で 1 周する
（ばんざい debug hook は廃止 — capture は Processing 内で自動）。
canned take への実変換は `allowCannedTakeConversion`（使い捨てコピー推奨）。
