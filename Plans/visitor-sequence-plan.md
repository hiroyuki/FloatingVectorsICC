# 1 秒撮影ビジターシーケンス（2026-07-23 改訂 = 練習 2 回 + プレイバック提示）

「ばんざいで良い瞬間を選ぶ」方式を廃止し、**カウントダウン → 0 から 1 秒間の
動きがそのまま最終作品**になるフローに改訂。2026-07-23 に FreeMove を廃止して
**練習 2 回（TestMove1/2）**に置き換え、結果提示を**ライン+点群のループ再生 +
自動 orbit カメラ**に変更（TSDF メッシュはモード中ずっと非表示。draw のみ抑制で
capture/export は従来どおりメッシュを使う）。ここは運用者向け要約。

## シーケンス

1. **Idle** — 無人時は**床グリッドのみ**。人が入るとライブ点群がそのまま見える。
   presence 検知で開始
2. **Calibrate**（10 秒）— 開始 SE + 大の字シルエット「この ポーズを とってね」。
   大の字を 0.5 秒ホールドすると **per-camera 生骨格から骨長プロファイルを計測**
   （= **BT 精度向上**。ライブ融合に即適用 + 後段の v11s 変換にも使用）。
   タイムアウト時も窓の間の収集分からベストエフォートでプロファイルを作って続行(骨長はポーズ不変。サンプル不足時のみ既定)
3. **TestMove1**（練習 1、director 所有・TestMoveDone で進行）—
   「じぶんの うごきを とるれんしゅうを しよう」→「５びょうたったら
   １びょうかん とるよ たくさん うごいてね」（各 testIntroSeconds=4s）→ 画面クリア →
   **カウントダウン 5**（上に「ヒント すきな どうぶつに なってみよう」）→
   **1 秒収録** → **v11s 変換**（本番と同一パイプライン、プログレスバー）→
   「できたよ！」+ **ライン付き 1 秒を 3 回ループ再生**（点群+ライン、
   visitor カメラ 2 面が自動 orbit）→ ライブに復帰。**アップロードなし**
4. **TestMove2**（練習 2）—「どうだった？ もういちど れんしゅうするよ」→
   以降は練習 1 と同じ（ヒントは「すきな キャラクターに なってみよう」）
5. **Shoot（ほんばん）** —「じゃぁ ほんばんだよ」（2.5 秒）→
   **カウントダウン 5**（上に「じぶんの すきな うごきを してみよう」）→
   **0（シャッター SE）→ 1 秒間が本番**「さつえいちゅう！」→ 収録停止
6. **Processing** — v11s 変換（per-visitor プロファイル、数秒〜十数秒）→
   テイクを 1 回だけ再生（loop OFF、終端で自動停止）→ **最後の 1 秒窓
   （historySamples=30）で curved line を capture**。プログレスバー表示
7. **ResultShow** —「できたよ！」+ **テイクを 3 回ループ再生**（ライン+点群、
   orbit）。最終ループはテイク終端で停止＝**ラインの止まったモデルで凍結**。
   裏で glb/usdz 書き出し → LFKS アップロード。QR は再生 3 回 + Export 完了 +
   最低 5 秒がすべて揃ってから
8. **QrShow** — **QR は画面右**「にじげんコードの しゃしんを とったら
   おうちでも みれるよ」。中央は凍結したラインモデル（orbit 継続）（30 秒 or 離脱）
9. → Idle（次の人へ。orbit カメラは元の構図に自動復帰）

## dev ジャンプ（数字キー / Experience Dev Panel）

0=Idle(頭出し) 1=Consent 2=Welcome 3=Calibrate 4=TestMove1 5=TestMove2
6=Shoot 7=Processing 8=ResultShow 9=QrShow。skipFreeMove は **skipTestMoves**
に改名（両練習をスキップ）。

## rig-less（Mac / 収録再生）dev ループ

- **presence 監視は OFF**（ライブ renderer 0台なら `ComputePresent` が常に true。
  トグル無しの確定仕様 — 退場フローの検証は実機で行う）
- **タップ収録**: dummyShoot でも入場再生が流れていれば、カウントダウン0からの
  1秒は `SensorRecorder.StartPlaybackTapRecording` で**再生ストリームから本当に
  収録**される（本番と同じ長さのテイク・bodies_v11s はパススルー・キャリブは
  ソーステイクからコピー）。タップ済みテイクは **v11s 変換をスキップ**（既に
  変換済みデータの再収録のため）。再生が無い時だけ缶詰テイク
  （devCannedTakeRoot、末尾 devCannedTakeTailSeconds のみループ）で代用
- タップ収録の保存先: 通常の録画先（folderPathMacOverride）に書けない場合は
  `persistentDataPath/VisitorTakes/` へフォールバック
- **注意: 1周あたり練習2回+本番= 約1.8GB 書き込む**。dev ループを回しっぱなしに
  すると VisitorTakes がディスクを食い潰すので、定期的に削除すること

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
