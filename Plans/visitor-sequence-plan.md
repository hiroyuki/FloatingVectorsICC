# ポーズ駆動ビジターシーケンス（2026-07-17 実装）

旧「3 回スナップショット + 球選択」フローを置き換えた新シーケンスの設計メモ。
実装プランの全文は開発セッションのプランファイル参照。ここは運用者向け要約。

## シーケンス

1. **Attract** — ゴースト再生（従来どおり）。presence 検知で開始
2. **Calibrate**（10 秒）— 開始 SE + 大の字ガイド表示「この ポーズを とってね」。
   スケルトンが大の字を 0.5 秒ホールドすると手足の長さ等を計測
   （**ポーズ検知しきい値の個人適応**に使用。ばんざい判定マージンが腕長に比例）。
   タイムアウト時はデフォルト値で続行
3. **Explore**（10 秒）—「おもしろい うごきを さがしてみよう」→ SensorRecorder で
   raw 収録（visitorRecordingRoot 配下にタイムスタンプフォルダ）。ラスト 3 秒カウントダウン
4. **Processing** — FusedTakeConverter が worker スレッドで v11s 変換
   （RTMPose 融合 → bodies_main を in-place 置換、.k4abt バックアップ）。
   プログレスバー表示。失敗/タイムアウト（90 秒）は k4abt bodies のまま再生続行
5. **Watch** — 変換済みテイクをループ再生（1 周目 = 鑑賞）。
   merger は recorded bodies を消費（ignoreRecordedBodies=false + muteWorkerIngest）、
   live は sculpture ソースから suppress、k4abt worker は LiveSkeletonFeed が
   live フレームで回す（ばんざい検知用）
6. **BanzaiWait** — ループ継続 +「ばんざいの ポーズを してね」。
   ばんざい 0.4 秒ホールド → その再生時点で TSDFSnapshot capture。
   3 ループ経過ならランダム地点にシークして同処理
7. **Exporting** — glb/usdz 書き出し + LFKS アップロード（再生はループ継続）
8. **QrShow** —「いりぐちの にじげんコードを スキャンしてね」+ QR（30 秒 or 離脱）
9. → Attract（次の人へ）

## 主要コンポーネント

| 役割 | クラス |
|---|---|
| 状態遷移 | `ExperienceStateMachine`（EditMode テスト有） |
| 進行役 | `ExperienceDirector` |
| ライブ全関節骨格 | `LiveSkeletonFeed`（merger のフレーム所有権の正確な補集合で worker に供給） |
| ポーズ判定 | `PoseClassifiers` + `PoseHoldDetector`（純関数、テスト有） |
| v11s 変換 | `FusedTakeConverter`（runtime、worker スレッド） |
| 変換共通部 | `FusedSnapshotEncoder`（export/live/converter の 3 者で共用） |
| UI | `VisitorMessageUI`（ShowPoseGuide / ShowProgress 追加）+ `StickFigureTexture` |

## 排他制御の要点

- **k4abt worker のフレーム所有権**: merger は「再生中 && そのテイクに serial がある」
  ならライブフレームを流さない（SkeletonMerger.cs:893）。LiveSkeletonFeed はその
  正確な補集合（かつ ignoreRecordedBodies=false のとき）だけ流す → 常にフィーダは 1 つ
- **merger 汚染防止**: 再生中の live worker 結果は playhead-ahead ガードで落ちる +
  `muteWorkerIngest` で明示ミュート
- **Attract ゴーストとの区別**: attract 中（ignoreRecordedBodies=true）の worker 出力は
  録画由来なので LiveSkeletonFeed は ingest しない

## フォールバック連鎖

ポーズ不検知 → デフォルト計測値 / 変換失敗 → k4abt bodies / ばんざい不検知 →
ランダム地点 / capture 失敗 → Exporting が謝罪表示 → Attract。どの経路でも
来場者が dead-end しない。
