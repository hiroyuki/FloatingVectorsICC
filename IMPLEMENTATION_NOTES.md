## 実装概要
PointCloudRecorder の再生中ポーズ状態で Left/Right 矢印キーによる1フレーム単位のコマ送り/コマ戻し機能を追加。BT(K4abt worker) の inference と reconstructed point cloud mesh が再生時と同じパスで再発火するので、生成 mesh と収録 point cloud をフレーム単位で詳細に確認できる。複数カメラを跨いだステップは all-or-nothing (どれかのトラックが端なら全体が動かない) で、混在 timestamp による検査困難を回避する。Inspector にも ◀Step / Step▶ ボタンを追加した。

## 追加ファイル
- なし

## 修正ファイル
- `Assets/Scripts/PointCloud/PointCloudRecorder.cs`:
  - `StepForward()` / `StepBackward()` public API + ContextMenu を追加。再生中でなければ no-op。再生中で未ポーズなら自動的に `PausePlayback()` を先に呼ぶ。
  - 内部実装 `StepCursor(int delta)`: 全 non-empty トラックが範囲内に移動できるか先に precheck し、できなければステップしない (どのトラックが端にあるかを `StatusMessage` に出す)。OK なら全トラックの `PlaybackCursor` を delta だけシフトして `SetCursorAndEmit` で reconstruct + `OnPlaybackRawFrame` 発火 + bbox filter を実行 (=通常再生で playhead が当該フレームを越えた時と同じ処理)。
  - 内部実装 `SyncWallClockTo(ulong playheadNs)`: ステップ後、`_playbackWallStart` を再計算して resume したときに新しい playhead から続きを再生できるようにする。`(playheadNs - _playbackTrackStartNs)` の ulong underflow を防ぐためのガード入り。
  - playhead anchor 計算: `[maxSteppedTs, minNextTs)` 範囲内に置くことで resume 時の Update() auto-advance が次フレームを silently スキップしないようにする。range が empty な pathological ケース (混在 framerate) では `minNextTs-1` (0 ガード付き) にフォールバックして auto-advance を抑制する。下限は `_playbackTrackStartNs`。
  - `Update()` 内の入力処理に `Input.GetKeyDown(KeyCode.RightArrow)` → `StepForward()` と `Input.GetKeyDown(KeyCode.LeftArrow)` → `StepBackward()` を追加。既存の Space=TogglePause はそのまま。
- `Assets/Scripts/PointCloud/Editor/PointCloudRecorderEditor.cs`:
  - 既存の Rec/Play/Pause ボタン行の下に `◀ Step` / `Step ▶` ボタン行を追加。`CurrentState == Playing` のときだけ有効。

## 追加 GameObject (Hierarchy)
- なし

## 修正 GameObject (Hierarchy)
- なし (既存 `_Playback_<serial>` GO に対する mesh 更新は既存パス経由で起きる)

## 動作確認方法
1. Editor で playback シーンを開き、`PointCloudRecorder` がある GameObject の Inspector で **Read** して recording をロードする (例: persistentDataPath/Recordings/<dataset>)
2. **Play** を押して再生開始 → Game View に point cloud が表示されることを確認
3. **Space** キーで pause (Inspector の State が Playing、IsPaused=true、StatusMessage が "Playback paused.")
4. **→ (Right Arrow)** を押す: 全カメラの cursor が1フレーム進み、reconstructed mesh と BT skeleton/trail が更新される。StatusMessage に `Stepped → to N.NNNs (paused)` と表示
5. **← (Left Arrow)** を押す: cursor が1フレーム戻り、同様に mesh / BT が更新される。StatusMessage に `Stepped ← to N.NNNs (paused)`
6. Inspector の **◀ Step / Step ▶** ボタンでも同じ動作になる
7. 端 (最後のフレーム) で → を押すと StatusMessage が `Step forward: track <serial> at last frame (N/N).` になり、cursor は動かない (最初のフレームで ← も同様に "at first frame.")
8. ステップ後に Space で **Resume**: 停止していた位置から再生が再開され、いきなり次フレーム以降にジャンプしない (anchor 設計により Update() auto-advance が抑制されている)

失敗時の典型症状:
- Step しても point cloud / BT skeleton が更新されない → そもそも IsPaused が立っていない / `CurrentState != Playing` / OnPlaybackRawFrame 購読者が居ない (BodyTrackingMultiLive が disabled になっている可能性)
- Step 後 Resume したら大量のフレームを飛ばす → playbackRate が極端に大きいか、recording のトラック間 timestamp が極端にずれている可能性 (anchor 計算は最良努力)
- Editor Console に "Step forward: track ... at last frame" が出るが意図と違う → 特定カメラのみフレーム数が少ない recording なので、その境界以降は all-or-nothing で進めない (仕様)
