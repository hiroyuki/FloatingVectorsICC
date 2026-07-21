# 床面3点クリック水平化（floor plane leveling）— Windows 実機確認手順

**ブランチ**: `worktree-floor-plane-level`（`origin` に push 済み, commit `5db5e1d`）
**このドキュメント**: Mac 側でコンパイルまでしか確認できないため、Windows 実機での検証項目をまとめたもの。

## 概要（何が変わったか）

床チューン中に **ライブ点群の床を3点以上クリック → 最小二乗で平面フィット →
法線を +Y に回して高さ0** に水平化する機能を追加。従来は高さ（`rebaseFloorY`）
のみ調整可能だったが、これで**傾き（ピッチ/ロール）補正**も可能になった。

- 傾き補正はカメラ由来の `WorldFrameRebase` に手を入れず、**ワールド空間の後段補正
  Pose** として rebase の後に左合成（localScale の Y反転契約は不変）。
- `calibration/floor.yaml` に `rebase_floor_y` と並べて
  `floor_leveling_pos` / `floor_leveling_rot`（クォータニオン）を保存。
  旧形式（`rebase_floor_y` のみ）は identity として後方互換読み込み。
- ピッキングは `AsyncGPUReadback` で GPU 頂点バッファを読み戻す。**ライブ点群のみ対象**。

## Windows での取得

```
git fetch origin
git checkout worktree-floor-plane-level
```

Unity で開くと新規2スクリプト（`FloorPlaneMath.cs` / `FloorPointPicker.cs`）の
`.meta` が生成される。

## 操作（床チューン中）

| キー / 操作 | 動作 |
|---|---|
| `G` | 床チューンの ON/OFF（`floorTuneKey`） |
| `↑` / `↓` | 高さ（`rebaseFloorY`）を上下、`Shift` で 1/10 ステップ |
| 左クリック | 床の点をピック（≥3点） |
| `Backspace` | 直前のピックを取消 |
| `Enter` | ピック点から平面フィット → 水平化 & y=0 を適用 |
| `R` | 傾き補正をリセット（identity に戻す） |
| `G` / `Esc` | 床チューンを抜ける |

HUD に `click floor ×N/3`（ピック数）と現在の `y` が表示され、適用時に
「補正した傾き角」がステータスに出る。ピック点は黄（既定マテリアル）の球で可視化。

## 確認ポイント

1. **コンパイルエラーが無いこと**（Unity コンソール）
2. Play → `G` で床チューンに入る
3. ライブ点群の**床を3点以上クリック** → `Enter` で水平化 & y=0
4. `SensorManager.applyWorldRebase` が **ON** であること
   （OFF だと「levelling has no effect」警告が出て無効）
5. `calibration/floor.yaml` に `floor_leveling_pos/rot` が保存され、
   **再起動後も復元**されること
6. 反復ピック（適用後にもう一度3点）で残差が減り、**収束**すること
7. `R` で傾きが水平（identity）に戻ること

## 特に見てほしい所（Mac で検証できなかった部分）

- **クリックのピッキング**が実機で効くか。`AsyncGPUReadback` 依存の**唯一の未検証箇所**。
  - 反応しない / ズレる場合の一次対処: `CalibrationRuntimeUI` の
    `floorPickRadiusPx`（既定 12px）を大きくする。
  - それでも駄目なら readback 経路（`mesh.GetVertexBuffer(0)` の有効性、頂点ストライド
    24byte / position offset 0 の前提）を見直す。フィードバックをもらえれば対応する。

## スコープ / 既知の制約

- **ライブ経路（`SensorManager`）のみ適用**。ピッカーもライブ点群のみ対象。
- **playback の床水平化は対象外**（playback は元々 `rebaseFloorY` / `floor.yaml` を
  読まない挙動。ピッカーを live-only にしてスコープを揃えた ← Codex レビュー指摘対応）。
- 実運用（Windows・4台ライブ）はこの経路なので問題なし。

## レビュー状況

- Codex レビュー（gpt-5.5）で **2ラウンド → APPROVED**。
  - Round 1: live/playback 不整合（BLOCKING）+ 無効深度センチネルのガード2件を指摘 → 全対応。
  - Round 2: 指摘なし・承認。
- **main へのマージは保留中**（CLAUDE.md「main マージ前に Unity コンソールで
  コンパイルエラーゼロを必ず確認」ルールを Mac では満たせないため）。
  Windows でコンパイル確認が取れ次第、`.meta` 追加コミット → main マージを行う。

## 変更ファイル

- 新規 `Assets/Scripts/Calibration/FloorPlaneMath.cs` — 平面フィット / 水平化 Pose / Pose 合成（純粋計算）
- 新規 `Assets/Scripts/PointCloud/FloorPointPicker.cs` — GPU 点群のクリックピッキング（ライブのみ）
- `Assets/Scripts/PointCloud/PointCloudRecording.cs` — `WriteFloor` / `TryReadFloor`（leveling 対応・後方互換）
- `Assets/Scripts/PointCloud/SensorManager.cs` — `rebaseFloorLeveling` 読込 & rebase への合成
- `Assets/Scripts/Calibration/RuntimeUI/CalibrationRuntimeUI.cs` — 3点ピック UI / マーカー / 保存
