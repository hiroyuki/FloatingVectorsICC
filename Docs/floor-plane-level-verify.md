# 床面水平化（floor plane leveling）— 実機検証記録

> **2026-07-21 更新: 3点クリックは廃止され、全点最小二乗フィットに置き換わった。**
> 操作は「床チューン中に `Enter`」のみ。詳細は末尾の「全点フィットへの移行」を参照。
> 以下 §「操作」の3点ピック記述は歴史的経緯として残す。

**状態**: Windows 実機（5080 セット / Femto Bolt 4台ライブ）で全項目検証済み → `main` にマージ済み。
**このドキュメント**: 元は Mac 側で書いた「Windows でここを見てほしい」チェックリスト。
実機検証で**設計上の欠陥が2件見つかり修正**したため、結果と最終仕様に書き換えたもの。

## 概要（何ができるか）

床チューン中に **ライブ点群の床を3点以上クリック → 最小二乗で平面フィット →
法線を +Y に回して高さ0** に水平化する。従来は高さ（`rebaseFloorY`）のみ調整可能だったが、
これで**傾き（ピッチ/ロール）補正**も可能。

- 傾き補正はカメラ由来の `WorldFrameRebase` に手を入れず、**ワールド空間の後段補正
  Pose** として rebase の後に左合成（localScale の Y反転契約は不変）。
- `calibration/floor.yaml` に `rebase_floor_y` と並べて
  `floor_leveling_pos` / `floor_leveling_rot`（クォータニオン）を保存。
  旧形式（`rebase_floor_y` のみ）は identity として後方互換読み込み。
- ピッキングは `AsyncGPUReadback` で GPU 頂点バッファを読み戻す。**ライブ点群のみ対象**。

## 操作（床チューン中）

| キー / 操作 | 動作 |
|---|---|
| `G` | 床チューンの ON/OFF（`floorTuneKey`）。Display 1 に俯瞰カメラが立つ |
| 左クリック | 床の点をピック（≥3点）。**Display 1 の画面で**クリックする |
| `Enter` | ピック点から平面フィット → 水平化 & y=0 を適用（**傾きと高さを同時に**） |
| `Backspace` | 直前のピックを取消 |
| `R` | 傾き補正をリセット（identity に戻す） |
| `↑` / `↓` | 高さのみ微調整、`Shift` で 1/10 ステップ。**`↑` で床が下がる**（下記） |
| `G` / `Esc` | 床チューンを抜ける |

HUD に `click floor ×N/3`（ピック数）と現在の `y` が表示され、適用時に
「補正した傾き角」がステータスに出る。ピック点は球で可視化。

### `↑` / `↓` の向きは見た目と逆

`rebaseFloorY` は「キャリブ座標系のどの Y を新しい y=0 にするか」というレバーなので、
値を上げると**世界が下がる**。実測：

```
rebaseFloorY  -0.9000  →  床の平均 y =  0.0009
rebaseFloorY  -0.8500  →  床の平均 y = -0.0491     ← +0.05 したら床が 0.05 下がった
```

内部の意味と一致させるため反転はせず、HUD を `↑ floor down / ↓ up` と明示する方針にした。
そもそも 3点ピックが傾きと高さを同時に合わせるので、**アローキーは追い込み用**。

## 実機で見つかった欠陥と修正

チェックリストの「特に見てほしい所」は `AsyncGPUReadback` が効くかだったが、
readback 自体は初回から動作した（9/9 ヒット、1ピックあたり約 78ms）。
**実際に壊れていたのは別の2点**。

### 1. ピッカーが表示範囲を無視していた（真因）

床チューンは `filterMode = KeepInside` でセンシングボックス内だけを表示するが、
`FloorPointPicker` は生の頂点バッファを読むため、**画面に出ていない壁・天井・遠方のゴミまで
候補**だった。`TryPick` は「カーソル下で一番手前」を返すので、床をクリックしたつもりが
手前の不可視の壁を掴む。

結果、フィットされた平面がほぼ垂直になり、**82°のロールが適用されて `floor.yaml` にも保存**
されるという破壊的な事故が起きた。

```
掴んでいた点   (-2.49, 3.06, 0.50) (-2.50, 3.54, 0.31) (-2.51, 3.11, -0.39)
               x が -2.5 で固定 = 壁。box の X 範囲 [-1.62, 1.58] の外
適用された補正  euler (0.21, 0.18, 81.93)   up (-0.990, 0.140, 0.004)
```

→ `TryPick` に `restrictTo`（`BoundingVolume`）を追加し、**表示に使っている OBB 内のみ**
を候補にした。

### 2. カメラが水平・目線で床が見えなかった

`Main Camera` は水平を向いており、床は視線に対してほぼ平行で「線」にしか見えない。
どのレイも壁や人体に先に当たる。

→ 床チューン中だけ `_FloorTuneCamera` を Display 1 に立て、`floorTunePitchDeg`（既定 55°）
で俯瞰する。**既存カメラは動かさない** — `Main Camera` の `CameraOrbitController` が
auto-orbit で毎フレーム姿勢を奪い返すため（実測で pos が勝手に動いていた）。
Display 2 も来場者UIも無傷。

### 安全弁

法線が水平から `floorMaxTiltDeg`（既定 40°）以上ずれたフィットは**適用を拒否**する。
上記の壁の3点を流し込んだ結果：

```
status = "fitted plane is 88.7° off horizontal (limit 40°) — those picks are on
          a wall, not the floor. Not applied."
leveling は変化なし
```

### なぜマウス座標がディスプレイに依存するのか

クリックは 2D 座標でしかなく、ワールド座標に落とすには**どのカメラで投影するか**が要る。
`Input.mousePosition` は単一のディスプレイ座標系（プライマリ＝Display 1）しか話さないのに、
ステージカメラは Display 2 に描いている。座標を変換するのではなく、**クリックする画面と
投影に使うカメラを同一にする**（＝ Display 1 に専用カメラを立てる）ことで構造的に解決した。

## 検証結果（5080 セット / Femto Bolt 4台ライブ）

| 項目 | 結果 |
|---|---|
| コンパイルエラー | 0 |
| `AsyncGPUReadback` ピッキング | 動作。1ピック約 78ms（4台 × 92,160 点） |
| `SensorManager.applyWorldRebase` | ON |
| 平面フィット精度（合成データ） | 法線を完全一致で復元、残差 Y = 0.000000 |
| `floor.yaml` round-trip | 一致。旧形式も identity として読み込み |
| 傾きすぎフィットの拒否 | 88.7° を拒否、leveling 不変 |
| 終了時の後始末 | `_FloorTuneCamera` 破棄 / `PickCameraOverride` クリア / `Main Camera` 復帰 / キャンバス復帰 / ゲート解除 |

### 適用結果（操作者が実際にクリックして適用）

```
status   "Floor levelled (corrected 0.2° tilt)   (saved to floor.yaml)"
leveling pos   (-0.001, -0.922,  0.003)
         euler (359.91,  0.00, 359.78)     ← 実質 identity
         up    ( 0.0039, 1.0000, -0.0016)  ← 鉛直から 0.24°
```

適用後の床の高さ（ボックス内 14,472 点、5cm ビン）：

```
  y=-0.05 : 6,866  ████████████
  y= 0.00 : 7,168  █████████████   ← 14,034 点が y=0 をまたいで乗った
  y= 0.15 :    16
  ...（以降は人・什器・天井の疎な点）

床帯（|y| < 0.15）  n = 14,034   平均 y = 0.0006 m   標準偏差 0.0063 m
```

**平均 0.6mm / ばらつき ±6mm。** 6mm は Femto Bolt の深度ノイズそのもので、これ以上は詰められない。

なお床は元々 **y ≈ +0.9 に浮いていた**（`rebaseFloorY = -0.9` が逆向きに効いていた）。
今回の適用でそれも同時に 0 へ落ちている。

## スコープ / 既知の制約

- **ライブ経路（`SensorManager`）のみ適用**。ピッカーもライブ点群のみ対象。
- **playback の床水平化は対象外**（playback は元々 `rebaseFloorY` / `floor.yaml` を
  読まない挙動。ピッカーを live-only にしてスコープを揃えた）。
- 実運用（Windows・4台ライブ）はこの経路なので問題なし。
- `CameraHealthMonitor`（カメラ故障アラート）は床チューン中も表示したまま。
  故障を隠すべきでないという判断。邪魔なら `OperatorOverlayGate.Suppressed` を
  見るように変えられる。

## ディスプレイ番号は 1オリジン

**呼称・コメント・ツールチップは常に 1オリジン**（`Display 1` = 操作者画面）。
Unity の `targetDisplay` API は 0 始まりなので `Display N == targetDisplay N-1`。
API 値を書く箇所は必ず両方を併記する。

| GameObject | `targetDisplay` | 呼称 |
|---|---|---|
| `Display1 Black` | 0 | Display 1（操作者 HUD） |
| `Main Camera` | 1 | Display 2 |
| `Display3 Camera` | 2 | Display 3 |

`Display2 Camera` は実体が Display 3 だったため `Display3 Camera` にリネーム済み。
`_VisitorUI_Display{N}` の生成名も 1オリジンに揃えた。

## レビュー状況

- Mac 実装時: Codex レビュー **2ラウンド → APPROVED**
  （live/playback 不整合の BLOCKING + 無効深度センチネルのガード2件を対応）。
- Windows 実機修正後: Codex レビュー **1ラウンド → APPROVED**（指摘なし）。
  床チューンの enter/exit ライフサイクル（一時カメラ、static フラグ2つ、Canvas トグル）の
  全 teardown 経路がカバーされていることを確認済み。
- `main` にマージ済み（`a0a5c35`）。

## 変更ファイル

- 新規 `Assets/Scripts/Calibration/FloorPlaneMath.cs` — 平面フィット / 水平化 Pose / Pose 合成（純粋計算）
- 新規 `Assets/Scripts/PointCloud/FloorPointPicker.cs` — GPU 点群のクリックピッキング（ライブのみ、`restrictTo` で表示範囲に限定）
- `Assets/Scripts/PointCloud/PointCloudRecording.cs` — `WriteFloor` / `TryReadFloor`（leveling 対応・後方互換）
- `Assets/Scripts/PointCloud/SensorManager.cs` — `rebaseFloorLeveling` 読込 & rebase への合成
- `Assets/Scripts/Calibration/RuntimeUI/CalibrationRuntimeUI.cs` — 3点ピック UI / 俯瞰カメラ / 傾き拒否 / マーカー / 保存
- `Assets/Scripts/Shared/OperatorOverlayGate.cs` — `FloorTuneActive` + `Suppressed`
- `Assets/Scripts/CameraControl/Display1OperatorHud.cs`, `Assets/Scripts/PointCloud/MultiCameraDebugView.cs` — 床チューン中は降りる
- `Assets/main.unity` — `Display2 Camera` → `Display3 Camera`

## 全点フィットへの移行（2026-07-21、4070 セット実機）

3点クリックは**廃止**。床チューン中の `Enter` が、見えている床の点を全部使った
最小二乗フィットで傾きと高さを同時に決めるようになった。
`FloorPointPicker.cs` は削除、`FloorCloudFitter.cs` が置き換える。

### 廃止の理由（4070 実機で観測）

- クリックが空振りする（`no cloud point under the cursor` が連発）
- 3点では点の選び方に結果が左右される。壁を掴む事故は `restrictTo` で塞いだが、
  そもそも「どこを3回押すか」が精度を決めてしまう構造が残っていた
- **アローキーの毎押し保存が事故になった**: `rebaseFloorY` が 0.01 刻みで
  **-0.14 → -1.63 まで約 1.4m** 歩き、その都度 `floor.yaml` に保存された。
  そこで Enter を押すと leveling が打ち消しの `-1.66m` を持ち、
  「2つの大きな値が打ち消し合う」壊れやすい状態が保存された

### 現仕様

| キー | 動作 |
|---|---|
| `G` | 床チューン ON/OFF（Display 1 に俯瞰カメラ） |
| `Enter` | **全点フィット** → 傾き＋高さを適用 → `floor.yaml` 保存 |
| `↑` / `↓` | 高さのみ微調整（`↑` で床が下がる）。保存は 0.75 秒デバウンス＋チューン退出時 |
| `G` / `Esc` | 退出（保留中の微調整はここで確定保存） |

- `R`（reset tilt）は**削除**。leveling は傾きだけでなく**高さも持つ**ため、
  identity に戻すと床が保持していた高さ分だけ飛ぶ（実測で最大 1.6m）。
  Enter が常に絶対値を出すので、identity を経由する必要がない
- **無人前提**。帯選択は靴と床を区別しないので、フィット中は誰もエリアに入らない運用とする

### アルゴリズム（`FloorCloudFitter.TryFit`）

1. **粗探索** — 全点の高さヒストグラム（5cm ビン）の最頻ビン → そのビン内の点の平均で床高さを確定
   （固定帯だと、床が y=+1.57 に飛んだ状態から復帰できない）
2. **帯フィット** — 床高さ ±`floorFitBandMeters`（0.12m）の点で平面フィット
3. **±2σ トリムで2回リフィット** — 巾木・ケーブル・スタンドの脚など
   **床より上にしか無い**構造による上方バイアスを除去
4. サンプル範囲は BoundingVolume の XZ から `floorFitInsetMeters`（0.4m）内側。
   隅は疎でノイジーな上、重心から遠く傾きへのてこが最大なので外す。
   **箱の回転は無視**（ワールド軸で切る）— 傾いた箱で切ると床が非対称に削られる

`CalibrationRuntimeUI.ApplyFloorLeveling` はこれを2パス回す。
1パス目でどこにあった床でも水平・y=0 に乗り、2パス目で残差を詰める。

### 実機結果（4070 セット / Femto Bolt 4台ライブ）

```
通常フィット      40,591 点  傾き 0.02°  mean -0.6mm  σ 7.1mm   36ms
壊れた状態から復帰  rebaseFloorY=-1.54 / 傾き2.7° → Enter 1回で mean -0.13mm / 傾き 0.028°  78ms
```

サンプル範囲の A/B（フィットとは独立した中央部計測で判定）:

| | inset 0m（箱いっぱい） | inset 0.4m（採用） |
|---|---|---|
| フィット点数 | 40,711 | 17,609 |
| フィット後の傾き | 0.000° | 0.000° |
| 独立計測 mean | +2.06 mm | -0.91 mm |
| 独立計測 σ | 8.35 mm | 8.36 mm |

深度ノイズ σ 8.4mm に対し平均差 1〜2mm で**統計的に区別できない**。
隅の異物を構造的に排除できる 0.4m を採用した。

カメラ別の床（同時刻、中央部 |x|,|z|<1.4m）:

| serial | 平均高さ | 傾き |
|---|---|---|
| CL8F253004L | +2.0 mm | 0.59° |
| CL8F25300EG | -0.8 mm | 0.65° |
| CL8F253004N | -4.9 mm | 0.88° |
| CL8F253004Z | -6.0 mm | 0.92° |

台間の高さ差は最大 8mm（ノイズ σ 6〜8mm と同程度）、傾きは向きがバラバラで
合成すると打ち消し合い全体では 0.000°。

### 「床が斜め」の誤診に注意

4070 実機で「床が斜め」に見えた原因は**床ではなく BoundingVolume だった**。
7/17 に傾いた床へ合わせるため箱を 1.88° 傾けてあり（`bounding-floor` 作業）、
床を世界側で水平化した後もその補正が残って二重補正になっていた。
床の点自体は 0.000° / mean ±2mm。**箱の回転は 0 が正**（`ExperienceSpaceBuilder` が
`Quaternion.identity` を必ず書く）。斜めに見えたらまず箱の rotation を疑う。
