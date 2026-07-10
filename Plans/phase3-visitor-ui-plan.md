# Phase 3 実装プラン: 来場者向け UI + 3ディスプレイ + HUD 切替（改訂1）

Parent plan: `Plans/experience-flow-plan.md`（Codex 承認済み。Phase 1,2 実装済み・main マージ済み）。

## 画面構成（確定仕様）

- **Display 0**（物理画面A・オペレーター）: `Display1 Black` カメラ（黒背景）。IMGUI の `Display1OperatorHud` と `MultiCameraDebugView`（4カメラタイル）が載る — IMGUI はビルドでは primary display のみに描画される既存前提
- **Display 1 / 2**（物理画面B/C・来場者）: `Main Camera` / `Display2 Camera` が彫刻を表示。この上に文言・カウントダウン・QR・赤アラートをオーバーレイ

## 新規ファイル

### 1. `Assets/Scripts/Experience/VisitorMessageUI.cs`（Experience asmdef、`UnityEngine.UI` 参照を asmdef に追加）

uGUI をランタイムで**プログラム生成**（シーン/プレハブ非依存。Phase 5 で ExperienceDirector が所有、Phase 3 検証は RunCommand で生成）。

- **Canvas 構成**: 来場者 display（既定 {1, 2}）ごとに 1 Canvas。`RenderMode.ScreenSpaceCamera` + その display の表示カメラを割当（ビルドではカメラの targetDisplay がそのまま routing、**Editor では `cam.Render()`→RenderTexture で UI 込みのスクリーンショットが撮れる** — Phase 2 で確立した検証手法を踏襲）。カメラ解決は `targetDisplay == n` の有効カメラを検索、無ければ warning + その display をスキップ。CanvasScaler = ScaleWithScreenSize(1920×1080)
- **オペレーター用アラート = IMGUI**（Codex 指摘1対応）: display 0 の HUD / 4カメラタイルは IMGUI で、IMGUI は uGUI Canvas の**上**に描画されるため display 0 の uGUI アラートは覆われうる。よって display 0 用の uGUI Canvas は作らず、**VisitorMessageUI 自身の OnGUI で赤アラートを IMGUI 描画**（primary display = display 0 に載る、`GUI.depth = -1000`）。加えて共有ゲートで他の IMGUI を抑制:
  - 新規 `Assets/Scripts/Shared/OperatorOverlayGate.cs`（Shared asmdef の static クラス。Experience→PointCloud の逆参照は循環になるため、全員が見える Shared に置く）: `public static bool AlertActive`
  - `ShowAlert` が true / `ClearAlert`・`ClearEverything`・`OnDestroy` が false をセット
  - `Display1OperatorHud.OnGUI` と `MultiCameraDebugView.OnGUI` は先頭で `if (OperatorOverlayGate.AlertActive) return;`（アラート中は HUD/タイルを描かない）
- **API**（全 display 同内容を一括制御）:
  - `ShowMessage(string text)` — 中央に白文字（フォントサイズ config）
  - `ShowCountdown(int seconds)` — 大きな数字（メッセージと同スロット、上書き）
  - `ShowAlert(string text)` — 赤文字 + 半透明黒バックドロップ、**最優先**（QR/メッセージより手前、オペレーター Canvas にも表示）。`ClearAlert()` で解除
  - `ShowQr(Texture2D qr, string caption)` — 中央に RawImage（テクスチャは呼び側で Point filter 済み）+ 下にキャプション。アラート表示中は裏に隠れる（優先順位は sibling order で保証）
  - `ClearAll()` — アラート以外を消す / `ClearEverything()` — アラート含め全消し
- **フォント**: `Font.GetOSInstalledFontNames()` を起動時に probe し、`fontCandidates`（既定 ["Yu Gothic Medium", "游ゴシック Medium", "Yu Gothic", "Hiragino Kaku Gothic ProN"]）から最初に見つかった名前で `Font.CreateDynamicFontFromOSFont`。見つからなければ `Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf")` にフォールバック + どれを選んだかログ（Mac 検証用）
- Canvas は `overrideSorting` 相当として `sortingOrder = 5000` を明示（Codex 指摘: 将来の別 Canvas に覆われない）
- テキストは wrap + RectTransform 内に収める（長い caption/文言でもレイアウト破壊しない。caption は 1 行 truncate）
- 生成 GO は `DontSave`、`OnDestroy` で破棄。Update 不要（静的表示、点滅等の演出はしない）

### 2. 既存ファイルの変更

- `CameraControl/MultiDisplayActivator.cs` — スクリプト既定値 `displayCount = 2 → 3`（シーンは既に 3 なので挙動不変。新規シーン用の既定を合わせるだけ）
- `PointCloud/MultiCameraDebugView.cs` — `public bool Visible { get => _visible; set => _visible = value; }` 公開（既存 private `_visible` を包む。既存トグルキーはそのまま — 排他は HUD 側の毎フレーム同期で担保）+ `OnGUI` 先頭に `if (Shared.OperatorOverlayGate.AlertActive) return;`（PointCloud asmdef は Shared を参照済み）
- `CameraControl/Display1OperatorHud.cs`（Assembly-CSharp、両 asmdef が見える）—
  - フィールド追加: `public MultiCameraDebugView debugView;`（OnEnable auto-resolve）、`public KeyCode viewSwitchKey = KeyCode.Tab;`、`public bool exclusiveWithDebugView = true;`
  - `Update()` 追加: viewSwitchKey は `debugView.Visible` だけをトグル。**排他は single source of truth**（Codex 指摘2対応）: `exclusiveWithDebugView && debugView != null` のとき毎フレーム `visible = !debugView.Visible` に同期 — debugView 側の既存トグルキー（backquote）で切り替えても排他が保たれる。debugView 未解決時は viewSwitchKey で `visible` を直接トグル
  - `OnGUI` 先頭に `if (Shared.OperatorOverlayGate.AlertActive) return;`
  - 既存の HUD 内容・Auto Orbit・Export ボタンは無変更

## 実機検証（Unity MCP、AI 自身が実施）

1. コンパイル + 各表示カメラの targetDisplay 検証（Display1 Black=0 / Main Camera=1 / Display2 Camera=2、シーン値を RunCommand で assert）
2. Play 中に VisitorMessageUI を生成 → `ShowMessage("こんにちは")` → Display1/2 両カメラを RT に明示レンダリングし、文言が両画面に出ることをスクリーンショット確認（Phase 2 で確立した手法）
3. `ShowCountdown(3)` / `ShowQr(dummy 64px texture)` / `ShowAlert(...)` の各状態を同様に撮影。**アラートが QR より手前**であること、アラート中は `OperatorOverlayGate.AlertActive=true` で HUD / 4カメラタイルの OnGUI が抑制されること（IMGUI が uGUI を覆う問題の回帰確認）
4. フォントロードログ確認（"Yu Gothic Medium" が Windows で解決されること）
5. Tab トグル: RunCommand でキー相当の操作（`debugView.Visible` / `hud.visible` を直接反転して両状態のスクリーンショット。実キー入力は Editor フォーカス依存のため状態遷移ロジックのみ assert）
6. ClearAll / ClearEverything で全消去 → 素の彫刻表示に戻ること

## リスク / 制約

- **IMGUI は primary display のみ**（既存制約）: オペレーター HUD / 4カメラタイルは display 0 専用のまま。来場者画面は uGUI で分離
- ScreenSpaceCamera は camera の farClip/planeDistance に依存 → planeDistance=1、UI レイヤーは既定のまま（カメラ cullingMask FFFFFFFF 確認済み）
- 文言の内容・タイミングは Phase 5（ステートマシン）の管轄。本フェーズは表示機構のみ
- Yu Gothic Medium が無い環境（Mac）は Hiragino → LegacyRuntime の順でフォールバック、ログで検出可能
