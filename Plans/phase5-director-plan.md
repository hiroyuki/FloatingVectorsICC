# Phase 5 実装プラン: ExperienceDirector + ステートマシン E2E（dry）（改訂1）

Parent plan: `Plans/experience-flow-plan.md`（Codex 承認済み。Phase 1-4 実装済み・main マージ済み）。

## 目的

体験フローの中枢を作り、**dry モードで E2E**（全ステート遷移 → 3キャプチャ → 球選択 → export → dry publish → QR 表示 → Attract 復帰）を AI 自身が検証する。実カメラ・実アップロード・アトラクト playback は Phase 6/7。

## 新規ファイル（すべて Experience asmdef）

### 1. `ExperienceStateMachine.cs` — 純 C#（EditMode テスト可能）

```csharp
public enum ExperienceState { Attract, Welcome, FreePlay, Ready,
    PromptAnimal, PromptMantis, PromptFree, Select, Exporting, QrShow, Fault }

public struct ExperienceInputs   // director が毎 Tick 供給
{
    public bool Present;         // PresenceDetector.IsPresent
    public bool Fault;           // 全画面赤アラート条件（Phase 6 で CameraHealthMonitor 接続、Phase 5 は debug）
    public bool CaptureDone;     // 現 Prompt のキャプチャ完了
    public int  SelectedIndex;   // Select 中の球選択（-1 = 未選択）
    public bool ExportDone;      // Exporting 完了（成功）
    public bool ExportFailed;    // Exporting 失敗（→ 謝罪文言 → Attract）
}

public sealed class ExperienceStateMachine
{
    public ExperienceState State { get; }
    public float TimeInState { get; }
    public event Action<ExperienceState /*from*/, ExperienceState /*to*/> Changed;
    public void Tick(float scaledDt, in ExperienceInputs inputs, ExperienceTimings t);
    public void ForceTransition(ExperienceState to, string reason); // director の Fault/Exit 用
}
```

- **ExperienceTimings** = 純 POCO（ScriptableObject から詰め替え）: welcomeSeconds / freePlaySeconds / readySeconds / promptSeconds / selectTimeoutSeconds(0=無限) / exportFailNoticeSeconds / qrShowSeconds / **skip フラグ**（skipAttract, skipWelcome, skipFreePlay, captureCount 1..3, skipExportUpload, skipQr）/ **timeMultiplier**（開発用、Tick の dt に乗算）
- 遷移規則（FSM 内に実装、テスト対象）:
  - Fault=true → どこからでも Fault。Fault 中 Fault=false → Attract
  - Attract: Present → Welcome（skipAttract は「初回起動時も Attract を飛ばして Welcome へ」）
  - Welcome: 経過 welcomeSeconds → FreePlay。**Present=false → Attract**（離脱、以降 QrShow まで同じ離脱規則。Exporting だけは完走）
  - FreePlay → Ready → PromptAnimal → (captureCount>=2: PromptMantis) → (captureCount>=3: PromptFree) → Select（各タイマー経過 + CaptureDone 待ち: Prompt は promptSeconds 経過 && CaptureDone で次へ）
  - Select: SelectedIndex >= 0 → Exporting。selectTimeout 経過 → Attract
  - Exporting: ExportDone → QrShow（skipQr なら Attract）。**ExportFailed は FSM 内部でラッチ**（Codex 指摘1対応）: 1フレームのパルスで `_failNoticeElapsed` の計時を開始し、Exporting に留まったまま exportFailNoticeSeconds 経過 → Attract（director はこの間、謝罪文言を表示）。1フレームパルスのテストを必須化
  - QrShow: qrShowSeconds 経過 or Present=false（デバウンス済）→ Attract
  - **CaptureDone のプロンプト毎リセット契約**（Codex 指摘3対応）: FSM は CaptureDone 入力を**内部ラッチ**し、**各 Prompt ステート Enter 時にラッチをクリア**する — director が入力フラグを下げ忘れても、前のプロンプトの完了が次のプロンプトを素通しさせない。director 側も state Changed で自フラグをリセット（二重防御）。「各 Prompt は自分のキャプチャ完了を要する」テストを追加
  - skip フラグの意味 = 「Enter 直後に即、次ステートへ遷移」（Enter は必ず通す — E2E 検証の抜け防止、ユーザー要望どおり）
- 純 C#（UnityEngine 非依存は Mathf 程度に留める）→ `Assets/Tests/Editor/ExperienceStateMachineTests.cs` で全遷移・skip・離脱・Fault 割り込みをテスト

### 2. `ExperienceConfig.cs` — ScriptableObject（CreateAssetMenu）

- rigSerialOrder(4)、上記 ExperienceTimings 一式、文言（welcome / freePlay / ready / promptAnimal / promptMantis バリアント配列 / promptFree バリアント配列 / exporting / exportFailed / crowd 注意 / qr キャプション）、カウントダウン秒（キャプチャ前）
- 球レイアウト: xOffsets = {-1, 0, +1}、height = 1.2（**床基準 — 実床 y は SpaceBuilder.floorY を加算**）、radius、dwellSeconds
- `dryRunPublish = true`（Phase 5 既定）、QR URL 種別 enum { Usdz, Glb, First }
- occupancyThreshold / inset / yBand の体験用上書き値（Enter 時に PresenceDetector へ push）
- 検証用に `Editor/` メニュー無しでも `ScriptableObject.CreateInstance` + 既定値で動く（シーンアセット非依存）

### 3. `ExperienceDirector.cs` — MonoBehaviour（ショーの単一オーナー）

- `Shared.IViewToggle` 実装（`ViewLabel = "Experience mode"`、`Visible` = モード on/off）→ 既存 Views パネルに自動出現。Inspector からも `enabledMode` トグル
- **Enter**（スナップショット → 適用）:
  1. スナップショット: BonePoseHistory.historySamples / curves.visible / SensorManager+SensorRecorder の applyWorldRebase / PresenceDetector 閾値系
  2. rebase on（rigSerialOrder を config から両コンポーネントへ）+ `ApplyExtrinsicsToLive()` / `RefreshExtrinsicsAndReapply()`（可逆な一点フック、Phase 2 検証済み）
  3. `ExperienceSpaceBuilder.Apply()`（TSDF/グリッド追従は自動）
  4. VisitorMessageUI / PresenceDetector を生成（自 GO 配下）。DwellSphere ×3 を config レイアウトで生成（非アクティブ、Select で activate）
  5. FSM を Attract から開始
- **Exit**（逆順復元）: FSM 停止 → Exporting 中なら CancellationTokenSource.Cancel + タスク observe（fire-and-forget 禁止）→ 生成 GO 破棄（VisitorMessageUI は ClearEverything → Destroy、**alert gate 残留対策**）→ SpaceBuilder.Restore → rebase 復元 + 再適用 → スナップショット値復元
- **同期 ExportFiles 中の Exit**（Codex 指摘4対応）: ローカル書き出し（`ExportFiles`、~2-3s 同期）はメインスレッドで走るため、その間の Exit 要求はフレーム上到達しない = ブロックは**最大書き出し1回分で自然に解消**し、アトミック .tmp+rename により部分ファイルも残らない。Phase 5（dry）ではこれを仕様として明記し許容。Phase 7 で実 upload を入れる際、書き出し自体の Task.Run 化を Phase 1 実測（decimate 1.2s）と合わせて再判断
- **ステート side effects**（Changed ハンドラ + Update）:
  - 各ステートで VisitorMessageUI.ShowMessage(config 文言)。PromptMantis/Free はバリアント配列からランダム
  - Prompt 末尾のキャプチャ手順（親プラン確定シーケンス）: 残り countdown 秒で ShowCountdown → `historySamples` 保存→15 に設定 → **readiness 待ち**: `PointCloudMotionCurves.BuildVersion`（**新設 public カウンタ**、ビルド成功毎に ++）が 2 進む（=15 窓で確実に1回以上フルビルド）まで待つ → `TSDFSnapshotBuilder.Capture()` 同期実行 → historySamples 復元 → CaptureDone=true。失敗（mesh 0 等）はログ + snapshot 無しで続行（3 個未満でも Select は成立、無い分の球は非表示）
  - **readiness ハング対策**（Codex 指摘2対応）: Enter 時のスナップショットに `curves.visible` / `curves.freeze` を含め、**強制的に visible=true / freeze=false** にする（Exit で復元）— どちらかが立っていると Update が early-return して BuildVersion が永遠に進まない。さらに readiness 待ちに **タイムアウト（scaled 2s）**: 超過したら warning ログ + そのまま Capture 実行（TryReadCurvePolylines が失敗しても mesh-only capture は成立する設計 = Phase 1 の契約）
  - Select: 3 球 activate（snapshot と対応）+ `BuildDisplayMeshes` の表示 GO を球の上に配置（unlit 頂点色、DwellWire 同様 Resources シェーダ`SnapshotVertexColor` 新設 or 既存 Particles/Unlit — **ビルド安全のため Resources 新設**）。DwellSphere.OnSelected → SelectedIndex
  - Exporting: 選択 snapshot を `ExportFiles`（`~/Documents/FloatingVectorsPrints/exp_<日時>.glb/.usdz`）→ `ISculptureResultPublisher.PublishAsync`（dry）→ URL 決定（enum）→ CaptureDone/ExportDone 入力へ。将来の実 upload も想定し **Task を director が保持して観察**、例外は ExportFailed 扱い
  - QrShow: `QrUrlPresenter.Present(url)` → Texture2D → VisitorMessageUI.ShowQr
  - CrowdActive（体験中）: 白文字注意を ShowMessage に重ねず**専用スロットが無いので message を一時差し替え**（アラートではない）→ 解除で元の文言へ
  - Fault: VisitorMessageUI.ShowAlert（全画面赤 + オペレーター IMGUI + HUD 抑制は Phase 3 実装済み）。`debugForceFault` フィールドで Phase 5 検証
- Attract ステート（Phase 5 は簡易版）: 文言のみ（「あそびにきてね」等）。playback ループ再生は Phase 6

### 4. `Publishing/ISculptureResultPublisher.cs` + `DryRunPublisher.cs`

```csharp
public interface ISculptureResultPublisher
{
    Task<PublishResult> PublishAsync(string glbPath, string usdzPath, CancellationToken ct);
}
public struct PublishResult { public bool Success; public string GlbUrl, UsdzUrl; public string Error; }
```
- DryRunPublisher: 設定可能な遅延（既定 1s）後にフェイク URL を返す。ct でキャンセル可能
- LfksUploadPublisher（実 upload.ps1 実行）は **Phase 7**

### 5. `Publishing/IUrlPresenter.cs` + `QrUrlPresenter.cs` + `ThirdParty/QrCodeGen 一式`

- QR 生成: **manuelbl/QrCodeGenerator（MIT、Nayuki 移植の保守版）から `QrCode.cs` / `QrSegment.cs` / `BitArrayExtensions.cs` をベンダリング**（`Assets/Scripts/Experience/ThirdParty/`）。**コミット SHA 固定の raw URL から取得**（Codex 指摘5対応、floating branch 不可）し、取得元 URL+SHA と MIT ライセンス全文を `ThirdParty/QRCODEGEN-LICENSE.txt` として同梱、各ファイル冒頭に出典コメント追記。取得物は目視で「ネットワーク/IO 呼び出しが無い純計算コード」であることを確認してから取り込む（サプライチェーン注意）。ネット不可なら QR をプレースホルダにして Phase 7 へ延期
- `QrUrlPresenter.Present(string url) → Texture2D`: QrCode.EncodeText(url, Ecc.Medium) → module 毎 1px の Texture2D（Point filter、白黒）→ VisitorMessageUI.ShowQr 側で拡大表示

### 6. 既存ファイルの小変更

- `Experience/Experience.asmdef` — **`TSDF` 参照を追加**（Codex 指摘: ExperienceDirector が TSDFSnapshotBuilder.Capture/BuildDisplayMeshes/ExportFiles を呼ぶため。コンパイル確認を Phase 5 検証に含める）

- `BodyTracking/PointCloudMotionCurves.cs` — `public int BuildVersion { get; private set; }` をビルド成功毎に ++（1行、挙動不変）
- `Experience/VisitorMessageUI.cs` — **Phase 3 の Codex NON_BLOCKING 対応**: `OnDisable` で alert 所有中なら gate=false、`OnEnable` で `_alertOwned` なら gate=true 復元
- `CameraControl/Display1OperatorHud.cs` — Experience Mode トグル（IMGUI チェックボックス、director.Visible を叩く）を HUD に追加（親プラン記載）

## 検証（E2E dry、Unity MCP、録画 12-50-09 再生下）

1. EditMode: ExperienceStateMachineTests（全遷移 / skip 各種 / 離脱 / Fault 割り込み / captureCount 1..3 / timeMultiplier）全 green + 既存 35 テスト green
2. Play + 再生 + `debugForcePresence` + `debugForceDwell`: director.Visible=true → Attract→…→QrShow→Attract の**フル 1 周**をログで assert（timeMultiplier 高速化）
3. 3 キャプチャ: 各 Prompt で snapshot 取得（tris>0）、historySamples が 15→復元されること、BuildVersion 待ちが機能すること（待ち中に固まらない）
4. Select: 表示 GO 3 体 + 球 3 個、debugForceDwell 成立 → Exporting
5. Exporting: exp_*.glb/.usdz が書かれ、DryRunPublisher の URL が返り、QR Texture2D 非 null、ShowQr 表示（スクリーンショット）
6. **Exit 復元**: モード off → historySamples / curves.visible / **curves.freeze** / rebase フラグ / BoundingVolume transform / FloorOrigin.boundingBox の全復元を数値 assert。Exporting 中 off のキャンセル経路（部分ファイル無し）
7. Fault: `debugForceFault` → 全画面赤 + HUD 抑制 → 解除 → Attract
8. CrowdActive: PresenceDetector.DebugSetRawPersonCount(2) → 注意文言 → 解除

## リスク

- QR ベンダリングのネットワーク取得（WebFetch）が失敗する場合 → 代替ソース、最悪 Phase 7 まで QR をプレースホルダ市松に
- キャプチャの同期ヒッチ ~2s（Phase 1 実測、decimate 支配）→ countdown 演出中に実行して隠す。不足なら weld/smooth/decimate を Task.Run へ（実測して判断）
- Select 画面の表示 GO の見せ方（配置・スケール）は目視チューニング項目（Phase 7 仕上げ）
