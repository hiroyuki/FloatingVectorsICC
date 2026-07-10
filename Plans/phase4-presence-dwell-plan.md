# Phase 4 実装プラン: Presence 検知 + DwellSphere（改訂1）

Parent plan: `Plans/experience-flow-plan.md`（Codex 承認済み。Phase 1-3 実装済み・main マージ済み）。

## 目的

体験フローの入力系を作る:
- **在場検知**: 体験中 = BT（SkeletonMerger の merged 骨盤が OBB 内、デバウンス付き）／アトラクト中 = live 点群の OBB 内占有カウント（BT はアトラクト再生が worker を占有するため使えない — master plan 確定事項）
- **選択入力**: 手（HAND_LEFT / HAND_RIGHT、どちらでも）を球内に 1 秒維持で成立する DwellSphere（ワイヤー球 + 進捗円弧 + グロー + SE スロット）

単一人物前提（CLAUDE.md）。2人以上は Alert 用の PersonCount のみ（追跡はしない）。

## 変更・新規ファイル

### 1. `BodyTracking/SkeletonMerger.cs` — merged 結果の公開 API（読み取り専用）

```csharp
public struct PersonSample
{
    public uint Id;
    public Vector3 PelvisWorld;
    public Vector3 HandLeftWorld, HandRightWorld;
    public bool HandLeftTracked, HandRightTracked; // confidence >= LOW
}
public int PersonCount { get; }                  // このフレームの merged 人数
public IReadOnlyList<PersonSample> Persons { get; }
public bool TryGetPrimaryPerson(out PersonSample p); // 先頭（単一人物前提で十分）
```

- `ApplyMergedSkeletons()` の各 cluster ループ末尾（`requireMinWorkerCount` 通過後、Refine 済みの `_mergedSkel` から）で `_persons` リストに詰める。ワールド変換は既存の merged 骨格の変換経路（`BodyTrackingShared` の k4a mm ↔ Unity 変換、BodyVisualPool が使うのと同一）を流用。関節 index は `k4abt_joint_id_t.K4ABT_JOINT_PELVIS / HAND_LEFT / HAND_RIGHT`（マジックナンバー不使用）
- **座標契約の明記**（Codex 指摘2対応、コードコメントに残す）: `_mergedSkel.Joints[].Position` は「Unity ワールド位置を `UnityToK4Amm` で encode した合成 k4a-mm 値」。復元は `K4AmmToUnity` **のみ**（`SkeletonWorldTransform.ToWorld` を通すと二重変換になる — worker 生スナップショット用の関数であって merged 用ではない）
- リストはフレーム毎に Clear→再構築（alloc なしで Capacity 維持）。`OnDisable` で Clear（既存の `_latestBySerial.Clear()` と同じ場所）
- merge パイプライン自体には**一切手を入れない**（読み出し追加のみ、Dev モード挙動不変）
- CrowdActive（2人以上）の判定・デバウンスは merger に持たせず PresenceDetector 側（merger は生の PersonCount だけ）

### 2. `Experience/PresenceDetector.cs` + `Experience/Resources/PresenceOccupancy.compute`（新規）

**BT 経路（体験中）**:
- `IsPersonInside` — `TryGetPrimaryPerson` の骨盤が sensing OBB（`BoundingVolume`）内。OBB は `insetMeters`（XZ 内側マージン）+ `yBandMin/yBandMax`（ワールド y 帯、絨毯反射・縁の保護者対策 — master plan のリスク項目を config 化）で絞る
- デバウンス: `enterDebounceSeconds`（既定 0.3）連続で内側→true、`exitDebounceSeconds`（既定 1.0）連続で外→false（BT ドロップアウトのちらつき吸収）
- `PersonCount`（デバウンス付き、Alert 用）: raw merger.PersonCount が `crowdDebounceSeconds` 連続 >=2 で `CrowdActive=true`
- `TryGetHands(out Vector3 left, out bool leftValid, out Vector3 right, out bool rightValid)` — primary person の手（tracked フラグ透過）

**占有経路（アトラクト中）**:
- compute kernel 1 本: source 点群バッファ（`ObColorPoint` 24B raw、`PointCloudMotionCurves` の collect kernel と同一契約: `mesh.vertexBufferTarget & Raw`、stride 24、1e10 hole dummy スキップ、`sanityRange` ガード）を `L2W` でワールドへ → `worldToBox`（OBB inverse）でボックスローカル → 内側判定（**inset は XZ のみ、Y は別系統** — Codex 指摘1対応）:
  ```
  insideXZ    = abs(local.x) <= 0.5 - insetX && abs(local.z) <= 0.5 - insetZ
  insideBoxY  = abs(local.y) <= 0.5                    // OBB 高さガード
  insideWorldY = world.y >= yBandMin && world.y <= yBandMax
  ```
  3 条件すべてで `InterlockedAdd(_Counter)`。`insetX = insetMeters / boxSizeX`, `insetZ = insetMeters / boxSizeZ` を CPU 側で計算し、**`insetMeters >= 半辺（X or Z）なら 0 にクランプ + warning**（領域反転の禁止）。BT 経路（骨盤 in-OBB）も同一の XZ-inset + world-Y 帯判定を共有ヘルパー（C# 側 `PresenceDetector.IsInsideSensingVolume(Vector3 world)`) で使う — GPU/CPU の判定式を1対1対応させる
- source 解決: **live `PointCloudRenderer` の mesh のみ**（既定。アトラクトのゴースト=playback を数えたら本末転倒）。`overrideSources`（MeshFilter[]）で差し替え可能 — **ライブカメラ無しの検証はこれで playback メッシュを数えて kernel を検証**
- `occupancyIntervalSeconds`（既定 0.2）ごとに dispatch → `AsyncGPUReadback`（前回 pending 中はスキップ、`SystemInfo.supportsAsyncGPUReadback` false なら同期 GetData にフォールバック）
- `OccupancyCount`（最新読み戻し値）、`OccupancyActive` = count >= `occupancyThreshold`（既定 1500、現地調整）+ 同じ enter/exit デバウンス
- **`debugForcePresence`**: true で `IsPersonInside`/`OccupancyActive`/`IsPresent` を強制 true（Mac / ハード無し検証）
- `IsPresent` = `IsPersonInside || OccupancyActive`（director はモードに応じて個別プロパティも参照可）
- OnDisable: readback 破棄・counter буffer解放・状態リセット

### 3. `Experience/DwellSphere.cs`（新規、`WireVisualizationBehaviour` 継承）

- **形状**: 3 直交円（XY/YZ/XZ 平面、各 64 セグメントの Lines トポロジ）。`CreateMesh` で構築、`UpdateMesh` で進捗変化時のみ再構築（既存の per-frame rebuild フック、CameraPoseMarker と同パターン）
- **進捗表現**: 待機時は各円が `idleArcFraction`（既定 0.25 = 1/4 円弧）。手が `radius`（既定 0.18m）内にある間 `progress += dt / dwellSeconds`（既定 1.0s）で円弧が伸び、`progress=1` で完全な円。手が離れたら即リセット（仕様「離れるとリセット」）
- **成立時**: `OnSelected`（`UnityEvent` + C# `event Action<DwellSphere>`）1 回発火 → グロー: 頂点色を `activeColor * glowBrightness`（既定 ×6、HDR — Bloom 有効化時に光る。unlit 頂点色 shader は値をそのまま通す）から `glowSeconds`（既定 0.4）で通常色へランプ → `Locked` 状態（再発火しない）。`ResetSphere()` で待機に戻る（director が状態遷移で呼ぶ）
- **入力**: `presence` 参照（PresenceDetector.TryGetHands）。tracked な手のどちらかが半径内で dwell 進行。`debugForceDwell` で強制進行（Mac 検証）
- **Update の置き場**（Codex 指摘3対応）: `WireVisualizationBehaviour.Update()` は **private 非仮想**なので、サブクラスに `Update()` を書くと基底のメッセージが**隠されて可視化同期が死ぬ**。dwell 進行・グロー減衰は `UpdateMesh(Mesh)` override 内に置く（基底の SyncVisualization → UpdateMesh が毎フレーム呼ぶ、CameraPoseMarker と同パターン）。戻り値 true は円弧形状が実際に変わったフレームのみ（無駄な色再適用を避ける）。`Show` は常時 true 運用（director が GO ごと activate/deactivate）
- serialized 値の妥当性: radius/dwellSeconds/glow 系は `[Min(...)]` で非負・有限を担保
- **SE**: `public AudioClip selectedClip;` + 自前 `AudioSource`（無ければ AddComponent）で `PlayOneShot`（音素材は後日 — スロットのみ）
- 色: `idleColor`（白 α低め）/ `activeColor`（進行中・成立）。`WireColor` override が現在色を返し、基底の色変化検知で再適用される

### 4. `Experience.asmdef` — 変更不要（PointCloud / BodyTracking 参照済み）

## 実機検証（Unity MCP、録画 12-50-09、AI 自身が実施）

1. コンパイル + 既存 EditMode テスト（35本）green 維持
2. Play + 再生 → `merger.PersonCount == 1`、`TryGetPrimaryPerson` の骨盤位置が点群上（Phase 2 検証と同じ位置照合）。手の tracked フラグと位置が妥当（骨盤から < 1.2m）
3. PresenceDetector（BT 経路）: BoundingVolume（Dev 配置のまま）で `IsPersonInside=true` を assert。骨盤が外になる位置に OBB を一時移動 → exit デバウンス後 false
4. 占有経路: `overrideSources` に `_Playback_*` の MeshFilter を指定 → `OccupancyCount > 0` を assert、閾値と比較して `OccupancyActive` 遷移。interval 中の readback pending スキップの動作確認
5. DwellSphere: 再生中の手の位置を数秒サンプリング → 滞留点に球を配置（dwellSeconds を一時 0.3 に）→ progress 上昇 → `OnSelected` 発火を assert。球を遠方へ移動 → 進捗リセット。グロー中の頂点色 > 1 を assert（HDR ランプ）。スクリーンショットで円弧の見た目確認
6. `debugForcePresence` / `debugForceDwell` で BT 無しでも全プロパティが立つこと（Mac 相当の dry パス）
7. 2人検出（録画は1人なので）: merger.PersonCount を一時的に偽装できないため、CrowdActive はデバウンスロジックの EditMode 相当（PresenceDetector に internal テストフック `DebugSetRawPersonCount` を持たせ RunCommand で駆動）で検証

## リスク

- 占有閾値のロバスト性（絨毯反射・縁の保護者）→ inset / y 帯 / 閾値を全部 serialized にして現地調整（master plan 記載どおり）
- ライブカメラが今接続されていない → 占有経路の source 既定（live renderer）は契約検証のみ、kernel は overrideSources で実証。Phase 6（Windows 実機）で live 4 台の実測
- 球の半径・配置（X=-1/0/+1m、高さ1.2m）は Phase 5 の config 管轄。本フェーズは部品のみ
- SkeletonMerger の変換ヘルパー名は実装時にコードで確定（`BodyTrackingShared` の既存関数を使い、推測で書かない）
