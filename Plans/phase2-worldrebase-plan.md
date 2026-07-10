# Phase 2 実装プラン: World rebase + センシングエリア（改訂1）

Parent plan: `Plans/experience-flow-plan.md`（全体プランは Codex 承認済み。Phase 1 実装済み・main マージ済み）。本プランは Phase 2 の実装詳細。

## 目的

extrinsics.yaml のグローバル座標系（キャリブ時の board 基準）を、**原点=4カメラ重心の床投影、X̂=カメラ1→2方向（yaw+XZ 平行移動のみ、Y-up と床高はキャリブのまま）** に再基準化する。これで選択球の配置（X=-1/0/+1m）やセンシングエリアが座標固定で書ける。既定 off（Dev モード完全不変）。

## 座標契約

- rebase は `ExtrinsicsApply.ToUnityLocal()` の**後**、Unity 空間の Pose として左から合成する。OpenCV/mm 空間では合成しない（per-GO の localScale.y=-1 flip と干渉するため）
- 合成式: `localPosition' = R.rotation * localPosition + R.position`、`localRotation' = R.rotation * localRotation`。**localScale には一切触れない**（Y-flip 維持）
- **親 transform 前提の明示**（Codex 指摘1）: 合成結果を localPosition/localRotation に代入する規約は、live renderer（SensorManager 子）と playback GO（SensorRecorder 子）の**親がワールド identity のときのみ** live/playback で同一ワールドになる。適用ヘルパーは親の `localToWorldMatrix` が identity（許容誤差付き）でない場合 **warning を出して rebase を適用しない**（従来 pose のみ適用）。シーン上の SensorManager / SensorRecorder GO は現状 identity（検証項目に含める）
- live と playback は**同一 yaml → 同一の決定的 Pose**（共有 mutable state 無し）

## 新規ファイル

### 1. `Assets/Scripts/Calibration/WorldFrameRebase.cs`（純 static 数学、Calibration asmdef）

```csharp
public static class WorldFrameRebase
{
    // camPosUnity: ToUnityLocal 済みのカメラ位置（Unity 空間、rig serial 順 1..4）。
    // 成功時 rebase = 「新world ← 旧world」Pose（yaw+XZ のみ、y 成分 0）。
    public static bool TryCompute(IReadOnlyList<Vector3> camPosUnity,
                                  out Pose rebase, out string reason);

    // yaml パース結果 + serial 順から位置列を引いて TryCompute する共通入口。
    // 全 call site（live/playback）はこれだけを使う。
    public static bool TryComputeFromCalibrations(
        IReadOnlyList<(string serial, Vector3 posUnity)> cams,
        IReadOnlyList<string> rigSerialOrder,
        out Pose rebase, out string reason);

    // ToUnityLocal + rebase 合成の最終 local pose を返す唯一のヘルパー
    // （Codex 指摘3: 3 call site で数学を重複させない）。
    public static void ComposeUnityPoseAfterToUnityLocal(
        in Pose unityLocal, in Pose rebase, out Vector3 pos, out Quaternion rot);
}
```

数学:
- centroid の XZ 投影を原点（y=0、床高=キャリブ frame のまま）
- `X̂ = normalize(projXZ(p2 - p1))`、`Ŷ = up`、`Ẑ = Vector3.Cross(X̂, Ŷ)`（Unity LH の正規直交 basis）
- **Ẑ 符号検証は fail 扱い**（Codex 指摘2）: `dot(Ẑ, normalize(projXZ(p3 - p2))) < cos(10°)` なら **false を返し rebase を適用しない**（warning ではなく不適用）。serial 順の取り違えや鏡映 rig を疑わしい basis のまま流さない
- `yawRot = Quaternion.LookRotation(Ẑ, Ŷ)`、`rebase = Pose(-(yawRot⁻¹ * centroidXZ), yawRot⁻¹)`

入力検証（Codex 指摘6: 失敗時は**部分適用せず**全カメラ従来 pose）:
- count != 4 / serial 重複 / rigSerialOrder に無い serial / 位置が非有限（NaN/Inf）
- `|projXZ(p2-p1)| < 1cm` / `|projXZ(p3-p2)| < 1cm`（退化）
- 位置の絶対値 > 100 m（yaml 破損ガード。extrinsics.yaml は信頼しない数値入力として扱う）

### 2. `Assets/Scripts/Experience/ExperienceSpaceBuilder.cs` + `Experience.asmdef`（新設、参照: PointCloud, Calibration）

- `Apply()`: シーンの配置済みカメラ transform（live renderer または `_Playback_*`、serial 順）から XZ 位置を取得 → 対角方向に各 1m 内側の 4 点 → それを包む axis-aligned 矩形 → 既存 `BoundingVolume` を position=(cx,1.25,cz), **rotation=identity**, scale=(sx,2.5,sz) に設定、`FloorOrigin.boundingBox` を接続
- **グリッド追従の制約**（Codex 指摘5）: `FloorOrigin` は bounding box の bottom-center **位置のみ**追従し回転は追従しない（FloorOrigin.cs:200）。Experience の box は rebase 済み world で axis-aligned（rotation=identity）に**固定**するので回転追従は不要 — この前提を Apply() 内で assert（box rotation ≠ identity なら warning）
- `Restore()`: Apply 前の BoundingVolume transform / FloorOrigin.boundingBox 参照を復元（Phase 5 で ExperienceDirector の Enter/Exit スナップショットに接続）
- MonoBehaviour（ContextMenu で手動 Apply/Restore）。既定では何もしない

## 既存ファイルの変更（全てフラグゲート、既定 off）

### 3. `Calibration/ExtrinsicsApply.cs`

overload 追加のみ:
```csharp
public static void ApplyToTransform(Transform t, in ObExtrinsic ocv, in Pose unityRebase)
```
内部で `ToUnityLocal` → `WorldFrameRebase.ComposeUnityPoseAfterToUnityLocal` → 代入。既存 2 引数版は無変更（Dev パスは bit 同一）。

### 4. `PointCloud/SensorManager.cs`

- フィールド追加: `public bool applyWorldRebase = false;` `public string[] rigSerialOrder = new string[0];`
- `ApplyExtrinsicsToLive()`: フラグ on 時、yaml の calibrations から `TryComputeFromCalibrations` → 成功時のみ全 renderer を rebase 付き overload で適用。失敗時は reason を warning ログ + **全カメラ従来適用**（部分適用なし）。適用前に親 transform identity チェック（上記座標契約）
- 可逆性: フラグを戻して `ApplyExtrinsicsToLive()` 再実行で復元（既存メソッドは再入可能）

### 5. `PointCloud/SensorRecorder.cs`

- フィールド追加: 同上 2 フィールド
- `RefreshExtrinsicsAndReapply()`（:1314）と `EnsurePlaybackObject()`（:2059）の 2 call site を、フラグ on 時のみ rebase 付き overload に切替。Pose は `_tracks` の GlobalTrColorCamera 全件から `TryComputeFromCalibrations`（SensorManager と同一ヘルパー、数学の重複なし）
- `EnsurePlaybackObject` は track 毎に呼ばれるため、Pose は playback セッション開始時に 1 回計算してキャッシュ（yaml 再読込 = `RefreshExtrinsicsAndReapply` で再計算）

### 6. `Assets/Tests/Editor/WorldFrameRebaseTests.cs`（Calibration.Tests.Editor.asmdef）

- **具体配置テスト**（Codex 指摘2）: 正方形 4 カメラの具体座標（例 p1=(2,1.8,-2), p2=(2,1.8,2) …）→ rebase 適用後の各カメラ期待位置を数値で assert（basis の向き・符号を固定化）
- 全カメラに同一 yaw+平行移動を与えた入力 → rebase 後配置が canonical と一致（不変性）
- 退化・不正入力一式（count!=4、serial 重複、p1≈p2、p3≈p2、NaN、>100m）→ false
- Ẑ と p3-p2 の 10°超ズレ → false（fail 扱いの回帰）
- **scale 非干渉**（Codex 指摘4）: 同一 ObExtrinsic を scale=(1,1,1) と (1,-1,1) の GO に rebase 付きで適用し、(a) localScale が変化しないこと、(b) OpenCV 側で Y 反転済みのメッシュ点ペアの world 位置が両 GO で一致すること（Y-flip×rebase の非干渉）
- 既存 `SkeletonWorldTransformTests` が green のまま（renderer scale を意図的に無視する契約の非回帰）

## 実機検証（Unity MCP、AI 自身が実施）

1. EditMode テスト全 green（`run_tests`）
2. SensorManager / SensorRecorder GO の親がワールド identity であることを確認（座標契約の前提）
3. Play + 録画 2026-07-08_12-50-09（実 extrinsics.yaml）で recorder の `applyWorldRebase=true` + rigSerialOrder 設定 → `_Playback_*` 4 GO の位置平均 XZ ≈ (0,0)（<数 cm）、カメラ1→2 ≈ +X（<1°）、床高 y 不変、localScale=(1,-1,1) 維持を RunCommand で assert
4. 実 yaml で TryCompute が fail しないこと（fail した場合 = rig の番号回りが仮定と逆 → X̂/Ẑ 定義を rig 実態に合わせて確定し、テストの期待値も更新してから先へ進む）
5. ExperienceSpaceBuilder.Apply → BoundingVolume が原点中心・カメラ対角 1m 内側・rotation=identity、FloorOrigin グリッド追従を数値 assert + スクリーンショット。Restore で完全復元
6. **rebase 後の点群/彫刻がシーンの TSDFVolume 境界内に収まるか**（rebase で内容が原点付近へ移動するため）。外れる場合は TSDFVolume/FloorOrigin の Experience 用配置を SpaceBuilder の管理対象に追加
7. BT（SkeletonMerger）が rebase 後も正常動作（skeleton が点群に重なる）

## リスク

- キャリブのグローバル座標が床基準 Y-up でない場合、床投影の前提が崩れる → 検証 3 の床高チェックで検出、必要なら床平面推定を追加（親プラン記載の未決事項）
- TSDFVolume のカバー範囲（検証 6）
- rigSerialOrder 入力ミス / yaml 破損 → TryCompute が false → rebase 不適用（従来動作）+ warning。部分適用は起きない
