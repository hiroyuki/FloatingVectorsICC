# リアルタイム軽量化ガイド（TSDF mesh / curved line）

> **⚠️ 2026-07-07 追記・要改訂**: 本ガイドの主軸「三角形を減らす」は Unity MCP 実測で**否定された**。
> フレーム時間の 90% は毎フレームの GPU コンピュート（うち **TSDF 統合 `TSDFIntegrator` が ~25ms で支配的**）、
> ラスタ描画は 8% だけ。三角形削減（`smoothSubdiv` / `meshMaxTriangles`）はその 8% しか触れず、ほぼ効かない。
> → 新しい方針・実測値・計測手法・改造プランは **`Plans/tsdf-integration-perf-refactor.md`** を参照。
> 以下の記述は「curved line が最大の三角形供給源」という誤った前提に立っているため、鵜呑みにしないこと。

2026-07-07 時点。Web エクスポート実装（Export Web: GLB+USDZ）の削減実験で得た実測値をもとに、
リアルタイム表示の負荷ノブと推奨手順をまとめる。

## 前提: 何が重いか

| 描画物 | 経路 | 毎フレームの規模（デフォルト設定） |
|---|---|---|
| curved line | `MotionCurvesBuild.compute` → `DrawProceduralIndirect`（**MC を通らない**） | 20000 シード × 31 区間 × subdiv 4 ≈ **250万セグメント = 約500万三角形** |
| TSDF mesh | TSDF volume → Marching Cubes → mesh | `meshMaxTriangles` キャップ 333,333 |

**シーンで最大の三角形供給源は curved line**（TSDF mesh の約15倍）。軽量化はまずカーブ側から。

curved line はカメラ向きリボン（セグメントあたりクワッド1枚 = 2三角形）で描画している。
チューブ実体ではない。voxel に焼くのはプリント用 Fuse curves のみで、リアルタイム表示とは無関係。

## エクスポート実験から得た根拠

1. **カーブは約5倍過剰サンプリング**
   Export Web の Douglas-Peucker 簡略化（許容誤差 1.5mm）で、チューブ三角形が
   1本あたり 520 → 105 に減っても見た目が変わらなかった。
   動きの遅い区間では Catmull-Rom の分割点（subdiv 4）が密すぎる。
2. **TSDF mesh も 25% 削減で見た目が保てる**
   QEM デシメーション 198k → 150k 三角形で品質劣化なし（Web エクスポートの実測）。
   ただし QEM はオフライン処理（数百ms〜数秒）なので毎フレームには使えない。

## 負荷ノブ一覧

### curved line 側（PointCloudMotionCurves）— MC と無関係

| ノブ | 効果 | 品質影響 |
|---|---|---|
| `smoothSubdiv` 4 → 2 | セグメント数 **半減**（最優先） | 上記の通りほぼ不可視 |
| `seedCount` 20000 → 減 | 本数に線形比例 | 線の密度が下がる（見た目に直結） |
| `history.CurveSamples` | 1本の長さ（履歴フレーム数）に線形比例 | 軌跡が短くなる |
| `ribbonWidth` | ラスタ負荷（フィル）のみ | 線の太さ |

### TSDF mesh 側 — こちらが MC の設定

| ノブ | 効果 | 品質影響 |
|---|---|---|
| `TSDFView.useFullGridMC = false` | active-block MC（表面積比例のディスパッチ、実装済み） | **なし**（品質そのまま計算量だけ減る） |
| `meshMaxTriangles` | 表示三角形キャップ | 超過分が欠ける |
| `voxelSize` を粗く | volume 全体の計算量 | 解像度が落ちる（最後の手段） |

## リアルタイムをチューブにしたい場合

- 実ジオメトリのチューブ化（断面4角形）はセグメントあたり 2 → 8 三角形で**約4倍**
  （500万 → 2000万三角形/フレーム）。細線はサブピクセル三角形だらけになりラスタ効率も悪化。**非推奨**。
- 代替: **疑似円柱法線（fake round ribbon）**。`MotionCurves.shader` のフラグメントで
  リボン幅方向の位置から円柱相当の法線を計算してライティングに使う。
  ポリゴン数そのまま（追加コストほぼゼロ）で丸い陰影になり、
  エクスポートしたチューブ（GLB/USDZ）との見た目差もほぼ埋まる。未実装、小規模変更。

## 推奨手順

1. **計測が先**: Unity Profiler（GPU）でフレーム内訳を確認。
   ボトルネックが TSDF 統合や MC の compute dispatch にある場合、
   三角形を減らしても効かない。
2. 効果順の第一手: `smoothSubdiv` 4 → 2 ＋ `TSDFView.useFullGridMC = false`。
   どちらも見た目の劣化なしで、カーブ頂点負荷半減 + MC 計算量削減。
3. まだ足りなければ `seedCount` / `CurveSamples` を作品として許せる範囲で削る。
4. 見た目の丸みが欲しければ疑似円柱法線をシェーダに追加（ジオメトリは増やさない）。

## 関連

- Web エクスポートの削減実装: `Assets/Scripts/TSDF/PrintExport/`
  （`MeshDecimator.cs` = QEM、`TSDFPrintExporter.SimplifyPolyline` = Douglas-Peucker）
- active-block MC: `TSDFView.useFullGridMC`（`Plans/tsdf-hires-mesh-plan.md`）
