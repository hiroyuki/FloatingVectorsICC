# 3D プリント納品ルート（決定稿 2026-07-19 / ボディ入り 41MB）

TSDF 彫刻を 3D プリント用 3MF として書き出す**確定手順**。作例:
`Dropbox/projects/ICC/print/test0719/print_20260719_160329_f00122_final.3mf`（**41MB**）。

## いちばん大事な結論

- **三角形数を落とすレバーは `webCurveTolerance`（ポリライン簡略化）。`printSeedStride` ではない。**
  - `webCurveTolerance` は名前に反して **STL/OBJ のチューブ経路でも効く**
    （`TSDFPrintExporter.BuildCurveAndBridgeTubes` に渡っている）。
  - 直線区間のセグメントが消えるので、`0.0015 → 0.003` にするだけで tri 数が桁で落ちる。
- **納品は `ExportObj()` → `fix_body.py`。`Export3mf()` を直に押さない。**
  - `Export3mf()` 直書き × `tolerance 0.0015` × `stride 2` が、Mac で数百MB〜1GB になった元凶。
- **stride は原因ではない。** stride 2 の決定稿でも 41MB に収まっている。

## 手順

1. **シーン設定を決定稿にする**（下表）。特に `webCurveTolerance = 0.003`。
2. Play → 目的のフレームで **Pause**（`CurveSamples` を満たすため十分再生してから）。
3. Print Export パネルで **Export OBJ**（`ExportObj()`。2マテリアルの OBJ+MTL が出る）。
4. ターミナルで:
   ```
   python eval/tools/fix_body.py 出力.obj 出力_final.3mf
   ```
   `pymeshfix` が **Human スパンだけ** watertight 再構築し、3色 basematerials 付き
   Standard 3MF を書き出す（チューブ・床は素通り）。
   - 依存: `pip install pymeshfix numpy`（Python 3.8+。venv 推奨。未導入だと動かない）
5. 結果 ≈ **41MB** の納品 3MF。

## 決定稿のシーン値（Assets/main.unity）

| 項目 | 決定稿 | 備考 |
|---|---|---|
| `printSeedStride` | **2** | ライン密度。40 だとライン少なすぎ |
| `webCurveTolerance` | **0.003** | ★tri数レバー。0.0015 のままだと数百MB |
| `printRadius` | **0.009** | 9mm |
| `stlCurveSides` | **4** | 断面4角 |
| `tubeTailTaper` | **1** | テーパーなし＝均一バー |
| `stlIncludeBridges` | **0** | 黒ブリッジ株はノイズなので無し |
| `tubeHeadTrim` | **0.04** | 先端を体内へ（下記の bury 併用時は不要）|
| `stlBlackLineRatio` | **0** | 全白ライン |
| `stlIncludeBody` / `stlAutoCloseHoles` | **1 / 1** | ボディ入り・穴閉じ |

### ★ CurveSamples（トレイル長）の罠

- `BonePoseHistory.historySamples = 32` が**必須**。
- シーンの序列値が **4** に戻る個体があり、**Play のたび 4 に戻る**。
  32 で書き出さないと**別物**が出る（過去にこれでハマった）。
- 書き出し前に `historySamples == 32` を必ず確認する。

## Mac 固有の注意

- **`stlCurveSupportPillars` と `stlLinkIsolatedChains` は O(n²) 単一スレッド**で、
  高密度（stride 2 など）だと数十分ハングする。原因は**サポート柱と孤立チェーン結線**。
  - 決定稿ルート（OBJ→fix_body）では tolerance 0.003 で tri が激減するので回避できるが、
    これらを ON にすると依然重い。Mac で回すなら OFF が無難（本番構造が要るなら別途最適化）。
  - **`stlCullInteriorFaces` はボトルネックではない**（完走する）。速度目的で OFF にしても
    速くならず、埋もれ面が残ってファイルが膨らむだけなので触らない。
- **メモリは問題ではない**（実機 96GB / M3 Max）。過去の「クラッシュ」は主に
  メインスレッド占有による無応答の誤認。書き出しコードは Mac/Windows 共通の C#。

## このセッションで追加した改良（決定稿より後・任意）

`TSDFPrintExporter.cs` / `PrintExportPanel.cs`（※コミット前）に以下を追加済み:

- **`stlBuryTubeHeads`（既定 ON）**: 穴閉じ済みボリュームの SDF で各チューブ最新端を
  `printRadius` ぶん体内へ埋める。突き出るチューブだけ正確に処理し、宙に浮くトレイルは
  そのまま。ON のとき `tubeHeadTrim` より優先（決定稿の 0.04 は fallback になる）。
- **`stlFloorRaise`（既定 0.05）**: 最下点から 5cm 上に床面を上げ、床下の頂点を面まで
  引き上げてクリップ（床下はプリントされない）。カラースパン保持のため三角形は残す。
- **二ポーズ（過去ボディ）**: 「最古ボディを取込」で過去フレームのボディを保存し、
  現フレームの書き出しに2体目の人体色シェルとして同梱（カーブが2ポーズを繋ぐ）。

決定稿を**完全再現**したいときは bury/floorRaise を OFF・`tubeHeadTrim=0.04` に戻す。
決定稿＋改良で行くなら bury ON のままで良い（tri を増やさない）。
