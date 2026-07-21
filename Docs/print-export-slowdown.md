# 印刷書き出しが12分超で固まる問題 — 原因と対処

調査日: 2026-07-21 / 検証機: Windows (RTX 4070 セット)

Mac で 3MF 書き出しが極端に遅い・巨大になる件を追った結果、**Mac 固有の問題ではなく
コミット済みシーンの設定が原因**と判明した。Windows でも同じ設定なら同じだけ遅い。

> **解決済み (2026-07-21, Mac セッション)**: 下記 3 機能のうち O(n²) だった 2 つ
> (`stlContactBeads` / `stlCullInteriorFaces`) は**コード自体から削除**、残る
> `stlCurveSupportPillars` はコード既定を `false` にした（機能は温存）。
> `main.unity` 作業ツリーも 3 フラグ `0`。もう「フラグを戻し忘れる」再発経路は無い。
> 詳細は末尾「## 解決」。

---

## TL;DR

高密度 (stride 2) で書き出しがハングする原因は、点密度に対して固定セルの空間ハッシュを
使う 3 機能が**実質 O(n²)** になること。恒久対処として:

| 機能 | 旧コミット値 | 対処 (2026-07-21) |
|---|---|---|
| `stlContactBeads` | `1` | **コードごと削除**（`AppendContactBeads` 撤去）。「コブだらけ」で却下済み |
| `stlCullInteriorFaces` | `1` | **コードごと削除**（`CullInteriorChainTris` 撤去）。`8188b13` の実験フラグ |
| `stlCurveSupportPillars` | `1` | **コード既定 `false`**（機能は温存・`AppendBranchStrut` 共有のため）。「ストラット案は全没」|

`printSeedStride` が 40 のうちは ON でも実害が出ないため、今まで表面化しなかった。
**stride を 2 に下げた瞬間に破綻していた。**

---

## 症状

`printSeedStride = 2` で `ExportObj` / `Export3mf` を実行すると、
`TSDFPrintExporter.AppendContactBeads` でメインスレッドが 12 分以上ブロックされる。
Unity はハングではなく CPU 約 1.9 コアを回し続け、RAM は 19 GB まで伸びる。

---

## 計測データ

すべて **同一テイク・同一フレーム (f1937) の同一 paused 状態**から書き出した実測値。

テイク: `D:\Dropbox\projects\ICC\Recordings\RecordingBase\2026-07-14_15-09-52`
(`useV11sBodies = true`、`SeekToFrame(1937)` / preroll 75s)

| RUN | stride | 3フラグ | tubes | linkPts | 生成 tris | 所要 |
|---|---|---|---|---|---|---|
| C（コミット値そのまま） | 40 | **ON** | 881 | 192,058 | 884,979 | 9.8 s |
| D（C から3フラグのみ OFF） | 40 | OFF | 882 | 192,276 | 437,836 | **3.6 s** |
| A（C の stride のみ 2） | 2 | **ON** | 18,160 | 2,624,198 | — | **12分超で打切** |
| F（D の stride のみ 2） | 2 | OFF | 17,258 | 3,762,244 | 6,073,948 | 34.5 s |
| **B（決定稿）** | **2** | **OFF** | 17,288 | 2,968,316 | 2,772,916 | **10.9 s** |

> RUN A のみ完走させていない。12 分時点で打ち切り、ハートビート実測レート
> (131,209 / 2,624,198 点を 31,393 ms) から**ビーズ段だけで約 628 秒**と外挿した。

### なぜ stride を下げると爆発するか

ビーズもサポートも空間ハッシュで近傍探索するが、**セルサイズが点密度に対して固定**。

- ビーズ: `cell = max(0.03, printRadius * 4)` → 半径 9 mm なら **3.6 cm**
- サポート: `cell = max(0.05, stlSupportSearchRadius)` → **15 cm**

コストは `点数 × 27セル × 平均バケット占有数`。stride を 40 → 2 にすると
点数が 20 倍になると同時に**バケット占有も 10 倍**になるため、積で効く。

```
stride 40 :   192,058 pts, mean bucket  21.4, max   410 →      110,806,568 pair tests →   2.5 s
stride  2 : 2,624,198 pts, mean bucket 208.1, max 5,527 →   14,743,732,365 pair tests → 628 s
```

つまり線形に見えて**実質二乗**。133 倍に膨らむ。

---

## なぜ「昨日まで問題なかった」のか

**リポジトリ側は何も変わっていない。** 07-19 のセッション記録に当時の実値が残っていた。

```
stlCurveSupportPillars = false
stlContactBeads        = false
stlCullInteriorFaces   = false
printSeedStride        = 2
→ print_20260719_165742_f05299.obj
   3,052,591 tris (169,699 body + 29,999 tubes / 0 bridges + 71 supports + floor)
   210 MB, total 17.5s
```

07-19 の作業中にビーズ ON の遅さ（`contact beads: 75267 welds → total 371.9s`、
`30513 welds → 168.2s`）を踏んで OFF に倒し、**以降すべての納品物を
stride 2 + 3フラグ OFF で書き出していた**。

**しかしその設定は一度もシーンに保存されなかった。**
`main.unity` には 3 フラグが `1` のまま残ってコミットされている。

- 07-19〜07-20: 同じエディタセッションが生きている間は手動 OFF が効いていた → 速い
- Unity 再起動後: コミット値に戻る → stride 2 と組み合わさって 12 分超
- **Mac: 手動 OFF を一度も持っていない。pull したシーンのまま＝常に ON**

`3452ca2`（"chore(scene): save scene WIP … + settings drift"）が悪い値を焼き付け、
それを打ち消していたエディタ上の一時設定が再起動で消えた、というのが正確な因果。

---

## 検証済みの推奨構成（RUN B）

```
printSeedStride        = 2
webCurveTolerance      = 0.003     # 三角形を落とす主レバー。stride ではない
printRadius            = 0.009     # 9mm
stlCurveSides          = 4         # 四角バー
tubeTailTaper          = 1         # テーパーなし・全長均一
tubeAlignUp            = true      # ダイヤ向き
tubeHeadTrim           = 0.04      # 新しい端が体表を突き抜けない
stlBlackLineRatio      = 0         # 全白
stlIncludeBody         = true
stlAutoCloseHoles      = true
stlIncludeBridges      = false     # 黒ブリッジ株はノイズ
stlContactBeads        = false     # ★
stlCullInteriorFaces   = false     # ★
stlCurveSupportPillars = false     # ★
stlRootWireframe       = false
fuseSminK              = 0
stlLinkIsolatedChains  = true
stlFloorPillarBand     = 0.05
stlFloorPillarRadius   = 0.012
targetHeightMm         = 300
```

**CurveSamples (`BonePoseHistory.historySamples`) = 32 が必須**（シーン保存済み）。

### 出力（検証実施済み）

```
print_20260721_002007_f01937.obj         190 MB   2,772,916 tris   10.9 s
print_20260721_002007_f01937_final.3mf  31.4 MB   2,867,662 tris   50.7 s
```

3MF の中身も機械的に検証済み:

| 項目 | 結果 |
|---|---|
| サイズ | **31.4 MB**（50MB 制限をクリア） |
| OPC 構造 | `[Content_Types].xml` / `_rels/.rels` / `3D/3dmodel.model` 正常 |
| 単位 | `millimeter` |
| マテリアル | Body `#000000` / Curves `#FFFFFF` / Human `#669EFF` の3色 |
| スパン | Body 1,900 / Curves 2,660,256 / Human 205,506 tris |
| 頂点インデックス | 最大 1,539,797 < 頂点数 1,539,798（範囲内） |
| 寸法 | 276.5 × 300.0 × 276.5 mm（`targetHeightMm=300` どおり） |
| ボディ修復 | MeshFix が 71,194v/110,760f → 102,743v/205,506f watertight |

> スライサでの目視確認のみ未実施。

### 3MF の作り方

Unity の `Export3mf` を直接使わず、**`ExportObj` → `fix_body.py`** を通す。

```bash
python eval/tools/fix_body.py input.obj output.3mf
```

`fix_body.py` は Human スパンだけを MeshFix で watertight 再構築し
（Curves/Body スパンは素通し）、3色 basematerials 入りの Standard 3MF を書き出す。
カメラ死角（頭頂・肩・腕）の穴は voxel 閉包でもファンキャップでも塞がらないため、
この後処理は必須。

pymeshfix は **pyenv 3.11.1** に入っている（Windows 機）:
`C:\Users\hori\.pyenv\pyenv-win\versions\3.11.1\python.exe`

---

## 切り分け用ログ（旧）

> **注 (2026-07-21)**: 調査中に足していた段別タイマー（`beads:` / `interior cull:` など）は
> `AppendContactBeads` / `CullInteriorChainTris` ごと削除したため、現行コードには無い。
> 書き出し末尾のサマリ（tris 内訳・所要 ms・出力 MB）だけが残っている。
> 将来また段別に切り分けたくなったら、`AssembleShells` の各ステージに
> `Stopwatch` を一時的に挿すこと。

Mac 側のログ: `~/Library/Logs/Unity/Editor.log`

---

## 書き出し済み STL/OBJ の stride を逆算する方法

tube 1 本は独立した閉じた殻なので、OBJ の `Curves` スパンの連結成分数がそのまま
tube 本数になる。`printSeeds = seedCount / stride` が上限。

```
07-19 f5299: Curves 連結成分 29,999 → 上限 30,000 = 60,000 / 2  → stride 2
今日 RUN D  : Curves 連結成分    443 → tubes 882（blackRatio 0.5 で白黒に二分）→ stride 40
```

---

## Mac 側で確認してほしいこと

1. **`main.unity` を pull 後、3フラグが `0` になっているか**（本 md と同時にはまだ直していない）
2. Mac の 540 MB 3MF の正体。上の RUN F でも 6,073,948 tris / OBJ 430 MB 止まりで、
   540 MB の 3MF はこの構成では説明がつかない。Unity Console の
   `OBJ:` / `3MF:` 行（tris と MB が出る）を 1 行控えてほしい
3. `SensorRecorder.playbackFolderPathMacOverride` が
   **`2026-07-14_13-33-44` という別テイク**を指している。
   Windows 側 (`playbackFolderPath`) は `2026-07-14_15-09-52`。比較時は要注意

---

## 解決 (2026-07-21, Mac セッション)

恒久対処としてフラグ運用ではなくコードから断った（Unity 再コンパイル・エラー 0 確認済み）。

- `stlContactBeads` フィールドと `AppendContactBeads` メソッドを**削除**
  （呼び出し・`wantLinkData` 条件も撤去）。
- `stlCullInteriorFaces` フィールドと `CullInteriorChainTris` メソッドを**削除**
  （`chainTriRanges` / 不要になった `triBefore` も撤去）。
- `stlCurveSupportPillars` は温存しコード既定を `true → false` に変更
  （`AppendBranchStrut` をサポート柱側で共有しているため機能自体は残す）。
- 撤去した 2 機能と `stlLinkIsolatedChains` が共有していた `linkPts` 生成は、
  まだ使う `stlLinkIsolatedChains` 用に残置。
- `main.unity` 作業ツリーは 3 フラグとも既に `0`
  （`stlContactBeads` / `stlCullInteriorFaces` は orphan キーとして次のシーン保存で自動的に落ちる）。

Mac の書き出し実測（改良・床カット込み）: `ExportObj` 18.4s → `fix_body.py` →
`_final.3mf` 約 37 MB / 約 53 s。

## 未対応

- [ ] スライサ（Bambu Studio）での目視確認
- [ ] Mac 側 540 MB の原因特定（撤去済み機能が原因だった可能性が高い＝再現しないはず）
