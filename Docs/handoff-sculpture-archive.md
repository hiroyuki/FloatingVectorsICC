# 引き継ぎ: 体験データ（exp_*.glb/usdz）のアーカイブ回収 — Windows 機での作業

2026-11-14 作成（Mac Studio のセッションから引き継ぎ）。
やまなしメディア芸術アワード応募作業（締切 2026-11-19 17:00、応募自体は Mac 側で進行中・この回収はブロッカーではない）の関連タスク。

## 目的

展示期間中に来場者が生成した体験データ（彫刻の glb/usdz）を手元にアーカイブする。

## 背景（Mac 側で確認済みの事実）

- **LFKS（ntticc.lfks.app）のトークンは本番・開発とも失効済み**。2026-11-14 に
  `/api/browse?dir=sculptures&token=...` を両トークンで叩き、どちらも
  `{"error":"invalid or expired token"}` を確認（HTML ページ自体は 200 を返すが一覧 API は不可）。
  サーバー経由での回収はユニバ（坂田さん）にトークン再発行を依頼しない限り不可能。
- 公開ビューア `hyper.ntticc.or.jp/kids2026/unknown-but-yours/` は意図的に一覧ページを持たない
  （`?id=<filename>` を知らないと開けない）。ビューア JS に埋まっている鍵は使わない方針。
- よって **展示機のローカルファイルが唯一の自力回収経路**。
  書き出しは `ExperienceDirector` が `~/Documents/FloatingVectorsPrints/exp_<stamp>.glb` /
  `exp_<stamp>.usdz` に行い、**アップロード成否に関わらずローカルに保持**する契約
  （`Assets/Scripts/Experience/ExperienceDirector.cs:3796` 付近、`Plans/phase7-upload-plan.md`）。

## 作業手順（4070 機・5080 機の両方で）

1. `C:\Users\hori\Documents\FloatingVectorsPrints\` を確認:
   ```powershell
   Get-ChildItem $HOME\Documents\FloatingVectorsPrints | Measure-Object Length -Sum
   Get-ChildItem $HOME\Documents\FloatingVectorsPrints -Filter exp_* | Sort-Object Name
   ```
2. 中身（`exp_*.glb` / `exp_*.usdz`、あれば `print_*.stl` 等も）を Dropbox にコピー:
   `Dropbox\projects\ICC\sculptures_archive\4070\`（または `5080\`）
   — Mac に自動同期させるため。Dropbox の空き容量に注意（数 GB 規模の可能性あり）。
3. 件数・合計サイズ・最古/最新のタイムスタンプをこのファイルに追記してコミットする。

## 回収結果

### 4070 機（2026-11-16 回収）

実際の置き場所は `Dropbox\projects\ICC\Recordings\4070\`（上記手順の `sculptures_archive\4070\` ではない）。
`FloatingVectorsPrints\` 配下が展示機の `~/Documents/FloatingVectorsPrints` のコピー。
※ Dropbox 同期の途中で 2 回集計を誤った（592 → 2,616 ファイル）。以下は Windows 側のコピーと
Dropbox 同期が完全に終わった後（2026-11-16 午後）の値。

| 項目 | 値 |
|---|---|
| ファイル数 | 4,510（glb 2,255 / usdz 2,255） |
| 体験（ユニーク stamp）数 | 2,255 |
| glb + usdz 両方あり | 2,255（glb のみ 0 / usdz のみ 0） |
| 最古 | exp_20260724_222529（開幕前日のテスト） |
| 最新 | exp_20260830_180119（最終日） |
| 合計サイズ |  53G（glb 1 本 約 11〜13 MB） |

日別の体験数（stamp 数）: 7/24:3, 7/25:79, 7/26:86, 7/28:42, 7/29:35, 7/30:68, 7/31:66, 8/1:76,
8/3:4, 8/4:39, 8/5:37, 8/6:61, 8/7:48, 8/8:81, 8/9:108, 8/11:89, 8/12:74, 8/13:90,
8/14:83, 8/15:111, 8/16:113, 8/17:4, 8/18:64, 8/19:55, 8/20:53, 8/21:38, 8/22:84,
8/23:111, 8/25:47, 8/26:61, 8/27:70, 8/28:54, 8/29:103, 8/30:118

同フォルダ直下にはテスト期の `exp_20260718_080937.glb/usdz` と `web_20260709_*.usdz` 4 本も
同梱されている（展示機 Documents 直下にあったもの。Mac 側の iCloud 分と重複の可能性あり）。

glb の中身（Blender で確認）: 単一メッシュ `Sculpture`、頂点カラー（AO 焼き込み済み）、
Principled BSDF の `VertexColour` マテリアル、UV・テクスチャ無し。約 22 万頂点 / 41 万面。

全体のサムネイル一覧: Dropbox `projects/ICC/capture/thumbs_4070.html`（1 体 1 枚、日付順、
stamp 表示）。生成スクリプトは `みちのからだ応募/video/tools/batch_thumbs.py` と `make_contact_sheet.py`。

### 5080 機

未回収。

## 注意

- 体験者の同意範囲は「Web での閲覧・ダウンロード」のみ。**回収したデータを作品ページや
  応募資料に再掲載するのは別途検討が必要**（応募 HANDOFF §7 の制約。アーカイブ保管は問題ない）。
- serial 等マシンローカルな値を git 管理ファイルに書かない（CLAUDE.md 既存ルール）。

## Mac 側に既にあるもの（参考・重複回収不要）

- テスト期の自前書き出し: iCloud Drive `Downloads/exp_*.usdz` 7 本（7/10〜7/24、iPhone AR 保存分）
  + `web_*.usdz` 10 本
- 3D プリント最終データ: Dropbox `projects/ICC/print/`（obj / 3mf、print0729・print0828 ほか）
- raw センサー収録（RCSV）: Dropbox `projects/ICC/Recordings/` — 理論上ここから再書き出しも可能

## 代替経路（ローカルに無かった場合）

ユニバ坂田さん（ICC 経由可）に LFKS トークン再発行を依頼 →
`Docs/lfks-upload-guide.md` の手順の browse API で一覧取得 → 一括ダウンロード。
