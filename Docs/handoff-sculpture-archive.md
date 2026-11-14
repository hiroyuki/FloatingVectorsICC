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
