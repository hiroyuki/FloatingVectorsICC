# 引き継ぎ: やまなしメディア芸術アワード応募作業（マシン移動用）

2026-11-16 作成、Mac Studio のセッションから。**締切: 2026-11-19（木）17:00 JST**、
応募フォームは https://y-artaward.jp 。

このリポジトリは public のため、応募 PR 文・個人的な経緯を含む資料は git に置かない。
**正本は Dropbox `projects/ICC/みちのからだ応募/`** に置いた（Dropbox 同期で各マシンに届く）:

- `HANDOFF.md` … 応募全体の引き継ぎ書（PR 文確定稿 §4、要項の要点 §5、クレジット §6、
  記録映像の編集案 §11、チェックリスト §12）。**作業前に必ずこれを読む**
- `floatingvectors_site/` … 作品ページのデプロイ元一式（index.html + video.mp4 + images/）

## 現状（完了済み）

- 作品ページは **Cloudflare Pages で公開済み・完成状態**:
  https://floatingvectors.com/michinokarada/ （ルートは /michinokarada/ へ 302、
  https://floatingvectors.pages.dev/michinokarada/ も同内容で生きている）
  - **2026-11-16 に独自ドメイン接続済み**: お名前.com の NS を Cloudflare
    （brenda / micah.ns.cloudflare.com）へ変更、ゾーンは Free プラン・レコードは
    apex と www の CNAME → floatingvectors.pages.dev（Proxied）の 2 件のみ。
    Pages の Custom domains は両方 Active / SSL enabled。メール系レコードは
    お名前.com 既定値だったので削除済み（このドメインでメールは未使用）
  - クレジット確定（撮影:丸尾隆一、連絡先行は削除）、アーティスト写真設置、
    記録映像（8/8・65秒・1テイク）を自前ホスティングの mp4 モーダルで埋め込み済み
  - og:url / og:image / canonical は https://floatingvectors.com/michinokarada/ の絶対URL
- Discord（ICC キッズ・プログラム 2026）の未読分は全件読了（11/14）。
  MTG メモ 4 本は Mac の `~/Downloads/堀さんとMTG - *.md`
- 体験データ回収の引き継ぎは別紙 `Docs/handoff-sculpture-archive.md`（LFKS トークンは失効確認済み、
  回収は展示機ローカル `~/Documents/FloatingVectorsPrints` から）

## 残作業（締切まで）

1. **フォーム送信（〜11/19 17:00）**: PR 文は Dropbox の HANDOFF.md §4 の確定稿。
   作品 URL は https://floatingvectors.com/michinokarada/
   （2027-03-31 まで閲覧可能を維持する要項あり — ドメイン期限は 2027-11-13、
   Pages は無料枠なので条件を満たす）。
   サムネイルは `floatingvectors_site/michinokarada/images/hero.jpg` 使用可
2. **赤羽さん・鹿島田さんに応募を伝える**（要項「共同制作者の許諾」。未連絡）
3. **ICC に写真・映像使用の一言連絡**（「なんでも使っていい」と言われているが形に残す）
4. ~~NS 移管 + Pages カスタムドメイン追加~~ — 2026-11-16 完了（上記）
5. （任意）記録映像の本編集版（HANDOFF.md §11 の 2 分構成）ができたら video.mp4 を差し替え

## 再デプロイ手順（ページを直したとき）

```
cd <Dropboxの floatingvectors_site>
set CLOUDFLARE_ACCOUNT_ID=86cffa015396f5c2afcb18680b937800
npx wrangler pages deploy . --project-name floatingvectors --branch main
```

- 初回は `npx wrangler login` でブラウザ認可（アカウントは Hori@p4n.jp 側を選ぶ）
- Node 22 未満のマシンでは `wrangler@3` を指定（Mac は Node 20 のためそうしていた）
- ファイル 1 つ 25MiB 上限（video.mp4 はこのために 13.5MB に圧縮済み。4K 元データは
  Mac の `~/Downloads/michinokarada_記録映像_20260808_4K元データ.mov`）
