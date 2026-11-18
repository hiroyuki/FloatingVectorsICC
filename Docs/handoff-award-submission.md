# 引き継ぎ: やまなしメディア芸術アワード応募作業（マシン移動用）

2026-11-16 作成、Mac Studio のセッションから。**締切: 2026-11-19（木）17:00 JST**、
応募フォームは https://y-artaward.jp 。

このリポジトリは public のため、応募 PR 文・個人的な経緯を含む資料は git に置かない。
**正本は Dropbox `projects/ICC/みちのからだ応募/`** に置いた（Dropbox 同期で各マシンに届く）:

- `HANDOFF.md` … 応募全体の引き継ぎ書（PR 文確定稿 §4、要項の要点 §5、クレジット §6、
  記録映像の編集案 §11、チェックリスト §12）。**作業前に必ずこれを読む**
- `floatingvectors_site/` … 作品ページのデプロイ元一式（`_redirects` + `unknownbutyours/{index.html, video.mp4, images/}`）

## 現状（完了済み）

- 作品ページは **Cloudflare Pages で公開済み・完成状態**:
  https://floatingvectors.com/unknownbutyours/ （ルートは /unknownbutyours/ へ 302、
  https://floatingvectors.pages.dev/unknownbutyours/ も同内容で生きている）
  - **2026-11-16 にパスを /michinokarada/ → /unknownbutyours/ に変更**（ユーザー指示）。
    旧パスは `_redirects` で 301（`/michinokarada/*` → `/unknownbutyours/:splat`）。
    Dropbox のデプロイ元も `floatingvectors_site/unknownbutyours/` に改名済み
  - **2026-11-16 に独自ドメイン接続済み**: お名前.com の NS を Cloudflare
    （brenda / micah.ns.cloudflare.com）へ変更、ゾーンは Free プラン・レコードは
    apex と www の CNAME → floatingvectors.pages.dev（Proxied）の 2 件のみ。
    Pages の Custom domains は両方 Active / SSL enabled。メール系レコードは
    お名前.com 既定値だったので削除済み（このドメインでメールは未使用）
  - クレジット確定（撮影:丸尾隆一、連絡先行は削除）、アーティスト写真設置、
    記録映像（8/8・65秒・1テイク）を自前ホスティングの mp4 モーダルで埋め込み済み
  - og:url / og:image / canonical は https://floatingvectors.com/unknownbutyours/ の絶対URL
- Discord（ICC キッズ・プログラム 2026）の未読分は全件読了（11/14）。
  MTG メモ 4 本は Mac の `~/Downloads/堀さんとMTG - *.md`
- 体験データ回収の引き継ぎは別紙 `Docs/handoff-sculpture-archive.md`（LFKS トークンは失効確認済み、
  回収は展示機ローカル `~/Documents/FloatingVectorsPrints` から）

## 応募済み（2026-11-18）

- 2026-11-18 午前にフォームから応募し、自動返信メールで受付内容を確認した。
  区分：個人／発表形式：展示／作品時間：4 分 00 秒／作品 URL：https://floatingvectors.com/unknownbutyours/
- 記録映像の最終版は `video/v29_rough_1118.mp4`（約 2 分 46 秒）
- **作品ページは 2027-03-31 まで、同じ URL でログインなしで見られる状態を保つ**（要項）
- 入選発表は 2026 年 12 月下旬
- 赤羽さん・鹿島田さんへの連絡、ICC への写真・映像使用の連絡はどちらも済み。応募まわりの残作業は無し

## 2026-11-18 の追加・変更

- 記録映像は `video/v29_rough_1118.mp4`（約 2 分 46 秒。最後の 9 枚表示に「会期を通して、たくさんのユニークなかたちが生まれました。」を重ねた）。ページの `video.mp4` も同じ内容。
  変更の経緯は Dropbox の `video/CUTLIST.md` 冒頭
- ページ: OG 画像を `images/og.jpg`（HERO 写真 + 題名 + 撮影クレジット、1200×630）に変更。
  HERO のクレジットは画像の右下に重ねた。動画の表紙は 4 枚表示のコマ（`video-poster.jpg`）
- 「かたち（AR体験）」: three.js の表示の右下に「ARで見る」→ `unknownbutyours/ar/?n=1〜9`。
  iOS は USDZ を AR Quick Look、Android は Draco 圧縮の GLB を Scene Viewer で開く。
  PC では二次元コード（`ar/qr/`、`npx qrcode` で生成）。**実機での AR 起動は未確認**
- 3D プリントの文末に「（ダンサー：小暮香帆）」
- 5080 機のアーカイブは回収済み（`Docs/handoff-sculpture-archive.md`）

## 残作業（締切まで）

1. ~~フォーム送信（〜11/19 17:00）~~ — 2026-11-18 送信済み。: PR 文は Dropbox の HANDOFF.md §4 の確定稿。
   作品 URL は https://floatingvectors.com/unknownbutyours/
   （2027-03-31 まで閲覧可能を維持する要項あり — ドメイン期限は 2027-11-13、
   Pages は無料枠なので条件を満たす）。
   サムネイルは `floatingvectors_site/unknownbutyours/images/hero.jpg` 使用可
2. ~~赤羽さん・鹿島田さんに応募を伝える~~ — 連絡済み
3. ~~ICC に写真・映像使用の一言連絡~~ — 連絡済み
4. ~~NS 移管 + Pages カスタムドメイン追加~~ — 2026-11-16 完了（上記）
5. （任意）記録映像の本編集版（HANDOFF.md §11 の 2 分構成）ができたら video.mp4 を差し替え

## 再デプロイ手順（ページを直したとき）

```
cd <Dropboxの floatingvectors_site>
set CLOUDFLARE_ACCOUNT_ID=86cffa015396f5c2afcb18680b937800
npx wrangler pages deploy . --project-name floatingvectors --branch main
```

- 初回は `npx wrangler login` でブラウザ認可（アカウントは Hori@p4n.jp 側を選ぶ）。
  Mac Studio は 2026-11-16 時点で refresh token が生きており再ログイン不要だった
- **video.mp4 が Dropbox 上で 0 byte（未同期 placeholder）のままデプロイしない**こと。
  空ファイルで上書きされる。`stat -f %z` で 13514039 を確認してから。間に合わなければ
  公開中の URL から `curl -o video.mp4 https://floatingvectors.com/unknownbutyours/video.mp4`
  で取り直せる
- Node 22 未満のマシンでは `wrangler@3` を指定（Mac は Node 20 のためそうしていた）
- ファイル 1 つ 25MiB 上限（video.mp4 はこのために 13.5MB に圧縮済み。4K 元データは
  Mac の `~/Downloads/michinokarada_記録映像_20260808_4K元データ.mov`）
