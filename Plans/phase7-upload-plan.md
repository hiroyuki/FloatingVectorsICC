# Phase 7 実装プラン: LFKS 実アップロード + 仕上げ（改訂1）

Parent plan: `Plans/experience-flow-plan.md`（Phase 1-6 実装済み・main マージ済み）。ライブカメラ 4 台接続済み・ネットワーク有効のため、**実アップロードと QR 実 URL まで本セッションで検証**する。

## 信頼境界とトークンの扱い（親プラン確定事項の具体化）

- `Assets/StreamingAssets/lfks/upload.ps1` — LFKS サーバーから**一度だけ手動ダウンロードしたローカルコピー**をコミット（毎回の curl 実行はしない）。実行前に **SHA-256 を config の既知値（平文で持つ、秘密ではない）と照合**、不一致なら実行せず error（Fault ログ）
- **ラッパ廃止**（Codex 指摘2対応 — ラッパ自体が token 信頼境界に入り、それも pin しなければ意味がない）: C# が
  `powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "& '<pinned upload.ps1>' -Token $env:LFKS_TOKEN -File '<path>' -Directory '<dir>' -Json"`
  を組み立てる。`$env:LFKS_TOKEN` は**リテラル文字列**としてコマンドラインに乗る（token 実値は乗らない、展開は子 PowerShell プロセス内）。実行されるスクリプトは SHA 照合済みの upload.ps1 **1 本だけ**。パスはシングルクォートで囲み、`'` を含むパスは reject（我々の生成パスに `'` は入らない — 防御的ガード）
- **token の供給**: ① `<persistentDataPath>/lfks-token.txt`（gitignore 圏外のユーザーローカル、優先） ② 環境変数 `LFKS_TOKEN`。C# は読み取った token（**Trim() し、空なら reject** — Codex 指摘3）を `ProcessStartInfo.EnvironmentVariables["LFKS_TOKEN"]` で子プロセスにのみ渡す。**config アセット・コマンドラインには載せない**
- 注: リポジトリの `Docs/lfks-upload-guide.md` に token が平文でコミット済み（ユーザー判断）— 本フェーズはそれを変更しないが、コードは guide に依存しない

## 変更・新規

### 1. `Experience/Publishing/LfksUploadPublisher.cs`（新規）

```csharp
public sealed class LfksUploadPublisher : ISculptureResultPublisher
{
    public LfksUploadPublisher(string uploadScriptPath, string expectedScriptSha256,
                               string token, string remoteDirectory, float timeoutSeconds = 60f);
    public Task<PublishResult> PublishAsync(string glbPath, string usdzPath, CancellationToken ct);
}
```

- `PublishAsync`: ① upload.ps1 の SHA-256 照合（不一致 → Success=false, Error に期待/実測ハッシュ。**実行しない**）② glb → usdz の順に 1 ファイルずつ `RunUploadAsync`。両方成功で URL 2 本返す
- `RunUploadAsync`（1 ファイル）: 上記 `-Command` 形式の `Process`（stdout/stderr redirect、LFKS_TOKEN は環境変数で注入）。**デッドロック回避**（Codex 指摘1対応）: `Start()` 直後に `StandardOutput.ReadToEndAsync()` / `StandardError.ReadToEndAsync()` を**先に開始**し、`WaitForExitAsync`（相当）→ 両 Task を await（パイプが埋まっても hang しない）。**timeout 60s + 1 リトライ**（親プラン確定）。ct キャンセル/タイムアウトで `Process.Kill(entireProcessTree: true)`
- stdout の行から **「trim して `{` で始まり `}` で終わる」最後の JSON らしい行**を選び（Codex 指摘4 — 進捗行・空行を吸収）`JsonUtility.FromJson<LfksResult>`（DTO: id/name/relativePath/size/downloadUrl/fs）。downloadUrl 空 → 失敗扱い
- 失敗時: ローカル glb/usdz は**保持**（既に `~/Documents/FloatingVectorsPrints` に正規ファイルとして存在 — Phase 1 のアトミック書き込み契約）、Error に stderr 要約

### 2. `Experience/ExperienceConfig.cs` — 追加フィールド

- `uploadScriptSha256`（平文 hex）
- `lfksRemoteDirectory`（既定 "sculptures"）
- `publishTimeoutSeconds`（既定 60）

### 3. `Experience/ExperienceDirector.cs` — publisher 選択

- `ExportAndPublish` 内: `config.dryRunPublish ? DryRunPublisher : LfksUploadPublisher`（wrapper/script パスは `Application.streamingAssetsPath/lfks/...`、token 解決は director 側ヘルパー: persistentDataPath/lfks-token.txt → env）。token 未解決かつ !dryRun → ExportFailed 扱い（謝罪文言 → Attract、ローカルファイル保持）
- QR URL は実 downloadUrl（既存 qrUrlKind 選択のまま）

### 4. アセット取り込み手順（実装時に実施・検証）

1. `curl -o Assets/StreamingAssets/lfks/upload.ps1 https://ntticc.lfks.app/upload.ps1`（一度だけ。2026-07 に本番ドメイン ntticc.lfks.app 版へ更新済み）
2. 取得物を**目視レビュー**（外部送信先が LFKS ドメインのみであること、危険な操作が無いこと）
3. SHA-256 を計算し config 既定値に記載
4. token を `<persistentDataPath>/lfks-token.txt` に書く（guide から。リポジトリ外）

## 検証（実ネットワーク）

1. コンパイル + 46 テスト green
2. **Publisher 単体**: 小さなテストファイルで PublishAsync 実行 → 返った downloadUrl に `curl -I` で 200/OK を確認。SHA 不一致ケース（config を1文字変える）→ 実行されず Error。token 無しケース → Error
3. **E2E 実アップロード**: dryRunPublish=false で体験ループ 1 周（ライブカメラ）→ exp_*.glb/.usdz が LFKS に上がり、QR に**実 downloadUrl** が表示される（スクリーンショット）→ downloadUrl を curl で検証
4. タイムアウト/キャンセル: publishTimeoutSeconds を極小にして失敗経路（謝罪 → Attract、ローカル保持）を確認
5. ソーク: 短時間の連続ループ（3周以上、Phase 6 で複数周は実証済み）でリーク/停止が無いこと

## 仕上げ項目（本フェーズで判断だけ、実施は現地）

- Bloom: `DefaultVolumeProfile` の intensity 0 のまま（DwellSphere グローは HDR tint で bloom 無しでも視認可 — Phase 4 検証済み）。現地の暗環境で判断
- 文言・タイミングの最終調整、球の配置/ミニチュアの見た目 → ExperienceConfig で現地チューニング
- Codex 指摘の deferred 項目（TSDF 部分バッチの seam、USB 物理抜きテスト、同一データ検出の実データ確認、入場遷移 <2s 実測）→ 現地セットアップチェックリストとして `Docs/onsite-checklist.md` に書き出す

## リスク

- upload.ps1 の中身が想定外（マルチパート等で stdout に複数行）→ 「最後の JSON 行」パースで吸収（guide の実行例で確認済みの出力形式）
- token 期限 2026-10-30 — 展示期間に合わせ再発行はユーザー管理
- staging サーバーの安定性 — リトライ 1 回 + 失敗時ローカル保持で運用カバー
