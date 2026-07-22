# 新規 Windows アカウントに Claude Code + MCP for Unity を入れる手順

現地オペレーション用に **PC に新しい Windows ユーザーアカウントを作った場合**、Claude Code は
そのアカウントには入っていない。Claude Code / uv / `cc` エイリアス / MCP for Unity 登録は
**すべてユーザープロファイル配下（per-user）** なので、アカウントごとにセットアップし直す必要がある。

4070 セットの `GEMBA` アカウントで実施済み。**5080 セットでも同じ手順を踏む。**

## マシン全体 vs ユーザーごと

| 対象 | スコープ | 新アカウントで再インストール要る？ |
|---|---|---|
| Git (`C:\Program Files\Git`) | マシン全体 | 不要 |
| Node.js (`C:\Program Files\nodejs`) | マシン全体 | 不要 |
| Unity Editor / Hub | マシン全体 | 不要（ライセンスのサインインは要る場合あり） |
| **Claude Code** (`%USERPROFILE%\.local\bin\claude.exe`) | ユーザー | **要** |
| **uv / uvx** (`%USERPROFILE%\.local\bin`) | ユーザー | **要** |
| **`cc` エイリアス**（PowerShell プロファイル） | ユーザー | **要** |
| **UnityMCP の MCP 登録**（`%USERPROFILE%\.claude.json`） | ユーザー | **要** |
| **UnityMCP サーバープロセス**（127.0.0.1:8080） | ログオンセッション | **要**（アカウント切替で落ちる） |
| Claude のログイン認証 | ユーザー | **要**（コピー不可・手動ログイン） |
| Python | — | **不要**。uvx が `mcpforunityserver` 用の CPython を自動取得する |

## 手順

### 1. スクリプトを実行する

**セットアップしたいアカウントでログインした状態で**、リポジトリの

```
Tools\setup-claude-account.cmd        ← ダブルクリックでOK
```

を実行する。`.ps1` を直接ダブルクリックするとメモ帳が開く／ExecutionPolicy で弾かれるので、
**必ず `.cmd` の方**を使う（中で `powershell -ExecutionPolicy Bypass -File ...` を呼んでいる）。

コマンドラインから叩く場合:

```powershell
powershell -ExecutionPolicy Bypass -File <repo>\Tools\setup-claude-account.ps1
```

管理者権限は不要。**プロジェクトパスは指定不要** — スクリプト自身の 1 つ上（= リポジトリルート）を
使うので、`F:\FloatingVectorsICC` でも `D:\GitHub\FloatingVectorsICC` でもそのまま動く。
別の clone を登録したいときだけ `-ProjectDir` を渡す。

> **注意: リポジトリはユーザープロファイルの外に置く**
> `C:\Users\<user>\...` に clone してあると、その中身は他アカウントから読めない（ACL）ので、
> 新アカウントから `.cmd` を実行できない。共有ドライブ（4070 セットの `F:\`、5080 機の
> `D:\GitHub\` など）に置くこと。

> **注意: `.ps1` の中身をコンソールに貼り付けて実行しない**
> `$PSScriptRoot` が空になり、リポジトリの場所を自力で解決できない
> （フォールバックでカレントディレクトリを使うので、意図しない場所を登録しうる）。
> `.cmd` から起動するか `-File` で渡すこと。

スクリプトがやること（7ステップ、既存ファイルは上書きせず警告する）:

1. Claude Code インストール（`claude.ai/install.ps1`）＋ ユーザー PATH に `.local\bin` 追加
2. uv / uvx インストール（`astral.sh/uv/install.ps1`）
3. PowerShell プロファイル（5.1 / 7 の両方）に `cc` と `Start-UnityMcp` を定義
4. `%USERPROFILE%\.claude\settings.json` を作成（model / effortLevel / theme など）
5. UnityMCP サーバー起動 → 8080 が listen しているか確認
6. プロジェクトメモリのコピー（`-MemorySource` にコピー元があれば。無ければスキップ）
7. プロジェクトディレクトリで `claude mcp add --transport http UnityMCP http://127.0.0.1:8080/mcp`

### 2. スクリプト後の手作業（自動化できない分）

1. **PowerShell を開き直す**（プロファイル反映）
2. `cd F:\FloatingVectorsICC` → `claude` → **ブラウザで Claude にログイン**
   （認証情報はアカウント間でコピーできない）
3. codex プラグインが要るなら:
   ```
   claude plugin marketplace add openai/codex-plugin-cc
   claude plugin install codex@openai-codex
   ```
4. 以後は `cc` で起動（= `claude --dangerously-skip-permissions`）

### 3. 旧アカウントのプロジェクトメモリを引き継ぐ場合（任意）

Claude のプロジェクトメモリ（`~/.claude/projects/<プロジェクトキー>/memory/`）は per-user。
**管理者でない限り他ユーザーのプロファイルは読めない**ので、**アカウントを切り替える前に**
旧アカウント側で共有場所へ出しておく:

```
Tools\export-claude-memory.cmd        ← 旧アカウントでダブルクリック
```

プロジェクトキー（`F--FloatingVectorsICC` / `C--Users-hori-Documents-GitHub-FloatingVectorsICC` など）は
リポジトリのパスから自動で導出する。書き出し先は
`C:\Users\Public\claude-memory\<キー>\memory`。

新アカウントでセットアップスクリプトを流すと step 6 が拾って取り込む。旧と新でリポジトリの
パスが違うとキーも変わるが、共有場所に候補が 1 つしか無ければそれを使う（複数あるときは
`-MemorySource` で明示）。取り込み後、共有場所のコピーは消してよい:

```powershell
Remove-Item -Recurse 'C:\Users\Public\claude-memory'
```

## 運用メモ

- **UnityMCP サーバーはログオンセッションに紐づく**。アカウントを切り替えたりサインアウトすると
  8080 のプロセスが落ちる。Claude 側で `mcp__UnityMCP__*` が "Unable to connect" になったら
  PowerShell で `Start-UnityMcp`（プロファイルで定義済み）。Unity Editor 側のプラグインは
  ~30 秒で自動再接続する
- サーバーログ: `%LOCALAPPDATA%\UnityMCP\Logs\unity_mcp_server.log`
- サーバー起動コマンドの実体（PyPI パッケージ名は `mcpforunityserver`、エントリポイントは
  `mcp-for-unity`。素の `uvx mcp-for-unity` は失敗する）:
  ```
  uvx --from mcpforunityserver mcp-for-unity --transport http --http-url http://127.0.0.1:8080/mcp
  ```
- `.ps1` は **UTF-8 BOM 付き**で保存すること。BOM 無しだと Windows PowerShell 5.1 が cp932 として
  読んで日本語コメントが壊れ、パースエラーになる
