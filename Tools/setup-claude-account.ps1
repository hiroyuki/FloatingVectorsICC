# setup-claude-account.ps1
#
# Windows の新しいユーザーアカウント（例: 現地オペレーション用の GEMBA）に
# Claude Code + cc エイリアス + MCP for Unity を一括セットアップする。
#
# 使い方: セットアップしたいアカウントでログインし、PowerShell で
#   powershell -ExecutionPolicy Bypass -File <repo>\Tools\setup-claude-account.ps1
#
# 管理者権限は不要（すべてユーザープロファイル配下にインストールする）。
# 手順の背景は Docs/claude-account-setup.md を参照。

[CmdletBinding()]
param(
    # Unity プロジェクトのパス（UnityMCP をこのプロジェクトに登録する）
    [string] $ProjectDir = 'F:\FloatingVectorsICC',

    # 旧アカウントから持ち込む Claude プロジェクトメモリのコピー元。
    # 存在しなければスキップする。
    [string] $MemorySource = 'C:\Users\Public\claude-memory\F--FloatingVectorsICC\memory',

    # メモリのコピー先プロジェクトキー（persistentDataPath ではなく Claude 側のキー）
    [string] $MemoryProjectKey = 'F--FloatingVectorsICC'
)

$ErrorActionPreference = 'Stop'
function Step($m) { Write-Host "`n=== $m ===" -ForegroundColor Cyan }
function Ok($m)   { Write-Host "  [ok] $m" -ForegroundColor Green }
function Warn($m) { Write-Host "  [!!] $m" -ForegroundColor Yellow }

$LocalBin = Join-Path $env:USERPROFILE '.local\bin'

# ------------------------------------------------------------------
Step '1/7  Claude Code をインストール'
if (Test-Path (Join-Path $LocalBin 'claude.exe')) {
    Ok 'すでにインストール済み'
} else {
    Invoke-RestMethod https://claude.ai/install.ps1 | Invoke-Expression
    Ok 'インストール完了'
}

# PATH に .local\bin を通す（ユーザー環境変数 + 現セッション）
$userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
if ($userPath -notlike "*$LocalBin*") {
    [Environment]::SetEnvironmentVariable('Path', "$userPath;$LocalBin", 'User')
    Ok "PATH に $LocalBin を追加"
}
if ($env:Path -notlike "*$LocalBin*") { $env:Path = "$env:Path;$LocalBin" }

# ------------------------------------------------------------------
Step '2/7  uv / uvx をインストール（MCP for Unity サーバー用）'
if (Test-Path (Join-Path $LocalBin 'uvx.exe')) {
    Ok 'すでにインストール済み'
} else {
    Invoke-RestMethod https://astral.sh/uv/install.ps1 | Invoke-Expression
    Ok 'インストール完了'
}
# python は別途不要 — uvx が mcpforunityserver 用の CPython を自動取得する

# ------------------------------------------------------------------
Step '3/7  PowerShell プロファイルに cc / Start-UnityMcp を定義'
$profileBody = @'
# --- Claude Code ---
function cc { claude --dangerously-skip-permissions @args }

# MCP for Unity サーバー (http://127.0.0.1:8080/mcp) を起動する
function Start-UnityMcp {
    $listening = (Get-NetTCPConnection -LocalPort 8080 -State Listen -ErrorAction SilentlyContinue)
    if ($listening) { Write-Host 'UnityMCP server は既に 8080 で稼働中'; return }
    Start-Process -WindowStyle Hidden uvx -ArgumentList @(
        '--from','mcpforunityserver','mcp-for-unity',
        '--transport','http','--http-url','http://127.0.0.1:8080/mcp')
    Write-Host 'UnityMCP server を起動しました（Unity Editor 側は ~30 秒で自動再接続）'
}
'@

# Windows PowerShell 5.1 と PowerShell 7 の両方に配置しておく
$profilePaths = @($PROFILE.CurrentUserCurrentHost)
$pwshProfile = Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'PowerShell\Microsoft.PowerShell_profile.ps1'
if ($profilePaths -notcontains $pwshProfile) { $profilePaths += $pwshProfile }

foreach ($p in $profilePaths) {
    $dir = Split-Path $p -Parent
    if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Force -Path $dir | Out-Null }
    $existing = if (Test-Path $p) { Get-Content $p -Raw } else { '' }
    if ($existing -match 'function cc ') {
        Ok "cc は定義済み: $p"
    } else {
        Add-Content -Path $p -Value $profileBody -Encoding utf8
        Ok "追記: $p"
    }
}
# 現セッションでも即使えるように
Invoke-Expression $profileBody

# ------------------------------------------------------------------
Step '4/7  Claude の settings.json'
$claudeDir = Join-Path $env:USERPROFILE '.claude'
if (-not (Test-Path $claudeDir)) { New-Item -ItemType Directory -Force -Path $claudeDir | Out-Null }
$settingsPath = Join-Path $claudeDir 'settings.json'
if (Test-Path $settingsPath) {
    Warn "既存の settings.json は上書きしません: $settingsPath"
} else {
    @'
{
  "model": "claude-opus-4-8[1m]",
  "enabledPlugins": {
    "codex@openai-codex": true
  },
  "extraKnownMarketplaces": {
    "openai-codex": {
      "source": {
        "source": "github",
        "repo": "openai/codex-plugin-cc"
      }
    }
  },
  "effortLevel": "high",
  "autoUpdatesChannel": "latest",
  "tui": "fullscreen",
  "skipDangerousModePermissionPrompt": true,
  "theme": "dark"
}
'@ | Set-Content -Path $settingsPath -Encoding utf8
    Ok "作成: $settingsPath"
}

# ------------------------------------------------------------------
Step '5/7  UnityMCP サーバーを起動'
Start-UnityMcp
Start-Sleep -Seconds 3
if (Get-NetTCPConnection -LocalPort 8080 -State Listen -ErrorAction SilentlyContinue) {
    Ok '8080 で listen 中'
} else {
    Warn '8080 がまだ上がっていません。数秒待って `Start-UnityMcp` を再実行してください'
    Warn 'ログ: %LOCALAPPDATA%\UnityMCP\Logs\unity_mcp_server.log'
}

# ------------------------------------------------------------------
Step '6/7  プロジェクトメモリを取り込む（任意）'
$memDst = Join-Path $env:USERPROFILE ".claude\projects\$MemoryProjectKey\memory"
if (Test-Path $MemorySource) {
    if (Test-Path $memDst) {
        Warn "既存のメモリがあるので上書きしません: $memDst"
    } else {
        New-Item -ItemType Directory -Force -Path (Split-Path $memDst -Parent) | Out-Null
        Copy-Item $MemorySource $memDst -Recurse -Force
        Ok "コピー: $((Get-ChildItem $memDst -File).Count) ファイル -> $memDst"
    }
} else {
    Warn "コピー元が無いのでスキップ: $MemorySource"
}

# ------------------------------------------------------------------
Step '7/7  プロジェクトに UnityMCP を登録'
if (Test-Path $ProjectDir) {
    Push-Location $ProjectDir
    try {
        $existing = & claude mcp list 2>&1 | Out-String
        if ($existing -match 'UnityMCP') {
            Ok '登録済み'
        } else {
            & claude mcp add --transport http UnityMCP http://127.0.0.1:8080/mcp
            Ok "登録: UnityMCP -> http://127.0.0.1:8080/mcp ($ProjectDir)"
        }
    } catch {
        Warn "claude mcp add に失敗: $($_.Exception.Message)"
        Warn 'Claude に未ログインの可能性。ログイン後に下記を実行してください:'
        Warn "  cd $ProjectDir; claude mcp add --transport http UnityMCP http://127.0.0.1:8080/mcp"
    } finally { Pop-Location }
} else {
    Warn "$ProjectDir が見つかりません（ドライブ未接続 / パス違い）"
}

# ------------------------------------------------------------------
Write-Host @"

=====================================================================
セットアップ完了。残りの手動作業:

 1) PowerShell を開き直す（プロファイル反映のため）
 2) cd $ProjectDir ; claude
      -> 初回はブラウザで Claude にログイン（認証はアカウント間でコピー不可）
 3) ログイン後、codex プラグインを入れる場合:
      claude plugin marketplace add openai/codex-plugin-cc
      claude plugin install codex@openai-codex
 4) 以後は cc で起動（= claude --dangerously-skip-permissions）
 5) Unity Editor を開くと ~30 秒で UnityMCP に自動接続。
    サーバーが落ちたら PowerShell で Start-UnityMcp
=====================================================================
"@ -ForegroundColor White
