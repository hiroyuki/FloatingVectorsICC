# export-claude-memory.ps1
#
# 「今使っているアカウント」の Claude プロジェクトメモリを、他アカウントから読める
# 共有場所（C:\Users\Public\claude-memory）に書き出す。
#
# 他ユーザーのプロファイル（%USERPROFILE%\.claude）は管理者でないと読めないので、
# アカウントを切り替える *前に* このスクリプトを旧アカウントで実行しておく。
# 新アカウント側では Tools\setup-claude-account.ps1 の step 6 が自動で拾う。
#
# 使い方: Tools\export-claude-memory.cmd をダブルクリック、または
#   powershell -ExecutionPolicy Bypass -File <repo>\Tools\export-claude-memory.ps1

[CmdletBinding()]
param(
    # 対象の Unity プロジェクト（既定: このスクリプトの 1 つ上 = リポジトリルート）
    [string] $ProjectDir = (Split-Path $PSScriptRoot -Parent),

    # 書き出し先ルート
    [string] $StageRoot = 'C:\Users\Public\claude-memory'
)

$ErrorActionPreference = 'Stop'
function Ok($m)   { Write-Host "  [ok] $m" -ForegroundColor Green }
function Warn($m) { Write-Host "  [!!] $m" -ForegroundColor Yellow }

$ProjectDir = [System.IO.Path]::GetFullPath($ProjectDir)
$key = ($ProjectDir.TrimEnd('\')) -replace '[:\\/]', '-'

$src = Join-Path $env:USERPROFILE ".claude\projects\$key\memory"
$dst = Join-Path $StageRoot "$key\memory"

Write-Host "ProjectDir : $ProjectDir"
Write-Host "MemoryKey  : $key"
Write-Host "Source     : $src"
Write-Host "Dest       : $dst"

if (-not (Test-Path $src)) {
    Warn "このアカウントにはメモリがありません: $src"
    Warn '（プロジェクトキーが違う可能性。%USERPROFILE%\.claude\projects\ を確認）'
    exit 1
}

New-Item -ItemType Directory -Force -Path (Split-Path $dst -Parent) | Out-Null
if (Test-Path $dst) { Remove-Item $dst -Recurse -Force }
Copy-Item $src $dst -Recurse -Force
Ok "$((Get-ChildItem $dst -File -Recurse).Count) ファイルを書き出しました"

Write-Host @"

=====================================================================
書き出し完了。次にやること:

 1) 新しいアカウントでログインする
 2) <repo>\Tools\setup-claude-account.cmd を実行（step 6 が上記を取り込む）
 3) 取り込みを確認したら共有場所を消す:
      Remove-Item -Recurse '$StageRoot'
=====================================================================
"@ -ForegroundColor White
