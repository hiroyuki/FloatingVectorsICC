@echo off
REM ダブルクリック起動用ランチャー。
REM 同じフォルダの setup-claude-account.ps1 を ExecutionPolicy Bypass で実行する。
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup-claude-account.ps1" %*
echo.
pause
