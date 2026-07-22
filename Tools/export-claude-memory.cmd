@echo off
REM ダブルクリック起動用ランチャー。
REM 同じフォルダの export-claude-memory.ps1 を ExecutionPolicy Bypass で実行する。
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0export-claude-memory.ps1" %*
echo.
pause
