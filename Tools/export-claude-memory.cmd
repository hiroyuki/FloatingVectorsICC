@echo off
REM Double-click launcher for export-claude-memory.ps1.
REM ASCII only: cmd.exe reads .cmd files in the OEM codepage, so non-ASCII
REM comments get mangled and can be executed as commands.
chcp 65001 >NUL
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0export-claude-memory.ps1" %*
echo.
pause
