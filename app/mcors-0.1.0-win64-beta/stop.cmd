@echo off
REM stop mcors engine and dashboard started by start.cmd
REM keep pure ASCII (cmd.exe parses .cmd as GBK)
echo === stopping mcors ===
taskkill /F /IM cors-engine.exe >nul 2>nul && echo   engine stopped

REM The dashboard runs on the bundled runtime\node.exe (or a system node.exe),
REM in a window titled "mcors-dashboard". Kill by title first - it is the only
REM reliable handle, because both executables are named node.exe.
taskkill /F /FI "WINDOWTITLE eq mcors-dashboard*" /IM node.exe >nul 2>nul && echo   dashboard stopped

REM Fallback for a dashboard started without that window title (e.g. launched
REM by hand from a shell): match the collector script path instead.
REM PowerShell + CIM is used because WMIC was removed from recent Windows
REM builds; the old wmic one-liner silently did nothing there, leaving the
REM dashboard holding port 8181 and breaking the next start.
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "Get-CimInstance Win32_Process -Filter \"Name='node.exe'\" | Where-Object { $_.CommandLine -like '*collector*index.js*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }" >nul 2>nul
echo   dashboard node processes stopped
echo done.
endlocal
exit /b 0