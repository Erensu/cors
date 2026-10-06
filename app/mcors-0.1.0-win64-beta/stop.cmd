@echo off
REM stop mcors engine and dashboard started by start.cmd
REM keep pure ASCII (cmd.exe parses .cmd as GBK)
echo === stopping mcors ===
taskkill /F /IM cors-engine.exe >nul 2>nul && echo   engine stopped
taskkill /F /FI "WINDOWTITLE eq mcors-dashboard*" /IM node.exe >nul 2>nul && echo   dashboard stopped
REM fallback: dashboard node without a matching title filter may survive
wmic process where "name='node.exe'" get processid,commandline 2>nul | findstr /i "collector\\index.js" >nul && (
    for /f "tokens=2" %%P in ('wmic process where "name='node.exe'" get processid^,commandline 2^>nul ^| findstr /i "collector..index.js"') do taskkill /F /PID %%P >nul 2>nul
    echo   dashboard node processes stopped
)
echo done.
endlocal
exit /b 0
