@echo off
REM ============================================================================
REM  mcors one-click launcher (engine + dashboard)
REM
REM  IMPORTANT: keep this file pure ASCII (cmd.exe parses .cmd as GBK).
REM
REM  Why the engine needs "start" with its own console:
REM    src/common/vt.c (WIN32 branch) unconditionally opens CONOUT$/CONIN$.
REM    If that fails, main() prints "console open error" and exits. So the
REM    engine MUST own a real console window - it cannot run silently.
REM
REM  Layout expected (relative to this file):
REM    cors-engine.exe  libuv.dll  conf\  viz\collector\  viz\web\
REM
REM  Usage:
REM    start.cmd            engine + dashboard + browser
REM    start.cmd engine     engine only
REM    start.cmd dash       dashboard only
REM ============================================================================
setlocal EnableDelayedExpansion

pushd "%~dp0" || (echo [ERROR] cannot locate package root
pause
exit /b 1)

set "MODE=%~1"
if "%MODE%"=="" set "MODE=all"

set "ENGINE=%~dp0cors-engine.exe"
set "CONF=%~dp0conf\cors.conf"

REM ---- locate node.exe without any machine-specific path -------------------
set "NODE="
for %%P in ("%LOCALAPPDATA%\Programs\nodejs\node.exe" "%ProgramFiles%\nodejs\node.exe" "%ProgramFiles(x86)%\nodejs\node.exe") do (
    if not defined NODE if exist %%P set "NODE=%%~P"
)
if not defined NODE (
    for /d %%D in ("%USERPROFILE%\.workbuddy\binaries\node\versions\*") do (
        if exist "%%D\node.exe" set "NODE=%%D\node.exe"
    )
)
if not defined NODE (
    where node >nul 2>nul && set "NODE=node"
)

if /I "%MODE%"=="dash" goto dash
if /I "%MODE%"=="engine" goto engine

:engine
if not exist "%ENGINE%" (
    echo [ERROR] cors-engine.exe not found next to start.cmd
    popd & exit /b 1
)
if exist "%ENGINE%.running" (
    echo [WARN] engine seems already running ^(stale marker^). If not, delete %ENGINE%.running
)
echo === starting mcors engine (console port 9000) ===
start "mcors-engine" /D "%~dp0" "%ENGINE%" -o "%CONF%" -p 9000 -t 1 -s
REM give the engine time to bind ports
ping -n 4 127.0.0.1 >nul
if /I "%MODE%"=="engine" (
    echo   engine window launched. Type "start-gui" in that window to open the dashboard.
    popd & exit /b 0
)

:dash
if not defined NODE (
    echo [ERROR] node.exe not found. Install Node.js, or add it to PATH.
    echo         The dashboard needs it to serve the web console.
    popd & exit /b 1
)
echo === starting dashboard (web port 8181) ===
start "mcors-dashboard" /D "%~dp0viz" cmd /c "set MCORS_SOURCE=live&& set MCORS_CONSOLE_PORT=9000&& set MCORS_WEB_PORT=8181&& "%NODE%" collector\index.js"
ping -n 4 127.0.0.1 >nul
start "" http://localhost:8181/
echo.
echo   dashboard : http://localhost:8181/
echo   stop      : stop.cmd  (or close the two windows titled mcors-*)
echo   re-open   : start.cmd dash
echo   LAN access: start.cmd dash  after setting MCORS_WEB_HOST - see doc\手册
popd
endlocal
exit /b 0
