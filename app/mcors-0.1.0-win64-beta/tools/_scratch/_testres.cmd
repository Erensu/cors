@echo off
setlocal EnableDelayedExpansion
pushd "%~dp0"
set "NODE="
set "BUNDLED_NODE=%~dp0runtime\node.exe"
if exist "%BUNDLED_NODE%" set "NODE=%BUNDLED_NODE%"
if not defined NODE (
    for %%P in ("%LOCALAPPDATA%\Programs\nodejs\node.exe" "%ProgramFiles%\nodejs\node.exe" "%ProgramFiles(x86)%\nodejs\node.exe") do (
        if not defined NODE if exist %%P set "NODE=%%~P"
    )
)
if not defined NODE (
    where node >nul 2>nul && set "NODE=node"
)
echo RESOLVED=[%NODE%]
if defined NODE (
  "%NODE%" -e "console.log('exec ok', process.version)"
)
popd
