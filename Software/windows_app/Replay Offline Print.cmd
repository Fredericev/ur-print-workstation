@echo off
setlocal
set "PYTHONUTF8=1"
set "PYTHONDONTWRITEBYTECODE=1"
set "UR_PRINT_DATA_DIR=%LOCALAPPDATA%\UR Print Workstation"
set "UR_PRINT_PACKAGED_PROJECT_ROOT=%~dp0project"
"%~dp0python\python.exe" "%~dp0project\Software\windows_tools.py" offline-rviz --open-browser %*
set "WORKSTATION_RESULT=%ERRORLEVEL%"
if not "%WORKSTATION_RESULT%"=="0" pause
exit /b %WORKSTATION_RESULT%
