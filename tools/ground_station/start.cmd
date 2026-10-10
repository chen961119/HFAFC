@echo off
setlocal
set "COFLY_PYTHON=.venv\Scripts\python.exe"
pushd "%~dp0"
if errorlevel 1 goto failed
set "COFLY_PUSHED=1"
echo [CoFly Autopilot] Working directory: %CD%
if not exist "%COFLY_PYTHON%" (
    echo [CoFly Autopilot] Creating environment: .venv
    py -3 -m venv .venv
    if errorlevel 1 goto failed
)
echo [CoFly Autopilot] Python: %COFLY_PYTHON%
"%COFLY_PYTHON%" -c "import tkinter" >nul 2>&1
if errorlevel 1 goto failed
"%COFLY_PYTHON%" -c "import serial" >nul 2>&1
if errorlevel 1 (
    "%COFLY_PYTHON%" -m pip install -r requirements.txt
    if errorlevel 1 goto failed
)
"%COFLY_PYTHON%" main.py %*
if errorlevel 1 goto failed
popd
exit /b 0
:failed
echo.
echo Unable to start. Install Python 3 with Tcl/Tk and pip, then retry.
if defined COFLY_PUSHED popd
pause
exit /b 1
