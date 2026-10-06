@echo off
setlocal
set "HFAFC_PYTHON=.venv\Scripts\python.exe"
pushd "%~dp0"
if errorlevel 1 goto failed
set "HFAFC_PUSHED=1"
echo [HFAFC] Working directory: %CD%
if not exist "%HFAFC_PYTHON%" (
    echo [HFAFC] Creating environment: .venv
    py -3 -m venv .venv
    if errorlevel 1 goto failed
)
echo [HFAFC] Python: %HFAFC_PYTHON%
"%HFAFC_PYTHON%" -c "import tkinter" >nul 2>&1
if errorlevel 1 goto failed
"%HFAFC_PYTHON%" -c "import serial" >nul 2>&1
if errorlevel 1 (
    "%HFAFC_PYTHON%" -m pip install -r requirements.txt
    if errorlevel 1 goto failed
)
"%HFAFC_PYTHON%" main.py %*
if errorlevel 1 goto failed
popd
exit /b 0
:failed
echo.
echo Unable to start. Install Python 3 with Tcl/Tk and pip, then retry.
if defined HFAFC_PUSHED popd
pause
exit /b 1
