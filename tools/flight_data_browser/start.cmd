@echo off
setlocal
pushd "%~dp0"
if errorlevel 1 goto failed
if not exist ".venv\Scripts\python.exe" (
    py -3 -m venv .venv
    if errorlevel 1 goto failed
)
".venv\Scripts\python.exe" -c "import numpy, pandas, matplotlib, tkinter" >nul 2>&1
if errorlevel 1 (
    ".venv\Scripts\python.exe" -m pip install -r requirements.txt
    if errorlevel 1 goto failed
)
".venv\Scripts\python.exe" main.py %*
if errorlevel 1 goto failed
popd
exit /b 0
:failed
echo Unable to start. Install Python 3.10+ with Tcl/Tk and pip, then retry.
pause
exit /b 1
