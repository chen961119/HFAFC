$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
if (-not (Test-Path -LiteralPath '.venv/Scripts/python.exe')) {
    py -3 -m venv .venv
    if ($LASTEXITCODE -ne 0) { throw 'Failed to create Python environment.' }
}
& ./.venv/Scripts/python.exe -m pip install -r requirements.txt 'pyinstaller==6.22.3'
if ($LASTEXITCODE -ne 0) { throw 'Failed to install packaging dependencies.' }
& ./.venv/Scripts/python.exe -m PyInstaller --noconfirm --onefile --windowed `
    --name HFAFCParameterConsole --distpath dist --workpath build --specpath build main.py
if ($LASTEXITCODE -ne 0) { throw 'Failed to build desktop software.' }
Write-Output (Join-Path $PSScriptRoot 'dist/HFAFCParameterConsole.exe')
