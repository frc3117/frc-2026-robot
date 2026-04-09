@echo off
setlocal enabledelayedexpansion

REM Get the directory this script is in
set "BASEDIR=%~dp0"
cd /d "%BASEDIR%"

REM Activate virtual environment
call ".venv\Scripts\activate.bat"

REM Install wheels from whl-requirements.txt, skipping blank lines
for /f "usebackq delims=" %%L in ("whl-requirements.txt") do (
    set "line=%%L"
    if not "!line: =!"=="" (
        pip install --upgrade --find-links=../whl --only-binary=:all: "!line!"
    )
)

REM Sync RobotPy
robotpy sync --find-links ../whl --use-certifi

endlocal