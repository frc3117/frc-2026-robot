@echo off
setlocal

REM Get the directory this script is in
set "BASEDIR=%~dp0"
cd /d "%BASEDIR%"

REM Add --force-install if needed:
REM robotpy deploy --team 3117 --nc-ds --force-install
robotpy deploy --team 3117 --nc-ds

endlocal