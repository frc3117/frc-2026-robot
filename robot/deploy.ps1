$BaseDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $BaseDir

robotpy deploy --team 3117 --nc-ds # --force-install