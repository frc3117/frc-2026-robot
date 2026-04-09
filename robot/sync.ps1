$BaseDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $BaseDir

# Activate virtual environment
& ".\.venv\Scripts\Activate.ps1"

function Install-Wheel { param ([string]$Package)
    pip install --upgrade --find-links=../whl --only-binary=:all: $Package
}

Get-Content "whl-requirements.txt" | ForEach-Object {
    $line = $_.Trim()
    if ($line -ne "") {
        Install-Wheel $line
    }
}

robotpy sync --find-links ../whl --use-certifi