# Local compile check (PowerShell). Requires arduino-cli on PATH.
$ErrorActionPreference = "Stop"
Set-Location (Split-Path -Parent $PSScriptRoot)

arduino-cli version | Out-Null
if (-not $?) {
    Write-Error "Install Arduino CLI: https://arduino.github.io/arduino-cli/"
}

arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install AccelStepper
arduino-cli lib install --git-url https://github.com/autowp/arduino-mcp2515.git

arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 master
arduino-cli compile --fqbn arduino:avr:uno slave
Write-Host "OK: master (Mega) + slave (Uno)"
