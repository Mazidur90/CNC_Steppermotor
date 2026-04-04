#!/usr/bin/env bash
# Local compile check (same FQBNs as CI). Requires arduino-cli on PATH.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

arduino-cli version >/dev/null 2>&1 || {
  echo "Install Arduino CLI: https://arduino.github.io/arduino-cli/"
  exit 1
}

arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install AccelStepper
arduino-cli lib install --git-url https://github.com/autowp/arduino-mcp2515.git

arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 master
arduino-cli compile --fqbn arduino:avr:uno slave
echo "OK: master (Mega) + slave (Uno)"
