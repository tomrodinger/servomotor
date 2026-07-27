#!/bin/bash
# Refresh the installed Servomotor Arduino library from the repo, then compile
# and flash the on-device test suite to the ESP32-S3.
#   ./flash_suite.sh [port]      (default /dev/cu.usbmodem2101)
set -e
cd "$(dirname "$0")"
PORT="${1:-/dev/cu.usbmodem2101}"
FQBN="esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc"

echo "== Refreshing installed library from repo =="
../copy_stuff_to_Arduino.sh > /dev/null
echo "== Compiling + flashing test suite ($PORT) =="
arduino-cli compile --upload -p "$PORT" --fqbn "$FQBN" .
echo "== Done. Use run_suite_mac.py to start and collect a run. =="
