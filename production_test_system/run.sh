#!/usr/bin/env bash
# Launch the production test system.
#   ./run.sh              -> real hardware (uses the serial ports assigned in the UI)
#   ./run.sh --sim        -> simulated rack, no hardware needed (separate demo DB)
# Then open http://localhost:8000/ in a browser.
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -d .venv ]; then
  echo "Creating virtualenv and installing dependencies..."
  python3 -m venv .venv
  ./.venv/bin/pip install --upgrade pip
  ./.venv/bin/pip install -r requirements.txt
fi

HOST="${PTS_HOST:-0.0.0.0}"
PORT="${PTS_PORT:-8000}"

if [ "${1:-}" = "--sim" ]; then
  export PTS_SIMULATE=1
  echo "Starting in SIMULATION mode (no hardware)."
fi

exec ./.venv/bin/python -m uvicorn backend.app:app --host "$HOST" --port "$PORT"
