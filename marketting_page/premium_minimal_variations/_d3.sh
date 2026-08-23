#!/bin/bash
# Usage: ./_d3.sh <url> [width] [height]  -> prints DOM after load, with a hard timeout
set -u
URL="$1"; W="${2:-1440}"; H="${3:-1400}"
profile=$(mktemp -d)
out=$(mktemp)
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless --disable-gpu --no-sandbox --hide-scrollbars \
  --force-device-scale-factor=1 \
  --user-data-dir="$profile" --crash-dumps-dir="$profile/crash" \
  --disable-breakpad --no-first-run --no-default-browser-check \
  --disable-background-networking --disable-sync --disable-component-update \
  --disable-default-apps --disable-extensions \
  --window-size="$W,$H" \
  --virtual-time-budget=6000 --dump-dom "$URL" >"$out" 2>/dev/null &
cpid=$!
for _ in $(seq 1 40); do
  if [ -s "$out" ]; then sleep 0.5; break; fi
  sleep 0.5
done
kill "$cpid" 2>/dev/null
pkill -f "$profile" 2>/dev/null
cat "$out"
rm -f "$out"; rm -rf "$profile"
