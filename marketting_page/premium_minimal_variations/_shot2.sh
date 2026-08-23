#!/bin/bash
set -u
URL="$1"; OUT="$2"; W="${3:-1440}"; H="${4:-700}"
profile=$(mktemp -d); rm -f "$OUT"
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless --disable-gpu --no-sandbox --force-device-scale-factor=1 \
  --user-data-dir="$profile" --crash-dumps-dir="$profile/crash" \
  --disable-breakpad --no-first-run --no-default-browser-check \
  --disable-background-networking --disable-sync --disable-component-update \
  --disable-default-apps --disable-extensions \
  --window-size="$W,$H" --virtual-time-budget="${VTB:-14000}" \
  --screenshot="$OUT" "$URL" >/dev/null 2>&1 &
cpid=$!
for _ in $(seq 1 60); do [ -s "$OUT" ] && { sleep 1; break; }; sleep 1; done
kill "$cpid" 2>/dev/null; pkill -f "$profile" 2>/dev/null; rm -rf "$profile"
[ -s "$OUT" ] && echo "OK $OUT" || { echo FAILED; exit 1; }
