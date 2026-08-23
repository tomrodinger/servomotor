#!/bin/bash
# dump-dom of a harness page to a file. Isolated profile; only ever kills its own pid/profile.
set -u
URL="$1"; OUT="$2"; W="${3:-1500}"; H="${4:-1000}"
profile=$(mktemp -d)
rm -f "$OUT"
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless --disable-gpu --no-sandbox --hide-scrollbars \
  --force-prefers-reduced-motion --force-device-scale-factor=1 \
  --user-data-dir="$profile" --crash-dumps-dir="$profile/crash" \
  --disable-breakpad --no-first-run --no-default-browser-check \
  --disable-background-networking --disable-sync --disable-component-update \
  --disable-default-apps --disable-extensions \
  --window-size="$W,$H" --virtual-time-budget=9000 --dump-dom "$URL" > "$OUT" 2>/dev/null &
cpid=$!
for _ in $(seq 1 25); do
  if [ -s "$OUT" ] && grep -q '</html>' "$OUT" 2>/dev/null; then break; fi
  sleep 1
done
kill "$cpid" 2>/dev/null
pkill -f "$profile" 2>/dev/null
rm -rf "$profile"
[ -s "$OUT" ] && echo "OK $OUT" || echo "EMPTY"
