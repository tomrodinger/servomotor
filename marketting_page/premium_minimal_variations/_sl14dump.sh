#!/bin/bash
# ./_sl14dump.sh <url> <out.html> [w] [h]   — dump-dom of a probe page.
# REDUCE=1 forces prefers-reduced-motion. Isolated profile; kills only its own pid/profile.
set -u
URL="$1"; OUT="$2"; W="${3:-1500}"; H="${4:-1200}"
RM=""
[ "${REDUCE:-0}" = "1" ] && RM="--force-prefers-reduced-motion"
profile=$(mktemp -d)
rm -f "$OUT"
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless --disable-gpu --no-sandbox --hide-scrollbars \
  $RM --force-device-scale-factor=1 \
  --user-data-dir="$profile" --crash-dumps-dir="$profile/crash" \
  --disable-breakpad --no-first-run --no-default-browser-check \
  --disable-background-networking --disable-sync --disable-component-update \
  --disable-default-apps --disable-extensions \
  --window-size="$W,$H" --virtual-time-budget="${VTB:-12000}" --dump-dom "$URL" > "$OUT" 2>/dev/null &
cpid=$!
for _ in $(seq 1 30); do
  if [ -s "$OUT" ] && grep -q '</html>' "$OUT" 2>/dev/null; then break; fi
  sleep 1
done
kill "$cpid" 2>/dev/null
pkill -f "$profile" 2>/dev/null
rm -rf "$profile"
[ -s "$OUT" ] && echo "OK $OUT" || echo "EMPTY"
