#!/bin/bash
# Usage: ./_od12dump.sh <url> [width] [height] [wait]  -> DOM dump, self-terminating
set -u
URL="$1"; W="${2:-1440}"; H="${3:-1400}"; WAIT="${4:-14}"
profile=$(mktemp -d); OUTF="$profile/dom.html"
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless --disable-gpu --no-sandbox --hide-scrollbars \
  --force-device-scale-factor=1 \
  --user-data-dir="$profile" --crash-dumps-dir="$profile/crash" \
  --disable-breakpad --no-first-run --no-default-browser-check \
  --disable-background-networking --disable-sync --disable-component-update \
  --disable-default-apps --disable-extensions \
  --window-size="$W,$H" \
  --virtual-time-budget=9000 --dump-dom "$URL" > "$OUTF" 2>/dev/null &
cpid=$!
for _ in $(seq 1 "$WAIT"); do [ -s "$OUTF" ] && { sleep 1; break; }; sleep 1; done
kill "$cpid" 2>/dev/null
pkill -f "$profile" 2>/dev/null
cat "$OUTF"
rm -rf "$profile"
