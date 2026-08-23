#!/bin/bash
# Usage: ./_dump.sh <url> [width] [height]  -> prints DOM after load (for measurement probes)
set -u
URL="$1"; W="${2:-1440}"; H="${3:-1200}"
profile=$(mktemp -d)
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless --disable-gpu --no-sandbox --hide-scrollbars \
  --force-device-scale-factor=1 \
  --user-data-dir="$profile" --crash-dumps-dir="$profile/crash" \
  --disable-breakpad --no-first-run --no-default-browser-check \
  --disable-background-networking --disable-sync --disable-component-update \
  --disable-default-apps --disable-extensions \
  --window-size="$W,$H" \
  --virtual-time-budget=9000 --dump-dom "$URL" 2>/dev/null
pkill -f "$profile" 2>/dev/null
rm -rf "$profile"
