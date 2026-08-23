#!/bin/bash
# Usage: ./_v7dump.sh <url> [window-w] [window-h]
# Headless dump-dom of a probe page, run detached and polled (headless Chrome often
# will not exit on its own). Only ever kills its own unique temp profile.
set -u
URL="$1"; W="${2:-1500}"; H="${3:-1000}"
profile=$(mktemp -d)
raw="$profile/dom.html"
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless --disable-gpu --no-sandbox --hide-scrollbars \
  --force-device-scale-factor=1 \
  --user-data-dir="$profile" --crash-dumps-dir="$profile/crash" \
  --disable-breakpad --no-first-run --no-default-browser-check \
  --disable-background-networking --disable-sync --disable-component-update \
  --disable-default-apps --disable-extensions \
  --window-size="$W,$H" \
  --virtual-time-budget=${VTB:-10000} --dump-dom "$URL" >"$raw" 2>/dev/null &
cpid=$!
for _ in $(seq 1 40); do
  if [ -s "$raw" ] && grep -q '</html>' "$raw" 2>/dev/null; then break; fi
  sleep 1
done
kill "$cpid" 2>/dev/null
pkill -f "$profile" 2>/dev/null
python3 -c "
import sys,re,html
s=open(sys.argv[1],encoding='utf-8',errors='replace').read()
m=re.search(r'<pre id=\"o\">(.*?)</pre>', s, re.S)
print(html.unescape(m.group(1)) if m else 'NO OUTPUT BLOCK')
" "$raw"
rm -rf "$profile"
