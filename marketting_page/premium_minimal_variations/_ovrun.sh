#!/bin/bash
set -u
W="${1:-1440}"
profile=$(mktemp -d)
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless --disable-gpu --no-sandbox --hide-scrollbars \
  --force-device-scale-factor=1 \
  --user-data-dir="$profile" --crash-dumps-dir="$profile/crash" \
  --disable-breakpad --no-first-run --no-default-browser-check \
  --disable-background-networking --disable-sync --disable-component-update \
  --disable-default-apps --disable-extensions \
  --window-size=1200,900 \
  --virtual-time-budget=12000 --dump-dom \
  "http://127.0.0.1:8912/premium_minimal_variations/_ovmeas.html?w=$W" 2>/dev/null \
  | python3 -c "
import sys,re,html
d=sys.stdin.read()
m=re.search(r'<pre id=\"o\">(.*?)</pre>', d, re.S)
print(html.unescape(m.group(1)) if m else 'NO OUTPUT')
"
pkill -f "$profile" 2>/dev/null
rm -rf "$profile"
