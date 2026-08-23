#!/bin/bash
# TEMPORARY (slogan cycler verification for v17) — delete when done.
# Usage: ./_sl17dump.sh "<url>" [width]
set -u
URL="$1"; W="${2:-1440}"
profile=$(mktemp -d)
raw="$profile/dom.html"
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless --disable-gpu --no-sandbox --hide-scrollbars \
  --force-device-scale-factor=1 --user-data-dir="$profile" \
  --crash-dumps-dir="$profile/crash" --disable-breakpad --no-first-run \
  --no-default-browser-check --disable-background-networking --disable-sync \
  --disable-component-update --disable-default-apps --disable-extensions \
  --window-size="$W,1400" --virtual-time-budget=5000 --dump-dom "$URL" \
  > "$raw" 2>/dev/null &
cpid=$!
for _ in $(seq 1 25); do
  if [ -s "$raw" ] && grep -q '</html>' "$raw" 2>/dev/null; then break; fi
  sleep 1
done
kill "$cpid" 2>/dev/null
pkill -f "$profile" 2>/dev/null
python3 - "$raw" <<'PY'
import sys,re,html
try: s=open(sys.argv[1],encoding='utf-8',errors='replace').read()
except Exception as e: print('READ FAIL',e); raise SystemExit
m=re.search(r'data-sl="([^"]*)"',s)
print(html.unescape(m.group(1)) if m else 'NO PAYLOAD (%d bytes)'%len(s))
PY
rm -rf "$profile"
