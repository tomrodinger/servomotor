#!/bin/bash
set -u
URL="$1"; OUTF="$2"
P=$(mktemp -d)
rm -f "$OUTF"
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless --disable-gpu --no-sandbox \
  --force-prefers-reduced-motion --force-device-scale-factor=1 \
  --user-data-dir="$P" --crash-dumps-dir="$P/crash" --disable-breakpad --no-first-run \
  --no-default-browser-check --disable-background-networking --disable-sync \
  --disable-component-update --disable-default-apps --disable-extensions \
  --window-size=1500,900 --virtual-time-budget="${VTB:-20000}" --dump-dom "$URL" > "$OUTF" 2>/dev/null &
pid=$!
for _ in $(seq 1 40); do
  grep -q DONE "$OUTF" 2>/dev/null && break
  sleep 1
done
kill "$pid" 2>/dev/null
pkill -f "$P" 2>/dev/null
rm -rf "$P"
sed -n '/RESULTS/,/DONE/p' "$OUTF"
