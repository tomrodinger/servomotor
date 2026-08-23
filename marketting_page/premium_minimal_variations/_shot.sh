#!/bin/bash
# Usage: ./_shot.sh <url> <out.png> [width] [height]
#
# Two things this works around, both verified on this machine:
#
# 1. Headless Chrome REFUSES to lay out below 500 CSS px. Ask for --window-size=390 and
#    document.documentElement.clientWidth still reports 500; the PNG is just cropped to 390.
#    That makes a 500px layout look like it overflows horizontally, and hides real bugs in
#    any media query below 500px. So for width < 500 we render the page inside a fixed-width
#    iframe on a host page (the iframe gets a TRUE narrow layout viewport) and crop to it.
#
# 2. Headless Chrome often does not exit after --screenshot, so run it detached, poll for
#    the file, then kill it.
#
# --force-prefers-reduced-motion matters too: pages using IntersectionObserver scroll-reveal
# leave everything below the fold invisible in a tall screenshot without it.
set -u
URL="$1"; OUT="$2"; W="${3:-1280}"; H="${4:-2600}"
HERE="$(cd "$(dirname "$0")" && pwd)"
MIN=500

run_chrome() {  # run_chrome <url> <out> <w> <h>
  local u="$1" o="$2" w="$3" h="$4"
  local profile; profile=$(mktemp -d)
  rm -f "$o"
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
    --headless --disable-gpu --no-sandbox --hide-scrollbars \
    --force-prefers-reduced-motion --force-device-scale-factor=1 \
    --user-data-dir="$profile" --crash-dumps-dir="$profile/crash" \
    --disable-breakpad --no-first-run --no-default-browser-check \
    --disable-background-networking --disable-sync --disable-component-update \
    --disable-default-apps --disable-extensions \
    --window-size="$w,$h" \
    --virtual-time-budget="${VTB:-8000}" --screenshot="$o" "$u" >/dev/null 2>&1 &
  local cpid=$!
  for _ in $(seq 1 60); do
    [ -s "$o" ] && { sleep 1; break; }
    sleep 1
  done
  # ONLY ever kill what this function launched. NEVER `pkill -f "Google Chrome"` or
  # `killall "Google Chrome"` — Tom's real browser's argv contains that exact substring,
  # so those patterns SIGTERM his actual session and silently destroy his open tabs
  # (it looks like a mystery crash because SIGTERM is the graceful-shutdown path).
  kill "$cpid" 2>/dev/null
  pkill -f "$profile" 2>/dev/null
  rm -rf "$profile"
}

if [ "$W" -ge "$MIN" ]; then
  run_chrome "$URL" "$OUT" "$W" "$H"
else
  # narrow: real $W-wide layout inside an iframe, host page rendered at the 500px floor
  HARNESS="$HERE/_narrow_$$_$W.html"
  cat > "$HARNESS" <<EOF
<!doctype html><meta charset="utf-8"><title>narrow harness</title>
<style>html,body{margin:0;padding:0;background:#fff}
iframe{width:${W}px;height:${H}px;border:0;display:block}</style>
<iframe src="${URL}" scrolling="no"></iframe>
EOF
  HARNESS_URL="http://127.0.0.1:8912/premium_minimal_variations/$(basename "$HARNESS")"
  RAW="${OUT%.png}_raw.png"
  run_chrome "$HARNESS_URL" "$RAW" "$MIN" "$H"
  rm -f "$HARNESS"
  if [ -s "$RAW" ]; then
    python3 - "$RAW" "$OUT" "$W" <<'PY'
import sys
from PIL import Image
raw, out, w = sys.argv[1], sys.argv[2], int(sys.argv[3])
im = Image.open(raw)
im.crop((0, 0, min(w, im.width), im.height)).save(out)
PY
    rm -f "$RAW"
  fi
fi

if [ -s "$OUT" ]; then
  echo "OK $OUT ($(stat -f%z "$OUT") bytes, layout width ${W}px)"
else
  echo "FAILED $OUT"; exit 1
fi
