#!/bin/bash
# _cap.sh <url> <outprefix> <width> <pageheight> [sliceheight]
set -u
URL="$1"; PRE="$2"; W="$3"; PH="$4"; SL="${5:-3000}"
HERE="$(cd "$(dirname "$0")" && pwd)"
OFF=0; N=0
while [ "$OFF" -lt "$PH" ]; do
  H=$SL
  F="$HERE/_cap_$$.html"
  cat > "$F" <<EOF
<!doctype html><meta charset="utf-8"><title>c</title>
<style>html,body{margin:0;padding:0;background:#fff;overflow:hidden}
#w{position:relative;width:${W}px;height:${H}px;overflow:hidden}
iframe{position:absolute;top:-${OFF}px;left:0;width:${W}px;height:${PH}px;border:0}</style>
<div id="w"><iframe src="${URL}" scrolling="no"></iframe></div>
EOF
  P=$(mktemp -d)
  OUT=$(printf "%s_%02d.png" "$PRE" "$N")
  rm -f "$OUT"
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless --disable-gpu --no-sandbox \
    --hide-scrollbars --force-prefers-reduced-motion --force-device-scale-factor=1 \
    --user-data-dir="$P" --window-size="$W,$H" --virtual-time-budget=8000 --screenshot="$OUT" \
    "http://127.0.0.1:8912/premium_minimal_variations/$(basename $F)" >/dev/null 2>&1 &
  CP=$!
  for _ in $(seq 1 40); do [ -s "$OUT" ] && { sleep 1; break; }; sleep 1; done
  kill $CP 2>/dev/null; pkill -f "$P" 2>/dev/null; rm -rf "$P" "$F"
  echo "$OUT off=$OFF"
  OFF=$((OFF+SL)); N=$((N+1))
done
