#!/bin/bash
# TEMP verification helper for v5 slogan cycler (delete when done).
# Usage: ./_h5.sh "<query string without ?>" <out.png> <layoutWidth> <sliceTop> <sliceHeight>
set -u
QS="$1"; OUT="$2"; W="$3"; TOP="$4"; SH="$5"
HERE="$(cd "$(dirname "$0")" && pwd)"
HOSTW=$W; [ "$W" -lt 500 ] && HOSTW=500
ESC=$(printf '%s' "$QS" | sed 's/&/\&amp;/g')
H="$HERE/_h5frame_$$.html"
cat > "$H" <<EOF2
<!doctype html><meta charset="utf-8"><title>s</title>
<style>html,body{margin:0;padding:0;background:#fff;overflow:hidden}
#c{position:absolute;left:0;top:0;width:${W}px;height:${SH}px;overflow:hidden}
iframe{position:absolute;left:0;top:-${TOP}px;width:${W}px;height:9000px;border:0;display:block}</style>
<div id="c"><iframe src="v5.html?${ESC}" scrolling="no"></iframe></div>
EOF2
U="http://127.0.0.1:8912/premium_minimal_variations/$(basename "$H")"
RAW="${OUT%.png}_raw.png"
p=$(mktemp -d); rm -f "$RAW"
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless --disable-gpu --no-sandbox \
 --hide-scrollbars --force-prefers-reduced-motion --force-device-scale-factor=1 \
 --user-data-dir="$p" --window-size="$HOSTW,$SH" --virtual-time-budget=9000 --screenshot="$RAW" "$U" >/dev/null 2>&1 &
cp=$!
for _ in $(seq 1 60); do [ -s "$RAW" ] && { sleep 1; break; }; sleep 1; done
kill $cp 2>/dev/null; pkill -f "$p" 2>/dev/null; rm -rf "$p"; rm -f "$H"
if [ -s "$RAW" ]; then
python3 - "$RAW" "$OUT" "$W" <<'PY'
import sys
from PIL import Image
raw,out,w=sys.argv[1],sys.argv[2],int(sys.argv[3])
im=Image.open(raw); im.crop((0,0,min(w,im.width),im.height)).save(out)
PY
rm -f "$RAW"; echo "OK $OUT"
else echo "FAILED $OUT"; exit 1; fi
