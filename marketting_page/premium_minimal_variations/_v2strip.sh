#!/bin/bash
# Usage: ./_v2strip.sh <fromIndex> <out.png> [width] [height] [x0 y0 x1 y1] [tlist]
set -u
I="$1"; OUT="$2"; W="${3:-1440}"; H="${4:-900}"
X0="${5:-0}"; Y0="${6:-120}"; X1="${7:-1440}"; Y1="${8:-420}"
TS="${9:-0.15 0.35 0.5 0.65 0.85}"
HERE="$(cd "$(dirname "$0")" && pwd)"
FILES=""
for t in $TS; do
  f="/tmp/_v2f_${I}_${t}_${W}.png"
  "$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v2.html?motion=1&seek=${I},${t}" "$f" "$W" "$H" >/dev/null 2>&1
  FILES="$FILES $f"
done
python3 - "$OUT" "$X0" "$Y0" "$X1" "$Y1" "$TS" $FILES <<'PY'
import sys
from PIL import Image, ImageDraw
out=sys.argv[1]; x0,y0,x1,y1=map(int,sys.argv[2:6]); ts=sys.argv[6].split(); files=sys.argv[7:]
ims=[Image.open(f).convert('RGB').crop((x0,y0,x1,y1)) for f in files]
w,h=ims[0].size
sheet=Image.new('RGB',(w,len(ims)*(h+20)),(255,255,255))
d=ImageDraw.Draw(sheet)
for i,im in enumerate(ims):
    y=i*(h+20)
    d.text((6,y+5),"t=%s"%ts[i],fill=(180,0,0))
    sheet.paste(im,(0,y+20))
    d.line([0,y+19,w,y+19],fill=(220,215,205))
sheet.save(out)
print("SHEET",out,sheet.size)
PY
