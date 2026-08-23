#!/bin/bash
# Usage: ./_v3strip.sh <fromIndex> <out.png> [width] [height] [x0 y0 x1 y1]
# Renders t=0.15/0.35/0.5/0.65/0.85 of the transition i -> i+1 and lays the
# five crops out as one contact strip. Temporary review tool for v3.html.
set -u
I="$1"; OUT="$2"; W="${3:-1440}"; H="${4:-980}"
X0="${5:-140}"; Y0="${6:-120}"; X1="${7:-740}"; Y1="${8:-660}"
HERE="$(cd "$(dirname "$0")" && pwd)"
TS="0.15 0.35 0.5 0.65 0.85"
FILES=""
for t in $TS; do
  f="/tmp/_v3f_${I}_${t}.png"
  "$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v3.html?seek=${I}&t=${t}" "$f" "$W" "$H" >/dev/null 2>&1
  FILES="$FILES $f"
done
python3 - "$OUT" "$X0" "$Y0" "$X1" "$Y1" $FILES <<'PY'
import sys
from PIL import Image, ImageDraw
out=sys.argv[1]; x0,y0,x1,y1=map(int,sys.argv[2:6]); files=sys.argv[6:]
ims=[Image.open(f).crop((x0,y0,x1,y1)) for f in files]
w,h=ims[0].size
gap=10
sheet=Image.new('RGB',(len(ims)*w+(len(ims)-1)*gap,h+22),(255,255,255))
d=ImageDraw.Draw(sheet)
labels=["t=0.15","t=0.35","t=0.50","t=0.65","t=0.85"]
for i,im in enumerate(ims):
    x=i*(w+gap)
    sheet.paste(im,(x,22))
    d.text((x+6,6),labels[i] if i<len(labels) else "",fill=(0,0,0))
    d.rectangle([x,21,x+w-1,21+h],outline=(210,205,195))
sheet.save(out)
print("SHEET",out,sheet.size)
PY
