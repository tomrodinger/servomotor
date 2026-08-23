#!/bin/bash
# TEMPORARY (v17 slogan verification) — delete when done.
# Usage: ./_sl17strip.sh <pairIndex> <out.png> [width] [x0 y0 x1 y1]
set -u
I="$1"; OUT="$2"; W="${3:-1440}"
X0="${4:-160}"; Y0="${5:-140}"; X1="${6:-820}"; Y1="${7:-520}"
TS="0.15 0.35 0.5 0.65 0.85"
FILES=""
for t in $TS; do
  f="/tmp/_s17_${I}_${t}_${W}.png"
  ./_shot.sh "http://127.0.0.1:8912/premium_minimal_variations/v17.html?motion=1&seek=${I},${t}" "$f" "$W" 900 >/dev/null
  FILES="$FILES $f"
done
python3 - "$OUT" "$X0" "$Y0" "$X1" "$Y1" $FILES <<'PY'
import sys
from PIL import Image, ImageDraw
out=sys.argv[1]; x0,y0,x1,y1=[int(v) for v in sys.argv[2:6]]
files=sys.argv[6:]
labels=["t=0.15","t=0.35","t=0.50","t=0.65","t=0.85"]
ims=[Image.open(f).convert("RGB").crop((x0,y0,min(x1,Image.open(f).width),y1)) for f in files]
w=max(i.width for i in ims); h=ims[0].height
sheet=Image.new("RGB",(w,(h+22)*len(ims)),(0,0,0))
d=ImageDraw.Draw(sheet)
for n,im in enumerate(ims):
    y=n*(h+22)
    sheet.paste(im,(0,y))
    d.rectangle([0,y+h,w,y+h+21],fill=(30,60,30))
    d.text((6,y+h+6),labels[n],fill=(200,255,200))
sheet.save(out)
print("wrote",out,sheet.size)
PY
