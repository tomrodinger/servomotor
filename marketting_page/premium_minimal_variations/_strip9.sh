#!/bin/bash
# TEMP: shoot a t-sweep for pair i and tile it vertically. Delete when done.
# Usage: ./_strip9.sh <i> <out.png> [w] [top] [sh] [tlist]
set -u
I="$1"; OUT="$2"; W="${3:-1440}"; TOP="${4:-110}"; SH="${5:-330}"
TL="${6:-0.15,0.35,0.5,0.65,0.85}"
HERE="$(cd "$(dirname "$0")" && pwd)"
FILES=""
IFS=','; for t in $TL; do
  f="/tmp/_s9_${I}_${t}.png"
  "$HERE/_h9.sh" "motion=1&i=$I&t=$t" "$f" "$W" "$TOP" "$SH" >/dev/null 2>&1
  FILES="$FILES $f:$t"
done
unset IFS
python3 - "$OUT" $FILES <<'PY'
import sys
from PIL import Image, ImageDraw
out = sys.argv[1]
items = [a.rsplit(':',1) for a in sys.argv[2:]]
ims = [(Image.open(p).convert('RGB'), lab) for p,lab in items]
w = max(i.width for i,_ in ims); h = sum(i.height for i,_ in ims) + 22*len(ims)
canvas = Image.new('RGB',(w,h),(255,255,255)); d=ImageDraw.Draw(canvas)
y=0
for im,lab in ims:
    d.rectangle([0,y,w,y+20], fill=(20,20,20))
    d.text((8,5+y), 't = '+lab, fill=(255,255,255))
    y+=22
    canvas.paste(im,(0,y)); y+=im.height
    d.line([0,y-1,w,y-1], fill=(200,60,60))
canvas.save(out)
print('OK', out)
PY
