#!/bin/bash
# _od12strip.sh <out.png> <width> <label> <q1> <q2> ...   query strings appended to v12.html
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$1"; W="$2"; LBL="$3"; shift 3
TMPS=()
n=0
for q in "$@"; do
  f="/tmp/_od12_${LBL}_${n}.png"
  "$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v12.html?motion=1&pause=1&$q" "$f" "$W" 760 >/dev/null
  TMPS+=("$f:$q")
  n=$((n+1))
done
python3 - "$OUT" "${TMPS[@]}" <<'PY'
import sys
from PIL import Image, ImageDraw
out=sys.argv[1]; items=[a.rsplit(':',1) for a in sys.argv[2:]]
crops=[]
for path,label in items:
    im=Image.open(path).convert('RGB')
    # hero headline band: below nav+eyebrow
    crops.append((im.crop((0,130,im.width,130+300)),label))
W=crops[0][0].width; H=sum(c.height+22 for c,_ in crops)
sheet=Image.new('RGB',(W,H),(250,250,252)); d=ImageDraw.Draw(sheet)
y=0
for c,label in crops:
    d.rectangle([0,y,W,y+20],fill=(20,20,22)); d.text((6,5),label,fill=(255,255,255))
    sheet.paste(c,(0,y+22)); y+=c.height+22
    d.line([0,y-1,W,y-1],fill=(200,200,205))
sheet.save(out); print("OK",out,sheet.size)
PY
