#!/bin/bash
# _x15dump.sh <tag> <width> <height> <spec...>   spec = "i" for rest, "i:t" for seek
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
TAG="$1"; W="$2"; H="$3"; shift 3
OUTS=()
for s in "$@"; do
  if [[ "$s" == *:* ]]; then
    i="${s%%:*}"; t="${s##*:}"; q="?sl=$i&t=$t"; nm="${TAG}_${i}_t${t}"
  else
    q="?sl=$s"; nm="${TAG}_${s}_rest"
  fi
  "$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v15.html$q" "/tmp/$nm.png" "$W" "$H" >/dev/null
  OUTS+=("/tmp/$nm.png")
done
python3 - "/tmp/${TAG}_sheet.png" "${OUTS[@]}" <<'PY'
import sys
from PIL import Image, ImageDraw
out=sys.argv[1]; paths=sys.argv[2:]
ims=[]
for p in paths:
    try: im=Image.open(p).convert('RGB')
    except Exception: continue
    im=im.crop((0,56,im.width,min(im.height,56+330)))
    ims.append((p,im))
if not ims: sys.exit("none")
w=max(i.width for _,i in ims); hh=sum(i.height+22 for _,i in ims)
sheet=Image.new('RGB',(w,hh),(255,0,255)); d=ImageDraw.Draw(sheet); y=0
for p,i in ims:
    sheet.paste(i,(0,y+22)); d.text((6,6+y),p.split('/')[-1],fill=(0,0,0)); y+=i.height+22
sheet.save(out); print("SHEET",out,sheet.size)
PY
