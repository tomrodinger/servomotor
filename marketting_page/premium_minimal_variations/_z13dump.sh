#!/bin/bash
# TEMPORARY (v13 slogan verification) — delete when done.
# _z13dump.sh <tag> <width> <height> <cropTop> <cropH> <spec...>   spec = "i" rest, "i:t" seek
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
TAG="$1"; W="$2"; H="$3"; CT="$4"; CH="$5"; shift 5
OUTS=()
for s in "$@"; do
  if [[ "$s" == *:* ]]; then
    i="${s%%:*}"; t="${s##*:}"; q="?sl=$i&t=$t"; nm="${TAG}_${i}_t${t}"
  else
    q="?sl=$s"; nm="${TAG}_${s}_rest"
  fi
  "$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v13.html$q" "/tmp/$nm.png" "$W" "$H" >/dev/null
  OUTS+=("/tmp/$nm.png")
done
python3 - "/tmp/${TAG}_sheet.png" "$CT" "$CH" "${OUTS[@]}" <<'PY'
import sys
from PIL import Image, ImageDraw
out=sys.argv[1]; ct=int(sys.argv[2]); ch=int(sys.argv[3]); paths=sys.argv[4:]
ims=[]
for p in paths:
    try: im=Image.open(p).convert('RGB')
    except Exception: continue
    ims.append((p,im.crop((0,ct,im.width,min(im.height,ct+ch)))))
if not ims: sys.exit("none")
w=max(i.width for _,i in ims); hh=sum(i.height+20 for _,i in ims)
sheet=Image.new('RGB',(w,hh),(255,0,255)); d=ImageDraw.Draw(sheet); y=0
for p,i in ims:
    sheet.paste(i,(0,y+20)); d.text((6,5+y),p.split('/')[-1],fill=(0,0,0)); y+=i.height+20
sheet.save(out); print("SHEET",out,sheet.size)
PY
