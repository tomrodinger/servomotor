#!/bin/bash
# _v4strip.sh <i> <out.png> [w] — vertical contact sheet of one transition i -> i+1
set -u
I="$1"; OUT="$2"; W="${3:-1440}"
HERE="$(cd "$(dirname "$0")" && pwd)"
TS=(0 0.15 0.35 0.5 0.65 0.85 1)
FILES=()
for t in "${TS[@]}"; do
  f="/tmp/_v4f_${I}_${t}_${W}.png"
  "$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v4.html?anim=1&seek=${I},${t}" "$f" "$W" 700 >/dev/null 2>&1
  FILES+=("$f")
done
python3 - "$OUT" "${FILES[@]}" <<'PY'
import sys
from PIL import Image, ImageDraw
out = sys.argv[1]; files = sys.argv[2:]
labels = ["t=0","t=0.15","t=0.35","t=0.50","t=0.65","t=0.85","t=1"]
ims = [Image.open(f).convert("RGB") for f in files]
w = ims[0].width
top, bot = 130, 500
crops = [im.crop((0, top, w, min(bot, im.height))) for im in ims]
ch = crops[0].height
sheet = Image.new("RGB", (w, ch*len(crops)), (255,0,0))
d = ImageDraw.Draw(sheet)
for k,c in enumerate(crops):
    sheet.paste(c, (0, k*ch))
    d.text((10, k*ch+6), labels[k], fill=(255,90,90))
    d.line([(0,k*ch),(w,k*ch)], fill=(60,20,20))
sheet.save(out)
print(out, sheet.size)
PY
