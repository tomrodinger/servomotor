#!/bin/bash
# Usage: ./_sl7strip.sh <sloganIndex> <out.png> [width] [y0] [y1]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
I="$1"; OUT="$2"; W="${3:-1440}"; Y0="${4:-230}"; Y1="${5:-590}"
TS=(0.15 0.35 0.50 0.65 0.85)
FILES=()
for t in "${TS[@]}"; do
  f="/tmp/_sl7_${I}_${t}_${W}.png"
  "$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v7.html?__sl=${I}&__t=${t}" "$f" "$W" 780 >/dev/null
  FILES+=("$f")
done
python3 - "$OUT" "$Y0" "$Y1" "${FILES[@]}" <<'PY'
import sys
from PIL import Image, ImageDraw
out, y0, y1 = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
paths = sys.argv[4:]
ims = [Image.open(p).crop((0, y0, Image.open(p).width, y1)) for p in paths]
w = max(i.width for i in ims); h = sum(i.height for i in ims) + 4*(len(ims)-1)
canvas = Image.new("RGB", (w, h), (255, 0, 255))
y = 0
for im in ims:
    canvas.paste(im, (0, y)); y += im.height + 4
canvas.save(out)
print("wrote", out, canvas.size)
PY
