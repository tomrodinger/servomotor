#!/bin/bash
# _v10strip.sh <outname> <width> <top> <bottom> <label:query> [label:query ...]
# Shoots each state, crops the hero band, stacks vertically with labels.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$1"; W="$2"; TOP="$3"; BOT="$4"; shift 4
TMP="$HERE/_scratch/v10"
mkdir -p "$TMP"
args=()
i=0
for spec in "$@"; do
  lab="${spec%%:*}"; q="${spec#*:}"
  f="$TMP/f_$$_${i}.png"
  "$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v10.html?$q" "$f" "$W" 760 >/dev/null
  args+=("$f" "$lab")
  i=$((i+1))
done
python3 - "$OUT" "$W" "$TOP" "$BOT" "${args[@]}" <<'PY'
import sys
from PIL import Image, ImageDraw
out = sys.argv[1]; W = int(sys.argv[2]); TOP=int(sys.argv[3]); BOT=int(sys.argv[4])
rest = sys.argv[5:]
pairs = [(rest[i], rest[i+1]) for i in range(0, len(rest), 2)]
inset = 100 if W >= 1200 else (10 if W >= 700 else 0)
tiles = []
for f, lab in pairs:
    im = Image.open(f).convert("RGB")
    b = (inset, TOP, min(W-inset, im.width), min(BOT, im.height))
    tiles.append((im.crop(b), lab))
tw = max(t.width for t, _ in tiles)
th = max(t.height for t, _ in tiles)
LH = 20
sheet = Image.new("RGB", (tw, (th+LH+3)*len(tiles)), (232, 232, 236))
d = ImageDraw.Draw(sheet)
y = 0
for t, lab in tiles:
    d.rectangle([0, y, tw, y+LH], fill=(20, 20, 24))
    d.text((7, y+4), lab, fill=(255, 255, 255))
    sheet.paste(t, (0, y+LH))
    y += th + LH + 3
sheet.save(out)
print("sheet", out, sheet.size)
PY
