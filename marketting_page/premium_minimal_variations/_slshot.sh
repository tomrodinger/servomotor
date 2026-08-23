#!/bin/bash
# _slshot.sh <query> <out.png> [w] [h] [cropbox]  — shoot v6 and crop to the hero headline
set -u
Q="$1"; OUT="$2"; W="${3:-1440}"; H="${4:-900}"; CROP="${5:-140,130,780,500}"
HERE="$(cd "$(dirname "$0")" && pwd)"
RAW="${OUT%.png}_full.png"
"$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v6.html?$Q" "$RAW" "$W" "$H" >/dev/null 2>&1
python3 - "$RAW" "$OUT" "$CROP" <<'PY'
import sys
from PIL import Image
raw,out,crop=sys.argv[1],sys.argv[2],sys.argv[3]
l,t,r,b=[int(x) for x in crop.split(',')]
im=Image.open(raw)
im.crop((l,t,min(r,im.width),min(b,im.height))).save(out)
PY
rm -f "$RAW"
echo "OK $OUT"
