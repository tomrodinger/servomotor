#!/bin/bash
# _c6.sh <tag> <query> <w> <h> <cropy0> <cropy1>   — screenshot v6 with a query string, crop the hero
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
TAG="$1"; Q="$2"; W="${3:-1440}"; H="${4:-900}"; Y0="${5:-120}"; Y1="${6:-520}"
mkdir -p /tmp/sl6b
"$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v6.html?$Q" "/tmp/sl6b/${TAG}_full.png" "$W" "$H" >/dev/null
python3 - "/tmp/sl6b/${TAG}_full.png" "/tmp/sl6b/${TAG}.png" "$Y0" "$Y1" <<'PY'
import sys
from PIL import Image
im=Image.open(sys.argv[1]); y0=int(sys.argv[3]); y1=int(sys.argv[4])
im.crop((0,y0,im.width,min(y1,im.height))).save(sys.argv[2])
PY
echo "made /tmp/sl6b/${TAG}.png"
