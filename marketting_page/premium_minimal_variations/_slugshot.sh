#!/bin/bash
# Usage: ./_slugshot.sh <query> <out.png> [w] [h] [crop l,t,r,b]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
Q="$1"; OUT="$2"; W="${3:-1440}"; H="${4:-1000}"; CROP="${5:-}"
"$HERE/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/v3.html$Q" "$OUT" "$W" "$H" >/dev/null || exit 1
if [ -n "$CROP" ]; then
python3 - "$OUT" "$CROP" <<'PY'
import sys
from PIL import Image
p, c = sys.argv[1], [int(x) for x in sys.argv[2].split(',')]
im = Image.open(p)
im.crop((c[0], c[1], min(c[2], im.width), min(c[3], im.height))).save(p)
PY
fi
echo "OK $OUT"
