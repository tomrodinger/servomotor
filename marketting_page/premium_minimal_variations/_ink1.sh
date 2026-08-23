#!/bin/bash
set -u
D="/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page/premium_minimal_variations"
OUT="$1"; W="$2"; TOP="$3"; CH="$4"; PAIR="$5"
F="$D/_ink1_frame.html"
{
echo '<!doctype html><meta charset="utf-8"><style>'
echo 'html,body{margin:0;padding:0;background:#fff;}'
echo ".cell{position:relative;width:${W}px;height:${CH}px;overflow:hidden;}"
echo ".cell iframe{position:absolute;left:0;top:-${TOP}px;width:${W}px;height:$((TOP+CH+200))px;border:0;}"
echo '</style>'
for i in $(seq 0 20); do
  t=$(python3 -c "print($i/20)")
  echo "<div class=\"cell\"><iframe src=\"v1.html?motion=1&seek=${PAIR},${t}\" scrolling=\"no\"></iframe></div>"
done
} > "$F"
"$D/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/_ink1_frame.html" "$OUT" "$W" "$((CH*21+2))" >/dev/null
python3 - "$OUT" "$CH" <<'PY'
import sys
from PIL import Image
im=Image.open(sys.argv[1]).convert('L'); ch=int(sys.argv[2])
import statistics
for i in range(21):
    box=im.crop((0,i*ch,im.width,(i+1)*ch))
    px=list(box.getdata())
    ink=sum((255-p) for p in px)/len(px)
    print(f"t={i/20:.2f} ink={ink:7.3f}")
PY
