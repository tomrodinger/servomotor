#!/bin/bash
# usage: _qa16_mslice.sh <iframew> <offset> <height> <out>
D=/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page/premium_minimal_variations
W="$1"; OFF="$2"; H="$3"; OUT="$4"
cat > "$D/_qa16_mframe.html" <<HTML
<!doctype html><meta charset="utf-8">
<style>html,body{margin:0;padding:0;background:#ff00ff;overflow:hidden;}
iframe{position:absolute;left:0;top:-${OFF}px;width:${W}px;height:40000px;border:0;}</style>
<iframe src="http://127.0.0.1:8912/premium_minimal_variations/v16.html" scrolling="no"></iframe>
HTML
"$D/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/_qa16_mframe.html" "$OUT" "$W" "$H"
