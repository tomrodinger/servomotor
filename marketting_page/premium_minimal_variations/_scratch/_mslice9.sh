#!/bin/bash
# usage: _mslice9.sh <offset> <height> <out>
D=/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page/premium_minimal_variations
OFF="$1"; H="$2"; OUT="$3"
cat > "$D/_mfr9.html" <<HTML
<!doctype html><meta charset="utf-8">
<style>html,body{margin:0;padding:0;background:#ff00ff;overflow:hidden;}
iframe{position:absolute;left:0;top:-${OFF}px;width:390px;height:30000px;border:0;}</style>
<iframe src="http://127.0.0.1:8912/premium_minimal_variations/v9.html" scrolling="no"></iframe>
HTML
"$D/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/_mfr9.html" "$OUT" 390 "$H"
