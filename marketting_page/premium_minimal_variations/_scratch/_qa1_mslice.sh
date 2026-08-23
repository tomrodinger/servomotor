#!/bin/bash
OFF=$1; OUT=$2; H=${3:-1800}
D="/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page/premium_minimal_variations"
cat > "$D/_qa1_mframe.html" <<HTML
<!doctype html><meta charset="utf-8">
<style>html,body{margin:0;padding:0;background:#ff00ff;overflow:hidden;}
iframe{position:absolute;left:0;top:-${OFF}px;width:390px;height:40000px;border:0;}</style>
<iframe src="_qa1_probe.html" scrolling="no"></iframe>
HTML
"$D/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/_qa1_mframe.html" "$OUT" 500 "$H" >/dev/null 2>&1
echo "$OUT $(stat -f%z "$OUT" 2>/dev/null)"
