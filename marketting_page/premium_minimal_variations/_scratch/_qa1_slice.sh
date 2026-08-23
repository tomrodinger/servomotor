#!/bin/bash
W=$1; H=$2; OFF=$3; OUT=$4; SRC=${5:-_qa1_probe.html}
D="/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page/premium_minimal_variations"
cat > "$D/_qa1_frame.html" <<HTML
<!doctype html><meta charset="utf-8">
<style>html,body{margin:0;padding:0;background:#ff00ff;overflow:hidden;}
iframe{position:absolute;left:0;top:-${OFF}px;width:${W}px;height:40000px;border:0;}</style>
<iframe src="${SRC}" scrolling="no"></iframe>
HTML
"$D/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/_qa1_frame.html" "$OUT" "$W" "$H" >/dev/null 2>&1
echo "$OUT $(stat -f%z "$OUT" 2>/dev/null) bytes"
