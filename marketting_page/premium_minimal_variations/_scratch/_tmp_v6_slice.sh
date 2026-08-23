#!/bin/bash
# usage: _tmp_v6_slice.sh <width> <sliceheight> <offset> <out.png> <src>
W=$1; H=$2; OFF=$3; OUT=$4; SRC=${5:-v6.html}
D="/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page/premium_minimal_variations"
NAME="_tmp_v6_frame_$(echo $OFF)_$W.html"
cat > "$D/$NAME" <<HTML
<!doctype html><meta charset="utf-8">
<style>html,body{margin:0;padding:0;background:#fff;overflow:hidden;}
iframe{position:absolute;left:0;top:-${OFF}px;width:${W}px;height:22000px;border:0;}</style>
<iframe src="$SRC" scrolling="no"></iframe>
HTML
"$D/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/$NAME" "$OUT" "$W" "$H" >/dev/null 2>&1
rm -f "$D/$NAME"
echo "$OUT $(stat -f%z "$OUT" 2>/dev/null) bytes"
