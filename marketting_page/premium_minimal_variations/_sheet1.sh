#!/bin/bash
# _sheet1.sh <outpng> <width> <cliptop> <clipheight> <label> <q1> [q2 ...]
set -u
D="/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page/premium_minimal_variations"
OUT="$1"; W="$2"; TOP="$3"; CH="$4"; LABEL="$5"; shift 5
HOSTW=$W
IFRH=$((TOP+CH+200))
F="$D/_sheet1_frame.html"
{
echo '<!doctype html><meta charset="utf-8"><style>'
echo 'html,body{margin:0;padding:0;background:#fff;font:11px ui-monospace,Menlo,monospace;}'
echo ".cell{position:relative;width:${W}px;height:${CH}px;overflow:hidden;border-bottom:1px solid #f00;}"
echo ".cell iframe{position:absolute;left:0;top:-${TOP}px;width:${W}px;height:${IFRH}px;border:0;}"
echo '.lab{position:absolute;left:4px;top:2px;z-index:9;color:#c00;background:#fff;padding:1px 4px;}'
echo '</style>'
for q in "$@"; do
  echo "<div class=\"cell\"><span class=\"lab\">$LABEL $q</span><iframe src=\"v1.html?$q\" scrolling=\"no\"></iframe></div>"
done
} > "$F"
N=$#
H=$((CH*N+4))
"$D/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/_sheet1_frame.html" "$OUT" "$HOSTW" "$H"
