#!/bin/bash
# _v2sheet.sh <out.png> <width> <top> <cellH> <query1> [query2 ...]
# Stacks v2.html rendered with each query string, cropped to a hero band.
set -u
D="/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page/premium_minimal_variations"
OUT="$1"; W="$2"; TOP="$3"; CH="$4"; shift 4
F="$D/_v2frame.html"
N=$#
{
echo '<!doctype html><meta charset="utf-8"><style>'
echo 'html,body{margin:0;padding:0;background:#fff;}'
echo ".cell{position:relative;width:${W}px;height:${CH}px;overflow:hidden;border-bottom:1px solid #c00;}"
echo ".cell iframe{position:absolute;left:0;top:-${TOP}px;width:${W}px;height:$((TOP+CH+300))px;border:0;}"
echo '</style>'
for q in "$@"; do
  echo "<div class=\"cell\"><iframe src=\"v2.html?motion=1&${q}\" scrolling=\"no\"></iframe></div>"
done
} > "$F"
"$D/_shot.sh" "http://127.0.0.1:8912/premium_minimal_variations/_v2frame.html" "$OUT" "$W" "$((CH*N+4))"
