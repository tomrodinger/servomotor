#!/usr/bin/env python3
"""Contact-sheet harness for the v11 slogan cycler.
Usage: _sl11strip.py <out.png> <width> <clipTop> <clipH> <q1> <q2> ...
Each q is a query string appended to v11.html (without leading ?).
"""
import os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
out, W, TOP, CH = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
qs = sys.argv[5:]
IH = TOP + CH + 60

rows = []
for q in qs:
    rows.append(
        f'<div class="c"><div class="lbl">{q}</div>'
        f'<iframe src="v11.html?{q}" scrolling="no"></iframe></div>')

html = f"""<!doctype html><meta charset="utf-8"><title>strip</title>
<style>
html,body{{margin:0;background:#888;}}
.c{{position:relative;width:{W}px;height:{CH}px;overflow:hidden;margin-bottom:3px;background:#fff;}}
.c iframe{{position:absolute;left:0;top:-{TOP}px;width:{W}px;height:{IH}px;border:0;}}
.lbl{{position:absolute;right:0;top:0;z-index:9;background:#111;color:#0f0;
     font:11px monospace;padding:2px 6px;}}
</style>
{''.join(rows)}
"""
name = f"_sl11h_{os.getpid()}.html"
path = os.path.join(HERE, name)
open(path, "w").write(html)
url = f"http://127.0.0.1:8912/premium_minimal_variations/{name}"
try:
    subprocess.run([os.path.join(HERE, "_shot.sh"), url, out, str(W),
                    str(len(qs) * (CH + 3) + 20)], check=True)
finally:
    os.remove(path)
