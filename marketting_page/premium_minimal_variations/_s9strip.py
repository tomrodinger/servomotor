#!/usr/bin/env python3
"""Build a contact-sheet harness of v9 hero crops and screenshot it.

usage: _s9strip.py <out.png> <W> <crop_top> <crop_h> <label:query> ...
"""
import os, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
out, W, TOP, CH = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
specs = sys.argv[5:]

rows = []
for s in specs:
    label, q = s.split(':', 1)
    rows.append(
        f'<div class="r"><div class="lb">{label}</div>'
        f'<div class="cl"><iframe src="v9.html?{q}" scrolling="no"></iframe></div></div>'
    )

html = f"""<!doctype html><meta charset="utf-8"><title>s</title>
<style>
html,body{{margin:0;background:#111;font:11px/1.2 ui-monospace,monospace;color:#9f9}}
.r{{position:relative;margin:0 0 3px}}
.lb{{position:absolute;left:0;top:0;z-index:5;background:#111;color:#8f8;padding:2px 6px}}
.cl{{position:relative;width:{W}px;height:{CH}px;overflow:hidden;background:#fff}}
iframe{{position:absolute;left:0;top:-{TOP}px;width:{W}px;height:{TOP+CH+40}px;border:0}}
</style>
""" + "\n".join(rows)

hp = os.path.join(HERE, f"_s9h_{os.getpid()}.html")
open(hp, "w").write(html)
url = f"http://127.0.0.1:8912/premium_minimal_variations/{os.path.basename(hp)}"
H = len(rows) * (CH + 3) + 10
try:
    subprocess.run([os.path.join(HERE, "_shot.sh"), url, out, str(max(W, 500)), str(H)], check=False)
finally:
    os.remove(hp)
