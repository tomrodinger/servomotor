import sys, os
# usage: _z13gen.py out.html W CLIPTOP CLIPH  q1 q2 q3 ...
out, W, CT, CH = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
qs = sys.argv[5:]
rows=[]
for q in qs:
    rows.append(
      '<div class="c"><div class="lab">%s</div>'
      '<div class="w"><iframe src="v13.html?anim=1&%s" scrolling="no"></iframe></div></div>' % (q, q))
html = """<!doctype html><meta charset="utf-8"><title>s</title>
<style>html,body{margin:0;background:#888;font:11px/1.2 ui-monospace,monospace}
.c{margin:0 0 4px}
.lab{background:#222;color:#0f0;padding:2px 6px}
.w{position:relative;width:%dpx;height:%dpx;overflow:hidden;background:#fff}
iframe{position:absolute;left:0;top:%dpx;width:%dpx;height:1200px;border:0}
</style>
%s""" % (W, CH, -CT, W, "\n".join(rows))
open(out,"w").write(html)
print(out)
