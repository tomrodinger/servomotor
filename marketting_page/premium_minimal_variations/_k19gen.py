import sys
# usage: gen.py out.html W IFRAME_H CROP_TOP CROP_H  label|query ...
out, W, IH, CT, CH = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
items = sys.argv[6:]
rows = []
for it in items:
    lab, q = it.split('|', 1)
    rows.append('<div class="row"><div class="lab">%s</div><div class="clip">'
                '<iframe src="/premium_minimal_variations/v19.html%s" scrolling="no"></iframe>'
                '</div></div>' % (lab, q))
html = """<!doctype html><meta charset="utf-8"><title>k19</title>
<style>
html,body{margin:0;padding:0;background:#fff;font:11px/1.2 -apple-system,sans-serif}
.row{position:relative;border-bottom:1px solid #f00;}
.lab{position:absolute;left:2px;top:2px;z-index:9;color:#c00;font-weight:700;background:rgba(255,255,255,.85);padding:1px 4px;}
.clip{width:%dpx;height:%dpx;overflow:hidden;position:relative;}
iframe{width:%dpx;height:%dpx;border:0;display:block;position:absolute;left:0;top:%dpx;}
</style>
%s
""" % (W, CH, W, IH, -CT, ''.join(rows))
open(out, 'w').write(html)
