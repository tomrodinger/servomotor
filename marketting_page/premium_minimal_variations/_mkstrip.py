import sys, os
# usage: _mkstrip.py out.html width rowh "q1" "q2" ...
out, w, rh = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
qs = sys.argv[4:]
rows = "".join(
  f'<div class="r"><iframe src="v1.html?{q}" scrolling="no"></iframe><span class="lb">{q}</span></div>'
  for q in qs)
open(out,"w").write(f"""<!doctype html><meta charset=utf-8><title>strip</title>
<style>html,body{{margin:0;background:#999}}
.r{{position:relative;width:{w}px;height:{rh}px;overflow:hidden;margin-bottom:2px;background:#fff}}
iframe{{width:{w}px;height:2600px;border:0;display:block;margin-top:-150px}}
.lb{{position:absolute;left:0;top:0;background:#000;color:#0f0;font:11px monospace;padding:2px 6px;z-index:9}}
</style>{rows}""")
print(out)
