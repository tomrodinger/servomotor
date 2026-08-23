#!/bin/bash
# Renders a full-page screenshot of every variant into thumbs/ and writes overview.html,
# a scrollable contact sheet of all 20 so they can be compared at a glance.
# Requires the static server on :8912 (already running) and Google Chrome.
set -u
cd "$(dirname "$0")"
mkdir -p thumbs
BASE="http://127.0.0.1:8912/premium_minimal_variations/"

for n in $(seq 1 20); do
  [ -f "v$n.html" ] || { echo "skip v$n (missing)"; continue; }
  ./_shot.sh "${BASE}v$n.html" "thumbs/v$n.png" 1280 7200 >/dev/null 2>&1 \
    && echo "shot v$n" || echo "FAILED v$n"
done
./_shot.sh "${BASE}_BASELINE_v10_premium_minimal.html" "thumbs/baseline.png" 1280 7200 >/dev/null 2>&1

python3 - <<'PY'
import os
names = [
    "Director’s Cut","Ivory & Ink","Editorial Serif","Midnight Inverse","Swiss Grid",
    "Mono Technical","Green Field","Asymmetric Split","Soft Neutral","Gallery",
    "Blueprint","Big Numerals","Graphite & Chrome","Whisper Gradient","Hairline Catalogue",
    "Sage & Stone","Terminal Minimal","Magazine","Scroll Choreography","Statement Type",
]
cards = []
for i, nm in enumerate(names, 1):
    if not os.path.exists(f"thumbs/v{i}.png"):
        continue
    cards.append(f'''  <a class="card" href="v{i}.html" target="_blank" rel="noopener">
    <div class="shot"><img src="thumbs/v{i}.png" alt="{nm} full page" loading="lazy"></div>
    <div class="cap"><b>{i}</b> {nm}</div>
  </a>''')
if os.path.exists("thumbs/baseline.png"):
    cards.append('''  <a class="card base" href="_BASELINE_v10_premium_minimal.html" target="_blank" rel="noopener">
    <div class="shot"><img src="thumbs/baseline.png" alt="baseline full page" loading="lazy"></div>
    <div class="cap"><b>—</b> Original Premium Minimal</div>
  </a>''')

html = '''<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>M17 — 20 Premium Minimal variations · contact sheet</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box}
  body{background:#0d0d0e;color:#e8e8ec;
       font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;padding:20px}
  h1{font-size:15px;font-weight:700;letter-spacing:-.01em;margin-bottom:4px}
  h1 span{color:#7AB648}
  p.sub{font-size:12.5px;color:#75757c;margin-bottom:20px}
  p.sub a{color:#9dcb75}
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:18px}
  .card{display:block;text-decoration:none;color:inherit;
        border:1px solid #262629;border-radius:10px;overflow:hidden;background:#161618;
        transition:border-color .15s,transform .15s}
  .card:hover{border-color:#7AB648;transform:translateY(-2px)}
  .card.base{opacity:.72}
  .shot{height:520px;overflow:hidden;background:#fff}
  .shot img{width:100%;display:block}
  .cap{padding:9px 12px;font-size:12.5px;color:#b9b9c0;border-top:1px solid #262629}
  .cap b{color:#7AB648;margin-right:7px}
</style></head><body>
<h1><span>Gearotons</span> M17 — 20 variations on “Premium Minimal”</h1>
<p class="sub">Top ~520px of each page. Click any card to open it full size ·
<a href="index.html">switch to the tabbed viewer</a></p>
<div class="grid">
''' + "\n".join(cards) + '''
</div></body></html>'''
open("overview.html", "w").write(html)
print("wrote overview.html with", len(cards), "cards")
PY
