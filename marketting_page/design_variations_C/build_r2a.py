#!/usr/bin/env python3
"""Round 2 — v11–v20 company pastiches."""
from pathlib import Path
from _content import (
    TITLE, TAGLINE, SHOP, DOCS, GITHUB, CODE, INSTALL, ARDUINO,
    MODELS, FEATURES, INTRO, INTRO2, COMPANY, spec_rows, esc, shell,
)

OUT = Path(__file__).parent


def w(name, html):
    (OUT / name).write_text(html, encoding="utf-8")
    print("wrote", name, len(html))


def spec_table(th_style="", extra=""):
    return f"""<div style="overflow:auto;{extra}">
    <table style="width:100%;border-collapse:collapse">
      <thead><tr>
        <th style="text-align:left;padding:10px 8px;{th_style}">Parameter</th>
        <th style="text-align:left;padding:10px 8px;{th_style}">M17-60</th>
        <th style="text-align:left;padding:10px 8px;{th_style}">M17-48</th>
        <th style="text-align:left;padding:10px 8px;{th_style}">M17-40</th>
      </tr></thead>
      <tbody>{spec_rows()}</tbody>
    </table></div>"""


def v11():
    """NVIDIA — dark, lime, cinematic industry tiles."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.5 "Avenir Next",Avenir,"Helvetica Neue",Helvetica,sans-serif;background:#000;color:#ddd}
img{max-width:100%;height:auto;display:block}
a{color:#fff;text-decoration:none}
nav{position:sticky;top:0;z-index:30;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#000;border-bottom:1px solid #1a1a1a}
nav img{height:24px;filter:invert(1)}
nav ul{display:flex;gap:22px;list-style:none;font-size:13px}
.cta{background:#76B900;color:#000;padding:8px 16px;border-radius:2px;font-weight:700;font-size:13px}
.cta:hover{background:#8fd014}
.hero{position:relative;min-height:86vh;display:grid;place-items:center;overflow:hidden;background:radial-gradient(80% 60% at 70% 40%,#163000 0%,#000 60%)}
.hero img{max-height:70vh;filter:drop-shadow(0 0 80px rgba(118,185,0,.25))}
.hero .copy{position:absolute;left:6vw;bottom:10vh;max-width:22em}
.k{font-size:11px;letter-spacing:.22em;text-transform:uppercase;color:#76B900}
h1{font-size:clamp(40px,7vw,80px);color:#fff;letter-spacing:-.03em;line-height:.95;margin:8px 0}
section{padding:72px 6vw}
h2{color:#fff;font-size:32px;margin-bottom:12px}
.tiles{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin-top:28px}
.tiles article{background:#111;min-height:220px;padding:20px;display:flex;flex-direction:column;justify-content:flex-end;background-size:cover;background-position:center}
.tiles h3{color:#fff;font-size:20px}
.news{display:grid;grid-template-columns:repeat(4,1fr);gap:16px}
.news article{background:#111;padding:0 0 16px}
.news img{height:140px;width:100%;object-fit:cover}
.news h3{color:#fff;font-size:15px;padding:12px 14px 0}
.news p{padding:6px 14px;font-size:13px;color:#aaa}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.models article{background:#111;padding:20px}
.models h3{color:#76B900}
pre{background:#0b0b0b;border:1px solid #222;padding:20px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace;color:#cde8a0}
footer{padding:48px 6vw 72px;border-top:1px solid #1a1a1a;display:grid;grid-template-columns:2fr 1fr 1fr 1fr;gap:24px;font-size:13px;color:#888}
footer a{color:#888;display:block;margin:4px 0}
footer a:hover{color:#76B900}
.os img{height:40px;background:#fff;padding:3px;margin-right:8px}
@media(max-width:800px){nav ul,.tiles,.news,.models,footer{grid-template-columns:1fr}nav ul{display:none}}
"""
    news = "".join(
        f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:8]
    )
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt='' style='background:#fff;margin:12px 0'><p>{m['torque']} · {m['power']} · {m['weight']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <ul>
    <li><a href="#features">Products</a></li>
    <li><a href="#models">Models</a></li>
    <li><a href="#start">Developers</a></li>
    <li><a href="#apps">Industries</a></li>
  </ul>
  <a class="cta" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17">
  <div class="copy">
    <p class="k">Physical AI · Motion</p>
    <h1>All-in-one. On the bus.</h1>
    <p>{TAGLINE}</p>
    <p style="margin-top:16px"><a class="cta" href="{SHOP}">Shop Now</a></p>
  </div>
</header>
<section id="apps">
  <p class="k">Industries</p>
  <h2>Where the shaft goes to work.</h2>
  <div class="tiles">
    <article style="background-image:linear-gradient(to top,#000 20%,transparent 70%),url('../robotics_small.jpg')"><h3>Robotics</h3></article>
    <article style="background-image:linear-gradient(to top,#000 20%,transparent 70%),url('../automation_small.jpg')"><h3>Automation</h3></article>
    <article style="background-image:linear-gradient(to top,#000 20%,transparent 70%),url('../test_rack_small.jpg')"><h3>Test</h3></article>
    <article style="background-image:linear-gradient(to top,#000 20%,transparent 70%),url('../transparent/M17_series_overview_transparent_small.png')"><h3>Education</h3></article>
  </div>
</section>
<section id="features">
  <p class="k">Platform</p>
  <h2>Integrated compute for the joint.</h2>
  <p style="max-width:60ch">{INTRO} {INTRO2}</p>
  <div class="news" style="margin-top:28px">{news}</div>
</section>
<section id="models">
  <p class="k">SKU</p>
  <h2>Three packages.</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #333;color:#76B900')}
</section>
<section id="start">
  <p class="k">Developers</p>
  <h2>High-level commands. No pulse train.</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:10px;color:#888">{INSTALL} · {ARDUINO}</p>
</section>
<section id="about">
  <p class="k">Open</p>
  <h2>Firmware on GitHub.</h2>
  <p style="max-width:60ch">{COMPANY}</p>
  <p class="os" style="margin:16px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
  <a class="cta" href="{GITHUB}">GitHub</a>
</section>
<footer>
  <div>
    <strong style="color:#fff">Gearotons</strong>
    <p style="margin-top:8px">{TAGLINE}</p>
    <p style="margin-top:16px"><a class="cta" href="{SHOP}">Shop Now</a></p>
  </div>
  <div><a href="#features">Features</a><a href="#models">Models</a><a href="#start">Code</a></div>
  <div><a href="{DOCS}">Documentation</a><a href="{GITHUB}">GitHub</a></div>
  <div><a href="{SHOP}">Store</a><a href="#about">Company</a></div>
</footer>
"""
    w("v11.html", shell(TITLE, css, body))


def v12():
    """Apple — stacked product stories, SF-like, Learn more / Shop."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:17px/1.47 -apple-system,BlinkMacSystemFont,"Helvetica Neue",Helvetica,Arial,sans-serif;background:#fff;color:#1d1d1f}
img{max-width:100%;height:auto;display:block}
a{color:#0066cc;text-decoration:none}
a:hover{text-decoration:underline}
nav{position:sticky;top:0;z-index:20;background:rgba(251,251,253,.86);backdrop-filter:blur(16px);display:flex;justify-content:center;gap:28px;align-items:center;height:48px;font-size:12px;color:#1d1d1f}
nav img{height:18px}
nav a{color:#1d1d1f}
nav a.shop{color:#0066cc}
.band{padding:80px 24px 64px;text-align:center}
.band.dark{background:#000;color:#f5f5f7}
.band.gray{background:#f5f5f7}
.band h1,.band h2{font-size:clamp(36px,6vw,56px);letter-spacing:-.02em;font-weight:600;margin:8px 0}
.band p.sub{font-size:21px;color:#6e6e73;max-width:22em;margin:0 auto 16px}
.band.dark p.sub{color:#a1a1a6}
.pair{display:flex;justify-content:center;gap:24px;font-size:17px}
.band img{margin:36px auto 0;max-height:520px}
.split2{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px;background:#fff}
.split2 .cell{background:#f5f5f7;text-align:center;padding:48px 20px 0;min-height:480px}
.split2 h3{font-size:28px;font-weight:600}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;max-width:1080px;margin:0 auto}
.models article{padding:12px}
pre{text-align:left;background:#1d1d1f;color:#f5f5f7;padding:24px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace;border-radius:12px;max-width:720px;margin:24px auto}
footer{background:#f5f5f7;padding:40px 24px 64px;font-size:12px;color:#6e6e73}
.os{display:flex;justify-content:center;gap:20px;margin:16px}
.os img{height:40px}
@media(max-width:800px){.split2,.models{grid-template-columns:1fr}nav .hide{display:none}}
"""
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}<br>{m['height']} · {m['weight']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="hide" href="#features">M17</a>
  <a class="hide" href="#models">Models</a>
  <a class="hide" href="#start">Developers</a>
  <a class="hide" href="#about">About</a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<section class="band" id="top">
  <h1>M17</h1>
  <p class="sub">{TAGLINE}</p>
  <div class="pair"><a href="#features">Learn more</a><a href="{SHOP}">Shop</a></div>
  <img src="../transparent/one_motor_transparent_small.png" alt="M17">
</section>
<section class="band dark" id="features">
  <h2>Four instruments. One outline.</h2>
  <p class="sub">{INTRO}</p>
  <div class="pair"><a href="#models">See the models</a><a href="{SHOP}">Shop</a></div>
  <img src="../transparent/one_motor_transparent_small.png" alt="">
</section>
<section class="band gray">
  <h2>A bus, not a nest of wires.</h2>
  <p class="sub">RS-485 daisy-chain. High-level commands. 32 kHz closed loop. Standard NEMA 17 mounting.</p>
  <img src="../transparent/adapter_and_wire_transparent_small.png" alt="Adapter" style="max-width:560px">
</section>
<div class="split2">
  <div class="cell">
    <h3>For the bench.</h3>
    <p class="sub" style="font-size:17px">Raspberry Pi, Arduino, ESP32, Mac, PC.</p>
    <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="">
  </div>
  <div class="cell">
    <h3>For the rack.</h3>
    <p class="sub" style="font-size:17px">Robotics, CNC, instruments, test jigs.</p>
    <img src="../test_rack_small.jpg" alt="">
  </div>
</div>
<section class="band" id="models">
  <h2>Three sizes.</h2>
  <p class="sub">Same control. Three torques.</p>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #d2d2d7', 'max-width:900px;margin:32px auto 0;text-align:left')}
</section>
<section class="band gray" id="start">
  <h2>One revolution. Two seconds.</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="color:#6e6e73">{INSTALL} · {ARDUINO}</p>
</section>
<section class="band" id="about">
  <h2>Open, on purpose.</h2>
  <p class="sub">{COMPANY}</p>
  <div class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </div>
  <div class="pair"><a href="{GITHUB}">GitHub</a><a href="{DOCS}">Documentation</a></div>
</section>
<footer>
  <p>Gearotons · Shenzhen · 2022</p>
  <p style="margin-top:12px"><a href="{SHOP}">Shop Now</a> · <a href="{DOCS}">Documentation</a></p>
</footer>
"""
    w("v12.html", shell(TITLE, css, body))


def v13():
    """Google / about.google — friendly, colorful dots, rounded wells."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.6 Arial,"Gill Sans",Helvetica,sans-serif;background:#fff;color:#202124}
img{max-width:100%;height:auto;display:block}
a{color:#1a73e8;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 32px;background:#fff}
nav img{height:28px}
nav ul{display:flex;gap:20px;list-style:none;font-size:14px}
.dots span{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:4px}
.shop{background:#1a73e8;color:#fff;padding:10px 22px;border-radius:24px;font-weight:600}
.shop:hover{background:#174ea6;color:#fff}
.hero{padding:72px 32px 40px;text-align:center;max-width:980px;margin:0 auto}
.hero h1{font-size:clamp(36px,6vw,64px);font-weight:500;letter-spacing:-.03em;line-height:1.1}
.hero p{font-size:20px;color:#5f6368;margin:16px auto 28px;max-width:28em}
.well{border-radius:32px;overflow:hidden;margin:32px auto;max-width:1100px}
.well img{width:100%}
.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:20px;max-width:1100px;margin:0 auto;padding:24px 32px 64px}
.cards article{background:#f8f9fa;border-radius:24px;overflow:hidden}
.cards img{height:180px;width:100%;object-fit:cover}
.cards div{padding:18px}
h2{font-size:32px;font-weight:500;text-align:center;margin:48px 0 12px}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:16px;max-width:900px;margin:0 auto;padding:0 32px 48px}
.feats article{padding:16px 0;border-top:1px solid #e8eaed}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;max-width:1100px;margin:0 auto;padding:0 32px}
.models article{background:#fff;border:1px solid #e8eaed;border-radius:24px;padding:16px}
pre{background:#202124;color:#e8eaed;padding:20px;overflow:auto;border-radius:16px;font:13px/1.55 ui-monospace,Menlo,monospace;max-width:900px;margin:0 auto}
footer{padding:48px 32px 72px;text-align:center}
.os{display:flex;justify-content:center;gap:20px;margin:16px}
.os img{height:44px}
@media(max-width:800px){.cards,.models,.feats{grid-template-columns:1fr}nav ul{display:none}}
"""
    feats = "".join(f"<article><strong>{t}</strong><p>{d}</p></article>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} · {m['weight']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons">
    <span class="dots" style="margin-left:8px"><span style="background:#4285F4"></span><span style="background:#EA4335"></span><span style="background:#FBBC05"></span><span style="background:#34A853"></span></span>
  </a>
  <ul>
    <li><a href="#features">Product</a></li>
    <li><a href="#models">Models</a></li>
    <li><a href="#start">Build</a></li>
    <li><a href="#about">Company</a></li>
  </ul>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <h1>Motion control,<br>made simple.</h1>
  <p>{TAGLINE}. Motor, driver, controller, and encoder — one small package you can talk to from any computer.</p>
  <a class="shop" href="{SHOP}">Shop Now</a>
</header>
<div class="well"><img src="../transparent/M17_series_overview_transparent_small.png" alt="M17 series"></div>
<h2>Made for people who build</h2>
<div class="cards">
  <article><img src="../robotics_small.jpg" alt=""><div><h3>Robotics &amp; CNC</h3><p>Closed loop at 32 kHz. High-level moves, not pulse trains.</p></div></article>
  <article><img src="../automation_small.jpg" alt=""><div><h3>Labs &amp; instruments</h3><p>12–24 V. Standard NEMA 17. Daisy-chain on RS-485.</p></div></article>
  <article><img src="../transparent/kit_with_three_motors_transparent_small.png" alt=""><div><h3>Classrooms</h3><p>Python and Arduino. Documentation an AI can finish with you.</p></div></article>
</div>
<section id="features">
  <h2>What you get</h2>
  <p style="text-align:center;color:#5f6368;max-width:40em;margin:0 auto 24px">{INTRO}</p>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>Three sizes</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #e8eaed', 'max-width:1100px;margin:24px auto;padding:0 32px')}
</section>
<section id="start" style="padding:48px 32px">
  <h2>Start here</h2>
  <p style="text-align:center;color:#5f6368;margin-bottom:16px">{INTRO2}</p>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="text-align:center;margin-top:10px;color:#5f6368">{INSTALL} · {ARDUINO}</p>
</section>
<footer id="about">
  <h2>About Gearotons</h2>
  <p style="max-width:40em;margin:0 auto;color:#5f6368">{COMPANY}</p>
  <div class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </div>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <p style="margin-top:16px"><a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v13.html", shell(TITLE, css, body))


def v14():
    """Microsoft — Fluent tiles, Segoe, mega-nav, #0078D4."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:15px/1.5 "Segoe UI",SegoeUI,Tahoma,Geneva,sans-serif;background:#fff;color:#1b1b1b}
img{max-width:100%;height:auto;display:block}
a{color:#0067b8;text-decoration:none}
nav{position:sticky;top:0;z-index:20;background:#fff;border-bottom:1px solid #e6e6e6}
.nav1{display:flex;align-items:center;gap:18px;padding:10px 24px;flex-wrap:wrap}
nav img{height:24px}
.nav1 a{color:#1b1b1b;font-size:13px}
.shop{background:#0067b8;color:#fff!important;padding:8px 14px}
.shop:hover{background:#005a9e}
.hero{display:grid;grid-template-columns:1fr 1fr;gap:24px;align-items:center;padding:48px 24px;max-width:1200px;margin:0 auto}
.hero h1{font-size:clamp(28px,4vw,44px);font-weight:600}
.tiles{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;padding:12px 24px 48px;max-width:1200px;margin:0 auto}
.tiles article{background:#f2f2f2;padding:0 0 16px}
.tiles img{height:160px;width:100%;object-fit:cover}
.tiles h3{padding:12px 14px 0;font-size:16px}
.tiles p{padding:6px 14px;font-size:13px}
.tiles a{padding:0 14px;font-weight:600}
h2{font-size:24px;padding:24px 24px 0;max-width:1200px;margin:0 auto}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;padding:16px 24px;max-width:1200px;margin:0 auto}
.models article{border:1px solid #e6e6e6;padding:14px}
pre{background:#1b1b1b;color:#eee;padding:18px;overflow:auto;font:12.5px/1.55 Consolas,monospace;margin:16px 24px;max-width:1152px}
footer{background:#f2f2f2;padding:32px 24px;display:grid;grid-template-columns:repeat(4,1fr);gap:16px;font-size:12px}
.os img{height:40px;margin-right:8px}
@media(max-width:800px){.hero,.tiles,.models,footer{grid-template-columns:1fr}}
"""
    tiles = [
        (FEATURES[0][0], FEATURES[0][1], "../transparent/one_motor_transparent_small.png"),
        (FEATURES[1][0], FEATURES[1][1], "../transparent/adapter_and_wire_transparent_small.png"),
        (FEATURES[6][0], FEATURES[6][1], "../transparent/motor_back_transparent_small.png"),
        (FEATURES[11][0], FEATURES[11][1], "../transparent/kit_with_three_motors_transparent_small.png"),
        (FEATURES[2][0], FEATURES[2][1], "../connection_diagram.jpg"),
        (FEATURES[14][0], FEATURES[14][1], "../robotics_small.jpg"),
        (FEATURES[4][0], FEATURES[4][1], "../M17-60_dimensions.png"),
        (FEATURES[13][0], FEATURES[13][1], "../automation_small.jpg"),
    ]
    tile_html = "".join(
        f"<article><img src='{img}' alt=''><h3>{t}</h3><p>{d}</p><a href='{SHOP}'>Shop now &gt;</a></article>"
        for t, d, img in tiles
    )
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} · {m['power']}</p><a href='{SHOP}'>Shop now &gt;</a></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <div class="nav1">
    <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
    <a href="#features">M17</a>
    <a href="#models">Models</a>
    <a href="#start">Developers</a>
    <a href="#apps">Solutions</a>
    <a href="#about">Company</a>
    <a href="{DOCS}">Documentation</a>
    <a href="{GITHUB}">GitHub</a>
    <span style="flex:1"></span>
    <a class="shop" href="{SHOP}">Shop</a>
  </div>
</nav>
<header class="hero" id="top">
  <div>
    <p style="color:#0067b8;font-weight:600">Hi there — welcome to Gearotons</p>
    <h1>Motion control for every bench.</h1>
    <p style="margin:12px 0 20px">{TAGLINE}. {INTRO}</p>
    <a class="shop" href="{SHOP}">Shop Now</a>
  </div>
  <img src="../transparent/M17_series_overview_transparent_small.png" alt="M17 series">
</header>
<h2 id="features">For personal and professional benches</h2>
<div class="tiles">{tile_html}</div>
<h2 id="models">Compare models</h2>
<div class="models">{models}</div>
{spec_table('border-bottom:1px solid #e6e6e6;background:#f2f2f2', 'padding:0 24px 32px;max-width:1200px;margin:0 auto')}
<h2 id="start">Get started with Python</h2>
<p style="padding:8px 24px;max-width:1200px;margin:0 auto">{INTRO2}</p>
<pre><code>{esc(CODE)}</code></pre>
<p style="padding:0 24px 32px">{INSTALL} · {ARDUINO}</p>
<h2 id="apps">Solutions</h2>
<div class="tiles">
  <article><img src="../robotics_small.jpg" alt=""><h3>Robotics</h3><p>Arms, mobiles, teaching platforms.</p></article>
  <article><img src="../automation_small.jpg" alt=""><h3>Automation</h3><p>CNC, printers, fixtures.</p></article>
  <article><img src="../test_rack_small.jpg" alt=""><h3>Test</h3><p>Racks and jigs on one bus.</p></article>
  <article><img src="../transparent/kit_with_three_motors_transparent_small.png" alt=""><h3>Education</h3><p>A full kit for a course.</p></article>
</div>
<section id="about" style="padding:24px;max-width:1200px;margin:0 auto">
  <h2 style="padding:0">Company</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin-top:12px">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
</section>
<footer>
  <div><strong>Shop</strong><p><a href="{SHOP}">Store</a></p><p><a href="#models">Models</a></p></div>
  <div><strong>Learn</strong><p><a href="{DOCS}">Docs</a></p><p><a href="#start">Python</a></p></div>
  <div><strong>Open source</strong><p><a href="{GITHUB}">GitHub</a></p></div>
  <div><strong>Gearotons</strong><p>Shenzhen · 2022</p><p><a class="shop" href="{SHOP}">Shop Now</a></p></div>
</footer>
"""
    w("v14.html", shell(TITLE, css, body))


def v15():
    """Amazon retail PDP energy."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:14px/1.5 Arial,Helvetica,sans-serif;background:#fff;color:#0f1111}
img{max-width:100%;height:auto;display:block}
a{color:#007185;text-decoration:none}
a:hover{color:#c45500;text-decoration:underline}
.top{background:#131921;color:#fff;display:flex;align-items:center;gap:16px;padding:10px 16px;position:sticky;top:0;z-index:20}
.top img{height:28px;background:#fff;border-radius:2px;padding:2px}
.search{flex:1;background:#fff;border-radius:4px;height:36px;display:flex;align-items:center;padding:0 12px;color:#888}
.shop{background:#ffd814;color:#0f1111;padding:8px 14px;border-radius:20px;font-weight:700;text-decoration:none}
.shop:hover{background:#f7ca00;text-decoration:none;color:#0f1111}
.sub{background:#232f3e;color:#fff;padding:8px 16px;font-size:13px}
.sub a{color:#fff;margin-right:16px}
.crumb{padding:10px 16px;color:#565959;font-size:12px}
.pdp{display:grid;grid-template-columns:1.2fr .8fr;gap:28px;padding:8px 16px 32px;max-width:1200px}
.buybox{border:1px solid #d5d9d9;border-radius:8px;padding:16px}
.buybox .price-row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #eee}
.orange{background:#ffa41c;color:#0f1111;display:block;text-align:center;padding:12px;border-radius:20px;font-weight:700;margin-top:12px;text-decoration:none}
.orange:hover{background:#ff8f00;text-decoration:none;color:#0f1111}
h1{font-size:24px;font-weight:500;line-height:1.3}
.bullets{padding:8px 16px 32px 36px;max-width:900px}
h2{font-size:20px;padding:16px 16px 8px;border-top:1px solid #e7e7e7}
.cmp td,.cmp th{border:1px solid #e7e7e7;padding:8px;text-align:left}
pre{background:#232f3e;color:#fff;padding:16px;overflow:auto;font:12.5px/1.5 Consolas,monospace;margin:8px 16px}
.apps{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;padding:8px 16px 32px}
.apps img{height:140px;width:100%;object-fit:cover}
footer{background:#232f3e;color:#ddd;padding:32px 16px;font-size:12px}
.os img{height:40px;background:#fff;padding:3px;margin-right:8px}
@media(max-width:800px){.pdp,.apps{grid-template-columns:1fr}}
"""
    bullets = "".join(f"<li><strong>{t}.</strong> {d}</li>" for t, d in FEATURES[:8])
    body = f"""
<div class="top">
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <div class="search">Search Gearotons</div>
  <a class="shop" href="{SHOP}">Shop</a>
</div>
<div class="sub">
  <a href="#features">Features</a>
  <a href="#models">Compare</a>
  <a href="#start">Setup</a>
  <a href="#apps">Best uses</a>
  <a href="#about">About</a>
</div>
<p class="crumb">Motion control › Servomotors › NEMA 17 › M17 Series</p>
<div class="pdp" id="top">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17">
  <div>
    <h1>Gearotons M17 Series Servomotor — all-in-one motor, driver, controller &amp; encoder</h1>
    <p style="margin:8px 0;color:#007185">Visit the Gearotons store</p>
    <p>{TAGLINE}</p>
    <div class="buybox">
      <div class="price-row"><span>Voltage</span><strong>12–24 V</strong></div>
      <div class="price-row"><span>Max speed</span><strong>560 RPM</strong></div>
      <div class="price-row"><span>Loop</span><strong>32 kHz PID</strong></div>
      <div class="price-row"><span>Mount</span><strong>NEMA 17</strong></div>
      <div class="price-row"><span>Models</span><strong>M17-60 · 48 · 40</strong></div>
      <a class="orange" href="{SHOP}">Shop Now</a>
      <p style="margin-top:10px;font-size:12px">Ships from the Gearotons store. Documentation at gearotons.com.</p>
    </div>
  </div>
</div>
<h2 id="features">About this item</h2>
<ul class="bullets">{bullets}</ul>
<p style="padding:0 16px 24px;max-width:70ch">{INTRO} {INTRO2}</p>
<h2 id="models">Compare with similar items</h2>
{spec_table('background:#f0f2f2;border-bottom:1px solid #e7e7e7', 'padding:8px 16px 24px')}
<div style="display:grid;grid-template-columns:repeat(3,1fr);gap:12px;padding:8px 16px 24px">
  {''.join(f"<article style='border:1px solid #e7e7e7;padding:8px'><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>" for m in MODELS)}
</div>
<h2 id="start">From the manufacturer — getting started</h2>
<pre><code>{esc(CODE)}</code></pre>
<p style="padding:0 16px 24px">{INSTALL} · {ARDUINO}</p>
<h2 id="apps">Customers also use these for</h2>
<div class="apps">
  <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
  <figure><img src="../automation_small.jpg" alt=""><figcaption>Automation</figcaption></figure>
  <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Testing</figcaption></figure>
  <figure><img src="../transparent/kit_with_three_motors_transparent_small.png" alt=""><figcaption>Teaching kits</figcaption></figure>
</div>
<section id="about" style="padding:16px">
  <h2 style="border:0;padding-left:0">About Gearotons</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin-top:12px">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
</section>
<footer>
  <p><a href="{SHOP}" style="color:#ffd814">Shop Now</a> · <a href="{DOCS}" style="color:#fff">Documentation</a> · <a href="{GITHUB}" style="color:#fff">GitHub</a></p>
  <p style="margin-top:12px">Gearotons · Shenzhen · 2022</p>
</footer>
"""
    w("v15.html", shell(TITLE, css, body))


def v16():
    """TSMC — navy, gold, formal, process nodes as models."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.6 Georgia,"Times New Roman",serif;background:#001028;color:#e8e0d0}
img{max-width:100%;height:auto;display:block}
a{color:#c5a46e}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 32px;background:#001028;border-bottom:1px solid #c5a46e}
nav img{height:26px;filter:invert(1)}
.shop{border:1px solid #c5a46e;padding:8px 16px;text-decoration:none;letter-spacing:.14em;text-transform:uppercase;font-size:12px;font-family:Helvetica,Arial,sans-serif}
.shop:hover{background:#c5a46e;color:#001028}
.hero{padding:72px 32px;max-width:1100px;margin:0 auto;display:grid;grid-template-columns:1fr 1fr;gap:40px;align-items:center}
h1{font-size:clamp(32px,5vw,52px);font-weight:400;color:#fff}
.gold{color:#c5a46e;font-family:Helvetica,Arial,sans-serif;font-size:11px;letter-spacing:.22em;text-transform:uppercase}
section{padding:56px 32px;max-width:1100px;margin:0 auto;border-top:1px solid #1a3358}
h2{font-weight:400;font-size:32px;color:#fff;margin:8px 0 16px}
.nodes{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
.nodes article{border:1px solid #c5a46e;padding:20px}
.nodes h3{color:#c5a46e;font-weight:400}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:20px}
pre{background:#000814;border:1px solid #1a3358;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace;color:#d7c49a}
.apps img{height:180px;width:100%;object-fit:cover;filter:sepia(.4) hue-rotate(180deg)}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.os img{height:44px;background:#fff;padding:4px;margin-right:10px}
table{width:100%;border-collapse:collapse}
th,td{border:1px solid #1a3358;padding:8px;text-align:left}
@media(max-width:800px){.hero,.nodes,.feats,.apps{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article><p class='gold'>Process</p><h3>{m['name']}</h3><img src='{m['dim']}' alt='' style='background:#fff;margin:12px 0'><p>Torque node {m['torque']}<br>Stack {m['height']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <p class="gold">Technology · M17 platform</p>
    <h1>Trusted motion, manufactured as a family.</h1>
    <p style="margin-top:16px">{TAGLINE}</p>
    <p style="margin-top:12px">{INTRO}</p>
  </div>
  <img src="../transparent/M17_series_overview_transparent_small.png" alt="">
</header>
<section id="models">
  <p class="gold">Technology nodes</p>
  <h2>Three heights. One control layer.</h2>
  <div class="nodes">{models}</div>
  <div style="overflow:auto;margin-top:24px">{spec_table('color:#c5a46e;border-bottom:1px solid #1a3358')}</div>
</section>
<section id="features">
  <p class="gold">Capabilities</p>
  <h2>Design rules</h2>
  <div class="feats">{feats}</div>
  <p style="margin-top:20px">{INTRO2}</p>
</section>
<section id="start">
  <p class="gold">Customer enablement</p>
  <h2>Host listing</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <p class="gold">Applications</p>
  <h2>End markets</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
  </div>
</section>
<section id="about">
  <p class="gold">Corporate</p>
  <h2>Open process</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin:16px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <a class="shop" href="{DOCS}">Documentation</a>
  <a class="shop" href="{GITHUB}">GitHub</a>
</section>
"""
    w("v16.html", shell(TITLE, css, body))


def v17():
    """SpaceX — black, Futura, full-bleed, outline buttons."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.45 Futura,"Century Gothic","Avenir Next",Avenir,Helvetica,sans-serif;background:#000;color:#fff}
img{max-width:100%;height:auto;display:block}
a{color:#fff;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:18px 28px;background:linear-gradient(#000,transparent)}
nav img{height:22px;filter:invert(1)}
nav ul{display:flex;gap:22px;list-style:none;font-size:13px;letter-spacing:.12em;text-transform:uppercase}
.out{border:2px solid #fff;padding:10px 18px;letter-spacing:.14em;text-transform:uppercase;font-size:13px}
.out:hover{background:#fff;color:#000}
.hero{min-height:100vh;display:flex;flex-direction:column;justify-content:flex-end;padding:40px 6vw 80px;background:#000;position:relative}
.hero img{position:absolute;right:4vw;top:10vh;max-height:80vh}
.hero h1{font-size:clamp(56px,12vw,140px);font-weight:500;letter-spacing:.08em;line-height:.85}
.full{min-height:70vh;display:flex;align-items:flex-end;padding:40px 6vw;background-size:cover;background-position:center}
.full h2{font-size:clamp(32px,6vw,64px);text-transform:uppercase;letter-spacing:.06em}
.pad{padding:80px 6vw}
.feats{display:grid;grid-template-columns:1fr 1fr 1fr;gap:24px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
pre{background:#0a0a0a;padding:20px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace;border:1px solid #222}
.os img{height:44px;background:#fff;padding:4px;margin-right:10px}
@media(max-width:800px){nav ul{display:none}.hero img{position:static;max-height:40vh}.feats,.models{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p style='color:#bbb;margin-top:8px'>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt='' style='background:#fff;margin:12px 0'><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <ul>
    <li><a href="#features">Vehicle</a></li>
    <li><a href="#models">Family</a></li>
    <li><a href="#start">GNC</a></li>
    <li><a href="#about">Updates</a></li>
  </ul>
  <a class="out" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17">
  <p style="letter-spacing:.3em;text-transform:uppercase;font-size:12px">Gearotons</p>
  <h1>M17</h1>
  <p style="max-width:22em;margin:16px 0 24px">{TAGLINE}</p>
  <a class="out" href="{SHOP}">Shop Now</a>
</header>
<section class="full" style="background-image:linear-gradient(to top,#000 10%,transparent 50%),url('../robotics_small.jpg')" id="features">
  <div>
    <h2>Integrated.</h2>
    <p style="max-width:28em;margin-top:8px">{INTRO}</p>
  </div>
</section>
<section class="pad">
  <div class="feats">{feats}</div>
</section>
<section class="full" style="background-image:linear-gradient(to top,#000 15%,transparent 55%),url('../test_rack_small.jpg')">
  <div>
    <h2>Count the motors.</h2>
    <p style="max-width:28em;margin-top:8px">One RS-485 bus. Any number of axes. {INTRO2}</p>
  </div>
</section>
<section class="pad" id="models">
  <h2>Family</h2>
  <div class="models" style="margin-top:24px">{models}</div>
  {spec_table('border-bottom:1px solid #333', 'margin-top:24px')}
</section>
<section class="pad" id="start">
  <h2>Guidance</h2>
  <pre style="margin-top:16px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px;color:#888">{INSTALL} · {ARDUINO}</p>
</section>
<section class="pad" id="about">
  <h2>Open source</h2>
  <p style="max-width:40em;margin:12px 0">{COMPANY}</p>
  <p class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
  <a class="out" href="{SHOP}">Shop Now</a>
  <a class="out" href="{DOCS}">Docs</a>
  <a class="out" href="{GITHUB}">GitHub</a>
</section>
"""
    w("v17.html", shell(TITLE, css, body))


def v18():
    """Broadcom — enterprise catalog, boxed categories."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:15px/1.5 Arial,Helvetica,sans-serif;background:#fff;color:#222}
img{max-width:100%;height:auto;display:block}
a{color:#0066a1;text-decoration:none}
.util{background:#003d63;color:#fff;font-size:12px;padding:6px 24px;display:flex;justify-content:flex-end;gap:16px}
.util a{color:#fff}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 24px;background:#fff;border-bottom:3px solid #0066a1}
nav img{height:28px}
.shop{background:#0066a1;color:#fff;padding:8px 14px}
.shop:hover{background:#003d63}
.hero{padding:36px 24px;background:#f4f7fa;display:grid;grid-template-columns:1.2fr .8fr;gap:24px;align-items:center}
.hero h1{font-size:32px;color:#003d63}
.cats{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;padding:24px}
.cats article{border:1px solid #d0d7de;padding:16px;min-height:140px}
.cats article:hover{border-color:#0066a1}
.cats h3{color:#003d63;margin-bottom:8px}
.arrow{color:#0066a1;font-weight:700}
h2{font-size:22px;color:#003d63;padding:24px 24px 8px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;padding:8px 24px 24px}
.models article{border:1px solid #d0d7de;padding:12px}
pre{background:#0b1f2e;color:#d6e8f5;padding:16px;overflow:auto;font:12.5px/1.5 Consolas,monospace;margin:8px 24px}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;padding:8px 24px 32px}
.apps img{height:150px;width:100%;object-fit:cover}
footer{background:#003d63;color:#fff;padding:32px 24px}
footer a{color:#fff}
.os img{height:40px;background:#fff;padding:3px;margin-right:8px}
@media(max-width:800px){.hero,.cats,.models,.apps{grid-template-columns:1fr}}
"""
    cats = "".join(
        f"<article><h3>{t}</h3><p>{d}</p><p class='arrow'>Explore →</p></article>"
        for t, d in FEATURES[:8]
    )
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} · {m['power']}</p></article>"
        for m in MODELS
    )
    body = f"""
<div class="util"><a href="{DOCS}">Support</a><a href="{GITHUB}">Developers</a></div>
<nav>
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <p style="color:#0066a1;font-weight:700;letter-spacing:.06em;text-transform:uppercase;font-size:12px">Products · Motion semiconductors in a package</p>
    <h1>Connecting every axis on one bus.</h1>
    <p style="margin:12px 0">{TAGLINE}. {INTRO}</p>
    <a class="shop" href="#models">Explore products</a>
  </div>
  <img src="../transparent/one_motor_transparent_small.png" alt="">
</header>
<h2 id="features">Product categories</h2>
<div class="cats">{cats}</div>
<h2 id="models">M17 family</h2>
<div class="models">{models}</div>
{spec_table('background:#f4f7fa;border-bottom:1px solid #d0d7de', 'padding:0 24px 24px')}
<h2 id="start">Software &amp; tools</h2>
<p style="padding:0 24px">{INTRO2}</p>
<pre><code>{esc(CODE)}</code></pre>
<p style="padding:8px 24px">{INSTALL} · {ARDUINO}</p>
<h2>Solutions</h2>
<div class="apps">
  <img src="../robotics_small.jpg" alt="">
  <img src="../automation_small.jpg" alt="">
  <img src="../test_rack_small.jpg" alt="">
</div>
<section id="about" style="padding:24px">
  <h2 style="padding:0">About</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin-top:12px">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
</section>
<footer>
  <p>© Gearotons · Shenzhen · 2022</p>
  <p style="margin-top:12px"><a href="{SHOP}">Shop Now</a> · <a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v18.html", shell(TITLE, css, body))


def v19():
    """Aramco — teal, sand, at-a-glance stats, magazine tiles."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.6 Georgia,"Palatino Linotype",Palatino,serif;background:#f6efe3;color:#1a2a28}
img{max-width:100%;height:auto;display:block}
a{color:#004c45}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#004c45}
nav img{height:26px;filter:invert(1)}
.shop{background:#e8d9c0;color:#004c45;padding:8px 16px;text-decoration:none;font-family:Helvetica,Arial,sans-serif;font-size:13px;letter-spacing:.08em;text-transform:uppercase}
.shop:hover{background:#fff}
.hero{min-height:70vh;display:flex;align-items:flex-end;padding:40px 6vw;background:linear-gradient(to top,#004c45 0%,transparent 55%),url('../automation_small.jpg') center/cover}
.hero h1{font-size:clamp(36px,6vw,64px);color:#fff;font-weight:400;max-width:16ch}
.glance{display:grid;grid-template-columns:repeat(4,1fr);background:#004c45;color:#e8d9c0}
.glance div{padding:28px 20px;border-right:1px solid #16665e}
.glance b{display:block;font-size:32px;color:#fff;font-family:Helvetica,Arial,sans-serif}
section{padding:56px 6vw}
h2{font-weight:400;font-size:32px;color:#004c45}
.mag{display:grid;grid-template-columns:2fr 1fr 1fr;gap:12px;margin-top:24px}
.mag article{background:#fff;position:relative}
.mag img{height:240px;width:100%;object-fit:cover}
.mag h3{padding:12px 14px}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:16px;margin-top:16px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.models article{background:#fff;padding:14px;border-top:4px solid #004c45}
pre{background:#1a2a28;color:#e8d9c0;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.os img{height:44px;background:#fff;padding:4px;margin-right:8px}
footer{background:#004c45;color:#e8d9c0;padding:40px 6vw}
footer a{color:#e8d9c0}
@media(max-width:800px){.glance,.mag,.feats,.models{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} · {m['power']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <p style="color:#e8d9c0;letter-spacing:.18em;text-transform:uppercase;font-size:12px;font-family:Helvetica,Arial,sans-serif">How energy becomes motion</p>
    <h1>Where a joint is an opportunity.</h1>
  </div>
</header>
<div class="glance">
  <div><b>32 kHz</b>control loop</div>
  <div><b>560</b>RPM</div>
  <div><b>12–24 V</b>rail</div>
  <div><b>3</b>models</div>
</div>
<section>
  <h2>{TAGLINE}</h2>
  <p style="max-width:60ch;margin-top:12px">{INTRO}</p>
</section>
<section id="features">
  <h2>Elements</h2>
  <div class="mag">
    <article><img src="../transparent/one_motor_transparent_small.png" alt=""><h3>The package</h3></article>
    <article><img src="../transparent/adapter_and_wire_transparent_small.png" alt=""><h3>The bus</h3></article>
    <article><img src="../transparent/motor_back_transparent_small.png" alt=""><h3>The label</h3></article>
  </div>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>The field</h2>
  <div class="models">{models}</div>
  {spec_table('background:#004c45;color:#e8d9c0', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Engineering notes</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <h2>Projects</h2>
  <div class="mag">
    <article><img src="../robotics_small.jpg" alt=""><h3>Robotics</h3></article>
    <article><img src="../automation_small.jpg" alt=""><h3>Industry</h3></article>
    <article><img src="../test_rack_small.jpg" alt=""><h3>Testing</h3></article>
  </div>
</section>
<section id="about">
  <h2>About</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin-top:12px">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
</section>
<footer>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <p style="margin-top:16px"><a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v19.html", shell(TITLE, css, body))


def v20():
    """Tesla — extreme reduction, centered, thin type."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:15px/1.45 "Avenir Next",Avenir,"Helvetica Neue",Helvetica,sans-serif;background:#fff;color:#171a20;font-weight:400}
img{max-width:100%;height:auto;display:block}
a{color:#171a20;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:16px 28px;background:rgba(255,255,255,.85);backdrop-filter:blur(12px)}
nav img{height:20px}
nav ul{display:flex;gap:20px;list-style:none;font-size:14px}
.hero{min-height:92vh;display:flex;flex-direction:column;align-items:center;justify-content:flex-start;padding:48px 20px 0;text-align:center}
.hero h1{font-size:clamp(40px,6vw,56px);font-weight:500;letter-spacing:.04em}
.hero p{color:#5c5e62;margin-top:8px}
.btns{display:flex;gap:16px;margin:24px 0 32px}
.b1{background:#171a20;color:#fff;padding:10px 48px;border-radius:4px;min-width:200px}
.b2{background:#eee;padding:10px 48px;border-radius:4px;min-width:200px}
.b1:hover{background:#393c41;color:#fff}
.b2:hover{background:#ddd}
section{padding:72px 20px;text-align:center}
h2{font-weight:500;font-size:32px;letter-spacing:.02em}
.specstrip{display:flex;justify-content:center;gap:48px;margin:32px 0;flex-wrap:wrap}
.specstrip b{display:block;font-size:28px;font-weight:500}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:960px;margin:0 auto}
pre{text-align:left;background:#f4f4f4;padding:20px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace;max-width:720px;margin:20px auto}
.wide img{width:100%;max-height:70vh;object-fit:cover}
.os{display:flex;justify-content:center;gap:16px;margin:16px}
.os img{height:40px}
@media(max-width:800px){nav ul{display:none}.models,.btns{grid-template-columns:1fr;flex-direction:column}}
"""
    models = "".join(
        f"<article><img src='{m['dim']}' alt=''><h3 style='margin:12px 0 4px;font-weight:500'>{m['name']}</h3><p style='color:#5c5e62'>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <ul>
    <li><a href="#models">M17-60</a></li>
    <li><a href="#models">M17-48</a></li>
    <li><a href="#models">M17-40</a></li>
    <li><a href="#start">Software</a></li>
  </ul>
  <a href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <h1>M17</h1>
  <p>{TAGLINE}</p>
  <div class="btns">
    <a class="b1" href="{SHOP}">Shop Now</a>
    <a class="b2" href="{DOCS}">Documentation</a>
  </div>
  <img src="../transparent/one_motor_transparent_small.png" alt="M17" style="max-height:56vh">
</header>
<section id="features">
  <h2>All-in-one</h2>
  <p style="max-width:36em;margin:12px auto;color:#5c5e62">{INTRO}</p>
  <div class="specstrip">
    <div><b>32 kHz</b><span>closed loop</span></div>
    <div><b>560 RPM</b><span>maximum</span></div>
    <div><b>12–24 V</b><span>input</span></div>
    <div><b>NEMA 17</b><span>mount</span></div>
  </div>
</section>
<div class="wide"><img src="../robotics_small.jpg" alt=""></div>
<section>
  <h2>One connection</h2>
  <p style="max-width:36em;margin:12px auto;color:#5c5e62">{INTRO2}</p>
</section>
<section id="models">
  <h2>Models</h2>
  <div class="models" style="margin-top:28px">{models}</div>
  {spec_table('border-bottom:1px solid #eee', 'max-width:900px;margin:32px auto 0;text-align:left')}
</section>
<section id="start">
  <h2>Software</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="color:#5c5e62">{INSTALL} · {ARDUINO}</p>
</section>
<div class="wide"><img src="../test_rack_small.jpg" alt=""></div>
<section id="about">
  <h2>Gearotons</h2>
  <p style="max-width:36em;margin:12px auto;color:#5c5e62">{COMPANY}</p>
  <div class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </div>
  <div class="btns" style="justify-content:center">
    <a class="b1" href="{SHOP}">Shop Now</a>
    <a class="b2" href="{GITHUB}">GitHub</a>
  </div>
</section>
"""
    w("v20.html", shell(TITLE, css, body))


if __name__ == "__main__":
    v11(); v12(); v13(); v14(); v15()
    v16(); v17(); v18(); v19(); v20()
