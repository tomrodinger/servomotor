#!/usr/bin/env python3
"""Round 2 — v21–v30 company pastiches."""
from pathlib import Path
from _content import (
    TITLE, TAGLINE, SHOP, DOCS, GITHUB, CODE, INSTALL, ARDUINO,
    MODELS, FEATURES, INTRO, INTRO2, COMPANY, spec_rows, esc, shell,
)

OUT = Path(__file__).parent


def w(name, html):
    (OUT / name).write_text(html, encoding="utf-8")
    print("wrote", name, len(html))


def spec_table(th="", wrap=""):
    return f"""<div style="overflow:auto;{wrap}">
    <table style="width:100%;border-collapse:collapse">
      <thead><tr>
        <th style="text-align:left;padding:10px 8px;{th}">Parameter</th>
        <th style="text-align:left;padding:10px 8px;{th}">M17-60</th>
        <th style="text-align:left;padding:10px 8px;{th}">M17-48</th>
        <th style="text-align:left;padding:10px 8px;{th}">M17-40</th>
      </tr></thead>
      <tbody>{spec_rows()}</tbody>
    </table></div>"""


def v21():
    """Meta — stacked statements, horizon gradient, rounded."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:18px/1.45 "Avenir Next",Avenir,Helvetica,sans-serif;background:#fff;color:#1c2b33}
img{max-width:100%;height:auto;display:block}
a{color:#1c2b33;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:16px 28px;background:#fff}
nav img{height:24px}
.shop{background:#1c2b33;color:#fff;padding:10px 20px;border-radius:999px;font-size:14px}
.shop:hover{background:#0668e1;color:#fff}
.stmt{padding:20vh 8vw 16vh;font-size:clamp(36px,6.5vw,72px);font-weight:500;letter-spacing:-.035em;line-height:1.05}
.stmt span{display:block;color:#8a96a3}
.hero-img{padding:0 8vw 10vh}
.hero-img img{width:100%;border-radius:28px;max-height:70vh;object-fit:cover}
section{padding:72px 8vw}
h2{font-size:clamp(28px,4vw,44px);font-weight:500;letter-spacing:-.03em}
.soft{display:grid;grid-template-columns:1fr 1fr;gap:16px;margin-top:28px}
.soft article{background:#f1f4f7;border-radius:24px;padding:24px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
.models article{background:#f1f4f7;border-radius:24px;padding:20px}
pre{background:#1c2b33;color:#e7eef5;padding:22px;overflow:auto;border-radius:20px;font:13px/1.55 ui-monospace,Menlo,monospace}
.apps{display:grid;grid-template-columns:1.2fr 1fr;gap:16px}
.apps img{width:100%;border-radius:24px;height:280px;object-fit:cover}
.os img{height:44px;margin-right:10px}
@media(max-width:800px){.soft,.models,.apps{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p style='color:#5b6b75;margin-top:8px'>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt='' style='border-radius:12px;margin:12px 0'><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header id="top">
  <p class="stmt">We're building<br>all-in-one motion.<span>And the bus that makes it possible.</span></p>
  <div class="hero-img"><img src="../transparent/M17_series_overview_transparent_small.png" alt=""></div>
</header>
<section id="features">
  <h2>New ways to connect a shaft to a thought.</h2>
  <p style="max-width:40em;color:#5b6b75;margin-top:12px">{INTRO} {TAGLINE}.</p>
  <div class="soft">{feats}</div>
</section>
<section id="models">
  <h2>Three bodies. One conversation.</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #e3e8ed', 'margin-top:24px')}
</section>
<section id="start">
  <h2>Speak in degrees.</h2>
  <p style="color:#5b6b75;margin:12px 0">{INTRO2}</p>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:10px;color:#5b6b75">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
  </div>
</section>
<section id="about">
  <h2>Open, and committed to it.</h2>
  <p style="max-width:40em;color:#5b6b75;margin:12px 0">{COMPANY}</p>
  <p class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
  <p style="margin-top:20px"><a class="shop" href="{SHOP}">Shop Now</a>
  <a href="{DOCS}" style="margin-left:16px">Documentation</a>
  <a href="{GITHUB}" style="margin-left:12px">GitHub</a></p>
</section>
"""
    w("v21.html", shell(TITLE, css, body))


def v22():
    """Samsung — dark cinematic, Learn more / Buy, product stages."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.5 "Avenir Next",Avenir,Helvetica,sans-serif;background:#000;color:#fff}
img{max-width:100%;height:auto;display:block}
a{color:#fff;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#000}
nav img{height:24px;filter:invert(1)}
.buy{color:#2189ff}
.buy:hover{text-decoration:underline}
.hero{min-height:88vh;display:flex;flex-direction:column;align-items:center;justify-content:center;text-align:center;padding:40px 20px;background:radial-gradient(circle at 50% 40%,#1a2744,#000 70%)}
.hero h1{font-size:clamp(36px,6vw,64px);font-weight:600}
.links{margin:16px 0 28px;display:flex;gap:28px;color:#2189ff}
.stage{padding:80px 6vw;text-align:center;border-top:1px solid #1a1a1a}
.stage h2{font-size:clamp(28px,4vw,44px);font-weight:600}
.row3{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-top:36px}
.row3 article{background:#0d0d0d;padding:20px}
pre{text-align:left;background:#111;padding:20px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace;max-width:800px;margin:20px auto}
.os img{height:42px;background:#fff;padding:4px;margin:0 6px}
@media(max-width:800px){.row3{grid-template-columns:1fr}}
"""
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='../transparent/one_motor_transparent_small.png' alt='' style='max-height:220px;margin:12px auto'><p>{m['torque']} · {m['power']}</p><p class='links' style='justify-content:center'><a href='{SHOP}' class='buy'>Buy</a></p></article>"
        for m in MODELS
    )
    feats = "".join(f"<article><h3>{t}</h3><p style='color:#aaa;margin-top:8px'>{d}</p></article>" for t, d in FEATURES[:6])
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="buy" href="{SHOP}">Buy</a>
</nav>
<header class="hero" id="top">
  <p style="letter-spacing:.2em;text-transform:uppercase;font-size:12px;color:#2189ff">M17 Series</p>
  <h1>Experience a whole new axis.</h1>
  <p style="color:#bbb;margin-top:8px">{TAGLINE}</p>
  <p class="links"><a href="#features">Learn more</a><a class="buy" href="{SHOP}">Buy</a></p>
  <img src="../transparent/one_motor_transparent_small.png" alt="M17" style="max-height:48vh">
</header>
<section class="stage" id="models">
  <h2>The lineup</h2>
  <p style="color:#aaa">{INTRO}</p>
  <div class="row3">{models}</div>
</section>
<section class="stage" id="features">
  <h2>Answering the bench</h2>
  <div class="row3">{feats}</div>
  {spec_table('border-bottom:1px solid #333', 'max-width:960px;margin:32px auto 0;text-align:left')}
</section>
<section class="stage" id="start">
  <h2>Start moving</h2>
  <p style="color:#aaa;max-width:40em;margin:0 auto">{INTRO2}</p>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="color:#888">{INSTALL} · {ARDUINO}</p>
</section>
<section class="stage">
  <h2>In the world</h2>
  <div class="row3">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
  </div>
</section>
<section class="stage" id="about">
  <h2>Open source</h2>
  <p style="max-width:36em;margin:12px auto;color:#aaa">{COMPANY}</p>
  <p>
    <img src="../Open-source-hardware-logo.svg.png" alt="" class="os" style="height:42px;background:#fff;padding:4px">
    <img src="../Open_Source_Initiative.svg.png" alt="" style="height:42px;background:#fff;padding:4px">
  </p>
  <p class="links" style="justify-content:center"><a href="{DOCS}">Learn more</a><a class="buy" href="{SHOP}">Buy</a><a href="{GITHUB}">GitHub</a></p>
</section>
"""
    w("v22.html", shell(TITLE, css, body))


def v23():
    """Micron — navy/cyan bento, datasheet energy."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.5 "Avenir Next",Avenir,Helvetica,sans-serif;background:#07131f;color:#d7e6f2}
img{max-width:100%;height:auto;display:block}
a{color:#00a3e0}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 24px;background:#07131f;border-bottom:1px solid #143049}
nav img{height:24px;filter:invert(1)}
.shop{background:#00a3e0;color:#07131f;padding:8px 14px;font-weight:700;text-decoration:none}
.shop:hover{background:#4cc4f0;color:#07131f}
.hero{padding:56px 6vw}
.hero h1{font-size:clamp(32px,5vw,56px);color:#fff}
.bento{display:grid;grid-template-columns:2fr 1fr 1fr;grid-template-rows:auto auto;gap:12px;padding:0 6vw 48px}
.bento article{background:#0d2236;padding:18px;border-radius:4px}
.bento .wide{grid-column:1/2;grid-row:1/3}
.bento img{width:100%;height:100%;object-fit:cover;max-height:360px}
section{padding:48px 6vw}
h2{color:#fff}
.feats{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}
.feats article{background:#0d2236;padding:14px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
pre{background:#031018;border:1px solid #143049;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace;color:#9ad8f0}
.os img{height:42px;background:#fff;padding:3px;margin-right:8px}
@media(max-width:800px){.bento,.feats,.models{grid-template-columns:1fr}.bento .wide{grid-column:auto}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article style='background:#0d2236;padding:14px'><h3>{m['name']}</h3><img src='{m['dim']}' alt='' style='background:#fff;margin:8px 0'><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <p style="color:#00a3e0;letter-spacing:.16em;text-transform:uppercase;font-size:12px">The layer under the motion</p>
  <h1>Gearotons is the critical layer powering the joint.</h1>
  <p style="max-width:40em;margin-top:12px">{TAGLINE}. {INTRO}</p>
</header>
<div class="bento">
  <article class="wide"><img src="../transparent/one_motor_transparent_small.png" alt=""></article>
  <article><h3>HBM of the bench</h3><p>32 kHz loop, on-package encoder.</p></article>
  <article><h3>Gen-anything bus</h3><p>RS-485. Daisy-chain any number.</p></article>
  <article><h3>Edge motion</h3><p>Pi, Arduino, ESP32, Mac, PC.</p></article>
  <article><h3>Open docs</h3><p>Written for you — and for your AI.</p></article>
</div>
<section id="features">
  <h2>Design tools, in the motor</h2>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>Datasheet</h2>
  <div class="models">{models}</div>
  {spec_table('color:#00a3e0;border-bottom:1px solid #143049', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Bring-up listing</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <h2>Markets</h2>
  <div class="models">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
  </div>
</section>
<section id="about">
  <h2>About</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin:12px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <a href="{DOCS}" style="margin-left:12px">Documentation</a>
  <a href="{GITHUB}" style="margin-left:12px">GitHub</a>
</section>
"""
    w("v23.html", shell(TITLE, css, body))


def v24():
    """Berkshire Hathaway — 1996 Times, default links, HR rules."""
    # Almost no CSS. This is the joke and the craft.
    extra = '<style>body{margin:16px;font:16px/1.45 Times,"Times New Roman",serif;background:#fff;color:#000}img{max-width:100%;height:auto}table{border-collapse:collapse}td,th{border:1px solid #000;padding:4px 8px;text-align:left}pre{background:#f4f4f4;padding:12px;overflow:auto}</style>'
    feats = "".join(f"<li><b>{t}</b> — {d}</li>" for t, d in FEATURES[:10])
    models = "".join(
        f"<p><b>{m['name']}</b><br><img src='{m['dim']}' alt='{m['name']}' width='280'><br>{m['torque']}, {m['power']}, {m['height']}, {m['weight']}</p>"
        for m in MODELS
    )
    body = f"""
<p align="center">
  <img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons" width="280">
</p>
<p align="center"><b>GEAROTONS</b><br>
Shenzhen<br>
Official Home Page</p>
<hr>
<p align="center"><b>M17 SERIES SERVOMOTORS</b><br>
{TAGLINE}</p>
<hr>
<ul>
  <li><a href="#about">A Message about the Company</a></li>
  <li><a href="#features">Principal Characteristics of the M17</a></li>
  <li><a href="#models">The Three Models, with Specifications</a></li>
  <li><a href="#start">Instructions for the First Movement (Python)</a></li>
  <li><a href="#apps">Applications</a></li>
  <li><a href="{SHOP}">Shop — the Store</a></li>
  <li><a href="{DOCS}">Documentation</a></li>
  <li><a href="{GITHUB}">Source Code and Schematics (GitHub)</a></li>
</ul>
<hr>
<p id="top">{INTRO}</p>
<p>{INTRO2}</p>
<p align="center"><img src="../transparent/one_motor_transparent_small.png" alt="M17" width="360"></p>
<hr>
<p id="features"><b>Principal Characteristics</b></p>
<ul>{feats}</ul>
<hr>
<p id="models"><b>The Three Models</b></p>
{models}
<table>
  <tr><th>Parameter</th><th>M17-60</th><th>M17-48</th><th>M17-40</th></tr>
  {spec_rows()}
</table>
<hr>
<p id="start"><b>A Simple Program</b></p>
<pre>{esc(CODE)}</pre>
<p>{INSTALL}<br>{ARDUINO}</p>
<hr>
<p id="apps"><b>Applications</b></p>
<p>Robotics. CNC machines. Automated testing equipment. Scientific instruments. Automation. 3D printers. Education.</p>
<p>
  <img src="../robotics_small.jpg" alt="Robotics" width="220">
  <img src="../automation_small.jpg" alt="Automation" width="220">
  <img src="../test_rack_small.jpg" alt="Test rack" width="220">
</p>
<hr>
<p id="about"><b>About the Company</b></p>
<p>{COMPANY}</p>
<p>
  <img src="../Open-source-hardware-logo.svg.png" alt="OSH" height="48">
  <img src="../Open_Source_Initiative.svg.png" alt="OSI" height="48">
</p>
<hr>
<p align="center">
  <a href="{SHOP}"><b>SHOP NOW</b></a><br>
  If you have any comments about our WEB page, you may write the company. Due to the limited number of personnel, we may be unable to provide a direct response.
</p>
<p align="center">Copyright © 2022-2026 <b>Gearotons</b></p>
"""
    w("v24.html", f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{TITLE}</title>
{extra}
</head>
<body>
{body}
</body>
</html>
""")


def v25():
    """Eli Lilly — healthcare calm, red CTA only."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:18px/1.65 "Gill Sans","Gill Sans MT",Calibri,Helvetica,sans-serif;background:#fff;color:#2b2b2b}
img{max-width:100%;height:auto;display:block}
a{color:#2b2b2b}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:16px 32px;background:#fff;border-bottom:1px solid #eee}
nav img{height:28px}
.shop{background:#d52b1e;color:#fff;padding:10px 20px;text-decoration:none;border-radius:2px;font-size:14px}
.shop:hover{background:#b42318;color:#fff}
.hero{padding:72px 8vw;max-width:980px}
.hero h1{font-size:clamp(34px,5vw,56px);font-weight:400;line-height:1.15}
section{padding:56px 8vw;max-width:980px}
h2{font-weight:400;font-size:32px;margin-bottom:16px}
.feats li{list-style:none;padding:16px 0;border-top:1px solid #eee}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:20px}
pre{background:#f6f3ef;padding:20px;overflow:auto;font:14px/1.55 ui-monospace,Menlo,monospace}
.apps img{width:100%;margin:16px 0;max-height:320px;object-fit:cover}
.os img{height:44px;margin-right:12px}
footer{padding:48px 8vw 72px;border-top:1px solid #eee}
@media(max-width:800px){.models{grid-template-columns:1fr}}
"""
    feats = "".join(f"<li><strong>{t}.</strong> {d}</li>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} · {m['weight']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <h1>Precision motion, offered with care.</h1>
  <p style="margin-top:16px;max-width:38em">{TAGLINE}. {INTRO}</p>
</header>
<section>
  <img src="../transparent/one_motor_transparent_small.png" alt="M17">
</section>
<section id="features">
  <h2>What this motor is for</h2>
  <p>{INTRO2}</p>
  <ul class="feats">{feats}</ul>
</section>
<section id="models">
  <h2>Three carefully sized options</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #eee', 'margin-top:24px')}
</section>
<section id="start">
  <h2>A first session</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:10px;color:#666">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <h2>In practice</h2>
  <img class="apps" src="../automation_small.jpg" alt="">
  <p>Robotics, CNC, automated testing, scientific instruments, education.</p>
</section>
<section id="about">
  <h2>The company</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin-top:16px">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
</section>
<footer>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <p style="margin-top:16px"><a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v25.html", shell(TITLE, css, body))


def v26():
    """SK hynix — white, SK orange, product chips."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.55 "Avenir Next",Avenir,Helvetica,sans-serif;background:#fff;color:#111}
img{max-width:100%;height:auto;display:block}
a{color:#111;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#fff;border-bottom:1px solid #eee}
nav img{height:26px}
.shop{background:#ea002c;color:#fff;padding:8px 16px;font-weight:700}
.shop:hover{background:#b80022;color:#fff}
.hero{padding:64px 6vw;background:#111;color:#fff;display:grid;grid-template-columns:1fr 1fr;gap:32px;align-items:center}
.hero h1{font-size:clamp(32px,5vw,52px)}
.chips{display:flex;flex-wrap:wrap;gap:8px;padding:24px 6vw}
.chips span{border:1px solid #111;padding:6px 12px;font-size:13px}
section{padding:48px 6vw}
h2{font-size:28px}
.feats{display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
pre{background:#111;color:#fff;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.apps img{height:170px;width:100%;object-fit:cover}
.os img{height:42px;margin-right:8px}
@media(max-width:800px){.hero,.feats,.models,.apps{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3 style='color:#ea002c'>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article style='border-top:3px solid #ea002c;padding-top:12px'><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <p style="color:#ea002c;letter-spacing:.16em;text-transform:uppercase;font-size:12px">Technology innovator for a better bench</p>
    <h1>Growing with every axis you add.</h1>
    <p style="margin-top:12px">{TAGLINE}</p>
  </div>
  <img src="../transparent/M17_series_overview_transparent_small.png" alt="">
</header>
<div class="chips">
  <span>Motor</span><span>Driver</span><span>Controller</span><span>Encoder</span>
  <span>RS-485</span><span>32 kHz</span><span>NEMA 17</span><span>12–24 V</span>
</div>
<section id="features">
  <h2>Products &amp; solutions</h2>
  <p>{INTRO}</p>
  <div class="feats" style="margin-top:20px">{feats}</div>
</section>
<section id="models">
  <h2>Density options</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #eee;color:#ea002c', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Host</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <h2>Applications</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
  </div>
</section>
<section id="about">
  <h2>Sustainability of knowledge</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin:12px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <a href="{DOCS}" style="margin-left:12px">Documentation</a>
  <a href="{GITHUB}" style="margin-left:12px">GitHub</a>
</section>
"""
    w("v26.html", shell(TITLE, css, body))


def v27():
    """JPMorgan — navy, gold, cinematic bands, serif."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.6 Georgia,"Times New Roman",serif;background:#fff;color:#0b1f3a}
img{max-width:100%;height:auto;display:block}
a{color:#0b1f3a}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#0b1f3a}
nav img{height:24px;filter:invert(1)}
.shop{color:#c4a35a;border:1px solid #c4a35a;padding:8px 14px;text-decoration:none;font-family:Helvetica,Arial,sans-serif;font-size:12px;letter-spacing:.1em;text-transform:uppercase}
.shop:hover{background:#c4a35a;color:#0b1f3a}
.band{min-height:56vh;display:flex;align-items:flex-end;padding:40px 6vw;background-size:cover;background-position:center}
.band h1,.band h2{color:#fff;font-weight:400;font-size:clamp(32px,5vw,52px);max-width:16ch}
section{padding:56px 6vw;max-width:1100px;margin:0 auto}
.gold{color:#c4a35a;font-family:Helvetica,Arial,sans-serif;font-size:12px;letter-spacing:.16em;text-transform:uppercase}
.two{display:grid;grid-template-columns:1fr 1fr;gap:32px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
pre{background:#0b1f3a;color:#e8e0d0;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.os img{height:44px;margin-right:8px}
footer{background:#0b1f3a;color:#e8e0d0;padding:40px 6vw}
footer a{color:#c4a35a}
@media(max-width:800px){.two,.models{grid-template-columns:1fr}}
"""
    feats = "".join(f"<p><b>{t}.</b> {d}</p>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="band" id="top" style="background-image:linear-gradient(to top,#0b1f3a,transparent 55%),url('../automation_small.jpg')">
  <div>
    <p class="gold">Committed to service, precision and growth</p>
    <h1>Motion that strengthens the work.</h1>
  </div>
</header>
<section>
  <p class="gold">The firm</p>
  <h2 style="font-weight:400;font-size:32px;margin:8px 0">{TAGLINE}</h2>
  <p>{INTRO}</p>
</section>
<div class="band" style="background-image:linear-gradient(to top,#0b1f3a,transparent 50%),url('../robotics_small.jpg')">
  <h2>Making opportunities move.</h2>
</div>
<section id="features" class="two">
  <div>
    <p class="gold">Principles</p>
    {feats}
  </div>
  <img src="../transparent/one_motor_transparent_small.png" alt="">
</section>
<section id="models">
  <p class="gold">Holdings</p>
  <h2 style="font-weight:400;margin-bottom:16px">Three models</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #0b1f3a', 'margin-top:16px')}
</section>
<section id="start">
  <p class="gold">Operations</p>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section id="about">
  <p class="gold">A letter</p>
  <p>{COMPANY}</p>
  <p class="os" style="margin:12px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
</section>
<footer>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <p style="margin-top:16px"><a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v27.html", shell(TITLE, css, body))


def v28():
    """AMD — black, red accent, performance lockup."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.5 "Avenir Next",Avenir,Helvetica,sans-serif;background:#000;color:#eee}
img{max-width:100%;height:auto;display:block}
a{color:#fff;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#000;border-bottom:1px solid #222}
nav img{height:24px;filter:invert(1)}
.shop{background:#ed1c24;color:#fff;padding:8px 16px;font-weight:700}
.shop:hover{background:#fff;color:#ed1c24}
.hero{padding:64px 6vw;display:grid;grid-template-columns:1fr 1fr;gap:32px;align-items:center}
h1{font-size:clamp(40px,7vw,80px);font-weight:800;letter-spacing:-.04em;line-height:.9}
h1 i{font-style:italic;color:#ed1c24}
.lock{display:flex;gap:32px;margin:24px 0;font-size:28px;font-weight:700}
.lock span{display:block;font-size:12px;font-weight:500;color:#888;letter-spacing:.12em;text-transform:uppercase}
section{padding:56px 6vw}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:0}
.feats article{border-top:1px solid #222;padding:16px 16px 16px 0}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
pre{background:#111;border-left:4px solid #ed1c24;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.apps img{height:180px;width:100%;object-fit:cover}
.os img{height:42px;background:#fff;padding:3px;margin-right:8px}
@media(max-width:800px){.hero,.feats,.models,.apps,.lock{grid-template-columns:1fr;flex-direction:column}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p style='color:#aaa'>{d}</p></article>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article style='border:1px solid #222;padding:14px'><h3>{m['name']}</h3><img src='{m['dim']}' alt='' style='background:#fff;margin:8px 0'><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <h1>M17<br><i>together we advance</i> motion.</h1>
    <p style="margin-top:12px;color:#aaa">{TAGLINE}</p>
    <div class="lock">
      <div>560<span>RPM</span></div>
      <div>32 kHz<span>PID</span></div>
      <div>1.1 A<span>max</span></div>
    </div>
    <a class="shop" href="{SHOP}">Shop Now</a>
  </div>
  <img src="../transparent/one_motor_transparent_small.png" alt="M17">
</header>
<section id="features">
  <h2>High performance. On package.</h2>
  <p style="color:#aaa;max-width:50em">{INTRO} {INTRO2}</p>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>Choose your stack</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #333', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Developer path</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px;color:#888">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
  </div>
</section>
<section id="about">
  <h2>Open source</h2>
  <p style="color:#aaa">{COMPANY}</p>
  <p class="os" style="margin:12px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <a href="{DOCS}" style="margin-left:12px">Documentation</a>
  <a href="{GITHUB}" style="margin-left:12px">GitHub</a>
</section>
"""
    w("v28.html", shell(TITLE, css, body))


def v29():
    """Walmart — spark yellow, blue header, retail aisle."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.5 Helvetica,Arial,sans-serif;background:#fff;color:#2e2f32}
img{max-width:100%;height:auto;display:block}
a{color:#0071dc}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 20px;background:#0071dc}
nav img{height:28px;background:#fff;border-radius:50%;padding:3px}
.shop{background:#ffc220;color:#2e2f32;padding:10px 18px;text-decoration:none;font-weight:800;border-radius:999px}
.shop:hover{background:#ffd84d;color:#2e2f32}
.hero{padding:36px 20px;background:#e6f1fc;display:grid;grid-template-columns:1fr 1fr;gap:24px;align-items:center}
.hero h1{font-size:clamp(30px,5vw,48px);color:#0071dc}
.aisle{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;padding:24px 20px}
.aisle article{border:1px solid #e6e6e6;border-radius:12px;padding:12px;background:#fff}
.aisle h3{color:#0071dc}
section{padding:28px 20px}
.news{display:grid;grid-template-columns:1fr 1fr;gap:16px}
.news article{background:#f7f8f8;border-radius:12px;overflow:hidden}
.news img{height:160px;width:100%;object-fit:cover}
.news div{padding:14px}
pre{background:#2e2f32;color:#fff;padding:16px;overflow:auto;border-radius:8px;font:13px/1.5 ui-monospace,Menlo,monospace}
.os img{height:42px;margin-right:8px}
footer{background:#0071dc;color:#fff;padding:32px 20px}
footer a{color:#ffc220}
@media(max-width:800px){.hero,.aisle,.news{grid-template-columns:1fr}}
"""
    models = "".join(
        f"<article><img src='{m['dim']}' alt=''><h3>{m['name']}</h3><p>Pick your size · {m['torque']}</p><a class='shop' href='{SHOP}'>Shop</a></article>"
        for m in MODELS
    )
    news = "".join(
        f"<article><div><h3>{t}</h3><p>{d}</p></div></article>" for t, d in FEATURES[:6]
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <h1>Unmatched simplicity. Everyday motion.</h1>
    <p style="margin:12px 0">{TAGLINE}. Motor, driver, controller, encoder — one box you can actually finish a project with.</p>
    <a class="shop" href="{SHOP}">Shop Now</a>
  </div>
  <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="Kit">
</header>
<section id="models">
  <h2>Pick your size</h2>
  <div class="aisle">{models}</div>
  {spec_table('background:#e6f1fc', 'padding:0 20px')}
</section>
<section id="features">
  <h2>Why it belongs on the shelf</h2>
  <p>{INTRO} {INTRO2}</p>
  <div class="news" style="margin-top:16px">{news}</div>
</section>
<section id="start">
  <h2>Setup is a script</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <h2>Great for</h2>
  <div class="aisle">
    <article><img src="../robotics_small.jpg" alt=""><h3>Robotics</h3></article>
    <article><img src="../automation_small.jpg" alt=""><h3>Home shop CNC</h3></article>
    <article><img src="../test_rack_small.jpg" alt=""><h3>School labs</h3></article>
  </div>
</section>
<section id="about">
  <h2>About Gearotons</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin:12px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
  <a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a>
</section>
<footer>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <p style="margin-top:12px">Gearotons · Shenzhen · 2022</p>
</footer>
"""
    w("v29.html", shell(TITLE, css, body))


def v30():
    """ASML — Dutch precision, lithography-scale, quiet."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.6 "Avenir Next",Avenir,Helvetica,sans-serif;background:#fff;color:#12141a}
img{max-width:100%;height:auto;display:block}
a{color:#12141a}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:16px 32px;background:#fff}
nav img{height:26px}
.shop{text-decoration:none;border-bottom:1px solid #12141a;font-size:14px}
.shop:hover{color:#0055a5;border-color:#0055a5}
.hero{position:relative;min-height:78vh;display:flex;align-items:flex-end;padding:48px 6vw;background:#0b0e14;color:#fff}
.hero img{position:absolute;inset:0;width:100%;height:100%;object-fit:cover;opacity:.45}
.hero .t{position:relative;max-width:18em}
.hero h1{font-size:clamp(32px,5vw,56px);font-weight:500}
section{padding:64px 6vw;max-width:1100px;margin:0 auto}
h2{font-weight:500;font-size:32px}
.glance{display:grid;grid-template-columns:1fr 1fr;gap:40px;margin-top:24px}
.press{list-style:none}
.press li{padding:14px 0;border-top:1px solid #e6e6e6}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
pre{background:#12141a;color:#e8eaef;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.os img{height:44px;margin-right:8px}
@media(max-width:800px){.glance,.models{grid-template-columns:1fr}}
"""
    press = "".join(f"<li><strong>{t}</strong> — {d}</li>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} · {m['height']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <img src="../automation_small.jpg" alt="">
  <div class="t">
    <h1>Changing the bench, one degree at a time.</h1>
    <p style="margin-top:12px">{TAGLINE}</p>
  </div>
</header>
<section>
  <h2>We're probably already in the machine you're building.</h2>
  <p style="max-width:42em;margin-top:12px">{INTRO}</p>
</section>
<section class="glance" id="features">
  <div>
    <h2>At a glance</h2>
    <p style="margin-top:12px">{INTRO2}</p>
  </div>
  <ul class="press">{press}</ul>
</section>
<section id="models">
  <h2>The machines behind the motion</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #e6e6e6', 'margin-top:20px')}
</section>
<section id="start">
  <h2>Our technology, in a listing</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px;color:#555">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <img src="../connection_diagram.jpg" alt="Connection diagram">
</section>
<section id="about">
  <h2>Who we are</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin:16px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
  <a class="shop" href="{SHOP}">Shop Now</a> &nbsp;
  <a class="shop" href="{DOCS}">Documentation</a> &nbsp;
  <a class="shop" href="{GITHUB}">GitHub</a>
</section>
"""
    w("v30.html", shell(TITLE, css, body))


if __name__ == "__main__":
    v21(); v22(); v23(); v24(); v25()
    v26(); v27(); v28(); v29(); v30()
