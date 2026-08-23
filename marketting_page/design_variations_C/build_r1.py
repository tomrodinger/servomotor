#!/usr/bin/env python3
"""Round 1 — ten original directions. Unique architecture per file."""
from pathlib import Path
from _content import (
    TITLE, TAGLINE, SHOP, DOCS, GITHUB, CODE, INSTALL, ARDUINO,
    MODELS, FEATURES, INTRO, INTRO2, COMPANY, spec_rows, esc, shell,
)

OUT = Path(__file__).parent


def w(name, html):
    (OUT / name).write_text(html, encoding="utf-8")
    print("wrote", name, "bytes", len(html))


# ---------------------------------------------------------------------------
# v1 Swiss Catalog
# ---------------------------------------------------------------------------
def v1():
    css = """
:root{--g:#7AB648;--ink:#111;--rule:#111;--muted:#555;--bg:#fff}
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.5 Helvetica,Arial,"Helvetica Neue",sans-serif;color:var(--ink);background:var(--bg)}
img{max-width:100%;height:auto;display:block}
a{color:var(--ink)}
.wrap{max-width:1120px;margin:0 auto;padding:0 24px}
nav{position:sticky;top:0;z-index:20;background:#fff;border-bottom:1px solid #111}
nav .wrap{display:flex;align-items:center;justify-content:space-between;height:56px}
nav img{height:28px;width:auto}
nav ul{display:flex;gap:22px;list-style:none;font-size:12px;letter-spacing:.12em;text-transform:uppercase}
nav a{text-decoration:none}
.shop{border:1px solid #111;padding:8px 14px;text-decoration:none;font-size:12px;letter-spacing:.12em;text-transform:uppercase}
.shop:hover,.btn:hover{background:var(--g);border-color:var(--g);color:#fff}
.hero{padding:72px 0 48px;border-bottom:1px solid #111}
.kicker{font-size:11px;letter-spacing:.22em;text-transform:uppercase;margin-bottom:18px}
.hero h1{font-size:clamp(40px,8vw,88px);line-height:.9;font-weight:700;letter-spacing:-.04em;max-width:14ch}
.hero .sub{margin-top:22px;max-width:46ch;color:var(--muted);font-size:17px}
.hero-grid{display:grid;grid-template-columns:1.1fr .9fr;gap:48px;align-items:end;margin-top:48px}
.meta{display:grid;grid-template-columns:1fr 1fr;gap:18px 32px;font-size:13px;border-top:1px solid #111;padding-top:18px}
.meta b{display:block;font-size:11px;letter-spacing:.16em;text-transform:uppercase;font-weight:500}
section{padding:72px 0;border-bottom:1px solid #111}
.secno{font-size:11px;letter-spacing:.2em;text-transform:uppercase;margin-bottom:10px}
h2{font-size:clamp(28px,4vw,40px);letter-spacing:-.03em;font-weight:700}
.lead{max-width:62ch;margin:16px 0 36px;color:var(--muted)}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:0}
.feats li{list-style:none;border-top:1px solid #111;padding:18px 20px 18px 0;display:grid;grid-template-columns:48px 1fr;gap:16px}
.feats em{font-style:normal;font-size:13px;letter-spacing:.08em}
.feats strong{display:block;margin-bottom:4px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:24px}
.models article{border-top:2px solid #111;padding-top:16px}
.models h3{font-size:22px;margin:8px 0}
.models dl{display:grid;grid-template-columns:1fr 1fr;gap:8px 12px;font-size:13px;margin:16px 0}
.models dt{color:var(--muted)}
pre{background:#111;color:#f2f2f0;padding:22px;overflow:auto;font:13px/1.55 Menlo,Consolas,monospace}
.note{font-size:13px;color:var(--muted);margin-top:10px}
.apps{display:grid;grid-template-columns:2fr 1fr 1fr;gap:8px}
.apps figure{position:relative;overflow:hidden;background:#f4f4f4}
.apps img{width:100%;height:220px;object-fit:cover}
.apps figcaption{font-size:11px;letter-spacing:.14em;text-transform:uppercase;padding:8px 0}
.company{display:grid;grid-template-columns:1fr 1fr;gap:40px;align-items:center}
.os{display:flex;gap:28px;align-items:center;margin-top:24px}
.os img{height:52px;width:auto}
footer{padding:48px 0 64px}
.cta-row{display:flex;gap:12px;flex-wrap:wrap;margin:24px 0}
.btn{display:inline-block;border:1px solid #111;padding:12px 20px;text-decoration:none;font-size:13px;letter-spacing:.12em;text-transform:uppercase}
@media(max-width:800px){
  nav ul{display:none}
  .hero-grid,.feats,.models,.company,.apps{grid-template-columns:1fr}
  .apps img{height:180px}
}
"""
    feats = "".join(
        f"<li><em>{i:02d}</em><div><strong>{t}</strong><span>{d}</span></div></li>"
        for i, (t, d) in enumerate(FEATURES[:8], 1)
    )
    models = "".join(
        f"""<article>
          <p class="secno">Model</p>
          <h3>{m['name']}</h3>
          <img src="{m['dim']}" alt="{m['name']} dimensions">
          <dl>
            <dt>Torque</dt><dd>{m['torque']}</dd>
            <dt>Power</dt><dd>{m['power']}</dd>
            <dt>Height</dt><dd>{m['height']}</dd>
            <dt>Weight</dt><dd>{m['weight']}</dd>
          </dl>
        </article>"""
        for m in MODELS
    )
    body = f"""
<nav><div class="wrap">
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <ul>
    <li><a href="#features">Features</a></li>
    <li><a href="#models">Models</a></li>
    <li><a href="#start">Start</a></li>
    <li><a href="#apps">Use</a></li>
  </ul>
  <a class="shop" href="{SHOP}">Shop</a>
</div></nav>
<header class="hero" id="top"><div class="wrap">
  <p class="kicker">Gearotons · M17 Series</p>
  <h1>All-in-one motion, catalogued.</h1>
  <p class="sub">{TAGLINE}. Motor, driver, controller, and encoder — one package, one RS-485 bus.</p>
  <div class="hero-grid">
    <img src="../transparent/M17_series_overview_transparent_small.png" alt="M17-60, M17-48, and M17-40">
    <div>
      <p>{INTRO}</p>
      <div class="meta">
        <div><b>Loop</b>32 kHz PID</div>
        <div><b>Speed</b>560 RPM</div>
        <div><b>Voltage</b>12–24 V</div>
        <div><b>Mount</b>NEMA 17</div>
      </div>
    </div>
  </div>
</div></header>
<section id="features"><div class="wrap">
  <p class="secno">01 — Capabilities</p>
  <h2>What is integrated.</h2>
  <p class="lead">{INTRO2}</p>
  <ul class="feats">{feats}</ul>
</div></section>
<section id="models"><div class="wrap">
  <p class="secno">02 — Three bodies</p>
  <h2>Same control. Three torques.</h2>
  <p class="lead">Pick the stack height that matches the load. Footprint stays 42.2 × 42.2 mm.</p>
  <div class="models">{models}</div>
  <div style="overflow:auto;margin-top:40px">
    <table style="width:100%;border-collapse:collapse;font-size:14px">
      <thead><tr><th style="text-align:left;border-bottom:1px solid #111;padding:8px 8px 8px 0">Parameter</th>
      <th style="text-align:left;border-bottom:1px solid #111;padding:8px">M17-60</th>
      <th style="text-align:left;border-bottom:1px solid #111;padding:8px">M17-48</th>
      <th style="text-align:left;border-bottom:1px solid #111;padding:8px">M17-40</th></tr></thead>
      <tbody>{spec_rows()}</tbody>
    </table>
  </div>
</div></section>
<section id="start"><div class="wrap">
  <p class="secno">03 — First move</p>
  <h2>One revolution, two seconds.</h2>
  <p class="lead">Python on any desktop. The same ideas exist in the Arduino library.</p>
  <pre><code>{esc(CODE)}</code></pre>
  <p class="note">{INSTALL} · {ARDUINO}</p>
</div></section>
<section id="apps"><div class="wrap">
  <p class="secno">04 — Applications</p>
  <h2>From the teaching bench to the test rack.</h2>
  <div class="apps">
    <figure><img src="../robotics_small.jpg" alt="Robotics assembly"><figcaption>Robotics · CNC</figcaption></figure>
    <figure><img src="../automation_small.jpg" alt="Automation"><figcaption>Automation</figcaption></figure>
    <figure><img src="../test_rack_small.jpg" alt="Test rack"><figcaption>Testing</figcaption></figure>
  </div>
</div></section>
<section id="about"><div class="wrap company">
  <div>
    <p class="secno">05 — Open source</p>
    <h2>Firmware, libraries, schematics.</h2>
    <p class="lead">{COMPANY}</p>
    <div class="cta-row">
      <a class="btn" href="{GITHUB}">GitHub</a>
      <a class="btn" href="{DOCS}">Documentation</a>
    </div>
    <div class="os">
      <img src="../Open-source-hardware-logo.svg.png" alt="Open Source Hardware">
      <img src="../Open_Source_Initiative.svg.png" alt="Open Source Initiative">
    </div>
  </div>
  <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="Full kit with three motors">
</div></section>
<footer><div class="wrap">
  <p class="kicker">Gearotons</p>
  <h2>Put an M17 on the bus.</h2>
  <div class="cta-row">
    <a class="btn" href="{SHOP}">Shop Now</a>
    <a class="btn" href="{DOCS}">Documentation</a>
  </div>
  <p class="note">Founded in Shenzhen, 2022. {TAGLINE}.</p>
</div></footer>
"""
    w("v1.html", shell(TITLE, css, body))


# ---------------------------------------------------------------------------
# v2 Dark Atelier
# ---------------------------------------------------------------------------
def v2():
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:17px/1.55 "Avenir Next","Avenir",Gill Sans,Helvetica,sans-serif;background:#0b0b0c;color:#eceae6}
img{max-width:100%;height:auto;display:block}
a{color:#eceae6;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:18px 32px;background:rgba(11,11,12,.86);backdrop-filter:blur(12px);border-bottom:1px solid #222}
nav img{height:26px;filter:invert(1);width:auto}
nav .links{display:flex;gap:28px;font-size:13px;letter-spacing:.08em}
.cta{border:1px solid #eceae6;padding:8px 16px;border-radius:999px;font-size:13px}
.cta:hover{background:#eceae6;color:#0b0b0c}
.hero{min-height:92vh;display:grid;place-items:center;padding:48px 24px 24px;position:relative;overflow:hidden}
.hero img{max-height:72vh;margin:0 auto;filter:drop-shadow(0 40px 80px rgba(0,0,0,.6))}
.hero h1{position:absolute;left:6vw;bottom:10vh;font-size:clamp(48px,10vw,120px);font-weight:500;letter-spacing:-.045em;line-height:.85}
.hero p{position:absolute;right:6vw;bottom:12vh;max-width:28ch;font-size:15px;color:#b7b4ad}
section{padding:96px 8vw}
h2{font-size:clamp(32px,5vw,56px);font-weight:500;letter-spacing:-.03em;margin-bottom:20px}
.muted{color:#9c9890;max-width:58ch}
.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:28px;margin-top:48px}
.grid4 article{border-top:1px solid #2a2a2c;padding-top:16px}
.grid4 h3{font-size:16px;font-weight:600;margin-bottom:8px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:2px;background:#1a1a1c;margin:48px -8vw 0;padding:2px}
.models article{background:#0b0b0c;padding:32px}
.models h3{font-size:28px;font-weight:500}
.models ul{list-style:none;margin-top:20px;font-size:14px;color:#b7b4ad}
.models li{display:flex;justify-content:space-between;border-bottom:1px solid #222;padding:8px 0}
pre{background:#141416;padding:28px;overflow:auto;font:13px/1.6 ui-monospace,Menlo,monospace;color:#d7d3c8;border-radius:12px;margin-top:28px}
.apps{display:grid;grid-template-columns:1.4fr 1fr;gap:16px}
.apps img{width:100%;height:100%;object-fit:cover;max-height:420px;filter:grayscale(.2) contrast(1.05)}
.split{display:grid;grid-template-columns:1fr 1fr;gap:48px;align-items:center}
.os{display:flex;gap:24px;margin-top:24px}
.os img{height:48px;width:auto;background:#fff;padding:6px;border-radius:8px}
footer{padding:80px 8vw 96px;border-top:1px solid #222}
.big{font-size:clamp(36px,6vw,72px);letter-spacing:-.04em}
@media(max-width:800px){
  nav .links{display:none}
  .hero h1{position:static;margin-top:24px}
  .hero p{position:static;margin-top:12px}
  .grid4,.models,.split,.apps{grid-template-columns:1fr}
  .models{margin:32px 0 0}
}
"""
    feats = "".join(
        f"<article><h3>{t}</h3><p class='muted'>{d}</p></article>"
        for t, d in FEATURES[:8]
    )
    models = "".join(
        f"""<article>
          <h3>{m['name']}</h3>
          <img src="{m['dim']}" alt="{m['name']}" style="background:#fff;margin:16px 0">
          <ul>
            <li><span>Torque</span><span>{m['torque']}</span></li>
            <li><span>Power</span><span>{m['power']}</span></li>
            <li><span>Height</span><span>{m['height']}</span></li>
            <li><span>Weight</span><span>{m['weight']}</span></li>
          </ul>
        </article>"""
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <div class="links">
    <a href="#features">Object</a>
    <a href="#models">Sizes</a>
    <a href="#start">Code</a>
    <a href="#about">Studio</a>
  </div>
  <a class="cta" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17 servomotor">
  <h1>M17</h1>
  <p>{TAGLINE}. A closed-loop instrument the size of a NEMA 17 stepper.</p>
</header>
<section id="features">
  <h2>Four things, one body.</h2>
  <p class="muted">{INTRO}</p>
  <div class="grid4">{feats}</div>
</section>
<section id="models" style="padding-top:0">
  <h2>Three heights.</h2>
  <p class="muted">Same 42.2 mm square. Same 12–24 V. Same 560 RPM. Torque is the only argument.</p>
  <div class="models">{models}</div>
  <div style="overflow:auto;margin-top:48px">
    <table style="width:100%;border-collapse:collapse;font-size:14px;color:#cfcbc2">
      <thead><tr><th style="text-align:left;padding:10px 8px;border-bottom:1px solid #333">Parameter</th>
      <th style="text-align:left;padding:10px 8px;border-bottom:1px solid #333">M17-60</th>
      <th style="text-align:left;padding:10px 8px;border-bottom:1px solid #333">M17-48</th>
      <th style="text-align:left;padding:10px 8px;border-bottom:1px solid #333">M17-40</th></tr></thead>
      <tbody>{spec_rows()}</tbody>
    </table>
  </div>
</section>
<section id="start">
  <h2>Speak in degrees, not pulses.</h2>
  <p class="muted">No STEP/DIR timing. A trapezoid move is one call.</p>
  <pre><code>{esc(CODE)}</code></pre>
  <p class="muted" style="margin-top:12px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="In a machine">
    <div>
      <h2>Where it lives.</h2>
      <p class="muted">Robotics, CNC, automated testing, scientific instruments, 3D printers, classrooms.</p>
      <img src="../automation_small.jpg" alt="Automation" style="margin-top:20px;max-height:200px;object-fit:cover;width:100%">
    </div>
  </div>
</section>
<section id="about">
  <div class="split">
    <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="Kit">
    <div>
      <h2>Open by default.</h2>
      <p class="muted">{COMPANY}</p>
      <p style="margin-top:20px"><a class="cta" href="{GITHUB}">GitHub</a> &nbsp; <a class="cta" href="{DOCS}">Docs</a></p>
      <div class="os">
        <img src="../Open-source-hardware-logo.svg.png" alt="">
        <img src="../Open_Source_Initiative.svg.png" alt="">
      </div>
    </div>
  </div>
</section>
<footer>
  <p class="big">Bring one home.</p>
  <p style="margin:24px 0"><a class="cta" href="{SHOP}">Shop Now</a></p>
  <p class="muted">Gearotons · Shenzhen · 2022</p>
</footer>
"""
    w("v2.html", shell(TITLE, css, body))


# ---------------------------------------------------------------------------
# v3 Field Notes (editorial)
# ---------------------------------------------------------------------------
def v3():
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:18px/1.6 Georgia,"Iowan Old Style",Palatino,"Times New Roman",serif;background:#f6f1e8;color:#1c1914}
img{max-width:100%;height:auto;display:block}
a{color:#1c1914}
nav{position:sticky;top:0;z-index:20;background:#f6f1e8;border-bottom:1px solid #d8d0c2;display:flex;justify-content:space-between;align-items:center;padding:14px 5vw}
nav img{height:34px;width:auto}
nav a.word{font-family:Georgia,serif;font-style:italic;text-decoration:none}
.shop{font-family:Helvetica,Arial,sans-serif;font-size:12px;letter-spacing:.14em;text-transform:uppercase;text-decoration:none;border-bottom:1px solid #1c1914;padding-bottom:2px}
.shop:hover{color:#6a8f3a;border-color:#6a8f3a}
.mast{padding:56px 5vw 24px;text-align:center}
.mast .issue{font-family:Helvetica,Arial,sans-serif;font-size:11px;letter-spacing:.28em;text-transform:uppercase}
.mast h1{font-size:clamp(42px,8vw,84px);font-weight:400;letter-spacing:-.03em;margin:12px 0;line-height:.95}
.mast .deck{font-style:italic;font-size:22px;max-width:28ch;margin:0 auto;color:#4a453c}
.spread{display:grid;grid-template-columns:1.2fr .8fr;gap:0;margin:36px 0 0;border-top:1px solid #1c1914;border-bottom:1px solid #1c1914}
.spread img{width:100%;height:100%;object-fit:cover;max-height:560px}
.spread aside{padding:36px 32px;display:flex;flex-direction:column;justify-content:center}
.drop{float:left;font-size:78px;line-height:.8;padding:6px 10px 0 0;font-weight:400}
.pull{font-size:28px;line-height:1.25;margin:40px 8vw;text-align:center;font-style:italic}
.columns{column-count:2;column-gap:36px;max-width:900px;margin:0 auto;padding:0 5vw 64px}
.columns h2{column-span:all;font-weight:400;font-size:32px;margin:48px 0 16px}
.feats{max-width:900px;margin:0 auto;padding:0 5vw 64px}
.feats ol{list-style:none}
.feats li{display:grid;grid-template-columns:2ch 1fr;gap:16px;padding:14px 0;border-top:1px solid #d8d0c2;font-size:16px}
.feats span{font-family:Helvetica,Arial,sans-serif;font-size:12px;letter-spacing:.08em}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:20px;padding:0 5vw 64px}
.models article{background:#fff;padding:16px;border:1px solid #d8d0c2}
.models h3{font-weight:400;font-size:26px;margin-bottom:8px}
pre{background:#1c1914;color:#f6f1e8;padding:24px;overflow:auto;font:13px/1.55 Menlo,monospace;margin:0 5vw 12px}
.note{font-family:Helvetica,Arial,sans-serif;font-size:12px;color:#5a554c;padding:0 5vw 64px}
.film{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;padding:0 5vw 64px}
.film img{height:220px;width:100%;object-fit:cover}
.film figcaption{font-family:Helvetica,Arial,sans-serif;font-size:11px;letter-spacing:.12em;text-transform:uppercase;padding:8px 0}
footer{padding:48px 5vw 72px;border-top:1px solid #1c1914;text-align:center}
.btn{display:inline-block;margin:8px;padding:12px 22px;border:1px solid #1c1914;text-decoration:none;font-family:Helvetica,Arial,sans-serif;font-size:12px;letter-spacing:.14em;text-transform:uppercase}
.btn:hover{background:#1c1914;color:#f6f1e8}
.os{display:flex;justify-content:center;gap:24px;margin:20px 0}
.os img{height:46px}
@media(max-width:800px){
  .spread,.models,.film,.columns{grid-template-columns:1fr;column-count:1}
}
"""
    feats = "".join(
        f"<li><span>{i:02d}</span><div><strong>{t}.</strong> {d}</div></li>"
        for i, (t, d) in enumerate(FEATURES[:8], 1)
    )
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} · {m['power']} · {m['weight']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <a class="word" href="#essay">The essay</a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="mast" id="top">
  <p class="issue">A field note on integrated motion</p>
  <h1>The motor that ate the cabinet.</h1>
  <p class="deck">{TAGLINE}</p>
</header>
<div class="spread">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17 on white">
  <aside>
    <p><span class="drop">T</span>{INTRO}</p>
  </aside>
</div>
<p class="pull">“Enable MOSFETs.” Then “Trapezoid move.” The shaft turns. No pulse train. No second box.</p>
<div class="columns" id="essay">
  <h2>On the bench</h2>
  <p>{INTRO2}</p>
  <p>The old stack was a stepper, a driver, a motion card, an encoder, and a rat’s nest. The M17 collapses that into the outline of a NEMA 17. The protocol is RS-485. The libraries are Python and Arduino. The manuals are written so a person — or an AI — can finish the first sketch before the coffee cools.</p>
</div>
<div class="feats" id="features">
  <ol>{feats}</ol>
</div>
<div class="models" id="models">{models}</div>
<div style="overflow:auto;padding:0 5vw 48px">
  <table style="width:100%;border-collapse:collapse;font-size:15px">
    <thead><tr><th style="text-align:left;border-bottom:1px solid #1c1914;padding:8px">Parameter</th>
    <th style="text-align:left;border-bottom:1px solid #1c1914;padding:8px">M17-60</th>
    <th style="text-align:left;border-bottom:1px solid #1c1914;padding:8px">M17-48</th>
    <th style="text-align:left;border-bottom:1px solid #1c1914;padding:8px">M17-40</th></tr></thead>
    <tbody>{spec_rows()}</tbody>
  </table>
</div>
<pre id="start"><code>{esc(CODE)}</code></pre>
<p class="note">{INSTALL} · {ARDUINO}</p>
<div class="film">
  <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
  <figure><img src="../automation_small.jpg" alt=""><figcaption>Automation</figcaption></figure>
  <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Test racks</figcaption></figure>
</div>
<footer id="about">
  <p style="max-width:52ch;margin:0 auto 20px">{COMPANY}</p>
  <div class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </div>
  <a class="btn" href="{SHOP}">Shop Now</a>
  <a class="btn" href="{DOCS}">Documentation</a>
  <a class="btn" href="{GITHUB}">GitHub</a>
</footer>
"""
    w("v3.html", shell(TITLE, css, body))


# ---------------------------------------------------------------------------
# v4 Bench Notes (workshop)
# ---------------------------------------------------------------------------
def v4():
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.55 "Avenir Next",Avenir,Nunito,Helvetica,sans-serif;background:#e7d7b8;color:#2a2116}
img{max-width:100%;height:auto;display:block}
a{color:#2a2116}
.paper{max-width:1080px;margin:0 auto;background:#f3e6c8;min-height:100vh;box-shadow:0 0 0 1px #c9b48a, 0 20px 60px rgba(80,50,10,.12)}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 28px;background:#f3e6c8;border-bottom:2px dashed #8a6a32}
nav img{height:40px}
.stamp{font:700 11px/1 ui-monospace,Menlo,monospace;letter-spacing:.14em;text-transform:uppercase;border:2px solid #8b1e1e;color:#8b1e1e;padding:6px 8px;transform:rotate(-3deg)}
.shop{background:#2a2116;color:#f3e6c8;padding:10px 16px;text-decoration:none;font-weight:700}
.shop:hover{background:#5a7d32}
.hero{padding:36px 28px;display:grid;grid-template-columns:1fr 1fr;gap:28px;align-items:center}
h1{font-size:clamp(32px,5vw,52px);line-height:1.05}
.tape{display:inline-block;background:#f7e36a;padding:2px 8px;transform:rotate(-1deg);font-family:ui-monospace,Menlo,monospace;font-size:12px}
section{padding:36px 28px;border-top:2px dashed #8a6a32}
h2{font-size:28px;margin-bottom:12px}
.cards{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.cards article{background:#fff8e8;border:1px solid #c9b48a;padding:14px;box-shadow:3px 3px 0 #c9b48a}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.models article{background:#fff;border:1px solid #2a2116;padding:12px}
pre{background:#2a2116;color:#f3e6c8;padding:18px;overflow:auto;font:12.5px/1.55 ui-monospace,Menlo,monospace}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.apps figure{background:#fff8e8;border:1px solid #c9b48a;padding:8px}
.apps img{height:160px;width:100%;object-fit:cover}
footer{padding:36px 28px 56px;background:#2a2116;color:#f3e6c8}
footer a{color:#f3e6c8}
.os{display:flex;gap:16px;margin:16px 0}
.os img{height:44px;background:#fff;padding:4px}
@media(max-width:800px){.hero,.cards,.models,.apps{grid-template-columns:1fr}}
"""
    feats = "".join(
        f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:8]
    )
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} · {m['height']} · {m['weight']}</p></article>"
        for m in MODELS
    )
    body = f"""
<div class="paper">
<nav>
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <span class="stamp">Bench copy</span>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <p class="tape">NEMA 17 · RS-485 · 12–24 V</p>
    <h1>Unbox it. Bus it. Move it.</h1>
    <p style="margin-top:12px">{TAGLINE}. The kit is three motors, an adapter, and the cables you actually need.</p>
  </div>
  <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="Kit flat-lay">
</header>
<section id="features">
  <h2>What’s in the tin</h2>
  <p>{INTRO}</p>
  <div class="cards" style="margin-top:18px">{feats}</div>
</section>
<section id="models">
  <h2>Pick a stack height</h2>
  <div class="models">{models}</div>
  <div style="overflow:auto;margin-top:18px;background:#fff8e8;padding:8px">
    <table style="width:100%;border-collapse:collapse;font-size:13px">
      <thead><tr><th style="text-align:left;padding:6px">Parameter</th><th style="text-align:left;padding:6px">M17-60</th>
      <th style="text-align:left;padding:6px">M17-48</th><th style="text-align:left;padding:6px">M17-40</th></tr></thead>
      <tbody>{spec_rows()}</tbody>
    </table>
  </div>
</section>
<section id="start">
  <h2>First sketch</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:14px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
  <img src="../transparent/adapter_and_wire_transparent_small.png" alt="RS-485 adapter" style="margin-top:16px;max-width:420px">
</section>
<section>
  <h2>Where people put them</h2>
  <div class="apps">
    <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robotics / CNC</figcaption></figure>
    <figure><img src="../automation_small.jpg" alt=""><figcaption>Automation</figcaption></figure>
    <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Test jigs</figcaption></figure>
  </div>
</section>
<footer id="about">
  <h2>Open hardware, open software</h2>
  <p>{COMPANY}</p>
  <div class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </div>
  <p><a class="shop" href="{SHOP}">Shop Now</a> &nbsp; <a href="{DOCS}">Docs</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
</div>
"""
    w("v4.html", shell(TITLE, css, body))


# ---------------------------------------------------------------------------
# v5 Instrument Lab
# ---------------------------------------------------------------------------
def v5():
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:15px/1.5 "Avenir Next",Avenir,Helvetica,sans-serif;background:#f4f7f4;color:#102016}
img{max-width:100%;height:auto;display:block}
a{color:#102016;text-decoration:none}
nav{position:sticky;top:0;z-index:20;background:#fff;border-bottom:1px solid #c9d6c9;display:flex;justify-content:space-between;align-items:center;padding:10px 28px}
nav img{height:28px}
nav ul{display:flex;gap:18px;list-style:none;font-size:12px;letter-spacing:.08em;text-transform:uppercase}
.shop{background:#7AB648;color:#fff;padding:8px 14px;font-weight:700;border-radius:2px}
.shop:hover{background:#5f9234}
.hero{padding:48px 28px 24px;max-width:1200px;margin:0 auto}
.hero h1{font-size:clamp(34px,5vw,56px);letter-spacing:-.03em}
.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:1px;background:#c9d6c9;margin:28px 0;border:1px solid #c9d6c9}
.metrics div{background:#fff;padding:18px}
.metrics b{display:block;font-size:28px;font-variant-numeric:tabular-nums;letter-spacing:-.03em}
.metrics span{color:#4a5c4a;font-size:12px;letter-spacing:.08em;text-transform:uppercase}
section{max-width:1200px;margin:0 auto;padding:40px 28px}
h2{font-size:28px;margin-bottom:12px}
.feats{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:12px}
.feats article{background:#fff;border:1px solid #c9d6c9;padding:14px;min-height:140px}
.feats h3{font-size:14px;margin-bottom:6px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.models article{background:#fff;border:1px solid #c9d6c9;padding:14px}
table{width:100%;border-collapse:collapse;background:#fff;font-variant-numeric:tabular-nums}
th,td{border:1px solid #c9d6c9;padding:8px 10px;text-align:left}
th{background:#e8f0e4;font-size:12px;letter-spacing:.06em;text-transform:uppercase}
pre{background:#102016;color:#d7f5c8;padding:20px;overflow:auto;font:12.5px/1.55 ui-monospace,Menlo,monospace}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.apps img{height:180px;width:100%;object-fit:cover}
.split{display:grid;grid-template-columns:1fr 1fr;gap:24px;align-items:center}
.os{display:flex;gap:16px;margin-top:16px}
.os img{height:48px}
footer{background:#102016;color:#e8f0e4;padding:48px 28px}
footer a{color:#e8f0e4}
footer .shop{color:#fff}
@media(max-width:900px){.metrics,.feats,.models,.apps,.split{grid-template-columns:1fr 1fr} nav ul{display:none}}
@media(max-width:560px){.metrics,.feats,.models,.apps,.split{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} rated · {m['power']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <ul>
    <li><a href="#features">Readouts</a></li>
    <li><a href="#models">Models</a></li>
    <li><a href="#start">Protocol</a></li>
  </ul>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <p style="letter-spacing:.18em;text-transform:uppercase;font-size:11px;color:#4a5c4a">Instrument series · M17</p>
  <h1>Closed-loop motion, specified.</h1>
  <p style="max-width:60ch;margin-top:10px">{TAGLINE}. {INTRO}</p>
  <div class="metrics">
    <div><b>32 kHz</b><span>PID loop</span></div>
    <div><b>560</b><span>RPM max</span></div>
    <div><b>12–24</b><span>Volts</span></div>
    <div><b>42.2</b><span>mm square</span></div>
  </div>
  <img src="../transparent/M17_series_overview_transparent_small.png" alt="Three M17 models">
</header>
<section id="features">
  <h2>Measured capabilities</h2>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>Comparison window</h2>
  <div class="models">{models}</div>
  <div style="overflow:auto;margin-top:20px">
    <table>
      <thead><tr><th>Parameter</th><th>M17-60</th><th>M17-48</th><th>M17-40</th></tr></thead>
      <tbody>{spec_rows()}</tbody>
    </table>
  </div>
</section>
<section id="start">
  <h2>Command example</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:14px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px;color:#4a5c4a">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <h2>Deployed on</h2>
  <div class="apps">
    <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robotics &amp; CNC</figcaption></figure>
    <figure><img src="../automation_small.jpg" alt=""><figcaption>Instruments</figcaption></figure>
    <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Test equipment</figcaption></figure>
  </div>
</section>
<section id="about" class="split">
  <div>
    <h2>Open instrument</h2>
    <p>{COMPANY}</p>
    <p style="margin-top:12px"><a class="shop" href="{GITHUB}">GitHub</a> <a href="{DOCS}" style="margin-left:12px">Documentation</a></p>
    <div class="os">
      <img src="../Open-source-hardware-logo.svg.png" alt="">
      <img src="../Open_Source_Initiative.svg.png" alt="">
    </div>
  </div>
  <img src="../transparent/motor_back_transparent_small.png" alt="Back of motor with labels">
</section>
<footer>
  <h2>Order an M17</h2>
  <p style="margin:12px 0 20px">Three torques. One protocol.</p>
  <a class="shop" href="{SHOP}">Shop Now</a>
</footer>
"""
    w("v5.html", shell(TITLE, css, body))


# ---------------------------------------------------------------------------
# v6 Kinetic Poster
# ---------------------------------------------------------------------------
def v6():
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.4 Impact,"Arial Narrow",Haettenschweiler,sans-serif;background:#fff;color:#111}
img{max-width:100%;height:auto;display:block}
a{color:#111;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 20px;background:#fff;border-bottom:8px solid #111}
nav img{height:28px}
.shop{background:#7AB648;color:#111;padding:10px 16px;font-size:18px;letter-spacing:.02em}
.shop:hover{background:#111;color:#7AB648}
.poster{padding:12px 20px 40px}
.poster h1{font-size:clamp(64px,16vw,180px);line-height:.8;letter-spacing:-.04em;text-transform:uppercase}
.bar{display:flex;flex-wrap:wrap;gap:8px;margin:16px 0}
.bar span{background:#111;color:#fff;padding:6px 10px;font-size:18px}
.grid{display:grid;grid-template-columns:2fr 1fr;gap:8px}
.grid .g{background:#7AB648;padding:16px}
.grid .g.dark{background:#111;color:#fff}
.sans{font-family:Helvetica,Arial,sans-serif;font-weight:400;font-size:16px;line-height:1.5}
section{padding:28px 20px;border-top:8px solid #111}
h2{font-size:clamp(36px,8vw,72px);line-height:.9;text-transform:uppercase;margin-bottom:16px}
.feats{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:8px}
.feats article{border:4px solid #111;padding:12px;min-height:130px}
.feats h3{font-size:22px;text-transform:uppercase;line-height:1}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.models article{background:#111;color:#fff;padding:16px}
.models h3{font-size:42px}
pre{background:#111;color:#7AB648;padding:20px;overflow:auto;font:13px/1.5 ui-monospace,Menlo,monospace}
.apps{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
.apps img{height:200px;width:100%;object-fit:cover;filter:contrast(1.15) saturate(.7)}
footer{background:#7AB648;padding:40px 20px 64px}
.os{display:flex;gap:16px;margin:16px 0}
.os img{height:48px;background:#fff;padding:4px}
@media(max-width:800px){.grid,.feats,.models,.apps{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p class='sans'>{d}</p></article>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article><h3>{m['name']}</h3><p class='sans'>{m['torque']} · {m['power']} · {m['weight']}</p><img src='{m['dim']}' alt='' style='background:#fff;margin-top:8px'></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">SHOP</a>
</nav>
<header class="poster" id="top">
  <h1>M17<br>SERIES</h1>
  <div class="bar">
    <span>MOTOR</span><span>DRIVER</span><span>CONTROLLER</span><span>ENCODER</span>
  </div>
  <div class="grid">
    <img src="../transparent/one_motor_transparent_small.png" alt="M17" style="background:#eee">
    <div>
      <div class="g"><p class="sans">{TAGLINE}</p></div>
      <div class="g dark" style="margin-top:8px"><p class="sans">{INTRO}</p></div>
    </div>
  </div>
</header>
<section id="features">
  <h2>Hard facts</h2>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>60 / 48 / 40</h2>
  <div class="models">{models}</div>
  <div style="overflow:auto;margin-top:16px" class="sans">
    <table style="width:100%;border-collapse:collapse">
      <thead><tr><th style="text-align:left;border-bottom:4px solid #111;padding:8px">Parameter</th>
      <th style="text-align:left;border-bottom:4px solid #111;padding:8px">M17-60</th>
      <th style="text-align:left;border-bottom:4px solid #111;padding:8px">M17-48</th>
      <th style="text-align:left;border-bottom:4px solid #111;padding:8px">M17-40</th></tr></thead>
      <tbody>{spec_rows()}</tbody>
    </table>
  </div>
</section>
<section id="start">
  <h2>Type this</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p class="sans" style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <h2>Use it</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Testing">
  </div>
</section>
<footer id="about">
  <h2>Open source</h2>
  <p class="sans" style="max-width:60ch">{COMPANY}</p>
  <div class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </div>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <a class="shop" href="{DOCS}" style="background:#111;color:#fff;margin-left:8px">Docs</a>
</footer>
"""
    w("v6.html", shell(TITLE, css, body))


# ---------------------------------------------------------------------------
# v7 Control Room
# ---------------------------------------------------------------------------
def v7():
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:13px/1.5 ui-monospace,Menlo,Consolas,monospace;background:#07110c;color:#b6f5c4}
img{max-width:100%;height:auto;display:block}
a{color:#b6f5c4}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:10px 16px;background:#04100a;border-bottom:1px solid #1c3d28}
nav img{height:22px;filter:invert(1) sepia(1) hue-rotate(80deg) saturate(3)}
.shop{border:1px solid #7AB648;color:#7AB648;padding:6px 10px;text-decoration:none}
.shop:hover{background:#7AB648;color:#04100a}
.shell{display:grid;grid-template-columns:220px 1fr;min-height:100vh}
.rail{border-right:1px solid #1c3d28;padding:16px;background:#06140c}
.rail h2{font-size:11px;letter-spacing:.16em;color:#7AB648;margin:18px 0 8px}
.rail ul{list-style:none}
.rail li{padding:4px 0;border-bottom:1px dotted #1c3d28}
main{padding:20px 24px 64px}
h1{font-size:28px;font-weight:500;color:#e8ffe9;margin:8px 0 16px}
.led{display:inline-block;width:8px;height:8px;border-radius:50%;background:#7AB648;box-shadow:0 0 8px #7AB648;margin-right:6px}
.panel{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin:16px 0}
.panel div{border:1px solid #1c3d28;padding:10px;background:#0a1810}
.panel b{display:block;font-size:22px;color:#fff}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.feats article{border:1px solid #1c3d28;padding:10px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.models article{border:1px solid #1c3d28;padding:10px}
.models img{background:#fff;margin:8px 0}
pre{background:#020805;border:1px solid #1c3d28;padding:14px;overflow:auto;color:#9dffb0}
table{width:100%;border-collapse:collapse}
th,td{border:1px solid #1c3d28;padding:6px 8px;text-align:left}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.apps img{height:150px;width:100%;object-fit:cover;filter:hue-rotate(70deg) saturate(.4) contrast(1.1)}
.os img{height:40px;background:#fff;padding:3px;margin-right:8px}
@media(max-width:900px){.shell{grid-template-columns:1fr}.rail{display:none}.panel,.models,.apps,.feats{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><strong>CH{i:02d}  {t}</strong><p>{d}</p></article>" for i, (t, d) in enumerate(FEATURES[:8], 1))
    models = "".join(
        f"<article><div><span class='led'></span>{m['name']}</div><img src='{m['dim']}' alt=''><p>TQ {m['torque']}<br>PWR {m['power']}<br>M {m['weight']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <span>BUS 230400 · ADDR READY</span>
  <a class="shop" href="{SHOP}">SHOP</a>
</nav>
<div class="shell">
<aside class="rail">
  <h2>Channels</h2>
  <ul>
    <li><a href="#features">01 FEATURES</a></li>
    <li><a href="#models">02 MODELS</a></li>
    <li><a href="#start">03 COMMAND</a></li>
    <li><a href="#apps">04 PLANTS</a></li>
    <li><a href="#about">05 SOURCE</a></li>
  </ul>
  <h2>Status</h2>
  <ul>
    <li>LOOP 32 kHz</li>
    <li>VBUS 12–24</li>
    <li>NMAX 560 RPM</li>
    <li>PROT OC/OV/OT</li>
  </ul>
</aside>
<main id="top">
  <p><span class="led"></span>SYSTEM ONLINE</p>
  <h1>M17 / MOTION CONTROLLER</h1>
  <p>{TAGLINE}</p>
  <p style="margin-top:10px;max-width:70ch">{INTRO}</p>
  <div class="panel">
    <div><b>32k</b>PID Hz</div>
    <div><b>560</b>RPM</div>
    <div><b>1.1</b>A max</div>
    <div><b>3</b>models</div>
  </div>
  <img src="../transparent/one_motor_transparent_small.png" alt="M17" style="max-height:360px;margin:12px auto">
  <section id="features" style="margin-top:28px">
    <h2 style="color:#7AB648">01 FEATURE MATRIX</h2>
    <div class="feats" style="margin-top:10px">{feats}</div>
  </section>
  <section id="models" style="margin-top:28px">
    <h2 style="color:#7AB648">02 UNIT SELECT</h2>
    <div class="models" style="margin-top:10px">{models}</div>
    <div style="overflow:auto;margin-top:12px">
      <table>
        <thead><tr><th>PARAM</th><th>M17-60</th><th>M17-48</th><th>M17-40</th></tr></thead>
        <tbody>{spec_rows()}</tbody>
      </table>
    </div>
  </section>
  <section id="start" style="margin-top:28px">
    <h2 style="color:#7AB648">03 HOST COMMAND</h2>
    <pre style="margin-top:10px"><code>{esc(CODE)}</code></pre>
    <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
  </section>
  <section id="apps" style="margin-top:28px">
    <h2 style="color:#7AB648">04 PLANTS</h2>
    <div class="apps" style="margin-top:10px">
      <img src="../robotics_small.jpg" alt="">
      <img src="../automation_small.jpg" alt="">
      <img src="../test_rack_small.jpg" alt="">
    </div>
    <p style="margin-top:8px">ROBOTICS · CNC · AUTOMATION · INSTRUMENTS · 3D PRINT · EDUCATION</p>
  </section>
  <section id="about" style="margin-top:28px">
    <h2 style="color:#7AB648">05 OPEN SOURCE</h2>
    <p style="margin:10px 0">{COMPANY}</p>
    <p><a class="shop" href="{GITHUB}">GITHUB</a> <a class="shop" href="{DOCS}">DOCS</a></p>
    <p class="os" style="margin-top:12px">
      <img src="../Open-source-hardware-logo.svg.png" alt="">
      <img src="../Open_Source_Initiative.svg.png" alt="">
    </p>
  </section>
  <p style="margin-top:36px"><a class="shop" href="{SHOP}">SHOP NOW</a></p>
</main>
</div>
"""
    w("v7.html", shell(TITLE, css, body))


# ---------------------------------------------------------------------------
# v8 Object Room
# ---------------------------------------------------------------------------
def v8():
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.6 "Gill Sans","Gill Sans MT",Calibri,Helvetica,sans-serif;background:#fafafa;color:#222}
img{max-width:100%;height:auto;display:block}
a{color:#222;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:20px 8vw;background:#fafafa}
nav img{height:24px}
.shop{font-size:13px;letter-spacing:.12em;text-transform:uppercase;border-bottom:1px solid #222}
.shop:hover{color:#7AB648;border-color:#7AB648}
.hero{min-height:86vh;display:flex;flex-direction:column;align-items:center;justify-content:center;padding:24px}
.hero img{max-height:62vh}
.cap{margin-top:28px;text-align:center}
.cap h1{font-weight:400;font-size:22px;letter-spacing:.04em}
.cap p{color:#666;font-size:14px;margin-top:6px}
section{max-width:640px;margin:0 auto;padding:72px 24px}
h2{font-weight:400;font-size:20px;letter-spacing:.08em;text-transform:uppercase;margin-bottom:20px}
.feats li{list-style:none;padding:14px 0;border-top:1px solid #e4e4e4}
.models{display:grid;gap:32px}
.models article{display:grid;grid-template-columns:140px 1fr;gap:20px;align-items:center}
pre{background:#111;color:#fafafa;padding:20px;overflow:auto;font:12.5px/1.55 ui-monospace,Menlo,monospace}
.wide{max-width:960px;margin:0 auto;padding:0 24px 72px}
.wide img{width:100%;max-height:360px;object-fit:cover}
footer{text-align:center;padding:72px 24px 96px}
.os{display:flex;justify-content:center;gap:20px;margin:20px 0}
.os img{height:44px}
@media(max-width:700px){.models article{grid-template-columns:1fr}}
"""
    feats = "".join(f"<li><strong>{t}.</strong> {d}</li>" for t, d in FEATURES[:7])
    models = "".join(
        f"<article><img src='{m['dim']}' alt=''><div><h3>{m['name']}</h3><p>{m['torque']} · {m['height']} · {m['weight']}</p></div></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17 servomotor">
  <div class="cap">
    <h1>M17 Series Servomotor</h1>
    <p>{TAGLINE}</p>
  </div>
</header>
<section>
  <h2>Label</h2>
  <p>{INTRO}</p>
  <p style="margin-top:16px;color:#555">{INTRO2}</p>
</section>
<section id="features">
  <h2>Notes</h2>
  <ul class="feats">{feats}</ul>
</section>
<section id="models">
  <h2>Three objects</h2>
  <div class="models">{models}</div>
</section>
<div class="wide" style="overflow:auto">
  <table style="width:100%;border-collapse:collapse;font-size:14px">
    <thead><tr><th style="text-align:left;padding:8px;border-bottom:1px solid #ddd">Parameter</th>
    <th style="text-align:left;padding:8px;border-bottom:1px solid #ddd">M17-60</th>
    <th style="text-align:left;padding:8px;border-bottom:1px solid #ddd">M17-48</th>
    <th style="text-align:left;padding:8px;border-bottom:1px solid #ddd">M17-40</th></tr></thead>
    <tbody>{spec_rows()}</tbody>
  </table>
</div>
<section id="start">
  <h2>A first movement</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:10px;color:#666">{INSTALL} · {ARDUINO}</p>
</section>
<div class="wide">
  <img src="../automation_small.jpg" alt="In use">
</div>
<footer id="about">
  <h2>Studio</h2>
  <p style="max-width:46ch;margin:0 auto">{COMPANY}</p>
  <div class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </div>
  <p><a class="shop" href="{SHOP}">Shop Now</a> &nbsp; <a class="shop" href="{DOCS}">Documentation</a> &nbsp; <a class="shop" href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v8.html", shell(TITLE, css, body))


# ---------------------------------------------------------------------------
# v9 Classroom Kit
# ---------------------------------------------------------------------------
def v9():
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:17px/1.5 "Trebuchet MS","Gill Sans",Helvetica,sans-serif;background:#fffdf7;color:#243047}
img{max-width:100%;height:auto;display:block}
a{color:#243047}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 24px;background:#fffdf7;border-bottom:4px solid #243047}
nav img{height:36px}
.shop{background:#ffb703;padding:10px 16px;text-decoration:none;font-weight:800;border:3px solid #243047;box-shadow:3px 3px 0 #243047}
.shop:hover{background:#7AB648}
.hero{padding:40px 24px;display:grid;grid-template-columns:1fr 1fr;gap:28px;align-items:center;max-width:1100px;margin:0 auto}
h1{font-size:clamp(32px,5vw,52px);line-height:1.05}
.steps{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;padding:12px 24px 40px;max-width:1100px;margin:0 auto}
.steps article{border:3px solid #243047;padding:14px;background:#fff;min-height:140px}
.steps b{display:block;font-size:28px;color:#e85d04}
section{padding:40px 24px;max-width:1100px;margin:0 auto}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.feats article{background:#cde7f5;border:3px solid #243047;padding:14px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.models article:nth-child(1){background:#ffd6a5}
.models article:nth-child(2){background:#caffbf}
.models article:nth-child(3){background:#bdb2ff}
.models article{border:3px solid #243047;padding:12px}
pre{background:#243047;color:#fffdf7;padding:18px;overflow:auto;font:13px/1.5 ui-monospace,Menlo,monospace;border:3px solid #243047}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.apps figure{border:3px solid #243047;background:#fff;padding:8px}
.apps img{height:160px;width:100%;object-fit:cover}
footer{background:#243047;color:#fffdf7;padding:40px 24px 64px}
footer a{color:#fffdf7}
.os{display:flex;gap:16px;margin:16px 0}
.os img{height:46px;background:#fff;padding:4px}
@media(max-width:800px){.hero,.steps,.feats,.models,.apps{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']} · good for {['heavier arms','most benches','small robots'][i]}</p></article>"
        for i, m in enumerate(MODELS)
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <p style="font-weight:800;letter-spacing:.08em;text-transform:uppercase;font-size:13px">For the classroom and the club</p>
    <h1>Teach real servos without the scary cabinet.</h1>
    <p style="margin-top:12px">{TAGLINE}. One motor. One USB-RS485 adapter. A Python script. A spinning shaft.</p>
  </div>
  <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="Classroom kit">
</header>
<div class="steps">
  <article><b>1</b>Plug the adapter into a computer.</article>
  <article><b>2</b>Daisy-chain the motors on RS-485.</article>
  <article><b>3</b>Install <code>servomotor</code>.</article>
  <article><b>4</b>Call <code>trapezoid_move</code>.</article>
</div>
<section id="features">
  <h2>Why teachers pick it</h2>
  <p>{INTRO}</p>
  <div class="feats" style="margin-top:16px">{feats}</div>
</section>
<section id="models">
  <h2>Three sizes for three budgets</h2>
  <div class="models">{models}</div>
  <div style="overflow:auto;margin-top:16px;background:#fff;border:3px solid #243047">
    <table style="width:100%;border-collapse:collapse">
      <thead><tr><th style="text-align:left;padding:8px">Parameter</th><th style="text-align:left;padding:8px">M17-60</th>
      <th style="text-align:left;padding:8px">M17-48</th><th style="text-align:left;padding:8px">M17-40</th></tr></thead>
      <tbody>{spec_rows()}</tbody>
    </table>
  </div>
</section>
<section id="start">
  <h2>The whole first lesson</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <h2>Projects people actually build</h2>
  <div class="apps">
    <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robot arms</figcaption></figure>
    <figure><img src="../automation_small.jpg" alt=""><figcaption>Small machines</figcaption></figure>
    <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Lab racks</figcaption></figure>
  </div>
</section>
<footer id="about">
  <h2>Open source, on purpose</h2>
  <p>{COMPANY}</p>
  <div class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </div>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <a href="{DOCS}" style="margin-left:16px">Documentation</a>
  <a href="{GITHUB}" style="margin-left:12px">GitHub</a>
</footer>
"""
    w("v9.html", shell(TITLE, css, body))


# ---------------------------------------------------------------------------
# v10 Blueprint
# ---------------------------------------------------------------------------
def v10():
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:15px/1.5 "Century Gothic",Futura,Avenir,Helvetica,sans-serif;background:#0b1c3a;color:#d7ecff}
img{max-width:100%;height:auto;display:block}
a{color:#9ad4ff}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 24px;background:#0b1c3a;border-bottom:1px solid #3aa0ff}
nav img{height:26px;filter:invert(1)}
.shop{border:1px solid #3aa0ff;color:#3aa0ff;padding:8px 14px;text-decoration:none;letter-spacing:.12em;text-transform:uppercase;font-size:12px}
.shop:hover{background:#3aa0ff;color:#0b1c3a}
.hero{padding:48px 24px;max-width:1140px;margin:0 auto;display:grid;grid-template-columns:1fr 1fr;gap:32px;align-items:center}
h1{font-size:clamp(32px,5vw,54px);letter-spacing:.06em;text-transform:uppercase;color:#fff}
.rule{height:1px;background:#3aa0ff;margin:16px 0}
section{padding:48px 24px;max-width:1140px;margin:0 auto;border-top:1px solid #1e4a7a}
h2{letter-spacing:.14em;text-transform:uppercase;font-size:14px;color:#3aa0ff;margin-bottom:14px}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:0}
.feats article{border:1px solid #1e4a7a;padding:14px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.models article{border:1px solid #3aa0ff;padding:12px;background:rgba(58,160,255,.05)}
.models img{background:#fff}
pre{background:#061226;border:1px solid #1e4a7a;padding:18px;overflow:auto;font:12.5px/1.55 ui-monospace,Menlo,monospace;color:#b8e0ff}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.apps img{height:170px;width:100%;object-fit:cover;filter:sepia(1) hue-rotate(180deg) saturate(.8)}
.os{display:flex;gap:16px;margin:16px 0}
.os img{height:46px;background:#fff;padding:4px}
table{width:100%;border-collapse:collapse}
th,td{border:1px solid #1e4a7a;padding:8px;text-align:left}
@media(max-width:800px){.hero,.feats,.models,.apps{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><strong>{t}</strong><p>{d}</p></article>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt='{m['name']} drawing'><p>{m['torque']} · {m['height']} · {m['weight']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <p style="letter-spacing:.2em;text-transform:uppercase;font-size:11px;color:#3aa0ff">Sheet 01 · Assembly</p>
    <h1>M17<br>all-in-one servo</h1>
    <div class="rule"></div>
    <p>{TAGLINE}</p>
    <p style="margin-top:12px">{INTRO}</p>
  </div>
  <img src="../transparent/M17_series_overview_transparent_small.png" alt="Series overview">
</header>
<section id="features">
  <h2>Notes — do not scale</h2>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>Detail drawings</h2>
  <div class="models">{models}</div>
  <div style="overflow:auto;margin-top:18px">
    <table>
      <thead><tr><th>Parameter</th><th>M17-60</th><th>M17-48</th><th>M17-40</th></tr></thead>
      <tbody>{spec_rows()}</tbody>
    </table>
  </div>
</section>
<section id="start">
  <h2>Revision A — host listing</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
  <img src="../connection_diagram.jpg" alt="Connection diagram" style="margin-top:16px;background:#fff;padding:8px">
</section>
<section>
  <h2>Site photos</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
  </div>
</section>
<section id="about">
  <h2>Title block</h2>
  <p>{COMPANY}</p>
  <div class="os">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </div>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <a class="shop" href="{DOCS}">Docs</a>
  <a class="shop" href="{GITHUB}">GitHub</a>
</section>
"""
    w("v10.html", shell(TITLE, css, body))


if __name__ == "__main__":
    v1(); v2(); v3(); v4(); v5()
    v6(); v7(); v8(); v9(); v10()

