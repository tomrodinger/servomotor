#!/usr/bin/env python3
"""Round 2 — v31–v40 company pastiches."""
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


def v31():
    """Visa — blue/gold bands, geometric, network clarity."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.5 "Avenir Next",Avenir,Helvetica,sans-serif;background:#fff;color:#1a1f71}
img{max-width:100%;height:auto;display:block}
a{color:#1a1f71}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#1a1f71}
nav img{height:24px;filter:invert(1)}
.shop{background:#f7b600;color:#1a1f71;padding:10px 18px;text-decoration:none;font-weight:700;border-radius:6px}
.shop:hover{background:#ffd24a}
.hero{background:#1a1f71;color:#fff;padding:72px 6vw 48px}
.hero h1{font-size:clamp(56px,12vw,140px);line-height:.8;letter-spacing:-.04em}
.goldline{height:6px;background:#f7b600}
section{padding:56px 6vw}
.bands{display:grid;grid-template-columns:1fr 1fr;gap:0}
.bands article{padding:32px}
.bands .b{background:#1a1f71;color:#fff}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
pre{background:#1a1f71;color:#fff;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace;border-radius:8px}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.apps img{height:170px;width:100%;object-fit:cover}
.os img{height:42px;margin-right:8px}
footer{background:#1a1f71;color:#fff;padding:40px 6vw}
footer a{color:#f7b600}
@media(max-width:800px){.bands,.models,.apps{grid-template-columns:1fr}}
"""
    feats = "".join(
        f"<article class='{'b' if i%2 else ''}'><h3>{t}</h3><p>{d}</p></article>"
        for i, (t, d) in enumerate(FEATURES[:6])
    )
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <p style="letter-spacing:.2em;text-transform:uppercase;font-size:12px;color:#f7b600">Everywhere you build</p>
  <h1>M17</h1>
  <p style="max-width:28em;margin-top:16px">{TAGLINE}. One network. Any number of motors.</p>
</header>
<div class="goldline"></div>
<section>
  <h2>Accepted on every host that matters.</h2>
  <p style="max-width:46em;margin-top:12px">{INTRO}</p>
</section>
<section id="features" class="bands">{feats}</section>
<section id="models">
  <h2>Choose a denomination</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:2px solid #1a1f71', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Clear the transaction</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
  </div>
</section>
<section id="about">
  <h2>The issuer</h2>
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
    w("v31.html", shell(TITLE, css, body))


def v32():
    """J&J — red, two doorways, mission hero."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:17px/1.6 "Gill Sans","Gill Sans MT",Calibri,Helvetica,sans-serif;background:#fff;color:#222}
img{max-width:100%;height:auto;display:block}
a{color:#222}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#fff}
nav img{height:28px}
.shop{background:#eb1700;color:#fff;padding:10px 18px;text-decoration:none}
.shop:hover{background:#c41200;color:#fff}
.hero{padding:64px 6vw;display:grid;grid-template-columns:1fr 1fr;gap:32px;align-items:center}
.hero h1{font-size:clamp(32px,5vw,52px);font-weight:500;line-height:1.15}
.doors{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:0 6vw 48px}
.doors article{background:#f6f3ef;padding:28px;min-height:280px;display:flex;flex-direction:column;justify-content:flex-end}
.doors h2{font-weight:500}
section{padding:48px 6vw}
.stories{display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px}
.stories article{border-top:3px solid #eb1700;padding-top:12px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
pre{background:#f6f3ef;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.os img{height:44px;margin-right:8px}
footer{padding:40px 6vw 64px;border-top:1px solid #eee}
@media(max-width:800px){.hero,.doors,.stories,.models{grid-template-columns:1fr}}
"""
    stories = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <h1>We’re tackling the hard motion problems.</h1>
    <p style="margin-top:14px">{TAGLINE}. {INTRO}</p>
  </div>
  <img src="../transparent/one_motor_transparent_small.png" alt="">
</header>
<div class="doors">
  <article>
    <h2>Education</h2>
    <p>A kit a class can finish in an afternoon. Python, Arduino, and manuals an AI can read with you.</p>
  </article>
  <article>
    <h2>Industry</h2>
    <p>Robotics, CNC, instruments, test jigs. One RS-485 bus. Closed loop at 32 kHz.</p>
  </article>
</div>
<section id="features">
  <h2>Innovation notes</h2>
  <div class="stories">{stories}</div>
</section>
<section id="models">
  <h2>The family</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #eee', 'margin-top:16px')}
</section>
<section id="start">
  <h2>A first procedure</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <img src="../automation_small.jpg" alt="" style="max-height:360px;width:100%;object-fit:cover">
</section>
<section id="about">
  <h2>Our credo, in short</h2>
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
    w("v32.html", shell(TITLE, css, body))


def v33():
    """Intel — blue, product badges, dark lockup then white."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.5 "Segoe UI",Tahoma,Helvetica,sans-serif;background:#fff;color:#212121}
img{max-width:100%;height:auto;display:block}
a{color:#0071c5}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 24px;background:#fff;border-bottom:1px solid #e5e5e5}
nav img{height:26px}
.shop{background:#0071c5;color:#fff;padding:8px 16px;text-decoration:none}
.shop:hover{background:#005a9e;color:#fff}
.hero{background:#0b1220;color:#fff;padding:64px 6vw;display:grid;grid-template-columns:1fr 1fr;gap:32px;align-items:center}
.badge{width:220px;height:220px;border:4px solid #0071c5;border-radius:12px;display:grid;place-items:center;margin:0 auto;background:radial-gradient(circle,#123,#0b1220)}
.badge img{max-height:160px}
.claim{padding:40px 6vw;text-align:center;font-size:28px}
.badges{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;padding:0 6vw 48px}
.badges article{border:2px solid #0071c5;padding:16px;text-align:center}
section{padding:40px 6vw}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:16px}
pre{background:#0b1220;color:#d6e8ff;padding:18px;overflow:auto;font:13px/1.55 Consolas,monospace}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.apps img{height:160px;width:100%;object-fit:cover}
.os img{height:42px;margin-right:8px}
@media(max-width:800px){.hero,.badges,.feats,.apps{grid-template-columns:1fr}}
"""
    models = "".join(
        f"<article><img src='{m['dim']}' alt=''><h3>{m['name']}</h3><p>{m['torque']} · {m['power']}</p></article>"
        for m in MODELS
    )
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:8])
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <p style="color:#00aeef;letter-spacing:.14em;text-transform:uppercase;font-size:12px">Built for motion</p>
    <h1>Meet the future of the joint.</h1>
    <p style="margin-top:12px">{TAGLINE}</p>
    <p style="margin-top:16px"><a class="shop" href="{SHOP}">Explore the capabilities</a></p>
  </div>
  <div class="badge"><img src="../transparent/one_motor_transparent_small.png" alt="M17"></div>
</header>
<p class="claim">We put the driver, the controller, and the encoder in the motor so builders can move faster — from first sketch to a rack of axes.</p>
<div class="badges" id="models">{models}</div>
<section id="features">
  <h2>Break the cabinet barrier</h2>
  <p>{INTRO} {INTRO2}</p>
  <div class="feats" style="margin-top:16px">{feats}</div>
  {spec_table('background:#e8f3fb;border-bottom:1px solid #c5dff0', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Developer zone</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <div class="apps">
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
    w("v33.html", shell(TITLE, css, body))


def v34():
    """ExxonMobil — energy red, fact strip (no fake prices), three units."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.55 Helvetica,Arial,sans-serif;background:#fff;color:#1b1b1b}
img{max-width:100%;height:auto;display:block}
a{color:#e31937}
nav{position:sticky;top:0;z-index:20;background:#fff}
.ticker{background:#0b1f3a;color:#fff;padding:8px 20px;font-size:13px;display:flex;gap:28px;flex-wrap:wrap;letter-spacing:.04em}
nav .bar{display:flex;justify-content:space-between;align-items:center;padding:12px 20px;border-bottom:3px solid #e31937}
nav img{height:26px}
.shop{background:#e31937;color:#fff;padding:8px 16px;text-decoration:none;font-weight:700}
.shop:hover{background:#b8142c;color:#fff}
.hero{padding:48px 6vw;background:#0b1f3a;color:#fff;display:grid;grid-template-columns:1.2fr .8fr;gap:28px;align-items:center}
.units{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;padding:32px 6vw}
.units article{border-top:4px solid #e31937;padding-top:14px}
section{padding:40px 6vw}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:16px}
pre{background:#0b1f3a;color:#fff;padding:18px;overflow:auto;font:13px/1.55 Consolas,monospace}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.apps img{height:160px;width:100%;object-fit:cover}
.os img{height:42px;margin-right:8px}
footer{background:#0b1f3a;color:#fff;padding:32px 6vw}
footer a{color:#fff}
@media(max-width:800px){.hero,.units,.feats,.apps{grid-template-columns:1fr}}
"""
    units = [
        ("Applications", "Robotics, CNC, instruments, test, education — the field the shaft serves.", "../robotics_small.jpg"),
        ("Product solutions", "Three models. One protocol. Motor, driver, controller, encoder.", "../transparent/M17_series_overview_transparent_small.png"),
        ("Efficiency", "Closed loop draws what the load needs — not a constant holding current.", "../transparent/one_motor_transparent_small.png"),
    ]
    unit_html = "".join(
        f"<article><img src='{img}' alt='' style='margin-bottom:10px'><h3>{t}</h3><p>{d}</p></article>"
        for t, d, img in units
    )
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<p><b>{m['name']}</b> — {m['torque']}, {m['power']}, {m['weight']}<br><img src='{m['dim']}' alt='' style='max-width:280px'></p>"
        for m in MODELS
    )
    body = f"""
<nav>
  <div class="ticker">
    <span>12–24 V</span><span>560 RPM</span><span>NEMA 17</span><span>32 kHz PID</span><span>3 models</span>
  </div>
  <div class="bar">
    <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
    <a class="shop" href="{SHOP}">Shop</a>
  </div>
</nav>
<header class="hero" id="top">
  <div>
    <h1>The need for motion is universal.</h1>
    <p style="margin-top:12px">{TAGLINE}. {INTRO}</p>
    <p style="margin-top:16px"><a class="shop" href="#models">Who we are</a></p>
  </div>
  <img src="../automation_small.jpg" alt="">
</header>
<div class="units">{unit_html}</div>
<section id="features">
  <h2>Trending capabilities</h2>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>The portfolio</h2>
  {models}
  {spec_table('background:#f4f4f4;border-bottom:1px solid #ddd', 'margin-top:12px')}
</section>
<section id="start">
  <h2>Operations notes</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
    <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="">
  </div>
</section>
<section id="about">
  <h2>About us</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin:12px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
</section>
<footer>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <p style="margin-top:12px"><a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v34.html", shell(TITLE, css, body))


def v35():
    """Tencent — statement hero, for makers / labs / future."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.55 "Avenir Next",Avenir,Helvetica,sans-serif;background:#fff;color:#111}
img{max-width:100%;height:auto;display:block}
a{color:#111}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:16px 28px;background:#fff}
nav img{height:26px}
.shop{color:#00c800;border:1px solid #00c800;padding:8px 16px;text-decoration:none;border-radius:999px}
.shop:hover{background:#00c800;color:#111}
.hero{min-height:70vh;display:grid;place-items:center;text-align:center;padding:48px 20px}
.hero h1{font-size:clamp(36px,6vw,64px);font-weight:500}
.story{display:grid;grid-template-columns:1fr 1fr;min-height:60vh}
.story .t{padding:48px 8vw;display:flex;flex-direction:column;justify-content:center}
.story img{width:100%;height:100%;object-fit:cover}
.k{color:#00c800;font-size:13px;letter-spacing:.12em;text-transform:uppercase}
section{padding:56px 8vw}
.feats{display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
pre{background:#111;color:#b6ffb6;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.os img{height:42px;margin-right:8px}
footer{padding:40px 8vw 64px}
@media(max-width:800px){.story,.feats,.models{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <h1>We build technologies<br>that create the future of a joint.</h1>
    <p style="margin-top:16px;color:#555">{TAGLINE}</p>
    <p style="margin-top:20px"><a class="shop" href="#about">Who we are</a></p>
  </div>
</header>
<div class="story">
  <div class="t">
    <p class="k">For makers</p>
    <h2>Three friends sharing one adapter.</h2>
    <p style="margin-top:12px">And any number of motors on the bus.</p>
  </div>
  <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="">
</div>
<div class="story">
  <img src="../automation_small.jpg" alt="">
  <div class="t">
    <p class="k">For labs</p>
    <h2>One breakthrough on the bench.</h2>
    <p style="margin-top:12px">{INTRO}</p>
  </div>
</div>
<div class="story">
  <div class="t">
    <p class="k">For the future</p>
    <h2>Documentation an AI can finish with you.</h2>
    <p style="margin-top:12px">{INTRO2}</p>
  </div>
  <img src="../test_rack_small.jpg" alt="">
</div>
<section id="features">
  <h2>What we create</h2>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>Everything we make in this series</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #eee', 'margin-top:16px')}
</section>
<section id="start">
  <h2>A listing</h2>
  <pre><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section id="about">
  <h2>Value for users, tech for good</h2>
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
    w("v35.html", shell(TITLE, css, body))


def v36():
    """Mastercard — bold urban, red/orange dots as decoration (not the mark)."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:18px/1.4 "Avenir Next",Avenir,Helvetica,sans-serif;background:#f6f1ea;color:#111}
img{max-width:100%;height:auto;display:block}
a{color:#111;text-decoration:none}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:16px 28px;background:#f6f1ea}
nav img{height:26px}
.dots{display:flex;gap:6px}
.dots i{width:18px;height:18px;border-radius:50%;display:block}
.shop{background:#111;color:#fff;padding:10px 18px}
.shop:hover{background:#eb001b;color:#fff}
.hero{padding:64px 8vw 40px}
.hero h1{font-size:clamp(48px,10vw,112px);line-height:.85;letter-spacing:-.045em;font-weight:800}
section{padding:40px 8vw}
.poster{display:grid;grid-template-columns:1.2fr .8fr;gap:24px;align-items:center}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:20px}
.feats article{font-size:22px;font-weight:700;line-height:1.2}
.feats p{font-size:16px;font-weight:400;margin-top:8px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
pre{background:#111;color:#f6f1ea;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.os img{height:42px;margin-right:8px}
footer{padding:40px 8vw 72px}
@media(max-width:800px){.poster,.feats,.models{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article>{t}<p>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article style='background:#fff;padding:12px'><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <span class="dots"><i style="background:#eb001b"></i><i style="background:#ff5f00"></i></span>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <h1>Priceless<br>is a shaft<br>that just works.</h1>
  <p style="margin-top:16px;max-width:22em">{TAGLINE}</p>
</header>
<section class="poster">
  <img src="../transparent/one_motor_transparent_small.png" alt="">
  <p style="font-size:24px;font-weight:600">{INTRO}</p>
</section>
<section id="features">
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2 style="font-size:40px;margin-bottom:16px">Three.</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:2px solid #111', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Start</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <img src="../robotics_small.jpg" alt="" style="width:100%;max-height:360px;object-fit:cover">
</section>
<section id="about">
  <h2>Open</h2>
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
    w("v36.html", shell(TITLE, css, body))


def v37():
    """Applied Materials — teal, navy, process window, three capabilities."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.55 "Avenir Next",Avenir,Helvetica,sans-serif;background:#f4f7f7;color:#0b1f33}
img{max-width:100%;height:auto;display:block}
a{color:#0b1f33}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#0b1f33}
nav img{height:24px;filter:invert(1)}
.shop{background:#00a3a1;color:#fff;padding:8px 16px;text-decoration:none}
.shop:hover{background:#067f7d;color:#fff}
.hero{padding:64px 6vw;background:#0b1f33;color:#fff}
.hero h1{font-size:clamp(32px,5vw,52px);max-width:16ch}
.caps{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;padding:32px 6vw}
.caps article{background:#fff;padding:20px;border-top:4px solid #00a3a1}
section{padding:48px 6vw}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
pre{background:#0b1f33;color:#cdeff0;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.apps img{width:100%;height:180px;object-fit:cover;filter:saturate(.6)}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.os img{height:42px;margin-right:8px}
@media(max-width:800px){.caps,.feats,.models,.apps{grid-template-columns:1fr}}
"""
    caps = [
        ("Integrate", "Motor, driver, controller, encoder — one process of assembly."),
        ("Connect", "RS-485 daisy-chain. Any number of axes from one host."),
        ("Control", "32 kHz PID. Trapezoid moves. High-level commands."),
    ]
    cap_html = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in caps)
    feats = "".join(f"<article><strong>{t}</strong><p>{d}</p></article>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article style='background:#fff;padding:12px'><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <p style="color:#00a3a1;letter-spacing:.16em;text-transform:uppercase;font-size:12px">Materials. Motion. Possible.</p>
  <h1>The process window for a joint.</h1>
  <p style="margin-top:14px;max-width:36em">{TAGLINE}. {INTRO}</p>
</header>
<div class="caps">{cap_html}</div>
<section id="features">
  <h2>Equipment notes</h2>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>Tool configurations</h2>
  <div class="models">{models}</div>
  {spec_table('background:#e4f3f3;border-bottom:1px solid #b7d9d8', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Host recipe</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
  </div>
</section>
<section id="about">
  <h2>The company</h2>
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
    w("v37.html", shell(TITLE, css, body))


def v38():
    """AbbVie — pharma purple, pipeline stages."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:17px/1.6 "Gill Sans","Gill Sans MT",Calibri,Helvetica,sans-serif;background:#fff;color:#2a2140}
img{max-width:100%;height:auto;display:block}
a{color:#6b2d7b}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:14px 28px;background:#fff}
nav img{height:26px}
.shop{background:#6b2d7b;color:#fff;padding:10px 18px;text-decoration:none}
.shop:hover{background:#4e215b;color:#fff}
.hero{padding:64px 8vw;display:grid;grid-template-columns:1fr 1fr;gap:32px;align-items:center}
.hero h1{font-size:clamp(32px,5vw,50px);font-weight:400}
.pipe{display:grid;grid-template-columns:repeat(4,1fr);gap:0;margin:32px 8vw;border:1px solid #e6dceb}
.pipe article{padding:20px;border-right:1px solid #e6dceb}
.pipe b{display:block;color:#6b2d7b;font-size:13px;letter-spacing:.1em}
section{padding:48px 8vw}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:16px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
pre{background:#f7f2f9;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace}
.os img{height:42px;margin-right:8px}
footer{padding:40px 8vw 64px;background:#f7f2f9}
@media(max-width:800px){.hero,.pipe,.feats,.models{grid-template-columns:1fr}}
"""
    pipe = [
        ("01 Discover", "All-in-one package — four instruments, one outline."),
        ("02 Connect", "RS-485. Daisy-chain any number."),
        ("03 Command", "Trapezoid move. No STEP/DIR."),
        ("04 Deploy", "Robotics, CNC, labs, class."),
    ]
    pipe_html = "".join(f"<article><b>{t}</b><p style='margin-top:8px'>{d}</p></article>" for t, d in pipe)
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <h1>Precision motion for the instruments that measure the world.</h1>
    <p style="margin-top:14px">{TAGLINE}. {INTRO}</p>
  </div>
  <img src="../transparent/one_motor_transparent_small.png" alt="">
</header>
<div class="pipe">{pipe_html}</div>
<section id="features">
  <h2>Research notes</h2>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>Study arms</h2>
  <div class="models">{models}</div>
  {spec_table('border-bottom:1px solid #e6dceb', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Method</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <img src="../automation_small.jpg" alt="" style="width:100%;max-height:340px;object-fit:cover">
</section>
<section id="about">
  <h2>Who we are</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin:12px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
</section>
<footer>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <p style="margin-top:12px"><a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v38.html", shell(TITLE, css, body))


def v39():
    """Cisco — enterprise networking, bridge metaphor, solution brief."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:15px/1.55 "Segoe UI",Tahoma,Helvetica,sans-serif;background:#fff;color:#1b1b1b}
img{max-width:100%;height:auto;display:block}
a{color:#049fd9}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 24px;background:#0d274d}
nav img{height:24px;filter:invert(1)}
.shop{background:#049fd9;color:#fff;padding:8px 14px;text-decoration:none}
.shop:hover{background:#037aa8;color:#fff}
.hero{padding:48px 6vw;background:#0d274d;color:#fff;display:grid;grid-template-columns:1.1fr .9fr;gap:28px;align-items:center}
.bridge{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;padding:24px 6vw;background:#e8f6fb}
.bridge article{background:#fff;padding:14px;border-left:4px solid #049fd9}
section{padding:40px 6vw}
.feats{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
pre{background:#0d274d;color:#d6f0fa;padding:16px;overflow:auto;font:12.5px/1.55 Consolas,monospace}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.apps img{height:150px;width:100%;object-fit:cover}
.os img{height:40px;margin-right:8px}
footer{background:#0d274d;color:#9cb4c8;padding:32px 6vw;font-size:12px}
footer a{color:#fff}
@media(max-width:800px){.hero,.bridge,.feats,.models,.apps{grid-template-columns:1fr}}
"""
    hosts = ["Raspberry Pi", "Arduino", "ESP32", "Mac / PC"]
    bridge = "".join(f"<article><h3>{h}</h3><p>One RS-485 adapter. The same commands.</p></article>" for h in hosts)
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:6])
    models = "".join(
        f"<article style='border:1px solid #d0e4ee;padding:12px'><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p>{m['torque']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <div>
    <p style="color:#049fd9;letter-spacing:.12em;text-transform:uppercase;font-size:12px">Bridge the axis</p>
    <h1>Networking for motors that need to speak.</h1>
    <p style="margin-top:12px">{TAGLINE}. {INTRO}</p>
  </div>
  <img src="../transparent/adapter_and_wire_transparent_small.png" alt="RS-485 adapter">
</header>
<div class="bridge">{bridge}</div>
<section id="features">
  <h2>Solution brief</h2>
  <div class="feats">{feats}</div>
</section>
<section id="models">
  <h2>Product family</h2>
  <div class="models">{models}</div>
  {spec_table('background:#e8f6fb;border-bottom:1px solid #c5e0ea', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Reference listing</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <div class="apps">
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
</section>
<footer>
  <p>Gearotons · Shenzhen · founded 2022. Firmware, libraries, and schematics on GitHub.</p>
  <p style="margin-top:12px"><a href="{SHOP}">Shop Now</a> · <a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v39.html", shell(TITLE, css, body))


def v40():
    """Caterpillar — yellow/black, condensed, machinery lineup."""
    css = """
*{box-sizing:border-box;margin:0;padding:0}
html{scroll-behavior:smooth}
body{font:16px/1.45 Arial,Helvetica,sans-serif;background:#111;color:#f2f2f2}
img{max-width:100%;height:auto;display:block}
a{color:#111}
nav{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:12px 20px;background:#ffcd11}
nav img{height:28px}
.shop{background:#111;color:#ffcd11;padding:10px 16px;text-decoration:none;font-weight:800;letter-spacing:.06em;text-transform:uppercase}
.shop:hover{background:#333;color:#ffcd11}
.hero{padding:48px 6vw;border-bottom:16px solid #ffcd11}
.hero h1{font-size:clamp(40px,8vw,84px);font-weight:900;text-transform:uppercase;letter-spacing:-.03em;line-height:.9;color:#ffcd11}
section{padding:40px 6vw}
h2{color:#ffcd11;text-transform:uppercase;letter-spacing:.04em}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.feats article{background:#1a1a1a;padding:14px;border-left:8px solid #ffcd11}
.lineup{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.lineup article{background:#ffcd11;color:#111;padding:14px}
pre{background:#000;color:#ffcd11;padding:18px;overflow:auto;font:13px/1.55 ui-monospace,Menlo,monospace;border:4px solid #ffcd11}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.apps img{height:170px;width:100%;object-fit:cover;filter:contrast(1.1)}
.os img{height:42px;background:#fff;padding:3px;margin-right:8px}
footer{background:#ffcd11;color:#111;padding:36px 6vw}
@media(max-width:800px){.feats,.lineup,.apps{grid-template-columns:1fr}}
"""
    feats = "".join(f"<article><h3>{t}</h3><p>{d}</p></article>" for t, d in FEATURES[:8])
    models = "".join(
        f"<article><h3>{m['name']}</h3><img src='{m['dim']}' alt=''><p><b>{m['torque']}</b> · {m['weight']}</p></article>"
        for m in MODELS
    )
    body = f"""
<nav>
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a class="shop" href="{SHOP}">Shop</a>
</nav>
<header class="hero" id="top">
  <p style="letter-spacing:.2em;text-transform:uppercase;font-size:12px">Built for it</p>
  <h1>M17<br>does the work.</h1>
  <p style="max-width:32em;margin-top:16px">{TAGLINE}. Motor, driver, controller, encoder. No extra cabinet.</p>
</header>
<section id="features">
  <h2>Spec the job</h2>
  <p>{INTRO}</p>
  <div class="feats" style="margin-top:16px">{feats}</div>
</section>
<section id="models">
  <h2>The lineup</h2>
  <div class="lineup">{models}</div>
  {spec_table('background:#ffcd11;color:#111', 'margin-top:16px')}
</section>
<section id="start">
  <h2>Start the engine</h2>
  <p>{INTRO2}</p>
  <pre style="margin-top:12px"><code>{esc(CODE)}</code></pre>
  <p style="margin-top:8px">{INSTALL} · {ARDUINO}</p>
</section>
<section>
  <h2>On site</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="">
    <img src="../automation_small.jpg" alt="">
    <img src="../test_rack_small.jpg" alt="">
  </div>
</section>
<section id="about">
  <h2>The company</h2>
  <p>{COMPANY}</p>
  <p class="os" style="margin:12px 0">
    <img src="../Open-source-hardware-logo.svg.png" alt="">
    <img src="../Open_Source_Initiative.svg.png" alt="">
  </p>
</section>
<footer>
  <a class="shop" href="{SHOP}">Shop Now</a>
  <p style="margin-top:12px"><a href="{DOCS}">Documentation</a> · <a href="{GITHUB}">GitHub</a></p>
</footer>
"""
    w("v40.html", shell(TITLE, css, body))


if __name__ == "__main__":
    v31(); v32(); v33(); v34(); v35()
    v36(); v37(); v38(); v39(); v40()
