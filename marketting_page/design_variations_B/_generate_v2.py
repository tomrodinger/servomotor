#!/usr/bin/env python3
"""
Design Exercise B — pass 2: STRUCTURALLY distinct pages.
Each of v1–v40 has unique body HTML architecture (not a shared skeleton + theme).
"""
from pathlib import Path
from html import escape as esc

OUT = Path(__file__).resolve().parent
STORE = "https://gearotons.com/store"
DOCS = "https://gearotons.com"
GH = "https://github.com/tomrodinger/servomotor"
TITLE = "Gearotons — M17 Series Servomotors"

CODE = """import servomotor
from servomotor import M3

servomotor.open_serial_port()
motor = M3("X", time_unit="seconds",
                position_unit="degrees")
motor.system_reset()
motor.enable_mosfets()
motor.trapezoid_move(360, 2)  # one revolution in 2 s"""

SPECS = [
    ("Operating Voltage", "12–24V", "12–24V", "12–24V"),
    ("Rated Torque", "0.65 N·m", "0.55 N·m", "0.42 N·m"),
    ("Maximum Speed", "560 RPM", "560 RPM", "560 RPM"),
    ("Maximum Current", "1.1 A", "1.1 A", "1.1 A"),
    ("Rated Power", "38 W", "32 W", "25 W"),
    ("Height", "59.8 mm", "48.6 mm", "41.6 mm"),
    ("Footprint", "42.2 × 42.2 mm", "42.2 × 42.2 mm", "42.2 × 42.2 mm"),
    ("Weight", "470 g", "360 g", "285 g"),
]

MODELS = [
    ("M17-60", "0.65 N·m", "38 W", "470 g", "59.8 mm", "Highest torque"),
    ("M17-48", "0.55 N·m", "32 W", "360 g", "48.6 mm", "Balanced pick"),
    ("M17-40", "0.42 N·m", "25 W", "285 g", "41.6 mm", "Compact"),
]

FEATURES = [
    ("All-in-one package", "Motor + driver + motion controller + encoder in one body."),
    ("Daisy-chain RS-485", "Any number of motors on one bus from a single connection."),
    ("High-level commands", "Trapezoid move and enable — no timing-critical STEP/DIR."),
    ("NEMA 17 form factor", "Nearly the size of a comparable stepper; standard mounting."),
    ("Closed-loop 32 kHz", "Built-in encoder + PID for precise position control."),
    ("12–24 V protection", "Wide range with over-current, voltage, and temperature guards."),
    ("Cross-platform", "Raspberry Pi, Arduino, ESP32, Mac, and PC."),
    ("Open documentation", "Tutorials and AI-friendly docs to ship faster."),
]


def specs_table(cls="specs"):
    rows = "".join(
        f"<tr><th scope='row'>{esc(p)}</th><td>{esc(a)}</td><td>{esc(b)}</td><td>{esc(c)}</td></tr>"
        for p, a, b, c in SPECS
    )
    return (
        f"<table class='{cls}'><thead><tr>"
        f"<th>Parameter</th><th>M17-60</th><th>M17-48</th><th>M17-40</th>"
        f"</tr></thead><tbody>{rows}</tbody></table>"
    )


def code_pre(cls="code"):
    return f"<pre class='{cls}'><code>{esc(CODE)}</code></pre>"


def base_css(extra=""):
    return f"""
*,*::before,*::after{{box-sizing:border-box}}
html{{scroll-behavior:smooth;max-width:100%;overflow-x:hidden}}
body{{margin:0;line-height:1.5;-webkit-font-smoothing:antialiased;overflow-x:hidden}}
img{{max-width:100%;height:auto;display:block}}
a{{color:inherit}}
table{{border-collapse:collapse;width:100%}}
pre{{overflow:auto;max-width:100%}}
{extra}
"""


def doc(css, body, js=""):
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{esc(TITLE)}</title>
<style>
{base_css(css)}
</style>
</head>
<body>
{body}
{js}
</body>
</html>
"""


LABELS = {}


def register(n, label, html):
    LABELS[n] = label
    (OUT / f"v{n}.html").write_text(html, encoding="utf-8")
    print(f"wrote v{n}.html — {label}")


# =============================================================================
# ROUND 1 — original structural directions
# =============================================================================

def build_v1():
    """Precision Lab: asymmetric 50/50 hero, sticky side rail features."""
    css = """
body{font-family:Avenir,Helvetica Neue,Helvetica,Arial,sans-serif;background:#f4f6f2;color:#141714}
.top{position:sticky;top:0;z-index:30;background:rgba(244,246,242,.96);border-bottom:1px solid #d5dbd0;
  display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw}
.top img{height:32px}.top nav{display:flex;gap:1.1rem;align-items:center;font-size:.88rem}
.top a{text-decoration:none}.cta{background:#7AB648;color:#fff;padding:.5rem 1rem;border-radius:4px;font-weight:700;text-decoration:none}
.cta:hover{background:#68993c}
.hero{display:grid;grid-template-columns:1.05fr .95fr;min-height:78vh}
.hero-copy{padding:7vh 4vw;display:flex;flex-direction:column;justify-content:center;border-right:1px solid #d5dbd0}
.hero-copy h1{font-size:clamp(2rem,4vw,3.2rem);letter-spacing:-.03em;margin:0 0 1rem;line-height:1.08}
.hero-copy p{font-size:1.08rem;max-width:36rem;opacity:.9}
.hero-cta{display:flex;gap:.7rem;margin-top:1.3rem;flex-wrap:wrap}
.ghost{border:1.5px solid #141714;padding:.5rem 1rem;border-radius:4px;text-decoration:none;font-weight:600}
.hero-vis{background:#fff;display:flex;align-items:center;justify-content:center;padding:2rem}
.hero-vis img{max-height:68vh;width:auto;max-width:100%}
.band{padding:3.5rem 3vw;display:grid;grid-template-columns:260px 1fr;gap:2rem}
.rail{position:sticky;top:4.5rem;align-self:start}
.rail h2{margin:0 0 .8rem;font-size:.85rem;letter-spacing:.14em;text-transform:uppercase;color:#5a6b52}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:1rem}
.feat{background:#fff;border:1px solid #d5dbd0;padding:1.1rem}
.feat h3{margin:0 0 .35rem;font-size:1rem}.feat p{margin:0;font-size:.92rem;opacity:.88}
.models{padding:3rem 3vw;background:#fff;border-top:1px solid #d5dbd0}
.mgrid{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem;margin:1.2rem 0}
.model{border:1px solid #d5dbd0;padding:1.1rem}.model strong{font-size:1.25rem;color:#3d6b1f}
table.specs th,table.specs td{border-bottom:1px solid #d5dbd0;padding:.6rem .45rem;text-align:left;font-size:.9rem}
.dev{display:grid;grid-template-columns:1fr 1fr;gap:2rem;padding:3rem 3vw}
pre.code{background:#141714;color:#d8f0c4;padding:1.1rem;font:.84rem/1.5 Menlo,Consolas,monospace;margin:0}
.apps{padding:2.5rem 3vw;display:grid;grid-template-columns:repeat(4,1fr);gap:.75rem}
.apps figure{margin:0}.apps img{width:100%;aspect-ratio:1;object-fit:cover}
.apps figcaption{font-size:.8rem;margin-top:.3rem}
footer{background:#141714;color:#e8efe6;padding:2.8rem 3vw}
footer .row{display:flex;gap:.7rem;flex-wrap:wrap;margin:1rem 0}
footer a.ghost{color:#e8efe6;border-color:#e8efe6;text-decoration:none}
.muted{opacity:.72;font-size:.9rem}
.dims{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem;margin-top:1.2rem}
@media(max-width:900px){
  .hero,.band,.dev,.mgrid,.dims{grid-template-columns:1fr}
  .apps{grid-template-columns:1fr 1fr}.feats{grid-template-columns:1fr}.rail{position:static}
}"""
    body = f"""
<header class="top">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <nav>
    <a href="#features">Features</a><a href="#models">Models</a><a href="#start">Start</a>
    <a class="cta" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <div class="hero-copy">
    <p class="muted" style="letter-spacing:.12em;text-transform:uppercase;font-size:.72rem;font-weight:700">M17 Series · Laboratory precision</p>
    <h1>All-in-one motion control without the stack of boxes</h1>
    <p>Motor, driver, motion controller, and encoder in one NEMA 17 package. Daisy-chain any number over RS-485 from a single connection.</p>
    <div class="hero-cta">
      <a class="cta" href="{STORE}">Shop Now</a>
      <a class="ghost" href="{DOCS}">Documentation</a>
    </div>
  </div>
  <div class="hero-vis"><img src="../M17_series_overview.jpg" alt="M17 Series three models"></div>
</section>
<section class="band" id="features">
  <div class="rail">
    <h2>Key facts</h2>
    <p>Closed-loop PID at 32 kHz. 12–24 V. Max 560 RPM. Three torque tiers. Open-source firmware and libraries.</p>
    <p style="margin-top:1rem"><a class="cta" href="{STORE}">Shop Now</a></p>
  </div>
  <div class="feats">
    {"".join(f"<article class='feat'><h3>{esc(t)}</h3><p>{esc(d)}</p></article>" for t,d in FEATURES[:6])}
  </div>
</section>
<section class="models" id="models">
  <h2 style="margin:0 0 .4rem">Three models · one protocol</h2>
  <p class="muted">Pick torque and budget — same software, same mounting.</p>
  <div class="mgrid">
    {"".join(f"<div class='model'><h3>{m[0]}</h3><strong>{m[1]}</strong><p>{m[2]} · {m[3]} · {m[5]}</p></div>" for m in MODELS)}
  </div>
  {specs_table()}
  <div class="dims">
    <img src="../M17-60_dimensions.png" alt="M17-60 dimensions" loading="lazy">
    <img src="../M17-48_dimensions.png" alt="M17-48 dimensions" loading="lazy">
    <img src="../M17-40_dimensions.png" alt="M17-40 dimensions" loading="lazy">
  </div>
</section>
<section class="dev" id="start">
  <div>
    <h2>Minutes to first move</h2>
    <p><code>pip3 install servomotor</code> · Arduino Library Manager: “Servomotor”</p>
    <img src="../adapter_and_wire_small.jpg" alt="RS485 adapter" style="margin-top:1rem;max-width:320px;border:1px solid #d5dbd0">
  </div>
  {code_pre()}
</section>
<section class="apps" id="apps">
  <figure><img src="../robotics_small.jpg" alt="Robotics" loading="lazy"><figcaption>Robotics</figcaption></figure>
  <figure><img src="../automation_small.jpg" alt="CNC" loading="lazy"><figcaption>CNC</figcaption></figure>
  <figure><img src="../test_rack_small.jpg" alt="Test" loading="lazy"><figcaption>Automated testing</figcaption></figure>
  <figure><img src="../kit_with_three_motors_small.jpg" alt="Education" loading="lazy"><figcaption>Education</figcaption></figure>
</section>
<footer id="about">
  <h2>Open source hardware &amp; software</h2>
  <p>Gearotons — founded Shenzhen 2022 by a Canadian entrepreneur. Firmware, libraries, and schematics on GitHub.</p>
  <div class="row">
    <img src="../Open_Source_Initiative.svg.png" alt="OSI" style="height:40px;width:auto">
    <img src="../Open-source-hardware-logo.svg.png" alt="OSHW" style="height:40px;width:auto">
  </div>
  <div class="row">
    <a class="cta" href="{STORE}">Shop Now</a>
    <a class="ghost" href="{DOCS}">Documentation</a>
    <a class="ghost" href="{GH}">GitHub</a>
  </div>
  <p class="muted">Affordable and Simple All-in-One Motion Control</p>
</footer>"""
    register(1, "Precision Lab", doc(css, body))


def build_v2():
    """Dark Command: full-viewport stage + HORIZONTAL SCROLL feature strip."""
    css = """
body{font-family:ui-sans-serif,system-ui,Segoe UI,Helvetica,Arial,sans-serif;background:#070908;color:#dce8d8}
.bar{display:flex;justify-content:space-between;align-items:center;padding:.75rem 3vw;border-bottom:1px solid #1e2a20;
  position:sticky;top:0;background:rgba(7,9,8,.94);z-index:20}
.brand{display:flex;gap:.55rem;align-items:center;font-weight:700;letter-spacing:.08em;text-transform:uppercase;font-size:.78rem;text-decoration:none}
.brand img{height:26px}.bar nav{display:flex;gap:1rem;font-size:.84rem}
.bar a{text-decoration:none;color:#9fb89a}.bar a.buy{color:#0b0d0c;background:#8fd45a;padding:.42rem .85rem;font-weight:700}
.stage{min-height:90vh;display:grid;place-items:center;text-align:center;padding:3.5rem 4vw;
  background:radial-gradient(ellipse at 50% 28%,rgba(143,212,90,.16),transparent 55%)}
.stage img{max-height:40vh;margin:0 auto 1.3rem;filter:drop-shadow(0 28px 55px rgba(0,0,0,.55))}
.stage h1{font-size:clamp(2.1rem,5.5vw,3.8rem);margin:0;letter-spacing:-.03em}
.tele{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:.78rem;color:#8fd45a;margin:1rem 0;letter-spacing:.06em}
.stage p{max-width:34rem;margin:0 auto 1.4rem;opacity:.86}
.btns{display:flex;gap:.65rem;justify-content:center;flex-wrap:wrap}
.btns a{text-decoration:none;padding:.65rem 1.15rem;font-weight:700}
.btns .p{background:#8fd45a;color:#070908}.btns .g{border:1px solid #8fd45a;color:#8fd45a}
.scroll-label{padding:1rem 3vw;font-family:ui-monospace,Menlo,monospace;font-size:.72rem;color:#8fd45a;letter-spacing:.12em;text-transform:uppercase}
.strip{display:flex;gap:1rem;overflow-x:auto;padding:0 3vw 2rem;scroll-snap-type:x mandatory;-webkit-overflow-scrolling:touch}
.strip article{min-width:min(270px,78vw);scroll-snap-align:start;background:#101512;border:1px solid #243028;padding:1.15rem;flex-shrink:0}
.strip h3{margin:0 0 .35rem;color:#8fd45a;font-size:.98rem}
.mid{display:grid;grid-template-columns:1.1fr .9fr;border-top:1px solid #1e2a20}
.mid>*{padding:2.8rem 3vw}.mid-left{border-right:1px solid #1e2a20}
.mid h2{margin-top:0;color:#8fd45a;font-size:.92rem;letter-spacing:.1em;text-transform:uppercase}
table.specs th,table.specs td{border-bottom:1px solid #243028;padding:.5rem .35rem;text-align:left;font-size:.82rem;font-family:ui-monospace,Menlo,monospace}
pre.code{background:#000;border:1px solid #243028;color:#b8f08a;padding:1rem;font-size:.8rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1px;background:#1e2a20}
.apps figure{margin:0;background:#0c100e;position:relative}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover;opacity:.88}
.apps figcaption{position:absolute;left:.75rem;bottom:.55rem;font-size:.84rem;font-weight:700}
footer{padding:2.8rem 3vw;border-top:1px solid #1e2a20}
footer .row{display:flex;gap:.65rem;flex-wrap:wrap;margin:1rem 0}
footer a{text-decoration:none;padding:.55rem .95rem;border:1px solid #8fd45a;color:#8fd45a}
footer a.p{background:#8fd45a;color:#070908;font-weight:700}
@media(max-width:800px){.mid,.apps{grid-template-columns:1fr}.mid-left{border-right:0;border-bottom:1px solid #1e2a20}}
"""
    body = f"""
<header class="bar">
  <a class="brand" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav><a href="#sys">System</a><a href="#models">Models</a><a href="#code">Code</a>
    <a class="buy" href="{STORE}">Shop Now</a></nav>
</header>
<section class="stage" id="top">
  <div>
    <img src="../transparent/one_motor_transparent_small.png" alt="M17 motor">
    <div class="tele">RS485 · 32kHz PID · 12–24V · NEMA17 · OPEN SOURCE</div>
    <h1>Command every axis from one bus</h1>
    <p>All-in-one servomotors for machines that need closed-loop control without a cabinet full of drives.</p>
    <div class="btns"><a class="p" href="{STORE}">Shop Now</a><a class="g" href="{DOCS}">Documentation</a></div>
  </div>
</section>
<div class="scroll-label" id="sys">// capabilities — scroll →</div>
<div class="strip">
  <article><h3>01 · Integrate</h3><p>Motor, driver, controller, encoder — one body.</p></article>
  <article><h3>02 · Daisy-chain</h3><p>Any number of motors on one RS-485 link.</p></article>
  <article><h3>03 · High-level move</h3><p>Trapezoid profiles; no STEP/DIR timing.</p></article>
  <article><h3>04 · Protect</h3><p>Current, voltage, temperature guards onboard.</p></article>
  <article><h3>05 · Cross-platform</h3><p>Pi, Arduino, ESP32, Mac, PC.</p></article>
  <article><h3>06 · Documented</h3><p>Tutorials + AI-friendly docs.</p></article>
</div>
<section class="mid" id="models">
  <div class="mid-left">
    <h2>Telemetry / models</h2>
    <p>M17-60 · 0.65 N·m · 38 W · 470 g<br>M17-48 · 0.55 N·m · 32 W · 360 g<br>M17-40 · 0.42 N·m · 25 W · 285 g</p>
    {specs_table()}
  </div>
  <div id="code">
    <h2>First motion</h2>
    {code_pre()}
    <p style="font-size:.88rem;opacity:.75;margin-top:1rem">pip3 install servomotor</p>
    <img src="../transparent/adapter_and_wire_transparent_small.png" alt="Adapter" style="margin-top:1rem;max-height:150px">
  </div>
</section>
<section class="apps" id="apps">
  <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
  <figure><img src="../automation_small.jpg" alt=""><figcaption>CNC / automation</figcaption></figure>
  <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Test equipment</figcaption></figure>
</section>
<footer id="about">
  <h2>Open source · Gearotons · Shenzhen 2022</h2>
  <p>Firmware, libraries, schematics: public. Built for makers, educators, and industry.</p>
  <div class="row"><a class="p" href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></div>
</footer>"""
    register(2, "Dark Command", doc(css, body))


def build_v3():
    """Editorial Serif: magazine masthead, multi-column, full-bleed breaks, pull quotes."""
    css = """
body{font-family:Georgia,Times New Roman,serif;background:#f7f3eb;color:#1c1915}
.mast{border-bottom:3px double #1c1915;padding:1.1rem 5vw .7rem;text-align:center}
.mast .logo{height:40px;margin:0 auto .45rem}
.mast h1{font-weight:400;font-size:clamp(1.7rem,3.8vw,2.5rem);margin:0}
.mast .sub{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.72rem;letter-spacing:.18em;text-transform:uppercase;margin:.35rem 0 0}
.nav{display:flex;justify-content:center;gap:1.4rem;padding:.65rem 5vw;border-bottom:1px solid #cfc6b6;
  font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.78rem;text-transform:uppercase;letter-spacing:.08em;position:sticky;top:0;background:#f7f3eb;z-index:20}
.nav a{text-decoration:none}.nav a.shop{font-weight:800;border-bottom:2px solid #7AB648}
.lede{display:grid;grid-template-columns:1.2fr .8fr;gap:2rem;padding:2.4rem 5vw;border-bottom:1px solid #cfc6b6}
.lede h2{font-weight:400;font-size:clamp(1.7rem,3.4vw,2.7rem);line-height:1.15;margin:0 0 1rem}
.drop::first-letter{float:left;font-size:3.6rem;line-height:1;padding-right:.35rem;font-weight:700}
.pull{font-size:1.4rem;font-style:italic;border-left:3px solid #7AB648;padding:.45rem 0 .45rem 1rem;margin:1.4rem 0;max-width:28rem}
.cols{column-count:3;column-gap:1.7rem;padding:2.4rem 5vw;border-bottom:1px solid #cfc6b6}
.cols h3{column-span:all;font-weight:400;border-bottom:1px solid #1c1915;padding-bottom:.35rem;margin:0 0 1rem}
.break img{width:100%;max-height:52vh;object-fit:cover}
.caption{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.74rem;padding:.45rem 5vw 1.3rem;opacity:.72;border-bottom:1px solid #cfc6b6}
.models{padding:2.4rem 5vw;display:grid;grid-template-columns:1fr 1.4fr;gap:2rem}
table.specs{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.84rem}
table.specs th,table.specs td{border-bottom:1px solid #cfc6b6;padding:.5rem;text-align:left}
.code-wrap{background:#1c1915;color:#f7f3eb;padding:2.4rem 5vw}
pre.code{font-family:Menlo,Consolas,monospace;font-size:.84rem;margin:0}
.gallery{display:grid;grid-template-columns:repeat(3,1fr)}
.gallery figure{margin:0;border-right:1px solid #cfc6b6;border-bottom:1px solid #cfc6b6}
.gallery img{width:100%;aspect-ratio:4/3;object-fit:cover}
.gallery figcaption{padding:.45rem .7rem;font-size:.84rem;font-style:italic}
footer{padding:2.4rem 5vw;text-align:center;border-top:3px double #1c1915}
footer .btns{display:flex;gap:.7rem;justify-content:center;flex-wrap:wrap;margin:1rem 0;font-family:Avenir,Helvetica,Arial,sans-serif}
footer a{text-decoration:none;padding:.65rem 1.1rem;border:1px solid #1c1915}
footer a.p{background:#1c1915;color:#f7f3eb}
@media(max-width:900px){.lede,.models{grid-template-columns:1fr}.cols{column-count:1}.gallery{grid-template-columns:1fr}}
"""
    body = f"""
<header class="mast">
  <img class="logo" src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons">
  <h1>The M17 Series</h1>
  <p class="sub">Affordable and Simple All-in-One Motion Control · A product journal</p>
</header>
<nav class="nav">
  <a href="#story">Story</a><a href="#features">Features</a><a href="#models">Models</a><a href="#code">Code</a>
  <a class="shop" href="{STORE}">Shop Now</a>
</nav>
<section class="lede" id="story">
  <div>
    <h2>Motion control, edited down to one package</h2>
    <p class="drop">The M17 Series Servomotors integrate a motor, motor driver, motion controller, and encoder in a single compact body. With RS-485, you daisy-chain any number of axes from one connection point.</p>
    <p class="pull">“High-level commands like trapezoid moves — not a wire forest of STEP and DIR.”</p>
    <p>Three models share control characteristics while offering torque from 0.42 to 0.65 N·m. Ideal for robotics, CNC, automated testing, scientific instruments, and education.</p>
  </div>
  <div><img src="../one_motor_small.jpg" alt="Single M17 motor"></div>
</section>
<section class="cols" id="features">
  <h3>In this issue — capabilities</h3>
  <p><strong>Closed loop at 32 kHz.</strong> Built-in encoder and PID keep position honest without an external drive.</p>
  <p><strong>NEMA 17 footprint.</strong> Nearly the size of a comparable stepper, standard mounting, no awkward protrusions.</p>
  <p><strong>12–24 V input</strong> with over-current, over-voltage, and over-temperature protection onboard.</p>
  <p><strong>560 RPM max</strong> with torque-to-weight on par with an equivalent stepper — more efficient in closed loop.</p>
  <p><strong>Open documentation</strong> and AI-friendly materials so your tools can help you ship faster.</p>
  <p><strong>Open source</strong> firmware, libraries, and schematics for makers and engineers alike.</p>
</section>
<figure class="break"><img src="../kit_with_three_motors.jpg" alt="Full kit flat lay"></figure>
<p class="caption">Kit view: three motors, adapter PCB, cables, knobs — a complete path from unboxing to motion.</p>
<section class="models" id="models">
  <div>
    <h3>Choose your torque</h3>
    <p><strong>M17-60</strong> — 0.65 N·m · 38 W · 470 g<br>
    <strong>M17-48</strong> — 0.55 N·m · 32 W · 360 g<br>
    <strong>M17-40</strong> — 0.42 N·m · 25 W · 285 g</p>
    <p style="font-family:Avenir,Helvetica,Arial,sans-serif"><a href="{STORE}" style="font-weight:700">Shop the series →</a></p>
  </div>
  {specs_table()}
</section>
<section class="code-wrap" id="code">
  <h3>From install to revolution</h3>
  {code_pre()}
  <p style="font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.88rem;opacity:.8;margin:1rem 0 0">pip3 install servomotor</p>
</section>
<section class="gallery" id="apps">
  <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
  <figure><img src="../automation_small.jpg" alt=""><figcaption>CNC machines</figcaption></figure>
  <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Automated testing</figcaption></figure>
</section>
<footer id="about">
  <p>Gearotons is a startup making precision motion accessible — founded in Shenzhen in 2022 by a Canadian entrepreneur.</p>
  <div class="btns">
    <a class="p" href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a>
  </div>
  <div style="display:flex;gap:1rem;justify-content:center;margin-top:1rem">
    <img src="../Open_Source_Initiative.svg.png" alt="OSI" style="height:34px">
    <img src="../Open-source-hardware-logo.svg.png" alt="OSHW" style="height:34px">
  </div>
</footer>"""
    register(3, "Editorial Serif", doc(css, body))


def build_v4():
    """Soft Consumer: bento/masonry homepage card grid."""
    css = """
body{font-family:Avenir Next,Avenir,Helvetica Neue,Helvetica,Arial,sans-serif;background:#eef6e8;color:#1a2418}
.nav{position:sticky;top:0;z-index:20;background:rgba(238,246,232,.94);backdrop-filter:blur(8px);
  display:flex;justify-content:space-between;align-items:center;padding:.7rem 4vw}
.nav img{height:34px}.nav .links{display:flex;gap:.8rem;align-items:center}
.pill{display:inline-block;padding:.5rem 1.15rem;border-radius:999px;text-decoration:none;font-weight:700;font-size:.9rem}
.pill.g{background:#7AB648;color:#fff}.pill.o{background:#fff;border:1px solid #c5d8b8}
.hero-banner{padding:2rem 4vw 1rem;text-align:center}
.hero-banner h1{font-size:clamp(1.9rem,4vw,2.8rem);margin:.3rem 0;letter-spacing:-.02em}
.hero-banner p{max-width:34rem;margin:.6rem auto 1rem;opacity:.85}
.bento{display:grid;grid-template-columns:repeat(6,1fr);grid-auto-rows:minmax(120px,auto);gap:1rem;padding:1rem 4vw 3rem}
.card{background:#fff;border-radius:20px;padding:1.2rem;box-shadow:0 8px 28px rgba(60,90,40,.08);overflow:hidden}
.card h3{margin:.2rem 0 .4rem;font-size:1.05rem}
.card p{margin:0;font-size:.92rem;opacity:.88}
.c-hero{grid-column:span 4;grid-row:span 2;display:grid;grid-template-columns:1fr 1fr;gap:1rem;align-items:center;padding:1.5rem}
.c-hero img{max-height:280px;margin:0 auto}
.c-tall{grid-column:span 2;grid-row:span 2}
.c-wide{grid-column:span 3}
.c-mid{grid-column:span 2}
.c-full{grid-column:span 6}
.soft{background:linear-gradient(145deg,#f7fbf4,#e4f0d8)}
table.specs{font-size:.85rem}table.specs th,table.specs td{padding:.5rem;border-bottom:1px solid #e0ebd6;text-align:left}
pre.code{background:#1a2418;color:#d8f0c4;padding:1rem;border-radius:14px;font-size:.82rem;margin:0}
.apps{display:grid;grid-template-columns:repeat(4,1fr);gap:.8rem}
.apps img{border-radius:12px;width:100%;aspect-ratio:1;object-fit:cover}
footer{padding:2.5rem 4vw;text-align:center}
footer .row{display:flex;gap:.7rem;justify-content:center;flex-wrap:wrap;margin:1rem 0}
@media(max-width:900px){
  .bento{grid-template-columns:1fr 1fr}.c-hero,.c-tall,.c-wide,.c-mid,.c-full{grid-column:span 2;grid-row:auto}
  .c-hero{grid-template-columns:1fr}.apps{grid-template-columns:1fr 1fr}
}
@media(max-width:520px){.bento{grid-template-columns:1fr}.c-hero,.c-tall,.c-wide,.c-mid,.c-full{grid-column:span 1}}
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div class="links">
    <a class="pill o" href="#features">Explore</a>
    <a class="pill g" href="{STORE}">Shop Now</a>
  </div>
</header>
<div class="hero-banner" id="top">
  <p style="font-size:.8rem;letter-spacing:.12em;text-transform:uppercase;color:#5a7a48;font-weight:700">M17 Series</p>
  <h1>Motion control that feels simple</h1>
  <p>Affordable and Simple All-in-One Motion Control — motor, driver, controller, and encoder together.</p>
  <a class="pill g" href="{STORE}">Shop Now</a>
  <a class="pill o" href="{DOCS}" style="margin-left:.4rem">Documentation</a>
</div>
<div class="bento" id="features">
  <div class="card c-hero soft">
    <div>
      <h3>See the family</h3>
      <p>Three torque tiers, one protocol. Daisy-chain over RS-485 from a single connection on Pi, Arduino, ESP32, Mac, or PC.</p>
    </div>
    <img src="../transparent/M17_series_overview_transparent_small.png" alt="M17 series">
  </div>
  <div class="card c-tall">
    <img src="../transparent/one_motor_transparent_small.png" alt="Motor" style="max-height:200px;margin:0 auto">
    <h3>Integrated package</h3>
    <p>No external drive cabinet for each axis. Closed-loop PID at 32 kHz.</p>
  </div>
  <div class="card c-mid"><h3>NEMA 17 size</h3><p>Standard mounting. Nearly the same size as a comparable stepper — no protrusions.</p></div>
  <div class="card c-mid"><h3>High-level moves</h3><p>Enable MOSFETs, trapezoid move — skip STEP/DIR timing headaches.</p></div>
  <div class="card c-wide soft"><h3>Protected &amp; efficient</h3><p>Over-current, over-voltage, over-temperature. More power efficient in closed loop vs a conventional stepper. Max 560 RPM.</p></div>
  <div class="card c-wide"><h3>Open source</h3><p>Firmware, libraries, and schematics on GitHub. Gearotons — Shenzhen 2022, Canadian founder.</p>
    <div style="display:flex;gap:.6rem;margin-top:.8rem">
      <img src="../Open_Source_Initiative.svg.png" alt="OSI" style="height:32px">
      <img src="../Open-source-hardware-logo.svg.png" alt="OSHW" style="height:32px">
    </div>
  </div>
  <div class="card c-full" id="models">
    <h3>Models &amp; specs</h3>
    <p style="margin-bottom:1rem">M17-60 (0.65 N·m) · M17-48 (0.55 N·m) · M17-40 (0.42 N·m) — all 12–24 V, footprint 42.2 × 42.2 mm</p>
    {specs_table()}
  </div>
  <div class="card c-full" id="start">
    <h3>Get started with Python</h3>
    <p style="margin-bottom:.8rem"><code>pip3 install servomotor</code></p>
    {code_pre()}
  </div>
  <div class="card c-full" id="apps">
    <h3>Applications</h3>
    <div class="apps" style="margin-top:.8rem">
      <figure style="margin:0"><img src="../robotics_small.jpg" alt=""><figcaption style="font-size:.8rem;margin-top:.3rem">Robotics</figcaption></figure>
      <figure style="margin:0"><img src="../automation_small.jpg" alt=""><figcaption style="font-size:.8rem;margin-top:.3rem">Automation</figcaption></figure>
      <figure style="margin:0"><img src="../test_rack_small.jpg" alt=""><figcaption style="font-size:.8rem;margin-top:.3rem">Test</figcaption></figure>
      <figure style="margin:0"><img src="../kit_with_three_motors_small.jpg" alt=""><figcaption style="font-size:.8rem;margin-top:.3rem">Education</figcaption></figure>
    </div>
  </div>
</div>
<footer>
  <p>Gearotons · Affordable and Simple All-in-One Motion Control</p>
  <div class="row">
    <a class="pill g" href="{STORE}">Shop Now</a>
    <a class="pill o" href="{DOCS}">Documentation</a>
    <a class="pill o" href="{GH}">GitHub</a>
  </div>
</footer>"""
    register(4, "Soft Consumer", doc(css, body))


def build_v5():
    """Blueprint Grid: blueprint bg, two-pane technical drawing + callouts."""
    css = """
body{font-family:ui-monospace,Menlo,Consolas,monospace;background:#0b1a2e;color:#c5e0ff;
  background-image:linear-gradient(rgba(80,140,220,.08) 1px,transparent 1px),
  linear-gradient(90deg,rgba(80,140,220,.08) 1px,transparent 1px);background-size:24px 24px}
.top{position:sticky;top:0;z-index:20;background:rgba(11,26,46,.95);border-bottom:1px solid #2a4a72;
  display:flex;justify-content:space-between;align-items:center;padding:.6rem 3vw;font-size:.8rem}
.top .brand{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700;letter-spacing:.1em;text-transform:uppercase}
.top img{height:28px}.top nav{display:flex;gap:1rem;align-items:center}
.top a{text-decoration:none;color:#9ec4ef}.top a.shop{background:#7AB648;color:#0b1a2e;padding:.4rem .85rem;font-weight:700}
.pane{display:grid;grid-template-columns:1.15fr .85fr;min-height:85vh;border-bottom:1px solid #2a4a72}
.draw{padding:2rem 3vw;display:flex;flex-direction:column;justify-content:center;border-right:1px solid #2a4a72;position:relative}
.draw img{max-height:55vh;margin:1rem auto;background:rgba(255,255,255,.04);padding:1rem;border:1px dashed #3a6a9a}
.callouts{padding:2rem 3vw;display:flex;flex-direction:column;justify-content:center;gap:1rem}
.callout{border-left:3px solid #7AB648;padding:.4rem 0 .4rem .9rem}
.callout h3{margin:0 0 .25rem;font-size:.85rem;color:#7AB648;letter-spacing:.08em;text-transform:uppercase}
.callout p{margin:0;font-size:.88rem;font-family:Avenir,Helvetica,Arial,sans-serif;line-height:1.45}
.hero-title{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:clamp(1.6rem,3vw,2.4rem);margin:0 0 .6rem;color:#fff}
.sheet{padding:2.5rem 3vw;font-family:Avenir,Helvetica,Arial,sans-serif}
.sheet h2{font-family:ui-monospace,Menlo,monospace;font-size:.85rem;letter-spacing:.12em;text-transform:uppercase;color:#7AB648}
table.specs{font-family:ui-monospace,Menlo,monospace;font-size:.8rem;margin-top:1rem;background:rgba(0,0,0,.25)}
table.specs th,table.specs td{border:1px solid #2a4a72;padding:.5rem;text-align:left}
.dimrow{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem;margin:1.5rem 0}
.dimrow img{background:#fff;padding:.5rem}
.two{display:grid;grid-template-columns:1fr 1fr;gap:2rem;padding:0 3vw 2.5rem}
pre.code{background:#061018;border:1px solid #2a4a72;color:#9fef9f;padding:1rem;font-size:.8rem}
.apps{display:grid;grid-template-columns:repeat(4,1fr);gap:1px;background:#2a4a72;border-top:1px solid #2a4a72}
.apps figure{margin:0;background:#0b1a2e}.apps img{width:100%;aspect-ratio:1;object-fit:cover;opacity:.9}
.apps figcaption{padding:.4rem .6rem;font-size:.75rem}
footer{padding:2.5rem 3vw;border-top:1px solid #2a4a72;font-family:Avenir,Helvetica,Arial,sans-serif}
footer a{color:#7AB648;margin-right:1rem}
@media(max-width:900px){.pane,.two,.dimrow{grid-template-columns:1fr}.draw{border-right:0;border-bottom:1px solid #2a4a72}.apps{grid-template-columns:1fr 1fr}}
"""
    body = f"""
<header class="top">
  <a class="brand" href="#top"><img src="../Gearotons_Logo.png" alt="">GEAROTONS · DWG</a>
  <nav>
    <a href="#blueprint">Blueprint</a><a href="#models">Specs</a><a href="#start">Code</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="pane" id="top">
  <div class="draw" id="blueprint">
    <p style="font-size:.72rem;letter-spacing:.15em;color:#7AB648">SHEET 01 · M17 SERIES</p>
    <h1 class="hero-title">Technical overview: all-in-one servomotor</h1>
    <img src="../connection_diagram.jpg" alt="Connection diagram">
    <p style="font-size:.75rem;opacity:.7;text-align:center">RS-485 daisy-chain topology · single host connection</p>
  </div>
  <div class="callouts">
    <div class="callout"><h3>A · Package</h3><p>Motor + driver + motion controller + encoder integrated.</p></div>
    <div class="callout"><h3>B · Bus</h3><p>Control any number of motors over one RS-485 bus.</p></div>
    <div class="callout"><h3>C · Form</h3><p>NEMA 17 mounting, footprint 42.2 × 42.2 mm, no protrusions.</p></div>
    <div class="callout"><h3>D · Loop</h3><p>Encoder + PID at 32 kHz. Max speed 560 RPM. 12–24 V.</p></div>
    <div class="callout"><h3>E · Hosts</h3><p>Raspberry Pi, Arduino, ESP32, Mac, PC.</p></div>
    <p><a class="shop" href="{STORE}" style="background:#7AB648;color:#0b1a2e;padding:.5rem 1rem;text-decoration:none;font-weight:700;display:inline-block">Shop Now</a></p>
  </div>
</section>
<section class="sheet" id="models">
  <h2>Dimensional &amp; electrical data</h2>
  <p>M17-60 · 0.65 N·m · 38 W · 470 g · 59.8 mm height<br>
  M17-48 · 0.55 N·m · 32 W · 360 g · 48.6 mm height<br>
  M17-40 · 0.42 N·m · 25 W · 285 g · 41.6 mm height</p>
  {specs_table()}
  <div class="dimrow">
    <img src="../M17-60_dimensions.png" alt="M17-60" loading="lazy">
    <img src="../M17-48_dimensions.png" alt="M17-48" loading="lazy">
    <img src="../M17-40_dimensions.png" alt="M17-40" loading="lazy">
  </div>
</section>
<section class="two" id="start">
  <div>
    <h2 style="font-size:.85rem;letter-spacing:.1em;text-transform:uppercase;color:#7AB648">Procedure · first move</h2>
    <p style="font-family:Avenir,Helvetica,Arial,sans-serif">pip3 install servomotor · or Arduino Library Manager: “Servomotor”</p>
    <img src="../adapter_and_wire_small.jpg" alt="Adapter" style="margin-top:1rem;border:1px solid #2a4a72">
  </div>
  {code_pre()}
</section>
<section class="apps" id="apps">
  <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
  <figure><img src="../automation_small.jpg" alt=""><figcaption>CNC</figcaption></figure>
  <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Test racks</figcaption></figure>
  <figure><img src="../motor_back_small.jpg" alt=""><figcaption>Label / QR</figcaption></figure>
</section>
<footer id="about">
  <p>OPEN SOURCE · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
  <p style="opacity:.7;font-size:.9rem">Affordable and Simple All-in-One Motion Control</p>
</footer>"""
    register(5, "Blueprint Grid", doc(css, body))


def build_v6():
    """Bold Poster: huge stacked typography dominates, thick borders, minimal chrome."""
    css = """
body{font-family:Impact,Haettenschweiler,Arial Black,sans-serif;background:#fff;color:#111;letter-spacing:-.02em}
.nav{display:flex;justify-content:space-between;align-items:center;padding:.5rem 3vw;border-bottom:6px solid #111;position:sticky;top:0;background:#fff;z-index:20}
.nav img{height:36px}.nav a{font-family:Arial,Helvetica,sans-serif;font-weight:800;text-decoration:none;margin-left:1rem;font-size:.9rem}
.nav a.shop{background:#7AB648;color:#fff;padding:.45rem .9rem}
.poster{padding:2rem 3vw;border-bottom:6px solid #111}
.poster h1{font-size:clamp(3.2rem,12vw,9rem);line-height:.88;margin:0;text-transform:uppercase}
.poster .sub{font-family:Arial,Helvetica,sans-serif;font-size:1.15rem;font-weight:700;max-width:28rem;margin:1.2rem 0;letter-spacing:0}
.poster img{width:100%;max-height:50vh;object-fit:contain;border:6px solid #111;margin:1.5rem 0}
.block{border-bottom:6px solid #111;padding:2rem 3vw}
.block h2{font-size:clamp(1.8rem,5vw,3.5rem);margin:0 0 1rem;text-transform:uppercase;line-height:.95}
.feat-list{font-family:Arial,Helvetica,sans-serif;font-weight:700;font-size:1.1rem;letter-spacing:0}
.feat-list li{margin:.6rem 0;padding-left:.3rem;border-left:6px solid #7AB648}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:0;border-bottom:6px solid #111}
.models article{border-right:6px solid #111;padding:1.5rem 3vw}
.models article:last-child{border-right:0}
.models h3{font-size:clamp(1.5rem,3vw,2.2rem);margin:0}
.models p{font-family:Arial,Helvetica,sans-serif;font-weight:700;letter-spacing:0}
table.specs{font-family:Arial,Helvetica,sans-serif;font-size:.88rem;letter-spacing:0}
table.specs th,table.specs td{border:3px solid #111;padding:.55rem;text-align:left}
pre.code{font-family:Menlo,Consolas,monospace;background:#111;color:#7AB648;padding:1.2rem;font-size:.85rem;letter-spacing:0;font-weight:400}
.apps{display:grid;grid-template-columns:1fr 1fr;gap:0}
.apps figure{margin:0;border-right:6px solid #111;border-bottom:6px solid #111}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover}
.apps figcaption{font-family:Arial,Helvetica,sans-serif;font-weight:800;padding:.5rem 1rem;text-transform:uppercase}
footer{padding:2rem 3vw;background:#111;color:#fff}
footer a{color:#7AB648;font-family:Arial,Helvetica,sans-serif;font-weight:800;margin-right:1.2rem;text-decoration:none}
@media(max-width:800px){.models,.apps{grid-template-columns:1fr}.models article{border-right:0;border-bottom:6px solid #111}}
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="#features">Features</a><a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="poster" id="top">
  <h1>All<br>In<br>One</h1>
  <p class="sub">M17 Series servomotors. Motor. Driver. Controller. Encoder. One package. One RS-485 bus.</p>
  <a class="shop" href="{STORE}" style="display:inline-block;background:#7AB648;color:#fff;padding:.7rem 1.3rem;text-decoration:none;font-family:Arial,sans-serif;font-weight:800">Shop Now</a>
  <img src="../M17_series_overview.jpg" alt="M17 series">
</section>
<section class="block" id="features">
  <h2>What you get</h2>
  <ul class="feat-list">
    <li>Daisy-chain any number of motors from one connection</li>
    <li>High-level trapezoid moves — no STEP/DIR timing</li>
    <li>NEMA 17 mounting · 12–24 V · 560 RPM max · 32 kHz PID</li>
    <li>Protection: over-current, over-voltage, over-temperature</li>
    <li>Pi · Arduino · ESP32 · Mac · PC</li>
  </ul>
</section>
<section class="models" id="models">
  <article><h3>M17-60</h3><p>0.65 N·m<br>38 W · 470 g<br>59.8 mm</p></article>
  <article><h3>M17-48</h3><p>0.55 N·m<br>32 W · 360 g<br>48.6 mm</p></article>
  <article><h3>M17-40</h3><p>0.42 N·m<br>25 W · 285 g<br>41.6 mm</p></article>
</section>
<section class="block">
  <h2>Specs</h2>
  {specs_table()}
</section>
<section class="block" id="start">
  <h2>Code</h2>
  <p style="font-family:Arial,sans-serif;font-weight:700;letter-spacing:0">pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="apps" id="apps">
  <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
  <figure><img src="../automation_small.jpg" alt=""><figcaption>Automation</figcaption></figure>
  <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Testing</figcaption></figure>
  <figure><img src="../kit_with_three_motors_small.jpg" alt=""><figcaption>Kits</figcaption></figure>
</section>
<footer id="about">
  <h2 style="margin-top:0;text-transform:uppercase">Open source · Gearotons · 2022</h2>
  <p style="font-family:Arial,sans-serif;font-weight:400;letter-spacing:0;opacity:.85">Founded Shenzhen by a Canadian entrepreneur. Affordable and Simple All-in-One Motion Control.</p>
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(6, "Bold Poster", doc(css, body))


def build_v7():
    """Swiss Grid: strict modular multi-row grid, red accent, Helvetica, numbered modules."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#f2f2f2;color:#111}
.top{position:sticky;top:0;z-index:20;background:#fff;border-bottom:1px solid #111;
  display:grid;grid-template-columns:1fr auto auto;align-items:center;padding:0}
.top .logo{padding:.6rem 1.2rem;border-right:1px solid #111}
.top img{height:28px}
.top nav{display:flex}
.top nav a{padding:.75rem 1.1rem;text-decoration:none;border-right:1px solid #111;font-size:.8rem;font-weight:500}
.top a.shop{background:#e30613;color:#fff;padding:.75rem 1.3rem;text-decoration:none;font-weight:700;font-size:.8rem}
.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:0;border-left:1px solid #111}
.mod{border-right:1px solid #111;border-bottom:1px solid #111;padding:1.4rem;background:#fff;min-height:120px}
.mod .n{font-size:.7rem;color:#e30613;font-weight:700;letter-spacing:.1em}
.mod h2,.mod h1{margin:.3rem 0 .5rem;font-weight:500;letter-spacing:-.02em}
.span6{grid-column:span 6}.span4{grid-column:span 4}.span8{grid-column:span 8}.span3{grid-column:span 3}.span12{grid-column:span 12}
.mod img{width:100%;height:100%;object-fit:cover;max-height:320px}
table.specs{font-size:.8rem}table.specs th,table.specs td{border:1px solid #111;padding:.45rem;text-align:left;background:#fff}
pre.code{background:#111;color:#eee;padding:1rem;font-size:.8rem;margin:0;font-family:Menlo,Consolas,monospace}
footer.mod{background:#111;color:#fff}
footer.mod a{color:#fff;margin-right:1rem}
@media(max-width:900px){.span6,.span4,.span8,.span3{grid-column:span 12}}
"""
    body = f"""
<header class="top">
  <a class="logo" href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <nav>
    <a href="#m01">01</a><a href="#m02">02</a><a href="#m03">03</a><a href="#m04">04</a>
  </nav>
  <a class="shop" href="{STORE}">Shop Now</a>
</header>
<main class="grid" id="top">
  <section class="mod span8" id="m01">
    <div class="n">MODULE 01 · HERO</div>
    <h1 style="font-size:clamp(1.8rem,4vw,3rem)">M17 Series Servomotors</h1>
    <p>Affordable and Simple All-in-One Motion Control. Motor, driver, controller, encoder — one NEMA 17 package. RS-485 daisy-chain.</p>
    <p style="margin-top:1rem"><a href="{STORE}" style="background:#e30613;color:#fff;padding:.55rem 1rem;text-decoration:none;font-weight:700">Shop Now</a>
    <a href="{DOCS}" style="margin-left:.6rem">Documentation</a></p>
  </section>
  <section class="mod span4" style="padding:0">
    <img src="../one_motor_small.jpg" alt="M17 motor" style="max-height:none;height:100%;object-fit:cover">
  </section>
  <section class="mod span3" id="m02"><div class="n">02</div><h2>Integrate</h2><p>All-in-one package — no drive cabinet per axis.</p></section>
  <section class="mod span3"><div class="n">03</div><h2>Bus</h2><p>Any number of motors on one RS-485 link.</p></section>
  <section class="mod span3"><div class="n">04</div><h2>Loop</h2><p>32 kHz PID · encoder onboard · 560 RPM max.</p></section>
  <section class="mod span3"><div class="n">05</div><h2>Hosts</h2><p>Pi · Arduino · ESP32 · Mac · PC.</p></section>
  <section class="mod span4" id="m03"><div class="n">06 · M17-60</div><h2>0.65 N·m</h2><p>38 W · 470 g · 59.8 mm · highest torque</p></section>
  <section class="mod span4"><div class="n">07 · M17-48</div><h2>0.55 N·m</h2><p>32 W · 360 g · 48.6 mm · balanced</p></section>
  <section class="mod span4"><div class="n">08 · M17-40</div><h2>0.42 N·m</h2><p>25 W · 285 g · 41.6 mm · compact</p></section>
  <section class="mod span12">
    <div class="n">09 · SPECIFICATION TABLE</div>
    {specs_table()}
  </section>
  <section class="mod span6" id="m04">
    <div class="n">10 · GET STARTED</div>
    <h2>Python</h2>
    <p style="font-size:.9rem">pip3 install servomotor</p>
    {code_pre()}
  </section>
  <section class="mod span6">
    <div class="n">11 · APPLICATIONS</div>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:.5rem;margin-top:.5rem">
      <img src="../robotics_small.jpg" alt="Robotics" style="aspect-ratio:1;object-fit:cover;max-height:160px">
      <img src="../automation_small.jpg" alt="CNC" style="aspect-ratio:1;object-fit:cover;max-height:160px">
      <img src="../test_rack_small.jpg" alt="Test" style="aspect-ratio:1;object-fit:cover;max-height:160px">
      <img src="../kit_with_three_motors_small.jpg" alt="Kit" style="aspect-ratio:1;object-fit:cover;max-height:160px">
    </div>
  </section>
  <footer class="mod span12" id="about">
    <div class="n" style="color:#e30613">12 · COMPANY</div>
    <h2>Open source · Gearotons · Shenzhen 2022</h2>
    <p>Canadian founder. Firmware, libraries, schematics public.</p>
    <p style="margin-top:1rem">
      <a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a>
    </p>
    <div style="display:flex;gap:.8rem;margin-top:1rem">
      <img src="../Open_Source_Initiative.svg.png" alt="OSI" style="height:32px;filter:invert(1)">
      <img src="../Open-source-hardware-logo.svg.png" alt="OSHW" style="height:32px">
    </div>
  </footer>
</main>"""
    register(7, "Swiss Grid", doc(css, body))


def build_v8():
    """Warm Workshop: scrapbook / polaroid photo wall + kraft paper, irregular layout."""
    css = """
body{font-family:Georgia,Times,serif;background:#c4a574;color:#2a1f14;
  background-image:radial-gradient(rgba(80,50,20,.06) 1px,transparent 1px);background-size:12px 12px}
.tape-nav{position:sticky;top:0;z-index:20;background:#f5e6c8;border-bottom:3px dashed #8b6914;
  display:flex;justify-content:space-between;align-items:center;padding:.6rem 3vw;transform:rotate(-.3deg)}
.tape-nav img{height:36px}.tape-nav a{font-family:Avenir,Helvetica,Arial,sans-serif;text-decoration:none;margin-left:.9rem;font-weight:700;font-size:.88rem}
.tape-nav a.shop{background:#7AB648;color:#fff;padding:.45rem .9rem;border-radius:3px}
.wall{padding:2rem 3vw;display:flex;flex-wrap:wrap;gap:1.5rem;justify-content:center;align-items:flex-start}
.polaroid{background:#fffef8;padding:.7rem .7rem 1.5rem;box-shadow:3px 4px 12px rgba(40,25,10,.25);max-width:260px}
.polaroid img{width:100%;aspect-ratio:1;object-fit:cover;background:#eee}
.polaroid figcaption{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.82rem;margin-top:.6rem;text-align:center}
.r1{transform:rotate(-3deg)}.r2{transform:rotate(2.5deg)}.r3{transform:rotate(-1.5deg)}.r4{transform:rotate(4deg)}.r5{transform:rotate(-2deg)}
.note{background:#fff9c4;padding:1.2rem 1.4rem;max-width:320px;box-shadow:2px 3px 8px rgba(0,0,0,.15);transform:rotate(1.5deg);
  font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.95rem}
.note h1{font-family:Georgia,serif;font-size:1.6rem;margin:0 0 .5rem}
.paper{background:#f5e6c8;margin:1rem 4vw 2rem;padding:2rem;box-shadow:0 4px 16px rgba(40,25,10,.2);border:1px solid #b8955a}
.paper h2{margin-top:0}
table.specs{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.85rem;background:#fffef8}
table.specs th,table.specs td{border:1px solid #c4a574;padding:.5rem;text-align:left}
pre.code{background:#2a1f14;color:#e8d5a8;padding:1rem;font-size:.82rem;font-family:Menlo,monospace}
footer{background:#2a1f14;color:#f5e6c8;padding:2.5rem 4vw;margin-top:1rem}
footer a{color:#7AB648;margin-right:1rem;font-family:Avenir,Helvetica,Arial,sans-serif;font-weight:700}
@media(max-width:600px){.polaroid{max-width:100%}.r1,.r2,.r3,.r4,.r5{transform:none}}
"""
    body = f"""
<header class="tape-nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="#wall">Photos</a><a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<div class="wall" id="top">
  <div class="note">
    <h1>Workshop notes</h1>
    <p><strong>M17 Series</strong> — all-in-one servomotors. Affordable and Simple All-in-One Motion Control.</p>
    <p>Motor + driver + controller + encoder. RS-485 daisy-chain. 12–24 V. Open source.</p>
    <p style="margin-top:1rem"><a href="{STORE}" style="background:#7AB648;color:#fff;padding:.5rem 1rem;text-decoration:none;font-weight:700;border-radius:3px">Shop Now</a></p>
  </div>
  <figure class="polaroid r1"><img src="../one_motor_small.jpg" alt=""><figcaption>Single motor study</figcaption></figure>
  <figure class="polaroid r2"><img src="../kit_with_three_motors_small.jpg" alt=""><figcaption>Full kit flat-lay</figcaption></figure>
  <figure class="polaroid r3"><img src="../adapter_and_wire_small.jpg" alt=""><figcaption>RS-485 adapter</figcaption></figure>
  <figure class="polaroid r4" id="wall"><img src="../robotics_small.jpg" alt=""><figcaption>Robotics build</figcaption></figure>
  <figure class="polaroid r5"><img src="../automation_small.jpg" alt=""><figcaption>Automation cell</figcaption></figure>
  <figure class="polaroid r1"><img src="../test_rack_small.jpg" alt=""><figcaption>Test rack</figcaption></figure>
  <figure class="polaroid r2"><img src="../motor_back_small.jpg" alt=""><figcaption>Back label &amp; buttons</figcaption></figure>
  <div class="note" style="transform:rotate(-2deg);background:#e8f5d8">
    <h2 style="margin:0 0 .5rem;font-size:1.2rem">Features scrap</h2>
    <ul style="margin:0;padding-left:1.1rem;font-size:.9rem">
      <li>High-level trapezoid moves</li>
      <li>NEMA 17 standard mount</li>
      <li>32 kHz closed-loop PID</li>
      <li>Pi / Arduino / ESP32 / PC</li>
      <li>Over-current / voltage / temp protect</li>
    </ul>
  </div>
</div>
<section class="paper" id="models">
  <h2>Model comparison (workshop clipboard)</h2>
  <p style="font-family:Avenir,Helvetica,Arial,sans-serif">M17-60 · 0.65 N·m · 38 W · 470 g &nbsp;|&nbsp; M17-48 · 0.55 N·m · 32 W · 360 g &nbsp;|&nbsp; M17-40 · 0.42 N·m · 25 W · 285 g</p>
  {specs_table()}
</section>
<section class="paper" id="start">
  <h2>First spin (Python)</h2>
  <p style="font-family:Avenir,Helvetica,Arial,sans-serif">pip3 install servomotor</p>
  {code_pre()}
</section>
<footer id="about">
  <h2 style="margin-top:0">Open source bench</h2>
  <p>Gearotons — founded Shenzhen 2022 by a Canadian entrepreneur. Firmware &amp; libraries on GitHub.</p>
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
  <div style="display:flex;gap:.8rem;margin-top:1rem">
    <img src="../Open_Source_Initiative.svg.png" alt="OSI" style="height:36px">
    <img src="../Open-source-hardware-logo.svg.png" alt="OSHW" style="height:36px">
  </div>
</footer>"""
    register(8, "Warm Workshop", doc(css, body))


def build_v9():
    """Neon Arcade: split dual panels, game-UI stats bars, neon on dark purple."""
    css = """
body{font-family:ui-sans-serif,system-ui,Segoe UI,Helvetica,Arial,sans-serif;background:#12081f;color:#f0e6ff}
.hud{position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;
  padding:.55rem 3vw;background:rgba(18,8,31,.95);border-bottom:2px solid #ff2bd6;box-shadow:0 0 20px rgba(255,43,214,.25)}
.hud .brand{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:800;letter-spacing:.08em;text-transform:uppercase;font-size:.78rem;color:#00f0ff}
.hud img{height:26px}.hud nav a{color:#c9b3ff;text-decoration:none;margin-left:1rem;font-size:.85rem}
.hud a.shop{background:linear-gradient(90deg,#ff2bd6,#7b2fff);color:#fff;padding:.4rem .9rem;font-weight:800;border-radius:2px}
.split{display:grid;grid-template-columns:1fr 1fr;min-height:88vh}
.left{padding:3rem 3vw;display:flex;flex-direction:column;justify-content:center;border-right:2px solid #3d1f6b;
  background:radial-gradient(circle at 30% 40%,rgba(255,43,214,.15),transparent 50%)}
.right{padding:2rem 3vw;display:flex;flex-direction:column;justify-content:center;align-items:center;
  background:radial-gradient(circle at 70% 50%,rgba(0,240,255,.12),transparent 45%)}
.left h1{font-size:clamp(1.9rem,4vw,3rem);margin:0 0 .8rem;text-shadow:0 0 18px rgba(0,240,255,.5)}
.neon{color:#00f0ff;text-shadow:0 0 12px #00f0ff}
.stat{margin:1rem 0}
.stat label{display:flex;justify-content:space-between;font-size:.75rem;letter-spacing:.08em;text-transform:uppercase;color:#c9b3ff}
.bar{height:10px;background:#2a1545;border:1px solid #ff2bd6;margin-top:.3rem;border-radius:2px;overflow:hidden}
.bar i{display:block;height:100%;background:linear-gradient(90deg,#00f0ff,#ff2bd6);box-shadow:0 0 10px #00f0ff}
.right img{max-height:50vh;filter:drop-shadow(0 0 30px rgba(255,43,214,.4))}
.panel{padding:2rem 3vw;border-top:2px solid #3d1f6b}
.panel h2{color:#ff2bd6;text-transform:uppercase;letter-spacing:.1em;font-size:.9rem}
.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.cards article{border:1px solid #7b2fff;padding:1rem;background:rgba(60,20,100,.35);box-shadow:0 0 15px rgba(123,47,255,.2)}
.cards h3{margin:0 0 .4rem;color:#00f0ff}
table.specs{font-size:.82rem;font-family:ui-monospace,Menlo,monospace}
table.specs th,table.specs td{border:1px solid #5a2a9a;padding:.45rem;text-align:left}
pre.code{background:#0a0414;border:1px solid #00f0ff;color:#00f0ff;padding:1rem;font-size:.8rem;box-shadow:0 0 12px rgba(0,240,255,.2)}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{border:2px solid #ff2bd6;width:100%;aspect-ratio:16/10;object-fit:cover}
footer{padding:2.5rem 3vw;border-top:2px solid #ff2bd6;text-align:center}
footer a{color:#00f0ff;margin:0 .7rem;font-weight:700}
@media(max-width:850px){.split,.cards,.apps{grid-template-columns:1fr}.left{border-right:0;border-bottom:2px solid #3d1f6b}}
"""
    body = f"""
<header class="hud">
  <a class="brand" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons // M17</a>
  <nav>
    <a href="#stats">Stats</a><a href="#models">Models</a><a href="#code">Code</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="split" id="top">
  <div class="left" id="stats">
    <p class="neon" style="font-size:.75rem;letter-spacing:.15em;font-weight:800">PLAYER ONE · MOTION READY</p>
    <h1>Level up your axes</h1>
    <p>All-in-one servomotors. Daisy-chain over RS-485. Closed-loop control without the boss fight of drive cabinets.</p>
    <div class="stat"><label><span>Torque tier</span><span>0.42–0.65 N·m</span></label><div class="bar"><i style="width:85%"></i></div></div>
    <div class="stat"><label><span>Max speed</span><span>560 RPM</span></label><div class="bar"><i style="width:70%"></i></div></div>
    <div class="stat"><label><span>PID loop</span><span>32 kHz</span></label><div class="bar"><i style="width:95%"></i></div></div>
    <div class="stat"><label><span>Voltage</span><span>12–24 V</span></label><div class="bar"><i style="width:60%"></i></div></div>
    <p style="margin-top:1.2rem">
      <a class="shop" href="{STORE}" style="background:linear-gradient(90deg,#ff2bd6,#7b2fff);color:#fff;padding:.55rem 1.1rem;text-decoration:none;font-weight:800">Shop Now</a>
      <a href="{DOCS}" style="margin-left:.8rem;color:#00f0ff">Documentation</a>
    </p>
  </div>
  <div class="right">
    <img src="../transparent/one_motor_transparent_small.png" alt="M17 motor">
  </div>
</section>
<section class="panel" id="features">
  <h2>Power-ups</h2>
  <div class="cards">
    <article><h3>Integrate</h3><p>Motor + driver + controller + encoder.</p></article>
    <article><h3>Multiplayer bus</h3><p>Any number of motors on one RS-485 link.</p></article>
    <article><h3>Combo moves</h3><p>High-level trapezoid — no STEP/DIR timing.</p></article>
    <article><h3>Shields up</h3><p>Over-current, voltage, temperature protection.</p></article>
    <article><h3>Cross-platform</h3><p>Pi, Arduino, ESP32, Mac, PC.</p></article>
    <article><h3>Open source</h3><p>Firmware &amp; libraries on GitHub.</p></article>
  </div>
</section>
<section class="panel" id="models">
  <h2>Select character</h2>
  <p>M17-60 (0.65 N·m) · M17-48 (0.55 N·m) · M17-40 (0.42 N·m) · footprint 42.2 × 42.2 mm</p>
  {specs_table()}
</section>
<section class="panel" id="code">
  <h2>Cheat code · Python</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="panel" id="apps">
  <h2>Stages</h2>
  <div class="apps">
    <figure style="margin:0"><img src="../robotics_small.jpg" alt=""><figcaption style="margin-top:.4rem">Robotics</figcaption></figure>
    <figure style="margin:0"><img src="../automation_small.jpg" alt=""><figcaption style="margin-top:.4rem">CNC</figcaption></figure>
    <figure style="margin:0"><img src="../test_rack_small.jpg" alt=""><figcaption style="margin-top:.4rem">Test lab</figcaption></figure>
  </div>
</section>
<footer id="about">
  <p>Gearotons · Shenzhen 2022 · Canadian founder · Open source hardware &amp; software</p>
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
  <p style="opacity:.7;font-size:.9rem">Affordable and Simple All-in-One Motion Control</p>
</footer>"""
    register(9, "Neon Arcade", doc(css, body))


def build_v10():
    """Luxury Minimal: almost single-column, massive whitespace, sequential one image at a time."""
    css = """
body{font-family:Optima,Palatino,Georgia,serif;background:#fafaf8;color:#1a1a1a}
.nav{position:fixed;top:0;left:0;right:0;z-index:30;display:flex;justify-content:space-between;align-items:center;
  padding:1.2rem 5vw;background:transparent}
.nav img{height:28px;opacity:.9}
.nav a{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.72rem;letter-spacing:.16em;text-transform:uppercase;text-decoration:none;margin-left:1.5rem;font-weight:500}
.nav a.shop{border-bottom:1px solid #1a1a1a;padding-bottom:.15rem}
.seq{max-width:720px;margin:0 auto;padding:8rem 6vw 4rem}
.seq section{min-height:70vh;display:flex;flex-direction:column;justify-content:center;padding:3rem 0;border:0}
.seq h1{font-weight:400;font-size:clamp(2rem,5vw,3.2rem);letter-spacing:-.02em;line-height:1.15;margin:0 0 1.5rem}
.seq h2{font-weight:400;font-size:clamp(1.4rem,3vw,2rem);margin:0 0 1rem}
.seq p{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:1rem;line-height:1.7;opacity:.82;max-width:34rem}
.seq img{width:100%;margin:2rem 0;max-height:55vh;object-fit:contain}
.seq .cap{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.75rem;letter-spacing:.08em;text-transform:uppercase;opacity:.5;margin-top:-1rem}
.line{width:40px;height:1px;background:#1a1a1a;margin:2rem 0}
table.specs{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.82rem;margin:1.5rem 0}
table.specs th,table.specs td{border-bottom:1px solid #ddd;padding:.65rem .4rem;text-align:left}
pre.code{background:#1a1a1a;color:#eaeaea;padding:1.4rem;font-size:.8rem;font-family:Menlo,monospace;font-weight:400}
.quiet-cta{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.8rem;letter-spacing:.12em;text-transform:uppercase;text-decoration:none;border-bottom:1px solid #1a1a1a;padding-bottom:.2rem}
footer{text-align:center;padding:4rem 6vw 6rem;font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.85rem}
footer a{margin:0 .8rem;text-decoration:none;letter-spacing:.08em;text-transform:uppercase;font-size:.72rem}
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="#product">Product</a>
    <a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop</a>
  </div>
</header>
<main class="seq" id="top">
  <section>
    <p class="cap">M17 Series</p>
    <h1>Quiet power.<br>Clear control.</h1>
    <p>Affordable and Simple All-in-One Motion Control. A servomotor that holds the driver, controller, and encoder inside one calm package.</p>
    <div class="line"></div>
    <p><a class="quiet-cta" href="{STORE}">Shop Now</a> &nbsp;&nbsp; <a class="quiet-cta" href="{DOCS}">Documentation</a></p>
  </section>
  <section id="product">
    <img src="../one_motor_small.jpg" alt="M17 motor">
    <p class="cap">One body</p>
    <h2>Everything that used to be a stack</h2>
    <p>Motor, driver, motion controller, and encoder — integrated. Daisy-chain any number over RS-485 from a single connection on Mac, PC, Raspberry Pi, Arduino, or ESP32.</p>
  </section>
  <section>
    <img src="../M17_series_overview.jpg" alt="Three models">
    <p class="cap">Three models</p>
    <h2>Torque, matched to need</h2>
    <p>M17-60 at 0.65 N·m · M17-48 at 0.55 N·m · M17-40 at 0.42 N·m. Same protocol. Same 42.2 × 42.2 mm footprint. 12–24 V. Max 560 RPM. Closed-loop PID at 32 kHz.</p>
  </section>
  <section id="models">
    <h2>Specifications</h2>
    {specs_table()}
  </section>
  <section id="start">
    <h2>Begin in minutes</h2>
    <p>pip3 install servomotor</p>
    {code_pre()}
  </section>
  <section id="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <p class="cap">Applications</p>
    <h2>Robotics, CNC, test, education</h2>
    <p>Built for machines that need honest closed-loop motion — and for classrooms that need something approachable.</p>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:1rem;margin-top:1.5rem">
      <img src="../automation_small.jpg" alt="Automation" style="margin:0">
      <img src="../test_rack_small.jpg" alt="Test" style="margin:0">
    </div>
  </section>
  <section id="about">
    <h2>Open by design</h2>
    <p>Gearotons was founded in Shenzhen in 2022 by a Canadian entrepreneur. Firmware, libraries, and schematics are open source.</p>
    <div style="display:flex;gap:1rem;margin:1.5rem 0">
      <img src="../Open_Source_Initiative.svg.png" alt="OSI" style="height:36px;width:auto;margin:0">
      <img src="../Open-source-hardware-logo.svg.png" alt="OSHW" style="height:36px;width:auto;margin:0">
    </div>
    <p><a class="quiet-cta" href="{STORE}">Shop Now</a></p>
  </section>
</main>
<footer>
  <p>Gearotons · Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(10, "Luxury Minimal", doc(css, body))


# =============================================================================
# ROUND 2 — company design-language pastiches (structure matching marketing sites)
# =============================================================================

def build_v11():
    """NVIDIA-like: black+lime, stacked full-width story heroes, horizontal card rails."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#000;color:#eee}
.nav{position:sticky;top:0;z-index:30;background:rgba(0,0,0,.92);display:flex;justify-content:space-between;align-items:center;padding:.7rem 4vw;border-bottom:1px solid #222}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700;letter-spacing:.04em}
.nav img{height:28px}.nav a{color:#ccc;text-decoration:none;margin-left:1.2rem;font-size:.88rem}
.nav a.shop{color:#76b900;font-weight:700}
.story{min-height:72vh;display:grid;place-items:center;text-align:center;padding:4rem 4vw;
  background:radial-gradient(ellipse at 50% 0%,#1a2a00 0%,#000 55%)}
.story h1{font-size:clamp(2.2rem,6vw,4.2rem);margin:0 0 1rem;letter-spacing:-.03em}
.story .lime{color:#76b900}
.story p{max-width:36rem;margin:0 auto 1.5rem;opacity:.85;font-size:1.1rem}
.story img{max-height:38vh;margin:0 auto 1.5rem;filter:drop-shadow(0 20px 40px rgba(118,185,0,.2))}
.btn{display:inline-block;background:#76b900;color:#000;padding:.65rem 1.3rem;text-decoration:none;font-weight:700;border-radius:2px}
.btn-o{display:inline-block;border:1px solid #76b900;color:#76b900;padding:.65rem 1.3rem;text-decoration:none;font-weight:600;margin-left:.5rem;border-radius:2px}
.cat{padding:3rem 0 1rem}
.cat h2{padding:0 4vw;font-size:1.5rem;margin:0 0 1rem}
.rail{display:flex;gap:1rem;overflow-x:auto;padding:0 4vw 2rem;scroll-snap-type:x mandatory}
.rail article{min-width:min(300px,80vw);flex-shrink:0;scroll-snap-align:start;background:#111;border:1px solid #222;border-radius:8px;padding:1.3rem}
.rail h3{color:#76b900;margin:0 0 .5rem}
.band{padding:4rem 4vw;border-top:1px solid #222}
.band.dark{background:#0a0a0a}
.band h2{font-size:clamp(1.6rem,3vw,2.4rem);margin:0 0 1rem}
table.specs{font-size:.85rem}table.specs th,table.specs td{border-bottom:1px solid #333;padding:.55rem;text-align:left}
pre.code{background:#0d0d0d;border:1px solid #333;color:#b8e986;padding:1.2rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover;border-radius:6px}
footer{padding:3rem 4vw;border-top:1px solid #222;background:#050505}
footer a{color:#76b900;margin-right:1.2rem}
@media(max-width:700px){.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#platforms">Platforms</a><a href="#models">Models</a><a href="#start">Develop</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="story" id="top">
  <div>
    <img src="../transparent/one_motor_transparent_small.png" alt="M17">
    <h1>The engine of <span class="lime">motion</span></h1>
    <p>M17 Series — all-in-one servomotors that put closed-loop control on every axis without a rack of drives.</p>
    <a class="btn" href="{STORE}">Shop Now</a>
    <a class="btn-o" href="{DOCS}">Documentation</a>
  </div>
</section>
<section class="cat" id="platforms">
  <h2>Built for every host</h2>
  <div class="rail">
    <article><h3>Raspberry Pi</h3><p>Control a bus of motors from one UART + RS-485 adapter.</p></article>
    <article><h3>Arduino</h3><p>Library Manager: “Servomotor” — high-level moves in sketches.</p></article>
    <article><h3>ESP32</h3><p>Wireless projects with the same RS-485 protocol.</p></article>
    <article><h3>Mac &amp; PC</h3><p>Python library for desktops and lab benches.</p></article>
    <article><h3>Education</h3><p>AI-friendly docs; ideal for teaching real motion control.</p></article>
  </div>
</section>
<section class="story" style="min-height:60vh;background:radial-gradient(ellipse at 50% 100%,#152200 0%,#000 60%)">
  <div>
    <img src="../transparent/M17_series_overview_transparent_small.png" alt="Series" style="max-height:32vh">
    <h1 style="font-size:clamp(1.6rem,4vw,2.8rem)">One protocol. <span class="lime">Three torque tiers.</span></h1>
    <p>0.42 · 0.55 · 0.65 N·m — same software path, same mounting.</p>
  </div>
</section>
<section class="band" id="models">
  <h2>Specifications</h2>
  <p style="opacity:.8;margin-bottom:1rem">12–24 V · max 560 RPM · 32 kHz PID · NEMA 17 · footprint 42.2 × 42.2 mm</p>
  {specs_table()}
</section>
<section class="band dark" id="start">
  <h2>Develop</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="band" id="apps">
  <h2>Where it runs</h2>
  <div class="apps">
    <figure style="margin:0"><img src="../robotics_small.jpg" alt=""><figcaption style="margin-top:.4rem">Robotics</figcaption></figure>
    <figure style="margin:0"><img src="../automation_small.jpg" alt=""><figcaption style="margin-top:.4rem">Automation</figcaption></figure>
    <figure style="margin:0"><img src="../test_rack_small.jpg" alt=""><figcaption style="margin-top:.4rem">Test systems</figcaption></figure>
  </div>
</section>
<footer id="about">
  <h2 style="color:#76b900">Open source</h2>
  <p>Gearotons · Shenzhen 2022 · Canadian founder. Affordable and Simple All-in-One Motion Control.</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(11, "NVIDIA-style", doc(css, body))


def build_v12():
    """Apple-like: centered product units stacked full-bleed, huge type, promo 2-up, pill blue links."""
    css = """
body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Helvetica Neue,Arial,sans-serif;background:#fff;color:#1d1d1f}
.nav{position:sticky;top:0;z-index:40;background:rgba(255,255,255,.8);backdrop-filter:saturate(180%) blur(16px);
  display:flex;justify-content:center;align-items:center;gap:1.8rem;padding:.55rem 2vw;font-size:.75rem}
.nav img{height:18px}.nav a{text-decoration:none;color:#1d1d1f;opacity:.88}
.nav a.shop{color:#0071e3}
.unit{text-align:center;padding:5rem 4vw 3rem;background:#fbfbfd}
.unit.dark{background:#000;color:#f5f5f7}
.unit h1{font-size:clamp(2.4rem,6vw,4.5rem);font-weight:600;letter-spacing:-.03em;margin:0 0 .6rem;line-height:1.05}
.unit .tag{font-size:1.2rem;opacity:.9;margin:0 auto 1.2rem;max-width:32rem}
.unit img{max-height:48vh;margin:2rem auto 0}
.links a{color:#0071e3;text-decoration:none;font-size:1.05rem;margin:0 .8rem}
.links a:hover{text-decoration:underline}
.promo{display:grid;grid-template-columns:1fr 1fr;gap:.8rem;padding:.8rem;background:#fff}
.promo article{background:#fbfbfd;text-align:center;padding:2.5rem 1.5rem 1.5rem;border-radius:18px;overflow:hidden}
.promo h2{font-size:1.6rem;margin:0 0 .4rem;font-weight:600}
.promo p{font-size:.95rem;opacity:.85;max-width:22rem;margin:0 auto .8rem}
.promo img{max-height:200px;margin:1rem auto 0}
.sheet{max-width:900px;margin:0 auto;padding:3rem 4vw}
.sheet h2{text-align:center;font-size:2rem;font-weight:600}
table.specs{font-size:.88rem;margin-top:1.5rem}
table.specs th,table.specs td{border-bottom:1px solid #d2d2d7;padding:.7rem .5rem;text-align:left}
pre.code{background:#1d1d1f;color:#f5f5f7;padding:1.3rem;border-radius:12px;font-size:.82rem}
footer{background:#f5f5f7;padding:2.5rem 4vw;text-align:center;font-size:.85rem;color:#6e6e73}
footer a{color:#0071e3;margin:0 .6rem;text-decoration:none}
@media(max-width:800px){.promo{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <a href="#m17">M17</a><a href="#models">Models</a><a href="#start">Get started</a>
  <a class="shop" href="{STORE}">Shop</a>
</header>
<section class="unit" id="top">
  <p style="color:#bf4800;font-weight:600;font-size:.9rem;margin:0 0 .5rem">New</p>
  <h1>M17 Series</h1>
  <p class="tag">Affordable and Simple All-in-One Motion Control.</p>
  <div class="links"><a href="{STORE}">Shop Now ›</a><a href="{DOCS}">Learn more ›</a></div>
  <img src="../M17_series_overview.jpg" alt="M17 Series">
</section>
<section class="unit dark" id="m17">
  <h1>Motor. Driver.<br>Controller. Encoder.</h1>
  <p class="tag">One package. Daisy-chain any number over RS-485 from a single connection.</p>
  <div class="links"><a href="#features" style="color:#2997ff">See features ›</a></div>
  <img src="../transparent/one_motor_transparent_small.png" alt="Motor">
</section>
<section class="promo" id="features">
  <article>
    <h2>Closed loop at 32 kHz</h2>
    <p>Built-in encoder and PID. High-level trapezoid moves — no STEP/DIR timing.</p>
    <a href="{DOCS}" style="color:#0071e3;text-decoration:none">Documentation ›</a>
    <img src="../motor_back_small.jpg" alt="Motor back">
  </article>
  <article>
    <h2>NEMA 17, refined</h2>
    <p>Nearly the size of a comparable stepper. Standard mounting. 12–24 V with full protection.</p>
    <a href="#models" style="color:#0071e3;text-decoration:none">Compare models ›</a>
    <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="Kit">
  </article>
</section>
<section class="sheet" id="models">
  <h2>Choose a model</h2>
  <p style="text-align:center;opacity:.8">M17-60 · 0.65 N·m · 470 g &nbsp;·&nbsp; M17-48 · 0.55 N·m · 360 g &nbsp;·&nbsp; M17-40 · 0.42 N·m · 285 g</p>
  {specs_table()}
</section>
<section class="sheet" id="start">
  <h2>Get started</h2>
  <p style="text-align:center">pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="promo" id="apps">
  <article>
    <h2>Robotics &amp; CNC</h2>
    <p>Machines that need honest closed-loop axes.</p>
    <img src="../robotics_small.jpg" alt="Robotics">
  </article>
  <article>
    <h2>Test &amp; education</h2>
    <p>Labs and classrooms with open documentation.</p>
    <img src="../test_rack_small.jpg" alt="Test">
  </article>
</section>
<footer id="about">
  <p>Open source hardware &amp; software · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(12, "Apple-style", doc(css, body))


def build_v13():
    """Google/Material: white, colorful multi-card grid, friendly large cards with soft shadow."""
    css = """
body{font-family:Arial,Helvetica,sans-serif;background:#fff;color:#202124}
.nav{position:sticky;top:0;z-index:20;background:#fff;box-shadow:0 1px 2px rgba(60,64,67,.15),0 2px 6px rgba(60,64,67,.1);
  display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw}
.nav img{height:32px}.nav a{text-decoration:none;color:#5f6368;margin-left:1.2rem;font-size:.9rem;font-weight:500}
.nav a.shop{background:#1a73e8;color:#fff;padding:.5rem 1.2rem;border-radius:4px}
.hero{padding:3.5rem 3vw 2rem;text-align:center}
.hero h1{font-size:clamp(2rem,4.5vw,3.2rem);font-weight:400;margin:0 0 .8rem;letter-spacing:-.02em}
.hero p{font-size:1.15rem;color:#5f6368;max-width:36rem;margin:0 auto 1.5rem}
.hero .cta{display:inline-block;background:#1a73e8;color:#fff;padding:.7rem 1.5rem;border-radius:4px;text-decoration:none;font-weight:500}
.hero .cta2{display:inline-block;color:#1a73e8;padding:.7rem 1.2rem;text-decoration:none;font-weight:500}
.hero img{max-width:min(640px,90%);margin:2rem auto 0}
.cards{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:1.2rem;padding:1rem 3vw 3rem;max-width:1200px;margin:0 auto}
.card{background:#fff;border-radius:12px;box-shadow:0 1px 2px rgba(60,64,67,.15),0 2px 6px rgba(60,64,67,.1);padding:1.4rem;transition:box-shadow .2s}
.card:hover{box-shadow:0 4px 12px rgba(60,64,67,.18)}
.dot{width:12px;height:12px;border-radius:50%;display:inline-block;margin-right:.4rem}
.card h3{margin:.5rem 0;font-weight:500;font-size:1.15rem}
.card p{margin:0;color:#5f6368;font-size:.95rem}
.section{padding:2rem 3vw;max-width:1000px;margin:0 auto}
.section h2{font-weight:400;font-size:1.8rem}
table.specs{font-size:.88rem;margin-top:1rem}
table.specs th,table.specs td{border-bottom:1px solid #e8eaed;padding:.65rem .5rem;text-align:left}
pre.code{background:#202124;color:#e8eaed;padding:1.2rem;border-radius:8px;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(4,1fr);gap:1rem}
.apps img{border-radius:12px;width:100%;aspect-ratio:1;object-fit:cover;box-shadow:0 1px 3px rgba(60,64,67,.15)}
footer{background:#f8f9fa;padding:2.5rem 3vw;margin-top:2rem;text-align:center;color:#5f6368}
footer a{color:#1a73e8;margin:0 .7rem;text-decoration:none}
@media(max-width:700px){.apps{grid-template-columns:1fr 1fr}}
"""
    colors = ["#ea4335", "#fbbc04", "#34a853", "#4285f4", "#ea4335", "#34a853"]
    feat_cards = "".join(
        f"<article class='card'><span class='dot' style='background:{colors[i%6]}'></span>"
        f"<h3>{esc(t)}</h3><p>{esc(d)}</p></article>"
        for i, (t, d) in enumerate(FEATURES[:6])
    )
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="#features">Features</a><a href="#models">Models</a><a href="#start">Start</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="hero" id="top">
  <h1>Motion control that just works</h1>
  <p>M17 Series servomotors integrate motor, driver, controller, and encoder — then daisy-chain over RS-485.</p>
  <a class="cta" href="{STORE}">Shop Now</a>
  <a class="cta2" href="{DOCS}">Documentation</a>
  <img src="../M17_series_overview.jpg" alt="M17 Series">
</section>
<section class="cards" id="features">{feat_cards}</section>
<section class="section" id="models">
  <h2>Three models</h2>
  <p style="color:#5f6368">M17-60 · 0.65 N·m · 38 W · 470 g &nbsp;·&nbsp; M17-48 · 0.55 N·m · 32 W · 360 g &nbsp;·&nbsp; M17-40 · 0.42 N·m · 25 W · 285 g</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Get started with Python</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Applications</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
    <img src="../kit_with_three_motors_small.jpg" alt="Kit">
  </div>
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p style="margin-top:.8rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
  <p style="margin-top:.8rem;font-size:.85rem">Affordable and Simple All-in-One Motion Control</p>
</footer>"""
    register(13, "Google-style", doc(css, body))


def build_v14():
    """Microsoft-like: mega product nav strip, large hero + secondary promo 2x2, Fluent blue."""
    css = """
body{font-family:Segoe UI,Segoe UI Variable,Helvetica Neue,Arial,sans-serif;background:#fff;color:#1b1b1b}
.meganav{background:#f2f2f2;border-bottom:1px solid #e0e0e0;padding:.45rem 3vw;display:flex;gap:1.5rem;overflow-x:auto;font-size:.82rem}
.meganav a{text-decoration:none;color:#1b1b1b;white-space:nowrap;font-weight:600}
.meganav a.active{border-bottom:2px solid #0067b8;padding-bottom:.2rem}
.top{display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw;position:sticky;top:0;background:#fff;z-index:20;box-shadow:0 1px 3px rgba(0,0,0,.06)}
.top img{height:30px}.top a{text-decoration:none;color:#0067b8;font-weight:600;margin-left:1rem;font-size:.9rem}
.top a.shop{background:#0067b8;color:#fff;padding:.5rem 1.1rem}
.hero{display:grid;grid-template-columns:1fr 1fr;min-height:70vh;background:linear-gradient(135deg,#e8f3fc,#fff 50%)}
.hero-copy{padding:4rem 3vw;display:flex;flex-direction:column;justify-content:center}
.hero-copy h1{font-size:clamp(2rem,4vw,3.2rem);font-weight:600;margin:0 0 1rem;line-height:1.15}
.hero-copy p{font-size:1.1rem;max-width:32rem;opacity:.9}
.hero-vis{display:flex;align-items:center;justify-content:center;padding:2rem}
.hero-vis img{max-height:55vh}
.btn{display:inline-block;background:#0067b8;color:#fff;padding:.65rem 1.3rem;text-decoration:none;font-weight:600;margin-top:1.2rem;margin-right:.5rem}
.btn-g{display:inline-block;border:1px solid #0067b8;color:#0067b8;padding:.65rem 1.3rem;text-decoration:none;font-weight:600}
.promos{display:grid;grid-template-columns:1fr 1fr;gap:1rem;padding:1.5rem 3vw}
.promo{background:#f5f5f5;padding:1.5rem;display:grid;grid-template-columns:1fr 1fr;gap:1rem;align-items:center;border-left:4px solid #0067b8}
.promo h3{margin:0 0 .4rem;font-weight:600}.promo p{margin:0;font-size:.95rem;opacity:.88}
.promo img{max-height:140px;margin:0 auto}
.section{padding:2.5rem 3vw}
table.specs th,table.specs td{border-bottom:1px solid #e0e0e0;padding:.6rem;text-align:left;font-size:.9rem}
pre.code{background:#1b1b1b;color:#d4edff;padding:1.2rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(4,1fr);gap:.8rem}
.apps img{width:100%;aspect-ratio:4/3;object-fit:cover}
footer{background:#1b1b1b;color:#fff;padding:2.5rem 3vw}
footer a{color:#6cb8f0;margin-right:1rem}
@media(max-width:900px){.hero,.promos,.promo,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<div class="meganav">
  <a class="active" href="#top">M17 Series</a>
  <a href="#features">Features</a>
  <a href="#models">Models</a>
  <a href="#start">Developer tools</a>
  <a href="#apps">Solutions</a>
  <a href="#about">Company</a>
</div>
<header class="top">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="{DOCS}">Documentation</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="hero" id="top">
  <div class="hero-copy">
    <p style="color:#0067b8;font-weight:600;margin:0 0 .5rem">M17 Series Servomotors</p>
    <h1>All-in-one motion for every axis</h1>
    <p>Integrate motor, driver, motion controller, and encoder. Control any number over one RS-485 bus from Raspberry Pi, Arduino, ESP32, Mac, or PC.</p>
    <div>
      <a class="btn" href="{STORE}">Shop Now</a>
      <a class="btn-g" href="{DOCS}">Documentation</a>
    </div>
  </div>
  <div class="hero-vis"><img src="../M17_series_overview.jpg" alt="M17 Series"></div>
</section>
<section class="promos" id="features">
  <article class="promo">
    <div><h3>High-level commands</h3><p>Trapezoid move and enable MOSFETs — no timing-critical STEP/DIR.</p></div>
    <img src="../adapter_and_wire_small.jpg" alt="Adapter">
  </article>
  <article class="promo">
    <div><h3>Closed-loop 32 kHz</h3><p>Encoder + PID onboard. Max 560 RPM. 12–24 V with protection.</p></div>
    <img src="../one_motor_small.jpg" alt="Motor">
  </article>
  <article class="promo">
    <div><h3>NEMA 17 form factor</h3><p>Standard mounting, footprint 42.2 × 42.2 mm, no protrusions.</p></div>
    <img src="../M17-48_dimensions.png" alt="Dimensions">
  </article>
  <article class="promo">
    <div><h3>Open source stack</h3><p>Firmware, libraries, and schematics on GitHub.</p></div>
    <img src="../kit_with_three_motors_small.jpg" alt="Kit">
  </article>
</section>
<section class="section" id="models">
  <h2>Compare models</h2>
  <p>M17-60 (0.65 N·m) · M17-48 (0.55 N·m) · M17-40 (0.42 N·m)</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Developer tools — Python</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Solutions</h2>
  <div class="apps">
    <figure style="margin:0"><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
    <figure style="margin:0"><img src="../automation_small.jpg" alt=""><figcaption>Automation</figcaption></figure>
    <figure style="margin:0"><img src="../test_rack_small.jpg" alt=""><figcaption>Test</figcaption></figure>
    <figure style="margin:0"><img src="../motor_back_small.jpg" alt=""><figcaption>Hardware</figcaption></figure>
  </div>
</section>
<footer id="about">
  <h2 style="margin-top:0">Gearotons</h2>
  <p>Founded Shenzhen 2022 by a Canadian entrepreneur. Affordable and Simple All-in-One Motion Control.</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(14, "Microsoft-style", doc(css, body))


def build_v15():
    """Amazon-like: dark navy nav + search field visual, LEFT gallery + RIGHT buy-box PDP."""
    css = """
body{font-family:Avenir,Helvetica Neue,Arial,sans-serif;background:#eaeded;color:#0f1111}
.nav{background:#131921;color:#fff;padding:.55rem 2vw;display:flex;align-items:center;gap:1rem;flex-wrap:wrap;position:sticky;top:0;z-index:30}
.nav img{height:32px;background:#fff;border-radius:2px;padding:2px}
.nav .search{flex:1;min-width:180px;display:flex;background:#fff;border-radius:4px;overflow:hidden}
.nav .search span{padding:.55rem 1rem;color:#555;font-size:.9rem;flex:1}
.nav .search b{background:#febd69;color:#131921;padding:.55rem 1rem;font-size:.9rem}
.nav a.shop{background:#ff9900;color:#111;padding:.45rem 1rem;text-decoration:none;font-weight:700;border-radius:3px;white-space:nowrap}
.sub{background:#232f3e;color:#fff;padding:.4rem 2vw;font-size:.85rem;display:flex;gap:1.2rem;overflow-x:auto}
.sub a{color:#ddd;text-decoration:none;white-space:nowrap}
.pdp{display:grid;grid-template-columns:1.1fr .9fr;gap:1.5rem;padding:1.5rem 2vw;max-width:1200px;margin:0 auto;background:#fff}
.gallery{display:grid;grid-template-columns:64px 1fr;gap:.8rem}
.thumbs button{display:block;width:100%;padding:0;border:1px solid #ddd;background:#fff;margin-bottom:.5rem;cursor:pointer}
.thumbs img{width:100%;aspect-ratio:1;object-fit:cover}
.main-img{border:1px solid #eee;padding:1rem;display:flex;align-items:center;justify-content:center;min-height:360px}
.main-img img{max-height:400px}
.buybox{border:1px solid #d5d9d9;border-radius:8px;padding:1.2rem}
.buybox h1{font-size:1.45rem;font-weight:500;margin:0 0 .5rem;line-height:1.3}
.buybox .price{font-size:1.5rem;color:#b12704;font-weight:700;margin:.6rem 0}
.buybox .row{font-size:.9rem;margin:.35rem 0;display:flex;justify-content:space-between;border-bottom:1px solid #eee;padding-bottom:.35rem}
.buybox .cta{display:block;text-align:center;background:#ffd814;border:1px solid #fcd200;color:#0f1111;padding:.7rem;border-radius:20px;text-decoration:none;font-weight:700;margin-top:1rem}
.buybox .cta2{display:block;text-align:center;background:#ffa41c;border:1px solid #ff8f00;color:#0f1111;padding:.7rem;border-radius:20px;text-decoration:none;font-weight:700;margin-top:.5rem}
.bullets{margin:1rem 0 0;padding-left:1.2rem;font-size:.92rem}
.below{max-width:1200px;margin:1rem auto;padding:1.5rem 2vw;background:#fff}
.below h2{font-size:1.3rem;border-bottom:1px solid #e7e7e7;padding-bottom:.5rem}
table.specs th,table.specs td{border:1px solid #e7e7e7;padding:.55rem;text-align:left;font-size:.88rem}
pre.code{background:#232f3e;color:#eee;padding:1rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(4,1fr);gap:.8rem}
.apps img{width:100%;aspect-ratio:1;object-fit:cover}
footer{background:#232f3e;color:#ddd;padding:2rem 2vw;margin-top:1rem;text-align:center}
footer a{color:#febd69;margin:0 .7rem}
@media(max-width:900px){.pdp,.gallery,.apps{grid-template-columns:1fr}.thumbs{display:flex;gap:.4rem;overflow-x:auto}.thumbs button{min-width:56px}}
"""
    js = """
<script>
(function(){
  var main=document.getElementById('mainShot');
  document.querySelectorAll('.thumbs button').forEach(function(b){
    b.addEventListener('click',function(){main.src=b.getAttribute('data-src');main.alt=b.getAttribute('data-alt');});
  });
})();
</script>
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div class="search" aria-hidden="true"><span>Search M17 Series…</span><b>Go</b></div>
  <a class="shop" href="{STORE}">Shop Now</a>
</header>
<div class="sub">
  <a href="#top">Product</a><a href="#features">About</a><a href="#models">Compare</a>
  <a href="#start">Get started</a><a href="#apps">Applications</a><a href="{DOCS}">Docs</a>
</div>
<section class="pdp" id="top">
  <div class="gallery">
    <div class="thumbs">
      <button type="button" data-src="../one_motor_small.jpg" data-alt="Motor"><img src="../one_motor_small.jpg" alt=""></button>
      <button type="button" data-src="../M17_series_overview.jpg" data-alt="Series"><img src="../M17_series_overview.jpg" alt=""></button>
      <button type="button" data-src="../kit_with_three_motors_small.jpg" data-alt="Kit"><img src="../kit_with_three_motors_small.jpg" alt=""></button>
      <button type="button" data-src="../adapter_and_wire_small.jpg" data-alt="Adapter"><img src="../adapter_and_wire_small.jpg" alt=""></button>
      <button type="button" data-src="../motor_back_small.jpg" data-alt="Back"><img src="../motor_back_small.jpg" alt=""></button>
    </div>
    <div class="main-img"><img id="mainShot" src="../one_motor_small.jpg" alt="M17 motor"></div>
  </div>
  <div class="buybox">
    <h1>Gearotons M17 Series All-in-One Servomotor</h1>
    <p style="color:#007185;font-size:.9rem">Visit the Gearotons store</p>
    <div class="price">See store for pricing</div>
    <div class="row"><span>Brand</span><span>Gearotons</span></div>
    <div class="row"><span>Voltage</span><span>12–24 V</span></div>
    <div class="row"><span>Max speed</span><span>560 RPM</span></div>
    <div class="row"><span>Interface</span><span>RS-485 daisy-chain</span></div>
    <div class="row"><span>Models</span><span>M17-60 / 48 / 40</span></div>
    <a class="cta" href="{STORE}">Shop Now</a>
    <a class="cta2" href="{DOCS}">Documentation</a>
    <ul class="bullets" id="features">
      <li>Motor + driver + motion controller + encoder integrated</li>
      <li>High-level trapezoid moves — no STEP/DIR timing</li>
      <li>NEMA 17 mounting · 32 kHz closed-loop PID</li>
      <li>Works with Pi, Arduino, ESP32, Mac, PC</li>
      <li>Open-source firmware and libraries</li>
    </ul>
  </div>
</section>
<section class="below" id="models">
  <h2>Product information</h2>
  <p>Three torque options: M17-60 (0.65 N·m, 38 W, 470 g), M17-48 (0.55 N·m, 32 W, 360 g), M17-40 (0.42 N·m, 25 W, 285 g). Footprint 42.2 × 42.2 mm.</p>
  {specs_table()}
</section>
<section class="below" id="start">
  <h2>Get started with Python</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="below" id="apps">
  <h2>Customers also use these for</h2>
  <div class="apps">
    <figure style="margin:0"><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
    <figure style="margin:0"><img src="../automation_small.jpg" alt=""><figcaption>CNC</figcaption></figure>
    <figure style="margin:0"><img src="../test_rack_small.jpg" alt=""><figcaption>Test equipment</figcaption></figure>
    <figure style="margin:0"><img src="../kit_with_three_motors_small.jpg" alt=""><figcaption>Education kits</figcaption></figure>
  </div>
</section>
<footer id="about">
  <p>Gearotons · Shenzhen 2022 · Canadian founder · Open source hardware &amp; software</p>
  <p>Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>
{js}"""
    register(15, "Amazon-style", doc(css, body))


def build_v16():
    """TSMC-like: corporate navy/red, multi-pillar technology sections, institutional."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#fff;color:#1a2744}
.nav{background:#0a1628;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw;position:sticky;top:0;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700;letter-spacing:.06em;text-transform:uppercase;font-size:.78rem}
.nav img{height:28px}.nav a{color:#c5d0e0;text-decoration:none;margin-left:1.1rem;font-size:.85rem}
.nav a.shop{background:#c8102e;color:#fff;padding:.45rem 1rem;font-weight:700}
.hero{background:linear-gradient(180deg,#0a1628 0%,#1a2744 100%);color:#fff;padding:4rem 3vw;text-align:center}
.hero h1{font-size:clamp(1.8rem,4vw,2.8rem);font-weight:500;margin:0 0 1rem}
.hero p{max-width:40rem;margin:0 auto 1.5rem;opacity:.9}
.pillars{display:grid;grid-template-columns:repeat(4,1fr);gap:0;border-top:4px solid #c8102e}
.pillar{padding:2rem 1.5rem;border-right:1px solid #dce3ef;background:#f7f9fc}
.pillar:last-child{border-right:0}
.pillar .n{color:#c8102e;font-weight:700;font-size:.75rem;letter-spacing:.12em;text-transform:uppercase}
.pillar h3{margin:.5rem 0;font-weight:500}
.section{padding:3rem 3vw;max-width:1100px;margin:0 auto}
.section h2{border-left:4px solid #c8102e;padding-left:.8rem;font-weight:500}
.two{display:grid;grid-template-columns:1fr 1fr;gap:2rem;align-items:center}
table.specs th,table.specs td{border:1px solid #dce3ef;padding:.55rem;text-align:left;font-size:.88rem}
table.specs thead{background:#0a1628;color:#fff}
pre.code{background:#0a1628;color:#c5e0ff;padding:1.1rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover}
footer{background:#0a1628;color:#c5d0e0;padding:2.5rem 3vw}
footer a{color:#fff;margin-right:1rem}
@media(max-width:900px){.pillars,.two,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#tech">Technology</a><a href="#models">Products</a><a href="#start">Support</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <p style="letter-spacing:.15em;text-transform:uppercase;font-size:.75rem;color:#c8102e;font-weight:700">Manufacturing-grade motion</p>
  <h1>M17 Series — process control at the axis</h1>
  <p>All-in-one servomotors integrating motor, driver, motion controller, and encoder. Built for education and industry with open documentation.</p>
  <a href="{STORE}" style="display:inline-block;background:#c8102e;color:#fff;padding:.65rem 1.4rem;text-decoration:none;font-weight:700">Shop Now</a>
  <a href="{DOCS}" style="display:inline-block;border:1px solid #fff;color:#fff;padding:.65rem 1.4rem;text-decoration:none;margin-left:.5rem">Documentation</a>
</section>
<section class="pillars" id="tech">
  <div class="pillar"><div class="n">Pillar 01</div><h3>Integration</h3><p>Motor + driver + controller + encoder in one package.</p></div>
  <div class="pillar"><div class="n">Pillar 02</div><h3>Interconnect</h3><p>RS-485 daisy-chain — any number of motors, one host.</p></div>
  <div class="pillar"><div class="n">Pillar 03</div><h3>Precision</h3><p>32 kHz PID closed loop. Max 560 RPM. 12–24 V.</p></div>
  <div class="pillar"><div class="n">Pillar 04</div><h3>Compatibility</h3><p>NEMA 17 mount. Pi, Arduino, ESP32, Mac, PC.</p></div>
</section>
<section class="section" id="models">
  <h2>Product family</h2>
  <div class="two" style="margin:1.5rem 0">
    <img src="../M17_series_overview.jpg" alt="M17 Series">
    <div>
      <p><strong>M17-60</strong> — 0.65 N·m · 38 W · 470 g · 59.8 mm</p>
      <p><strong>M17-48</strong> — 0.55 N·m · 32 W · 360 g · 48.6 mm</p>
      <p><strong>M17-40</strong> — 0.42 N·m · 25 W · 285 g · 41.6 mm</p>
      <p>Footprint 42.2 × 42.2 mm across the family.</p>
    </div>
  </div>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Engineering support — first move</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Application domains</h2>
  <div class="apps">
    <figure style="margin:0"><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
    <figure style="margin:0"><img src="../automation_small.jpg" alt=""><figcaption>Automation</figcaption></figure>
    <figure style="margin:0"><img src="../test_rack_small.jpg" alt=""><figcaption>Test systems</figcaption></figure>
  </div>
</section>
<footer id="about">
  <h2 style="color:#fff;margin-top:0">Open source · Gearotons</h2>
  <p>Founded Shenzhen 2022 by a Canadian entrepreneur. Affordable and Simple All-in-One Motion Control.</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(16, "TSMC-style", doc(css, body))


def build_v17():
    """Broadcom-like: dark enterprise header, product TAB UI (models as tabs with JS), dense datasheet."""
    css = """
body{font-family:Arial,Helvetica,sans-serif;background:#f4f4f4;color:#1a1a1a}
.nav{background:#111;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw;position:sticky;top:0;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700;font-size:.9rem}
.nav img{height:26px}.nav a{color:#ccc;text-decoration:none;margin-left:1rem;font-size:.85rem}
.nav a.shop{background:#cc0000;color:#fff;padding:.4rem .9rem;font-weight:700}
.hero{background:#1a1a1a;color:#fff;padding:2.5rem 3vw}
.hero h1{margin:0 0 .5rem;font-size:1.8rem}
.tabs{display:flex;background:#222;border-bottom:3px solid #cc0000}
.tabs button{flex:1;background:transparent;border:0;color:#aaa;padding:1rem;font-weight:700;cursor:pointer;font-size:.95rem}
.tabs button.on{background:#111;color:#fff;border-bottom:3px solid #cc0000;margin-bottom:-3px}
.panel{display:none;background:#fff;padding:2rem 3vw;border:1px solid #ddd;border-top:0}
.panel.on{display:grid;grid-template-columns:1fr 1.2fr;gap:2rem}
.panel h2{margin-top:0;color:#cc0000}
.ds{padding:2rem 3vw;background:#fff;margin-top:1rem;border:1px solid #ddd}
.ds h2{font-size:1.1rem;text-transform:uppercase;letter-spacing:.06em;border-left:4px solid #cc0000;padding-left:.6rem}
table.specs{font-size:.85rem;font-family:Menlo,Consolas,monospace}
table.specs th,table.specs td{border:1px solid #ccc;padding:.5rem;text-align:left}
table.specs th{background:#eee}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:.8rem;margin:1.5rem 0}
.feats article{border-left:3px solid #cc0000;padding:.5rem .8rem;background:#fafafa}
pre.code{background:#111;color:#f88;padding:1rem;font-size:.8rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:.8rem;margin-top:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover}
footer{background:#111;color:#ccc;padding:2rem 3vw;margin-top:1rem}
footer a{color:#f88;margin-right:1rem}
@media(max-width:800px){.panel.on,.feats,.apps{grid-template-columns:1fr}}
"""
    js = """
<script>
(function(){
  var tabs=document.querySelectorAll('.tabs button');
  var panels=document.querySelectorAll('.panel');
  tabs.forEach(function(t,i){
    t.addEventListener('click',function(){
      tabs.forEach(function(x){x.classList.remove('on')});
      panels.forEach(function(x){x.classList.remove('on')});
      t.classList.add('on');
      panels[i].classList.add('on');
    });
  });
})();
</script>
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">GEAROTONS</a>
  <nav>
    <a href="#products">Products</a><a href="#ds">Datasheet</a><a href="#start">Code</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <h1>M17 Series Servomotors — product family</h1>
  <p>All-in-one motion control. Motor, driver, controller, encoder. RS-485 multi-drop. Enterprise-ready documentation, open-source firmware.</p>
  <a href="{STORE}" style="display:inline-block;background:#cc0000;color:#fff;padding:.55rem 1.1rem;text-decoration:none;font-weight:700;margin-top:.5rem">Shop Now</a>
  <a href="{DOCS}" style="color:#f88;margin-left:1rem">Documentation</a>
</section>
<div id="products">
  <div class="tabs">
    <button type="button" class="on">M17-60</button>
    <button type="button">M17-48</button>
    <button type="button">M17-40</button>
  </div>
  <div class="panel on">
    <div><img src="../one_motor_small.jpg" alt="M17-60"><p style="font-size:.9rem;opacity:.8">Highest torque tier</p></div>
    <div>
      <h2>M17-60</h2>
      <p>Rated torque <strong>0.65 N·m</strong> · Power <strong>38 W</strong> · Weight <strong>470 g</strong> · Height <strong>59.8 mm</strong></p>
      <p>Operating voltage 12–24 V · Max speed 560 RPM · Max current 1.1 A · Footprint 42.2 × 42.2 mm</p>
      <p>Same protocol and mounting as M17-48 and M17-40.</p>
    </div>
  </div>
  <div class="panel">
    <div><img src="../one_motor_small.jpg" alt="M17-48"></div>
    <div>
      <h2>M17-48</h2>
      <p>Rated torque <strong>0.55 N·m</strong> · Power <strong>32 W</strong> · Weight <strong>360 g</strong> · Height <strong>48.6 mm</strong></p>
      <p>Operating voltage 12–24 V · Max speed 560 RPM · Max current 1.1 A · Footprint 42.2 × 42.2 mm</p>
    </div>
  </div>
  <div class="panel">
    <div><img src="../one_motor_small.jpg" alt="M17-40"></div>
    <div>
      <h2>M17-40</h2>
      <p>Rated torque <strong>0.42 N·m</strong> · Power <strong>25 W</strong> · Weight <strong>285 g</strong> · Height <strong>41.6 mm</strong></p>
      <p>Operating voltage 12–24 V · Max speed 560 RPM · Max current 1.1 A · Footprint 42.2 × 42.2 mm</p>
    </div>
  </div>
</div>
<section class="ds" id="ds">
  <h2>Feature set</h2>
  <div class="feats">
    {"".join(f"<article><strong>{esc(t)}</strong><br>{esc(d)}</article>" for t,d in FEATURES[:6])}
  </div>
  <h2>Full specification table</h2>
  {specs_table()}
</section>
<section class="ds" id="start">
  <h2>Reference code — Python</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="ds" id="apps">
  <h2>Target applications</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>
{js}"""
    register(17, "Broadcom-style", doc(css, body))


def build_v18():
    """SpaceX-like: sequential full-viewport black sections, huge uppercase, outlined white buttons."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#000;color:#fff}
.nav{position:fixed;top:0;left:0;right:0;z-index:40;display:flex;justify-content:space-between;align-items:center;padding:1rem 3vw;
  background:linear-gradient(180deg,rgba(0,0,0,.8),transparent)}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700;letter-spacing:.2em;text-transform:uppercase;font-size:.75rem}
.nav img{height:24px}.nav a{color:#fff;text-decoration:none;margin-left:1.2rem;font-size:.8rem;letter-spacing:.08em;text-transform:uppercase}
.nav a.shop{border:2px solid #fff;padding:.4rem .9rem}
.vh{min-height:100vh;display:flex;flex-direction:column;justify-content:center;align-items:center;text-align:center;padding:5rem 4vw;position:relative}
.vh h1{font-size:clamp(2.5rem,8vw,5.5rem);font-weight:700;letter-spacing:.04em;text-transform:uppercase;margin:0;line-height:1}
.vh h2{font-size:clamp(1.5rem,4vw,2.8rem);font-weight:600;letter-spacing:.08em;text-transform:uppercase;margin:0 0 1rem}
.vh p{max-width:32rem;opacity:.85;font-size:1.05rem}
.vh img{max-height:45vh;margin:1.5rem 0;filter:drop-shadow(0 20px 40px rgba(0,0,0,.5))}
.obtn{display:inline-block;border:2px solid #fff;color:#fff;padding:.7rem 1.6rem;text-decoration:none;text-transform:uppercase;letter-spacing:.12em;font-size:.8rem;font-weight:700;margin:.4rem}
.obtn.fill{background:#fff;color:#000}
.bg1{background:radial-gradient(ellipse at center,#1a1a1a 0%,#000 70%)}
.bg2{background:#0a0a0a}
.table-wrap{width:min(900px,100%);margin:1.5rem auto 0;text-align:left}
table.specs{font-size:.8rem;text-transform:none;letter-spacing:0}
table.specs th,table.specs td{border-bottom:1px solid #333;padding:.55rem;text-align:left}
pre.code{background:#111;border:1px solid #333;color:#ccc;padding:1.2rem;font-size:.8rem;text-align:left;width:min(640px,100%)}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1px;width:100%;max-width:1000px}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover;opacity:.85}
footer.vh{min-height:50vh}
@media(max-width:700px){.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#mission">Mission</a><a href="#models">Vehicles</a><a href="#start">Code</a>
    <a class="shop" href="{STORE}">Shop</a>
  </nav>
</header>
<section class="vh bg1" id="top">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17">
  <h1>M17 Series</h1>
  <p style="margin:1.2rem 0">Affordable and Simple All-in-One Motion Control</p>
  <div>
    <a class="obtn fill" href="{STORE}">Shop Now</a>
    <a class="obtn" href="{DOCS}">Documentation</a>
  </div>
</section>
<section class="vh bg2" id="mission">
  <h2>Mission brief</h2>
  <p>Integrate motor, driver, motion controller, and encoder. Daisy-chain any number of axes over RS-485 from one connection. Closed-loop PID at 32 kHz. NEMA 17. 12–24 V. Open source.</p>
  <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:2rem;margin-top:2rem;max-width:700px;width:100%">
    <div><div style="font-size:2rem;font-weight:700">32</div><div style="font-size:.75rem;letter-spacing:.1em;opacity:.7">KHZ PID</div></div>
    <div><div style="font-size:2rem;font-weight:700">560</div><div style="font-size:.75rem;letter-spacing:.1em;opacity:.7">RPM MAX</div></div>
    <div><div style="font-size:2rem;font-weight:700">3</div><div style="font-size:.75rem;letter-spacing:.1em;opacity:.7">TORQUE TIERS</div></div>
  </div>
</section>
<section class="vh bg1" id="models">
  <h2>Flight hardware</h2>
  <p>M17-60 · 0.65 N·m · 38 W · 470 g<br>M17-48 · 0.55 N·m · 32 W · 360 g<br>M17-40 · 0.42 N·m · 25 W · 285 g</p>
  <div class="table-wrap">{specs_table()}</div>
</section>
<section class="vh bg2" id="start">
  <h2>Countdown to first move</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="vh bg1" id="apps" style="padding:0;min-height:auto">
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer class="vh bg2" id="about">
  <h2>Open source</h2>
  <p>Gearotons · Shenzhen 2022 · Canadian founder. Firmware, libraries, schematics — public.</p>
  <div>
    <a class="obtn fill" href="{STORE}">Shop Now</a>
    <a class="obtn" href="{DOCS}">Docs</a>
    <a class="obtn" href="{GH}">GitHub</a>
  </div>
</footer>"""
    register(18, "SpaceX-style", doc(css, body))


def build_v19():
    """Aramco-like: deep blue institutional, LEFT sticky side navigation, report-style main column."""
    css = """
body{font-family:Georgia,Times New Roman,serif;background:#f0f4f8;color:#0c2340}
.shell{display:grid;grid-template-columns:240px 1fr;min-height:100vh}
.side{position:sticky;top:0;height:100vh;background:#0c2340;color:#e8eef5;padding:1.5rem 1.2rem;overflow-y:auto}
.side .logo{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-family:Helvetica,Arial,sans-serif;font-weight:700;font-size:.8rem;letter-spacing:.06em;text-transform:uppercase;margin-bottom:2rem}
.side img{height:28px}
.side nav a{display:block;color:#b8c9dc;text-decoration:none;padding:.55rem 0;border-bottom:1px solid rgba(255,255,255,.08);
  font-family:Helvetica,Arial,sans-serif;font-size:.88rem}
.side nav a:hover{color:#fff}
.side .shop{display:block;margin-top:1.5rem;background:#00a3e0;color:#fff;text-align:center;padding:.65rem;text-decoration:none;
  font-family:Helvetica,Arial,sans-serif;font-weight:700}
.main{padding:2.5rem 4vw 4rem;max-width:820px}
.main h1{font-weight:400;font-size:clamp(1.8rem,3.5vw,2.6rem);margin:0 0 1rem}
.main h2{font-weight:400;font-size:1.5rem;margin:2.5rem 0 1rem;border-bottom:2px solid #0c2340;padding-bottom:.4rem}
.main p{font-size:1.05rem;line-height:1.7}
.lead{font-size:1.2rem;color:#3a5068}
table.specs{font-family:Helvetica,Arial,sans-serif;font-size:.85rem}
table.specs th,table.specs td{border:1px solid #c5d0dc;padding:.55rem;text-align:left}
table.specs thead{background:#0c2340;color:#fff}
pre.code{background:#0c2340;color:#d0e8f8;padding:1.1rem;font-size:.82rem;font-family:Menlo,monospace}
.apps{display:grid;grid-template-columns:1fr 1fr;gap:1rem}
.apps img{width:100%;aspect-ratio:4/3;object-fit:cover}
.cta-row{margin:1.5rem 0}
.cta-row a{font-family:Helvetica,Arial,sans-serif;display:inline-block;background:#00a3e0;color:#fff;padding:.6rem 1.2rem;text-decoration:none;font-weight:700;margin-right:.5rem}
@media(max-width:900px){.shell{grid-template-columns:1fr}.side{position:relative;height:auto}}
"""
    body = f"""
<div class="shell">
  <aside class="side">
    <a class="logo" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
    <nav>
      <a href="#top">Overview</a>
      <a href="#features">Capabilities</a>
      <a href="#models">Product data</a>
      <a href="#start">Implementation</a>
      <a href="#apps">Applications</a>
      <a href="#about">About</a>
      <a href="{DOCS}">Documentation</a>
      <a href="{GH}">GitHub</a>
    </nav>
    <a class="shop" href="{STORE}">Shop Now</a>
  </aside>
  <main class="main" id="top">
    <p style="font-family:Helvetica,Arial,sans-serif;font-size:.75rem;letter-spacing:.12em;text-transform:uppercase;color:#00a3e0;font-weight:700">Product brief · M17 Series</p>
    <h1>All-in-one servomotors for reliable multi-axis systems</h1>
    <p class="lead">Affordable and Simple All-in-One Motion Control. Motor, driver, motion controller, and encoder in a single NEMA 17 package, networked over RS-485.</p>
    <div class="cta-row">
      <a href="{STORE}">Shop Now</a>
      <a href="{DOCS}" style="background:#0c2340">Documentation</a>
    </div>
    <img src="../M17_series_overview.jpg" alt="M17 Series" style="margin:1.5rem 0;border:1px solid #c5d0dc">
    <h2 id="features">Capabilities</h2>
    <ul>
      <li>Integrated motor + driver + controller + encoder</li>
      <li>Daisy-chain any number of motors on one RS-485 bus</li>
      <li>High-level commands (trapezoid move) — no STEP/DIR timing</li>
      <li>Closed-loop PID at 32 kHz · max 560 RPM · 12–24 V</li>
      <li>Protection: over-current, over-voltage, over-temperature</li>
      <li>Hosts: Raspberry Pi, Arduino, ESP32, Mac, PC</li>
    </ul>
    <h2 id="models">Product data</h2>
    <p>M17-60 (0.65 N·m) · M17-48 (0.55 N·m) · M17-40 (0.42 N·m). Footprint 42.2 × 42.2 mm.</p>
    {specs_table()}
    <h2 id="start">Implementation</h2>
    <p style="font-family:Helvetica,Arial,sans-serif">pip3 install servomotor</p>
    {code_pre()}
    <h2 id="apps">Applications</h2>
    <div class="apps">
      <img src="../robotics_small.jpg" alt="Robotics">
      <img src="../automation_small.jpg" alt="Automation">
      <img src="../test_rack_small.jpg" alt="Test">
      <img src="../kit_with_three_motors_small.jpg" alt="Education">
    </div>
    <h2 id="about">About Gearotons</h2>
    <p>Gearotons was founded in Shenzhen in 2022 by a Canadian entrepreneur. Firmware, libraries, and schematics are open source.</p>
    <div class="cta-row">
      <a href="{STORE}">Shop Now</a>
      <a href="{GH}" style="background:#0c2340">GitHub</a>
    </div>
  </main>
</div>"""
    register(19, "Aramco-style", doc(css, body))


def build_v20():
    """Meta-like: soft blue/white, large rounded 24px cards, connection-themed network layout."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#f0f2f5;color:#1c1e21}
.nav{position:sticky;top:0;z-index:20;background:#fff;display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw;box-shadow:0 1px 4px rgba(0,0,0,.08)}
.nav img{height:32px}.nav a{text-decoration:none;color:#0668e1;font-weight:600;margin-left:1rem}
.nav a.shop{background:#0668e1;color:#fff;padding:.5rem 1.2rem;border-radius:20px}
.hero{text-align:center;padding:3rem 3vw 1rem}
.hero h1{font-size:clamp(2rem,4vw,3rem);font-weight:700;margin:0 0 .8rem}
.hero p{font-size:1.15rem;color:#65676b;max-width:34rem;margin:0 auto 1.2rem}
.network{max-width:900px;margin:2rem auto;padding:2rem;background:#fff;border-radius:24px;box-shadow:0 4px 20px rgba(0,0,0,.06)}
.nodes{display:flex;flex-wrap:wrap;justify-content:center;align-items:center;gap:1rem}
.node{background:#e7f3ff;border:2px solid #0668e1;border-radius:50%;width:100px;height:100px;display:grid;place-items:center;font-weight:700;font-size:.8rem;text-align:center;color:#0668e1}
.node.host{background:#0668e1;color:#fff;width:120px;height:120px;border-radius:24px}
.line{width:40px;height:3px;background:#0668e1;opacity:.4}
.cards{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:1.2rem;padding:1rem 3vw 3rem;max-width:1100px;margin:0 auto}
.card{background:#fff;border-radius:24px;padding:1.5rem;box-shadow:0 4px 16px rgba(0,0,0,.06)}
.card h3{margin:0 0 .5rem;color:#0668e1}
.section{max-width:900px;margin:0 auto;padding:1rem 3vw 2rem}
.section h2{font-size:1.6rem}
table.specs th,table.specs td{border-bottom:1px solid #e4e6eb;padding:.6rem;text-align:left;font-size:.9rem}
pre.code{background:#1c1e21;color:#e7f3ff;padding:1.1rem;border-radius:16px;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{border-radius:16px;width:100%;aspect-ratio:1;object-fit:cover}
footer{text-align:center;padding:2.5rem 3vw;color:#65676b}
footer a{color:#0668e1;margin:0 .6rem;font-weight:600;text-decoration:none}
@media(max-width:700px){.apps{grid-template-columns:1fr}.line{display:none}}
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="#network">Connect</a><a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="hero" id="top">
  <h1>Connect every axis</h1>
  <p>M17 Series servomotors on one RS-485 network — motor, driver, controller, and encoder in each node.</p>
  <a class="shop" href="{STORE}" style="display:inline-block;background:#0668e1;color:#fff;padding:.6rem 1.4rem;border-radius:20px;text-decoration:none;font-weight:700">Shop Now</a>
  <a href="{DOCS}" style="margin-left:.8rem;color:#0668e1;font-weight:600;text-decoration:none">Documentation</a>
</section>
<section class="network" id="network">
  <h2 style="text-align:center;margin-top:0">Your motion network</h2>
  <div class="nodes">
    <div class="node host">Host<br>Pi / PC</div>
    <div class="line"></div>
    <div class="node">M17-60</div>
    <div class="line"></div>
    <div class="node">M17-48</div>
    <div class="line"></div>
    <div class="node">M17-40</div>
    <div class="line"></div>
    <div class="node">…more</div>
  </div>
  <p style="text-align:center;color:#65676b;margin:1.5rem 0 0">Daisy-chain any number of motors from a single connection point.</p>
</section>
<section class="cards" id="features">
  {"".join(f"<article class='card'><h3>{esc(t)}</h3><p>{esc(d)}</p></article>" for t,d in FEATURES[:6])}
</section>
<section class="section" id="models">
  <h2>Models</h2>
  <p>0.65 / 0.55 / 0.42 N·m · 12–24 V · 560 RPM max · footprint 42.2 × 42.2 mm</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Get started</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Applications</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p>Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(20, "Meta-style", doc(css, body))


def build_v21():
    """Tesla-like: full-bleed 100vh cinematic hero, dual CTAs, horizontal model selector, feature filmstrip."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#fff;color:#171a20}
.nav{position:fixed;top:0;left:0;right:0;z-index:40;display:flex;justify-content:space-between;align-items:center;padding:1rem 3vw}
.nav img{height:22px;filter:brightness(0) invert(1)}.nav a{color:#fff;text-decoration:none;margin-left:1rem;font-size:.85rem;font-weight:500}
.hero{min-height:100vh;background:#111 center/cover no-repeat;display:flex;flex-direction:column;align-items:center;justify-content:space-between;padding:6rem 4vw 3rem;text-align:center;color:#fff;
  background-image:linear-gradient(180deg,rgba(0,0,0,.45),rgba(0,0,0,.55)),url('../M17_series_overview.jpg');background-size:cover;background-position:center}
.hero h1{font-size:clamp(2.5rem,7vw,4.5rem);font-weight:500;margin:0;letter-spacing:-.02em}
.hero p{opacity:.9;font-size:1.15rem}
.dual{display:flex;gap:.8rem;flex-wrap:wrap;justify-content:center;margin-bottom:1rem}
.dual a{min-width:200px;padding:.75rem 1.5rem;text-decoration:none;font-weight:600;border-radius:4px;font-size:.9rem}
.dual .p{background:#3e6ae1;color:#fff}.dual .g{background:rgba(255,255,255,.65);color:#171a20;backdrop-filter:blur(8px)}
.selector{display:flex;justify-content:center;gap:0;border-bottom:1px solid #ddd;position:sticky;top:0;background:#fff;z-index:30}
.selector button{flex:1;max-width:200px;padding:1rem;border:0;background:transparent;font-weight:600;cursor:pointer;border-bottom:3px solid transparent;font-size:.9rem}
.selector button.on{border-bottom-color:#3e6ae1;color:#3e6ae1}
.model-panel{display:none;padding:3rem 4vw;text-align:center}
.model-panel.on{display:block}
.model-panel img{max-height:280px;margin:0 auto 1rem}
.film{display:flex;overflow-x:auto;gap:0;scroll-snap-type:x mandatory}
.film article{min-width:100%;scroll-snap-align:start;display:grid;grid-template-columns:1fr 1fr;min-height:50vh}
.film .t{padding:3rem 4vw;display:flex;flex-direction:column;justify-content:center}
.film img{width:100%;height:100%;object-fit:cover;min-height:280px}
.section{padding:3rem 4vw;max-width:960px;margin:0 auto}
table.specs th,table.specs td{border-bottom:1px solid #eee;padding:.6rem;text-align:left;font-size:.9rem}
pre.code{background:#171a20;color:#eee;padding:1.2rem;border-radius:4px;font-size:.82rem}
footer{background:#171a20;color:#fff;padding:3rem 4vw;text-align:center}
footer a{color:#3e6ae1;margin:0 .7rem;text-decoration:none;font-weight:600}
@media(max-width:800px){.film article{grid-template-columns:1fr}}
"""
    js = """
<script>
(function(){
  var btns=document.querySelectorAll('.selector button');
  var panels=document.querySelectorAll('.model-panel');
  btns.forEach(function(b,i){b.addEventListener('click',function(){
    btns.forEach(function(x){x.classList.remove('on')});
    panels.forEach(function(x){x.classList.remove('on')});
    b.classList.add('on');panels[i].classList.add('on');
  });});
})();
</script>
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo.png" alt="Gearotons"></a>
  <nav>
    <a href="#models">Models</a><a href="#features">Features</a><a href="#start">Code</a>
    <a href="{STORE}" style="font-weight:700">Shop</a>
  </nav>
</header>
<section class="hero" id="top">
  <div>
    <h1>M17 Series</h1>
    <p>Affordable and Simple All-in-One Motion Control</p>
  </div>
  <div class="dual">
    <a class="p" href="{STORE}">Order Now</a>
    <a class="g" href="{DOCS}">Documentation</a>
  </div>
</section>
<div class="selector" id="models">
  <button type="button" class="on">M17-60</button>
  <button type="button">M17-48</button>
  <button type="button">M17-40</button>
</div>
<div class="model-panel on">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17-60">
  <h2>M17-60</h2>
  <p>0.65 N·m · 38 W · 470 g · 59.8 mm height</p>
</div>
<div class="model-panel">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17-48">
  <h2>M17-48</h2>
  <p>0.55 N·m · 32 W · 360 g · 48.6 mm height</p>
</div>
<div class="model-panel">
  <img src="../transparent/one_motor_transparent_small.png" alt="M17-40">
  <h2>M17-40</h2>
  <p>0.42 N·m · 25 W · 285 g · 41.6 mm height</p>
</div>
<div class="film" id="features">
  <article>
    <div class="t"><h2>All-in-one</h2><p>Motor, driver, motion controller, and encoder — one package. No drive cabinet per axis.</p></div>
    <img src="../one_motor_small.jpg" alt="">
  </article>
  <article>
    <div class="t"><h2>One bus</h2><p>Daisy-chain any number of motors over RS-485 from Pi, Arduino, ESP32, Mac, or PC.</p></div>
    <img src="../adapter_and_wire_small.jpg" alt="">
  </article>
  <article>
    <div class="t"><h2>Closed loop</h2><p>32 kHz PID. High-level trapezoid moves. 12–24 V with full protection. Max 560 RPM.</p></div>
    <img src="../motor_back_small.jpg" alt="">
  </article>
</div>
<section class="section">
  <h2>Specifications</h2>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Get started</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Applications</h2>
  <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:.8rem">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <h2 style="margin-top:0">Open source</h2>
  <p>Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>
{js}"""
    register(21, "Tesla-style", doc(css, body))


def build_v22():
    """Samsung-like: OLED black, large product stage, vertical feature list with big numbers."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#000;color:#f5f5f5}
.nav{position:sticky;top:0;z-index:20;background:rgba(0,0,0,.9);display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw;border-bottom:1px solid #222}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700}
.nav img{height:26px}.nav a{color:#aaa;text-decoration:none;margin-left:1.1rem;font-size:.88rem}
.nav a.shop{color:#2189ff;font-weight:700}
.stage{text-align:center;padding:4rem 3vw 2rem}
.stage h1{font-size:clamp(2rem,5vw,3.5rem);font-weight:700;margin:0 0 .5rem}
.stage .blue{color:#2189ff}
.stage img{max-height:50vh;margin:2rem auto}
.flist{max-width:720px;margin:0 auto;padding:2rem 3vw}
.frow{display:grid;grid-template-columns:80px 1fr;gap:1.2rem;padding:1.5rem 0;border-bottom:1px solid #222;align-items:start}
.frow .num{font-size:2.2rem;font-weight:700;color:#2189ff;line-height:1}
.frow h3{margin:0 0 .3rem;font-size:1.2rem}
.section{padding:2.5rem 3vw;max-width:960px;margin:0 auto}
table.specs th,table.specs td{border-bottom:1px solid #333;padding:.55rem;text-align:left;font-size:.88rem}
pre.code{background:#111;border:1px solid #333;color:#7ec8ff;padding:1.1rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover;border-radius:8px}
footer{padding:3rem 3vw;border-top:1px solid #222;text-align:center}
footer a{color:#2189ff;margin:0 .7rem;text-decoration:none;font-weight:600}
@media(max-width:700px){.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#features">Features</a><a href="#models">Models</a><a href="#start">Start</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="stage" id="top">
  <p style="color:#2189ff;font-weight:600;letter-spacing:.1em;text-transform:uppercase;font-size:.8rem">M17 Series</p>
  <h1>Innovation in <span class="blue">every axis</span></h1>
  <p style="opacity:.8;max-width:32rem;margin:0 auto">All-in-one servomotors with closed-loop control, RS-485 daisy-chain, and open-source tools.</p>
  <p style="margin-top:1.2rem">
    <a href="{STORE}" style="display:inline-block;background:#2189ff;color:#fff;padding:.65rem 1.4rem;border-radius:20px;text-decoration:none;font-weight:700">Shop Now</a>
    <a href="{DOCS}" style="margin-left:.8rem;color:#2189ff">Documentation</a>
  </p>
  <img src="../transparent/M17_series_overview_transparent_small.png" alt="M17 Series">
</section>
<section class="flist" id="features">
  <div class="frow"><div class="num">01</div><div><h3>Integrated package</h3><p>Motor + driver + motion controller + encoder in one body.</p></div></div>
  <div class="frow"><div class="num">02</div><div><h3>Multi-motor bus</h3><p>Any number of motors on one RS-485 link from a single host.</p></div></div>
  <div class="frow"><div class="num">03</div><div><h3>High-level motion</h3><p>Trapezoid profiles — no timing-critical STEP/DIR signals.</p></div></div>
  <div class="frow"><div class="num">04</div><div><h3>32 kHz closed loop</h3><p>Encoder + PID. Max 560 RPM. 12–24 V with protection.</p></div></div>
  <div class="frow"><div class="num">05</div><div><h3>NEMA 17 ready</h3><p>Standard mounting, footprint 42.2 × 42.2 mm.</p></div></div>
  <div class="frow"><div class="num">06</div><div><h3>Cross-platform</h3><p>Raspberry Pi, Arduino, ESP32, Mac, and PC.</p></div></div>
</section>
<section class="section" id="models">
  <h2>Models</h2>
  <p>M17-60 · 0.65 N·m · 38 W · 470 g &nbsp;|&nbsp; M17-48 · 0.55 N·m · 32 W · 360 g &nbsp;|&nbsp; M17-40 · 0.42 N·m · 25 W · 285 g</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Get started</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Applications</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p>Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(22, "Samsung-style", doc(css, body))


def build_v23():
    """Berkshire-like: Times serif annual-letter longform, plain HTML table, minimal decoration."""
    css = """
body{font-family:Times New Roman,Times,serif;background:#fff;color:#000;max-width:720px;margin:0 auto;padding:1.5rem 4vw 4rem;font-size:1.05rem;line-height:1.55}
.nav{font-family:Arial,Helvetica,sans-serif;font-size:.85rem;border-bottom:1px solid #000;padding-bottom:.6rem;margin-bottom:1.5rem;display:flex;justify-content:space-between;align-items:center;position:sticky;top:0;background:#fff}
.nav img{height:28px}.nav a{color:#00c;margin-left:.8rem}
h1{font-size:1.6rem;font-weight:700;margin:1.5rem 0 .8rem}
h2{font-size:1.25rem;font-weight:700;margin:2rem 0 .6rem}
p{margin:0 0 .9rem}
a{color:#00c}
table.specs{font-family:Arial,Helvetica,sans-serif;font-size:.85rem;margin:1rem 0}
table.specs th,table.specs td{border:1px solid #000;padding:.4rem .5rem;text-align:left}
pre.code{font-family:Courier New,Courier,monospace;font-size:.85rem;background:#f5f5f5;border:1px solid #ccc;padding:1rem;overflow:auto}
img{max-width:100%;margin:1rem 0}
ul{margin:0 0 1rem}
footer{margin-top:3rem;border-top:1px solid #000;padding-top:1rem;font-family:Arial,Helvetica,sans-serif;font-size:.9rem}
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="#letter">Letter</a>
    <a href="#models">Data</a>
    <a href="{STORE}" style="font-weight:700">Shop Now</a>
  </div>
</header>
<article id="top">
  <p style="font-family:Arial,sans-serif;font-size:.8rem;text-transform:uppercase;letter-spacing:.05em">Gearotons · Product letter · M17 Series</p>
  <h1>To our customers and builders</h1>
  <p id="letter">We design the M17 Series Servomotors so that motion control stays honest and approachable. Each unit integrates a motor, motor driver, motion controller, and encoder. You connect them over RS-485, daisy-chain any number of axes, and speak high-level commands instead of wiring a forest of STEP and DIR signals.</p>
  <p>Affordable and Simple All-in-One Motion Control is not a slogan we invented for decoration — it is the product requirement. Three models share one protocol and one footprint family: M17-60 at 0.65 N·m, M17-48 at 0.55 N·m, and M17-40 at 0.42 N·m.</p>
  <img src="../M17_series_overview.jpg" alt="M17 Series overview">
  <h2>What the product does</h2>
  <ul>
    <li>Closed-loop PID at 32 kHz with a built-in encoder</li>
    <li>Operating voltage 12–24 V; maximum speed 560 RPM</li>
    <li>NEMA 17 mounting; footprint 42.2 × 42.2 mm</li>
    <li>Works with Raspberry Pi, Arduino, ESP32, Mac, and PC</li>
    <li>Over-current, over-voltage, and over-temperature protection</li>
  </ul>
  <h2 id="models">Specifications</h2>
  {specs_table()}
  <h2 id="start">Getting started</h2>
  <p>Install with <code>pip3 install servomotor</code>, then:</p>
  {code_pre()}
  <h2 id="apps">Applications</h2>
  <p>Robotics, CNC and automation, automated testing, scientific instruments, and education.</p>
  <div style="display:grid;grid-template-columns:1fr 1fr;gap:.8rem">
    <img src="../robotics_small.jpg" alt="Robotics" style="margin:0">
    <img src="../automation_small.jpg" alt="Automation" style="margin:0">
    <img src="../test_rack_small.jpg" alt="Test" style="margin:0">
    <img src="../kit_with_three_motors_small.jpg" alt="Kit" style="margin:0">
  </div>
  <h2 id="about">About the company</h2>
  <p>Gearotons was founded in Shenzhen in 2022 by a Canadian entrepreneur. The firmware, libraries, and schematics are open source. Documentation is public at <a href="{DOCS}">gearotons.com</a>; source lives on <a href="{GH}">GitHub</a>.</p>
  <p><a href="{STORE}"><strong>Shop Now</strong></a> · <a href="{DOCS}">Documentation</a> · <a href="{GH}">GitHub</a></p>
</article>
<footer>
  <p>Gearotons — open source hardware &amp; software</p>
</footer>"""
    register(23, "Berkshire-style", doc(css, body))


def build_v24():
    """Eli Lilly-like: medical red/serif, horizontal process pathway steps, clinical calm."""
    css = """
body{font-family:Georgia,Times New Roman,serif;background:#faf8f6;color:#2c1810}
.nav{position:sticky;top:0;z-index:20;background:#fff;border-bottom:1px solid #e8ddd4;display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw}
.nav img{height:32px}.nav a{font-family:Avenir,Helvetica,Arial,sans-serif;text-decoration:none;color:#2c1810;margin-left:1rem;font-size:.88rem}
.nav a.shop{background:#c8102e;color:#fff;padding:.5rem 1.1rem;font-weight:700}
.hero{padding:3.5rem 3vw;max-width:900px;margin:0 auto;text-align:center}
.hero h1{font-weight:400;font-size:clamp(1.9rem,4vw,2.8rem);margin:0 0 1rem}
.hero p{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:1.05rem;color:#5a4a40;max-width:36rem;margin:0 auto 1.5rem}
.path{display:flex;gap:0;max-width:1000px;margin:0 auto 3rem;padding:0 3vw;overflow-x:auto}
.step{flex:1;min-width:160px;background:#fff;border:1px solid #e8ddd4;padding:1.3rem 1rem;text-align:center;position:relative}
.step + .step{{border-left:0}}
.step .n{{display:inline-block;width:36px;height:36px;line-height:36px;border-radius:50%;background:#c8102e;color:#fff;font-family:Avenir,Helvetica,Arial,sans-serif;font-weight:700;margin-bottom:.6rem}}
.step h3{{margin:0 0 .4rem;font-weight:400;font-size:1.05rem}}
.step p{{margin:0;font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.85rem;color:#5a4a40}}
.section{{padding:2rem 3vw;max-width:900px;margin:0 auto}}
.section h2{{font-weight:400;border-bottom:2px solid #c8102e;padding-bottom:.4rem}}
.two{{display:grid;grid-template-columns:1fr 1fr;gap:2rem;align-items:center}}
table.specs{{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.85rem}}
table.specs th,table.specs td{{border-bottom:1px solid #e8ddd4;padding:.55rem;text-align:left}}
pre.code{{background:#2c1810;color:#f5e6dc;padding:1.1rem;font-size:.82rem;font-family:Menlo,monospace}}
.apps{{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}}
.apps img{{width:100%;aspect-ratio:4/3;object-fit:cover}}
footer{{background:#2c1810;color:#f5e6dc;padding:2.5rem 3vw;margin-top:2rem;text-align:center;font-family:Avenir,Helvetica,Arial,sans-serif}}
footer a{{color:#fff;margin:0 .7rem}}
@media(max-width:800px){{.two,.apps{{grid-template-columns:1fr}}}}
"""
    # Fix double braces - I accidentally used {{ in f-string. Need single braces for CSS.
    css = css.replace("{{", "{").replace("}}", "}")
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="#path">Pathway</a><a href="#models">Data</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="hero" id="top">
  <p style="font-family:Avenir,Helvetica,Arial,sans-serif;color:#c8102e;font-weight:700;font-size:.8rem;letter-spacing:.1em;text-transform:uppercase">M17 Series</p>
  <h1>Precision motion, clearly explained</h1>
  <p>All-in-one servomotors for laboratories, classrooms, and machines that need dependable closed-loop control.</p>
  <a href="{STORE}" style="font-family:Avenir,Helvetica,Arial,sans-serif;display:inline-block;background:#c8102e;color:#fff;padding:.6rem 1.3rem;text-decoration:none;font-weight:700">Shop Now</a>
  <a href="{DOCS}" style="font-family:Avenir,Helvetica,Arial,sans-serif;margin-left:1rem;color:#c8102e">Documentation</a>
</section>
<section class="path" id="path">
  <div class="step"><div class="n">1</div><h3>Integrate</h3><p>Motor, driver, controller, encoder — one package.</p></div>
  <div class="step"><div class="n">2</div><h3>Connect</h3><p>RS-485 daisy-chain from a single host.</p></div>
  <div class="step"><div class="n">3</div><h3>Command</h3><p>High-level trapezoid moves in software.</p></div>
  <div class="step"><div class="n">4</div><h3>Verify</h3><p>32 kHz closed loop with LED status.</p></div>
</section>
<section class="section" id="features">
  <div class="two">
    <img src="../one_motor_small.jpg" alt="Motor">
    <div>
      <h2>Key capabilities</h2>
      <ul style="font-family:Avenir,Helvetica,Arial,sans-serif">
        <li>NEMA 17 form factor · 12–24 V</li>
        <li>Max 560 RPM · three torque tiers</li>
        <li>Pi, Arduino, ESP32, Mac, PC</li>
        <li>Over-current / voltage / temperature protection</li>
        <li>Open-source firmware and libraries</li>
      </ul>
    </div>
  </div>
</section>
<section class="section" id="models">
  <h2>Clinical-grade clarity: model data</h2>
  <p style="font-family:Avenir,Helvetica,Arial,sans-serif">M17-60 (0.65 N·m) · M17-48 (0.55 N·m) · M17-40 (0.42 N·m)</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Protocol of first motion</h2>
  <p style="font-family:Avenir,Helvetica,Arial,sans-serif">pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Where it is used</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Gearotons · Shenzhen 2022 · Canadian founder · Open source</p>
  <p>Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(24, "Eli Lilly-style", doc(css, body))


def build_v25():
    """Micron-like: dark blue tech, metrics ribbon at top, dense datasheet."""
    css = """
body{font-family:Arial,Helvetica,sans-serif;background:#0a1628;color:#e0e8f0}
.nav{display:flex;justify-content:space-between;align-items:center;padding:.6rem 3vw;border-bottom:1px solid #1e3a5f;position:sticky;top:0;background:#0a1628;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700;color:#00d4ff}
.nav img{height:26px}.nav a{color:#9ab;text-decoration:none;margin-left:1rem;font-size:.85rem}
.nav a.shop{background:#00d4ff;color:#0a1628;padding:.4rem .9rem;font-weight:700}
.ribbon{display:grid;grid-template-columns:repeat(5,1fr);background:#061020;border-bottom:1px solid #1e3a5f}
.ribbon div{padding:1rem 3vw;text-align:center;border-right:1px solid #1e3a5f}
.ribbon div:last-child{border-right:0}
.ribbon b{display:block;font-size:1.3rem;color:#00d4ff}
.ribbon span{font-size:.7rem;letter-spacing:.08em;text-transform:uppercase;opacity:.7}
.hero{padding:2.5rem 3vw;display:grid;grid-template-columns:1fr 1fr;gap:2rem;align-items:center}
.hero h1{font-size:clamp(1.6rem,3vw,2.4rem);margin:0 0 .8rem}
.ds{padding:1.5rem 3vw}
.ds h2{font-size:.85rem;letter-spacing:.12em;text-transform:uppercase;color:#00d4ff;border-bottom:1px solid #1e3a5f;padding-bottom:.4rem}
table.specs{font-family:Menlo,Consolas,monospace;font-size:.8rem;margin-top:.8rem}
table.specs th,table.specs td{border:1px solid #1e3a5f;padding:.45rem;text-align:left}
table.specs th{background:#061020;color:#00d4ff}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:1.5rem;padding:0 3vw 2rem}
pre.code{background:#061020;border:1px solid #1e3a5f;color:#7fefdf;padding:1rem;font-size:.8rem}
.apps{display:grid;grid-template-columns:repeat(4,1fr);gap:1px;background:#1e3a5f}
.apps img{width:100%;aspect-ratio:1;object-fit:cover;display:block}
footer{padding:2rem 3vw;border-top:1px solid #1e3a5f}
footer a{color:#00d4ff;margin-right:1rem}
@media(max-width:900px){.ribbon{grid-template-columns:1fr 1fr}.hero,.grid2,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">GEAROTONS</a>
  <nav>
    <a href="#ds">Datasheet</a><a href="#start">Code</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<div class="ribbon" id="top">
  <div><b>12–24V</b><span>Voltage</span></div>
  <div><b>560</b><span>RPM max</span></div>
  <div><b>32 kHz</b><span>PID loop</span></div>
  <div><b>1.1 A</b><span>Max current</span></div>
  <div><b>3</b><span>Models</span></div>
</div>
<section class="hero">
  <div>
    <h1>M17 Series performance brief</h1>
    <p>All-in-one servomotor density: motor, driver, controller, encoder. RS-485 multi-drop. Open documentation.</p>
    <p><a href="{STORE}" style="background:#00d4ff;color:#0a1628;padding:.55rem 1.1rem;text-decoration:none;font-weight:700">Shop Now</a>
    <a href="{DOCS}" style="color:#00d4ff;margin-left:.8rem">Documentation</a></p>
  </div>
  <img src="../transparent/one_motor_transparent_small.png" alt="M17">
</section>
<section class="ds" id="ds">
  <h2>Electrical &amp; mechanical table</h2>
  <p style="font-size:.9rem;opacity:.85">M17-60 · 0.65 N·m · 38 W · 470 g · 59.8 mm &nbsp;|&nbsp; M17-48 · 0.55 N·m · 32 W · 360 g · 48.6 mm &nbsp;|&nbsp; M17-40 · 0.42 N·m · 25 W · 285 g · 41.6 mm</p>
  {specs_table()}
</section>
<section class="ds" id="features">
  <h2>Feature density</h2>
  <div style="display:grid;grid-template-columns:1fr 1fr;gap:.6rem;font-size:.9rem">
    {"".join(f"<div style='border:1px solid #1e3a5f;padding:.7rem'><strong style='color:#00d4ff'>{esc(t)}</strong><br>{esc(d)}</div>" for t,d in FEATURES[:6])}
  </div>
</section>
<section class="grid2" id="start">
  <div>
    <h2 style="font-size:.85rem;letter-spacing:.1em;text-transform:uppercase;color:#00d4ff">Bring-up code</h2>
    <p>pip3 install servomotor</p>
    {code_pre()}
  </div>
  <div>
    <h2 style="font-size:.85rem;letter-spacing:.1em;text-transform:uppercase;color:#00d4ff">Dimensions</h2>
    <img src="../M17-60_dimensions.png" alt="Dimensions" style="background:#fff;padding:.5rem">
  </div>
</section>
<section class="apps" id="apps">
  <img src="../robotics_small.jpg" alt="Robotics">
  <img src="../automation_small.jpg" alt="Automation">
  <img src="../test_rack_small.jpg" alt="Test">
  <img src="../kit_with_three_motors_small.jpg" alt="Kit">
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(25, "Micron-style", doc(css, body))


def build_v26():
    """Walmart-like: blue/yellow retail, category aisle grid of applications, value-forward."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#fff;color:#2e2f32}
.nav{background:#0071dc;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw;position:sticky;top:0;z-index:20}
.nav img{height:32px;background:#fff;border-radius:50%;padding:2px}
.nav a{color:#fff;text-decoration:none;margin-left:1rem;font-weight:600;font-size:.9rem}
.nav a.shop{background:#ffc220;color:#2e2f32;padding:.5rem 1.1rem;border-radius:999px}
.hero{background:#e6f1fc;padding:2.5rem 3vw;display:grid;grid-template-columns:1.2fr .8fr;gap:2rem;align-items:center}
.hero h1{font-size:clamp(1.8rem,4vw,2.6rem);margin:0 0 .8rem;color:#0071dc}
.hero .price-ish{font-size:1.1rem;font-weight:700}
.aisles{padding:2rem 3vw}
.aisles h2{margin:0 0 1rem}
.aisle-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:1rem}
.aisle{background:#f5f6f8;border-radius:12px;overflow:hidden;text-align:center;padding-bottom:1rem}
.aisle img{width:100%;aspect-ratio:1;object-fit:cover}
.aisle h3{margin:.8rem 0 .3rem;font-size:1rem}
.aisle p{margin:0;font-size:.85rem;opacity:.8;padding:0 .5rem}
.section{padding:1.5rem 3vw}
.models{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.models article{border:2px solid #0071dc;border-radius:12px;padding:1.2rem}
.models h3{color:#0071dc;margin:0 0 .4rem}
table.specs th,table.specs td{border:1px solid #e0e0e0;padding:.5rem;text-align:left;font-size:.88rem}
pre.code{background:#2e2f32;color:#ffc220;padding:1rem;border-radius:8px;font-size:.82rem}
footer{background:#041e42;color:#fff;padding:2.5rem 3vw;text-align:center}
footer a{color:#ffc220;margin:0 .7rem;font-weight:700}
@media(max-width:800px){.hero,.aisle-grid,.models{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a href="#top" style="display:flex;gap:.5rem;align-items:center;text-decoration:none;color:#fff;font-weight:800">
    <img src="../Gearotons_Logo.png" alt="">Gearotons
  </a>
  <div>
    <a href="#aisles">Shop aisles</a><a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="hero" id="top">
  <div>
    <h1>Roll back complexity. Keep the torque.</h1>
    <p>M17 Series all-in-one servomotors — motor, driver, controller, encoder together. Daisy-chain over RS-485. Three sizes for torque and budget.</p>
    <p class="price-ish">Affordable and Simple All-in-One Motion Control</p>
    <p style="margin-top:1rem">
      <a href="{STORE}" style="background:#ffc220;color:#2e2f32;padding:.65rem 1.3rem;border-radius:999px;text-decoration:none;font-weight:800">Shop Now</a>
      <a href="{DOCS}" style="margin-left:.8rem;color:#0071dc;font-weight:600">Documentation</a>
    </p>
  </div>
  <img src="../kit_with_three_motors_small.jpg" alt="Kit" style="border-radius:12px">
</section>
<section class="aisles" id="aisles">
  <h2>Aisles — pick your project</h2>
  <div class="aisle-grid">
    <div class="aisle"><img src="../robotics_small.jpg" alt=""><h3>Robotics</h3><p>Closed-loop axes for arms and drives</p></div>
    <div class="aisle"><img src="../automation_small.jpg" alt=""><h3>CNC &amp; automation</h3><p>Machines that need honest motion</p></div>
    <div class="aisle"><img src="../test_rack_small.jpg" alt=""><h3>Test equipment</h3><p>Racks of repeatable motion</p></div>
    <div class="aisle"><img src="../kit_with_three_motors_small.jpg" alt=""><h3>Education</h3><p>Open docs for classrooms</p></div>
  </div>
</section>
<section class="section" id="features">
  <h2>Why shoppers pick M17</h2>
  <ul>
    <li>All-in-one package — fewer boxes, less wiring</li>
    <li>One RS-485 bus for any number of motors</li>
    <li>High-level Python and Arduino commands</li>
    <li>NEMA 17 mounting · 12–24 V · 560 RPM max · 32 kHz PID</li>
  </ul>
</section>
<section class="section" id="models">
  <h2>Compare models</h2>
  <div class="models">
    <article><h3>M17-60</h3><p><strong>0.65 N·m</strong><br>38 W · 470 g · 59.8 mm</p></article>
    <article><h3>M17-48</h3><p><strong>0.55 N·m</strong><br>32 W · 360 g · 48.6 mm</p></article>
    <article><h3>M17-40</h3><p><strong>0.42 N·m</strong><br>25 W · 285 g · 41.6 mm</p></article>
  </div>
  <div style="margin-top:1.5rem">{specs_table()}</div>
</section>
<section class="section" id="start">
  <h2>Get started</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(26, "Walmart-style", doc(css, body))


def build_v27():
    """JPMorgan-like: navy finance, top KPI strip, multi-column report."""
    css = """
body{font-family:Georgia,Times New Roman,serif;background:#f5f6f8;color:#0d1b2a}
.nav{background:#0d1b2a;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw;position:sticky;top:0;z-index:20;font-family:Helvetica,Arial,sans-serif}
.nav img{height:28px}.nav a{color:#c5d0dc;text-decoration:none;margin-left:1rem;font-size:.85rem}
.nav a.shop{background:#c4a35a;color:#0d1b2a;padding:.4rem .9rem;font-weight:700}
.kpi{display:grid;grid-template-columns:repeat(4,1fr);background:#fff;border-bottom:1px solid #d0d5dd}
.kpi div{padding:1.2rem 3vw;border-right:1px solid #d0d5dd;font-family:Helvetica,Arial,sans-serif}
.kpi div:last-child{border-right:0}
.kpi b{display:block;font-size:1.5rem;color:#0d1b2a}
.kpi span{font-size:.75rem;text-transform:uppercase;letter-spacing:.06em;color:#5a6a7a}
.cols{display:grid;grid-template-columns:2fr 1fr;gap:2rem;padding:2.5rem 3vw;max-width:1100px;margin:0 auto}
.main h1{font-weight:400;font-size:2rem;margin-top:0}
.side-card{background:#fff;border-top:4px solid #0d1b2a;padding:1.2rem;margin-bottom:1rem;font-family:Helvetica,Arial,sans-serif;font-size:.9rem}
table.specs{font-family:Helvetica,Arial,sans-serif;font-size:.85rem;background:#fff}
table.specs th,table.specs td{border:1px solid #d0d5dd;padding:.5rem;text-align:left}
pre.code{background:#0d1b2a;color:#e8eef5;padding:1rem;font-size:.8rem;font-family:Menlo,monospace}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem;padding:0 3vw 2rem;max-width:1100px;margin:0 auto}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover}
footer{background:#0d1b2a;color:#c5d0dc;padding:2rem 3vw;font-family:Helvetica,Arial,sans-serif}
footer a{color:#c4a35a;margin-right:1rem}
@media(max-width:900px){.kpi{grid-template-columns:1fr 1fr}.cols,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a href="#top" style="display:flex;gap:.5rem;align-items:center;text-decoration:none;color:#fff;font-weight:700">
    <img src="../Gearotons_Logo.png" alt="">Gearotons
  </a>
  <nav>
    <a href="#report">Overview</a><a href="#models">Data</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<div class="kpi" id="top">
  <div><b>0.65 N·m</b><span>Peak tier torque</span></div>
  <div><b>560 RPM</b><span>Maximum speed</span></div>
  <div><b>12–24 V</b><span>Operating voltage</span></div>
  <div><b>32 kHz</b><span>PID frequency</span></div>
</div>
<div class="cols" id="report">
  <div class="main">
    <h1>M17 Series investment in simpler motion</h1>
    <p>The M17 Series Servomotors consolidate motor, driver, motion controller, and encoder into one package. Multi-axis systems share a single RS-485 bus. High-level commands replace timing-critical STEP/DIR wiring.</p>
    <p>Affordable and Simple All-in-One Motion Control — for education and industry. Open-source firmware and libraries reduce long-term integration risk.</p>
    <img src="../M17_series_overview.jpg" alt="Series" style="margin:1rem 0;border:1px solid #d0d5dd">
    <h2 id="features" style="font-weight:400">Operating highlights</h2>
    <ul>
      <li>NEMA 17 mounting · footprint 42.2 × 42.2 mm</li>
      <li>Three models: 0.65 / 0.55 / 0.42 N·m</li>
      <li>Hosts: Raspberry Pi, Arduino, ESP32, Mac, PC</li>
      <li>Protection: over-current, over-voltage, over-temperature</li>
    </ul>
    <h2 id="models" style="font-weight:400">Model table</h2>
    {specs_table()}
    <h2 id="start" style="font-weight:400">Implementation note</h2>
    <p style="font-family:Helvetica,Arial,sans-serif;font-size:.9rem">pip3 install servomotor</p>
    {code_pre()}
  </div>
  <aside>
    <div class="side-card">
      <strong>Action</strong>
      <p><a href="{STORE}" style="display:block;background:#c4a35a;color:#0d1b2a;text-align:center;padding:.6rem;text-decoration:none;font-weight:700;margin-top:.5rem">Shop Now</a></p>
      <p><a href="{DOCS}">Documentation →</a></p>
    </div>
    <div class="side-card">
      <strong>Company</strong>
      <p>Gearotons · Shenzhen 2022 · Canadian founder · Open source</p>
    </div>
    <div class="side-card">
      <strong>Models</strong>
      <p>M17-60 · 38 W · 470 g<br>M17-48 · 32 W · 360 g<br>M17-40 · 25 W · 285 g</p>
    </div>
  </aside>
</div>
<section class="apps" id="apps">
  <img src="../robotics_small.jpg" alt="Robotics">
  <img src="../automation_small.jpg" alt="Automation">
  <img src="../test_rack_small.jpg" alt="Test">
</section>
<footer id="about">
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(27, "JPMorgan-style", doc(css, body))


def build_v28():
    """SK Hynix-like: black/red, product family comparison cards side by side."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#fff;color:#111}
.nav{background:#000;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw;position:sticky;top:0;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700}
.nav img{height:26px}.nav a{color:#ccc;text-decoration:none;margin-left:1rem;font-size:.88rem}
.nav a.shop{background:#e60012;color:#fff;padding:.45rem 1rem;font-weight:700}
.hero{padding:3rem 3vw;text-align:center;background:#f7f7f7}
.hero h1{font-size:clamp(1.8rem,4vw,2.8rem);margin:0 0 .6rem}
.family{display:grid;grid-template-columns:repeat(3,1fr);gap:0;max-width:1100px;margin:0 auto;border:1px solid #ddd}
.card{padding:2rem 1.5rem;border-right:1px solid #ddd;text-align:center;background:#fff}
.card:last-child{border-right:0}
.card.feat{border-top:4px solid #e60012}
.card h2{margin:0 0 .5rem;font-size:1.4rem}
.card .t{font-size:2rem;font-weight:700;color:#e60012;margin:.5rem 0}
.card ul{text-align:left;font-size:.9rem;padding-left:1.1rem}
.card img{max-height:160px;margin:0 auto 1rem}
.section{padding:2.5rem 3vw;max-width:1000px;margin:0 auto}
table.specs th,table.specs td{border:1px solid #ddd;padding:.5rem;text-align:left;font-size:.88rem}
table.specs thead{background:#000;color:#fff}
pre.code{background:#111;color:#f88;padding:1rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover}
footer{background:#000;color:#ccc;padding:2rem 3vw;text-align:center}
footer a{color:#e60012;margin:0 .7rem;font-weight:700}
@media(max-width:800px){.family,.apps{grid-template-columns:1fr}.card{border-right:0;border-bottom:1px solid #ddd}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#family">Family</a><a href="#models">Specs</a><a href="#start">Code</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <h1>M17 product family</h1>
  <p>One architecture. Three torque points. Same software path.</p>
  <p><a href="{STORE}" style="display:inline-block;background:#e60012;color:#fff;padding:.6rem 1.3rem;text-decoration:none;font-weight:700">Shop Now</a>
  <a href="{DOCS}" style="margin-left:.8rem;color:#e60012;font-weight:600">Documentation</a></p>
</section>
<section class="family" id="family">
  <article class="card">
    <img src="../transparent/one_motor_transparent_small.png" alt="M17-40">
    <h2>M17-40</h2>
    <div class="t">0.42 N·m</div>
    <ul>
      <li>25 W rated power</li>
      <li>285 g · 41.6 mm height</li>
      <li>12–24 V · 560 RPM max</li>
      <li>Compact pick</li>
    </ul>
  </article>
  <article class="card feat">
    <img src="../transparent/one_motor_transparent_small.png" alt="M17-48">
    <h2>M17-48</h2>
    <div class="t">0.55 N·m</div>
    <ul>
      <li>32 W rated power</li>
      <li>360 g · 48.6 mm height</li>
      <li>12–24 V · 560 RPM max</li>
      <li>Balanced pick</li>
    </ul>
  </article>
  <article class="card">
    <img src="../transparent/one_motor_transparent_small.png" alt="M17-60">
    <h2>M17-60</h2>
    <div class="t">0.65 N·m</div>
    <ul>
      <li>38 W rated power</li>
      <li>470 g · 59.8 mm height</li>
      <li>12–24 V · 560 RPM max</li>
      <li>Highest torque</li>
    </ul>
  </article>
</section>
<section class="section" id="features">
  <h2>Shared technology</h2>
  <p>Motor + driver + controller + encoder · RS-485 daisy-chain · 32 kHz PID · NEMA 17 · footprint 42.2 × 42.2 mm · Pi / Arduino / ESP32 / Mac / PC · open source</p>
</section>
<section class="section" id="models">
  <h2>Full comparison</h2>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Get started</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Applications</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Gearotons · Shenzhen 2022 · Canadian founder · Open source</p>
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(28, "SK Hynix-style", doc(css, body))


def build_v29():
    """AMD-like: black/red aggressive, product stack cards with big torque numbers."""
    css = """
body{font-family:Arial Black,Arial,Helvetica,sans-serif;background:#000;color:#fff}
.nav{display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw;border-bottom:1px solid #222;position:sticky;top:0;background:#000;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-size:.85rem;letter-spacing:.06em}
.nav img{height:26px}.nav a{color:#aaa;text-decoration:none;margin-left:1rem;font-family:Arial,sans-serif;font-weight:700;font-size:.85rem}
.nav a.shop{background:#ed1c24;color:#fff;padding:.45rem 1rem}
.hero{padding:3rem 3vw;background:radial-gradient(ellipse at 20% 0%,#3a0000 0%,#000 55%)}
.hero h1{font-size:clamp(2.2rem,6vw,4rem);margin:0;line-height:.95;text-transform:uppercase}
.hero p{font-family:Arial,sans-serif;font-weight:400;max-width:36rem;opacity:.85;font-size:1.05rem}
.stack{padding:1rem 3vw 3rem;display:flex;flex-direction:column;gap:1rem}
.scard{display:grid;grid-template-columns:200px 1fr auto;gap:1.5rem;align-items:center;background:#111;border-left:6px solid #ed1c24;padding:1.5rem;font-family:Arial,sans-serif}
.scard .big{font-family:Arial Black,Arial,sans-serif;font-size:clamp(2rem,4vw,3rem);color:#ed1c24;line-height:1}
.scard h2{margin:0 0 .3rem;font-size:1.3rem;font-family:Arial Black,Arial,sans-serif}
.section{padding:2rem 3vw;font-family:Arial,sans-serif}
table.specs{font-size:.85rem}table.specs th,table.specs td{border-bottom:1px solid #333;padding:.5rem;text-align:left}
pre.code{background:#111;border:1px solid #ed1c24;color:#f88;padding:1rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover}
footer{padding:2.5rem 3vw;border-top:1px solid #222;font-family:Arial,sans-serif}
footer a{color:#ed1c24;margin-right:1rem;font-weight:700}
@media(max-width:800px){.scard{grid-template-columns:1fr}.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">GEAROTONS</a>
  <nav>
    <a href="#stack">Stack</a><a href="#models">Specs</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <h1>Push every<br>axis harder</h1>
  <p>M17 Series all-in-one servomotors. Closed-loop torque without a rack of drives. Daisy-chain RS-485. Open source.</p>
  <p style="margin-top:1.2rem">
    <a href="{STORE}" style="display:inline-block;background:#ed1c24;color:#fff;padding:.7rem 1.4rem;text-decoration:none;font-family:Arial,sans-serif;font-weight:800">Shop Now</a>
    <a href="{DOCS}" style="margin-left:1rem;color:#ed1c24;font-family:Arial,sans-serif;font-weight:700">Documentation</a>
  </p>
</section>
<section class="stack" id="stack">
  <article class="scard">
    <div class="big">0.65</div>
    <div><h2>M17-60</h2><p>N·m rated torque · 38 W · 470 g · 59.8 mm · highest tier</p></div>
    <img src="../transparent/one_motor_transparent_small.png" alt="" style="max-height:100px">
  </article>
  <article class="scard">
    <div class="big">0.55</div>
    <div><h2>M17-48</h2><p>N·m rated torque · 32 W · 360 g · 48.6 mm · balanced</p></div>
    <img src="../transparent/one_motor_transparent_small.png" alt="" style="max-height:100px">
  </article>
  <article class="scard">
    <div class="big">0.42</div>
    <div><h2>M17-40</h2><p>N·m rated torque · 25 W · 285 g · 41.6 mm · compact</p></div>
    <img src="../transparent/one_motor_transparent_small.png" alt="" style="max-height:100px">
  </article>
</section>
<section class="section" id="features">
  <h2 style="font-family:Arial Black,Arial,sans-serif;text-transform:uppercase">Built in</h2>
  <p>Motor + driver + controller + encoder · 32 kHz PID · 12–24 V · 560 RPM max · NEMA 17 · high-level trapezoid moves · Pi / Arduino / ESP32 / Mac / PC</p>
</section>
<section class="section" id="models">
  <h2 style="font-family:Arial Black,Arial,sans-serif;text-transform:uppercase">Full specs</h2>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2 style="font-family:Arial Black,Arial,sans-serif;text-transform:uppercase">Code</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2 style="font-family:Arial Black,Arial,sans-serif;text-transform:uppercase">Deploy</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(29, "AMD-style", doc(css, body))


def build_v30():
    """Visa-like: deep blue + gold, horizontal 4-step how-it-works process."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#fff;color:#1a1f71}
.nav{background:#1a1f71;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw;position:sticky;top:0;z-index:20}
.nav img{height:28px;background:#fff;border-radius:2px;padding:2px}
.nav a{color:#fff;text-decoration:none;margin-left:1rem;font-size:.9rem;font-weight:500}
.nav a.shop{background:#f7b600;color:#1a1f71;padding:.45rem 1rem;font-weight:700;border-radius:4px}
.hero{background:linear-gradient(135deg,#1a1f71,#0f1350);color:#fff;padding:3.5rem 3vw;text-align:center}
.hero h1{font-size:clamp(1.9rem,4vw,2.8rem);margin:0 0 .8rem}
.steps{display:grid;grid-template-columns:repeat(4,1fr);gap:0;max-width:1100px;margin:-2rem auto 2rem;padding:0 3vw;position:relative;z-index:2}
.step{background:#fff;border:1px solid #e0e3f0;padding:1.5rem 1.2rem;text-align:center;box-shadow:0 8px 24px rgba(26,31,113,.08)}
.step .n{width:40px;height:40px;line-height:40px;border-radius:50%;background:#1a1f71;color:#fff;font-weight:700;margin:0 auto .8rem}
.step h3{margin:0 0 .4rem;font-size:1rem}
.step p{margin:0;font-size:.88rem;color:#444}
.section{padding:2rem 3vw;max-width:1000px;margin:0 auto}
table.specs th,table.specs td{border-bottom:1px solid #e0e3f0;padding:.55rem;text-align:left;font-size:.9rem}
pre.code{background:#1a1f71;color:#f7e8b0;padding:1.1rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover;border-radius:8px}
footer{background:#1a1f71;color:#fff;padding:2.5rem 3vw;text-align:center}
footer a{color:#f7b600;margin:0 .7rem;font-weight:700}
@media(max-width:800px){.steps,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a href="#top" style="display:flex;gap:.5rem;align-items:center;text-decoration:none;color:#fff;font-weight:700">
    <img src="../Gearotons_Logo.png" alt="">Gearotons
  </a>
  <div>
    <a href="#how">How it works</a><a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="hero" id="top">
  <h1>Motion accepted everywhere your bus goes</h1>
  <p style="max-width:36rem;margin:0 auto 1.2rem;opacity:.9">M17 Series — all-in-one servomotors. One protocol. Three torque tiers. Open source.</p>
  <a href="{STORE}" style="display:inline-block;background:#f7b600;color:#1a1f71;padding:.65rem 1.4rem;text-decoration:none;font-weight:700;border-radius:4px">Shop Now</a>
  <a href="{DOCS}" style="margin-left:1rem;color:#f7b600;font-weight:600">Documentation</a>
</section>
<section class="steps" id="how">
  <div class="step"><div class="n">1</div><h3>Mount</h3><p>NEMA 17 standard. Footprint 42.2 × 42.2 mm.</p></div>
  <div class="step"><div class="n">2</div><h3>Wire</h3><p>Power 12–24 V. Daisy-chain RS-485.</p></div>
  <div class="step"><div class="n">3</div><h3>Install</h3><p>pip3 install servomotor or Arduino library.</p></div>
  <div class="step"><div class="n">4</div><h3>Move</h3><p>Trapezoid move — one revolution in seconds.</p></div>
</section>
<section class="section" id="features">
  <h2>Included in every unit</h2>
  <p>Motor + driver + motion controller + encoder · 32 kHz PID · over-current / voltage / temperature protection · max 560 RPM · hosts from Pi to PC</p>
  <img src="../connection_diagram.jpg" alt="Connection" style="margin:1rem 0;border:1px solid #e0e3f0">
</section>
<section class="section" id="models">
  <h2>Choose a tier</h2>
  <p>M17-60 · 0.65 N·m · 38 W · 470 g &nbsp;·&nbsp; M17-48 · 0.55 N·m · 32 W · 360 g &nbsp;·&nbsp; M17-40 · 0.42 N·m · 25 W · 285 g</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Sample authorization — Python</h2>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Accepted use cases</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Gearotons · Shenzhen 2022 · Canadian founder · Open source</p>
  <p>Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(30, "Visa-style", doc(css, body))


def build_v31():
    """ASML-like: near-black precision, light font weights, deep technical sections."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#0c0c0e;color:#e8e8ea;font-weight:300}
.nav{position:sticky;top:0;z-index:20;background:rgba(12,12,14,.95);border-bottom:1px solid #2a2a30;display:flex;justify-content:space-between;align-items:center;padding:.75rem 4vw}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:400;letter-spacing:.12em;text-transform:uppercase;font-size:.72rem}
.nav img{height:24px}.nav a{color:#aaa;text-decoration:none;margin-left:1.2rem;font-size:.82rem;font-weight:300}
.nav a.shop{color:#ff6a00;font-weight:400}
.hero{padding:6rem 4vw 4rem;max-width:900px}
.hero h1{font-weight:200;font-size:clamp(2rem,5vw,3.4rem);letter-spacing:-.02em;margin:0 0 1.2rem;line-height:1.15}
.hero p{font-size:1.1rem;opacity:.8;max-width:36rem;line-height:1.7}
.tech{padding:3rem 4vw;border-top:1px solid #2a2a30}
.tech h2{font-weight:300;font-size:1.6rem;margin:0 0 1.5rem}
.tech-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:2rem}
.tech-grid h3{font-weight:400;font-size:.85rem;letter-spacing:.1em;text-transform:uppercase;color:#ff6a00;margin:0 0 .6rem}
.tech-grid p{margin:0;opacity:.8;font-size:.95rem;line-height:1.6}
.deep{padding:3rem 4vw;display:grid;grid-template-columns:1fr 1fr;gap:3rem;border-top:1px solid #2a2a30;align-items:start}
table.specs{font-size:.82rem;font-weight:300}
table.specs th,table.specs td{border-bottom:1px solid #2a2a30;padding:.6rem .4rem;text-align:left;font-weight:300}
pre.code{background:#000;border:1px solid #2a2a30;color:#c8c8cc;padding:1.2rem;font-size:.8rem;font-weight:400}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1px;background:#2a2a30;border-top:1px solid #2a2a30}
.apps figure{margin:0;background:#0c0c0e}.apps img{width:100%;aspect-ratio:16/10;object-fit:cover;opacity:.8}
.apps figcaption{padding:.5rem .8rem;font-size:.8rem;font-weight:300;opacity:.7}
footer{padding:3rem 4vw;border-top:1px solid #2a2a30;font-weight:300}
footer a{color:#ff6a00;margin-right:1.2rem;text-decoration:none}
@media(max-width:900px){.tech-grid,.deep,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#tech">Systems</a><a href="#models">Data</a><a href="#start">Software</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <p style="font-size:.75rem;letter-spacing:.16em;text-transform:uppercase;color:#ff6a00;margin:0 0 1rem">Precision motion systems</p>
  <h1>Enabling closed-loop control at the point of motion</h1>
  <p>The M17 Series integrates motor, driver, motion controller, and encoder — a complete axis subsystem in a NEMA 17 envelope, networked over RS-485.</p>
  <p style="margin-top:1.5rem">
    <a href="{STORE}" style="color:#ff6a00;text-decoration:none;border-bottom:1px solid #ff6a00;padding-bottom:.15rem">Shop Now</a>
    <a href="{DOCS}" style="margin-left:1.5rem;color:#aaa;text-decoration:none">Documentation</a>
  </p>
</section>
<section class="tech" id="tech">
  <h2>System architecture</h2>
  <div class="tech-grid">
    <div><h3>Integration layer</h3><p>Motor, driver, controller, and encoder share one mechanical package — no external drive cabinet per axis.</p></div>
    <div><h3>Interconnect layer</h3><p>RS-485 multi-drop bus. Daisy-chain any number of motors from a single host connection.</p></div>
    <div><h3>Control layer</h3><p>32 kHz PID closed loop. High-level trapezoid profiles. 12–24 V. Max 560 RPM.</p></div>
  </div>
</section>
<section class="deep" id="models">
  <div>
    <h2 style="font-weight:300;margin-top:0">Model portfolio</h2>
    <p style="opacity:.8">M17-60 · 0.65 N·m · 38 W · 470 g · 59.8 mm<br>
    M17-48 · 0.55 N·m · 32 W · 360 g · 48.6 mm<br>
    M17-40 · 0.42 N·m · 25 W · 285 g · 41.6 mm</p>
    <p style="opacity:.8">Footprint 42.2 × 42.2 mm · NEMA 17 mounting</p>
    <img src="../M17-60_dimensions.png" alt="Dimensions" style="margin-top:1rem;background:#fff;padding:.5rem">
  </div>
  <div>{specs_table()}</div>
</section>
<section class="tech" id="start">
  <h2>Software bring-up</h2>
  <p style="opacity:.8">pip3 install servomotor · hosts: Raspberry Pi, Arduino, ESP32, Mac, PC</p>
  {code_pre()}
</section>
<section class="apps" id="apps">
  <figure><img src="../robotics_small.jpg" alt=""><figcaption>Robotics</figcaption></figure>
  <figure><img src="../automation_small.jpg" alt=""><figcaption>Automation equipment</figcaption></figure>
  <figure><img src="../test_rack_small.jpg" alt=""><figcaption>Test systems</figcaption></figure>
</section>
<footer id="about">
  <p>Open source hardware &amp; software · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
  <p style="opacity:.5;margin-top:1rem;font-size:.9rem">Affordable and Simple All-in-One Motion Control</p>
</footer>"""
    register(31, "ASML-style", doc(css, body))


def build_v32():
    """ExxonMobil-like: black/red industrial, split half-image half-text energy applications."""
    css = """
body{font-family:Arial,Helvetica,sans-serif;background:#fff;color:#111}
.nav{background:#000;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw;position:sticky;top:0;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700;text-transform:uppercase;font-size:.8rem;letter-spacing:.06em}
.nav img{height:26px}.nav a{color:#ccc;text-decoration:none;margin-left:1rem;font-size:.85rem}
.nav a.shop{background:#e31937;color:#fff;padding:.45rem 1rem;font-weight:700}
.hero{background:#000;color:#fff;padding:3rem 3vw;display:grid;grid-template-columns:1fr 1fr;gap:2rem;align-items:center}
.hero h1{font-size:clamp(1.8rem,4vw,2.8rem);text-transform:uppercase;margin:0 0 1rem}
.split{display:grid;grid-template-columns:1fr 1fr;min-height:50vh}
.split .img{min-height:280px;background-size:cover;background-position:center}
.split .txt{padding:2.5rem 3vw;display:flex;flex-direction:column;justify-content:center;background:#f5f5f5}
.split .txt h2{margin:0 0 .8rem;text-transform:uppercase;font-size:1.3rem;border-left:4px solid #e31937;padding-left:.7rem}
.split.flip .img{order:2}.split.flip .txt{order:1;background:#fff}
.section{padding:2.5rem 3vw}
table.specs th,table.specs td{border:1px solid #ddd;padding:.5rem;text-align:left;font-size:.88rem}
table.specs thead{background:#000;color:#fff}
pre.code{background:#000;color:#f88;padding:1rem;font-size:.82rem}
footer{background:#000;color:#ccc;padding:2rem 3vw}
footer a{color:#e31937;margin-right:1rem;font-weight:700}
@media(max-width:800px){.hero,.split{grid-template-columns:1fr}.split.flip .img,.split.flip .txt{order:0}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#apps">Applications</a><a href="#models">Specs</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <div>
    <h1>Power that moves machines</h1>
    <p>M17 Series all-in-one servomotors for industrial automation, robotics, and test systems. Motor, driver, controller, encoder — one package.</p>
    <p style="margin-top:1.2rem">
      <a href="{STORE}" style="display:inline-block;background:#e31937;color:#fff;padding:.6rem 1.2rem;text-decoration:none;font-weight:700">Shop Now</a>
      <a href="{DOCS}" style="margin-left:1rem;color:#e31937">Documentation</a>
    </p>
  </div>
  <img src="../transparent/one_motor_transparent_small.png" alt="M17">
</section>
<section class="split" id="apps">
  <div class="img" style="background-image:url('../robotics_small.jpg')"></div>
  <div class="txt">
    <h2>Robotics</h2>
    <p>Closed-loop axes with high-level trapezoid moves. Daisy-chain multiple joints on one RS-485 bus.</p>
  </div>
</section>
<section class="split flip">
  <div class="img" style="background-image:url('../automation_small.jpg')"></div>
  <div class="txt">
    <h2>Automation &amp; CNC</h2>
    <p>NEMA 17 mounting. 12–24 V. 32 kHz PID. Max 560 RPM. Protection built in.</p>
  </div>
</section>
<section class="split">
  <div class="img" style="background-image:url('../test_rack_small.jpg')"></div>
  <div class="txt">
    <h2>Test &amp; education</h2>
    <p>Open documentation and libraries for labs and classrooms. Three torque tiers for budget and load.</p>
  </div>
</section>
<section class="section" id="features">
  <h2 style="text-transform:uppercase;border-left:4px solid #e31937;padding-left:.7rem">Capabilities</h2>
  <ul>
    <li>All-in-one package · RS-485 multi-motor bus</li>
    <li>High-level commands — no STEP/DIR timing</li>
    <li>Pi, Arduino, ESP32, Mac, PC</li>
    <li>Open-source firmware and libraries</li>
  </ul>
</section>
<section class="section" id="models">
  <h2 style="text-transform:uppercase">Specifications</h2>
  <p>M17-60 · 0.65 N·m · 38 W · 470 g &nbsp;|&nbsp; M17-48 · 0.55 N·m · 32 W · 360 g &nbsp;|&nbsp; M17-40 · 0.42 N·m · 25 W · 285 g</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2 style="text-transform:uppercase">Start here</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<footer id="about">
  <p>Gearotons · Shenzhen 2022 · Canadian founder · Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(32, "ExxonMobil-style", doc(css, body))


def build_v33():
    """J&J-like: warm red care, storytelling bands alternating image/text."""
    css = """
body{font-family:Georgia,Times New Roman,serif;background:#fffaf7;color:#2b1a14}
.nav{position:sticky;top:0;z-index:20;background:#fff;border-bottom:1px solid #f0e0d8;display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw}
.nav img{height:32px}.nav a{font-family:Avenir,Helvetica,Arial,sans-serif;text-decoration:none;color:#2b1a14;margin-left:1rem;font-size:.88rem}
.nav a.shop{background:#d51900;color:#fff;padding:.5rem 1.1rem;border-radius:4px;font-weight:700}
.hero{text-align:center;padding:3.5rem 3vw 2rem;max-width:800px;margin:0 auto}
.hero h1{font-weight:400;font-size:clamp(1.9rem,4vw,2.8rem)}
.band{display:grid;grid-template-columns:1fr 1fr;align-items:center;min-height:420px}
.band .txt{padding:3rem 4vw}
.band h2{font-weight:400;font-size:1.6rem;margin:0 0 .8rem;color:#d51900}
.band p{font-family:Avenir,Helvetica,Arial,sans-serif;line-height:1.65;margin:0}
.band img{width:100%;height:100%;object-fit:cover;min-height:320px}
.band:nth-child(even) .txt{order:2}.band:nth-child(even) .media{order:1}
.section{padding:2.5rem 4vw;max-width:900px;margin:0 auto}
table.specs{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.88rem}
table.specs th,table.specs td{border-bottom:1px solid #f0e0d8;padding:.55rem;text-align:left}
pre.code{background:#2b1a14;color:#fde8e0;padding:1.1rem;font-size:.82rem;font-family:Menlo,monospace}
footer{background:#d51900;color:#fff;padding:2.5rem 3vw;text-align:center;font-family:Avenir,Helvetica,Arial,sans-serif}
footer a{color:#fff;margin:0 .7rem;font-weight:700}
@media(max-width:800px){.band{grid-template-columns:1fr}.band:nth-child(even) .txt,.band:nth-child(even) .media{order:0}}
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="#story">Story</a><a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="hero" id="top">
  <h1>Motion control made caring-simple</h1>
  <p style="font-family:Avenir,Helvetica,Arial,sans-serif;color:#5a4038">Affordable and Simple All-in-One Motion Control for people who build machines and teach the next builders.</p>
  <p style="margin-top:1.2rem">
    <a href="{STORE}" style="font-family:Avenir,Helvetica,Arial,sans-serif;display:inline-block;background:#d51900;color:#fff;padding:.6rem 1.3rem;text-decoration:none;font-weight:700;border-radius:4px">Shop Now</a>
    <a href="{DOCS}" style="font-family:Avenir,Helvetica,Arial,sans-serif;margin-left:1rem;color:#d51900">Documentation</a>
  </p>
</section>
<section id="story">
  <div class="band">
    <div class="txt">
      <h2>Everything in one body</h2>
      <p>Motor, driver, motion controller, and encoder live together. You spend less time on cabinets and more time on the machine.</p>
    </div>
    <div class="media"><img src="../one_motor_small.jpg" alt="Motor"></div>
  </div>
  <div class="band">
    <div class="txt">
      <h2>Share one connection</h2>
      <p>Daisy-chain any number of motors over RS-485. One host — Raspberry Pi, Arduino, ESP32, Mac, or PC.</p>
    </div>
    <div class="media"><img src="../adapter_and_wire_small.jpg" alt="Adapter"></div>
  </div>
  <div class="band">
    <div class="txt">
      <h2>Speak human commands</h2>
      <p>Enable MOSFETs. Trapezoid move. Closed-loop PID at 32 kHz — without STEP/DIR timing stress.</p>
    </div>
    <div class="media"><img src="../kit_with_three_motors_small.jpg" alt="Kit"></div>
  </div>
  <div class="band" id="apps">
    <div class="txt">
      <h2>Built for real work</h2>
      <p>Robotics, CNC, automated testing, scientific instruments, and education — with open documentation.</p>
    </div>
    <div class="media"><img src="../robotics_small.jpg" alt="Robotics"></div>
  </div>
</section>
<section class="section" id="models">
  <h2 style="font-weight:400;color:#d51900">Three models</h2>
  <p style="font-family:Avenir,Helvetica,Arial,sans-serif">M17-60 · 0.65 N·m · 38 W · 470 g &nbsp;·&nbsp; M17-48 · 0.55 N·m · 32 W · 360 g &nbsp;·&nbsp; M17-40 · 0.42 N·m · 25 W · 285 g</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2 style="font-weight:400;color:#d51900">First steps</h2>
  <p style="font-family:Avenir,Helvetica,Arial,sans-serif">pip3 install servomotor</p>
  {code_pre()}
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(33, "J&J-style", doc(css, body))


def build_v34():
    """Tencent-like: light dense Chinese-internet multi-panel app grid."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,"PingFang SC","Microsoft YaHei",sans-serif;background:#f5f6f8;color:#111}
.nav{background:#fff;position:sticky;top:0;z-index:20;display:flex;justify-content:space-between;align-items:center;padding:.6rem 3vw;box-shadow:0 1px 4px rgba(0,0,0,.06)}
.nav img{height:30px}.nav a{text-decoration:none;color:#111;margin-left:.8rem;font-size:.88rem}
.nav a.shop{background:#12b7f5;color:#fff;padding:.45rem 1rem;border-radius:4px;font-weight:700}
.hero{background:linear-gradient(180deg,#12b7f5 0%,#0e9fd9 100%);color:#fff;padding:2rem 3vw;display:grid;grid-template-columns:1.2fr .8fr;gap:1.5rem;align-items:center}
.hero h1{font-size:clamp(1.6rem,3.5vw,2.2rem);margin:0 0 .5rem}
.panels{display:grid;grid-template-columns:repeat(4,1fr);gap:.8rem;padding:1rem 3vw;margin-top:-1.5rem}
.panel{background:#fff;border-radius:10px;padding:1rem;box-shadow:0 2px 8px rgba(0,0,0,.06);min-height:120px}
.panel h3{margin:0 0 .4rem;font-size:.95rem;color:#12b7f5}
.panel p{margin:0;font-size:.85rem;opacity:.85}
.wide{grid-column:span 2}
.section{padding:1.5rem 3vw;background:#fff;margin:0 3vw 1rem;border-radius:10px}
table.specs{font-size:.85rem}table.specs th,table.specs td{border-bottom:1px solid #eee;padding:.5rem;text-align:left}
pre.code{background:#1a1a1a;color:#7fdbff;padding:1rem;border-radius:8px;font-size:.8rem}
.apps{display:grid;grid-template-columns:repeat(4,1fr);gap:.6rem}
.apps img{width:100%;aspect-ratio:1;object-fit:cover;border-radius:8px}
footer{padding:2rem 3vw;text-align:center;color:#666;font-size:.9rem}
footer a{color:#12b7f5;margin:0 .5rem;font-weight:600;text-decoration:none}
@media(max-width:900px){.hero,.panels,.apps{grid-template-columns:1fr 1fr}.wide{grid-column:span 2}}
@media(max-width:520px){.panels,.apps{grid-template-columns:1fr}.wide{grid-column:span 1}}
"""
    body = f"""
<header class="nav">
  <a href="#top"><img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons"></a>
  <div>
    <a href="#panels">Services</a><a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="hero" id="top">
  <div>
    <h1>M17 Series · all-in-one motion</h1>
    <p>Motor + driver + controller + encoder. RS-485 multi-motor. Open source tools.</p>
    <p style="margin-top:1rem">
      <a href="{STORE}" style="display:inline-block;background:#fff;color:#12b7f5;padding:.5rem 1.1rem;border-radius:4px;text-decoration:none;font-weight:700">Shop Now</a>
      <a href="{DOCS}" style="margin-left:.8rem;color:#fff">Docs</a>
    </p>
  </div>
  <img src="../transparent/M17_series_overview_transparent_small.png" alt="Series" style="max-height:180px;margin:0 auto">
</section>
<section class="panels" id="panels">
  <div class="panel"><h3>Integrate</h3><p>All-in-one package — no drive box per axis.</p></div>
  <div class="panel"><h3>Connect</h3><p>Daisy-chain any number on RS-485.</p></div>
  <div class="panel"><h3>Control</h3><p>32 kHz PID · trapezoid moves.</p></div>
  <div class="panel"><h3>Protect</h3><p>Current / voltage / temperature guards.</p></div>
  <div class="panel wide"><h3>Cross-platform hosts</h3><p>Raspberry Pi · Arduino · ESP32 · Mac · PC — one protocol.</p></div>
  <div class="panel wide"><h3>Open source</h3><p>Firmware, libraries, schematics on GitHub. Docs at gearotons.com.</p></div>
  <div class="panel"><h3>M17-60</h3><p>0.65 N·m · 38 W · 470 g</p></div>
  <div class="panel"><h3>M17-48</h3><p>0.55 N·m · 32 W · 360 g</p></div>
  <div class="panel"><h3>M17-40</h3><p>0.42 N·m · 25 W · 285 g</p></div>
  <div class="panel"><h3>Shared</h3><p>12–24 V · 560 RPM · NEMA 17</p></div>
</section>
<section class="section" id="models">
  <h2 style="margin-top:0;font-size:1.2rem">Specs</h2>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2 style="margin-top:0;font-size:1.2rem">Get started</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2 style="margin-top:0;font-size:1.2rem">Applications</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
    <img src="../kit_with_three_motors_small.jpg" alt="Kit">
  </div>
</section>
<footer id="about">
  <p>Gearotons · Shenzhen 2022 · Canadian founder · Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:.8rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(34, "Tencent-style", doc(css, body))


def build_v35():
    """Intel-like: blue layered architecture stack vertical diagram of integrated components."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#f4f7fb;color:#1d1e20}
.nav{background:#0068b5;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw;position:sticky;top:0;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700}
.nav img{height:26px}.nav a{color:#d0e8f8;text-decoration:none;margin-left:1rem;font-size:.88rem}
.nav a.shop{background:#00aeef;color:#fff;padding:.45rem 1rem;font-weight:700}
.hero{background:linear-gradient(180deg,#0068b5,#00aeef);color:#fff;padding:3rem 3vw;text-align:center}
.hero h1{font-size:clamp(1.8rem,4vw,2.6rem);margin:0 0 .6rem}
.stack-wrap{max-width:640px;margin:-2rem auto 2rem;padding:0 3vw;position:relative;z-index:2}
.layer{padding:1.2rem 1.5rem;margin-bottom:4px;color:#fff;text-align:center;font-weight:600;box-shadow:0 4px 12px rgba(0,0,0,.12)}
.layer small{display:block;font-weight:400;opacity:.9;font-size:.85rem;margin-top:.25rem}
.l1{background:#003c71}.l2{background:#0068b5}.l3{background:#00aeef;color:#003c71}.l4{background:#7fdbff;color:#003c71}.l5{background:#fff;color:#003c71;border:2px solid #0068b5}
.section{padding:2rem 3vw;max-width:1000px;margin:0 auto}
.two{display:grid;grid-template-columns:1fr 1fr;gap:2rem;align-items:center}
table.specs th,table.specs td{border-bottom:1px solid #d0dce8;padding:.55rem;text-align:left;font-size:.9rem}
pre.code{background:#003c71;color:#7fdbff;padding:1.1rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover}
footer{background:#003c71;color:#d0e8f8;padding:2.5rem 3vw;text-align:center}
footer a{color:#7fdbff;margin:0 .7rem;font-weight:700}
@media(max-width:800px){.two,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#arch">Architecture</a><a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <h1>M17 Series architecture</h1>
  <p>All-in-one motion compute at the axis — software-defined moves over RS-485.</p>
  <p style="margin-top:1rem">
    <a href="{STORE}" style="display:inline-block;background:#fff;color:#0068b5;padding:.55rem 1.2rem;text-decoration:none;font-weight:700">Shop Now</a>
    <a href="{DOCS}" style="margin-left:1rem;color:#fff">Documentation</a>
  </p>
</section>
<section class="stack-wrap" id="arch">
  <div class="layer l1">Application / host<small>Python · Arduino · ESP32 · Mac · PC</small></div>
  <div class="layer l2">RS-485 bus<small>Daisy-chain any number of motors</small></div>
  <div class="layer l3">Motion controller<small>High-level commands · trapezoid profiles</small></div>
  <div class="layer l4">Driver + PID @ 32 kHz<small>Closed-loop with onboard encoder</small></div>
  <div class="layer l5">Brushless motor · NEMA 17 package<small>12–24 V · max 560 RPM · integrated protection</small></div>
</section>
<section class="section" id="features">
  <div class="two">
    <img src="../one_motor_small.jpg" alt="Motor">
    <div>
      <h2>One package, full stack</h2>
      <p>What used to be four boxes is now one M17 unit. Same torque-to-weight as an equivalent stepper — more efficient in closed loop.</p>
    </div>
  </div>
</section>
<section class="section" id="models">
  <h2>Product SKUs</h2>
  <p>M17-60 · 0.65 N·m · 38 W · 470 g &nbsp;·&nbsp; M17-48 · 0.55 N·m · 32 W · 360 g &nbsp;·&nbsp; M17-40 · 0.42 N·m · 25 W · 285 g</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Developer path</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Solutions</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p>Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(35, "Intel-style", doc(css, body))


def build_v36():
    """Mastercard-like: black modern, large rounded cards, bold simple sections."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#f5f5f5;color:#141413}
.nav{background:#000;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.75rem 3vw;position:sticky;top:0;z-index:20;border-radius:0 0 16px 16px}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700}
.nav img{height:26px}.nav a{color:#ccc;text-decoration:none;margin-left:1rem;font-size:.9rem}
.nav a.shop{background:linear-gradient(90deg,#eb001b,#f79e1b);color:#fff;padding:.5rem 1.2rem;border-radius:999px;font-weight:700}
.hero{padding:3rem 3vw;text-align:center}
.hero h1{font-size:clamp(2rem,5vw,3.2rem);margin:0 0 .8rem;letter-spacing:-.02em}
.cards{display:grid;grid-template-columns:repeat(2,1fr);gap:1.2rem;padding:0 3vw 2rem;max-width:1000px;margin:0 auto}
.card{background:#fff;border-radius:24px;padding:2rem;box-shadow:0 8px 30px rgba(0,0,0,.06)}
.card.dark{background:#141413;color:#fff}
.card h2{margin:0 0 .6rem;font-size:1.4rem}
.card img{border-radius:16px;margin-top:1rem;width:100%;max-height:220px;object-fit:cover}
.section{padding:1rem 3vw 2rem;max-width:1000px;margin:0 auto}
.section .card{margin-bottom:1.2rem}
table.specs th,table.specs td{border-bottom:1px solid #e5e5e5;padding:.55rem;text-align:left;font-size:.9rem}
pre.code{background:#141413;color:#f7c948;padding:1.2rem;border-radius:16px;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{border-radius:16px;width:100%;aspect-ratio:1;object-fit:cover}
footer{background:#000;color:#fff;padding:2.5rem 3vw;text-align:center;border-radius:24px 24px 0 0;margin-top:1rem}
footer a{color:#f79e1b;margin:0 .7rem;font-weight:700}
@media(max-width:800px){.cards,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <div>
    <a href="#cards">Explore</a><a href="#models">Models</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </div>
</header>
<section class="hero" id="top">
  <h1>Simple. Powerful. Connected.</h1>
  <p style="max-width:32rem;margin:0 auto;opacity:.8">M17 Series servomotors — all-in-one motion control with open-source tools.</p>
  <p style="margin-top:1.2rem">
    <a href="{STORE}" style="display:inline-block;background:#141413;color:#fff;padding:.65rem 1.4rem;border-radius:999px;text-decoration:none;font-weight:700">Shop Now</a>
    <a href="{DOCS}" style="margin-left:.8rem;font-weight:600">Documentation</a>
  </p>
</section>
<section class="cards" id="cards">
  <article class="card dark" id="features">
    <h2>All-in-one</h2>
    <p>Motor, driver, controller, encoder — one package. Daisy-chain over RS-485.</p>
    <img src="../transparent/one_motor_transparent_small.png" alt="Motor" style="object-fit:contain;background:transparent;max-height:180px">
  </article>
  <article class="card">
    <h2>High-level moves</h2>
    <p>Trapezoid profiles and enable commands. No STEP/DIR timing. 32 kHz PID.</p>
    <img src="../adapter_and_wire_small.jpg" alt="Adapter">
  </article>
  <article class="card">
    <h2>Three models</h2>
    <p>0.65 · 0.55 · 0.42 N·m. Same protocol. NEMA 17. 12–24 V. 560 RPM max.</p>
    <img src="../M17_series_overview.jpg" alt="Series">
  </article>
  <article class="card dark">
    <h2>Open source</h2>
    <p>Firmware, libraries, schematics. Gearotons — Shenzhen 2022, Canadian founder.</p>
    <div style="display:flex;gap:.8rem;margin-top:1rem">
      <img src="../Open_Source_Initiative.svg.png" alt="OSI" style="height:40px;width:auto;object-fit:contain;max-height:40px">
      <img src="../Open-source-hardware-logo.svg.png" alt="OSHW" style="height:40px;width:auto;object-fit:contain;max-height:40px">
    </div>
  </article>
</section>
<section class="section" id="models">
  <div class="card">
    <h2>Specs</h2>
    {specs_table()}
  </div>
  <div class="card" id="start">
    <h2>Get started</h2>
    <p>pip3 install servomotor</p>
    {code_pre()}
  </div>
  <div class="card" id="apps">
    <h2>Applications</h2>
    <div class="apps" style="margin-top:1rem">
      <img src="../robotics_small.jpg" alt="Robotics">
      <img src="../automation_small.jpg" alt="Automation">
      <img src="../test_rack_small.jpg" alt="Test">
    </div>
  </div>
</section>
<footer id="about">
  <p>Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(36, "Mastercard-style", doc(css, body))


def build_v37():
    """AbbVie-like: navy + orange, clinical-style data table emphasis."""
    css = """
body{font-family:Georgia,Times New Roman,serif;background:#f7f5f2;color:#1b2a4a}
.nav{background:#1b2a4a;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw;position:sticky;top:0;z-index:20;font-family:Helvetica,Arial,sans-serif}
.nav img{height:28px}.nav a{color:#c5d0e0;text-decoration:none;margin-left:1rem;font-size:.85rem}
.nav a.shop{background:#e87722;color:#fff;padding:.45rem 1rem;font-weight:700}
.hero{padding:3rem 3vw;max-width:900px;margin:0 auto}
.hero h1{font-weight:400;font-size:clamp(1.8rem,3.5vw,2.5rem)}
.data{background:#fff;margin:0 3vw 2rem;padding:2rem;border-top:4px solid #e87722;box-shadow:0 4px 16px rgba(27,42,74,.06)}
.data h2{font-weight:400;margin-top:0;font-family:Helvetica,Arial,sans-serif;font-size:1.1rem;text-transform:uppercase;letter-spacing:.06em;color:#e87722}
table.specs{font-family:Helvetica,Arial,sans-serif;font-size:.88rem}
table.specs th,table.specs td{border:1px solid #d5dce8;padding:.6rem;text-align:left}
table.specs thead{background:#1b2a4a;color:#fff}
table.specs tbody tr:nth-child(even){background:#f0f3f8}
.section{padding:1.5rem 3vw;max-width:900px;margin:0 auto}
.feats{display:grid;grid-template-columns:1fr 1fr;gap:1rem;font-family:Helvetica,Arial,sans-serif}
.feats article{background:#fff;padding:1rem;border-left:3px solid #e87722}
pre.code{background:#1b2a4a;color:#f5d0b0;padding:1.1rem;font-size:.82rem;font-family:Menlo,monospace}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:4/3;object-fit:cover}
footer{background:#1b2a4a;color:#c5d0e0;padding:2rem 3vw;font-family:Helvetica,Arial,sans-serif}
footer a{color:#e87722;margin-right:1rem;font-weight:700}
@media(max-width:800px){.feats,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a href="#top" style="display:flex;gap:.5rem;align-items:center;text-decoration:none;color:#fff;font-weight:700">
    <img src="../Gearotons_Logo.png" alt="">Gearotons
  </a>
  <nav>
    <a href="#data">Data</a><a href="#start">Methods</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <p style="font-family:Helvetica,Arial,sans-serif;color:#e87722;font-weight:700;font-size:.8rem;letter-spacing:.1em;text-transform:uppercase">Product data sheet narrative</p>
  <h1>M17 Series — measured motion performance</h1>
  <p style="font-family:Helvetica,Arial,sans-serif;line-height:1.65">All-in-one servomotors integrating motor, driver, motion controller, and encoder. Designed for transparent specifications and open documentation.</p>
  <p style="font-family:Helvetica,Arial,sans-serif;margin-top:1rem">
    <a href="{STORE}" style="display:inline-block;background:#e87722;color:#fff;padding:.55rem 1.2rem;text-decoration:none;font-weight:700">Shop Now</a>
    <a href="{DOCS}" style="margin-left:1rem;color:#e87722">Documentation</a>
  </p>
</section>
<section class="data" id="data">
  <h2>Primary endpoints — model comparison</h2>
  <p style="font-family:Helvetica,Arial,sans-serif;font-size:.95rem">Footprint 42.2 × 42.2 mm for all models. Operating voltage 12–24 V. Maximum speed 560 RPM. Maximum current 1.1 A.</p>
  {specs_table()}
</section>
<section class="section" id="features">
  <h2 style="font-weight:400">Supporting observations</h2>
  <div class="feats">
    {"".join(f"<article><strong>{esc(t)}</strong><p style='margin:.3rem 0 0;font-size:.9rem'>{esc(d)}</p></article>" for t,d in FEATURES[:6])}
  </div>
</section>
<section class="section" id="start">
  <h2 style="font-weight:400">Methods — first motion</h2>
  <p style="font-family:Helvetica,Arial,sans-serif">pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2 style="font-weight:400">Use settings</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p>Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(37, "AbbVie-style", doc(css, body))


def build_v38():
    """Applied Materials-like: dark teal industrial equipment, process steps."""
    css = """
body{font-family:Arial,Helvetica,sans-serif;background:#0a1f24;color:#d8e8ec}
.nav{display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw;border-bottom:1px solid #1a3a42;position:sticky;top:0;background:#0a1f24;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700;letter-spacing:.04em}
.nav img{height:26px}.nav a{color:#8ab;text-decoration:none;margin-left:1rem;font-size:.85rem}
.nav a.shop{background:#00a3a1;color:#fff;padding:.45rem 1rem;font-weight:700}
.hero{padding:3rem 3vw;display:grid;grid-template-columns:1fr 1fr;gap:2rem;align-items:center}
.hero h1{font-size:clamp(1.7rem,3.5vw,2.5rem);margin:0 0 .8rem}
.process{padding:2rem 3vw}
.process h2{color:#00a3a1;text-transform:uppercase;font-size:.9rem;letter-spacing:.1em}
.steps{display:grid;grid-template-columns:repeat(5,1fr);gap:1px;background:#1a3a42;margin-top:1rem}
.steps div{background:#0f2a30;padding:1.2rem 1rem}
.steps .n{color:#00a3a1;font-weight:700;font-size:.75rem;letter-spacing:.08em}
.steps h3{margin:.4rem 0;font-size:1rem}
.steps p{margin:0;font-size:.85rem;opacity:.85}
.section{padding:2rem 3vw}
table.specs{font-size:.85rem}table.specs th,table.specs td{border:1px solid #1a3a42;padding:.5rem;text-align:left}
table.specs th{background:#0f2a30;color:#00a3a1}
pre.code{background:#061416;border:1px solid #1a3a42;color:#7fefdf;padding:1rem;font-size:.8rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover;border:1px solid #1a3a42}
footer{padding:2rem 3vw;border-top:1px solid #1a3a42}
footer a{color:#00a3a1;margin-right:1rem;font-weight:700}
@media(max-width:900px){.hero,.steps,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">GEAROTONS</a>
  <nav>
    <a href="#process">Process</a><a href="#models">Tools</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <div>
    <h1>Make precision motion possible</h1>
    <p>M17 Series equipment-class all-in-one servomotors. Integrate the drive stack into the motor. Network axes over RS-485.</p>
    <p style="margin-top:1rem">
      <a href="{STORE}" style="display:inline-block;background:#00a3a1;color:#fff;padding:.55rem 1.2rem;text-decoration:none;font-weight:700">Shop Now</a>
      <a href="{DOCS}" style="margin-left:.8rem;color:#00a3a1">Documentation</a>
    </p>
  </div>
  <img src="../transparent/kit_with_three_motors_transparent_small.png" alt="Kit">
</section>
<section class="process" id="process">
  <h2>Process flow</h2>
  <div class="steps">
    <div><div class="n">STEP 01</div><h3>Select</h3><p>Pick M17-60 / 48 / 40 by torque.</p></div>
    <div><div class="n">STEP 02</div><h3>Mount</h3><p>NEMA 17 · 42.2 × 42.2 mm footprint.</p></div>
    <div><div class="n">STEP 03</div><h3>Power</h3><p>12–24 V with onboard protection.</p></div>
    <div><div class="n">STEP 04</div><h3>Network</h3><p>RS-485 daisy-chain to host.</p></div>
    <div><div class="n">STEP 05</div><h3>Command</h3><p>Python/Arduino trapezoid moves.</p></div>
  </div>
</section>
<section class="section" id="features">
  <h2 style="color:#00a3a1;text-transform:uppercase;font-size:.9rem;letter-spacing:.1em">Equipment features</h2>
  <ul>
    <li>Motor + driver + controller + encoder integrated</li>
    <li>32 kHz closed-loop PID · max 560 RPM</li>
    <li>Pi, Arduino, ESP32, Mac, PC</li>
    <li>Open-source firmware and libraries</li>
  </ul>
</section>
<section class="section" id="models">
  <h2 style="color:#00a3a1;text-transform:uppercase;font-size:.9rem;letter-spacing:.1em">Tool specifications</h2>
  <p>M17-60 · 0.65 N·m · 38 W · 470 g &nbsp;|&nbsp; M17-48 · 0.55 N·m · 32 W · 360 g &nbsp;|&nbsp; M17-40 · 0.42 N·m · 25 W · 285 g</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2 style="color:#00a3a1;text-transform:uppercase;font-size:.9rem;letter-spacing:.1em">Recipe — first move</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2 style="color:#00a3a1;text-transform:uppercase;font-size:.9rem;letter-spacing:.1em">Fab applications</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Open source · Gearotons · Shenzhen 2022 · Canadian founder</p>
  <p><a href="{STORE}">Shop Now</a><a href="{DOCS}">Docs</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(38, "Applied Materials-style", doc(css, body))


def build_v39():
    """Cisco-like: network node diagram layout (motors as nodes on bus), blue."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#f2f5f8;color:#1b1c1d;font-weight:300}
.nav{background:#1b1c1d;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.65rem 3vw;position:sticky;top:0;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:400;letter-spacing:.04em}
.nav img{height:26px}.nav a{color:#b0c4d0;text-decoration:none;margin-left:1rem;font-size:.85rem;font-weight:300}
.nav a.shop{background:#049fd9;color:#fff;padding:.45rem 1rem;font-weight:400}
.hero{padding:3rem 3vw;text-align:center}
.hero h1{font-weight:200;font-size:clamp(1.8rem,4vw,2.8rem)}
.diagram{max-width:900px;margin:0 auto 2rem;padding:2rem;background:#fff;border:1px solid #d0dce6}
.bus{height:4px;background:#049fd9;margin:2rem 0;position:relative}
.nodes{display:flex;justify-content:space-between;flex-wrap:wrap;gap:1rem}
.node{text-align:center;flex:1;min-width:100px}
.node .box{border:2px solid #049fd9;padding:.8rem .5rem;background:#e8f6fc;font-weight:400;font-size:.85rem}
.node.host .box{background:#049fd9;color:#fff;border-color:#049fd9}
.node .stem{width:2px;height:24px;background:#049fd9;margin:0 auto}
.section{padding:2rem 3vw;max-width:1000px;margin:0 auto}
.section h2{font-weight:300;color:#049fd9}
.feats{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.feats article{background:#fff;border-top:3px solid #049fd9;padding:1.2rem}
table.specs th,table.specs td{border-bottom:1px solid #d0dce6;padding:.55rem;text-align:left;font-size:.9rem;font-weight:300}
pre.code{background:#1b1c1d;color:#7ec8e8;padding:1.1rem;font-size:.82rem;font-weight:400}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover}
footer{background:#1b1c1d;color:#b0c4d0;padding:2rem 3vw;font-weight:300}
footer a{color:#049fd9;margin-right:1rem}
@media(max-width:800px){.feats,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#network">Network</a><a href="#models">Products</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <h1>Build your motion network</h1>
  <p style="max-width:36rem;margin:0 auto;opacity:.85">M17 Series nodes on an RS-485 fabric — each node is a full servomotor stack.</p>
  <p style="margin-top:1.2rem">
    <a href="{STORE}" style="display:inline-block;background:#049fd9;color:#fff;padding:.55rem 1.2rem;text-decoration:none">Shop Now</a>
    <a href="{DOCS}" style="margin-left:1rem;color:#049fd9">Documentation</a>
  </p>
</section>
<section class="diagram" id="network">
  <h2 style="text-align:center;font-weight:300;margin-top:0;color:#049fd9">RS-485 multi-drop topology</h2>
  <div class="nodes">
    <div class="node host"><div class="box">Host<br>controller</div><div class="stem"></div></div>
    <div class="node"><div class="box">M17-60<br>node</div><div class="stem"></div></div>
    <div class="node"><div class="box">M17-48<br>node</div><div class="stem"></div></div>
    <div class="node"><div class="box">M17-40<br>node</div><div class="stem"></div></div>
    <div class="node"><div class="box">…n</div><div class="stem"></div></div>
  </div>
  <div class="bus"></div>
  <p style="text-align:center;font-size:.9rem;opacity:.8;margin:0">Single connection point · daisy-chain any number of motors · high-level commands on the wire</p>
</section>
<section class="section" id="features">
  <h2>Platform capabilities</h2>
  <div class="feats">
    <article><h3 style="margin-top:0;font-weight:400">Integrated node</h3><p>Motor + driver + controller + encoder.</p></article>
    <article><h3 style="margin-top:0;font-weight:400">Secure enough for labs</h3><p>Over-current, voltage, temperature protection.</p></article>
    <article><h3 style="margin-top:0;font-weight:400">Open endpoints</h3><p>Pi, Arduino, ESP32, Mac, PC. Open source stack.</p></article>
  </div>
</section>
<section class="section" id="models">
  <h2>Product catalog data</h2>
  <p>0.65 / 0.55 / 0.42 N·m · 12–24 V · 560 RPM · 32 kHz PID · NEMA 17</p>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Configuration sample</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Deployments</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Gearotons · Shenzhen 2022 · Canadian founder · Affordable and Simple All-in-One Motion Control</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(39, "Cisco-style", doc(css, body))


def build_v40():
    """Bank of America-like: blue/red, three product tier columns banking-style comparison."""
    css = """
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:#f5f5f5;color:#012169}
.nav{background:#012169;color:#fff;display:flex;justify-content:space-between;align-items:center;padding:.7rem 3vw;position:sticky;top:0;z-index:20}
.nav .b{display:flex;gap:.5rem;align-items:center;text-decoration:none;font-weight:700}
.nav img{height:28px}.nav a{color:#c5d0e8;text-decoration:none;margin-left:1rem;font-size:.88rem}
.nav a.shop{background:#e31837;color:#fff;padding:.5rem 1.1rem;font-weight:700}
.hero{background:#fff;padding:2.5rem 3vw;text-align:center;border-bottom:4px solid #e31837}
.hero h1{font-size:clamp(1.7rem,3.5vw,2.4rem);margin:0 0 .6rem}
.tiers{display:grid;grid-template-columns:repeat(3,1fr);gap:1.2rem;padding:2rem 3vw;max-width:1100px;margin:0 auto}
.tier{background:#fff;border:1px solid #d0d5e0;display:flex;flex-direction:column}
.tier header{background:#012169;color:#fff;padding:1.2rem;text-align:center}
.tier header.mid{background:#e31837}
.tier .body{padding:1.5rem;flex:1}
.tier .body .amt{font-size:2rem;font-weight:700;color:#012169;margin:.5rem 0}
.tier ul{padding-left:1.1rem;font-size:.9rem;line-height:1.7}
.tier footer{padding:1rem 1.5rem 1.5rem}
.tier footer a{display:block;text-align:center;background:#012169;color:#fff;padding:.65rem;text-decoration:none;font-weight:700}
.tier footer a.red{background:#e31837}
.section{padding:1.5rem 3vw;max-width:1000px;margin:0 auto}
.section h2{border-bottom:2px solid #012169;padding-bottom:.4rem}
table.specs th,table.specs td{border:1px solid #d0d5e0;padding:.5rem;text-align:left;font-size:.88rem;background:#fff}
table.specs thead{background:#012169;color:#fff}
pre.code{background:#012169;color:#d0e0ff;padding:1.1rem;font-size:.82rem}
.apps{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem}
.apps img{width:100%;aspect-ratio:16/10;object-fit:cover}
footer{background:#012169;color:#c5d0e8;padding:2.5rem 3vw;text-align:center}
footer a{color:#fff;margin:0 .7rem;font-weight:700}
@media(max-width:800px){.tiers,.apps{grid-template-columns:1fr}}
"""
    body = f"""
<header class="nav">
  <a class="b" href="#top"><img src="../Gearotons_Logo.png" alt="">Gearotons</a>
  <nav>
    <a href="#tiers">Compare</a><a href="#models">Details</a>
    <a class="shop" href="{STORE}">Shop Now</a>
  </nav>
</header>
<section class="hero" id="top">
  <h1>Choose the M17 tier that fits your build</h1>
  <p>Affordable and Simple All-in-One Motion Control — three clear product tiers, one open protocol.</p>
</section>
<section class="tiers" id="tiers">
  <article class="tier">
    <header><h2 style="margin:0;font-size:1.2rem">M17-40</h2><p style="margin:.3rem 0 0;opacity:.85;font-size:.85rem">Compact</p></header>
    <div class="body">
      <div class="amt">0.42 N·m</div>
      <ul>
        <li>25 W rated power</li>
        <li>285 g weight</li>
        <li>41.6 mm height</li>
        <li>12–24 V · 560 RPM</li>
        <li>Full all-in-one stack</li>
      </ul>
    </div>
    <footer><a href="{STORE}">Shop Now</a></footer>
  </article>
  <article class="tier">
    <header class="mid"><h2 style="margin:0;font-size:1.2rem">M17-48</h2><p style="margin:.3rem 0 0;opacity:.9;font-size:.85rem">Most balanced</p></header>
    <div class="body">
      <div class="amt">0.55 N·m</div>
      <ul>
        <li>32 W rated power</li>
        <li>360 g weight</li>
        <li>48.6 mm height</li>
        <li>12–24 V · 560 RPM</li>
        <li>Full all-in-one stack</li>
      </ul>
    </div>
    <footer><a class="red" href="{STORE}">Shop Now</a></footer>
  </article>
  <article class="tier">
    <header><h2 style="margin:0;font-size:1.2rem">M17-60</h2><p style="margin:.3rem 0 0;opacity:.85;font-size:.85rem">Highest torque</p></header>
    <div class="body">
      <div class="amt">0.65 N·m</div>
      <ul>
        <li>38 W rated power</li>
        <li>470 g weight</li>
        <li>59.8 mm height</li>
        <li>12–24 V · 560 RPM</li>
        <li>Full all-in-one stack</li>
      </ul>
    </div>
    <footer><a href="{STORE}">Shop Now</a></footer>
  </article>
</section>
<section class="section" id="features">
  <h2>Included with every tier</h2>
  <ul>
    <li>Motor + driver + motion controller + encoder</li>
    <li>RS-485 daisy-chain · high-level trapezoid moves</li>
    <li>32 kHz PID · NEMA 17 · footprint 42.2 × 42.2 mm</li>
    <li>Works with Pi, Arduino, ESP32, Mac, PC</li>
    <li>Open-source firmware and libraries</li>
  </ul>
</section>
<section class="section" id="models">
  <h2>Full disclosure table</h2>
  {specs_table()}
</section>
<section class="section" id="start">
  <h2>Open an account with motion — Python</h2>
  <p>pip3 install servomotor</p>
  {code_pre()}
</section>
<section class="section" id="apps">
  <h2>Where builders put them to work</h2>
  <div class="apps">
    <img src="../robotics_small.jpg" alt="Robotics">
    <img src="../automation_small.jpg" alt="Automation">
    <img src="../test_rack_small.jpg" alt="Test">
  </div>
</section>
<footer id="about">
  <p>Gearotons · Shenzhen 2022 · Canadian founder · Open source hardware &amp; software</p>
  <p style="margin-top:1rem"><a href="{STORE}">Shop Now</a><a href="{DOCS}">Documentation</a><a href="{GH}">GitHub</a></p>
</footer>"""
    register(40, "Bank of America-style", doc(css, body))


# =============================================================================
# Viewer + research + main
# =============================================================================

VIEWER_LABELS = {
    1: "Precision Lab",
    2: "Dark Command",
    3: "Editorial Serif",
    4: "Soft Consumer",
    5: "Blueprint Grid",
    6: "Bold Poster",
    7: "Swiss Grid",
    8: "Warm Workshop",
    9: "Neon Arcade",
    10: "Luxury Minimal",
    11: "NVIDIA-style",
    12: "Apple-style",
    13: "Google-style",
    14: "Microsoft-style",
    15: "Amazon-style",
    16: "TSMC-style",
    17: "Broadcom-style",
    18: "SpaceX-style",
    19: "Aramco-style",
    20: "Meta-style",
    21: "Tesla-style",
    22: "Samsung-style",
    23: "Berkshire-style",
    24: "Eli Lilly-style",
    25: "Micron-style",
    26: "Walmart-style",
    27: "JPMorgan-style",
    28: "SK Hynix-style",
    29: "AMD-style",
    30: "Visa-style",
    31: "ASML-style",
    32: "ExxonMobil-style",
    33: "J&J-style",
    34: "Tencent-style",
    35: "Intel-style",
    36: "Mastercard-style",
    37: "AbbVie-style",
    38: "Applied Materials-style",
    39: "Cisco-style",
    40: "Bank of America-style",
}


def write_viewer():
    tabs = "".join(
        f'<button type="button" class="tab" data-src="v{n}.html" data-n="{n}">'
        f'<span class="num">v{n}</span><span class="lab">{esc(VIEWER_LABELS[n])}</span></button>'
        for n in range(1, 41)
    )
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Design Variations B — M17 (pass 2 structural)</title>
<style>
*{{box-sizing:border-box}}
body{{margin:0;font-family:system-ui,Segoe UI,Helvetica,Arial,sans-serif;background:#111;color:#eee;height:100vh;display:flex;flex-direction:column}}
header{{display:flex;justify-content:space-between;align-items:center;padding:.6rem 1rem;background:#1a1a1a;border-bottom:1px solid #333;flex-wrap:wrap;gap:.5rem}}
header h1{{margin:0;font-size:1rem;font-weight:600}}
header .meta{{font-size:.75rem;opacity:.7}}
.controls button{{background:#333;color:#fff;border:1px solid #555;padding:.4rem .8rem;cursor:pointer;margin-left:.3rem;border-radius:4px}}
.controls button:hover{{background:#444}}
.layout{{display:grid;grid-template-columns:220px 1fr;flex:1;min-height:0}}
.tabs{{overflow-y:auto;border-right:1px solid #333;background:#161616;padding:.4rem}}
.tab{{display:flex;flex-direction:column;align-items:flex-start;width:100%;text-align:left;background:transparent;border:0;border-left:3px solid transparent;color:#bbb;padding:.45rem .6rem;cursor:pointer;margin-bottom:1px}}
.tab:hover{{background:#222;color:#fff}}
.tab.active{{background:#2a2a2a;border-left-color:#7AB648;color:#fff}}
.tab .num{{font-size:.7rem;opacity:.6;font-weight:700}}
.tab .lab{{font-size:.82rem}}
.frame-wrap{{min-height:0}}
iframe{{width:100%;height:100%;border:0;background:#fff}}
@media(max-width:700px){{.layout{{grid-template-columns:1fr}}.tabs{{display:flex;overflow-x:auto;overflow-y:hidden;border-right:0;border-bottom:1px solid #333}}.tab{{min-width:100px}}}}
</style>
</head>
<body>
<header>
  <div>
    <h1>M17 Design Variations B — pass 2</h1>
    <div class="meta">40 structurally distinct pages · Round 1 original · Round 2 company pastiches</div>
  </div>
  <div class="controls">
    <button type="button" id="prev">← Prev</button>
    <button type="button" id="next">Next →</button>
    <button type="button" id="open">Open tab</button>
  </div>
</header>
<div class="layout">
  <nav class="tabs" id="tabs">{tabs}</nav>
  <div class="frame-wrap"><iframe id="frame" title="Variant preview" src="v1.html"></iframe></div>
</div>
<script>
(function(){{
  const tabs=[...document.querySelectorAll('.tab')];
  const frame=document.getElementById('frame');
  let idx=0;
  function show(i){{
    idx=(i+tabs.length)%tabs.length;
    tabs.forEach((t,n)=>t.classList.toggle('active',n===idx));
    frame.src=tabs[idx].dataset.src;
    tabs[idx].scrollIntoView({{block:'nearest',inline:'nearest'}});
  }}
  tabs.forEach((t,i)=>t.addEventListener('click',()=>show(i)));
  document.getElementById('prev').onclick=()=>show(idx-1);
  document.getElementById('next').onclick=()=>show(idx+1);
  document.getElementById('open').onclick=()=>window.open(tabs[idx].dataset.src,'_blank');
  document.addEventListener('keydown',e=>{{
    if(e.key==='ArrowLeft')show(idx-1);
    if(e.key==='ArrowRight')show(idx+1);
  }});
  show(0);
}})();
</script>
</body>
</html>
"""
    (OUT / "index.html").write_text(html, encoding="utf-8")
    print("wrote index.html")


def write_design_notes():
    notes = """# Design language notes — pass 2 (structural rebuild)

**Pass 2 goal:** Each of v1–v40 has a **genuinely different page architecture**
(DOM structure / layout system), not merely different colors/fonts/buttons on a
shared skeleton. Pass 1 was rejected for visual-only variation.

Research method: market-cap ranking from CompaniesMarketCap (2026-07-17);
design notes from established public brand systems and marketing-site layout
patterns. No company trademarks, logos, product names, or copy — design
language only. All content is Gearotons M17.

## Pass 2 structural approach

| # | Label | Architecture (what differs) |
|---|-------|------------------------------|
| v1 | Precision Lab | Asymmetric 50/50 hero; sticky side rail + 2-col feature grid |
| v2 | Dark Command | Full-viewport dark stage; **horizontal scroll** capability strip |
| v3 | Editorial Serif | Magazine masthead; multi-column newspaper text; full-bleed photo break; pull quote |
| v4 | Soft Consumer | **Bento/masonry** CSS grid homepage of mixed-span cards |
| v5 | Blueprint Grid | Blueprint grid bg; two-pane drawing + numbered callouts; dimension sheets |
| v6 | Bold Poster | Huge stacked display type; thick black borders; 3-col model band |
| v7 | Swiss Grid | 12-col modular grid; numbered modules; red accent; Helvetica |
| v8 | Warm Workshop | Irregular **polaroid photo wall** + kraft paper notes |
| v9 | Neon Arcade | **Split dual panels**; game-UI stat bars; neon HUD |
| v10 | Luxury Minimal | Fixed tiny nav; **sequential single-column** scroll chapters; huge whitespace |
| v11 | NVIDIA-style | Stacked full-width story heroes; **horizontal card rails** |
| v12 | Apple-style | Centered full-bleed product units stacked; promo **2-up** grid; pill links |
| v13 | Google-style | Material multi-card auto-fill grid; soft elevation cards |
| v14 | Microsoft-style | **Mega product nav strip**; large hero; secondary **2×2 promo** cards |
| v15 | Amazon-style | Dark nav + fake search; **left gallery + right buy-box** PDP; thumb JS |
| v16 | TSMC-style | Institutional hero; **4 technology pillars** row; corporate table |
| v17 | Broadcom-style | Dark enterprise; **model tabs with JS**; dense datasheet blocks |
| v18 | SpaceX-style | **Sequential full-viewport** black sections; outlined CTAs; mission stats |
| v19 | Aramco-style | **Left sticky side navigation** + report-style single main column |
| v20 | Meta-style | Soft cards; **connection network diagram** (host + motor nodes) |
| v21 | Tesla-style | **100vh cinematic hero**; dual order CTAs; sticky model selector bar; filmstrip |
| v22 | Samsung-style | OLED product stage; **vertical numbered feature list** (big numerals) |
| v23 | Berkshire-style | Times longform **annual letter**; plain HTML; single narrow column |
| v24 | Eli Lilly-style | Medical calm; **horizontal 4-step pathway** process strip |
| v25 | Micron-style | Dark tech; **top metrics ribbon**; dense datasheet |
| v26 | Walmart-style | Retail blue/yellow; **category aisle grid** of applications |
| v27 | JPMorgan-style | Navy; **KPI strip**; multi-column report + sticky side cards |
| v28 | SK Hynix-style | **3 family comparison cards** side-by-side with featured middle |
| v29 | AMD-style | Aggressive stack; **big torque number product cards** stacked |
| v30 | Visa-style | Deep blue/gold; **horizontal 4-step how-it-works** overlapping hero |
| v31 | ASML-style | Near-black; light weights; deep **3-col system architecture** + 2-pane data |
| v32 | ExxonMobil-style | Industrial; **alternating half-image / half-text** application splits |
| v33 | J&J-style | Warm red; **storytelling bands** alternating image/text |
| v34 | Tencent-style | Dense **multi-panel app grid** over colorful hero |
| v35 | Intel-style | Blue gradient; **vertical layered architecture stack** diagram |
| v36 | Mastercard-style | Black modern; **large rounded 2×2 cards** + stacked content cards |
| v37 | AbbVie-style | Navy/orange; **clinical data table emphasis** as primary section |
| v38 | Applied Materials-style | Dark teal; **5-step process flow** industrial equipment |
| v39 | Cisco-style | **Network node diagram** (motors as bus nodes) as hero structure |
| v40 | Bank of America-style | Blue/red; **three product tier columns** banking comparison |

## Per-company notes (round 2)

### v11 NVIDIA
Black canvas, neon lime (#76b900), stacked story heroes, horizontal product rails.

### v12 Apple
SF-like system stack, huge centered product units, pill blue links (#0071e3), promo 2-up.

### v13 Alphabet / Google
Material white, colorful dots, soft elevation cards, friendly weight 400 headlines.

### v14 Microsoft
Mega nav strip, Fluent blue (#0067b8), left-accent promo cards 2×2.

### v15 Amazon
Navy nav (#131921), orange CTA, left thumbs + main image, right buy-box PDP density.

### v16 TSMC
Corporate navy/red, multi-pillar technology sections, institutional tables.

### v17 Broadcom
Black enterprise header, crimson accent, **tabbed product UI** (JS), dense datasheet.

### v18 SpaceX
Full-viewport black sections, huge uppercase, outlined white buttons, mission brief.

### v19 Saudi Aramco
Deep blue + cyan, **left sticky side nav**, report-style main column.

### v20 Meta
Meta blue (#0668E1), 24px rounded cards, connection network diagram.

### v21 Tesla
100vh cinematic hero, dual order-style CTAs, horizontal model selector, feature filmstrip.

### v22 Samsung
OLED black, Samsung blue, vertical feature list with large numbers.

### v23 Berkshire Hathaway
Times serif annual-letter longform, plain tables, minimal chrome.

### v24 Eli Lilly
Medical red, serif, horizontal clinical pathway steps.

### v25 Micron
Dark blue tech, cyan metrics ribbon, dense datasheet.

### v26 Walmart
Blue/yellow retail, aisle category grid, value-forward hero.

### v27 JPMorgan Chase
Navy finance, KPI strip (real specs only), multi-column report.

### v28 SK Hynix
Black/red, three family comparison cards side by side.

### v29 AMD
Black/red aggressive, big torque numbers on stack cards.

### v30 Visa
Deep blue + gold, 4-step how-it-works process.

### v31 ASML
Near-black precision, light font weights, deep technical sections.

### v32 ExxonMobil
Black/red industrial, split half-image half-text bands.

### v33 Johnson & Johnson
Warm red care, alternating image/text storytelling bands.

### v34 Tencent
Light dense multi-panel product grid, cyan accent.

### v35 Intel
Blue layered vertical architecture stack of integrated components.

### v36 Mastercard
Black modern, large rounded cards, bold simple sections.

### v37 AbbVie
Navy + orange, clinical-style data table as hero content.

### v38 Applied Materials
Dark teal industrial, 5-step process flow.

### v39 Cisco
Network node diagram (motors as nodes on bus), Cisco blue (#049fd9).

### v40 Bank of America
Flag blue/red, three product tier columns banking-style comparison.

## Approximations
- No headless screenshots in this run; structures derived from known marketing-site patterns.
- No trademarks/logos/product names of pastiche companies appear in any HTML.
"""
    research = OUT / "research"
    research.mkdir(exist_ok=True)
    (research / "DESIGN_NOTES.md").write_text(notes, encoding="utf-8")
    print("wrote research/DESIGN_NOTES.md")


def main():
    builders = [
        build_v1, build_v2, build_v3, build_v4, build_v5,
        build_v6, build_v7, build_v8, build_v9, build_v10,
        build_v11, build_v12, build_v13, build_v14, build_v15,
        build_v16, build_v17, build_v18, build_v19, build_v20,
        build_v21, build_v22, build_v23, build_v24, build_v25,
        build_v26, build_v27, build_v28, build_v29, build_v30,
        build_v31, build_v32, build_v33, build_v34, build_v35,
        build_v36, build_v37, build_v38, build_v39, build_v40,
    ]
    for b in builders:
        b()
    assert len(LABELS) == 40, f"expected 40 labels, got {len(LABELS)}"
    write_viewer()
    write_design_notes()
    print("done:", len(LABELS), "variants")


if __name__ == "__main__":
    main()
