#!/usr/bin/env python3
"""Generate 40 independent M17 marketing-page design variants + viewer + research.
Clean-room exercise B — does not read design_variations/."""

from pathlib import Path

OUT = Path(__file__).resolve().parent

# ---------------------------------------------------------------------------
# Shared Gearotons content (facts only from the handoff brief)
# ---------------------------------------------------------------------------

FEATURES = [
    ("All-in-one package", "Motor + driver + motion controller + encoder integrated in one compact body."),
    ("Daisy-chain RS-485", "Control any number of motors over one bus from a single connection point."),
    ("High-level commands", '"Enable MOSFETs" and "Trapezoid move" — no timing-critical STEP/DIR signals.'),
    ("NEMA 17 form factor", "Nearly the same size as a NEMA 17 stepper with the same specs — no protrusions."),
    ("Closed-loop at 32 kHz", "Built-in encoder + PID loop for high-precision position control."),
    ("12–24 V wide range", "Flexible power with integrated over-current, over-voltage, and over-temperature protection."),
    ("560 RPM max speed", "Same torque-to-weight ratio as an equivalent stepper — more power efficient in closed loop."),
    ("Cross-platform control", "Raspberry Pi, Arduino, ESP32, Mac, and PC. Excellent docs, AI-friendly."),
]

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

CODE = """import servomotor
from servomotor import M3

servomotor.open_serial_port()
motor = M3("X", time_unit="seconds",
                position_unit="degrees")
motor.system_reset()
motor.enable_mosfets()
motor.trapezoid_move(360, 2)  # one revolution in 2 s"""

APPS = [
    ("Robotics", "robotics_small.jpg", "robotics.jpg"),
    ("CNC machines", "automation_small.jpg", "automation.jpg"),
    ("Automated testing", "test_rack_small.jpg", "test_rack.jpg"),
    ("Scientific instruments", "one_motor_small.jpg", "one_motor.jpg"),
    ("Automation", "automation_small.jpg", "automation.jpg"),
    ("3D printers", "kit_with_three_motors_small.jpg", "kit_with_three_motors.jpg"),
    ("Education", "M17_series_overview.jpg", "M17_series_overview.jpg"),
]

# ---------------------------------------------------------------------------
# HTML helpers
# ---------------------------------------------------------------------------

def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
             .replace('"', "&quot;"))


def feat_html(n=6, style="cards"):
    items = FEATURES[:n]
    if style == "cards":
        return "\n".join(
            f'<article class="feat"><h3>{esc(t)}</h3><p>{esc(d)}</p></article>'
            for t, d in items
        )
    if style == "list":
        return "<ul class=\"feat-list\">" + "".join(
            f"<li><strong>{esc(t)}</strong> — {esc(d)}</li>" for t, d in items
        ) + "</ul>"
    if style == "numbered":
        return "\n".join(
            f'<div class="feat-n"><span class="n">{i:02d}</span>'
            f'<div><h3>{esc(t)}</h3><p>{esc(d)}</p></div></div>'
            for i, (t, d) in enumerate(items, 1)
        )
    return feat_html(n, "cards")


def specs_table():
    rows = "".join(
        f"<tr><th scope='row'>{esc(p)}</th>"
        f"<td>{esc(a)}</td><td>{esc(b)}</td><td>{esc(c)}</td></tr>"
        for p, a, b, c in SPECS
    )
    return f"""
<table class="specs">
<thead><tr><th>Parameter</th><th>M17-60</th><th>M17-48</th><th>M17-40</th></tr></thead>
<tbody>{rows}</tbody>
</table>"""


def models_cards():
    models = [
        ("M17-60", "0.65 N·m", "38 W", "470 g", "Highest torque for demanding loads"),
        ("M17-48", "0.55 N·m", "32 W", "360 g", "Balanced performance and size"),
        ("M17-40", "0.42 N·m", "25 W", "285 g", "Compact option for lighter axes"),
    ]
    return "\n".join(
        f"""<article class="model">
          <h3>{m}</h3>
          <p class="model-torque">{t}</p>
          <ul><li>{p} rated</li><li>{w}</li></ul>
          <p class="model-blurb">{esc(b)}</p>
        </article>"""
        for m, t, p, w, b in models
    )


def apps_html(style="grid"):
    if style == "grid":
        return "\n".join(
            f'<figure class="app"><img src="../{s}" alt="{esc(n)}" loading="lazy">'
            f'<figcaption>{esc(n)}</figcaption></figure>'
            for n, s, _ in APPS
        )
    return "\n".join(
        f'<div class="app-row"><img src="../{s}" alt="{esc(n)}" loading="lazy">'
        f'<span>{esc(n)}</span></div>'
        for n, s, _ in APPS
    )


def code_block():
    return f'<pre class="code"><code>{esc(CODE)}</code></pre>'


BASE_CSS = """
*,*::before,*::after{box-sizing:border-box}
html{scroll-behavior:smooth}
body{margin:0;line-height:1.55;-webkit-font-smoothing:antialiased}
img{max-width:100%;height:auto;display:block}
a{color:inherit}
.wrap{width:min(1120px,92%);margin:0 auto}
.btn{display:inline-block;padding:.85rem 1.4rem;text-decoration:none;font-weight:600;
  border-radius:6px;border:2px solid transparent;transition:transform .15s,background .15s,color .15s,border-color .15s}
.btn:hover{transform:translateY(-1px)}
.btn-ghost{background:transparent}
nav.top{position:sticky;top:0;z-index:50;backdrop-filter:blur(10px)}
nav.top .inner{display:flex;align-items:center;justify-content:space-between;gap:1rem;padding:.75rem 0}
nav.top .logo{display:flex;align-items:center;gap:.6rem;text-decoration:none;font-weight:700}
nav.top .logo img{height:36px;width:auto}
nav.top .links{display:flex;align-items:center;gap:1.1rem;flex-wrap:wrap}
nav.top .links a{text-decoration:none;font-size:.95rem;opacity:.9}
nav.top .links a:hover{opacity:1}
.section{padding:4.5rem 0}
.section h2{margin:0 0 1.25rem;font-size:clamp(1.6rem,3vw,2.2rem);letter-spacing:-.02em}
.lead{font-size:1.1rem;max-width:48rem;opacity:.88}
.grid-3{display:grid;grid-template-columns:repeat(3,1fr);gap:1.25rem}
.grid-2{display:grid;grid-template-columns:1fr 1fr;gap:2rem;align-items:center}
.grid-feats{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:1.1rem}
.feat,.model{padding:1.25rem;border-radius:12px}
.feat h3,.model h3{margin:0 0 .4rem;font-size:1.05rem}
.feat p,.model p{margin:0;font-size:.95rem;opacity:.9}
.model-torque{font-size:1.5rem!important;font-weight:700;margin:.3rem 0 .5rem!important}
.model ul{margin:.4rem 0;padding-left:1.1rem;opacity:.85}
table.specs{width:100%;border-collapse:collapse;font-size:.92rem}
table.specs th,table.specs td{padding:.7rem .8rem;text-align:left;border-bottom:1px solid rgba(127,127,127,.25)}
table.specs thead th{font-size:.8rem;text-transform:uppercase;letter-spacing:.04em;opacity:.7}
table.specs tbody th{font-weight:600}
pre.code{overflow-x:auto;padding:1.25rem 1.4rem;border-radius:10px;font-size:.88rem;line-height:1.5;
  font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
.apps{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:1rem}
.app{margin:0}
.app img{width:100%;aspect-ratio:1;object-fit:cover;border-radius:10px}
.app figcaption{margin-top:.45rem;font-size:.9rem;text-align:center}
footer.site{padding:3rem 0 2.5rem}
footer.site .cta-row{display:flex;flex-wrap:wrap;gap:.8rem;margin:1.2rem 0}
.muted{opacity:.7;font-size:.9rem}
.hero{padding:4rem 0 3rem}
.hero h1{margin:0 0 1rem;font-size:clamp(2rem,5vw,3.4rem);letter-spacing:-.03em;line-height:1.1}
.hero .tag{font-size:1.15rem;max-width:36rem;margin:0 0 1.5rem;opacity:.88}
.hero-cta{display:flex;flex-wrap:wrap;gap:.75rem}
.badge{display:inline-block;font-size:.75rem;font-weight:700;letter-spacing:.08em;text-transform:uppercase;
  padding:.35rem .7rem;border-radius:999px;margin-bottom:1rem}
.os-logos{display:flex;gap:1rem;align-items:center;flex-wrap:wrap;margin:1rem 0}
.os-logos img{height:48px;width:auto}
.feat-n{display:flex;gap:1rem;align-items:flex-start;margin-bottom:1.2rem}
.feat-n .n{font-weight:800;font-size:1.2rem;opacity:.4;min-width:2.2rem}
.feat-list{padding-left:1.2rem}
.feat-list li{margin-bottom:.7rem}
.dim-row{display:grid;grid-template-columns:repeat(3,1fr);gap:1rem;margin-top:1.5rem}
.dim-row figure{margin:0}
.dim-row figcaption{margin-top:.35rem;font-size:.85rem}
@media(max-width:800px){
  .grid-3,.grid-2,.dim-row{grid-template-columns:1fr}
  .hero{padding:2.5rem 0 2rem}
  nav.top .links a.hide-sm{display:none}
  table.specs{display:block;overflow-x:auto;-webkit-overflow-scrolling:touch}
}
"""


def page(title, css, body, extra_head=""):
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{esc(title)}</title>
<style>
{BASE_CSS}
{css}
</style>
{extra_head}
</head>
<body>
{body}
</body>
</html>
"""


def nav(d, logo_word=True, shop_class="btn btn-primary", logo_dark=False):
    logo = "../Gearotons_Logo.png"
    # wordmark only on white backgrounds — callers set logo_word False for dark
    mark = ""
    if logo_word and not logo_dark:
        mark = f'<img src="../Gearotons_Logo_and_Gearotons_Name.png" alt="Gearotons" style="height:40px">'
    else:
        mark = f'<img src="../Gearotons_Logo.png" alt="Gearotons"><span class="brand-name">Gearotons</span>'
    return f"""
<nav class="top">
  <div class="wrap inner">
    <a class="logo" href="#top">{mark}</a>
    <div class="links">
      <a class="hide-sm" href="#features">Features</a>
      <a class="hide-sm" href="#models">Models</a>
      <a class="hide-sm" href="#start">Get started</a>
      <a class="hide-sm" href="#apps">Applications</a>
      <a class="{shop_class}" href="https://gearotons.com/store">Shop Now</a>
    </div>
  </div>
</nav>"""


def footer_block(extra_class=""):
    return f"""
<footer class="site {extra_class}" id="about">
  <div class="wrap">
    <h2>Open source. Built for makers and engineers.</h2>
    <p class="lead">Gearotons is an innovative startup making precision motion control accessible —
    for makers, educators, and industry. Founded in Shenzhen in 2022 by a Canadian entrepreneur.
    Firmware, libraries, and schematics are open on GitHub.</p>
    <div class="os-logos">
      <img src="../Open_Source_Initiative.svg.png" alt="Open Source Initiative">
      <img src="../Open-source-hardware-logo.svg.png" alt="Open Source Hardware">
    </div>
    <div class="cta-row">
      <a class="btn btn-primary" href="https://gearotons.com/store">Shop Now</a>
      <a class="btn btn-ghost" href="https://gearotons.com">Documentation</a>
      <a class="btn btn-ghost" href="https://github.com/tomrodinger/servomotor">GitHub</a>
    </div>
    <p class="muted">© Gearotons · M17 Series Servomotors · Affordable and Simple All-in-One Motion Control</p>
  </div>
</footer>"""


def standard_sections(d, hero_img="../M17_series_overview.jpg", hero_img_alt="M17 Series overview",
                      use_transparent=False, feat_style="cards", feat_n=6):
    himg = hero_img
    if use_transparent:
        himg = "../transparent/M17_series_overview_transparent_small.png"
    return f"""
<header class="hero" id="top">
  <div class="wrap grid-2">
    <div>
      <span class="badge">{d.get('badge','M17 Series')}</span>
      <h1>{d.get('h1','All-in-one motion control, without the complexity')}</h1>
      <p class="tag">{d.get('tag','Affordable and Simple All-in-One Motion Control. Motor, driver, controller, and encoder in one NEMA 17 package — daisy-chain over RS-485.')}</p>
      <div class="hero-cta">
        <a class="btn btn-primary" href="https://gearotons.com/store">Shop Now</a>
        <a class="btn btn-ghost" href="https://gearotons.com">Documentation</a>
      </div>
    </div>
    <div class="hero-visual">
      <img src="{himg}" alt="{esc(hero_img_alt)}">
    </div>
  </div>
</header>

<section class="section" id="intro">
  <div class="wrap">
    <h2>{d.get('intro_h','Precision motion, one package')}</h2>
    <p class="lead">The M17 Series Servomotors integrate a motor, motor driver, motion controller, and encoder
    in a single compact package. RS-485 lets you daisy-chain any number of motors from one connection.
    Three models — M17-60, M17-48, M17-40 — offer flexible torque with consistent control characteristics.
    Ideal for robotics, CNC, automated testing, scientific instruments, and education.</p>
  </div>
</section>

<section class="section" id="features">
  <div class="wrap">
    <h2>Key features</h2>
    <div class="grid-feats">{feat_html(feat_n, feat_style)}</div>
  </div>
</section>

<section class="section" id="models">
  <div class="wrap">
    <h2>Three models. One control language.</h2>
    <p class="lead">Pick torque and budget — same protocol, same NEMA 17 mounting, same software.</p>
    <div class="grid-3" style="margin:1.5rem 0 2rem">{models_cards()}</div>
    {specs_table()}
    <div class="dim-row">
      <figure><img src="../M17-60_dimensions.png" alt="M17-60 dimensions" loading="lazy"><figcaption class="muted">M17-60</figcaption></figure>
      <figure><img src="../M17-48_dimensions.png" alt="M17-48 dimensions" loading="lazy"><figcaption class="muted">M17-48</figcaption></figure>
      <figure><img src="../M17-40_dimensions.png" alt="M17-40 dimensions" loading="lazy"><figcaption class="muted">M17-40</figcaption></figure>
    </div>
  </div>
</section>

<section class="section" id="start">
  <div class="wrap grid-2">
    <div>
      <h2>From pip install to motion in minutes</h2>
      <p class="lead">Control from Mac, PC, Raspberry Pi, Arduino, or ESP32 with a low-cost RS485 adapter.
      Install the Python library or grab the Arduino library from the Library Manager.</p>
      <p class="muted">pip3 install servomotor · Arduino Library Manager: “Servomotor”</p>
      <div style="margin-top:1rem">
        <img src="../adapter_and_wire_small.jpg" alt="RS485 adapter and cable" style="border-radius:10px;max-width:360px">
      </div>
    </div>
    <div>{code_block()}</div>
  </div>
</section>

<section class="section" id="apps">
  <div class="wrap">
    <h2>Built for real machines</h2>
    <p class="lead">Robotics · CNC · Automated testing · Scientific instruments · Automation · 3D printers · Education</p>
    <div class="apps" style="margin-top:1.5rem">{apps_html()}</div>
  </div>
</section>

<section class="section" id="kit">
  <div class="wrap grid-2">
    <img src="../kit_with_three_motors_small.jpg" alt="M17 kit with three motors" style="border-radius:12px">
    <div>
      <h2>Everything you need to start</h2>
      <p class="lead">Motors, adapter PCB, cables, and knobs — a complete path from unboxing to closed-loop motion.
      Standard NEMA 17 mounting. Robust protocol. Self-calibration, LED status, trapezoid profiles, comprehensive error handling.</p>
      <a class="btn btn-primary" href="https://gearotons.com/store">Shop the M17 Series</a>
    </div>
  </div>
</section>
""" + footer_block()


# ---------------------------------------------------------------------------
# Design definitions: v1–v10 original, v11–v40 company pastiches
# ---------------------------------------------------------------------------

DESIGNS = []

# ---- Round 1: originals ---------------------------------------------------

def add(num, label, company, css, body_fn, notes=""):
    DESIGNS.append({
        "num": num, "label": label, "company": company,
        "css": css, "body_fn": body_fn, "notes": notes,
    })


# v1 Precision Lab
add(1, "Precision Lab", None, """
:root{--bg:#f7f8f6;--ink:#121412;--green:#7AB648;--line:#d8ddd4;--card:#fff;--muted:#5a6158}
body{font-family:Avenir,Helvetica Neue,Helvetica,Arial,sans-serif;background:var(--bg);color:var(--ink)}
nav.top{background:rgba(247,248,246,.92);border-bottom:1px solid var(--line)}
.btn-primary{background:var(--green);color:#fff;border-color:var(--green)}
.btn-primary:hover{background:#68993c}
.btn-ghost{border-color:var(--ink);color:var(--ink)}
.btn-ghost:hover{background:var(--ink);color:#fff}
.badge{background:rgba(122,182,72,.15);color:#3d6b1f}
.feat,.model{background:var(--card);border:1px solid var(--line)}
table.specs{background:#fff;border:1px solid var(--line)}
pre.code{background:#1a1f1a;color:#e8f0e4}
.hero-visual img{border-radius:4px;border:1px solid var(--line)}
.section:nth-child(even){background:#fff}
footer.site{background:#121412;color:#e8f0e4}
footer.site .btn-ghost{border-color:#e8f0e4;color:#e8f0e4}
footer.site .btn-primary{background:var(--green)}
""", lambda: standard_sections({"h1":"Laboratory-grade motion in a NEMA 17 footprint","badge":"M17 Series · Precision"}))

# v2 Dark Command
add(2, "Dark Command", None, """
:root{--bg:#0b0d0c;--ink:#e8efe6;--green:#8fd45a;--card:#141816;--line:#243028}
body{font-family:ui-sans-serif,system-ui,Segoe UI,Helvetica,Arial,sans-serif;background:var(--bg);color:var(--ink)}
nav.top{background:rgba(11,13,12,.9);border-bottom:1px solid var(--line)}
.brand-name{color:var(--ink)}
.btn-primary{background:var(--green);color:#0b0d0c;border-color:var(--green)}
.btn-primary:hover{filter:brightness(1.08)}
.btn-ghost{border-color:var(--green);color:var(--green)}
.btn-ghost:hover{background:var(--green);color:#0b0d0c}
.badge{background:rgba(143,212,90,.12);color:var(--green);border:1px solid rgba(143,212,90,.35)}
.feat,.model{background:var(--card);border:1px solid var(--line);box-shadow:0 0 0 1px rgba(143,212,90,.05)}
.feat:hover,.model:hover{border-color:var(--green)}
table.specs th,table.specs td{border-color:var(--line)}
pre.code{background:#050605;color:#b8f08a;border:1px solid var(--line)}
.hero{background:radial-gradient(ellipse at 70% 20%,rgba(143,212,90,.12),transparent 50%),var(--bg)}
.hero-visual img{filter:drop-shadow(0 20px 40px rgba(0,0,0,.5))}
footer.site{border-top:1px solid var(--line);background:#050605}
.app img{border:1px solid var(--line)}
""", lambda: standard_sections({
    "h1": "Command every axis from one bus",
    "tag": "Dark, dense control for engineers who ship machines. All-in-one servos over RS-485.",
    "badge": "CLOSED-LOOP · 32 kHz",
}, use_transparent=True, hero_img="../transparent/one_motor_transparent_small.png", hero_img_alt="M17 motor"))

# v3 Editorial Magazine
add(3, "Editorial Serif", None, """
:root{--bg:#faf8f4;--ink:#1a1814;--accent:#7AB648;--rule:#cfc8bc}
body{font-family:Georgia,Times New Roman,serif;background:var(--bg);color:var(--ink)}
nav.top{background:var(--bg);border-bottom:1px solid var(--rule)}
nav.top .links a{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:.85rem;letter-spacing:.04em;text-transform:uppercase}
.btn{font-family:Avenir,Helvetica,Arial,sans-serif;border-radius:0;letter-spacing:.03em}
.btn-primary{background:var(--ink);color:#fff;border-color:var(--ink)}
.btn-primary:hover{background:var(--accent);border-color:var(--accent)}
.btn-ghost{border-color:var(--ink)}
.badge{font-family:Avenir,Helvetica,Arial,sans-serif;background:transparent;border-bottom:2px solid var(--accent);border-radius:0;padding:.2rem 0;letter-spacing:.12em}
.hero h1{font-weight:400;font-size:clamp(2.4rem,6vw,4rem);max-width:14ch}
.hero .tag{font-size:1.25rem;font-style:italic}
.section h2{font-weight:400;border-bottom:1px solid var(--rule);padding-bottom:.6rem}
.feat,.model{background:transparent;border-top:2px solid var(--ink);border-radius:0;padding:1rem 0}
.grid-feats{gap:2rem}
table.specs{font-family:Avenir,Helvetica,Arial,sans-serif}
pre.code{font-family:Menlo,Consolas,monospace;background:#1a1814;color:#faf8f4;border-radius:0}
.hero-visual img{border-radius:0}
footer.site{background:#1a1814;color:#faf8f4}
footer.site .btn-primary{background:var(--accent);border-color:var(--accent);color:#fff}
footer.site .btn-ghost{border-color:#faf8f4;color:#faf8f4}
.apps{gap:1.5rem}
.app figcaption{font-style:italic}
@media(min-width:900px){
  .hero .grid-2{grid-template-columns:1.1fr .9fr}
}
""", lambda: standard_sections({
    "h1": "Motion, edited for clarity",
    "tag": "Affordable and Simple All-in-One Motion Control — a product story for builders who care about craft.",
    "badge": "Featured product",
}, feat_style="list", feat_n=8))

# v4 Soft Consumer
add(4, "Soft Consumer", None, """
:root{--bg:#f3f6f1;--ink:#243028;--green:#7AB648;--soft:#e4efd8;--card:#fff}
body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,Helvetica,Arial,sans-serif;background:var(--bg);color:var(--ink)}
nav.top{background:rgba(243,246,241,.95)}
.btn{border-radius:999px;padding:.9rem 1.6rem}
.btn-primary{background:var(--green);color:#fff;border:none;box-shadow:0 8px 24px rgba(122,182,72,.35)}
.btn-primary:hover{box-shadow:0 10px 28px rgba(122,182,72,.45)}
.btn-ghost{border-color:rgba(36,48,40,.2);background:#fff}
.badge{background:var(--soft);color:#3d6b1f}
.hero{padding:5rem 0 3rem}
.hero h1{font-weight:700;letter-spacing:-.04em}
.feat,.model{background:var(--card);border:none;border-radius:20px;box-shadow:0 10px 30px rgba(36,48,40,.06)}
.section#features{background:linear-gradient(180deg,var(--soft),var(--bg))}
pre.code{background:#243028;color:#d8f0c4;border-radius:16px}
.hero-visual img{border-radius:24px;box-shadow:0 20px 50px rgba(36,48,40,.12)}
.app img{border-radius:16px}
footer.site{background:#fff;border-top:1px solid #e0e6dc}
""", lambda: standard_sections({
    "h1": "Motion control that feels simple",
    "tag": "Plug in, daisy-chain, move. The M17 Series is all-in-one servomotor control for makers and teams.",
    "badge": "Friendly power",
}))

# v5 Blueprint Tech
add(5, "Blueprint Grid", None, """
:root{--bg:#0a2744;--ink:#d6e8ff;--line:rgba(120,180,255,.22);--accent:#7AB648;--paper:#071c33}
body{font-family:Menlo,Consolas,ui-monospace,monospace;background:
  linear-gradient(var(--line) 1px,transparent 1px),
  linear-gradient(90deg,var(--line) 1px,transparent 1px),
  var(--bg);background-size:32px 32px,32px 32px,auto;color:var(--ink);font-size:14px}
nav.top{background:rgba(7,28,51,.92);border-bottom:1px solid var(--line);font-size:12px;letter-spacing:.06em;text-transform:uppercase}
.brand-name{letter-spacing:.08em}
.btn{border-radius:0;font-size:12px;letter-spacing:.08em;text-transform:uppercase}
.btn-primary{background:var(--accent);color:#041018;border-color:var(--accent)}
.btn-ghost{border-color:var(--ink);color:var(--ink)}
.badge{border:1px solid var(--line);background:transparent;color:#9ec5ff;border-radius:0}
.hero h1{font-family:Avenir,Helvetica,Arial,sans-serif;font-size:clamp(1.8rem,4vw,2.8rem);text-transform:uppercase;letter-spacing:.04em}
.hero .tag,.lead,.feat p{font-family:Avenir,Helvetica,Arial,sans-serif}
.section h2{font-family:Avenir,Helvetica,Arial,sans-serif;text-transform:uppercase;font-size:1.2rem;letter-spacing:.1em;color:#9ec5ff}
.feat,.model{background:rgba(7,28,51,.85);border:1px solid var(--line);border-radius:0}
table.specs{background:rgba(7,28,51,.9);font-size:12px}
table.specs th,table.specs td{border-color:var(--line)}
pre.code{background:#041018;border:1px solid var(--line);border-radius:0;color:#9dffb0}
.hero-visual{border:1px dashed var(--line);padding:1rem;background:rgba(7,28,51,.6)}
footer.site{background:#041018;border-top:1px solid var(--line)}
.app img{border:1px solid var(--line);border-radius:0}
.dim-row figure{border:1px solid var(--line);padding:.5rem;background:rgba(255,255,255,.95)}
.dim-row figcaption{color:#0a2744!important;opacity:1!important;font-size:11px}
""", lambda: standard_sections({
    "h1": "SPEC: M17 ALL-IN-ONE SERVO",
    "tag": "DWG · NEMA17 · RS485 · 12–24V · CLOSED LOOP 32kHz · TORQUE OPTIONS 0.42–0.65 N·m",
    "badge": "REV A · OPEN SOURCE",
}, use_transparent=True, feat_style="numbered", feat_n=8))

# v6 Bold Poster
add(6, "Bold Poster", None, """
:root{--bg:#fff;--ink:#0a0a0a;--green:#7AB648;--red:#e23}
body{font-family:Impact,Haettenschweiler,Arial Black,sans-serif;background:var(--bg);color:var(--ink)}
nav.top{background:#0a0a0a;color:#fff}
nav.top .links a{font-family:Arial,Helvetica,sans-serif;font-weight:700;text-transform:uppercase;font-size:.8rem}
.brand-name{text-transform:uppercase;letter-spacing:.05em}
.btn{font-family:Arial,Helvetica,sans-serif;border-radius:0;text-transform:uppercase;letter-spacing:.06em;font-weight:800}
.btn-primary{background:var(--green);color:#000;border:3px solid #000}
.btn-primary:hover{background:#000;color:var(--green)}
.btn-ghost{border:3px solid #000;color:#000}
.btn-ghost:hover{background:#000;color:#fff}
.badge{font-family:Arial,sans-serif;background:var(--green);color:#000;border-radius:0}
.hero{padding:3rem 0;border-bottom:6px solid #000}
.hero h1{font-size:clamp(3rem,10vw,6.5rem);line-height:.9;text-transform:uppercase;max-width:12ch}
.hero .tag{font-family:Arial,Helvetica,sans-serif;font-weight:600;font-size:1.2rem;max-width:28rem}
.section h2{font-size:clamp(2rem,5vw,3rem);text-transform:uppercase;line-height:1;border-left:12px solid var(--green);padding-left:1rem}
.lead,.feat p,.model p,table,.muted,p{font-family:Arial,Helvetica,sans-serif}
.feat,.model{border:3px solid #000;border-radius:0;background:#fff;box-shadow:6px 6px 0 #000}
.feat h3,.model h3{text-transform:uppercase;font-size:1.3rem}
table.specs{border:3px solid #000}
table.specs th,table.specs td{border-bottom:2px solid #000;font-family:Arial,sans-serif}
pre.code{border:3px solid #000;border-radius:0;background:#0a0a0a;color:#b8ff6a}
.hero-visual img{border:4px solid #000}
footer.site{background:var(--green);color:#000;border-top:6px solid #000}
footer.site .btn-primary{background:#000;color:var(--green);border-color:#000}
footer.site .btn-ghost{border-color:#000;color:#000}
.app img{border:3px solid #000;border-radius:0}
""", lambda: standard_sections({
    "h1": "ONE BOX. FULL CONTROL.",
    "tag": "Motor. Driver. Controller. Encoder. Daisy-chain the rest.",
    "badge": "M17",
}, feat_n=4))

# v7 Swiss Grid
add(7, "Swiss Grid", None, """
:root{--bg:#fff;--ink:#111;--red:#e30613;--gray:#666;--line:#111}
body{font-family:Helvetica Neue,Helvetica,Arial,sans-serif;background:var(--bg);color:var(--ink);font-size:16px}
nav.top{background:#fff;border-bottom:2px solid #111}
.btn{border-radius:0;font-weight:700;text-transform:uppercase;font-size:.75rem;letter-spacing:.08em}
.btn-primary{background:var(--red);color:#fff;border-color:var(--red)}
.btn-primary:hover{background:#111;border-color:#111}
.btn-ghost{border:2px solid #111}
.badge{background:transparent;color:var(--red);border:none;padding:0;font-size:.7rem}
.hero{padding:5rem 0 3rem}
.hero h1{font-weight:700;font-size:clamp(2.2rem,5vw,3.6rem);letter-spacing:-.04em;max-width:16ch}
.hero .grid-2{gap:3rem}
.section{padding:3.5rem 0;border-top:1px solid #ddd}
.section h2{font-size:1rem;text-transform:uppercase;letter-spacing:.12em;color:var(--red);font-weight:700}
.lead{max-width:40rem;color:var(--gray)}
.feat,.model{border:none;border-top:2px solid #111;border-radius:0;padding:1rem 0 0}
.grid-feats{gap:2rem}
table.specs{font-size:.85rem}
table.specs thead th{color:var(--red)}
pre.code{border-radius:0;background:#111;color:#eee}
.hero-visual{border-left:2px solid #111;padding-left:2rem}
footer.site{background:#111;color:#fff;border-top:none}
footer.site .btn-primary{background:var(--red)}
footer.site .btn-ghost{border-color:#fff;color:#fff}
.wrap{width:min(1080px,90%)}
""", lambda: standard_sections({
    "h1": "M17 Series Servomotors",
    "tag": "Affordable and Simple All-in-One Motion Control.",
    "badge": "Gearotons / Product",
}, feat_style="numbered", feat_n=6))

# v8 Warm Workshop
add(8, "Warm Workshop", None, """
:root{--bg:#f0e6d4;--ink:#2c2118;--wood:#8b5a2b;--green:#5f8f3c;--card:#faf3e6;--line:#d4c4a8}
body{font-family:Georgia,Palatino,Times New Roman,serif;background:var(--bg);color:var(--ink)}
nav.top{background:rgba(240,230,212,.95);border-bottom:1px solid var(--line)}
nav.top .links a{font-family:Avenir,Helvetica,Arial,sans-serif}
.btn{font-family:Avenir,Helvetica,Arial,sans-serif;border-radius:4px}
.btn-primary{background:var(--wood);color:#faf3e6;border-color:var(--wood)}
.btn-primary:hover{background:#6e4520}
.btn-ghost{border-color:var(--wood);color:var(--wood)}
.badge{background:#e8d5b5;color:var(--wood);font-family:Avenir,sans-serif}
.hero h1{font-weight:400}
.section h2{color:var(--wood)}
.feat,.model{background:var(--card);border:1px solid var(--line);border-radius:6px;box-shadow:2px 3px 0 rgba(44,33,24,.06)}
pre.code{background:#2c2118;color:#e8d5b5;border-radius:6px}
.hero-visual img{border:8px solid #fff;box-shadow:0 8px 24px rgba(44,33,24,.15);border-radius:2px;transform:rotate(-1deg)}
footer.site{background:#2c2118;color:#f0e6d4}
footer.site .btn-primary{background:var(--green);border-color:var(--green);color:#fff}
footer.site .btn-ghost{border-color:#f0e6d4;color:#f0e6d4}
.app img{border:4px solid #fff;box-shadow:0 4px 12px rgba(44,33,24,.12)}
""", lambda: standard_sections({
    "h1": "Workshop-ready closed-loop power",
    "tag": "For the bench, the lab, and the shop floor. Open-source tools, honest hardware.",
    "badge": "Maker friendly",
}))

# v9 Neon Arcade
add(9, "Neon Arcade", None, """
:root{--bg:#12081f;--ink:#f5e9ff;--pink:#ff4ecd;--cyan:#3df0ff;--green:#7dff6a;--card:#1c0f2e}
body{font-family:Trebuchet MS,Segoe UI,Helvetica,Arial,sans-serif;background:var(--bg);color:var(--ink)}
nav.top{background:rgba(18,8,31,.9);border-bottom:1px solid rgba(255,78,205,.35)}
.brand-name{background:linear-gradient(90deg,var(--pink),var(--cyan));-webkit-background-clip:text;color:transparent}
.btn{border-radius:8px;font-weight:800;text-transform:uppercase;letter-spacing:.05em}
.btn-primary{background:var(--pink);color:#12081f;border:none;box-shadow:0 0 20px rgba(255,78,205,.45)}
.btn-primary:hover{background:var(--cyan);box-shadow:0 0 24px rgba(61,240,255,.5)}
.btn-ghost{border-color:var(--cyan);color:var(--cyan)}
.badge{background:transparent;border:1px solid var(--green);color:var(--green)}
.hero{background:radial-gradient(circle at 20% 30%,rgba(255,78,205,.2),transparent 40%),
  radial-gradient(circle at 80% 20%,rgba(61,240,255,.15),transparent 35%),var(--bg)}
.hero h1{text-shadow:0 0 30px rgba(255,78,205,.35)}
.feat,.model{background:var(--card);border:1px solid rgba(255,78,205,.3);border-radius:14px}
.feat h3{color:var(--cyan)}
.model-torque{color:var(--green)!important}
table.specs th,table.specs td{border-color:rgba(255,78,205,.2)}
pre.code{background:#0a0412;color:var(--green);border:1px solid rgba(61,240,255,.3)}
footer.site{background:#0a0412;border-top:1px solid rgba(255,78,205,.3)}
.app img{border:2px solid rgba(61,240,255,.4)}
""", lambda: standard_sections({
    "h1": "Level up your motion game",
    "tag": "High-score closed-loop control for robots, CNC, and wild maker projects.",
    "badge": "PLAYER 1 · INSERT ADAPTER",
}, use_transparent=True, hero_img="../transparent/kit_with_three_motors_transparent_small.png"))

# v10 Luxury Minimal
add(10, "Luxury Minimal", None, """
:root{--bg:#fafafa;--ink:#1a1a1a;--soft:#888;--line:#e8e8e8;--accent:#7AB648}
body{font-family:Avenir Next,Avenir,Helvetica Neue,Helvetica,Arial,sans-serif;background:var(--bg);color:var(--ink);letter-spacing:.01em}
nav.top{background:rgba(250,250,250,.9);border-bottom:1px solid var(--line)}
.btn{border-radius:0;padding:1rem 2rem;font-weight:500;letter-spacing:.12em;text-transform:uppercase;font-size:.7rem}
.btn-primary{background:var(--ink);color:#fff;border-color:var(--ink)}
.btn-primary:hover{background:var(--accent);border-color:var(--accent)}
.btn-ghost{border:1px solid var(--ink)}
.badge{background:transparent;color:var(--soft);letter-spacing:.2em;padding:0}
.hero{padding:6rem 0 4rem}
.hero h1{font-weight:300;font-size:clamp(2.2rem,5vw,3.8rem);letter-spacing:-.02em;max-width:14ch}
.hero .tag{color:var(--soft);font-weight:300;font-size:1.05rem}
.section{padding:5rem 0}
.section h2{font-weight:300;letter-spacing:.08em;text-transform:uppercase;font-size:1rem;color:var(--soft)}
.lead{font-weight:300;color:#444}
.feat,.model{background:transparent;border:none;border-bottom:1px solid var(--line);border-radius:0;padding:1.5rem 0}
.grid-feats{gap:0 2rem}
table.specs{font-weight:300}
pre.code{background:#1a1a1a;color:#ccc;border-radius:0;font-weight:400}
.hero-visual img{max-width:90%;margin:0 auto}
footer.site{background:#1a1a1a;color:#fafafa;padding:5rem 0}
footer.site .btn-primary{background:#fafafa;color:#1a1a1a}
footer.site .btn-ghost{border-color:#fafafa;color:#fafafa}
.wrap{width:min(1000px,88%)}
""", lambda: standard_sections({
    "h1": "Quiet power. Clear intent.",
    "tag": "All-in-one servomotor control, reduced to essentials.",
    "badge": "M17",
}, feat_n=4))


# ---- Round 2: company pastiches (design system only) ---------------------

def company_css(theme):
    """theme keys: bg, ink, accent, accent_ink, card, line, font, nav_bg, btn_radius, hero_extra, dark"""
    r = theme.get("btn_radius", "6px")
    dark = theme.get("dark", False)
    font = theme.get("font", "system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif")
    return f"""
:root{{
  --bg:{theme['bg']};--ink:{theme['ink']};--accent:{theme['accent']};--accent-ink:{theme.get('accent_ink','#fff')};
  --card:{theme.get('card', theme['bg'])};--line:{theme.get('line','rgba(127,127,127,.25)')};
  --nav:{theme.get('nav_bg', theme['bg'])};--muted:{theme.get('muted','inherit')};
}}
body{{font-family:{font};background:var(--bg);color:var(--ink)}}
nav.top{{background:var(--nav);border-bottom:1px solid var(--line)}}
.brand-name{{color:var(--ink);font-weight:700}}
.btn{{border-radius:{r}}}
.btn-primary{{background:var(--accent);color:var(--accent-ink);border-color:var(--accent)}}
.btn-primary:hover{{filter:brightness(1.06)}}
.btn-ghost{{border-color:var(--ink);color:var(--ink);opacity:.9}}
.btn-ghost:hover{{background:var(--ink);color:var(--bg)}}
.badge{{background:rgba(127,127,127,.12);color:var(--ink)}}
.feat,.model{{background:var(--card);border:1px solid var(--line)}}
table.specs th,table.specs td{{border-color:var(--line)}}
pre.code{{background:{'#0d1117' if not dark else '#000'};color:{'#e6edf3' if not dark else '#7ee787'};border:1px solid var(--line)}}
.hero{{{theme.get('hero_extra','')}}}
footer.site{{background:{theme.get('footer_bg', theme['ink'] if not dark else '#000')};color:{theme.get('footer_ink', theme['bg'] if not dark else theme['ink'])};border-top:1px solid var(--line)}}
footer.site .btn-primary{{background:var(--accent);color:var(--accent-ink)}}
footer.site .btn-ghost{{border-color:currentColor;color:inherit}}
{theme.get('extra','')}
"""


COMPANIES = [
    # (num, name, label, theme, body overrides)
    (11, "NVIDIA", "NVIDIA", {
        "bg": "#000000", "ink": "#ffffff", "accent": "#76b900", "accent_ink": "#000",
        "card": "#111111", "line": "#2a2a2a", "nav_bg": "rgba(0,0,0,.9)", "dark": True,
        "font": "Helvetica Neue,Helvetica,Arial,sans-serif", "btn_radius": "4px",
        "hero_extra": "background:radial-gradient(ellipse at 60% 0%,rgba(118,185,0,.2),transparent 50%),#000;padding:5rem 0 3rem",
        "footer_bg": "#000", "footer_ink": "#fff",
        "extra": ".badge{color:#76b900;background:rgba(118,185,0,.12)}.hero h1{font-weight:700;letter-spacing:-.03em}.section h2{color:#76b900}.model-torque{color:#76b900!important}",
    }, {"h1": "The engine of motion intelligence", "tag": "All-in-one closed-loop control for every axis you build.", "badge": "M17 SERIES"}),

    (12, "Apple", "Apple", {
        "bg": "#fbfbfd", "ink": "#1d1d1f", "accent": "#0071e3", "accent_ink": "#fff",
        "card": "#fff", "line": "rgba(0,0,0,.08)", "nav_bg": "rgba(251,251,253,.8)",
        "font": "-apple-system,BlinkMacSystemFont,SF Pro Text,Helvetica Neue,Helvetica,Arial,sans-serif",
        "btn_radius": "980px",
        "hero_extra": "padding:5.5rem 0 3rem;text-align:center",
        "footer_bg": "#f5f5f7", "footer_ink": "#1d1d1f",
        "extra": """
.hero .grid-2{display:block;text-align:center}
.hero .tag{margin:0 auto 1.5rem;max-width:32rem}
.hero-cta{justify-content:center}
.hero-visual{margin:2.5rem auto 0;max-width:720px}
.hero h1{font-size:clamp(2.5rem,6vw,4.5rem);font-weight:600;letter-spacing:-.03em}
.section h2{font-weight:600;letter-spacing:-.02em;text-align:center}
.lead{margin-left:auto;margin-right:auto;text-align:center}
.feat,.model{border:none;border-radius:18px;background:#fff;box-shadow:0 2px 12px rgba(0,0,0,.04)}
.btn-ghost{border:none;color:#0071e3;background:transparent}
.btn-ghost:hover{background:transparent;color:#0077ed;text-decoration:underline}
footer.site{text-align:center}
footer.site .cta-row{justify-content:center}
""",
    }, {"h1": "M17. Motion, simplified.", "tag": "Motor, driver, controller, and encoder. One beautiful package.", "badge": "New"}),

    (13, "Alphabet", "Google", {
        "bg": "#fff", "ink": "#202124", "accent": "#1a73e8", "accent_ink": "#fff",
        "card": "#fff", "line": "#dadce0", "nav_bg": "#fff",
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "4px",
        "footer_bg": "#f8f9fa", "footer_ink": "#202124",
        "extra": """
.badge{background:#e8f0fe;color:#1967d2;border-radius:4px}
.btn-primary{box-shadow:0 1px 2px rgba(60,64,67,.3),0 1px 3px 1px rgba(60,64,67,.15)}
.feat,.model{border-radius:8px;box-shadow:0 1px 2px rgba(60,64,67,.3),0 1px 3px 1px rgba(60,64,67,.15);border:none}
.section h2{font-weight:400;font-size:1.75rem}
.hero h1{font-weight:400;font-size:clamp(2rem,4vw,3rem)}
nav.top{box-shadow:0 1px 2px rgba(60,64,67,.15)}
""",
    }, {"h1": "Motion control that just works", "tag": "Affordable and simple all-in-one servomotors — search no further for closed-loop axes.", "badge": "Product"}),

    (14, "Microsoft", "Microsoft", {
        "bg": "#fff", "ink": "#242424", "accent": "#0067b8", "accent_ink": "#fff",
        "card": "#f5f5f5", "line": "#e0e0e0", "nav_bg": "#fff",
        "font": "Segoe UI,SegoeUI,Helvetica Neue,Helvetica,Arial,sans-serif", "btn_radius": "2px",
        "footer_bg": "#f2f2f2", "footer_ink": "#242424",
        "extra": """
.btn-primary{background:#0067b8}
.badge{background:#e6f2fb;color:#0067b8}
.hero{background:linear-gradient(135deg,#f3f9ff,#fff 50%);padding:4rem 0}
.section h2{font-weight:600}
.feat h3{font-weight:600}
.grid-feats .feat{border-left:3px solid #0067b8}
""",
    }, {"h1": "Empower every machine builder", "tag": "Professional-grade all-in-one motion control for education and industry.", "badge": "M17 Series"}),

    (15, "Amazon", "Amazon", {
        "bg": "#fff", "ink": "#0f1111", "accent": "#ff9900", "accent_ink": "#111",
        "card": "#fff", "line": "#d5d9d9", "nav_bg": "#131921",
        "font": "Amazon Ember,Arial,sans-serif", "btn_radius": "8px",
        "footer_bg": "#232f3e", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}
nav.top .links a{color:#fff}
.brand-name{color:#fff}
.btn-primary{background:linear-gradient(180deg,#f7dfa5,#f0c14b);border-color:#a88734 #9c7e31 #846a29;color:#111}
.btn-primary:hover{filter:brightness(1.02)}
.btn-ghost{border-color:#d5d9d9;background:#fff}
.hero h1{font-size:clamp(1.8rem,3.5vw,2.4rem);font-weight:700}
.badge{background:#232f3e;color:#fff;border-radius:4px}
.feat,.model{border-radius:8px}
.section#models .model{border-top:3px solid #ff9900}
""",
    }, {"h1": "M17 Series Servomotors", "tag": "All-in-one motion control. Everyday low complexity. Shop motors, adapters, and more.", "badge": "Best seller potential"}),

    (16, "TSMC", "TSMC", {
        "bg": "#f5f7fa", "ink": "#1a2332", "accent": "#003366", "accent_ink": "#fff",
        "card": "#fff", "line": "#cfd8e3", "nav_bg": "#003366",
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "2px",
        "footer_bg": "#001a33", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#e6eef7}
.btn-primary{background:#c8102e;border-color:#c8102e}
.badge{background:#e8eef5;color:#003366}
.section h2{color:#003366;border-bottom:2px solid #c8102e;display:inline-block;padding-bottom:.3rem}
.hero{background:linear-gradient(180deg,#e8eef5,#f5f7fa)}
.feat{border-top:3px solid #003366}
""",
    }, {"h1": "Process-perfect motion hardware", "tag": "Precision manufacturing sensibility meets open-source control.", "badge": "Technology"}),

    (17, "Broadcom", "Broadcom", {
        "bg": "#fff", "ink": "#1b1b1b", "accent": "#cc092f", "accent_ink": "#fff",
        "card": "#f7f7f7", "line": "#ddd", "nav_bg": "#1b1b1b",
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "0",
        "footer_bg": "#1b1b1b", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#ddd}
.badge{background:#cc092f;color:#fff;border-radius:0}
.hero h1{font-weight:700;text-transform:uppercase;font-size:clamp(1.6rem,3vw,2.4rem);letter-spacing:.02em}
.section h2{text-transform:uppercase;font-size:1.1rem;letter-spacing:.06em}
.feat,.model{border-radius:0;border-left:4px solid #cc092f}
""",
    }, {"h1": "Connectivity-class motion control", "tag": "Enterprise-reliable RS-485 daisy-chain servomotors for serious systems.", "badge": "Infrastructure"}),

    (18, "SpaceX", "SpaceX", {
        "bg": "#000", "ink": "#fff", "accent": "#fff", "accent_ink": "#000",
        "card": "#0a0a0a", "line": "#222", "nav_bg": "rgba(0,0,0,.85)", "dark": True,
        "font": "Helvetica Neue,Helvetica,Arial,sans-serif", "btn_radius": "0",
        "hero_extra": "min-height:70vh;display:flex;align-items:center;background:linear-gradient(180deg,rgba(0,0,0,.2),#000),#0a0a0a",
        "footer_bg": "#000", "footer_ink": "#fff",
        "extra": """
.brand-name{letter-spacing:.2em;text-transform:uppercase;font-size:.85rem}
.btn{text-transform:uppercase;letter-spacing:.12em;font-size:.75rem;font-weight:600;padding:1rem 2rem}
.btn-primary{background:transparent;color:#fff;border:2px solid #fff}
.btn-primary:hover{background:#fff;color:#000}
.btn-ghost{border:2px solid #666;color:#ccc}
.hero h1{font-size:clamp(2.5rem,7vw,5rem);font-weight:700;text-transform:uppercase;letter-spacing:.04em}
.hero .tag{text-transform:uppercase;letter-spacing:.08em;font-size:.9rem;opacity:.75}
.badge{letter-spacing:.2em;border:1px solid #444;background:transparent}
.section h2{text-transform:uppercase;letter-spacing:.15em;font-size:.95rem;font-weight:600}
.feat,.model{background:transparent;border:1px solid #333;border-radius:0}
""",
    }, {"h1": "Occupy every degree of freedom", "tag": "All-in-one servomotors. Rapid iteration. Open source stack.", "badge": "FLIGHT HARDWARE ENERGY"}),

    (19, "Saudi Aramco", "Saudi Aramco", {
        "bg": "#f4f1ea", "ink": "#1a1a1a", "accent": "#00a3e0", "accent_ink": "#fff",
        "card": "#fff", "line": "#d9d2c5", "nav_bg": "#004c97",
        "font": "Georgia,Times New Roman,serif", "btn_radius": "2px",
        "footer_bg": "#004c97", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#e6f0fa;font-family:Arial,sans-serif}
.btn{font-family:Arial,sans-serif}
.badge{background:#00a3e0;color:#fff;font-family:Arial,sans-serif}
.section h2{color:#004c97}
.hero{background:linear-gradient(90deg,#e8f4fc,#f4f1ea)}
.feat{border-top:3px solid #00a3e0}
.lead,.hero .tag,p{font-family:Georgia,serif}
h1,h2,h3{font-family:Arial,sans-serif}
""",
    }, {"h1": "Energy for your motion systems", "tag": "Industrial-strength all-in-one control with open documentation.", "badge": "Industrial"}),

    (20, "Meta", "Meta", {
        "bg": "#fff", "ink": "#1c2b33", "accent": "#0668E1", "accent_ink": "#fff",
        "card": "#f0f2f5", "line": "#ced0d4", "nav_bg": "#fff",
        "font": "Helvetica Neue,Helvetica,Arial,sans-serif", "btn_radius": "8px",
        "footer_bg": "#1c2b33", "footer_ink": "#fff",
        "extra": """
.btn-primary{background:#0668E1;border-radius:24px;padding:.85rem 1.6rem}
.btn-ghost{border-radius:24px;border-color:#bcc0c4}
.badge{background:#e7f3ff;color:#0668E1;border-radius:8px}
.hero h1{font-weight:700;letter-spacing:-.03em}
.feat,.model{border:none;border-radius:16px}
.section#features{background:#f0f2f5}
""",
    }, {"h1": "Connect every axis", "tag": "Daisy-chain motors. Build robots, test rigs, and learning labs that move together.", "badge": "Build"}),

    (21, "Tesla", "Tesla", {
        "bg": "#fff", "ink": "#171a20", "accent": "#3e6ae1", "accent_ink": "#fff",
        "card": "#f4f4f4", "line": "#eee", "nav_bg": "rgba(255,255,255,.85)",
        "font": "Gotham,Helvetica Neue,Helvetica,Arial,sans-serif", "btn_radius": "4px",
        "hero_extra": "padding:6rem 0 4rem;text-align:center",
        "footer_bg": "#000", "footer_ink": "#fff",
        "extra": """
.hero .grid-2{display:flex;flex-direction:column;align-items:center;text-align:center}
.hero-cta{justify-content:center}
.hero h1{font-size:clamp(2.5rem,6vw,3.5rem);font-weight:500}
.hero .tag{max-width:28rem;margin:0 auto 1.5rem}
.hero-visual{margin-top:2rem;max-width:800px}
.btn{min-width:200px;text-align:center;font-weight:500}
.btn-primary{background:#3e6ae1}
.btn-ghost{background:rgba(0,0,0,.05);border:none}
.btn-ghost:hover{background:rgba(0,0,0,.1);color:inherit}
.section h2{text-align:center;font-weight:500}
.lead{text-align:center;margin-left:auto;margin-right:auto}
.badge{background:transparent;color:#5c5e62;letter-spacing:.1em}
.feat{background:#f4f4f4;border:none;border-radius:8px}
footer.site{text-align:center}
footer.site .cta-row{justify-content:center}
""",
    }, {"h1": "M17 Series", "tag": "All-in-one motion. Accelerated development.", "badge": "Servomotors"}),

    (22, "Samsung", "Samsung", {
        "bg": "#000", "ink": "#fff", "accent": "#1428a0", "accent_ink": "#fff",
        "card": "#121212", "line": "#2a2a2a", "nav_bg": "#000", "dark": True,
        "font": "SamsungOne,Helvetica Neue,Arial,sans-serif", "btn_radius": "20px",
        "footer_bg": "#000", "footer_ink": "#fff",
        "extra": """
.btn-primary{background:#1428a0}
.btn-ghost{border-color:#fff;color:#fff}
.badge{color:#2189ff;background:rgba(33,137,255,.12)}
.hero h1{font-weight:700;font-size:clamp(2.2rem,5vw,3.6rem)}
.section h2{font-weight:700}
.feat,.model{border-radius:16px}
.hero{background:radial-gradient(ellipse at center top,rgba(20,40,160,.35),#000 60%)}
""",
    }, {"h1": "Do what you can't — with closed-loop axes", "tag": "Galaxy of torque options in one clean NEMA 17 system.", "badge": "Innovation"}),

    (23, "Berkshire Hathaway", "Berkshire", {
        "bg": "#faf9f6", "ink": "#1c1c1c", "accent": "#1a3c6e", "accent_ink": "#fff",
        "card": "#fff", "line": "#ddd8ce", "nav_bg": "#1a3c6e",
        "font": "Times New Roman,Times,Georgia,serif", "btn_radius": "2px",
        "footer_bg": "#1a3c6e", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#dce6f5;font-family:Arial,sans-serif;font-size:.85rem}
.btn{font-family:Arial,sans-serif}
.badge{font-family:Arial,sans-serif;background:#e8eef5;color:#1a3c6e}
.hero h1{font-weight:400;font-size:clamp(2rem,4vw,2.8rem)}
.section h2{font-weight:400;border-bottom:1px solid #1a3c6e;padding-bottom:.4rem}
.feat{border:1px solid #ddd8ce;background:#fff}
table.specs{font-family:Arial,sans-serif;font-size:.88rem}
""",
    }, {"h1": "Owner's manual for serious builders", "tag": "Straightforward all-in-one servomotors. No hype — torque, docs, and open source.", "badge": "Annual letter energy"}),

    (24, "Eli Lilly", "Eli Lilly", {
        "bg": "#fff", "ink": "#212322", "accent": "#d52b1e", "accent_ink": "#fff",
        "card": "#f7f7f5", "line": "#e2e2de", "nav_bg": "#fff",
        "font": "Georgia,Times,serif", "btn_radius": "4px",
        "footer_bg": "#212322", "footer_ink": "#fff",
        "extra": """
nav.top{border-bottom:3px solid #d52b1e}
.btn{font-family:Arial,sans-serif}
.badge{background:#fde8e6;color:#d52b1e;font-family:Arial,sans-serif}
.hero h1{font-weight:400;color:#212322}
.section h2{color:#d52b1e;font-weight:400}
.feat{border-radius:8px;border-top:3px solid #d52b1e}
.lead{max-width:40rem}
""",
    }, {"h1": "Carefully engineered motion", "tag": "Reliable closed-loop control for scientific instruments and lab automation.", "badge": "Science-ready"}),

    (25, "Micron", "Micron", {
        "bg": "#0b1c2c", "ink": "#e8f1f8", "accent": "#00a3e0", "accent_ink": "#001018",
        "card": "#122536", "line": "#1e3a52", "nav_bg": "#071420", "dark": True,
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "4px",
        "footer_bg": "#071420", "footer_ink": "#e8f1f8",
        "extra": """
.badge{color:#00a3e0;background:rgba(0,163,224,.12)}
.hero{background:linear-gradient(120deg,#0b1c2c,#12324a)}
.section h2{color:#00a3e0}
.feat h3{color:#7fd4f5}
.model-torque{color:#00a3e0!important}
""",
    }, {"h1": "Density of capability per cubic centimeter", "tag": "All-in-one control silicon-adjacent thinking for mechanical systems.", "badge": "Performance memory of motion"}),

    (26, "Walmart", "Walmart", {
        "bg": "#fff", "ink": "#2e2f32", "accent": "#0071dc", "accent_ink": "#fff",
        "card": "#f2f8fd", "line": "#e3e4e5", "nav_bg": "#0071dc",
        "font": "Helvetica Neue,Helvetica,Arial,sans-serif", "btn_radius": "999px",
        "footer_bg": "#041e42", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#fff}
.btn-primary{background:#ffc220;color:#2e2f32;border-color:#ffc220;font-weight:700}
.btn-primary:hover{filter:brightness(1.05)}
.badge{background:#e6f1fc;color:#0071dc}
.hero h1{font-weight:700;color:#041e42}
.feat{border-radius:12px;border:none;background:#f2f8fd}
.section h2{color:#041e42}
""",
    }, {"h1": "Save money. Move more.", "tag": "Affordable all-in-one servomotors without cutting corners on control quality.", "badge": "Everyday low complexity"}),

    (27, "JPMorgan Chase", "JPMorgan", {
        "bg": "#fff", "ink": "#1a1a1a", "accent": "#005eb8", "accent_ink": "#fff",
        "card": "#f5f7fa", "line": "#d0d5dd", "nav_bg": "#003366",
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "0",
        "footer_bg": "#003366", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#cce0f5}
.badge{background:#e6f0fa;color:#003366;border-radius:0}
.hero{border-bottom:4px solid #005eb8}
.section h2{color:#003366;font-weight:700;font-size:1.4rem}
.feat{border-left:4px solid #005eb8;border-radius:0}
table.specs thead{background:#003366;color:#fff}
table.specs thead th{color:#fff;opacity:1}
""",
    }, {"h1": "Institutional-grade motion infrastructure", "tag": "Dependable RS-485 control architecture for multi-axis systems.", "badge": "Solutions"}),

    (28, "SK Hynix", "SK Hynix", {
        "bg": "#f7f8fa", "ink": "#111", "accent": "#e31c3d", "accent_ink": "#fff",
        "card": "#fff", "line": "#e0e3e8", "nav_bg": "#111",
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "4px",
        "footer_bg": "#111", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#ccc}
.btn-primary{background:#e31c3d}
.badge{background:#fde8ec;color:#e31c3d}
.hero{background:linear-gradient(180deg,#fff,#f7f8fa)}
.section h2{position:relative}
.feat{border-bottom:3px solid #e31c3d}
""",
    }, {"h1": "High-bandwidth control, compact package", "tag": "Semiconductor-era density applied to NEMA 17 motion.", "badge": "HBM energy for hardware"}),

    (29, "AMD", "AMD", {
        "bg": "#000", "ink": "#fff", "accent": "#ed1c24", "accent_ink": "#fff",
        "card": "#151515", "line": "#333", "nav_bg": "#000", "dark": True,
        "font": "Helvetica Neue,Arial,sans-serif", "btn_radius": "0",
        "footer_bg": "#000", "footer_ink": "#fff",
        "extra": """
.btn-primary{background:#ed1c24;border:none}
.btn-ghost{border:2px solid #fff}
.badge{color:#ed1c24;border:1px solid #ed1c24;background:transparent}
.hero h1{font-weight:800;text-transform:uppercase;letter-spacing:-.02em}
.section h2{text-transform:uppercase;letter-spacing:.06em;font-size:1.1rem}
.feat{border-top:2px solid #ed1c24;border-radius:0}
.hero{background:radial-gradient(ellipse at 80% 0%,rgba(237,28,36,.25),transparent 45%),#000}
.model-torque{color:#ed1c24!important}
""",
    }, {"h1": "Together we advance_ motion", "tag": "High-performance closed-loop control for builders who push limits.", "badge": "RYZEN OF AXES"}),

    (30, "Visa", "Visa", {
        "bg": "#fff", "ink": "#1a1f71", "accent": "#f7b600", "accent_ink": "#1a1f71",
        "card": "#f5f7fc", "line": "#d6d9e8", "nav_bg": "#1a1f71",
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "4px",
        "footer_bg": "#1a1f71", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#d0d4f0}
.btn-primary{background:#f7b600;border-color:#f7b600;font-weight:700}
.badge{background:#e8ebf8;color:#1a1f71}
.hero h1{color:#1a1f71;font-weight:700}
.section h2{color:#1a1f71}
.feat{border-radius:8px;border-bottom:3px solid #f7b600}
""",
    }, {"h1": "Accepted everywhere you build", "tag": "One protocol. Many motors. Trusted closed-loop transactions with your mechanics.", "badge": "Everywhere"}),

    (31, "ASML", "ASML", {
        "bg": "#0a0a0a", "ink": "#f2f2f2", "accent": "#ff6600", "accent_ink": "#000",
        "card": "#141414", "line": "#2c2c2c", "nav_bg": "#0a0a0a", "dark": True,
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "2px",
        "footer_bg": "#000", "footer_ink": "#f2f2f2",
        "extra": """
.btn-primary{background:#ff6600}
.badge{color:#ff6600;border:1px solid #ff6600;background:transparent}
.hero h1{font-weight:300;font-size:clamp(2rem,5vw,3.4rem);letter-spacing:-.02em}
.section h2{font-weight:300;color:#ff6600}
.feat{border:1px solid #333}
.lead{opacity:.8;font-weight:300}
""",
    }, {"h1": "Extreme precision. Accessible packaging.", "tag": "Lithography-grade attention to detail in open-source motion hardware.", "badge": "Systems"}),

    (32, "Exxon Mobil", "ExxonMobil", {
        "bg": "#fff", "ink": "#1c1c1c", "accent": "#ff0000", "accent_ink": "#fff",
        "card": "#f6f6f6", "line": "#ddd", "nav_bg": "#000",
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "0",
        "footer_bg": "#000", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#ccc}
.btn-primary{background:#ff0000}
.badge{background:#ff0000;color:#fff;border-radius:0}
.hero{background:#f6f6f6}
.section h2{text-transform:uppercase;letter-spacing:.08em;font-size:1rem;border-left:6px solid #ff0000;padding-left:.8rem}
.feat{border-radius:0}
""",
    }, {"h1": "Power your automation chain", "tag": "Industrial motion energy — motors that keep production moving.", "badge": "Energy · Industry"}),

    (33, "Johnson & Johnson", "J&J", {
        "bg": "#fff", "ink": "#212322", "accent": "#d51900", "accent_ink": "#fff",
        "card": "#f9f7f4", "line": "#e5e0d8", "nav_bg": "#fff",
        "font": "Georgia,Times,serif", "btn_radius": "24px",
        "footer_bg": "#212322", "footer_ink": "#fff",
        "extra": """
nav.top{border-bottom:1px solid #e5e0d8}
.btn{font-family:Arial,sans-serif}
.btn-primary{background:#d51900}
.badge{font-family:Arial,sans-serif;background:#fde8e4;color:#d51900}
.hero h1{font-weight:400;color:#212322}
.section h2{font-weight:400}
.feat{border-radius:16px}
.lead{font-size:1.15rem}
""",
    }, {"h1": "Healthier machines start with better control", "tag": "Safe, protected closed-loop motion for labs, education, and automation.", "badge": "Care · Science"}),

    (34, "Tencent", "Tencent", {
        "bg": "#fff", "ink": "#111", "accent": "#12b7f5", "accent_ink": "#fff",
        "card": "#f5f6f7", "line": "#e5e5e5", "nav_bg": "#fff",
        "font": "Helvetica Neue,PingFang SC,Arial,sans-serif", "btn_radius": "4px",
        "footer_bg": "#111", "footer_ink": "#fff",
        "extra": """
.btn-primary{background:#12b7f5}
.badge{background:#e6f8fe;color:#0a8ec4}
.hero h1{font-weight:600}
.feat{border-radius:8px;border:none;box-shadow:0 2px 8px rgba(0,0,0,.04)}
.section#features{background:#f5f6f7}
nav.top{box-shadow:0 1px 0 #eee}
""",
    }, {"h1": "Connect · Control · Create", "tag": "Multi-motor networks with simple high-level commands for the next project.", "badge": "Open platform"}),

    (35, "Intel", "Intel", {
        "bg": "#fff", "ink": "#1e1e1e", "accent": "#0071c5", "accent_ink": "#fff",
        "card": "#f0f0f0", "line": "#ddd", "nav_bg": "#0068b5",
        "font": "Intel Clear,Helvetica Neue,Arial,sans-serif", "btn_radius": "0",
        "footer_bg": "#1e1e1e", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#e6f3fc}
.btn-primary{background:#00aeef;border-color:#00aeef}
.badge{background:#e6f6fc;color:#0071c5}
.hero{background:linear-gradient(90deg,#0071c5 0%,#00aeef 40%,#fff 40.1%)}
@media(max-width:800px){.hero{background:#f4faff}}
.hero h1{font-weight:700}
.section h2{color:#0071c5}
.feat{border-top:4px solid #00aeef;border-radius:0}
""",
    }, {"h1": "Processors for position", "tag": "32 kHz PID loops on every motor — compute where the motion happens.", "badge": "Technology"}),

    (36, "Mastercard", "Mastercard", {
        "bg": "#fafafa", "ink": "#141413", "accent": "#eb001b", "accent_ink": "#fff",
        "card": "#fff", "line": "#e0e0e0", "nav_bg": "#000",
        "font": "Helvetica Neue,Arial,sans-serif", "btn_radius": "999px",
        "footer_bg": "#000", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#ddd}
.btn-primary{background:#141413;border-color:#141413}
.btn-primary:hover{background:#eb001b;border-color:#eb001b}
.badge{background:linear-gradient(90deg,#eb001b,#f79e1b);color:#fff}
.hero h1{font-weight:700;letter-spacing:-.03em}
.feat{border-radius:20px;box-shadow:0 8px 24px rgba(0,0,0,.06);border:none}
.section h2{font-weight:700}
""",
    }, {"h1": "Priceless control, honest hardware", "tag": "Start something great with all-in-one servomotors and open-source tools.", "badge": "For builders"}),

    (37, "AbbVie", "AbbVie", {
        "bg": "#fff", "ink": "#1d252d", "accent": "#071d49", "accent_ink": "#fff",
        "card": "#f4f6f8", "line": "#d5dbe3", "nav_bg": "#071d49",
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "4px",
        "footer_bg": "#071d49", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#c5d0e0}
.btn-primary{background:#e87722;border-color:#e87722}
.badge{background:#e87722;color:#fff}
.section h2{color:#071d49}
.feat{border-left:4px solid #e87722}
.hero{background:#f4f6f8}
""",
    }, {"h1": "Research-grade motion, production-ready", "tag": "Servomotors for instruments, automation, and discovery workflows.", "badge": "Life sciences adjacent"}),

    (38, "Applied Materials", "Applied Materials", {
        "bg": "#0d1b2a", "ink": "#eef2f6", "accent": "#00a3a1", "accent_ink": "#042",
        "card": "#132536", "line": "#243b53", "nav_bg": "#0a1520", "dark": True,
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "2px",
        "footer_bg": "#0a1520", "footer_ink": "#eef2f6",
        "extra": """
.btn-primary{background:#00a3a1;color:#042}
.badge{color:#00a3a1;background:rgba(0,163,161,.12)}
.hero{background:linear-gradient(135deg,#0d1b2a,#16324a)}
.section h2{color:#5ee0de}
.feat h3{color:#9ef0ee}
""",
    }, {"h1": "Materials meet motion", "tag": "Semiconductor equipment DNA applied to accessible multi-axis control.", "badge": "Make possible"}),

    (39, "Cisco", "Cisco", {
        "bg": "#fff", "ink": "#1b1c1d", "accent": "#049fd9", "accent_ink": "#fff",
        "card": "#f5f8fa", "line": "#d2dbe2", "nav_bg": "#1b1c1d",
        "font": "Helvetica Neue,Arial,sans-serif", "btn_radius": "4px",
        "footer_bg": "#1b1c1d", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#cfd6db}
.btn-primary{background:#049fd9}
.badge{background:#e6f7fc;color:#0179a8}
.hero h1{font-weight:300;font-size:clamp(2rem,4vw,3rem)}
.section h2{font-weight:300;color:#0179a8}
.feat{border-radius:4px;border-top:2px solid #049fd9}
.lead{max-width:42rem}
""",
    }, {"h1": "Networking for motors", "tag": "RS-485 bus architecture: one controller, many intelligent endpoints.", "badge": "Bridge to possibility"}),

    (40, "Bank of America", "Bank of America", {
        "bg": "#fff", "ink": "#012169", "accent": "#e31837", "accent_ink": "#fff",
        "card": "#f5f7fb", "line": "#c9d1e3", "nav_bg": "#012169",
        "font": "Arial,Helvetica,sans-serif", "btn_radius": "2px",
        "footer_bg": "#012169", "footer_ink": "#fff",
        "extra": """
nav.top{color:#fff}.brand-name{color:#fff}nav.top .links a{color:#c5d0f0}
.btn-primary{background:#e31837;border-color:#e31837}
.badge{background:#e31837;color:#fff}
.hero{background:linear-gradient(180deg,#e8eef8,#fff)}
.section h2{color:#012169;font-weight:700}
.feat{border-bottom:3px solid #e31837}
table.specs thead th{color:#012169}
""",
    }, {"h1": "Invest in axes that pay off", "tag": "Clear specs, open tools, and three torque tiers to match your budget.", "badge": "Higher standards"}),
]


def build_company_designs():
    for num, name, label, theme, overrides in COMPANIES:
        dark = theme.get("dark", False)
        use_t = dark
        hero = "../transparent/one_motor_transparent_small.png" if dark else "../M17_series_overview.jpg"
        css = company_css(theme)

        def make_body(ov=overrides, d=dark, h=hero, ut=use_t):
            return (nav({}, logo_word=not d, logo_dark=d)
                    + standard_sections(ov, hero_img=h, use_transparent=ut,
                                        hero_img_alt="M17 product"))

        add(num, label, name, css, make_body,
            notes=f"Pastiche of {name} public marketing design language (colors, type, density, CTAs). No trademarks used.")


def write_research():
    research = OUT / "research"
    research.mkdir(exist_ok=True)
    ranking = """# Top 30 companies by market capitalization

**Source:** CompaniesMarketCap.com (live ranking fetched 2026-07-17)
**Note:** SpaceX appears on this source's ranking (private valuation estimate). Others are public companies.

| Rank | Company | Market Cap (source) | Variant |
|-----:|---------|---------------------|---------|
| 1 | NVIDIA | $5.023 T | v11 |
| 2 | Apple | $4.894 T | v12 |
| 3 | Alphabet (Google) | $4.317 T | v13 |
| 4 | Microsoft | $2.979 T | v14 |
| 5 | Amazon | $2.688 T | v15 |
| 6 | TSMC | $2.125 T | v16 |
| 7 | Broadcom | $1.781 T | v17 |
| 8 | SpaceX | $1.727 T | v18 |
| 9 | Saudi Aramco | $1.719 T | v19 |
| 10 | Meta Platforms | $1.686 T | v20 |
| 11 | Tesla | $1.468 T | v21 |
| 12 | Samsung | $1.133 T | v22 |
| 13 | Berkshire Hathaway | $1.063 T | v23 |
| 14 | Eli Lilly | $1.042 T | v24 |
| 15 | Micron Technology | $963.59 B | v25 |
| 16 | Walmart | $914.78 B | v26 |
| 17 | JPMorgan Chase | $912.16 B | v27 |
| 18 | SK Hynix | $884.88 B | v28 |
| 19 | AMD | $816.83 B | v29 |
| 20 | Visa | $694.40 B | v30 |
| 21 | ASML | $685.56 B | v31 |
| 22 | Exxon Mobil | $604.95 B | v32 |
| 23 | Johnson & Johnson | $601.73 B | v33 |
| 24 | Tencent | $550.89 B | v34 |
| 25 | Intel | $487.42 B | v35 |
| 26 | Mastercard | $487.33 B | v36 |
| 27 | AbbVie | $449.45 B | v37 |
| 28 | Applied Materials | $445.35 B | v38 |
| 29 | Cisco | $432.21 B | v39 |
| 30 | Bank of America | $431.53 B | v40 |

## Round 1 original directions (v1–v10)

| Variant | Label | Direction |
|---------|-------|-----------|
| v1 | Precision Lab | Light engineering lab, brand green, crisp cards |
| v2 | Dark Command | Dark industrial HUD, neon green, transparent product |
| v3 | Editorial Serif | Magazine/editorial, Georgia, large type, rules |
| v4 | Soft Consumer | Rounded consumer-tech, soft greens, pill CTAs |
| v5 | Blueprint Grid | Blueprint grid, monospace, technical drawings |
| v6 | Bold Poster | Impact poster, thick borders, brutalist type |
| v7 | Swiss Grid | International Typographic Style, red accent, Helvetica |
| v8 | Warm Workshop | Kraft/paper workshop, wood tones, maker craft |
| v9 | Neon Arcade | Playful cyberpunk/neon for makers |
| v10 | Luxury Minimal | Sparse luxury, thin weights, generous whitespace |
"""
    (research / "RANKING_AND_MAPPING.md").write_text(ranking, encoding="utf-8")

    notes = """# Design language notes (round 2 pastiches)

Research method: live market-cap ranking from CompaniesMarketCap.com (2026-07-17);
design notes from established public brand systems and knowledge of each company's
current marketing website. Automated full-page screenshot capture was not available
in this environment; notes below are from brand-system knowledge + site familiarity.

**Hard rule applied:** no company trademarks, logos, product names, or marketing copy.
Only layout/type/color/spacing/CTA patterns. All page content is Gearotons M17.

## Per-company notes

### v11 NVIDIA
Black canvas, neon lime (#76b900), Helvetica-like sans, tech-hero radial glow,
uppercase energy in product storytelling, high contrast cards.

### v12 Apple
SF-like system stack, huge centered product hero, pill blue CTA (#0071e3),
lots of white, soft gray footer (#f5f5f7), minimal chrome, gentle shadows.

### v13 Alphabet / Google
Material-ish white, blue primary (#1a73e8), soft elevation cards, Arial,
friendly product headline weight 400, light nav shadow.

### v14 Microsoft
Segoe UI feel, Fluent-adjacent blues (#0067b8), left accent bars on cards,
corporate gradient hero, rounded-2px controls.

### v15 Amazon
Dark navy nav (#131921), orange gold CTA gradient, compact product title,
e-commerce density cues without copying Amazon UI chrome.

### v16 TSMC
Corporate navy/red manufacturing palette, uppercase section labels,
technical trust, clean tables, blue nav bar.

### v17 Broadcom
Black nav, crimson accent, uppercase industrial headlines, sharp 0-radius,
left red rail on feature cards.

### v18 SpaceX
Full black, white hairline buttons, huge uppercase headlines, letter-spacing,
mission-brief density, minimal ornament.

### v19 Saudi Aramco
Deep blue + cyan industrial energy, serif body for institutional tone,
Arabic-gulf corporate polish without any Aramco marks.

### v20 Meta
Meta blue (#0668E1), pill CTAs, soft gray section bands, rounded 16px cards,
connective social-product layout language.

### v21 Tesla
Centered cinematic hero, muted gray type, blue order-style CTA, near-fullscreen
product photography treatment, minimal nav.

### v22 Samsung
OLED black, Samsung blue accents, bold innovation headlines, large radius CTAs.

### v23 Berkshire Hathaway
Traditional serif editorial + navy institutional header — annual-report calm,
no gimmicks, tables first.

### v24 Eli Lilly
Medical red accent, serif headlines, clinical whitespace, science-instrument tone.

### v25 Micron
Deep tech blue-black, cyan accent, performance density language, dark product stage.

### v26 Walmart
Spark-yellow CTA on blue nav, friendly rounded buttons, value-forward headline.

### v27 JPMorgan Chase
Navy corporate, data-table emphasis, left blue rules, finance-grade trust layout.

### v28 SK Hynix
Black nav, Korean-tech red accent, clean white engineering cards.

### v29 AMD
Black + AMD red, aggressive uppercase, angular 0-radius, performance radial hero.

### v30 Visa
Deep Visa blue + gold CTA, "everywhere" global trust layout, clean sans.

### v31 ASML
Near-black precision systems site language, orange accent, light font weights.

### v32 ExxonMobil
Black/red energy industrial, uppercase section rules, heavy corporate bars.

### v33 Johnson & Johnson
Red care accent, serif warmth, soft rounded cards, health-adjacent calm.

### v34 Tencent
Light Chinese-internet product UI: cyan accent, soft cards, simple connect narrative.

### v35 Intel
Classic Intel blue gradient hero band, cyan secondary, compute-forward copy layout.

### v36 Mastercard
Black nav, red-gold badge gradient cue (not logo), rounded modern cards.

### v37 AbbVie
Deep pharma navy + orange secondary CTA, research instrumentation tone.

### v38 Applied Materials
Dark industrial teal, semiconductor equipment aesthetics, "make possible" energy.

### v39 Cisco
Dark charcoal nav, Cisco blue (#049fd9), light font weights, networking metaphor.

### v40 Bank of America
Flag blue/red corporate, value + tiers messaging, clear tabular specs.

## Approximations / blocked access
- No headless browser screenshots captured in this run.
- SpaceX design based on spacex.com public aesthetic (black/white mission style).
- Some regional sites (SK Hynix, Tencent, Aramco) approximated from known brand systems
  when full automated fetch of marketing pages was limited.
"""
    (research / "DESIGN_NOTES.md").write_text(notes, encoding="utf-8")


def write_viewer(labels):
    items = []
    for i, lab in labels:
        items.append(
            f'<button type="button" class="tab" data-src="v{i}.html" data-n="{i}">'
            f'<span class="num">{i}</span><span class="lab">{esc(lab)}</span></button>'
        )
    tabs = "\n".join(items)
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Design Variations B — M17 Gallery</title>
<style>
*{{box-sizing:border-box}}
body{{margin:0;font-family:system-ui,Segoe UI,Helvetica,Arial,sans-serif;background:#111;color:#eee;height:100vh;display:flex;flex-direction:column}}
header{{padding:.65rem 1rem;background:#1a1a1a;border-bottom:1px solid #333;display:flex;align-items:center;justify-content:space-between;gap:1rem;flex-wrap:wrap}}
header h1{{margin:0;font-size:1rem;font-weight:600}}
header .meta{{font-size:.8rem;opacity:.7}}
.controls{{display:flex;gap:.4rem;align-items:center}}
.controls button{{background:#2a2a2a;color:#eee;border:1px solid #444;padding:.4rem .7rem;border-radius:6px;cursor:pointer}}
.controls button:hover{{background:#3a3a3a}}
.layout{{display:grid;grid-template-columns:220px 1fr;flex:1;min-height:0}}
.tabs{{overflow:auto;border-right:1px solid #333;background:#161616;padding:.4rem}}
.tab{{display:flex;gap:.55rem;align-items:center;width:100%;text-align:left;background:transparent;border:none;color:#ccc;padding:.45rem .55rem;border-radius:6px;cursor:pointer;font:inherit}}
.tab:hover{{background:#242424}}
.tab.active{{background:#2d4a22;color:#fff}}
.tab .num{{font-variant-numeric:tabular-nums;opacity:.55;min-width:1.6rem;font-size:.8rem}}
.tab .lab{{font-size:.82rem}}
.frame-wrap{{min-height:0;background:#000}}
iframe{{width:100%;height:100%;border:0;background:#fff}}
@media(max-width:700px){{
  .layout{{grid-template-columns:1fr;grid-template-rows:140px 1fr}}
  .tabs{{display:flex;overflow-x:auto;overflow-y:hidden;border-right:none;border-bottom:1px solid #333}}
  .tab{{min-width:120px;flex-direction:column;align-items:flex-start}}
}}
</style>
</head>
<body>
<header>
  <div>
    <h1>M17 Design Variations B</h1>
    <div class="meta">40 independent variants · Round 1 original · Round 2 company pastiches</div>
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


def main():
    build_company_designs()
    assert len(DESIGNS) == 40, f"expected 40 designs, got {len(DESIGNS)}"
    labels = []
    for d in sorted(DESIGNS, key=lambda x: x["num"]):
        n = d["num"]
        body = nav({}, logo_word=(n not in (2, 5, 9) and not (11 <= n <= 40 and d.get("company") in {
            "NVIDIA", "SpaceX", "Samsung", "Micron", "AMD", "ASML", "Applied Materials",
            "Amazon", "Broadcom", "TSMC", "Saudi Aramco", "Berkshire Hathaway", "Walmart",
            "JPMorgan Chase", "SK Hynix", "Visa", "Exxon Mobil", "Mastercard", "AbbVie",
            "Cisco", "Bank of America", "Meta",
        })), logo_dark=(n in (2, 5, 9) or (d.get("company") in {
            "NVIDIA", "SpaceX", "Samsung", "Micron", "AMD", "ASML", "Applied Materials",
            "Amazon", "Broadcom", "TSMC", "Saudi Aramco", "Berkshire Hathaway", "Walmart",
            "JPMorgan Chase", "SK Hynix", "Visa", "Exxon Mobil", "Mastercard", "AbbVie",
            "Cisco", "Bank of America",
        })))
        # body_fn already includes nav for company designs; originals need nav prepended
        if n <= 10:
            full_body = nav({}, logo_word=(n not in (2, 5, 9)), logo_dark=(n in (2, 5, 9))) + d["body_fn"]()
        else:
            full_body = d["body_fn"]()
        html = page("Gearotons — M17 Series Servomotors", d["css"], full_body)
        path = OUT / f"v{n}.html"
        path.write_text(html, encoding="utf-8")
        labels.append((n, d["label"]))
        print(f"wrote {path.name} — {d['label']}")

    write_viewer(labels)
    write_research()
    print("wrote index.html + research/")
    print("done:", len(labels), "variants")


if __name__ == "__main__":
    main()
