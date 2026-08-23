#!/usr/bin/env python3
# ============================================================================
#  ARCHIVED - DO NOT RUN unless you know exactly what you are doing.
#
#  This assembled final/index.html ONCE, from the four designs Tom kept.
#  final/index.html is now HAND-AUTHORED - see MARKETING_PAGE_WORKFLOW.md.
#  Running this OVERWRITES the page and destroys every hand edit since.
#
#  It refuses to run without --i-know-this-overwrites-hand-edits.
# ============================================================================
"""Build the combined final page from the four designs Tom kept.

Synthesis, and where each part comes from:

  v1  Director's Cut     -> the whole white body: calm Apple-keynote typography, generous
                            whitespace, all sixteen content sections, correct four-motor data,
                            and a black code section (which is what v17's note asked for)
  v4  Midnight Inverse   -> a DARK hero: slogan, transition and motor on black
                            ("I like the top part ... on the black background. Below that we
                             should go to a white background")
  v17 Terminal Minimal   -> the key-spec strip under the hero (NEMA 17 / 12-24 V / closed loop /
                            32 kHz) and the mono micro-labels
                            ("the slogans and motor and key specs")
  v19 Scroll Choreography-> scroll-reveal on the body sections

The headline runs on the shared slogan engine rather than v1's bespoke cycler, so it gets the
canonical two-line rest state and can rotate through SEVERAL transitions — which is what Tom asked
for. Which effects are in rotation comes from ratings/transitions.json (everything he marked
"keep"); with no ratings yet it falls back to a hand-picked set.

  python3 build_final.py [--hero dark|light]
"""
import argparse
import json
import os
import re

ROOT = os.path.dirname(os.path.abspath(__file__))
VAR = os.path.join(ROOT, "premium_minimal_variations")
LAB = os.path.join(ROOT, "slogan_lab")
OUT = os.path.join(ROOT, "final")

FALLBACK_POOL = ["keynote-lift", "light-scanner", "wipe-gradient", "dissolve-focus-pull",
                 "slide-push", "hx-spotlight"]


def liked_effects():
    """Effects Tom marked keep. Falls back to a sane default before he has rated."""
    p = os.path.join(ROOT, "ratings", "transitions.json")
    try:
        d = json.load(open(p, encoding="utf-8"))
        keep = [k for k, v in d.items() if v.get("verdict") == "keep"]
        if keep:
            return keep, "ratings/transitions.json (%d kept)" % len(keep)
    except Exception:
        pass
    return FALLBACK_POOL, "fallback set (no keeps rated yet)"


def effect_sources(ids):
    """Inline only the effect files needed to satisfy `ids`, plus the engine."""
    files, wanted = [], set(ids)
    for fn in sorted(os.listdir(os.path.join(LAB, "effects"))):
        if not fn.endswith(".js"):
            continue
        src = open(os.path.join(LAB, "effects", fn), encoding="utf-8").read()
        have = set(re.findall(r"""\bid\s*:\s*['"]([a-z0-9][a-z0-9-]*)['"]""", src))
        if have & wanted:
            files.append((fn, src))
    return files


HERO = """
<section class="hero-final" id="top" data-hero="__HEROMODE__">
  <div class="wrap">
    <p class="eyebrow">M17 Series Servomotors</p>
    <h1 aria-live="off"><span id="slogan-stage"><span class="fx-light">__L0__</span><span class="fx-bold">__B0__</span></span></h1>
    <p class="lede">Motor, driver, motion controller and encoder — gathered into a single NEMA&nbsp;17
      body. You speak in degrees and seconds; it resolves the rest.</p>
    <div class="hero-cta">
      <a class="btn-primary" href="https://gearotons.com/store">Shop the M17 Series</a>
      <a class="arrow-link" href="https://gearotons.com">Read the documentation <span class="arr">&rarr;</span></a>
    </div>
    <div class="hero-img">
      <img src="../transparent/one_motor_transparent_small.png"
           alt="Gearotons M17 servomotor, three-quarter view">
    </div>
    <!-- v17's key-spec strip -->
    <div class="keyspec">
      <div><b>NEMA 17</b><span>standard mount</span></div>
      <div><b>12&ndash;24 V</b><span>supply range</span></div>
      <div><b>Closed loop</b><span>built-in encoder</span></div>
      <div><b>Four sizes</b><span>0.28 &ndash; 0.65 N&middot;m</span></div>
    </div>
  </div>
</section>
"""

CSS = """
/* ---------- final page: dark hero (v4) + v17 key-spec strip, white body (v1) ---------- */
.hero-final{
  --hero-bg:#0A0B0C; --hero-ink:#F5F5F7; --hero-mut:#9A9AA1; --hero-line:#26262A;
  background:var(--hero-bg); color:var(--hero-ink);
  padding:104px 24px 0; text-align:center; position:relative;
}
.hero-final[data-hero="light"]{
  --hero-bg:#FFFFFF; --hero-ink:#1d1d1f; --hero-mut:#6e6e73; --hero-line:#e8e8ed;
}
.hero-final .wrap{max-width:1080px;margin:0 auto}
.hero-final .eyebrow{
  font-size:11.5px;font-weight:600;letter-spacing:.16em;text-transform:uppercase;
  color:var(--hero-mut);margin-bottom:26px;
  font-family:ui-monospace,SFMono-Regular,Menlo,monospace;   /* v17 micro-label */
}
.hero-final h1{
  font-size:clamp(38px,6.4vw,74px);font-weight:300;letter-spacing:-.03em;line-height:1.06;
  margin:0 auto;max-width:16em;
}
.hero-final h1 .fx-light{font-weight:300;display:block}
.hero-final h1 .fx-bold{font-weight:650;display:block}
#slogan-stage{position:relative;display:block}
.hero-final .lede{
  font-size:clamp(16px,1.9vw,20px);color:var(--hero-mut);max-width:44em;
  margin:26px auto 0;line-height:1.5;
}
.hero-final .hero-cta{margin-top:34px;display:flex;gap:22px;justify-content:center;align-items:center;flex-wrap:wrap}
.hero-final .btn-primary{
  display:inline-block;background:#7AB648;color:#0b0b0c;font-weight:650;font-size:15px;
  border-radius:980px;padding:12px 30px;text-decoration:none;transition:background .2s ease;
}
.hero-final .btn-primary:hover{background:#8ac857}
.hero-final .arrow-link{color:var(--hero-ink);font-size:15px;text-decoration:none;display:inline-flex;gap:6px;align-items:center}
.hero-final .arrow-link .arr{color:#7AB648;transition:transform .22s ease}
.hero-final .arrow-link:hover .arr{transform:translateX(4px)}
.hero-final .hero-img{max-width:560px;margin:56px auto 0}
.hero-final .hero-img img{width:100%;height:auto;display:block;
  filter:drop-shadow(0 30px 52px rgba(0,0,0,.55))}
.hero-final[data-hero="light"] .hero-img img{filter:drop-shadow(0 22px 38px rgba(0,0,0,.16))}
.keyspec{
  display:grid;grid-template-columns:repeat(4,1fr);gap:0;margin:64px auto 0;max-width:940px;
  border-top:1px solid var(--hero-line);
}
.keyspec div{padding:20px 12px 26px;border-right:1px solid var(--hero-line)}
.keyspec div:last-child{border-right:0}
.keyspec b{display:block;font-size:19px;font-weight:600;letter-spacing:-.01em}
.keyspec span{
  display:block;margin-top:5px;color:var(--hero-mut);
  font-family:ui-monospace,SFMono-Regular,Menlo,monospace;
  font-size:10.5px;letter-spacing:.09em;text-transform:uppercase;
}
@media(max-width:760px){
  .hero-final{padding-top:64px}
  .keyspec{grid-template-columns:repeat(2,1fr)}
  .keyspec div:nth-child(2){border-right:0}
  .keyspec div:nth-child(1),.keyspec div:nth-child(2){border-bottom:1px solid var(--hero-line)}
}

/* the nav sits over the hero, so it has to follow the hero's palette */
body.hero-dark nav{
  background:rgba(10,11,12,.78);
  border-bottom:1px solid #1f2023;
}
body.hero-dark nav a,
body.hero-dark .nav-brand{color:#EDEDF0}
body.hero-dark .nav-links a{color:#9A9AA1}
body.hero-dark .nav-links a:hover{color:#fff}
body.hero-dark .nav-links a.btn.small,
body.hero-dark nav .btn{border-color:#4a4b50;color:#EDEDF0}
body.hero-dark nav .btn:hover{background:#7AB648;border-color:#7AB648;color:#0b0b0c}
/* once past the hero the page is white again, so the nav reverts */
body.hero-dark nav.past{
  background:rgba(255,255,255,.85);
  border-bottom:1px solid #e8e8ed;
}
body.hero-dark nav.past a,
body.hero-dark nav.past .nav-brand{color:#1d1d1f}
body.hero-dark nav.past .nav-links a{color:#6e6e73}
body.hero-dark nav.past .btn{border-color:#1d1d1f;color:#1d1d1f}
@media(prefers-reduced-motion:reduce){
  .hero-final .arrow-link .arr{transition:none}
}
"""

BOOT = """
<script>
/* ---- slogan cycler on the shared engine -------------------------------------------------
   Uses the canonical rest state, so every transition starts and ends on the same two-line
   rendering of the text, and any effect can follow any other. The pool is whatever Tom kept. */
(function () {
  var POOL = __POOL__;
  var SLOGANS = __SLOGANS__;
  var stage = document.getElementById('slogan-stage');
  if (!stage || !window.SloganFX) return;

  var reduce = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  /* Lock the headline to the tallest slogan BEFORE mounting, so the page never jumps as it
     cycles. Measured from the real h1 so the lock matches the live type exactly. */
  function lock() {
    var h1 = stage.parentNode, prev = stage.innerHTML, max = 0;
    var probe = document.createElement('span');
    probe.style.cssText = 'display:block;visibility:hidden;position:static';
    stage.style.height = 'auto';
    stage.appendChild(probe);
    for (var i = 0; i < SLOGANS.length; i++) {
      probe.innerHTML = '<span class="fx-light" style="display:block">' + SLOGANS[i][0] +
                        '</span><span class="fx-bold" style="display:block">' + SLOGANS[i][1] + '</span>';
      max = Math.max(max, probe.getBoundingClientRect().height);
    }
    stage.removeChild(probe);
    stage.innerHTML = prev;
    stage.style.height = Math.ceil(max) + 'px';
  }

  var pool = POOL.map(function (id) { return SloganFX.get(id); }).filter(Boolean);
  if (!pool.length) pool = [SloganFX.all()[0]];

  lock();
  var player = SloganFX.mount(stage, pool, SLOGANS, { start: 0, pick: 'sequential' });
  if (!reduce) player.play();

  document.addEventListener('visibilitychange', function () {
    if (document.hidden) player.pause(); else if (!reduce) player.play();
  });
  stage.addEventListener('mouseenter', function () { player.pause(); });
  stage.addEventListener('mouseleave', function () { if (!reduce) player.play(); });
  var rt = 0;
  window.addEventListener('resize', function () {
    clearTimeout(rt);
    rt = setTimeout(function () { lock(); }, 160);
  });
  window.__slogan = player;
})();
</script>
"""

SLOGANS = [
    ["A motor, and everything", "that usually surrounds it."],
    ["The whole drive system.", "Inside one motor."],
    ["Four systems.", "One motor."],
    ["Motion control,", "fully integrated."],
    ["You wanted motion.", "Not a wiring project."],
    ["A motor that", "needs nothing else."],
    ["The whole motion system.", "One motor."],
    ["Everything that moves it,", "inside it."],
    ["Motion control.", "All in one."],
    ["Stepper, driver, controller, encoder.", "One part number."],
    ["Bolts in like a stepper.", "Runs like a full servo."],
    ["Four bolts. One cable.", "A complete servo system."],
    ["Servomotor:", "stepper, driver, controller and encoder in one."],
    ["No driver board. No encoder to mount.", "It's all inside."],
    ["Driver, controller and encoder", "are no longer your problem."],
]


def build(hero_mode):
    src = open(os.path.join(VAR, "v1.html"), encoding="utf-8").read()

    # 1. strip v1's bespoke cycler (the script block that defines window.__slogan)
    blocks = list(re.finditer(r"<script\b[^>]*>.*?</script>", src, re.S))
    for m in reversed(blocks):
        if "__slogan" in m.group(0):
            src = src[:m.start()] + src[m.end():]

    # 2. swap the hero
    hs = src.index('<section class="hero')
    he = src.index("</section>", hs) + len("</section>")
    hero = (HERO.replace("__HEROMODE__", hero_mode)
                .replace("__L0__", SLOGANS[0][0])
                .replace("__B0__", SLOGANS[0][1]))
    src = src[:hs] + hero + src[he:]

    # 2b. de-emphasise Shenzhen in the MARKETING copy (Tom's call). The registered entity and
    #     address stay in the legal footer — that is the company's actual legal name.
    src = src.replace("Shenzhen, 2022.", "Founded in 2022.")
    src = src.replace("Founded in Shenzhen in 2022 by a Canadian entrepreneur",
                      "Founded in 2022 by a Canadian entrepreneur")

    # 3. our CSS last so it wins
    src = src.replace("</style>", CSS + "\n</style>", 1)

    # 4. inline the engine + only the effect files we actually need
    ids, origin = liked_effects()
    js = [open(os.path.join(LAB, "engine.js"), encoding="utf-8").read()]
    used = []
    for fn, code in effect_sources(ids):
        js.append("/* --- effects/%s --- */\n%s" % (fn, code))
        used.append(fn)
    boot = (BOOT.replace("__POOL__", json.dumps(ids))
                .replace("__SLOGANS__", json.dumps(SLOGANS, ensure_ascii=False)))
    payload = "<script>\n" + "\n".join(js) + "\n</script>\n" + boot
    src = src.replace("</body>", payload + "\n</body>", 1)

    if hero_mode == "dark":
        src = src.replace("<body>", '<body class="hero-dark">', 1)
        src = src.replace("</body>", """<script>
/* swap the nav back to the light palette once the dark hero has scrolled away */
(function(){
  var nav=document.querySelector('nav'), hero=document.querySelector('.hero-final');
  if(!nav||!hero||!window.IntersectionObserver) return;
  new IntersectionObserver(function(e){
    nav.classList.toggle('past', !e[0].isIntersecting);
  },{rootMargin:'-60px 0px 0px 0px'}).observe(hero);
})();
</script>
</body>""", 1)

    os.makedirs(OUT, exist_ok=True)
    name = "index.html" if hero_mode == "dark" else "index-light-hero.html"
    open(os.path.join(OUT, name), "w", encoding="utf-8").write(src)
    return name, ids, origin, used


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--i-know-this-overwrites-hand-edits", action="store_true",
                    help="required: this destroys hand edits to final/index.html")
    ap.add_argument("--hero", choices=["dark", "light", "both"], default="both")
    a = ap.parse_args()
    if not getattr(a, "i_know_this_overwrites_hand_edits", False):
        import sys as _sys
        print("REFUSING TO RUN.")
        print("final/index.html is hand-authored; this would overwrite it and lose hand edits.")
        print("See MARKETING_PAGE_WORKFLOW.md. If you really mean it:")
        print("  python3 ARCHIVED_build_final_from_scratch.py --i-know-this-overwrites-hand-edits")
        _sys.exit(2)
    modes = ["dark", "light"] if a.hero == "both" else [a.hero]
    for m in modes:
        name, ids, origin, used = build(m)
        print("wrote final/%s   hero=%s" % (name, m))
        print("   effects (%s): %s" % (origin, ", ".join(ids)))
        print("   inlined files: %s" % ", ".join(used))
