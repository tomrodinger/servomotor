/* Family: Light — glow, shadow and exposure.
 *
 * Six ways of using illumination, rather than motion, to trade one line for the next:
 *
 *   light-specular    a glossy highlight travels the line; the text swaps under the sheen
 *   light-neon        a neon tube stutters out and a second one stutters in (hashed, not random)
 *   light-spotlight   the house lights drop, a spot pans the stage, the lights come back up
 *   light-bloom       the line overexposes to white and the new one resolves out of the glare
 *   light-shadow      the cast shadow detaches, travels, and reforms as the new words
 *   light-scanner     a beam scans down the line, flipping it to the new one band by band
 *
 * PURITY. Every one of these is a pure function of progress t: no timers, no @keyframes, no CSS
 * transitions, no wall clock, no Math.random. The neon stutter — the one thing here that *looks*
 * random — comes from hash(step, seed) over a quantised progress, so frame(i, t) draws the exact
 * same picture every time and can be seeked, screenshotted and exported to GIF.
 *
 * NO DEAD FRAMES. The trap in a light-driven family is that "turn the light off" is an empty
 * stage. So nothing here ever goes dark on its own: the neon keeps its unlit glass visible (and
 * brightens that glass during a blackout, so the words never disappear), the spotlight leaves an
 * ambient level behind it, the bloom's white-out is a lit mass of letterforms rather than a wash,
 * and the shadow effects always have either text or a legible silhouette on screen.
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;

  /* ------------------------------------------------------------------ helpers */

  /* deterministic hash noise in [0,1) — the only source of "randomness" in this file */
  function rnd(i, s) {
    var x = Math.sin((i + 1) * 127.1 + (s + 1) * 311.7) * 43758.5453;
    return x - Math.floor(x);
  }
  function srnd(i, s) { return rnd(i, s) * 2 - 1; }

  /* A travelling boundary wants eased ends but an honest middle: a pure inOut curve has crossed
     3% of the stage at t=0.25, which reads as "nothing is happening yet". */
  function sweep(t) { return 0.40 * t + 0.60 * E.inOutCubic(t); }

  function mask(node, value) { node.style.webkitMaskImage = node.style.maskImage = value; }
  function rgba(c, a) { return 'rgba(' + c[0] + ',' + c[1] + ',' + c[2] + ',' + a.toFixed(3) + ')'; }

  function wrapOf(stage) {
    stage.innerHTML = '';
    var w = FX.el('div', 'lt-wrap', {
      position: 'relative', width: '100%', height: '100%', overflow: 'hidden',
    });
    stage.appendChild(w);
    return w;
  }

  /* One full-stage centred copy of a slogan. `clip` carries masks/clip-paths (the window),
     `inner` carries transforms and text colour (the content) — keeping them apart is what lets
     a highlight travel over text that is itself drifting. */
  function layer(wrap, s, css) {
    var clip = FX.el('div', 'lt-clip', {
      position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
      willChange: 'mask-image, clip-path, opacity',
    });
    if (css) for (var k in css) clip.style[k] = css[k];
    var inner = FX.el('div', 'lt-inner', {
      width: '100%', textAlign: 'center', willChange: 'transform, opacity, filter',
    });
    // Two ROWS, exactly like the canonical layer. A <br> between the halves used to do this, but
    // the engine forces .fx-light/.fx-bold to display:block, and a <br> sitting between two block
    // boxes generates its own anonymous line box — the headline rendered on THREE lines and jumped
    // the moment the effect layer took over. Blocks only, no <br>.
    var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
    var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
    inner.appendChild(a); inner.appendChild(b);
    clip.appendChild(inner);
    wrap.appendChild(clip);
    return { clip: clip, inner: inner };
  }

  function overlay(wrap, css) {
    var d = FX.el('div', null, {
      position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
      pointerEvents: 'none', willChange: 'opacity, background, transform',
    });
    if (css) for (var k in css) d.style[k] = css[k];
    wrap.appendChild(d);
    return d;
  }

  /* fade a decoration in at the start of the transition and out at the end, so t=0 and t=1 are
     exactly the two slogans at rest with nothing extra painted over them */
  function edges(t, a, b) { return Math.min(span(t, 0, a), span(1 - t, 0, b)); }

  /* Same, but with a DEAD BAND: exactly 0 until `d`, and again after 1-d.
     edges() only *approaches* zero at the endpoints, and the engine hands the effect frames as
     close in as t=0.008 — at which edges(t,0.06,..) is already 0.13. Thirteen percent of a
     white glare is not nothing: it is bright enough to count as ink, up in the corner where the
     canonical frame has none, which reads as a lamp switching on the instant the effect starts.
     Anything opaque that is NOT one of the two words wants this envelope, not the soft one. */
  function ornament(t, d, w) { return Math.min(span(t, d, d + w), span(1 - t, d, d + w)); }

  /* ============================================================ 1. specular sweep ==========
     A gloss highlight — the kind that runs across a polished button — travels the line. The
     swap happens exactly under the hot band, so the join between two different texts is never
     visible: it is always inside the brightest part of the frame. Four layers: a base pair
     split at the boundary, and a white "hot" pair masked to the band and clipped to their own
     side of it, so the highlight really is lit letterforms and not a bar drawn on top. */
  FX.register({
    id: 'light-specular',
    name: 'Specular Sweep',
    family: 'Light',
    blurb: 'A gloss highlight travels the line and the words change under it.',
    duration: 1150,
    dwell: 4400,
    theme: { bg: '#15171C', fg: '#E8EAF0',
             font: '"Avenir Next", "Segoe UI", Inter, system-ui, sans-serif' },
    setup: function (stage, ctx) {
      var w = wrapOf(stage);
      var L = {
        outB: layer(w, ctx.from), inB: layer(w, ctx.to),
        outH: layer(w, ctx.from), inH: layer(w, ctx.to),
      };
      [L.outH, L.inH].forEach(function (h) {
        h.inner.style.color = '#FFFFFF';
        h.clip.style.mixBlendMode = 'screen';
      });
      L.sheen = overlay(w, { mixBlendMode: 'screen' });
      stage._l = L;
    },
    frame: function (stage, t) {
      var L = stage._l;
      var x = lerp(-14, 114, sweep(t));            // boundary, % of stage width
      var xc = clamp(x, 0, 100);
      // A 0.7% feather made the join a hard cut, and where the two texts disagree mid-word the
      // seam read as a glyph collision ("wirinegrated") rather than as a highlight passing over.
      // Feather it wide enough to be a short dissolve, kept well inside the hot band.
      var f = 3.4, BW = 15;                        // boundary feather / half band width, %

      mask(L.inB.clip, 'linear-gradient(90deg, #000 ' + (x - f).toFixed(2) + '%,'
        + ' rgba(0,0,0,0) ' + (x + f).toFixed(2) + '%)');
      mask(L.outB.clip, 'linear-gradient(90deg, rgba(0,0,0,0) ' + (x - f).toFixed(2) + '%,'
        + ' #000 ' + (x + f).toFixed(2) + '%)');

      var band = 'linear-gradient(90deg, rgba(0,0,0,0) ' + (x - BW).toFixed(2) + '%,'
        + ' rgba(0,0,0,.22) ' + (x - BW * 0.55).toFixed(2) + '%,'
        + ' #000 ' + x.toFixed(2) + '%,'
        + ' rgba(0,0,0,.22) ' + (x + BW * 0.55).toFixed(2) + '%,'
        + ' rgba(0,0,0,0) ' + (x + BW).toFixed(2) + '%)';
      mask(L.inH.clip, band); mask(L.outH.clip, band);
      L.inH.clip.style.clipPath = 'inset(0 ' + (100 - xc).toFixed(2) + '% 0 0)';
      L.outH.clip.style.clipPath = 'inset(0 0 0 ' + xc.toFixed(2) + '%)';

      var hot = edges(t, 0.05, 0.05);
      L.inH.clip.style.opacity = String(hot);
      L.outH.clip.style.opacity = String(hot);
      var glow = '0 0 ' + (10 * hot).toFixed(1) + 'px rgba(255,255,255,' + (0.75 * hot).toFixed(3) + '),'
        + ' 0 0 ' + (30 * hot).toFixed(1) + 'px rgba(190,225,255,' + (0.45 * hot).toFixed(3) + ')';
      L.inH.inner.style.textShadow = glow;
      L.outH.inner.style.textShadow = glow;

      // parallax: the new line settles in from the left, the old one is nudged out to the right
      var s = E.outQuart(span(t, 0.04, 1)), o = E.inQuad(t);
      var dIn = 'translateX(' + (-20 * (1 - s)).toFixed(2) + 'px)';
      var dOut = 'translateX(' + (16 * o).toFixed(2) + 'px)';
      L.inB.inner.style.transform = L.inH.inner.style.transform = dIn;
      L.outB.inner.style.transform = L.outH.inner.style.transform = dOut;
      L.outB.inner.style.opacity = String(1 - 0.18 * o);

      // the raked sheen across the "glass" — pure decoration, it must not carry the reveal
      L.sheen.style.opacity = String(edges(t, 0.08, 0.10));
      L.sheen.style.background =
        'linear-gradient(101deg, rgba(255,255,255,0) ' + (x - BW * 2.0).toFixed(2) + '%,'
        + ' rgba(255,255,255,.07) ' + (x - BW * 0.5).toFixed(2) + '%,'
        + ' rgba(255,255,255,.30) ' + x.toFixed(2) + '%,'
        + ' rgba(255,255,255,.07) ' + (x + BW * 0.5).toFixed(2) + '%,'
        + ' rgba(255,255,255,0) ' + (x + BW * 2.0).toFixed(2) + '%)';
    },
    rest: function (stage) {
      var L = stage._l;
      mask(L.inB.clip, 'none');
      mask(L.outB.clip, 'linear-gradient(90deg, rgba(0,0,0,0) 100%, #000 101%)');
      L.inB.inner.style.transform = 'none';
      L.inH.clip.style.opacity = '0'; L.outH.clip.style.opacity = '0';
      L.sheen.style.opacity = '0';
    },
  });

  /* ================================================================ 2. neon flicker =========
     A dead tube is not an empty stage: the glass is still there. That is what keeps this one
     honest — each line has an "unlit glass" level as well as a lit level, and during the
     blackout between the two tubes the glass of the *current* line brightens, so there is
     always something to read. The stutter is hash-gated over quantised progress: irregular,
     with dim half-strikes and bright ignition pops, and byte-identical on every replay. */
  var NEON = [122, 224, 96];                       // Gearotons green, pushed toward a tube colour

  /* lit level for one tube. dir<0: burning out. dir>0: striking up. Returns 0..1.25 — over 1 is
     the ignition overdrive, which the glow uses and the ink clamps away. */
  function flicker(u, seed, dir) {
    if (u <= 0) return dir > 0 ? 0 : 1;
    if (u >= 1) return dir > 0 ? 1 : 0;
    var STEPS = 20;
    var k = Math.floor(u * STEPS);
    var r0 = rnd(k, seed), r1 = rnd(k, seed + 7), r2 = rnd(k - 1, seed);
    var pOn = dir > 0 ? clamp(1.5 * u * u, 0, 1) : clamp(1.15 - 1.5 * u, 0, 1);
    var on = r0 < pOn;
    if (dir < 0 && u < 0.16) on = true;            // hold steady just after t=0
    if (dir > 0 && u > 0.84) on = true;            // hold steady just before t=1
    if (!on) return r1 < 0.34 ? 0.12 + 0.16 * r0 : 0;
    var pop = (r2 >= (dir > 0 ? clamp(1.5 * Math.pow(Math.max(0, u - 1 / STEPS), 2), 0, 1)
                              : clamp(1.15 - 1.5 * Math.max(0, u - 1 / STEPS), 0, 1))) ? 0.28 : 0;
    return clamp(0.80 + 0.20 * r1 + pop, 0, 1.25);
  }

  /* `g` scales only the halo, never the ink. The canonical endpoints are plain unglowing text, so
     the halo has to ramp in after t=0 and back out before t=1 — otherwise a full neon bloom pops
     on the first effect frame and vanishes on the last. The ink stays at fg level while lit, which
     is what keeps the letterforms identical to the canon across the swap. */
  function tube(node, lit, glass, g) {
    var ink = clamp(lerp(glass, 1, clamp(lit, 0, 1)), 0, 1);
    var hl = clamp(lit, 0, 1.25) * (g === undefined ? 1 : g);
    node.style.color = 'rgba(255,253,244,' + ink.toFixed(3) + ')';
    if (hl < 0.02) { node.style.textShadow = 'none'; return; }
    node.style.textShadow =
      '0 0 ' + (2 + 3 * hl).toFixed(1) + 'px rgba(255,255,255,' + (0.80 * hl).toFixed(3) + '),'
      + ' 0 0 ' + (4 + 12 * hl).toFixed(1) + 'px ' + rgba(NEON, 0.85 * clamp(hl, 0, 1)) + ','
      + ' 0 0 ' + (8 + 30 * hl).toFixed(1) + 'px ' + rgba(NEON, 0.55 * clamp(hl, 0, 1)) + ','
      + ' 0 0 ' + (14 + 62 * hl).toFixed(1) + 'px ' + rgba(NEON, 0.30 * clamp(hl, 0, 1));
  }

  FX.register({
    id: 'light-neon',
    name: 'Neon Flicker',
    family: 'Light',
    blurb: 'One tube stutters out, the next strikes up — irregular, and identical every replay.',
    duration: 1600,
    dwell: 4600,
    theme: { bg: '#08090C', fg: '#FFFDF4',
             font: '"Avenir Next", "Helvetica Neue", Inter, system-ui, sans-serif' },
    setup: function (stage, ctx) {
      var w = wrapOf(stage);
      var L = {};
      L.spill = overlay(w, { mixBlendMode: 'screen' });
      L.out = layer(w, ctx.from);
      L.in = layer(w, ctx.to);
      stage._l = L;
    },
    frame: function (stage, t) {
      var L = stage._l;
      // each tube's whole life sits inside the half of the transition where it is mounted
      var lo = flicker(span(t, 0.06, 0.50), 1, -1);
      var li = flicker(span(t, 0.50, 0.94), 3, +1);
      var peak = Math.max(lo, li);
      var vis = edges(t, 0.12, 0.12);              // halo ramp, so the edges match the plain canon

      /* EXACTLY ONE TUBE IS EVER MOUNTED. The two clips used to overlap at full opacity from
         t=0.40 to t=0.54 — both tubes dark, both glasses visible, so the stage showed the old and
         the new line ghosted on top of each other at the same grey. That is the illegible smear
         this family is supposed to avoid. The sign now cuts from one tube to the other at t=0.5,
         inside the blackout, and the glass of whichever tube is mounted is boosted hard enough to
         stay readable — so every frame has one line on it and only one. */
      var hand = t < 0.5 ? 0 : 1;
      var boost = 1 - clamp(peak / 0.30, 0, 1);
      var glassO = lerp(0.13, 0.13 + 0.42 * (1 - hand), boost);
      var glassI = lerp(0.13, 0.13 + 0.42 * hand, boost);

      L.out.clip.style.opacity = hand ? '0' : '1';
      L.in.clip.style.opacity = hand ? '1' : '0';
      tube(L.out.inner, lo, glassO, vis);
      tube(L.in.inner, li, glassI, vis);

      // light spilling onto the wall behind the sign
      L.spill.style.opacity = String(clamp(peak, 0, 1) * vis);
      L.spill.style.background = 'radial-gradient(65% 120% at 50% 50%, '
        + rgba(NEON, 0.14) + ' 0%, ' + rgba(NEON, 0.05) + ' 45%, rgba(0,0,0,0) 78%)';
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.clip.style.opacity = '0';
      L.in.clip.style.opacity = '1';
      tube(L.in.inner, 1, 0.13);
      L.spill.style.opacity = '1';
    },
  });

  /* ============================================================== 3. spotlight pan ==========
     The house lights fade down, a hard spot pans across the stage, and the lights come back up
     on the new line. Ambient never reaches zero — outside the pool the text sits at a low but
     readable level, so the frame is never empty — and the swap boundary is hidden inside the
     brightest part of the pool. The beam above is a volumetric wedge that pivots on the lamp. */
  var WARM = [255, 232, 176];

  FX.register({
    id: 'light-spotlight',
    name: 'Spotlight Pan',
    family: 'Light',
    blurb: 'House lights down, a spot pans the stage and finds a different line, lights up.',
    duration: 1700,
    dwell: 4600,
    theme: { bg: '#0A0A0C', fg: '#F6F3EA',
             font: '"Avenir Next", "Segoe UI", Inter, system-ui, sans-serif' },
    setup: function (stage, ctx) {
      var w = wrapOf(stage);
      var L = {};
      L.pool = overlay(w, { mixBlendMode: 'screen' });
      L.cone = FX.el('div', null, {
        position: 'absolute', top: '-58%', height: '128%', width: '78%', left: '0',
        marginLeft: '-39%', pointerEvents: 'none',
        clipPath: 'polygon(47.5% 0%, 52.5% 0%, 100% 100%, 0% 100%)',
        background: 'linear-gradient(180deg, ' + rgba(WARM, 0.26) + ' 0%, '
          + rgba(WARM, 0.10) + ' 46%, ' + rgba(WARM, 0.02) + ' 100%)',
        filter: 'blur(6px)', mixBlendMode: 'screen',
        transformOrigin: '50% 0%', willChange: 'transform, opacity',
      });
      w.appendChild(L.cone);
      L.outB = layer(w, ctx.from); L.inB = layer(w, ctx.to);
      L.outH = layer(w, ctx.from); L.inH = layer(w, ctx.to);
      [L.outH, L.inH].forEach(function (h) {
        h.inner.style.color = '#FFF9EA';
        h.clip.style.mixBlendMode = 'screen';
      });
      stage._l = L;
    },
    frame: function (stage, t) {
      var L = stage._l;
      var x = lerp(-16, 116, sweep(t));            // pool centre, % of width
      var xc = clamp(x, 0, 100);
      // Feathered wide (was 0.7%) so the swap is a short dissolve inside the pool rather than a
      // hard cut — at 0.7% the two texts butted mid-word and read as broken glyphs.
      var f = 4.2, R = 26;                         // boundary feather / pool radius, %

      mask(L.inB.clip, 'linear-gradient(90deg, #000 ' + (x - f).toFixed(2) + '%,'
        + ' rgba(0,0,0,0) ' + (x + f).toFixed(2) + '%)');
      mask(L.outB.clip, 'linear-gradient(90deg, rgba(0,0,0,0) ' + (x - f).toFixed(2) + '%,'
        + ' #000 ' + (x + f).toFixed(2) + '%)');

      // House lights: down at the start, back up at the end, never all the way off. The ambient
      // floor was 0.26, which put the words at roughly 2:1 against this background for most of the
      // middle of the transition — technically not blank, but not readable either. 0.42 keeps the
      // "lights are down" reading while the line stays a line you can actually read.
      var AMB = 0.42;
      var dim = lerp(1, AMB, span(t, 0, 0.18));
      var up = lerp(AMB, 1, span(t, 0.80, 1));
      L.outB.inner.style.opacity = String(dim);
      L.inB.inner.style.opacity = String(Math.max(dim, up));

      var pool = 'radial-gradient(' + R + '% 116% at ' + x.toFixed(2) + '% 50%,'
        + ' #000 0%, rgba(0,0,0,.72) 52%, rgba(0,0,0,.18) 82%, rgba(0,0,0,0) 100%)';
      mask(L.inH.clip, pool); mask(L.outH.clip, pool);
      L.inH.clip.style.clipPath = 'inset(0 ' + (100 - xc).toFixed(2) + '% 0 0)';
      L.outH.clip.style.clipPath = 'inset(0 0 0 ' + xc.toFixed(2) + '%)';
      var hot = edges(t, 0.10, 0.12);
      L.inH.clip.style.opacity = String(hot);
      L.outH.clip.style.opacity = String(hot);
      var g = '0 0 ' + (12 * hot).toFixed(1) + 'px ' + rgba(WARM, 0.55 * hot)
        + ', 0 0 ' + (34 * hot).toFixed(1) + 'px ' + rgba(WARM, 0.32 * hot);
      L.inH.inner.style.textShadow = g; L.outH.inner.style.textShadow = g;

      // the pool on the floor, and the wedge of light above it
      L.pool.style.opacity = String(hot);
      L.pool.style.background = 'radial-gradient(' + (R * 1.25).toFixed(1) + '% 128% at '
        + x.toFixed(2) + '% 50%, ' + rgba(WARM, 0.20) + ' 0%, ' + rgba(WARM, 0.07) + ' 55%,'
        + ' rgba(0,0,0,0) 100%)';
      L.cone.style.opacity = String(hot * 0.9);
      L.cone.style.transform = 'translateX(' + x.toFixed(2) + '%) rotate('
        + ((x - 50) * 0.16).toFixed(2) + 'deg)';
    },
    rest: function (stage) {
      var L = stage._l;
      mask(L.inB.clip, 'none');
      mask(L.outB.clip, 'linear-gradient(90deg, rgba(0,0,0,0) 100%, #000 101%)');
      L.inB.inner.style.opacity = '1';
      L.inH.clip.style.opacity = '0'; L.outH.clip.style.opacity = '0';
      L.pool.style.opacity = '0'; L.cone.style.opacity = '0';
    },
  });

  /* ================================================================== 4. bloom out =========
     The line is driven past the sensor's range: it goes white-hot, the glare swells until the
     letterforms are barely holding together, and the new line resolves out of the same glare.
     The handover sits exactly at peak exposure, so it is hidden by light rather than by a cut.
     The peak frame is deliberately a bright mass of letterforms — occupied, not blank. */
  FX.register({
    id: 'light-bloom',
    name: 'Bloom',
    family: 'Light',
    blurb: 'The line overexposes to white and the next one resolves out of the glare.',
    duration: 1350,
    dwell: 4400,
    theme: { bg: '#0B0D11', fg: '#E7EBF3',
             font: '"Avenir Next", "Segoe UI", Inter, system-ui, sans-serif' },
    setup: function (stage, ctx) {
      var w = wrapOf(stage);
      var L = {};
      L.veil = overlay(w, { mixBlendMode: 'screen' });
      L.outG = layer(w, ctx.from); L.inG = layer(w, ctx.to);      // the blurred halo copies
      L.out = layer(w, ctx.from); L.in = layer(w, ctx.to);        // the sharp copies
      [L.outG, L.inG].forEach(function (h) {
        h.inner.style.color = '#FFFFFF';
        h.clip.style.mixBlendMode = 'screen';
      });
      stage._l = L;
    },
    frame: function (stage, t) {
      var L = stage._l;
      var e = Math.pow(Math.sin(Math.PI * clamp(t, 0, 1)), 0.85);   // exposure hump, 0 at both ends
      var swap = span(t, 0.42, 0.58);

      function expose(l, vis) {
        l.inner.style.opacity = String(vis);
        l.inner.style.color = 'rgb(' + Math.round(lerp(231, 255, e)) + ','
          + Math.round(lerp(235, 255, e)) + ',' + Math.round(lerp(243, 255, e)) + ')';
        l.inner.style.textShadow = e < 0.02 ? 'none'
          : '0 0 ' + (6 * e).toFixed(1) + 'px rgba(255,255,255,' + (0.85 * e).toFixed(3) + '),'
          + ' 0 0 ' + (26 * e).toFixed(1) + 'px rgba(226,238,255,' + (0.55 * e).toFixed(3) + ')';
      }
      expose(L.out, 1 - swap);
      expose(L.in, swap);

      // halo copies: same words, blurred and screened on top — this is what makes it read as
      // light blowing out of the glyphs rather than the text simply turning white
      var b = lerp(1.5, 13, e);
      L.outG.clip.style.opacity = String((1 - swap) * e * 0.95);
      L.inG.clip.style.opacity = String(swap * e * 0.95);
      L.outG.inner.style.filter = L.inG.inner.style.filter = 'blur(' + b.toFixed(2) + 'px)';
      var s = 1 + 0.045 * e;
      L.outG.inner.style.transform = L.inG.inner.style.transform = 'scale(' + s.toFixed(4) + ')';
      L.out.inner.style.transform = 'scale(' + (1 + 0.018 * e).toFixed(4) + ')';
      L.in.inner.style.transform = 'scale(' + (1 - 0.016 * e).toFixed(4) + ')';

      L.veil.style.opacity = String(e);
      L.veil.style.background = 'radial-gradient(58% 92% at 50% 50%,'
        + ' rgba(255,255,255,.17) 0%, rgba(214,232,255,.07) 52%, rgba(0,0,0,0) 88%)';
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.inner.style.opacity = '0';
      L.in.inner.style.opacity = '1';
      L.in.inner.style.textShadow = 'none';
      L.in.inner.style.transform = 'none';
      L.outG.clip.style.opacity = '0'; L.inG.clip.style.opacity = '0';
      L.veil.style.opacity = '0';
    },
  });

  /* ============================================================== 5. long shadow ============
     The sun drops, the cast shadow rakes out sideways and lengthens — then it lets go. The
     detached shadow slides away and a second shadow rakes in from the other side already
     carrying the new words; it stands back up under the light and the new line settles into it.
     The old text is still on screen while the new shadow arrives, so there is never a frame
     with nothing legible in it — worst case you are reading a skewed silhouette. */
  FX.register({
    id: 'light-shadow',
    name: 'Long Shadow',
    family: 'Light',
    blurb: 'The cast shadow detaches, travels, and reforms carrying the new words.',
    duration: 1600,
    dwell: 4600,
    theme: { bg: '#F3F0E8', fg: '#1B1D1A',
             font: '"Avenir Next", Optima, "Segoe UI", system-ui, sans-serif' },
    setup: function (stage, ctx) {
      var w = wrapOf(stage);
      var L = {};
      L.glare = overlay(w, {});
      L.outS = layer(w, ctx.from); L.inS = layer(w, ctx.to);      // shadows, painted first
      L.out = layer(w, ctx.from); L.in = layer(w, ctx.to);
      [L.outS, L.inS].forEach(function (s) {
        s.inner.style.color = '#1B1D1A';
        s.inner.style.transformOrigin = '50% 86%';
      });
      stage._l = L;
    },
    frame: function (stage, t) {
      var L = stage._l;

      /* one cast shadow: k=skew in degrees, ln=length (vertical stretch), dx=% of width away
         from its own text, a=alpha, bl=blur px */
      function cast(l, k, ln, dx, a, bl) {
        l.inner.style.transform = 'translateX(' + dx.toFixed(2) + '%) skewX(' + k.toFixed(2)
          + 'deg) scaleY(' + ln.toFixed(3) + ') scaleX(' + (1 + 0.05 * Math.abs(k) / 60).toFixed(3) + ')';
        l.inner.style.opacity = String(a);
        l.inner.style.filter = bl > 0.05 ? 'blur(' + bl.toFixed(2) + 'px)' : 'none';
      }

      /* THE SUN STARTS AND ENDS OVERHEAD.
         This used to open with the shadow already cast — skew -13deg, two-thirds length, 20%
         alpha — and close the same way, with the arriving shadow parked under the new line for
         the last sixth of the move. Both resting slogans are drawn by the engine with no shadow
         at all, so the effect punched a full cast shadow onto the headline on its first frame and
         yanked it off again on its last: measured, the ink box jumped 241px at t=0.008.
         The metaphor already had the answer in it and was only half wired — `rise` was computed
         and never used. Now the sun really does start straight overhead (no shadow to see), drop
         through the middle of the move where all the raking happens, and climb back overhead on
         the far side. The shadow is BORN out of nothing and RETRACTS into nothing, so the two
         endpoint frames are the bare words. */
      var drop = E.inOutCubic(span(t, 0, 0.48));         // sun falling toward the horizon
      var noon = E.inOutCubic(span(t, 0.78, 1));         // ...and climbing back overhead after
      var detach = E.inCubic(span(t, 0.34, 0.62));       // the old shadow letting go and leaving
      var arrive = E.outCubic(span(t, 0.38, 0.82));      // the new shadow raking in and standing up

      cast(L.outS,
        lerp(0, -62, drop),                              // rakes further over as the sun drops
        lerp(0.04, 1.55, drop),                          // ...and lengthens out from underfoot
        lerp(0, 62, detach),
        lerp(0, 0.30, drop) * (1 - span(t, 0.46, 0.72)),
        lerp(0.4, 5.5, detach));

      cast(L.inS,
        lerp(64, -13, arrive) * (1 - noon),              // comes in raked the other way, stands up
        lerp(1.60, 0.66, arrive) * (1 - noon) + 0.04 * noon,
        lerp(-64, 0, arrive),
        Math.min(span(t, 0.34, 0.48), 1) * lerp(0.30, 0.20, arrive) * (1 - noon),
        lerp(5.5, 0.4, arrive));

      // the text itself: the old holds until the new shadow has arrived, then hands over
      var fadeOut = span(t, 0.46, 0.66), fadeIn = span(t, 0.58, 0.82);
      L.out.inner.style.opacity = String(1 - fadeOut);
      L.out.inner.style.transform = 'translateY(' + (-4 * fadeOut).toFixed(2) + 'px)';
      L.in.inner.style.opacity = String(fadeIn);
      L.in.inner.style.transform = 'translateY(' + (6 * (1 - E.outCubic(fadeIn))).toFixed(2) + 'px)';

      // the light source itself, swinging across as the shadow swings the other way.
      // edges() left this at 13% on the first frame — a white core bright enough to register as
      // ink in the top-right corner, where the canonical frame has nothing at all. The lamp is a
      // decoration, so it takes the dead-banded envelope: off entirely at both endpoints.
      var lx = lerp(88, 12, E.inOutCubic(t));
      L.glare.style.opacity = String(ornament(t, 0.03, 0.10));
      L.glare.style.background = 'radial-gradient(46% 130% at ' + lx.toFixed(1) + '% 8%,'
        + ' rgba(255,255,255,.90) 0%, rgba(255,250,232,.42) 34%, rgba(255,247,224,0) 74%)';
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.inner.style.opacity = '0';
      L.outS.inner.style.opacity = '0';
      L.in.inner.style.opacity = '1';
      L.in.inner.style.transform = 'none';
      // Settled means sun overhead, so no shadow — the same picture the engine's canonical layer
      // draws. Parking a 20% cast shadow here contradicted it and was the reason the last frame
      // of the transition did not match the first frame of the rest.
      L.inS.inner.style.opacity = '0';
      L.inS.inner.style.transform = 'translateX(0%) skewX(0deg) scaleY(0.04) scaleX(1)';
      L.inS.inner.style.filter = 'none';
      L.glare.style.opacity = '0';
    },
  });

  /* ================================================================= 6. scanner ============
     A beam runs down the line. The stage is cut into horizontal bands; a band flips from the old
     text to the new one at the instant the beam crosses it, so the change happens band by band
     rather than as a fade. Each band is a window onto a full copy of the line offset by its own
     top, so the text stays continuous across the stack; a band nudges sideways and glows as the
     beam passes, then settles. Both halves are always fully opaque — there is no frame in which
     part of the stage is empty, only frames where part of it has already changed. */
  FX.register({
    id: 'light-scanner',
    name: 'Scanner Beam',
    family: 'Light',
    blurb: 'A beam scans down the line and flips it to the new one band by band.',
    duration: 1400,
    dwell: 4400,
    theme: { bg: '#06080A', fg: '#DCF3E6',
             font: '"Avenir Next", "Segoe UI", Inter, system-ui, sans-serif' },
    setup: function (stage, ctx) {
      var w = wrapOf(stage);
      var H = Math.max(80, ctx.H || stage.clientHeight || 180);
      var n = clamp(Math.round(H / 16), 10, 26);
      function face(s, y0) {
        var f = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0', top: (-y0) + 'px', height: H + 'px',
          display: 'flex', alignItems: 'center', justifyContent: 'center', willChange: 'opacity',
        });
        var inner = FX.el('div', null, { width: '100%', textAlign: 'center', willChange: 'transform' });
        var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
        var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
        inner.appendChild(a); inner.appendChild(b);
        f.appendChild(inner);
        return { el: f, inner: inner };
      }
      var bands = [];
      for (var k = 0; k < n; k++) {
        var y0 = Math.round(k * H / n), y1 = Math.round((k + 1) * H / n);
        var band = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0',
          top: y0 + 'px', height: (y1 - y0) + 'px', overflow: 'hidden',
        });
        var fo = face(ctx.from, y0), fi = face(ctx.to, y0);
        band.appendChild(fo.el); band.appendChild(fi.el);
        w.appendChild(band);
        bands.push({ out: fo, in: fi, c: (y0 + y1) / 2, h: Math.max(1, y1 - y0) });
      }
      var halo = overlay(w, { mixBlendMode: 'screen' });
      var beam = FX.el('div', null, {
        position: 'absolute', left: '-2%', width: '104%', height: '2px', top: '0',
        background: 'linear-gradient(90deg, rgba(122,182,72,0), #CFF7A6 18%, #FFFFFF 50%,'
          + ' #CFF7A6 82%, rgba(122,182,72,0))',
        boxShadow: '0 0 14px rgba(160,240,120,.95), 0 0 44px rgba(122,182,72,.55)',
        pointerEvents: 'none', willChange: 'transform, opacity',
      });
      w.appendChild(beam);
      stage._l = { bands: bands, beam: beam, halo: halo, H: H };
    },
    frame: function (stage, t) {
      var L = stage._l, B = L.bands, H = L.H;
      var y = lerp(-0.10, 1.10, sweep(t)) * H;
      var vis = edges(t, 0.05, 0.05);

      for (var k = 0; k < B.length; k++) {
        var b = B[k];
        var d = (y - b.c) / (3.2 * b.h);
        var flash = Math.exp(-d * d) * vis;                      // gaussian around the beam
        var passed = y >= b.c;
        var cur = passed ? b.in : b.out;
        var other = passed ? b.out : b.in;
        other.el.style.opacity = '0';
        cur.el.style.opacity = '1';
        var dx = srnd(k, 2) * 5 * flash;
        cur.inner.style.transform = 'translateX(' + dx.toFixed(2) + 'px) scaleX('
          + (1 + 0.012 * flash).toFixed(4) + ')';
        cur.el.style.filter = flash > 0.02
          ? 'brightness(' + (1 + 0.9 * flash).toFixed(3) + ')' : 'none';
        cur.inner.style.textShadow = flash > 0.02
          ? '0 0 ' + (10 * flash).toFixed(1) + 'px rgba(190,255,160,' + (0.8 * flash).toFixed(3) + '),'
            + ' 0 0 ' + (30 * flash).toFixed(1) + 'px rgba(122,182,72,' + (0.5 * flash).toFixed(3) + ')'
          : 'none';
      }

      L.beam.style.transform = 'translateY(' + y.toFixed(2) + 'px)';
      L.beam.style.opacity = String(vis);
      L.halo.style.opacity = String(vis);
      L.halo.style.background = 'linear-gradient(180deg, rgba(122,182,72,0) '
        + ((y - 3.2 * B[0].h) / H * 100).toFixed(2) + '%, rgba(150,235,110,.16) '
        + (y / H * 100).toFixed(2) + '%, rgba(122,182,72,0) '
        + ((y + 3.2 * B[0].h) / H * 100).toFixed(2) + '%)';
    },
    rest: function (stage) {
      var L = stage._l;
      L.bands.forEach(function (b) {
        b.out.el.style.opacity = '0';
        b.in.el.style.opacity = '1';
        b.in.inner.style.transform = 'none';
        b.in.inner.style.textShadow = 'none';
        b.in.el.style.filter = 'none';
      });
      L.beam.style.opacity = '0';
      L.halo.style.opacity = '0';
    },
  });
})();
