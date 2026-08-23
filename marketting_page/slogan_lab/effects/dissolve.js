/* Dissolve family — six ways for one line to stop existing and another to arrive.
 *
 * House rules (see engine.js): every effect is a PURE FUNCTION OF PROGRESS. No timers, no CSS
 * animations, no wall clock. frame(stage, t) must paint the identical picture for the same
 * (i, t) every single time, frame(...,0) is the FROM line at rest, frame(...,1) the TO line.
 *
 *   dissolve-focus-pull   per-word depth of field; a rack focus through a crowd of words
 *   dissolve-luminance    bright glyphs burn out first, new ones rise out of the black
 *   dissolve-word-wash    word-by-word crossfade sweeping in reading order
 *   dissolve-rack         whole-line defocus that refocuses PAST sharp and settles back
 *   dissolve-develop      photographic print: grain, latent image, contrast comes up
 *   dissolve-chromatic    CMY channels separate, drift, and snap back into registration
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;
  var NS = 'http://www.w3.org/2000/svg';
  var UID = 0;

  /* ------------------------------------------------------------------ tiny helpers */

  /* Everything is authored against a 26px stage (the grid tile size) and scaled from there,
     so blur radii and offsets stay proportional in the big focus-mode tile too. */
  function unit(stage) {
    var f = parseFloat(getComputedStyle(stage).fontSize);
    return (f && isFinite(f) ? f : 26) / 26;
  }

  /* Deterministic pseudo-random from an index — never Math.random(), the lab must be seekable. */
  function hash(i, s) {
    var x = Math.sin(i * 127.1 + (s || 0) * 311.7 + 0.37) * 43758.5453;
    return x - Math.floor(x);
  }

  /* 0 -> 1 -> 0 over [0,1]; the shape of a flash, a bloom, an overshoot. */
  function hump(x) { return Math.sin(Math.PI * clamp(x, 0, 1)); }

  function mix(a, b, t) {
    return 'rgb(' + Math.round(lerp(a[0], b[0], t)) + ',' +
                    Math.round(lerp(a[1], b[1], t)) + ',' +
                    Math.round(lerp(a[2], b[2], t)) + ')';
  }

  function wrapEl(extra) {
    var css = {
      position: 'relative', width: '100%', height: '100%',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    };
    if (extra) for (var k in extra) css[k] = extra[k];
    return FX.el('div', 'fx-wrap', css);
  }

  /* Absolutely positioned so the two lines never influence each other's layout. Flex static
     position keeps each one vertically centred whether it wraps to two lines or four. */
  function lineEl() {
    return FX.el('div', 'fx-line', {
      position: 'absolute', left: '0', right: '0', textAlign: 'center',
      willChange: 'transform,opacity,filter',
    });
  }

  function gap(text) {
    var sp = FX.el('span', null, { whiteSpace: 'pre' });
    sp.textContent = text;
    return sp;
  }

  /* light line / bold line, as one flat text block.
     Two BLOCK spans, and deliberately no <br> between them: the engine forces a lone
     .fx-light/.fx-bold pair to display:block so the effect matches its canonical two-line layer,
     and a <br> sitting between two blocks opens an extra empty line box. That extra line is a
     whole line-height of drift against the canon — the headline jumps apart at t->0 and snaps
     back together at t->1. */
  function fillPlain(node, s) {
    var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
    var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
    node.appendChild(a); node.appendChild(b);
    return [];
  }

  /* Same two block halves, but each one's text handed to a splitter. The split pieces live inside
     an inline-block so the half can be measured and width-matched to the canon (see fitLine). */
  function fillSplit(node, s, split) {
    var out = [], fits = [];
    var defs = [['fx-light', s.light], ['fx-bold', s.bold]];
    for (var k = 0; k < 2; k++) {
      var half = FX.el('span', defs[k][0], { display: 'block' });
      var inner = FX.el('span', 'fx-fit', { display: 'inline-block' });
      var made = split(defs[k][1]);
      inner.appendChild(made.frag);
      half.appendChild(inner);
      node.appendChild(half);
      out = out.concat(made.nodes);
      fits.push({ half: half, inner: inner, text: defs[k][1] });
    }
    node._fits = fits;
    return out;
  }

  /* ...split into inline-block words (wrapping stays word-safe) */
  function fillWords(node, s) {
    return fillSplit(node, s, function (txt) { return FX.words(txt); });
  }

  /* ...split per character, but grouped inside inline-block words so nothing breaks mid-word */
  function fillChars(node, s) {
    return fillSplit(node, s, function (txt) { return FX.charWords(txt); });
  }

  /* Cutting a line into word or character boxes throws away the kerning between the pieces, so the
     split copy sets ~0.8% wider than the engine's canonical single-string line (~1.6% for a
     per-character split). That is a visible spread at t->0 and t->1 — the headline breathes out as
     the effect takes over and sucks back in when it hands back. Measure the unsplit string once at
     setup and scale the split copy back onto exactly that width.
     Pure geometry, measured once, so frame(t) stays a pure function of t. Skipped when the half
     wraps (one horizontal scale is the wrong correction then) or when the ratio is implausible
     (a font still loading). */
  function fitLine(line) {
    var fits = line._fits;
    if (!fits) return;
    for (var i = 0; i < fits.length; i++) {
      var f = fits[i];
      f.inner.style.transform = 'none';
      var avail = f.half.getBoundingClientRect().width;
      var ref = FX.el('span', null, {
        position: 'absolute', left: '-9999px', top: '0',
        visibility: 'hidden', whiteSpace: 'pre',
      });
      ref.textContent = f.text;
      f.half.appendChild(ref);
      var r = ref.getBoundingClientRect();
      f.half.removeChild(ref);
      var box = f.inner.getBoundingClientRect();
      if (r.width < 1 || box.width < 1) continue;          // nothing to measure
      if (r.width > avail + 0.5) continue;                  // canon wraps here too
      if (box.height > r.height * 1.6) continue;            // the split copy wrapped
      var k = r.width / box.width;
      if (k > 0.9 && k < 1.1) f.inner.style.transform = 'scaleX(' + k.toFixed(5) + ')';
    }
  }

  /* Two stacked layers, built by whichever filler is passed in. */
  function build(stage, ctx, filler) {
    stage.innerHTML = '';
    var wrap = wrapEl();
    var o = lineEl(), n = lineEl();
    var ow = filler(o, ctx.from), nw = filler(n, ctx.to);
    wrap.appendChild(o); wrap.appendChild(n);
    stage.appendChild(wrap);
    fitLine(o); fitLine(n);
    stage._u = unit(stage);
    return { wrap: wrap, out: o, in: n, outParts: ow, inParts: nw };
  }

  /* =================================================================== 1. Focus Pull
     A rack focus through a crowd: every word sits at its own depth, so the old line falls out
     of focus in a scattered wave (near words bloom big and soft first) while the new one comes
     up out of the bokeh in a different scatter. Blur, scale and brightness move together the
     way a real out-of-focus highlight does — it spreads AND it lifts. */
  FX.register({
    id: 'dissolve-focus-pull',
    name: 'Focus Pull',
    family: 'Dissolve',
    blurb: 'Words sit at different depths; the lens racks from the old line to the new.',
    duration: 1300,
    setup: function (stage, ctx) { stage._L = build(stage, ctx, fillWords); },
    frame: function (stage, t) {
      var L = stage._L, u = stage._u, i, w, d, lead, p, f, q, soft;

      for (i = 0; i < L.outParts.length; i++) {
        w = L.outParts[i];
        d = hash(i, 1);                                   // this word's depth
        lead = 0.22 * d;
        p = E.inQuad(span(t, lead, lead + 0.38));          // defocus
        f = E.outCubic(span(t, lead + 0.10, lead + 0.44)); // fade trails the blur
        w.style.filter = 'blur(' + (8 * u * p).toFixed(2) + 'px) brightness(' + (1 + 0.28 * p).toFixed(3) + ')';
        w.style.opacity = (1 - f).toFixed(3);
        w.style.transform = 'translateY(' + (-4 * u * p).toFixed(2) + 'px) scale(' + (1 + 0.10 * p).toFixed(4) + ')';
      }

      /* The incoming words sit at their own depths too, but the band is kept tight enough that no
         word is still absent once the old line has gone: the deepest word starts at 0.40 and is
         already carrying opacity while the shallowest is settling. A wider spread looked lovely on
         paper and gave a mid-transition frame where half the words simply were not there yet. */
      for (i = 0; i < L.inParts.length; i++) {
        w = L.inParts[i];
        d = hash(i, 2);
        lead = 0.18 + 0.22 * d;
        q = E.outCubic(span(t, lead, lead + 0.44));
        soft = 1 - q;
        w.style.filter = 'blur(' + (8 * u * soft).toFixed(2) + 'px) brightness(' + (1 + 0.32 * soft).toFixed(3) + ')';
        w.style.opacity = E.outQuad(span(t, lead, lead + 0.28)).toFixed(3);
        w.style.transform = 'translateY(' + (6 * u * soft).toFixed(2) + 'px) scale(' + (1 - 0.09 * soft).toFixed(4) + ')';
      }
    },
    rest: function (stage) {
      var L = stage._L, i;
      for (i = 0; i < L.outParts.length; i++) { L.outParts[i].style.opacity = '0'; }
      for (i = 0; i < L.inParts.length; i++) {
        var w = L.inParts[i];
        w.style.opacity = '1'; w.style.filter = 'none'; w.style.transform = 'none';
      }
    },
  });

  /* =================================================================== 2. Luminance Dissolve
     A film dissolve keyed on brightness: the hottest glyphs blow out to white and leave first,
     the shadows hold on longest. The incoming line does the reverse — it comes up out of the
     black, darkest first, each character flashing once as it reaches full value.
     Themed dark, because a luminance dissolve is only legible against black. */
  var LU_FG = [244, 245, 247], LU_WHITE = [255, 255, 255], LU_DARK = [92, 96, 106];
  FX.register({
    id: 'dissolve-luminance',
    name: 'Luminance Burn',
    family: 'Dissolve',
    blurb: 'Bright glyphs blow out and go first; the new line rises out of the shadows.',
    duration: 1500,
    dwell: 4200,
    theme: { bg: '#0A0A0C', fg: '#F4F5F7', accent: '#7AB648' },
    setup: function (stage, ctx) { stage._L = build(stage, ctx, fillChars); },
    /* Two clocks per glyph, and the split is what keeps this legible.
     *
     * The BURN clock is wide open — 0.30 of spread — and it drives everything you actually read as
     * luminance keying: which glyph blows out to white first, flares, softens, swells. The
     * PRESENCE clock (plain opacity) is deliberately narrow, 0.10 of spread, so each line comes and
     * goes as a body.
     *
     * Both on the wide clock is what this effect used to do, and it meant that around the middle
     * each line was missing a random half of its characters at the same instant: two half-eaten
     * strings on top of each other, which reads as nothing at all. Now the mid frame is an ordinary
     * crossfade of two whole lines — you can read both — while the flashing still runs glyph by
     * glyph across them. */
    frame: function (stage, t) {
      var L = stage._L, u = stage._u, i, c, lum, lead, go, p, g, q, qe, fl;

      for (i = 0; i < L.outParts.length; i++) {
        c = L.outParts[i];
        lum = hash(i, 3);
        lead = 0.30 * (1 - lum);                       // burn: the bright ones are already leaving
        go = 0.18 + 0.06 * (1 - lum);                  // presence: the line leaves as a body
        q = span(t, go, go + 0.26);
        p = span(t, lead, lead + 0.32);
        g = hump(p);
        c.style.color = mix(LU_FG, LU_WHITE, E.outQuad(clamp(p * 1.6, 0, 1)));
        c.style.textShadow = p <= 0 ? 'none'
          : '0 0 ' + (17 * u * g).toFixed(1) + 'px rgba(255,255,255,' + (0.80 * g).toFixed(3) + '), ' +
            '0 0 ' + (42 * u * g).toFixed(1) + 'px rgba(150,215,110,' + (0.28 * g).toFixed(3) + ')';
        c.style.opacity = (1 - E.inQuad(q)).toFixed(3);
        // blur rides the PRESENCE clock, not the burn clock: a glyph that is still at full opacity
        // but already 6px soft is a hole in the line, which is what made the middle unreadable.
        c.style.filter = 'blur(' + (2.2 * u * E.inQuad(q)).toFixed(2) + 'px)';
        c.style.transform = 'scale(' + (1 + 0.07 * E.outQuad(p)).toFixed(4) + ')';
      }

      for (i = 0; i < L.inParts.length; i++) {
        c = L.inParts[i];
        lum = hash(i, 4);
        lead = 0.30 + 0.32 * lum;                      // flash: the shadows come up first
        go = 0.26 + 0.06 * lum;
        q = span(t, go, go + 0.30);
        qe = E.outCubic(q);
        fl = hump(span(t, lead, lead + 0.30));
        c.style.color = mix(LU_DARK, LU_FG, E.outQuad(clamp(q * 1.5, 0, 1)));
        c.style.opacity = qe.toFixed(3);
        c.style.textShadow = fl <= 0 ? 'none'
          : '0 0 ' + (14 * u * fl).toFixed(1) + 'px rgba(186,240,140,' + (0.50 * fl).toFixed(3) + ')';
        c.style.filter = 'blur(' + (3.2 * u * (1 - qe)).toFixed(2) + 'px)';
        c.style.transform = 'scale(' + lerp(1.06, 1, qe).toFixed(4) + ')';
      }
    },
    rest: function (stage) {
      var L = stage._L, i, c;
      for (i = 0; i < L.outParts.length; i++) L.outParts[i].style.opacity = '0';
      for (i = 0; i < L.inParts.length; i++) {
        c = L.inParts[i];
        c.style.opacity = '1'; c.style.color = ''; c.style.textShadow = 'none';
        c.style.filter = 'none'; c.style.transform = 'none';
      }
    },
  });

  /* =================================================================== 3. Word Wash
     A crossfade that respects reading order: each word is swapped in place, left to right, the
     old one sinking and softening a beat before the new one washes up under it. The incoming
     word arrives slightly wide (scaleX) and contracts as it sharpens, like a wet edge drying. */
  FX.register({
    id: 'dissolve-word-wash',
    name: 'Word Wash',
    family: 'Dissolve',
    blurb: 'Word by word, left to right: the old sinks away as the new washes up.',
    duration: 1150,
    setup: function (stage, ctx) { stage._L = build(stage, ctx, fillWords); },
    frame: function (stage, t) {
      var L = stage._L, u = stage._u, i, w, n, lead, p, q;

      n = Math.max(1, L.outParts.length);
      for (i = 0; i < L.outParts.length; i++) {
        w = L.outParts[i];
        lead = (i / n) * 0.42;
        p = E.inOutQuad(span(t, lead, lead + 0.40));
        w.style.opacity = (1 - p).toFixed(3);
        w.style.filter = 'blur(' + (4.5 * u * p).toFixed(2) + 'px)';
        w.style.transform = 'translateY(' + (-7 * u * p).toFixed(2) + 'px) scale(' + (1 - 0.05 * p).toFixed(4) + ')';
      }

      n = Math.max(1, L.inParts.length);
      for (i = 0; i < L.inParts.length; i++) {
        w = L.inParts[i];
        lead = 0.18 + (i / n) * 0.42;
        q = E.outQuint(span(t, lead, lead + 0.40));
        w.style.opacity = E.outQuad(span(t, lead, lead + 0.26)).toFixed(3);
        w.style.filter = 'blur(' + (5.5 * u * (1 - q)).toFixed(2) + 'px)';
        w.style.transform =
          'translateY(' + (11 * u * (1 - q)).toFixed(2) + 'px) ' +
          'scale(' + (1 + 0.055 * (1 - q)).toFixed(4) + ', ' + (1 - 0.02 * (1 - q)).toFixed(4) + ')';
      }
    },
    rest: function (stage) {
      var L = stage._L, i;
      for (i = 0; i < L.outParts.length; i++) L.outParts[i].style.opacity = '0';
      for (i = 0; i < L.inParts.length; i++) {
        var w = L.inParts[i];
        w.style.opacity = '1'; w.style.filter = 'none'; w.style.transform = 'none';
      }
    },
  });

  /* =================================================================== 4. Rack Overshoot
     One lens, one line, one continuous pull. The old line goes soft and washes out; the new one
     comes up out of the haze and — this is the point — travels PAST correct focus into an
     over-sharp, over-contrasty frame before easing back. The whole stage breathes a little,
     the way a real lens changes magnification as it focuses. */
  FX.register({
    id: 'dissolve-rack',
    name: 'Rack Overshoot',
    family: 'Dissolve',
    blurb: 'A single lens pull that refocuses past sharp, then relaxes into it.',
    duration: 1200,
    setup: function (stage, ctx) { stage._L = build(stage, ctx, fillPlain); },
    frame: function (stage, t) {
      var L = stage._L, u = stage._u;

      /* The old line holds its density while it goes soft — a defocused line is still a line, and
         fading it out at the same time as blurring it left the middle of the pull with almost no
         ink on the stage at all. It only starts to leave once the new one is already coming up. */
      var a = E.inOutQuad(span(t, 0, 0.52));
      L.out.style.filter =
        'blur(' + (7 * u * a).toFixed(2) + 'px) contrast(' + (1 - 0.42 * a).toFixed(3) + ') ' +
        'brightness(' + (1 + 0.18 * a).toFixed(3) + ')';
      L.out.style.opacity = (1 - E.inQuad(span(t, 0.22, 0.66))).toFixed(3);
      L.out.style.transform = 'scale(' + (1 + 0.055 * a).toFixed(4) + ')';

      var b = E.outQuart(span(t, 0.20, 0.86));
      var s = hump(span(t, 0.66, 0.94));                // the overshoot past sharp
      L.in.style.filter =
        'blur(' + (9 * u * (1 - b)).toFixed(2) + 'px) ' +
        'contrast(' + (lerp(0.72, 1, b) + 0.52 * s).toFixed(3) + ') ' +
        'brightness(' + (lerp(1.16, 1, b) - 0.05 * s).toFixed(3) + ')';
      L.in.style.opacity = E.outQuad(span(t, 0.20, 0.64)).toFixed(3);
      L.in.style.textShadow = s <= 0 ? 'none' : '0 0 ' + (0.85 * u * s).toFixed(2) + 'px currentColor';
      L.in.style.transform = 'scale(' + (lerp(0.955, 1, b) + 0.014 * s).toFixed(4) + ')';

      L.wrap.style.transform = 'scale(' + (1 + 0.020 * hump(t)).toFixed(4) + ')';
    },
    rest: function (stage) {
      var L = stage._L;
      L.wrap.style.transform = 'none';
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.filter = 'none';
      L.in.style.textShadow = 'none';
      L.in.style.transform = 'none';
    },
  });

  /* =================================================================== 5. Darkroom
     A print coming up in the developer. The old line breaks apart into grain (an SVG turbulence
     displacement whose amplitude is driven straight off t); the new one arrives as a flat,
     low-contrast latent image buried in that same grain, and contrast, density and detail come
     up together while the grain veil settles back out of sight. */
  function dispFilter(id, seed) {
    var svg = document.createElementNS(NS, 'svg');
    svg.setAttribute('width', '0'); svg.setAttribute('height', '0');
    svg.setAttribute('aria-hidden', 'true');
    svg.style.position = 'absolute'; svg.style.width = '0'; svg.style.height = '0';
    svg.style.overflow = 'hidden';
    var f = document.createElementNS(NS, 'filter');
    f.setAttribute('id', id);
    f.setAttribute('x', '-30%'); f.setAttribute('y', '-40%');
    f.setAttribute('width', '160%'); f.setAttribute('height', '180%');
    f.setAttribute('color-interpolation-filters', 'sRGB');
    var turb = document.createElementNS(NS, 'feTurbulence');
    turb.setAttribute('type', 'fractalNoise');
    turb.setAttribute('baseFrequency', '0.62 0.82');
    turb.setAttribute('numOctaves', '3');
    turb.setAttribute('seed', String(seed));
    turb.setAttribute('result', 'grain');
    var disp = document.createElementNS(NS, 'feDisplacementMap');
    disp.setAttribute('in', 'SourceGraphic');
    disp.setAttribute('in2', 'grain');
    disp.setAttribute('scale', '0');
    disp.setAttribute('xChannelSelector', 'R');
    disp.setAttribute('yChannelSelector', 'G');
    f.appendChild(turb); f.appendChild(disp); svg.appendChild(f);
    return { svg: svg, disp: disp };
  }

  var GRAIN_URI = "url(\"data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='140' height='140'%3E%3Cfilter id='g'%3E%3CfeTurbulence type='fractalNoise' baseFrequency='0.85' numOctaves='3' stitchTiles='stitch'/%3E%3C/filter%3E%3Crect width='140' height='140' filter='url(%23g)'/%3E%3C/svg%3E\")";

  FX.register({
    id: 'dissolve-develop',
    name: 'Darkroom',
    family: 'Dissolve',
    blurb: 'The old line breaks into grain; the new one develops up out of it.',
    duration: 1600,
    dwell: 4600,
    theme: { bg: '#F5F1E8', fg: '#221E1A', accent: '#7AB648' },
    setup: function (stage, ctx) {
      var L = build(stage, ctx, fillPlain);
      var ids = ['fxdev' + (++UID), 'fxdev' + (++UID)];
      var fo = dispFilter(ids[0], 4), fi = dispFilter(ids[1], 11);
      stage.appendChild(fo.svg); stage.appendChild(fi.svg);
      L.dispOut = fo.disp; L.dispIn = fi.disp;
      L.urlOut = 'url(#' + ids[0] + ')'; L.urlIn = 'url(#' + ids[1] + ')';
      L.veil = FX.el('div', null, {
        position: 'absolute', inset: '-10%', pointerEvents: 'none',
        backgroundImage: GRAIN_URI, backgroundRepeat: 'repeat',
        mixBlendMode: 'multiply', opacity: '0', willChange: 'opacity',
      });
      L.wrap.appendChild(L.veil);
      stage._L = L;
    },
    frame: function (stage, t) {
      var L = stage._L, u = stage._u;

      /* outgoing: dries out, breaks into grain, gone */
      var a = E.inQuad(span(t, 0, 0.58));
      L.dispOut.setAttribute('scale', (17 * u * a).toFixed(2));
      L.out.style.filter = L.urlOut + ' blur(' + (1.8 * u * a).toFixed(2) + 'px) ' +
        'contrast(' + (1 - 0.55 * a).toFixed(3) + ') brightness(' + (1 + 0.16 * a).toFixed(3) + ')';
      L.out.style.opacity = (1 - E.inOutQuad(span(t, 0.05, 0.62))).toFixed(3);

      /* incoming: latent image first — flat, pale, still full of grain — then density comes up */
      var b = E.outQuart(span(t, 0.26, 0.94));
      var soft = 1 - b;
      L.dispIn.setAttribute('scale', (19 * u * soft).toFixed(2));
      L.in.style.filter = L.urlIn + ' blur(' + (1.4 * u * soft).toFixed(2) + 'px) ' +
        'contrast(' + lerp(0.28, 1, E.outCubic(span(t, 0.40, 1))).toFixed(3) + ') ' +
        'brightness(' + lerp(1.30, 1, E.outCubic(span(t, 0.36, 1))).toFixed(3) + ')';
      L.in.style.opacity = E.outQuad(span(t, 0.26, 0.66)).toFixed(3);

      /* the bath itself: grain strongest right where the image is weakest */
      L.veil.style.opacity = (0.62 * hump(span(t, 0.02, 0.98))).toFixed(3);
    },
    rest: function (stage) {
      var L = stage._L;
      L.out.style.opacity = '0';
      L.dispIn.setAttribute('scale', '0');
      L.in.style.opacity = '1';
      L.in.style.filter = 'none';
      L.veil.style.opacity = '0';
    },
  });

  /* =================================================================== 6. Chromatic
     Cyan, magenta and yellow plates, multiplied together. In register they read as clean black
     text; out of register you get the full print-shop misregistration — colour fringes ghosting
     apart. The old line's plates fly apart and fade; the new line's converge from the opposite
     directions, cross very slightly past zero, and settle. */
  var CH_INK = [17, 17, 20];                          // the page's own ink, = theme.fg
  var CH_INK_CSS = 'rgb(17,17,20)';
  var CH_PAPER = [255, 255, 255];                     // multiply's identity: a blank plate
  var PLATES = [
    { rgb: [0, 229, 255], out: [-1.00, -0.34], in: [0.82, 0.62], sc: 0.994 },
    { rgb: [255, 0, 212], out: [0.92, -0.52], in: [-0.90, 0.44], sc: 1.000 },
    { rgb: [255, 232, 0], out: [0.10, 1.00], in: [0.04, -1.05], sc: 1.008 },
  ];

  FX.register({
    id: 'dissolve-chromatic',
    name: 'Chromatic Registration',
    family: 'Dissolve',
    blurb: 'CMY plates drift out of register, swap the line, and snap back to black.',
    duration: 1250,
    theme: { bg: '#FFFFFF', fg: '#111114' },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      /* multiply's identity is white, screen's is black. The two-beat inking below leans on
         that identity to keep a plate invisible until it is wanted, so both the blend mode and
         the identity colour have to flip together with the ground. */
      var dark = ctx.sc.dark;
      var C = stage._chr = {
        blend: dark ? 'screen' : 'multiply',
        ident: dark ? [0, 0, 0] : CH_PAPER,      // contributes nothing under this blend
        ink: dark ? [245, 245, 247] : CH_INK,    // a single neutral impression
      };
      C.inkCss = 'rgb(' + C.ink.join(',') + ')';
      C.identCss = 'rgb(' + C.ident.join(',') + ')';
      var wrap = wrapEl({ background: ctx.sc.paper, isolation: 'isolate' });

      function plates(s) {
        var list = [];
        for (var p = 0; p < PLATES.length; p++) {
          var d = lineEl();
          d.style.color = p < 2 ? C.identCss : C.inkCss;
          d.style.mixBlendMode = C.blend;
          fillPlain(d, s);
          wrap.appendChild(d);
          list.push(d);
        }
        return list;
      }
      var o = plates(ctx.from), n = plates(ctx.to);
      stage.appendChild(wrap);
      stage._u = unit(stage);
      stage._L = { wrap: wrap, out: o, in: n };
    },
    frame: function (stage, t) {
      var L = stage._L, u = stage._u, C = stage._chr, p, P, d, sep, sc, fade;

      /* outgoing: plates slip apart, each fading as it goes.
         A line in perfect register is ONE impression, not three. Three independently anti-aliased
         glyphs never multiply back to a neutral edge, so holding all three plates inked while they
         sit on top of each other gave the settled line a coloured rim (and, with neutral ink, a
         visibly heavier one) that the canonical line does not have. So the press is inked in two
         beats: the two colour plates lift off blank paper first — white is multiply's identity, so
         they contribute nothing until they do — and only then does the key plate turn from the
         page's ink to yellow. The composite is black through both beats; the reverse pair of beats
         (r1, r2 below) puts the new line back down to a single ink impression. */
      var a = E.inQuad(span(t, 0, 0.62));
      var aFade = E.inOutQuad(span(t, 0.10, 0.60));
      var s1 = span(t, 0.005, 0.06), s2 = span(t, 0.05, 0.12);
      for (p = 0; p < 3; p++) {
        P = PLATES[p]; d = L.out[p];
        sep = 14 * u * a;
        sc = lerp(1, P.sc, a * 2.2);
        d.style.color = p < 2 ? mix(C.ident, P.rgb, s1) : mix(C.ink, P.rgb, s2);
        d.style.transform =
          'translate(' + (P.out[0] * sep).toFixed(2) + 'px,' + (P.out[1] * sep).toFixed(2) + 'px) ' +
          'scale(' + sc.toFixed(4) + ')';
        d.style.filter = 'blur(' + (2.6 * u * a).toFixed(2) + 'px)';
        d.style.opacity = (1 - aFade).toFixed(3);
      }

      /* incoming: converge from the other side, cross a hair past register, settle */
      /* The wobble has to be OVER before the engine takes the frame back: any separation left at
         t=0.98 is a colour fringe that vanishes on the swap to canon. It lands by 0.94. */
      var b = E.outExpo(span(t, 0.28, 0.90));
      var over = hump(span(t, 0.66, 0.94)) * 0.09;      // the little cross-over wobble
      fade = E.outQuad(span(t, 0.26, 0.62));
      var r1 = span(t, 0.80, 0.89), r2 = span(t, 0.88, 0.97);   // key plate first, then the colours
      for (p = 0; p < 3; p++) {
        P = PLATES[p]; d = L.in[p];
        sep = 17 * u * ((1 - b) - over);
        sc = lerp(P.sc, 1, b);
        d.style.color = p < 2 ? mix(P.rgb, C.ident, r2) : mix(P.rgb, C.ink, r1);
        d.style.transform =
          'translate(' + (P.in[0] * sep).toFixed(2) + 'px,' + (P.in[1] * sep).toFixed(2) + 'px) ' +
          'scale(' + sc.toFixed(4) + ')';
        d.style.filter = 'blur(' + (2.8 * u * Math.abs(1 - b)).toFixed(2) + 'px)';
        d.style.opacity = fade.toFixed(3);
      }
    },
    rest: function (stage) {
      var L = stage._L, p;
      for (p = 0; p < 3; p++) {
        L.out[p].style.opacity = '0';
        L.in[p].style.opacity = '1';
        L.in[p].style.transform = 'none';
        L.in[p].style.filter = 'none';
      }
    },
  });
})();
