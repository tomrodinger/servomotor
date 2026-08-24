/* Liquid — six fluid / elastic / distortion transitions.
 *
 * Every effect here is a PURE FUNCTION OF PROGRESS: no timers, no CSS animations, no wall clock.
 * Randomness is hashed from the character index, so (i, t) always renders the identical picture.
 *
 *   liquid-goo-merge     letters collapse into one ink blob, the new line pulls back out of it
 *   liquid-ripple-wave   a water lens travels across the block, swapping glyphs under the crest
 *   liquid-stretch-snap  the old line is pulled like taffy until the strand necks and snaps
 *   liquid-mercury       glyphs ball up into beads that fuse, roll, and re-form as the new line
 *   liquid-drip          letters sag, string out, and fall; the new ones grow up from the baseline
 *   liquid-shear-wobble  a jelly shear runs through the words and rings down to rest
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;

  /* ------------------------------------------------------------------ tiny utils */

  /* deterministic pseudo-random in [0,1) from an integer index (+ optional salt) */
  function hash(k, s) {
    var x = Math.sin(k * 12.9898 + (s || 0) * 78.233) * 43758.5453;
    return x - Math.floor(x);
  }
  function gauss(x) { return Math.exp(-x * x); }
  /* exponential decay that is exactly 1 at u=0 and exactly 0 at u=1 — clean settle, no residue */
  function decay(u, k) {
    var e = Math.exp(-k);
    return (Math.exp(-k * u) - e) / (1 - e);
  }
  function n3(v) { return (Math.round(v * 1000) / 1000); }

  /* ------------------------------------------------------------------ builders */

  /* Flex-centred wrap. Absolutely positioned lines inside it are centred by the flex container's
     static position, exactly like the reference twoLayer(), so two lines can overlap perfectly. */
  function centerWrap(stage) {
    stage.innerHTML = '';
    var wrap = FX.el('div', 'fx-wrap', {
      position: 'relative', width: '100%', height: '100%',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    });
    stage.appendChild(wrap);
    return wrap;
  }

  /* Same, but the whole thing sits inside an over-sized opaque box that carries the goo filter
     chain (blur + contrast). The box is inset -60px so its own blurred edge is clipped away by
     the tile and never shows up as a soft rectangle. */
  function gooWrap(stage, bg) {
    stage.innerHTML = '';
    var box = FX.el('div', 'fx-goo', {
      position: 'absolute', left: '-60px', right: '-60px', top: '-60px', bottom: '-60px',
      background: bg, willChange: 'filter',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    });
    var wrap = FX.el('div', 'fx-wrap', {
      position: 'relative', width: 'calc(100% - 120px)', height: '100%',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    });
    box.appendChild(wrap);
    stage.appendChild(box);
    return { box: box, wrap: wrap };
  }

  /* The pre-image of a colour under contrast(c).
   *
   * contrast maps every channel v -> (v - 0.5) * c + 0.5, which is fine for the ink and wrong for
   * the GROUND: a near-black stage at 0.04 is dragged to pure black the moment the contrast rises,
   * so the whole panel visibly darkens for the length of the transition and snaps back at the end.
   * Pre-distorting the ground by the inverse means the filter lands it exactly back on the stage
   * colour, and the only thing the contrast is doing is what it is there for — thresholding the
   * blurred ink into beads. */
  function preContrast(rgb, c) {
    var m = /rgba?\(([^)]+)\)/.exec(rgb || '');
    if (!m || !(c > 0)) return rgb;
    var p = m[1].split(',');
    function inv(v) {
      var x = ((parseFloat(v) / 255 - 0.5) / c + 0.5) * 255;
      return Math.max(0, Math.min(255, Math.round(x)));
    }
    return 'rgb(' + inv(p[0]) + ',' + inv(p[1]) + ',' + inv(p[2]) + ')';
  }

  function absLine(wrap) {
    var d = FX.el('div', 'fx-line', {
      position: 'absolute', left: '0', right: '0', textAlign: 'center',
      willChange: 'transform,opacity,filter',
    });
    wrap.appendChild(d);
    return d;
  }

  /* One slogan as two rows (light over bold), split per character but grouped by word so a line
     can never break mid-word. Returns the flat character list in reading order. */
  function charLayer(wrap, s) {
    var d = absLine(wrap), nodes = [];
    [['fx-light', s.light], ['fx-bold', s.bold]].forEach(function (r) {
      var row = FX.el('div', r[0], { display: 'block' });
      var cw = FX.charWords(r[1]);
      row.appendChild(cw.frag);
      cw.nodes.forEach(function (n) { n.style.willChange = 'transform,opacity'; n._dx = 0; n._dy = 0; nodes.push(n); });
      d.appendChild(row);
    });
    return { el: d, nodes: nodes };
  }

  /* One slogan as two rows split per word. */
  function wordLayer(wrap, s) {
    var d = absLine(wrap), nodes = [];
    [['fx-light', s.light], ['fx-bold', s.bold]].forEach(function (r) {
      var row = FX.el('div', r[0], { display: 'block' });
      var ws = FX.words(r[1]);
      row.appendChild(ws.frag);
      ws.nodes.forEach(function (n) {
        n.style.willChange = 'transform,opacity,filter';
        n.style.transformOrigin = '50% 100%';
        n._dx = 0; n._dy = 0;
        nodes.push(n);
      });
      d.appendChild(row);
    });
    return { el: d, nodes: nodes };
  }

  /* A->B morph layer: light pairs with light, bold with bold, word by word (alignWords), so every
     cell holds both glyphs and every word stays an unbreakable inline-block. */
  function pairLayer(wrap, ctx, mk) {
    var d = absLine(wrap), cells = [];
    [['fx-light', ctx.from.light, ctx.to.light],
     ['fx-bold', ctx.from.bold, ctx.to.bold]].forEach(function (sp) {
      var row = FX.el('div', sp[0], { display: 'block' });
      var groups = FX.alignWords(sp[1], sp[2]);
      /* alignWords pads the WORD LIST as well as the letters, so the shorter side ends in empty
         groups. Their leading spaces are real width, and text-align:center then pushes that whole
         row off the canonical centre — 45 px on "Servomotor:" -> "No driver board. No encoder to
         mount.". So each gap carries the space it should show on each side, and travels with the
         first cell of the word that follows it. */
      var na = sp[1].split(/\s+/).filter(Boolean).length;
      var nb = sp[2].split(/\s+/).filter(Boolean).length;
      var pending = null;
      groups.forEach(function (g, gi) {
        var w = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
        var first = true;
        g.pairs.forEach(function (pr) {
          var c = FX.el('span', null, {
            display: 'inline-block', whiteSpace: 'pre', position: 'relative',
            willChange: 'transform,opacity',
          });
          c._a = pr[0]; c._b = pr[1]; c._dx = 0; c._dy = 0;
          c._blank = (!(pr[0] || '').trim() && !(pr[1] || '').trim());
          if (first) { c._gap = pending; first = false; }
          if (mk) mk(c, pr[0], pr[1]); else c.textContent = pr[0];
          w.appendChild(c);
          cells.push(c);
        });
        row.appendChild(w);
        if (gi < groups.length - 1) {
          var gap = FX.el('span', null, { whiteSpace: 'pre' });
          gap._a = (gi + 1 < na) ? ' ' : '';
          gap._b = (gi + 1 < nb) ? ' ' : '';
          gap.textContent = gap._a;
          row.appendChild(gap);
          pending = gap;
        }
      });
      d.appendChild(row);
    });
    return { el: d, cells: cells };
  }

  /* Show side A (useB false) or side B (true) in one pair cell, taking the space in front of its
     word with it. Text goes into c._txt when the cell has an inner glyph span (so an absolutely
     positioned sibling such as a bead survives the swap), otherwise into the cell itself. */
  function sideOf(c, useB) {
    var host = c._txt || c;
    var s = (useB ? c._b : c._a) || '';
    if (host.textContent !== s) host.textContent = s;
    var g = c._gap;
    if (g) {
      var gs = useB ? g._b : g._a;
      if (g.textContent !== gs) g.textContent = gs;
    }
  }

  /* Cache each node's offset from its layer centre. Transforms are cleared first so the read is of
     the RESTING layout; if the tile is hidden (zero-size) we simply retry on the next frame. */
  function measure(layer) {
    var host = layer.el, nodes = layer.nodes || layer.cells, i;
    if (!host.getBoundingClientRect().width) return false;
    host.style.transform = 'none';
    for (i = 0; i < nodes.length; i++) nodes[i].style.transform = 'none';
    var hr = host.getBoundingClientRect();
    if (!hr.width) return false;
    var cx = hr.left + hr.width / 2, cy = hr.top + hr.height / 2;
    var mx = 1, my = 1;
    for (i = 0; i < nodes.length; i++) {
      var r = nodes[i].getBoundingClientRect();
      var dx = r.left + r.width / 2 - cx, dy = r.top + r.height / 2 - cy;
      nodes[i]._dx = dx; nodes[i]._dy = dy;
      if (Math.abs(dx) > mx) mx = Math.abs(dx);
      if (Math.abs(dy) > my) my = Math.abs(dy);
    }
    layer.mx = mx; layer.my = my; layer.w = hr.width; layer.h = hr.height;
    return true;
  }
  function measured(stage) {
    if (stage._ok) return true;
    var ls = stage._layers, ok = true;
    for (var i = 0; i < ls.length; i++) if (!measure(ls[i])) ok = false;
    stage._ok = ok;
    return ok;
  }

  /* =========================================================== 1. Gooey merge ============== */
  FX.register({
    id: 'liquid-goo-merge',
    name: 'Goo Merge',
    family: 'Liquid',
    blurb: 'The old line pulls into a single blob of ink; the new one pulls back out of it.',
    duration: 1300,
    dwell: 4200,
    theme: {
      bg: '#FFFFFF', fg: '#0A0A0B', accent: '#7AB648',
      font: '"Helvetica Neue", Inter, Helvetica, Arial, sans-serif',
    },
    setup: function (stage, ctx) {
      var g = gooWrap(stage, ctx.sc.paper);
      stage._box = g.box;
      stage._paper = ctx.sc.paper;
      stage._out = charLayer(g.wrap, ctx.from);
      stage._in = charLayer(g.wrap, ctx.to);
      stage._layers = [stage._out, stage._in];
      stage._ok = false;
    },
    frame: function (stage, t) {
      measured(stage);
      var O = stage._out, I = stage._in, i, n;

      /* Surface tension. The window is narrow on purpose, and now narrower still: the goo chain is
         a threshold, and while it is at full strength there is no readable ink anywhere on stage.
         The peak sits at t=0.42 — the instant the blob exists — and is nearly spent by t=0.55,
         by which time the new line is already pulling out of it and legible again. */
      var g = Math.exp(-Math.pow((t - 0.42) / 0.115, 2));

      /* The filter is applied for the WHOLE transition, at identity values when g is small,
         rather than being switched between 'none' and a value at a threshold.
         Toggling `filter` on and off is not a no-op even when the value is nearly identity: it
         creates and destroys a stacking context, so the box is re-rasterised and the type shifts
         by a fraction of a pixel. Measured as a single frame-to-frame step of 0.036 at t=0.157
         against a background of 0.005 — a 7x pop in the middle of an otherwise smooth melt, and
         the only real discontinuity found in the whole set. Holding the filter on costs a
         blur(0px) that never renders differently, and the pop is gone.
         The effect layer is hidden at t<=0 and t>=1, so this never touches the canonical frames. */
      var gc = 1 + 13 * g;
      stage._box.style.filter = 'blur(' + n3(6.5 * g) + 'px) contrast(' + n3(gc) + ')';
      stage._box.style.background = preContrast(stage._paper, gc);   // hold the ground still

      /* Ink swells before it melts. Without this the 300-weight row is too thin to survive the
         contrast threshold and vanishes instead of going gooey. */
      var sw = n3(6 * Math.pow(g, 0.75));
      var shadow = g < 0.005 ? 'none'
        : '0 0 ' + sw + 'px currentColor, 0 0 ' + sw + 'px currentColor';

      /* the blob flattens under its own weight */
      var squash = 'scale(' + n3(1 + 0.05 * g) + ',' + n3(1 - 0.10 * g) + ')';
      O.el.style.transform = squash;
      I.el.style.transform = squash;
      O.el.style.textShadow = shadow;
      I.el.style.textShadow = shadow;

      /* The blob is a real beat, but a SHORT one, and it stays a fat blob rather than a dot: the
         letters keep 12% of their spread and 70% of their size, so there is always a deliberate
         mass of ink on stage instead of a speck. */
      var conv = E.inQuad(span(t, 0, 0.44));         // old letters run to the centre
      var out = 1 - E.outQuint(span(t, 0.42, 0.94));  // new letters run back out
      var oOp = String(1 - span(t, 0.42, 0.50));
      var iOp = String(span(t, 0.40, 0.48));

      for (i = 0; i < O.nodes.length; i++) {
        n = O.nodes[i];
        n.style.transform = 'translate(' + n3(-n._dx * conv * 0.82) + 'px,' + n3(-n._dy * conv * 0.86) + 'px) ' +
          'scale(' + n3(1 - 0.26 * conv) + ')';
        n.style.opacity = oOp;
      }
      for (i = 0; i < I.nodes.length; i++) {
        n = I.nodes[i];
        n.style.transform = 'translate(' + n3(-n._dx * out * 0.82) + 'px,' + n3(-n._dy * out * 0.86) + 'px) ' +
          'scale(' + n3(1 - 0.26 * out) + ')';
        n.style.opacity = iOp;
      }
    },
    rest: function (stage) {
      stage._box.style.filter = 'none';
      stage._out.el.style.transform = 'none';
      stage._in.el.style.transform = 'none';
      stage._out.el.style.textShadow = 'none';
      stage._in.el.style.textShadow = 'none';
      stage._out.nodes.forEach(function (n) { n.style.opacity = '0'; });
      stage._in.nodes.forEach(function (n) { n.style.opacity = '1'; n.style.transform = 'none'; });
    },
  });

  /* =========================================================== 2. Ripple wave ============== */
  FX.register({
    id: 'liquid-ripple-wave',
    name: 'Ripple Wave',
    family: 'Liquid',
    blurb: 'A lens of water travels diagonally across the block; glyphs change under the crest.',
    duration: 1500,
    dwell: 4300,
    theme: {
      bg: '#EEF4F7', fg: '#0E2734', accent: '#2E8FA8',
      font: '"Helvetica Neue", Inter, Helvetica, Arial, sans-serif',
    },
    setup: function (stage, ctx) {
      var wrap = centerWrap(stage);
      stage._L = pairLayer(wrap, ctx, function (c, a) { c.textContent = a; });
      stage._layers = [stage._L];
      stage._ok = false;
    },
    frame: function (stage, t) {
      measured(stage);
      var L = stage._L, cells = L.cells, n = cells.length;
      var mx = L.mx || 1, my = L.my || 1;

      /* wavefront crosses the block; mostly constant speed (a real wave) with a soft in/out */
      var travel = 0.72 * t + 0.28 * E.inOutCubic(t);
      var front = lerp(-0.60, 1.85, travel);
      var wakeEnv = 1 - E.inQuad(span(t, 0.72, 1));

      for (var k = 0; k < n; k++) {
        var c = cells[k];
        /* diagonal coordinate: mostly left-to-right, tilted down the wrapped lines */
        var u = L.mx ? (0.82 * ((c._dx + mx) / (2 * mx)) + 0.18 * ((c._dy + my) / (2 * my)))
                     : (k / Math.max(1, n - 1));
        var d = (u - front) / 0.17;
        var w = gauss(d);                 // crest
        var dw = -2 * d * w;              // slope of the crest -> refraction tilt
        var wake = 0.30 * gauss((d + 1.95) / 1.5) * wakeEnv;

        sideOf(c, u <= front);

        if (w < 0.004 && wake < 0.004) {
          if (c._live) { c.style.transform = 'none'; c.style.filter = 'none'; c.style.opacity = '1'; c._live = 0; }
          continue;
        }
        c._live = 1;
        var lift = -(0.40 * w + 0.15 * wake);
        var sy = 1 + 0.32 * w - 0.07 * wake;
        var sx = 1 - 0.14 * w + 0.04 * wake;
        var skew = clamp(17 * dw, -22, 22);
        c.style.transform = 'translateY(' + n3(lift) + 'em) skewX(' + n3(skew) + 'deg) ' +
          'scale(' + n3(sx) + ',' + n3(sy) + ')';
        c.style.opacity = String(n3(1 - 0.28 * w));
        c.style.filter = w > 0.06 ? 'blur(' + n3(1.7 * w) + 'px)' : 'none';
      }
    },
    rest: function (stage) {
      stage._L.cells.forEach(function (c) {
        sideOf(c, true);
        c.style.transform = 'none'; c.style.filter = 'none'; c.style.opacity = '1'; c._live = 0;
      });
    },
  });

  /* =========================================================== 3. Stretch & snap =========== */
  FX.register({
    id: 'liquid-stretch-snap',
    name: 'Stretch & Snap',
    family: 'Liquid',
    blurb: 'The old line is pulled like taffy until the strand necks and breaks; the new one springs out.',
    duration: 1250,
    dwell: 4200,
    theme: {
      bg: '#131315', fg: '#F6F4F0', accent: '#7AB648',
      font: '"Helvetica Neue", Inter, Helvetica, Arial, sans-serif',
    },
    setup: function (stage, ctx) {
      var wrap = centerWrap(stage);
      stage._out = charLayer(wrap, ctx.from);
      stage._in = charLayer(wrap, ctx.to);
      stage._layers = [stage._out, stage._in];
      stage._ok = false;
    },
    frame: function (stage, t, ctx) {
      measured(stage);
      var O = stage._out, I = stage._in, i, n, f, tr;
      var BREAK = 0.44;

      /* HOW FAR THE ENDS MAY TRAVEL. This is a budget, not a wish: the outermost glyph already
         sits mx from the centre, so anything it is given on top of that has to fit in the half
         width that is left. The old form handed out `pull` for the stretch and then a second,
         larger helping for the recoil, which on a full-width line ("stepper, driver, controller
         and encoder in one.") ran the taffy clean off both edges of the tile. Now one budget
         covers stretch AND recoil, and the per-glyph widening is scaled by the same budget so a
         line that is already full width barely stretches at all. */
      var half = (ctx.W || 600) * 0.5;
      var avail = Math.max(8, half - (O.mx || 1));    // free px beyond the outermost glyph centre
      var budget = clamp(avail - 46, avail * 0.2, avail * 0.9); // 46 px kept for the glyph itself
      var reach = budget / (O.mx || 1);               // travel, as a fraction of the spread
      var stretch = clamp(reach, 0, 1.2);
      var s = E.inQuad(span(t, 0, BREAK));            // the pull
      var rec = E.outQuart(span(t, BREAK, BREAK + 0.24)); // the recoil after the break
      var gone = span(t, BREAK, BREAK + 0.15);

      for (i = 0; i < O.nodes.length; i++) {
        n = O.nodes[i];
        f = Math.abs(n._dx) / (O.mx || 1);            // 0 at the neck, 1 at the ends
        var x = n._dx * reach * (0.66 * s + 0.34 * rec);
        var y = n._dy * (0.22 * s);
        var sx = 1 + (0.25 + 1.45 * stretch) * s * f - 0.6 * rec * f; // ends draw thin, then snap
        var sy = 1 - 0.52 * s * (0.35 + 0.65 * f);
        var neck = 1 - 0.92 * s * (1 - f) * (1 - f);  // the middle thins away first
        tr = 'translate(' + n3(x) + 'px,' + n3(y) + 'px) scale(' + n3(sx) + ',' + n3(Math.max(0.05, sy)) + ')';
        n.style.transform = tr;
        n.style.opacity = String(n3(clamp(neck * (1 - gone), 0, 1)));
      }

      /* the replacement is a compressed column of liquid that springs back into shape */
      for (i = 0; i < I.nodes.length; i++) {
        n = I.nodes[i];
        f = Math.abs(n._dx) / (I.mx || 1);
        var lead = 0.16 * f;                          // centre settles first, ends follow
        var hh = E.outBack(span(t, BREAK - 0.02 + lead, 1));
        var ee = E.outElastic(span(t, BREAK + lead, 1));
        /* outBack overshoots past 1, which here means the letters spring OUTWARD past their
           resting spread. Cap that: on a full-width line the overshoot is the difference between
           "springy" and "off the edge". */
        var k = clamp(1 - hh, -0.05, 1);
        var isx = lerp(0.24, 1, clamp(ee, 0, 1.3));
        var isy = 1 + 0.42 * k;
        n.style.transform = 'translate(' + n3(-n._dx * k) + 'px,' + n3(-n._dy * k * 0.6) + 'px) ' +
          'scale(' + n3(isx) + ',' + n3(isy) + ')';
        n.style.opacity = String(n3(span(t, BREAK, BREAK + 0.13)));
      }
    },
    rest: function (stage) {
      stage._out.nodes.forEach(function (n) { n.style.opacity = '0'; });
      stage._in.nodes.forEach(function (n) { n.style.opacity = '1'; n.style.transform = 'none'; });
    },
  });

  /* =========================================================== 4. Mercury beads ============ */
  FX.register({
    id: 'liquid-mercury',
    name: 'Mercury Beads',
    family: 'Liquid',
    blurb: 'Glyphs ball up into beads of metal that fuse and roll, then re-form as the new line.',
    duration: 1500,
    dwell: 4400,
    theme: {
      bg: '#000000', fg: '#FFFFFF', accent: '#7AB648',
      font: '"Helvetica Neue", Inter, Helvetica, Arial, sans-serif',
    },
    setup: function (stage, ctx) {
      var g = gooWrap(stage, ctx.sc.paper);
      stage._box = g.box;
      stage._paper = ctx.sc.paper;
      /* ONE glyph per cell, not two.
       *
       * Holding both glyphs in one inline-grid cell made every cell as wide as the WIDER of the
       * pair, so the resting line was ~15% too wide, riddled with gaps where a padding cell still
       * carried the other side's width, and on a long slogan it wrapped to three lines where the
       * canon has two. Both endpoints therefore snapped. Instead the cell holds a single glyph
       * that swaps A -> B while it is scaled to nothing under its own bead: the line is exactly
       * canonical A at t->0 and exactly canonical B at t->1, and the swap is never seen because
       * the bead is what occupies that instant. */
      stage._L = pairLayer(g.wrap, ctx, function (c, a, b) {
        var gl = FX.el('span', null, {
          display: 'inline-block', whiteSpace: 'pre', willChange: 'transform,opacity',
        });
        gl.textContent = a || '';
        c.appendChild(gl);
        c._txt = gl;
        if ((a || '').trim() || (b || '').trim()) {
          /* absolutely positioned, so a bead never adds width to its cell */
          var bead = FX.el('span', null, {
            position: 'absolute', left: '50%', top: '50%',
            width: '0.46em', height: '0.46em', marginLeft: '-0.23em', marginTop: '-0.23em',
            borderRadius: '50%', background: 'currentColor',
            transform: 'scale(0)', willChange: 'transform',
          });
          c.appendChild(bead);
          c._bead = bead;
        }
      });
      stage._layers = [stage._L];
      stage._ok = false;
    },
    frame: function (stage, t) {
      measured(stage);
      var L = stage._L, cells = L.cells, n = cells.length;

      /* Surface tension, kept on a short leash. The blur+contrast pair is what fuses beads, but
         it is also a threshold: push it far enough and every glyph stroke falls under the cut and
         the whole line goes white-on-white. Two things keep ink on screen — a modest peak, and a
         glow that swells the strokes so they survive the threshold instead of being erased by it.
         Both also fatten the strokes, so both are gated OFF near the edges, where the frame has
         to be indistinguishable from the engine's plain canonical text. */
      /* A narrow HUMP, not a half-sine. sin(pi*t)^1.35 is already at half strength by t=0.2 and
         does not drop back under it until t=0.8, so the blur and the contrast were smearing the
         WHOLE line for two thirds of the transition — the frames in between were unreadable white
         worms. That contradicted this effect's own design note above, which says only a slice is
         ever molten: the per-cell wave was doing its job, but a global filter was melting
         everything regardless of where the wave had got to. A gaussian centred on the swap is
         spent by t~0.25 and t~0.75, so the line reads at both ends of the melt and the beads are
         an event rather than a state. */
      var g = Math.exp(-Math.pow((t - 0.5) / 0.17, 2));
      var c = 1 + 6.5 * g;
      /* filter always applied, at identity when g is small: toggling `filter` on and off creates
         and destroys a stacking context and the type shifts a fraction of a pixel as it does */
      stage._box.style.filter = 'blur(' + n3(4.6 * g) + 'px) contrast(' + n3(c) + ')';
      stage._box.style.background = preContrast(stage._paper, c);
      var sw = n3(4.4 * Math.pow(g, 0.7));
      stage._L.el.style.textShadow =
        '0 0 ' + sw + 'px currentColor, 0 0 ' + sw + 'px currentColor';

      for (var k = 0; k < n; k++) {
        var c = cells[k];
        var r1 = hash(k, 1), r2 = hash(k, 2), r3 = hash(k, 3);
        /* Liquefaction is a WAVE, not a curtain: a wide lead spread and a short per-cell window
           mean only a slice of the line is ever molten. Ahead of the wave the old glyphs still
           read, behind it the new ones already do — so no frame is all beads. The last cell must
           still be fully re-formed before t reaches 1: 0.52 + 0.95*0.44 = 0.94. */
        var lead = 0.52 * (0.78 * (k / Math.max(1, n - 1)) + 0.22 * r1);
        var p = span(t, lead, lead + 0.44);

        /* The swap happens at p = 0.5, where BOTH halves of the glyph curve are at scale 0.1 and
           opacity 0 — the letter is gone and only the bead is on screen, so the width change the
           swap causes is invisible. */
        var useB = p >= 0.5;
        sideOf(c, useB);
        var sc = useB ? clamp(E.outBack(span(p, 0.62, 0.95)), 0.1, 1.3)
                      : 1 - 0.9 * E.inCubic(span(p, 0, 0.34));
        var op = useB ? span(p, 0.60, 0.82) : 1 - span(p, 0.14, 0.42);
        c._txt.style.transform = 'scale(' + n3(sc) + ')';
        c._txt.style.opacity = String(n3(op));

        var beadIn = E.outBack(span(p, 0.10, 0.46));
        var beadOut = E.inCubic(span(p, 0.58, 0.92));
        var bs = clamp(beadIn, 0, 1.4) * (1 - beadOut);

        if (c._bead) {
          if (bs < 0.004) {
            c._bead.style.transform = 'scale(0)';
          } else {
            var bump = Math.sin(Math.PI * span(p, 0.18, 0.86));
            /* beads slide toward the centre of the line and fuse, with a per-bead wander */
            var bx = -0.20 * c._dx * bump + (r2 - 0.5) * 0.7 * bump * 16;
            var by = ((r3 - 0.5) * 0.9 - 0.25) * bump * 14;
            var wob = 0.16 * bump * Math.sin(6.2 * p + r1 * 6.28);
            c._bead.style.transform = 'translate(' + n3(bx) + 'px,' + n3(by) + 'px) ' +
              'scale(' + n3(bs * (1 + wob)) + ',' + n3(bs * (1 - wob)) + ')';
          }
        }
      }
    },
    rest: function (stage) {
      stage._box.style.filter = 'none';
      stage._L.el.style.textShadow = 'none';
      stage._L.cells.forEach(function (c) {
        sideOf(c, true);
        c._txt.style.opacity = '1'; c._txt.style.transform = 'none';
        if (c._bead) c._bead.style.transform = 'scale(0)';
      });
    },
  });

  /* =========================================================== 5. Drip ==================== */
  /* Each character is a static cell holding a glyph that can move and a tail anchored where the
     glyph started, so the tail can string out behind the falling letter and then break. */
  function dripLayer(wrap, s, tails) {
    var d = absLine(wrap), nodes = [];
    [['fx-light', s.light], ['fx-bold', s.bold]].forEach(function (r) {
      var row = FX.el('div', r[0], { display: 'block' });
      var parts = r[1].split(/(\s+)/);
      parts.forEach(function (p) {
        if (!p) return;
        if (/^\s+$/.test(p)) {
          var gap = FX.el('span', null, { whiteSpace: 'pre' });
          gap.textContent = p;
          row.appendChild(gap);
          return;
        }
        var w = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
        for (var i = 0; i < p.length; i++) {
          var cell = FX.el('span', null, { display: 'inline-block', position: 'relative', whiteSpace: 'pre' });
          var gl = FX.el('span', null, { display: 'inline-block', willChange: 'transform,opacity,filter' });
          gl.textContent = p[i];
          cell.appendChild(gl);
          if (tails) {
            var tl = FX.el('span', null, {
              position: 'absolute', left: '50%', top: '0.70em',
              width: '0.075em', marginLeft: '-0.0375em', height: '0.40em',
              background: 'currentColor', borderRadius: '0 0 45% 45%',
              transformOrigin: '50% 0%', transform: 'scaleY(0)', opacity: '0',
              willChange: 'transform,opacity',
            });
            cell.appendChild(tl);
            gl._tail = tl;
          }
          w.appendChild(cell);
          nodes.push(gl);
        }
        row.appendChild(w);
      });
      d.appendChild(row);
    });
    return { el: d, nodes: nodes };
  }

  FX.register({
    id: 'liquid-drip',
    name: 'Drip',
    family: 'Liquid',
    blurb: 'Letters sag, string out and fall away; the new ones swell up from the baseline.',
    duration: 1600,
    dwell: 4500,
    theme: {
      bg: '#FAF8F4', fg: '#121110', accent: '#7AB648',
      font: 'Georgia, "Iowan Old Style", "Times New Roman", serif',
    },
    setup: function (stage, ctx) {
      var wrap = centerWrap(stage);
      stage._out = dripLayer(wrap, ctx.from, true);
      stage._in = dripLayer(wrap, ctx.to, false);
      stage._in.nodes.forEach(function (n) { n.style.transformOrigin = '50% 100%'; });
      stage._out.nodes.forEach(function (n) { n.style.transformOrigin = '50% 0%'; });
      stage._layers = [];
      stage._ok = true;
    },
    frame: function (stage, t) {
      var O = stage._out.nodes, I = stage._in.nodes, i, n, k;

      /* --- old line: sag, string out, detach, fall --- */
      for (i = 0; i < O.length; i++) {
        n = O[i];
        var r = hash(i, 5);
        var lead = 0.10 * (i / Math.max(1, O.length - 1)) + 0.30 * r;
        var p = span(t, lead, lead + 0.58);
        var sag = E.inQuad(span(p, 0, 0.34));
        var fall = E.inQuad(span(p, 0.30, 1));
        var y = 0.16 * sag + 3.6 * fall;                        // em
        var sy = 1 + 0.55 * sag - 0.42 * E.outCubic(span(p, 0.30, 0.62));
        var sx = 1 - 0.16 * sag + 0.14 * E.outCubic(span(p, 0.30, 0.62));
        var drift = (r - 0.5) * 0.5 * fall;
        n.style.transform = 'translate(' + n3(drift) + 'em,' + n3(y) + 'em) ' +
          'scale(' + n3(sx) + ',' + n3(Math.max(0.05, sy)) + ')';
        n.style.opacity = String(n3(1 - span(p, 0.52, 0.92)));
        n.style.filter = fall > 0.02 ? 'blur(' + n3(4.5 * fall) + 'px)' : 'none';

        if (n._tail) {
          /* the thread follows the drop, then necks and breaks */
          var brk = span(p, 0.40, 0.52);
          var len = clamp(y / 0.40, 0, 4.2) * (1 - E.inQuad(brk));
          n._tail.style.transform = 'scaleY(' + n3(len) + ') scaleX(' + n3(1 - 0.6 * brk) + ')';
          n._tail.style.opacity = String(n3(clamp(sag * 3, 0, 1) * (1 - brk)));
        }
      }

      /* --- new line: swells up out of the baseline, overshoots, settles --- */
      for (k = 0; k < I.length; k++) {
        n = I[k];
        var r2 = hash(k, 9);
        var lead2 = 0.38 + 0.24 * (0.6 * (k / Math.max(1, I.length - 1)) + 0.4 * r2);
        var q = span(t, lead2, Math.min(1, lead2 + 0.42));
        var rise = E.outBack(q);
        var iy = clamp(rise, 0, 1.25);
        var ix = lerp(1.5, 1, E.outCubic(q)) - 0.12 * (rise > 1 ? rise - 1 : 0);
        n.style.transform = 'scale(' + n3(ix) + ',' + n3(Math.max(0.02, iy)) + ')';
        n.style.opacity = String(n3(span(q, 0, 0.3)));
      }
    },
    rest: function (stage) {
      stage._out.nodes.forEach(function (n) {
        n.style.opacity = '0'; n.style.filter = 'none';
        if (n._tail) { n._tail.style.opacity = '0'; n._tail.style.transform = 'scaleY(0)'; }
      });
      stage._in.nodes.forEach(function (n) { n.style.opacity = '1'; n.style.transform = 'none'; });
    },
  });

  /* =========================================================== 6. Shear wobble ============= */
  FX.register({
    id: 'liquid-shear-wobble',
    name: 'Shear Wobble',
    family: 'Liquid',
    blurb: 'A jelly shear runs word by word through the line and rings down to rest.',
    duration: 1400,
    dwell: 4300,
    theme: {
      bg: '#0E1013', fg: '#F1F3EF', accent: '#7AB648',
      font: '"Helvetica Neue", Inter, Helvetica, Arial, sans-serif',
    },
    setup: function (stage, ctx) {
      var wrap = centerWrap(stage);
      stage._out = wordLayer(wrap, ctx.from);
      stage._in = wordLayer(wrap, ctx.to);
      stage._layers = [];
      stage._ok = true;
    },
    frame: function (stage, t) {
      var O = stage._out.nodes, I = stage._in.nodes, i, n;

      /* --- old line: the shear runs away with it until it smears out --- */
      var a = E.inCubic(span(t, 0, 0.5));
      for (i = 0; i < O.length; i++) {
        n = O[i];
        var ph = 2 * Math.PI * 1.35 * t - i * 0.55;
        var osc = Math.sin(ph);
        var ang = 46 * a * osc;
        var x = 0.55 * a * osc;
        var sy = 1 - 0.38 * a;
        var sx = 1 + 0.30 * a;
        n.style.transform = 'translate(' + n3(x) + 'em,0) skewX(' + n3(ang) + 'deg) ' +
          'scale(' + n3(sx) + ',' + n3(sy) + ')';
        n.style.opacity = String(n3(1 - E.inQuad(span(t, 0.14, 0.52))));
        n.style.filter = a > 0.02 ? 'blur(' + n3(6 * a) + 'px)' : 'none';
      }

      /* --- new line: struck once, then a damped ring that ends exactly at zero --- */
      for (i = 0; i < I.length; i++) {
        n = I[i];
        var t0 = 0.30 + Math.min(0.20, 0.042 * i);
        var win = 1 - t0;
        var u = span(t, t0, 1);
        var d = decay(u, 4.4);
        /* frequency held constant in real time, so later words don't ring faster */
        var cyc = 1.55 * (0.68 / win);
        var c1 = Math.cos(2 * Math.PI * cyc * u);
        var c2 = Math.cos(2 * Math.PI * cyc * u + 0.9);
        n.style.transform = 'translate(0,' + n3(-0.30 * d * c2) + 'em) ' +
          'skewX(' + n3(27 * d * c1) + 'deg) ' +
          'scale(' + n3(1 - 0.17 * d * c2) + ',' + n3(1 + 0.24 * d * c1) + ')';
        n.style.opacity = String(n3(span(u, 0, 0.22)));
        n.style.filter = d > 0.35 ? 'blur(' + n3(3.2 * (d - 0.35)) + 'px)' : 'none';
      }
    },
    rest: function (stage) {
      stage._out.nodes.forEach(function (n) { n.style.opacity = '0'; n.style.filter = 'none'; });
      stage._in.nodes.forEach(function (n) {
        n.style.opacity = '1'; n.style.transform = 'none'; n.style.filter = 'none';
      });
    },
  });
})();
