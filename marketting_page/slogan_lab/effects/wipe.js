/* Family: Wipe — masks and edges.
 *
 * Six ways of using a moving boundary to trade one line for the next: a hard bar, a soft
 * gradient sweep, an iris, a per-word column wipe, a diagonal blade and a venetian blind.
 *
 * Every one of them is a pure function of progress t. Nothing here uses a timer, a CSS
 * transition or @keyframes, so any frame can be seeked, screenshotted and exported.
 *
 * Shared structure: each layer is TWO elements —
 *     .wp-clip   full-stage box, carries the clip-path / mask  (the "window")
 *     .wp-inner  the centred text,        carries the transform (the "content")
 * so the window and the content can move independently. Doing the clip and the transform on
 * one element makes the mask travel with the text, which reads as sliding rather than
 * revealing. Keeping them apart is what buys the parallax.
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;
  var ACC = '#7AB648';                    // Gearotons green

  /* A travelling boundary wants softened ends but must not stall in the middle: a pure
     inOutQuart has covered 3% of the stage at t=0.25, which makes the first third of the
     transition look like nothing is happening. Blending it with a constant rate keeps the
     ends eased and the middle honest. */
  function sweep(t) { return 0.42 * t + 0.58 * E.inOutCubic(t); }

  /* ------------------------------------------------------------------ scaffolding */

  /* Two full-stage layers, old then new (new on top). Each layer is always TWO ROWS — light
     half above, bold half below — matching the engine's canonical rest state.
     Returns { wrap, out:{clip,inner}, in:{clip,inner} }. */
  function stack(stage, ctx) {
    stage.innerHTML = '';
    var wrap = FX.el('div', 'wp-wrap', {
      position: 'relative', width: '100%', height: '100%', overflow: 'hidden',
    });
    function layer(s) {
      var clip = FX.el('div', 'wp-clip', {
        position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        willChange: 'clip-path, mask-image',
      });
      var inner = FX.el('div', 'wp-inner', {
        width: '100%', textAlign: 'center', willChange: 'transform, opacity, filter',
      });
      // Two ROWS, never a <br>. The engine forces the two halves to display:block (the canonical
      // rest state is two block lines); a <br> between two blocks then contributes a THIRD, empty
      // line box, so the pair sat further apart and lower than the canon and the headline jumped
      // at t=0.02 / t=0.98. Set the blocks ourselves and drop the break.
      var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
      var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
      inner.appendChild(a); inner.appendChild(b);
      clip.appendChild(inner);
      wrap.appendChild(clip);
      return { clip: clip, inner: inner };
    }
    var out = layer(ctx.from), inn = layer(ctx.to);
    stage.appendChild(wrap);
    return { wrap: wrap, out: out, in: inn };
  }

  function mask(node, value) {
    node.style.webkitMaskImage = node.style.maskImage = value;
  }

  /* --------------------------------------------------- 1. hard bar wipe ---------- */
  /* One opaque edge crosses the stage. Ahead of it the old line, behind it the new one, and
     riding the boundary a green slab with a razor line at its leading edge. The two texts
     drift a little inside their windows so the reveal has parallax instead of feeling like
     a sheet of paper being slid over a printout. */
  FX.register({
    id: 'wipe-bar',
    name: 'Bar Wipe',
    family: 'Wipe',
    blurb: 'A green slab sweeps across; new line behind its edge, old line ahead of it.',
    duration: 1050,
    dwell: 4200,
    setup: function (stage, ctx) {
      var L = stack(stage, ctx);
      var trail = FX.el('div', null, {
        position: 'absolute', top: '0', bottom: '0', left: '0', width: '104px',
        background: 'linear-gradient(90deg, rgba(122,182,72,0) 0%, rgba(122,182,72,.10) 46%,'
                  + ' rgba(122,182,72,.34) 84%, rgba(122,182,72,.62) 100%)',
        willChange: 'transform, opacity', pointerEvents: 'none',
      });
      // wide enough to hide the junction where the two different texts butt together
      var edge = FX.el('div', null, {
        position: 'absolute', top: '0', bottom: '0', left: '0', width: '9px',
        background: 'linear-gradient(90deg, rgba(122,182,72,.72), ' + ACC + ' 55%, #9FD469)',
        boxShadow: '0 0 18px rgba(122,182,72,.7)',
        willChange: 'transform, opacity', pointerEvents: 'none',
      });
      L.wrap.appendChild(trail); L.wrap.appendChild(edge);
      L.trail = trail; L.edge = edge;
      stage._l = L;
    },
    frame: function (stage, t, ctx) {
      var L = stage._l, W = Math.max(1, ctx.W);
      // the edge overshoots both ends slightly so it enters and leaves off-stage
      var p = sweep(t);
      var x = lerp(-0.05, 1.07, p) * W;
      var pct = clamp(x / W * 100, 0, 100);

      L.in.clip.style.clipPath = 'inset(0 ' + (100 - pct) + '% 0 0)';
      L.out.clip.style.clipPath = 'inset(0 0 0 ' + pct + '%)';
      mask(L.in.clip, 'none'); mask(L.out.clip, 'none');

      // parallax: the new line settles in from the left, the old one is shoved right
      var s = E.outQuart(span(t, 0.06, 1));
      L.in.inner.style.transform = 'translateX(' + (-26 * (1 - s)) + 'px)';
      L.in.inner.style.opacity = '1';
      L.out.inner.style.transform = 'translateX(' + (20 * E.inQuad(t)) + 'px)';
      L.out.inner.style.opacity = String(1 - 0.25 * E.inQuad(t));

      var vis = Math.min(span(t, 0, 0.07), span(1 - t, 0, 0.10));
      L.trail.style.transform = 'translateX(' + (x - 104) + 'px)';
      L.trail.style.opacity = String(vis);
      L.edge.style.transform = 'translateX(' + (x - 5) + 'px)';
      L.edge.style.opacity = String(vis);
    },
    rest: function (stage) {
      var L = stage._l;
      L.in.clip.style.clipPath = 'none';
      L.out.clip.style.clipPath = 'inset(0 0 0 100%)';
      L.in.inner.style.transform = 'none';
      L.trail.style.opacity = '0'; L.edge.style.opacity = '0';
    },
  });

  /* ------------------------------------------- 2. soft gradient sweep ------------ */
  /* The same left-to-right idea with the hard edge replaced by a wide feather: the old line
     dissolves into the paper a beat before the new one condenses out of it, and a faint
     green bloom marks where the boundary is. Blur is tied to the sweep so the incoming text
     sharpens as the band passes over it. */
  FX.register({
    id: 'wipe-gradient',
    name: 'Gradient Sweep',
    family: 'Wipe',
    blurb: 'A wide feathered boundary drifts across; one line dissolves as the other condenses.',
    duration: 1400,
    dwell: 4600,
    theme: { bg: '#FAF9F5', fg: '#20211E',
             font: '"Avenir Next", "Segoe UI", Inter, system-ui, sans-serif' },
    setup: function (stage, ctx) {
      var L = stack(stage, ctx);
      var band = FX.el('div', null, {
        position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
        pointerEvents: 'none', willChange: 'background, opacity',
      });
      L.wrap.appendChild(band);
      L.band = band;
      stage._l = L;
    },
    frame: function (stage, t) {
      var L = stage._l;
      /* Feather half-width, % of the sweep. At 24 the soft band covered nearly HALF the stage,
         so at t=0.5 both slogans sat on top of each other at similar strength and the middle of
         the transition read as a double exposure rather than a sweep. 13 keeps the boundary
         obviously soft while the overlap stays a narrow ghosting zone. */
      var F = 13;
      var e = lerp(-F, 100 + F, E.inOutCubic(t));     // centre of the feather
      var a = (e - F).toFixed(2), b = (e + F).toFixed(2);

      mask(L.in.clip, 'linear-gradient(97deg, #000 ' + a + '%, rgba(0,0,0,0) ' + b + '%)');
      mask(L.out.clip, 'linear-gradient(97deg, rgba(0,0,0,0) ' + a + '%, #000 ' + b + '%)');
      L.in.clip.style.clipPath = 'none'; L.out.clip.style.clipPath = 'none';

      var s = E.outCubic(span(t, 0.1, 1));
      L.in.inner.style.filter = 'blur(' + (4 * (1 - s)) + 'px)';
      L.in.inner.style.transform = 'translateX(' + (-18 * (1 - s)) + 'px) scale(' + (1 - 0.014 * (1 - s)) + ')';
      // the departing line goes soft faster than the arriving one sharpens, so wherever the two
      // do overlap one of them is clearly the ghost
      var o = E.inQuad(span(t, 0, 0.9));
      L.out.inner.style.filter = 'blur(' + (7 * o) + 'px)';
      L.out.inner.style.transform = 'translateX(' + (16 * o) + 'px) scale(' + (1 + 0.014 * o) + ')';

      var glow = Math.min(span(t, 0, 0.12), span(1 - t, 0, 0.14));
      L.band.style.opacity = String(glow);
      L.band.style.background =
        'linear-gradient(97deg, rgba(122,182,72,0) ' + (e - F * 1.1).toFixed(2) + '%,'
        + ' rgba(122,182,72,.20) ' + e.toFixed(2) + '%,'
        + ' rgba(122,182,72,0) ' + (e + F * 1.1).toFixed(2) + '%)';
    },
    rest: function (stage) {
      var L = stage._l;
      mask(L.in.clip, 'none'); mask(L.out.clip, 'none');
      L.out.clip.style.clipPath = 'inset(0 0 0 100%)';
      L.in.inner.style.filter = 'none';
      L.in.inner.style.transform = 'none';
      L.band.style.opacity = '0';
    },
  });

  /* ------------------------------------------------------- 3. iris ------------- */
  /* A circle opens from the centre. The old line is eaten from the inside out by a matching
     hole — the two share one boundary, so no frame ever shows both stacked — and a thin
     green ring rides that boundary and fades as it leaves the stage.
     Radius maths: clip-path circle(P%) resolves against hypot(w,h)/√2, so P >= 70.71 covers
     the corners; a radial-gradient's 100% is hypot(w,h)/2, hence the ×1.4142 conversion. */
  FX.register({
    id: 'wipe-iris',
    name: 'Iris',
    family: 'Wipe',
    blurb: 'A ring opens from the centre; the old line is eaten out of the middle.',
    duration: 1250,
    dwell: 4400,
    setup: function (stage, ctx) {
      var L = stack(stage, ctx);
      var ring = FX.el('div', null, {
        position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
        pointerEvents: 'none', willChange: 'background, opacity',
      });
      L.wrap.appendChild(ring);
      L.ring = ring;
      stage._l = L;
    },
    frame: function (stage, t) {
      var L = stage._l;
      /* outQuint here punched a 150px hole at t=0.02 and had the ring off-stage by t=0.15: the
         iris POPPED open at the start and then the remaining 85% of the transition had nothing
         left to do. inOutQuad leaves the centre still at t=0, and 78% covers the corners
         (>70.71) only at t=0.78, so the aperture uses the whole transition. */
      var r = lerp(0, 78, E.inOutQuad(t));           // clip-path radius, %
      L.in.clip.style.clipPath = 'circle(' + r.toFixed(2) + '% at 50% 50%)';
      mask(L.in.clip, 'none');

      // the hole in the old layer sits on the same boundary, with a 3% feather inside it
      var g = r * 1.4142;                            // same radius, gradient units
      mask(L.out.clip,
        'radial-gradient(circle at 50% 50%, rgba(0,0,0,0) ' + Math.max(0, g - 4).toFixed(2) + '%,'
        + ' #000 ' + g.toFixed(2) + '%)');

      // the aperture IS the reveal, so the incoming line stays fully opaque behind it — fading it
      // as well left the first frames showing a washed-out new line inside the hole
      var s = E.inOutCubic(t);
      L.in.inner.style.transform = 'scale(' + lerp(0.93, 1, s).toFixed(4) + ')';
      L.in.inner.style.opacity = '1';
      var o = E.inQuad(t);
      L.out.inner.style.transform = 'scale(' + (1 + 0.09 * o) + ')';
      L.out.inner.style.filter = 'blur(' + (3.5 * o) + 'px)';
      L.out.inner.style.opacity = String(1 - 0.35 * o);

      var vis = Math.min(span(t, 0, 0.06), span(1 - t, 0, 0.22));
      L.ring.style.opacity = String(vis);
      L.ring.style.background =
        'radial-gradient(circle at 50% 50%,'
        + ' rgba(122,182,72,0) ' + Math.max(0, g - 1.6).toFixed(2) + '%,'
        + ' rgba(122,182,72,.85) ' + g.toFixed(2) + '%,'
        + ' rgba(122,182,72,.28) ' + (g + 1.4).toFixed(2) + '%,'
        + ' rgba(122,182,72,0) ' + (g + 9).toFixed(2) + '%)';
    },
    rest: function (stage) {
      var L = stage._l;
      L.in.clip.style.clipPath = 'none';
      L.in.inner.style.transform = 'none';
      L.in.inner.style.opacity = '1';
      mask(L.out.clip, 'radial-gradient(circle at 50% 50%, rgba(0,0,0,0) 200%, #000 200%)');
      L.out.inner.style.opacity = '0';
      L.ring.style.opacity = '0';
    },
  });

  /* ------------------------------------------- 4. per-word column wipe ----------- */
  /* Each word is its own little window. The old word is erased upward out of its box while
     the new one is drawn upward into it, and the columns fire left-to-right with a lead so
     the line reads as a wave rather than a switch. The clip lives on the word span and the
     movement on a span inside it, so the text really is being uncovered. */
  function wordsInto(container, text, cls, pairs) {
    var parts = text.split(/\s+/).filter(Boolean);
    parts.forEach(function (p, i) {
      /* No overflow:hidden here. An inline-block with overflow != visible takes its BOTTOM MARGIN
         EDGE as its baseline instead of its text's baseline, which pushed the light row 13px off
         the canonical position and opened an extra gap between the two lines. The clip-path set in
         frame() does all the masking anyway; the padding (cancelled by the negative margin, so
         layout is unchanged) only widens the box so inset(0 …) does not shave ascenders. */
      var outer = FX.el('span', cls, {
        display: 'inline-block',
        padding: '0.26em 0.04em', margin: '-0.26em -0.04em',
        willChange: 'clip-path',
      });
      var inner = FX.el('span', null, { display: 'inline-block', willChange: 'transform' });
      inner.textContent = p;
      outer.appendChild(inner);
      container.appendChild(outer);
      if (i < parts.length - 1) {
        var gap = FX.el('span', null, { whiteSpace: 'pre' });
        gap.textContent = ' ';
        container.appendChild(gap);
      }
      pairs.push({ o: outer, i: inner });
    });
  }

  /* inset that ignores the word box's 0.26em breathing padding: v% of the GLYPH band, measured
     from the top (fromTop) or the bottom. */
  function pctPad(v, fromTop) {
    var pad = (0.26 - 0.0052 * v).toFixed(4) + 'em';
    var len = 'calc(' + v.toFixed(2) + '% + ' + pad + ')';
    return fromTop ? 'inset(' + len + ' 0 0 0)' : 'inset(0 0 ' + len + ' 0)';
  }

  FX.register({
    id: 'wipe-columns',
    name: 'Word Columns',
    family: 'Wipe',
    blurb: 'Word by word, the old text is erased upward and the new drawn up in its place.',
    duration: 1300,
    dwell: 4400,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'wp-wrap', {
        position: 'relative', width: '100%', height: '100%', overflow: 'hidden',
      });
      function layer(s) {
        var box = FX.el('div', null, {
          position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
        });
        var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
        var pairs = [];
        var light = FX.el('span', 'fx-light', { display: 'block' });
        wordsInto(light, s.light, null, pairs);
        inner.appendChild(light);
        var bold = FX.el('span', 'fx-bold', { display: 'block' });
        wordsInto(bold, s.bold, null, pairs);
        inner.appendChild(bold);
        box.appendChild(inner); wrap.appendChild(box);
        return pairs;
      }
      stage._out = layer(ctx.from);
      stage._in = layer(ctx.to);
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      var O = stage._out, I = stage._in;
      var no = O.length, ni = I.length;
      var k;
      for (k = 0; k < no; k++) {
        var lead = no > 1 ? (k / (no - 1)) * 0.34 : 0;
        /* inOutQuad, not inOutQuart: a quartic ease-IN spends the first third of each word's
           window moving the cut through descender slack, so at t=0.15 the wave had not visibly
           started at all. */
        var p = E.inOutQuad(span(t, lead, lead + 0.42));
        /* Erased from the bottom edge upward: the word vanishes as if lifted out.
           The 0.26em padding is empty, so a plain 0→100% inset spends its first and last 15%
           eating padding and the word looks frozen at both ends of its window. Offsetting the
           inset by the padding makes the whole travel land on actual glyph. */
        O[k].o.style.clipPath = pctPad(100 * p, false);
        O[k].i.style.transform = 'translateY(' + (-0.14 * p) + 'em)';
        O[k].i.style.opacity = String(1 - 0.15 * p);
      }
      for (k = 0; k < ni; k++) {
        // the new words follow closely enough that a line is never left empty mid-transition
        var lead2 = 0.18 + (ni > 1 ? (k / (ni - 1)) * 0.34 : 0);
        var q = E.outQuart(span(t, lead2, lead2 + 0.42));
        I[k].o.style.clipPath = pctPad(100 * (1 - q), true);
        I[k].i.style.transform = 'translateY(' + (0.16 * (1 - q)) + 'em)';
      }
    },
    rest: function (stage) {
      stage._out.forEach(function (w) { w.o.style.clipPath = 'inset(0 0 100% 0)'; });
      stage._in.forEach(function (w) {
        w.o.style.clipPath = 'none';
        w.i.style.transform = 'none';
      });
    },
  });

  /* ------------------------------------------------------ 5. diagonal blade ------ */
  /* A raked edge crosses the stage and the two lines live on either side of it. The blade
     itself is a lit filament with a glow; both texts slide a few pixels along the blade's
     normal so the cut feels like it has force behind it.
     Geometry: the boundary runs from (p+s, 0) to (p-s, 100%) in % of the box, so the shown
     region for the incoming line is the polygon to its left. */
  FX.register({
    id: 'wipe-blade',
    name: 'Blade',
    family: 'Wipe',
    blurb: 'A raked, lit edge rakes across and swaps the line on either side of itself.',
    duration: 1100,
    dwell: 4200,
    theme: { bg: '#101014', fg: '#F2F2F6' },
    setup: function (stage, ctx) {
      var L = stack(stage, ctx);
      var blade = FX.el('div', null, {
        position: 'absolute', top: '50%', left: '50%', width: '2px', height: '320%',
        background: 'linear-gradient(180deg, rgba(122,182,72,.25), #B7F07A 42%, #B7F07A 58%, rgba(122,182,72,.25))',
        boxShadow: '0 0 22px rgba(122,182,72,.9), 0 0 60px rgba(122,182,72,.35)',
        pointerEvents: 'none', willChange: 'transform, opacity',
      });
      L.wrap.appendChild(blade);
      L.blade = blade;
      stage._l = L;
    },
    frame: function (stage, t, ctx) {
      var L = stage._l;
      var W = Math.max(1, ctx.W), H = Math.max(1, ctx.H);
      var s = 17;                                            // rake, % of width per half-height
      /* sweep(), not inOutQuart: with the quartic the blade was still off-stage at t=0.36 and had
         already left by t=0.64, so two thirds of the transition were a frozen picture. The blended
         curve keeps the entry and exit eased while the middle travels at an honest rate. */
      var p = lerp(-s - 3, 100 + s + 3, sweep(t));           // midpoint of the edge, % of width

      var top = (p + s).toFixed(2), bot = (p - s).toFixed(2);
      L.in.clip.style.clipPath =
        'polygon(-40% -40%, ' + top + '% -40%, ' + bot + '% 140%, -40% 140%)';
      L.out.clip.style.clipPath =
        'polygon(' + top + '% -40%, 140% -40%, 140% 140%, ' + bot + '% 140%)';

      /* Blade placement: MOVE first, then rotate about the moved centre. Rotating first and then
         stepping along the blade's own x-axis put the crossing at dh/cos² and slid the finite bar
         dh·tan(rake) up the stage, so at this ~50° rake the blade was off the top of the box for
         everything except t=0.5 — the lit edge simply was not there for most of the sweep.
         Translating in page coordinates first puts the bar's centre exactly on the boundary's
         midpoint, which is where the clip's own midpoint is. */
      var dx = (2 * s / 100) * W;                            // horizontal run over the full height
      var ang = Math.atan2(dx, H);                           // radians, clockwise from vertical
      var dh = (p / 100) * W - W / 2;                        // boundary midpoint, px from centre
      L.blade.style.transform =
        'translate(-50%,-50%) translateX(' + dh.toFixed(2) + 'px)'
        + ' rotate(' + (ang * 180 / Math.PI).toFixed(3) + 'deg)';
      L.blade.style.opacity = String(Math.min(span(t, 0, 0.08), span(1 - t, 0, 0.10)));

      // both texts slide along the blade normal: old shoved ahead, new settling back
      var nx = Math.cos(ang), ny = -Math.sin(ang) * 0.55;    // unit-ish normal, flattened
      var a = E.outQuart(span(t, 0.05, 1));
      L.in.inner.style.transform =
        'translate(' + (-30 * nx * (1 - a)).toFixed(2) + 'px,' + (-30 * ny * (1 - a)).toFixed(2) + 'px)';
      var o = E.inQuad(t);
      L.out.inner.style.transform =
        'translate(' + (24 * nx * o).toFixed(2) + 'px,' + (24 * ny * o).toFixed(2) + 'px)';
      L.out.inner.style.opacity = String(1 - 0.3 * o);
    },
    rest: function (stage) {
      var L = stage._l;
      L.in.clip.style.clipPath = 'none';
      L.in.inner.style.transform = 'none';
      L.out.clip.style.clipPath = 'polygon(140% -40%, 140% -40%, 140% 140%, 140% 140%)';
      L.blade.style.opacity = '0';
    },
  });

  /* ------------------------------------------------------ 6. venetian blind ------ */
  /* The line is cut into horizontal slats. Each slat is a real 3D flap: the old face turns
     edge-on and the new face turns back to flat, with the handover happening while the slat
     is invisibly thin, so you never see the two texts crossing. Slats fire top to bottom.
     Every slat holds a full copy of the line offset upward by its own top, so the text is
     continuous across the stack — boundaries are integers to keep the seams invisible. */
  FX.register({
    id: 'wipe-blinds',
    name: 'Venetian Blind',
    family: 'Wipe',
    blurb: 'Horizontal slats turn edge-on and come back showing the new line.',
    duration: 1350,
    dwell: 4600,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      // H only picks how MANY slats; nothing is positioned from it
      var H = Math.max(60, ctx.H || stage.clientHeight || 160);
      var n = Math.max(6, Math.min(12, Math.round(H / 44)));
      var wrap = FX.el('div', 'wp-wrap', {
        position: 'relative', width: '100%', height: '100%', overflow: 'hidden',
        perspective: '1100px',
      });
      /* Geometry in PERCENT of the slat, never in measured pixels. Sizing the faces from ctx.H
         put the headline 45px above the canonical position whenever the measured height did not
         match the box the slats actually filled — the text was centred in the WRONG height. A
         face is n×100% of its slat tall and offset -k×100%, which is exactly the wrap height and
         exactly the slat's own top, whatever that height turns out to be. */
      function face(s, k) {
        var f = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0',
          top: (-k * 100) + '%', height: (n * 100) + '%',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          willChange: 'opacity',
        });
        var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
        var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
        var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
        inner.appendChild(a); inner.appendChild(b);
        f.appendChild(inner);
        return f;
      }
      stage._slats = [];
      for (var k = 0; k < n; k++) {
        var slat = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0',
          top: (k * 100 / n).toFixed(4) + '%', height: (100 / n).toFixed(4) + '%',
          overflow: 'hidden',
          transformOrigin: '50% 50%', willChange: 'transform, opacity',
        });
        var fo = face(ctx.from, k), fi = face(ctx.to, k);
        slat.appendChild(fo); slat.appendChild(fi);
        wrap.appendChild(slat);
        stage._slats.push({ el: slat, out: fo, in: fi });
      }
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      var S = stage._slats, n = S.length;
      for (var k = 0; k < n; k++) {
        /* The stagger is the whole effect. With a short lead spread and a long per-slat window
           every slat is edge-on at once and the stage empties out; real blinds turn slat by slat.
           So: a WIDE lead spread and a SHORT turn, which means at any t a band of slats near the
           top already shows the new line flat, a band near the bottom still shows the old one
           flat, and only two or three slats in between are mid-turn. */
        var lead = (k / Math.max(1, n - 1)) * 0.62;
        var p = span(t, lead, lead + 0.38);
        // one continuous quarter-turn out and a quarter-turn back
        var half = p < 0.5 ? E.inOutQuad(p / 0.5) : E.inOutQuad((p - 0.5) / 0.5);
        var ang = p < 0.5 ? -90 * half : -90 * (1 - half);
        var vis = Math.max(0, Math.cos(ang * Math.PI / 180));
        S[k].el.style.transform = 'rotateX(' + ang.toFixed(2) + 'deg)';
        /* A slat turned edge-on is already invisible — it has no height. Fading it as well used
           to drain the flat slats either side of the turn too, which is what made the middle of
           the transition read as blank. Keep them nearly solid; just shade the tilt. */
        S[k].el.style.opacity = (0.78 + 0.22 * vis).toFixed(3);
        var showNew = p >= 0.5;
        S[k].out.style.opacity = showNew ? '0' : '1';
        S[k].in.style.opacity = showNew ? '1' : '0';
      }
    },
    rest: function (stage) {
      stage._slats.forEach(function (s) {
        s.el.style.transform = 'none';
        s.el.style.opacity = '1';
        s.out.style.opacity = '0';
        s.in.style.opacity = '1';
      });
    },
  });
})();
