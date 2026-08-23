/* Slide family — six transitions built on travel: pushes, belts, rolls, shears and arcs.
 *
 * Every effect here is a pure function of progress t. Positions, opacities, shears and the
 * velocity-derived motion blur are all evaluated from t alone, so any frame can be seeked,
 * screenshotted or exported. Nothing animates itself.
 *
 * Shared rules used throughout:
 *   - lines are absolutely positioned, full stage width, centred text: they wrap to 2-3 lines
 *     for long slogans and never affect each other's layout.
 *   - words are inline-blocks separated by real text-node spaces, so the browser keeps its
 *     normal break opportunities and never breaks a word in half.
 *   - motion blur is derived from the actual derivative of the position curve and gated to
 *     zero at t=0 and t=1, so the resting frames are always crisp.
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, clamp = FX.clamp, el = FX.el;

  var GREEN = '#7AB648';

  /* ---------------------------------------------------------------- plumbing */

  /* Words as inline-blocks, joined by plain text nodes so wrapping behaves normally.
     (A white-space:pre spacer suppresses the break opportunity and can overflow the stage.) */
  function addWords(parent, text, cls, sink) {
    var parts = String(text || '').split(/\s+/).filter(Boolean);
    for (var i = 0; i < parts.length; i++) {
      var s = el('span', cls, { display: 'inline-block', willChange: 'transform,opacity,filter' });
      // position of this word WITHIN ITS OWN HALF, 0..1. The word lists are flat (light half
      // then bold half), so a stagger taken from the flat index makes one row overtake the
      // other; effects whose words move vertically need the per-row figure instead.
      s._wu = parts.length > 1 ? i / (parts.length - 1) : 0;
      s.textContent = parts[i];
      parent.appendChild(s);
      sink.push(s);
      if (i < parts.length - 1) parent.appendChild(document.createTextNode(' '));
    }
  }

  /* Two stacked full-width lines (old above new in z-order) plus their word lists. */
  function build(stage, ctx, o) {
    o = o || {};
    stage.innerHTML = '';
    var wrap = el('div', 'fx-wrap', {
      position: 'relative', width: '100%', height: '100%',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    });
    if (o.mask) { wrap.style.webkitMaskImage = wrap.style.maskImage = o.mask; }
    if (o.perspective) {
      wrap.style.perspective = o.perspective;
      wrap.style.perspectiveOrigin = '50% 50%';
    }
    function mk(s) {
      var d = el('div', 'fx-line', {
        position: 'absolute', left: '0', right: '0', textAlign: 'center',
        willChange: 'transform,opacity,filter', backfaceVisibility: 'hidden',
      });
      var w = [];
      /* Two BLOCK rows, no <br>. These layers tag individual WORDS .fx-light/.fx-bold, so
         the engine's two-line enforcement deliberately skips them (it only acts on a parent
         holding exactly one of each) and the <br> was the only thing separating the halves —
         which meant an extra anonymous line box and a headline that jumped a whole line the
         moment the effect took over. Giving each half its own block row separates them the
         same way the canonical layer does. Every caller passes split:true, so the old
         one-line branch was dead code and is gone. */
      var rowL = el('div', null, { display: 'block' });
      var rowB = el('div', null, { display: 'block' });
      addWords(rowL, s.light, 'fx-light', w);
      addWords(rowB, s.bold, 'fx-bold', w);
      d.appendChild(rowL); d.appendChild(rowB);
      wrap.appendChild(d);
      return { el: d, words: w };
    }
    var a = mk(ctx.from), b = mk(ctx.to);
    stage.appendChild(wrap);
    return { wrap: wrap, out: a.el, in: b.el, outW: a.words, inW: b.words };
  }

  /* outBack with a tunable overshoot: c=1.70158 is the stock ~10%; small c = a firm settle. */
  function back(s, c) {
    return 1 + (c + 1) * Math.pow(s - 1, 3) + c * Math.pow(s - 1, 2);
  }

  /* Numeric |derivative| of a progress curve — the source of every motion blur below. */
  function speed(fn, t) {
    var h = 0.012, a = clamp(t - h, 0, 1), b = clamp(t + h, 0, 1);
    if (b - a < 1e-6) return 0;
    return Math.abs(fn(b) - fn(a)) / (b - a);
  }

  function mb(px) { return px > 0.15 ? 'blur(' + px.toFixed(2) + 'px)' : 'none'; }
  function gate(t) { return Math.sin(Math.PI * clamp(t, 0, 1)); }
  function u(k, n) { return n > 1 ? k / (n - 1) : 0; }

  /* A one-shot bump on [a,b] that is zero AND flat at both ends — sin() alone leaves a steep
     slope at s=0, which reads as a jolt in the first frame after the engine hands over from the
     canonical rest state. Squaring it keeps the same peak but starts and finishes gently. */
  function bump(t, a, b) { var s = Math.sin(Math.PI * span(t, a, b)); return s * s; }

  function clearAll(nodes) {
    for (var i = 0; i < nodes.length; i++) {
      nodes[i].style.transform = 'none';
      nodes[i].style.opacity = '1';
      nodes[i].style.filter = 'none';
    }
  }

  /* ---- 1. Hard Push -------------------------------------------------------
     Filmstrip-rigid: the two lines are locked D apart and travel together, so the new line
     physically shoves the old one out. A bright seam rides the join, and the whole strip
     anticipates a hair to the right before the shove and overshoots a hair on arrival. */
  FX.register({
    id: 'slide-push',
    name: 'Hard Push',
    family: 'Slide',
    blurb: 'The new line shoves the old one off the edge, a lit seam riding the join.',
    duration: 860,
    dwell: 4200,
    setup: function (stage, ctx) {
      var L = build(stage, ctx, { split: true });
      L.bar = el('div', null, {
        position: 'absolute', left: '50%', top: '50%', width: '3px', height: '56%',
        borderRadius: '3px', pointerEvents: 'none', willChange: 'transform,opacity,filter',
        background: 'linear-gradient(180deg, rgba(122,182,72,0) 0%, ' + GREEN + ' 20%, ' +
                    GREEN + ' 80%, rgba(122,182,72,0) 100%)',
        boxShadow: '0 0 26px rgba(122,182,72,.55)',
      });
      L.wrap.appendChild(L.bar);
      stage._push = L;
    },
    frame: function (stage, t, ctx) {
      var L = stage._push, W = ctx.W || 600, D = W + 60;
      // one curve for the whole strip: a hair of anticipation, a firm shove through the middle,
      // then a small overshoot that pulls back to the mark.
      var f = function (x) {
        var s = span(x, 0.04, 1);
        return 0.35 * E.outCubic(s) + 0.65 * E.inOutQuart(s)   // punchy start, long settle
          - 0.034 * bump(x, 0, 0.18)                           // anticipation
          + 0.022 * bump(x, 0.58, 1);                          // overshoot, back to the mark
      };
      var p = f(t);
      var blur = Math.min(3.6, speed(f, t) * D / 620) * gate(t);
      var x = -D * p;

      L.out.style.transform = 'translate3d(' + x.toFixed(2) + 'px,0,0)';
      L.in.style.transform = 'translate3d(' + (x + D).toFixed(2) + 'px,0,0)';
      L.out.style.opacity = L.in.style.opacity = '1';
      L.out.style.filter = L.in.style.filter = mb(blur);

      // the seam sits at the join, so gate it on travel (peaks as it crosses the centre)
      var g = Math.sin(Math.PI * clamp(p, 0, 1));
      L.bar.style.transform = 'translate(-50%,-50%) translateX(' + (x + D / 2).toFixed(2) +
        'px) scaleY(' + (0.70 + 0.30 * g).toFixed(3) + ')';
      L.bar.style.opacity = (0.95 * g).toFixed(3);
      L.bar.style.filter = mb(blur * 0.55);
    },
    rest: function (stage, ctx) {
      var L = stage._push, D = (ctx.W || 600) + 60;
      L.out.style.transform = 'translate3d(' + (-D) + 'px,0,0)';
      L.in.style.transform = 'none';
      L.out.style.filter = L.in.style.filter = 'none';
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.bar.style.opacity = '0';
    },
  });

  /* ---- 2. Word Conveyor ---------------------------------------------------
     Every word rides its own belt. The old line is carried off left in reading order, the new
     one arrives on the same belt a beat later, each word stretching slightly along its travel
     and smearing at speed. The stagger is what turns a slide into a mechanism. */
  FX.register({
    id: 'slide-conveyor',
    name: 'Word Conveyor',
    family: 'Slide',
    blurb: 'Word by word, the old line is carried off left as the new one rides in behind it.',
    duration: 1250,
    dwell: 4400,
    setup: function (stage, ctx) { stage._cv = build(stage, ctx, { split: true }); },
    frame: function (stage, t, ctx) {
      var L = stage._cv, W = ctx.W || 600, TR = W * 0.78 + 90;
      var i, n, k, uu, d, p, o, sx, bl;

      n = L.outW.length;
      for (i = 0; i < n; i++) {
        uu = u(i, n);
        d = uu * 0.30;
        k = span(t, d, d + 0.34);
        p = E.inCubic(k);
        o = 1 - E.inQuad(span(t, d + 0.10, d + 0.34));
        sx = 1 + 0.09 * Math.sin(Math.PI * k);
        bl = 6.5 * Math.sin(Math.PI * k) * Math.sin(Math.PI * k);
        L.outW[i].style.transform = 'translate3d(' + (-TR * p).toFixed(2) + 'px,0,0) scaleX(' + sx.toFixed(3) + ')';
        L.outW[i].style.opacity = o.toFixed(3);
        L.outW[i].style.filter = mb(bl);
      }

      n = L.inW.length;
      for (i = 0; i < n; i++) {
        uu = u(i, n);
        d = 0.30 + uu * 0.30;
        k = span(t, d, d + 0.40);
        p = E.outQuint(k);
        o = E.outQuad(span(t, d, d + 0.24));
        sx = 1 + 0.09 * Math.sin(Math.PI * k);
        bl = 6.5 * Math.sin(Math.PI * k) * Math.sin(Math.PI * k);
        L.inW[i].style.transform = 'translate3d(' + (TR * (1 - p)).toFixed(2) + 'px,0,0) scaleX(' + sx.toFixed(3) + ')';
        L.inW[i].style.opacity = o.toFixed(3);
        L.inW[i].style.filter = mb(bl);
      }
      L.out.style.opacity = '1';
      L.in.style.opacity = '1';
    },
    rest: function (stage) {
      var L = stage._cv;
      clearAll(L.inW);
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
    },
  });

  /* ---- 3. Ticker Roll -----------------------------------------------------
     A departure-board window: the old line rolls up and out through a soft-edged aperture
     while the next one climbs into its place, each word a fraction behind the last so the
     line reads as a row of separate flaps rather than one slab. */
  FX.register({
    id: 'slide-ticker',
    name: 'Ticker Roll',
    family: 'Slide',
    blurb: 'The line rolls up and out of a soft-edged window; the next one climbs in behind it.',
    duration: 1150,
    dwell: 4300,
    theme: { bg: '#0F1113', fg: '#F1F3F5' },
    setup: function (stage, ctx) {
      stage._tk = build(stage, ctx, {
        split: true,
        mask: 'linear-gradient(180deg, rgba(0,0,0,0) 0%, #000 13%, #000 87%, rgba(0,0,0,0) 100%)',
      });
    },
    frame: function (stage, t, ctx) {
      var L = stage._tk, H = ctx.H || 160, TR = H * 0.62 + 46;
      var i, n, uu, d, k, p, o, bl;

      n = L.outW.length;
      for (i = 0; i < n; i++) {
        uu = u(i, n);
        d = uu * 0.14;
        k = span(t, d, d + 0.55);
        p = E.inCubic(k);
        // the aperture holds both lines for a moment; keep that beat short by dimming the
        // departing flap well before the arriving one reaches the window
        o = 1 - E.inQuad(span(t, d + 0.06, d + 0.42));
        bl = 5 * Math.sin(Math.PI * k) * k;
        L.outW[i].style.transform = 'translate3d(0,' + (-TR * p).toFixed(2) + 'px,0)';
        L.outW[i].style.opacity = o.toFixed(3);
        L.outW[i].style.filter = mb(bl);
      }

      n = L.inW.length;
      for (i = 0; i < n; i++) {
        uu = u(i, n);
        d = 0.36 + uu * 0.14;
        k = span(t, d, d + 0.50);   // last flap lands exactly at t=1, so t->1 matches the canon
        p = back(k, 0.55);
        o = E.outQuad(span(t, d, d + 0.24));
        bl = 5 * Math.sin(Math.PI * k) * (1 - k);
        L.inW[i].style.transform = 'translate3d(0,' + (TR * (1 - p)).toFixed(2) + 'px,0)';
        L.inW[i].style.opacity = o.toFixed(3);
        L.inW[i].style.filter = mb(bl);
      }
      L.out.style.opacity = '1';
      L.in.style.opacity = '1';
    },
    rest: function (stage) {
      var L = stage._tk;
      clearAll(L.inW);
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
    },
  });

  /* ---- 4. Raked Slide -----------------------------------------------------
     Diagonal travel with a shear that leans into the direction of motion: the old line is
     raked away up and to the right, the new one comes up from the lower left and
     straightens as it lands. The shear is proportional to how far each word still has to go. */
  FX.register({
    id: 'slide-raked',
    name: 'Raked Slide',
    family: 'Slide',
    blurb: 'Sheared diagonal travel: the old line rakes away up-right, the new one lands from below.',
    duration: 1000,
    dwell: 4300,
    setup: function (stage, ctx) { stage._rk = build(stage, ctx, { split: true }); },
    frame: function (stage, t, ctx) {
      var L = stage._rk, W = ctx.W || 600, H = ctx.H || 160;
      var DX = W * 0.30 + 60, DY = H * 0.28 + 26, SK = 17;
      var i, n, uu, d, k, p, o, bl, s;

      // Two rules keep the rake readable, and both were learned the hard way:
      //  - the stagger runs from the TAIL of each half (1 - _wu). Travel is to the right, so
      //    reading-order delays make every leading word overrun the one ahead of it and the
      //    sentence collapses into a blot.
      //  - it is taken per HALF (_wu), not from the flat word list. On the flat index the bold
      //    row leads the light row, rises into it, and the two rows interleave.
      n = L.outW.length;
      for (i = 0; i < n; i++) {
        uu = 1 - L.outW[i]._wu;
        d = uu * 0.20;
        k = span(t, d, d + 0.46);
        p = E.inQuad(k);
        // the old line must be gone before the new one reaches the middle, or the two cross
        // over the same ground and the words print on top of each other
        o = 1 - E.inQuad(span(t, d + 0.02, d + 0.30));
        bl = 6 * Math.sin(Math.PI * k) * k;
        s = 1 - 0.07 * p;
        L.outW[i].style.transform =
          'translate3d(' + (DX * p).toFixed(2) + 'px,' + (-DY * p).toFixed(2) + 'px,0) ' +
          'skewX(' + (-SK * p).toFixed(2) + 'deg) scale(' + s.toFixed(3) + ')';
        L.outW[i].style.opacity = o.toFixed(3);
        L.outW[i].style.filter = mb(bl);
      }

      n = L.inW.length;
      for (i = 0; i < n; i++) {
        uu = 1 - L.inW[i]._wu;
        d = 0.34 + uu * 0.20;
        k = span(t, d, d + 0.46);   // last word lands exactly at t=1
        p = E.outQuart(k);
        o = E.outQuad(span(t, d, d + 0.26));
        bl = 6 * Math.sin(Math.PI * k) * (1 - k);
        s = 1 - 0.07 * (1 - p);
        L.inW[i].style.transform =
          'translate3d(' + (-DX * (1 - p)).toFixed(2) + 'px,' + (DY * (1 - p)).toFixed(2) + 'px,0) ' +
          'skewX(' + (-SK * (1 - p)).toFixed(2) + 'deg) scale(' + s.toFixed(3) + ')';
        L.inW[i].style.opacity = o.toFixed(3);
        L.inW[i].style.filter = mb(bl);
      }
      L.out.style.opacity = '1';
      L.in.style.opacity = '1';
    },
    rest: function (stage) {
      var L = stage._rk;
      clearAll(L.inW);
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
    },
  });

  /* ---- 5. Elastic Whip ----------------------------------------------------
     The old line loads up a little to the right, then whips off left. The new line is thrown
     in from the right, overshoots its mark and rings down through a damped oscillation —
     a per-word delay makes the ring travel along the sentence like a cracked rope. */
  FX.register({
    id: 'slide-elastic',
    name: 'Elastic Whip',
    family: 'Slide',
    blurb: 'The old line loads and whips out; the new one is thrown in, overshoots and rings down.',
    duration: 1450,
    dwell: 4400,
    setup: function (stage, ctx) { stage._ew = build(stage, ctx, { split: true }); },
    frame: function (stage, t, ctx) {
      var L = stage._ew, W = ctx.W || 600, DX = W * 0.62 + 80;
      var i, n, uu, d, k, p, o, bl, x, sx;

      // The load is the whole line leaning back as one — it must NOT be staggered. Per-word
      // delays on a rightward move let each word overrun its neighbour, which welded the first
      // words together ("Amotor,") in the very first frames, right where the canon hands over.
      var load = 0.055 * bump(t, 0, 0.22);

      n = L.outW.length;
      for (i = 0; i < n; i++) {
        uu = u(i, n);
        d = uu * 0.10;
        k = span(t, d, d + 0.55);
        // load to the right, then accelerate away (the whip itself is staggered, and travels
        // left, so reading order is the order that keeps the words apart)
        p = E.inCubic(k) - load;
        o = 1 - E.inQuad(span(t, d + 0.10, d + 0.42));
        bl = 7 * Math.sin(Math.PI * k) * k;
        sx = 1 + 0.12 * Math.sin(Math.PI * k) * k;
        L.outW[i].style.transform = 'translate3d(' + (-DX * p).toFixed(2) + 'px,0,0) scaleX(' + sx.toFixed(3) + ')';
        L.outW[i].style.opacity = o.toFixed(3);
        L.outW[i].style.filter = mb(bl);
      }

      n = L.inW.length;
      for (i = 0; i < n; i++) {
        uu = u(i, n);
        d = 0.26 + uu * 0.12;
        k = span(t, d, d + 0.62);
        // main throw + a damped ring that is exactly zero at both ends
        x = DX * (1 - E.outQuint(k)) - 26 * Math.pow(2, -4.2 * k) * Math.sin(3 * Math.PI * k);
        o = E.outQuad(span(t, d, d + 0.22));
        bl = 7 * Math.sin(Math.PI * k) * (1 - k);
        sx = 1 + 0.12 * Math.sin(Math.PI * k) * (1 - k);
        L.inW[i].style.transform = 'translate3d(' + x.toFixed(2) + 'px,0,0) scaleX(' + sx.toFixed(3) + ')';
        L.inW[i].style.opacity = o.toFixed(3);
        L.inW[i].style.filter = mb(bl);
      }
      L.out.style.opacity = '1';
      L.in.style.opacity = '1';
    },
    rest: function (stage) {
      var L = stage._ew;
      clearAll(L.inW);
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
    },
  });

  /* ---- 6. Drum Carousel ---------------------------------------------------
     Both lines are panels on one shallow drum. The drum turns a single step: the old panel
     swings away to the left and back into depth, the new one rides up out of the right.
     A slight lift at mid-swing keeps the arc from reading as a flat pan. */
  FX.register({
    id: 'slide-carousel',
    name: 'Drum Carousel',
    family: 'Slide',
    blurb: 'Both lines are panels on one shallow drum that turns a single step and settles.',
    duration: 1300,
    dwell: 4500,
    setup: function (stage, ctx) {
      stage._cr = build(stage, ctx, { split: true, perspective: '1000px' });
    },
    frame: function (stage, t, ctx) {
      var L = stage._cr, W = ctx.W || 600, H = ctx.H || 160;
      var R = W * 0.60, A = 46, LIFT = Math.min(26, H * 0.13);

      // The drum must leave the mark the way it arrives at it: gently. Raw outBack has a slope
      // of 3.5 at x=0, so two frames in the panel had already swung 3 degrees and slid 35px —
      // a visible jump away from the canonical rest state. Easing the input fixes both ends.
      var f = function (x) { return back(E.inOutQuad(span(x, 0, 1)), 0.5); };
      var p = f(t);
      var bl = Math.min(5, speed(f, t) * 3.4) * gate(t);

      // outgoing panel: 0deg -> -A, rising slightly at mid-swing
      var yo = -LIFT * Math.sin(Math.PI * clamp(p, 0, 1));
      L.out.style.transform =
        'translateY(' + yo.toFixed(2) + 'px) translateZ(' + (-R).toFixed(1) + 'px) ' +
        'rotateY(' + (-A * p).toFixed(2) + 'deg) translateZ(' + R.toFixed(1) + 'px)';
      L.out.style.opacity = (1 - E.inQuad(span(t, 0.20, 0.66))).toFixed(3);
      L.out.style.filter = mb(bl);

      // incoming panel: +A -> 0 on the same drum
      var q = 1 - p;
      var yi = -LIFT * Math.sin(Math.PI * clamp(q, 0, 1));
      L.in.style.transform =
        'translateY(' + yi.toFixed(2) + 'px) translateZ(' + (-R).toFixed(1) + 'px) ' +
        'rotateY(' + (A * q).toFixed(2) + 'deg) translateZ(' + R.toFixed(1) + 'px)';
      L.in.style.opacity = E.outQuad(span(t, 0.14, 0.70)).toFixed(3);
      L.in.style.filter = mb(bl);
    },
    rest: function (stage) {
      var L = stage._cr;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.transform = 'none';
      L.in.style.filter = 'none';
      L.out.style.filter = 'none';
    },
  });
})();
