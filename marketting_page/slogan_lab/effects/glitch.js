/* Glitch family — digital artefacts, kept on a short leash.
 *
 * Six effects, all PURE FUNCTIONS OF PROGRESS: no timers, no CSS animations, no Math.random,
 * no wall clock. Every "random" quantity comes from hash(a,b), and every stepped, jumpy
 * quantity comes from quantising t itself, so frame(i, t) draws the identical picture forever.
 *
 *   glitch-rgb        the line separates into R/G/B passes that change over out of step
 *   glitch-tear       horizontal bands slip sideways and snap back, handing over as they go
 *   glitch-lock       the picture loses vertical hold, rolls two frames, holds, and locks
 *   glitch-mosh       macroblocks drift off their motion vectors and update one region at a time
 *   glitch-crt        the line collapses to a lit scanline and blooms back out as the new one
 *   glitch-blocks     the line coarsens into DCT blocks, re-quantises, and sharpens into the new one
 *
 * THE HOUSE RULE HERE IS: NO DEAD FRAMES. Glitch effects fail by going to noise. At every t the
 * stage holds either legible text, or text plainly in the middle of becoming other text, or —
 * for one of the six, for a couple of frames — a deliberate lit scanline that is itself the
 * picture. Amplitudes are therefore deliberately small, and wherever two lines are on stage at
 * once they are separated (in colour, in a band, in a block, or vertically) rather than
 * superimposed, because two texts printed over each other is exactly the unreadable smear the
 * rule is about. This is a premium product page: the artefacts read as a signal fault, not as
 * a broken page.
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;
  var ACC = '#7AB648';                                     // Gearotons green

  /* ------------------------------------------------------------------ helpers */

  /* deterministic hash noise in [0,1) — never Math.random() */
  function hash(a, b) {
    var x = Math.sin((a + 1) * 127.1 + (b + 1) * 311.7) * 43758.5453;
    return x - Math.floor(x);
  }
  function shash(a, b) { return hash(a, b) * 2 - 1; }

  /* smooth 0 -> 1 -> 0 over [0,1]; p shapes how flat it is at the ends */
  function bump(t, p) {
    var v = Math.sin(Math.PI * clamp(t, 0, 1));
    return p === undefined ? v : Math.pow(v, p);
  }

  /* triangular window: 1 at c, 0 at c +- w. "this thing happens HERE and nowhere else" */
  function near(t, c, w) { return clamp(1 - Math.abs(t - c) / w, 0, 1); }

  /* quantise progress into n discrete frames — this is what makes a jitter look digital
     rather than wobbly, and it stays a pure function of t. */
  function stepq(t, n) { return Math.floor(clamp(t, 0, 0.9999) * n); }

  function snap(v, q) { return Math.round(v / q) * q; }

  /* A stage-filling container. `bg` paints an opaque floor so mix-blend-mode has something
     defined to blend against, and `isolation` keeps that blending inside this box. */
  function wrapper(stage, bg) {
    stage.innerHTML = '';
    var w = FX.el('div', 'gl-wrap', {
      position: 'relative', width: '100%', height: '100%', overflow: 'hidden',
      isolation: 'isolate',
    });
    if (bg) {
      w.appendChild(FX.el('div', null, {
        position: 'absolute', top: '0', left: '0', right: '0', bottom: '0', background: bg,
      }));
    }
    stage.appendChild(w);
    return w;
  }

  /* One copy of a slogan, centred in whatever box `css` describes. Used both for full-stage
     layers and for the little windows that bands and macroblocks look through. `_inner` is the
     text itself, so a caller can move the window and the content independently. */
  function face(s, css) {
    var box = FX.el('div', 'gl-line', {
      position: 'absolute', display: 'flex', alignItems: 'center', justifyContent: 'center',
      willChange: 'transform, opacity',
    });
    if (css) for (var k in css) box.style[k] = css[k];
    var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
    /* Two BLOCK halves and no <br>. The engine forces .fx-light/.fx-bold siblings to block
       after setup(); a <br> between two blocks then becomes an anonymous line box of its own,
       so the layer drew THREE lines where the canon draws two — every transition opened and
       closed with the headline jumping apart and back. Declaring block here also means the
       geometry setup() measures (tear rows, block coverage) is the geometry that renders. */
    var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
    var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
    inner.appendChild(a); inner.appendChild(b);
    box.appendChild(inner);
    box._inner = inner;
    return box;
  }
  var FULL = { top: '0', left: '0', right: '0', bottom: '0' };

  /* Window onto the full-stage line, offset so the text stays put while the window moves. */
  function peep(s, x0, y0, W, H) {
    return face(s, { left: (-x0) + 'px', top: (-y0) + 'px', width: W + 'px', height: H + 'px' });
  }

  function scanlines(w, pitch, alpha) {
    var s = FX.el('div', null, {
      position: 'absolute', top: '0', left: '0', right: '0', bottom: '0', pointerEvents: 'none',
      background: 'repeating-linear-gradient(180deg,'
        + ' rgba(0,0,0,' + alpha + ') 0px, rgba(0,0,0,' + alpha + ') 1px,'
        + ' rgba(0,0,0,0) 1px, rgba(0,0,0,0) ' + pitch + 'px)',
      opacity: '0', mixBlendMode: 'multiply', willChange: 'opacity',
    });
    w.appendChild(s);
    return s;
  }

  function size(stage, ctx) {
    return {
      W: Math.max(120, ctx.W || stage.clientWidth || 380),
      H: Math.max(60, ctx.H || stage.clientHeight || 150),
    };
  }

  /* =============================================================== 1. channel split ======
   * Three coloured copies of the line, screen-blended, so perfectly registered they add back
   * to white — that is the rest state. As they shear apart they change over ONE CHANNEL AT A
   * TIME, red then green then blue, and the shear kicks widest at exactly that moment so the
   * cut happens under cover of the split.
   *
   * The stagger is deliberately tight — 35 milliseconds end to end. A generous, readable
   * stagger was the first thing tried and it is the wrong answer: for a fifth of a second you
   * are looking at two different sentences printed on top of each other in the same place,
   * which is precisely the unreadable smear this family must not produce. Two frames of
   * channel disagreement is what a real desync looks like anyway. A plain white copy
   * crossfades in at the very ends so the resting frames are clean rather than fringed by
   * three-way antialiasing.
   */
  var CHAN = ['#FF3B30', '#2BFF8C', '#4C7DFF'];
  var CHAN_DIR = [-1, 0.28, 1];
  var CHAN_SWAP = [0.465, 0.482, 0.50];

  FX.register({
    id: 'glitch-rgb',
    name: 'Channel Split',
    family: 'Glitch',
    blurb: 'Red, green and blue passes shear apart, change line out of step, and re-register.',
    duration: 1150,
    dwell: 4300,
    theme: { bg: '#0B0C0E', fg: '#F5F6F8' },
    setup: function (stage, ctx) {
      var w = wrapper(stage, '#0B0C0E');
      var G = { ch: [] };
      G.baseA = face(ctx.from, FULL); w.appendChild(G.baseA);
      G.baseB = face(ctx.to, FULL); w.appendChild(G.baseB);
      for (var k = 0; k < 3; k++) {
        var host = FX.el('div', null, {
          position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
          mixBlendMode: 'screen', color: CHAN[k], willChange: 'transform, opacity',
        });
        var a = face(ctx.from, FULL), b = face(ctx.to, FULL);
        host.appendChild(a); host.appendChild(b);
        w.appendChild(host);
        G.ch.push({ host: host, a: a, b: b });
      }
      G.sl = scanlines(w, 3, 0.45);
      stage._g = G;
    },
    frame: function (stage, t) {
      var G = stage._g;
      var env = bump(t, 0.7);
      /* The clean white copy owns the first and last few hundredths outright. The old gate
         (env * 3.4) was already at half strength by t = 0.02, so the very first frame after
         the canon showed a dimmed, three-way-fringed headline — a visible snap in brightness
         and in registration at both ends. Ramp it as a window on t instead. */
      var gate = Math.min(span(t, 0.03, 0.13), span(1 - t, 0.03, 0.13));
      var s = stepq(t, 11);
      var amp = 0.55 + 0.45 * hash(s, 3);         // the shear jumps between frames, digital
      var kick = 1 + 1.2 * near(t, 0.483, 0.10);  // widest exactly where the lines change over
      for (var k = 0; k < 3; k++) {
        var c = G.ch[k];
        var dx = (CHAN_DIR[k] * 9 + shash(k, s) * 4) * env * amp * kick;
        var dy = shash(k + 5, s) * 1.5 * env * amp;
        c.host.style.transform = 'translate(' + dx.toFixed(2) + 'px,' + dy.toFixed(2) + 'px)';
        // each channel dips as it flips, so the cut reads as a flicker rather than a pop
        c.host.style.opacity = String(gate * (1 - 0.45 * near(t, CHAN_SWAP[k], 0.035)));

        var nw = t >= CHAN_SWAP[k];
        c.a.style.opacity = nw ? '0' : '1';
        c.b.style.opacity = nw ? '1' : '0';
      }
      G.baseA.style.opacity = String(t < 0.5 ? 1 - gate : 0);
      G.baseB.style.opacity = String(t < 0.5 ? 0 : 1 - gate);
      G.sl.style.opacity = String(env * 0.4);
    },
    rest: function (stage) {
      var G = stage._g;
      G.baseA.style.opacity = '0';
      G.baseB.style.opacity = '1';
      G.ch.forEach(function (c) {
        c.host.style.opacity = '0';
        c.host.style.transform = 'none';
        c.a.style.opacity = '0';
        c.b.style.opacity = '1';
      });
      G.sl.style.opacity = '0';
    },
  });

  /* =================================================================== 2. scanline tear ==
   * The picture is cut into thin bands, each a window onto its own full copy of the line, so
   * an undisturbed stack is seamless. Two amplitudes are at work:
   *
   *   - a couple of pixels of per-band jitter, all the way through, which is the texture of a
   *     signal that has stopped being sure of itself;
   *   - three big tears, where whole SLABS of the picture slide apart and snap back.
   *
   * Where the tears sit is the whole game. The first version cut wherever the band grid fell
   * and slid the halves apart, which chops every glyph through the waist and leaves a frame
   * that is unreadable in the plain sense of the word — the exact hole this family must not
   * produce. So setup() MEASURES the two lines and puts the tears in the whitespace: above
   * the light line, in the gap between the lines, and below the bold line. A tear then slides
   * whole, intact lines of type against each other, which is both more legible and more like
   * real tearing than shredded glyphs ever were.
   *
   * The handover rides the middle tear: each slab swaps text within a few hundredths of t
   * around 0.5, so for a beat the top line is the new slogan and the bottom line is still the
   * old one. Both are fully legible; nothing is ever superimposed.
   */
  var TEAR_SWAP = [0.474, 0.482, 0.512, 0.520];      // per slab: top gap, line 1, line 2, foot
  var TEAR_D0 = [-0.55, -1, 1, 0.55];                // the big changeover tear
  var TEAR_D1 = [0.70, 0.90, -0.60, -0.90];          // a smaller one before it
  var TEAR_D2 = [-0.80, 0.65, -0.95, 0.70];          // and one after
  FX.register({
    id: 'glitch-tear',
    name: 'Scanline Tear',
    family: 'Glitch',
    blurb: 'The picture tears along the gaps between the lines, slides, and snaps back.',
    duration: 1050,
    dwell: 4200,
    theme: { bg: '#FBFBF9', fg: '#16171A' },
    setup: function (stage, ctx) {
      var d = size(stage, ctx);
      var w = wrapper(stage, ctx.sc.paper);

      /* measure where the type actually is — font, wrapping and stage height all move it */
      var probe = face(ctx.from, FULL);
      probe.style.visibility = 'hidden';
      w.appendChild(probe);
      var base = w.getBoundingClientRect();
      var H = Math.max(40, Math.round(base.height || d.H));
      var rl = probe._inner.firstChild.getBoundingClientRect();
      var rb = probe._inner.lastChild.getBoundingClientRect();
      w.removeChild(probe);

      function row(y) { return clamp(Math.round(y), 4, H - 4); }
      var cuts = [
        row(rl.top - base.top - 5),                                  // above the light line
        row(((rl.bottom - base.top) + (rb.top - base.top)) / 2),     // between the two lines
        row(rb.bottom - base.top + 5),                               // below the bold line
      ];

      /* band grid, with the three tear rows forced to be band boundaries so no band ever
         straddles a cut and gets dragged in two directions at once */
      var n = clamp(Math.round(H / 14), 8, 30);
      var edges = [];
      for (var k = 0; k <= n; k++) edges.push(Math.round(k * H / n));
      edges = edges.concat(cuts).sort(function (p, q) { return p - q; });

      stage._b = [];
      for (var e = 1; e < edges.length; e++) {
        var y0 = edges[e - 1], y1 = edges[e];
        if (y1 - y0 < 2) continue;
        var band = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0',
          top: y0 + 'px', height: (y1 - y0) + 'px', overflow: 'hidden',
          willChange: 'transform',
        });
        var fa = face(ctx.from, { left: '0', right: '0', top: (-y0) + 'px', height: H + 'px' });
        var fb = face(ctx.to, { left: '0', right: '0', top: (-y0) + 'px', height: H + 'px' });
        var isRow = cuts.indexOf(y0) >= 0;
        var seam = FX.el('div', null, {
          position: 'absolute', left: '9%', right: '9%', top: '0', height: '1px',
          background: ACC, opacity: '0', pointerEvents: 'none',
        });
        band.appendChild(fa); band.appendChild(fb);
        if (isRow) band.appendChild(seam);
        w.appendChild(band);
        var mid = (y0 + y1) / 2;
        stage._b.push({
          el: band, a: fa, b: fb, seam: isRow ? seam : null,
          slab: (mid > cuts[0] ? 1 : 0) + (mid > cuts[1] ? 1 : 0) + (mid > cuts[2] ? 1 : 0),
          k: stage._b.length,
        });
      }
      stage._reach = clamp(d.W * 0.028, 13, 34);
      stage._sl = scanlines(w, 3, 0.16);
    },
    frame: function (stage, t) {
      var B = stage._b, n = B.length;
      var reach = stage._reach;
      var ends = Math.min(span(t, 0, 0.05), span(1 - t, 0, 0.06));   // pristine at both rests
      var s = stepq(t, 12);

      // three tear events; each slides the four slabs by its own set of directions
      var e0 = Math.pow(near(t, 0.50, 0.11), 0.55);
      var e1 = Math.pow(near(t, 0.22, 0.07), 0.55) * 0.38;
      var e2 = Math.pow(near(t, 0.78, 0.07), 0.55) * 0.33;

      for (var i = 0; i < n; i++) {
        var b = B[i], sl = b.slab;
        var big = (TEAR_D0[sl] * e0 + TEAR_D1[sl] * e1 + TEAR_D2[sl] * e2) * reach;
        /* A few pixels of jitter, but CORRELATED down the stack rather than independent per
           band: neighbouring bands take almost the same offset, so a glyph shears like a
           picture on a wobbly signal instead of being minced into unrelated slivers. The
           quantised frame counter `s` is what keeps it digital. */
        var micro = Math.sin(b.k * 0.62 + s * 2.3) * 3.4 * ends;
        b.el.style.transform = 'translateX(' + snap(big + micro, 2) + 'px)';
        var nw = t >= TEAR_SWAP[sl];
        b.a.style.opacity = nw ? '0' : '1';
        b.b.style.opacity = nw ? '1' : '0';
        if (b.seam) b.seam.style.opacity = String(clamp(Math.abs(big) / (reach * 0.5), 0, 1) * 0.55);
      }
      stage._sl.style.opacity = String(ends * bump(t, 0.6) * 0.35);
    },
    rest: function (stage) {
      stage._b.forEach(function (b) {
        b.el.style.transform = 'none';
        b.a.style.opacity = '0'; b.b.style.opacity = '1';
        if (b.seam) b.seam.style.opacity = '0';
      });
      stage._sl.style.opacity = '0';
    },
  });

  /* ==================================================================== 3. signal lock ===
   * Vertical hold, lost and regained. Four stage-height frames are stacked in one column —
   * old, old, new, new — and the column rolls up through them: two frames in one burst, a
   * held beat on the new line so there is always a clean readable frame in the middle of the
   * transition, then one last slip that decelerates into the lock. Lit tear bars sit at every
   * frame join and are faded by how far the roll is from a whole frame, so they blaze while
   * the picture is slipping and are invisible the instant it is steady — including at t=0 and
   * t=1, where a bar parked on the stage edge would otherwise read as a stray rule.
   */
  FX.register({
    id: 'glitch-lock',
    name: 'Signal Lock',
    family: 'Glitch',
    blurb: 'The picture loses vertical hold, rolls, holds a beat, and locks on the new line.',
    duration: 1500,
    dwell: 4400,
    theme: { bg: '#0A0B0D', fg: '#EFF1F3' },
    setup: function (stage, ctx) {
      var d = size(stage, ctx);
      var w = wrapper(stage, '#0A0B0D');
      var col = FX.el('div', null, {
        position: 'absolute', left: '0', right: '0', top: '0', height: (4 * d.H) + 'px',
        willChange: 'transform',
      });
      var seams = [];
      for (var j = 0; j < 4; j++) {
        col.appendChild(face(j < 2 ? ctx.from : ctx.to, {
          left: '0', right: '0', top: (j * d.H) + 'px', height: d.H + 'px',
        }));
        if (j > 0) {
          var bar = FX.el('div', null, {
            position: 'absolute', left: '0', right: '0',
            top: (j * d.H - 1) + 'px', height: '3px', opacity: '0',
            background: 'linear-gradient(180deg, rgba(122,182,72,.15), #CFEFA6 45%, rgba(122,182,72,.15))',
            boxShadow: '0 0 14px rgba(122,182,72,.7)', willChange: 'opacity',
          });
          col.appendChild(bar);
          seams.push(bar);
        }
      }
      w.appendChild(col);
      stage._g = { col: col, seams: seams, H: d.H, sl: scanlines(w, 3, 0.45) };
    },
    frame: function (stage, t) {
      var G = stage._g, H = G.H;
      /* Three shoves with a held beat after each, the last one decelerating into the lock.
         The holds are not decoration: a picture caught halfway between two frames is the one
         state where the stage is mostly empty, so the roll spends as little time there as it
         can and parks on whole, readable frames in between. */
      var pos = E.inOutCubic(span(t, 0.08, 0.24))
              + E.inOutCubic(span(t, 0.32, 0.46))
              + E.outCubic(span(t, 0.60, 0.92));

      var envJ = Math.min(span(t, 0, 0.05), span(1 - t, 0, 0.10));
      var s = stepq(t, 18);
      var jy = shash(1, s) * 0.04 * H * envJ;
      var jx = snap(shash(2, s) * 5 * envJ, 2);
      var sx = 1 + (hash(3, s) > 0.72 ? 0.02 : 0) * envJ;

      G.col.style.transform =
        'translate(' + jx + 'px,' + (-pos * H + jy).toFixed(2) + 'px) scaleX(' + sx.toFixed(4) + ')';

      var slip = clamp(Math.abs(pos - Math.round(pos)) * 5, 0, 1);
      for (var i = 0; i < G.seams.length; i++) G.seams[i].style.opacity = String(slip);
      G.sl.style.opacity = String(envJ * 0.4);
    },
    rest: function (stage) {
      var G = stage._g;
      G.col.style.transform = 'translateY(' + (-3 * G.H) + 'px)';
      G.seams.forEach(function (b) { b.style.opacity = '0'; });
      G.sl.style.opacity = '0';
    },
  });

  /* ======================================================================= 4. datamosh ===
   * Every macroblock is a window onto its own copy of the line, so an untouched grid is a
   * perfect picture. Each block updates at its own moment, sweeping roughly left to right,
   * and drifts a few pixels off its motion vector as it does — old pixels persisting beside
   * new ones is exactly what a dropped keyframe looks like. A fifth of the blocks are stuck
   * and update late. Nothing is ever empty: every block always holds one line or the other,
   * so a mid-transition frame is a mosaic of two readable texts rather than a hole.
   */
  FX.register({
    id: 'glitch-mosh',
    name: 'Datamosh',
    family: 'Glitch',
    blurb: 'Macroblocks drift off their motion vectors and update region by region.',
    duration: 1200,
    dwell: 4300,
    theme: { bg: '#F2F2EF', fg: '#191A1D' },
    setup: function (stage, ctx) {
      var d = size(stage, ctx);
      var w = wrapper(stage, ctx.sc.paper);
      var cols = clamp(Math.round(d.W / 68), 5, 13);
      var rows = clamp(Math.round(d.H / 42), 3, 8);
      stage._m = [];
      for (var r = 0; r < rows; r++) {
        for (var c = 0; c < cols; c++) {
          var x0 = Math.round(c * d.W / cols), x1 = Math.round((c + 1) * d.W / cols);
          var y0 = Math.round(r * d.H / rows), y1 = Math.round((r + 1) * d.H / rows);
          var blk = FX.el('div', null, {
            position: 'absolute', left: x0 + 'px', top: y0 + 'px',
            width: (x1 - x0) + 'px', height: (y1 - y0) + 'px',
            overflow: 'hidden', willChange: 'transform',
          });
          var fa = peep(ctx.from, x0, y0, d.W, d.H);
          var fb = peep(ctx.to, x0, y0, d.W, d.H);
          blk.appendChild(fa); blk.appendChild(fb);
          w.appendChild(blk);
          var id = r * cols + c;
          var f = (c / Math.max(1, cols - 1)) * 0.62
                + (r / Math.max(1, rows - 1)) * 0.22
                + hash(id, 1) * 0.16;
          var sw = 0.22 + f * 0.42;
          if (hash(id, 7) > 0.82) sw = 0.68 + hash(id, 8) * 0.10;   // stuck blocks
          stage._m.push({
            el: blk, a: fa, b: fb, sw: sw,
            vx: shash(id, 2), vy: shash(id, 3) * 0.35,
          });
        }
      }
    },
    frame: function (stage, t) {
      var M = stage._m;
      for (var i = 0; i < M.length; i++) {
        var m = M[i];
        var v = Math.pow(near(t, m.sw, 0.2), 0.7);        // drift only around its own update
        m.el.style.transform = v > 0
          ? 'translate(' + snap(m.vx * 13 * v, 3) + 'px,' + snap(m.vy * 13 * v, 3) + 'px)'
          : 'none';
        var nw = t >= m.sw;
        m.a.style.opacity = nw ? '0' : '1';
        m.b.style.opacity = nw ? '1' : '0';
      }
    },
    rest: function (stage) {
      stage._m.forEach(function (m) {
        m.el.style.transform = 'none';
        m.a.style.opacity = '0'; m.b.style.opacity = '1';
      });
    },
  });

  /* ============================================================================ 5. CRT ===
   * Power cycle. The old line is squeezed down to a lit scanline and the new one blooms back
   * out of it, overshooting once before it settles. The short window where no text is on
   * stage is not empty: it is the beam itself, full width, blown out and glowing, which is
   * the one place in this family where covering the text completely is the point. The beam is
   * gated tightly around the changeover so the frames on either side still hold a squashed
   * but readable line, and the phosphor brightness decays as the picture re-forms.
   */
  FX.register({
    id: 'glitch-crt',
    name: 'CRT Collapse',
    family: 'Glitch',
    blurb: 'The line collapses to a lit scanline and blooms back out as the next one.',
    duration: 1250,
    dwell: 4400,
    theme: { bg: '#07080A', fg: '#EDEFF2' },
    setup: function (stage, ctx) {
      var w = wrapper(stage, '#07080A');
      var a = face(ctx.from, FULL), b = face(ctx.to, FULL);
      a._inner.style.willChange = b._inner.style.willChange = 'transform, filter';
      w.appendChild(a); w.appendChild(b);
      var glow = FX.el('div', null, {
        position: 'absolute', top: '0', left: '0', right: '0', bottom: '0', pointerEvents: 'none',
        background: 'radial-gradient(110% 22% at 50% 50%, rgba(190,228,255,.30), rgba(0,0,0,0) 70%)',
        opacity: '0', willChange: 'opacity',
      });
      var beam = FX.el('div', null, {
        position: 'absolute', left: '-3%', right: '-3%', top: '50%', height: '3px',
        marginTop: '-1.5px', borderRadius: '2px',
        background: 'linear-gradient(90deg, rgba(255,255,255,0) 0%, rgba(255,255,255,.4) 9%,'
          + ' #FFFFFF 34%, #FFFFFF 66%, rgba(255,255,255,.4) 91%, rgba(255,255,255,0) 100%)',
        boxShadow: '0 0 16px rgba(206,239,166,.8), 0 0 44px rgba(122,182,72,.35)',
        opacity: '0', pointerEvents: 'none', willChange: 'transform, opacity',
      });
      w.appendChild(glow); w.appendChild(beam);
      stage._g = { a: a, b: b, beam: beam, glow: glow, sl: scanlines(w, 3, 0.5) };
    },
    frame: function (stage, t) {
      var G = stage._g;

      var o = E.inCubic(span(t, 0.06, 0.34));                 // old line collapsing
      G.a.style.opacity = String(1 - span(t, 0.26, 0.34));
      G.a._inner.style.transform =
        'scale(' + (1 + 0.14 * o).toFixed(3) + ',' + (1 - 0.99 * o).toFixed(4) + ')';
      G.a._inner.style.filter = 'brightness(' + (1 + 2.2 * o).toFixed(2) + ')'
        + ' blur(' + (0.8 * o).toFixed(2) + 'px)';

      var n = E.outBack(span(t, 0.36, 0.90));                 // new line blooming back
      G.b.style.opacity = String(span(t, 0.36, 0.42));
      G.b._inner.style.transform =
        'scale(' + (1 + 0.12 * (1 - n)).toFixed(3) + ',' + (0.02 + 0.98 * n).toFixed(4) + ')';
      var hot = 1 - E.outCubic(span(t, 0.36, 0.82));
      G.b._inner.style.filter = 'brightness(' + (1 + 1.7 * hot).toFixed(2) + ')'
        + ' blur(' + (1 * hot).toFixed(2) + 'px)';

      var beam = Math.pow(near(t, 0.36, 0.17), 0.65);
      G.beam.style.opacity = String(beam);
      G.beam.style.transform = 'scaleY(' + (0.5 + 1.6 * beam).toFixed(3) + ')';
      G.glow.style.opacity = String(beam * 0.85);
      G.sl.style.opacity = String(bump(t, 0.7) * 0.35);
    },
    rest: function (stage) {
      var G = stage._g;
      G.a.style.opacity = '0';
      G.b.style.opacity = '1';
      G.b._inner.style.transform = 'none';
      G.b._inner.style.filter = 'none';
      G.beam.style.opacity = '0';
      G.glow.style.opacity = '0';
      G.sl.style.opacity = '0';
    },
  });

  /* ================================================================== 6. DCT blocking ====
   * A progressive-JPEG teardown and rebuild, measured for real. setup() rasterises both lines
   * once to an offscreen canvas — same font, same measured glyph positions as the live DOM —
   * and integrates the coverage into three block pyramids, coarse, medium and fine, so every
   * cell knows how much ink the old line puts in it and how much the new one does.
   *
   * Measuring real ink matters. Using each character's layout box instead turns any word into
   * one solid bar, and the whole effect collapses into featureless grey slabs — an unreadable
   * mid-transition, which is the one thing this family is not allowed to produce. With true
   * coverage the counters, the gaps between words and the descenders all survive quantisation,
   * so even the coarsest frame is recognisably a line of type.
   *
   * frame() then walks a single "detail" scalar from sharp text down to coarse blocks and back
   * up, crossfading the pyramid levels, while a separate morph slider lerps every cell from the
   * old coverage to the new one. Alphas are posterised, because banding is what makes it read
   * as quantisation rather than as a blur.
   */
  FX.register({
    id: 'glitch-blocks',
    name: 'Compression Bloom',
    family: 'Glitch',
    blurb: 'The line coarsens into DCT blocks, re-quantises, and sharpens back as the new one.',
    duration: 1400,
    dwell: 4500,
    theme: { bg: '#FAFAF7', fg: '#17181B' },
    setup: function (stage, ctx) {
      var d = size(stage, ctx);
      var w = wrapper(stage, ctx.sc.paper);

      /* the two sharp lines, built per character so we can measure real glyph positions;
         charWords keeps each word an inline-block, so nothing ever breaks mid-word */
      function sharp(s) {
        var box = FX.el('div', 'gl-line', {
          position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          willChange: 'opacity, filter',
        });
        var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
        // block halves, no <br> — see face(): the raster below measures these boxes, so the
        // layout it samples has to be the final one, not a pre-enforcement one
        var lt = FX.el('span', 'fx-light', { display: 'block' }), ca = FX.charWords(s.light);
        lt.appendChild(ca.frag);
        var bd = FX.el('span', 'fx-bold', { display: 'block' }), cb = FX.charWords(s.bold);
        bd.appendChild(cb.frag);
        inner.appendChild(lt); inner.appendChild(bd);
        box.appendChild(inner); w.appendChild(box);
        return { box: box, groups: [{ host: lt, nodes: ca.nodes }, { host: bd, nodes: cb.nodes }] };
      }
      var A = sharp(ctx.from), B = sharp(ctx.to);

      var base = w.getBoundingClientRect();
      var Wp = Math.max(16, Math.round(base.width || d.W));
      var Hp = Math.max(16, Math.round(base.height || d.H));
      var SC = 0.5;                                          // blocks are coarse; half res is plenty
      var cvW = Math.max(8, Math.round(Wp * SC)), cvH = Math.max(8, Math.round(Hp * SC));

      /* draw one line into an offscreen canvas at its real on-screen glyph positions */
      function raster(groups) {
        var cv = document.createElement('canvas');
        cv.width = cvW; cv.height = cvH;
        var g = cv.getContext('2d');
        if (!g) return null;
        g.textBaseline = 'top';
        g.fillStyle = '#000';
        for (var gi = 0; gi < groups.length; gi++) {
          var grp = groups[gi];
          var cs = window.getComputedStyle(grp.host);
          var fs = parseFloat(cs.fontSize) || 26;
          g.font = (cs.fontStyle || 'normal') + ' ' + (cs.fontWeight || '400') + ' '
                 + (fs * SC) + 'px ' + cs.fontFamily;
          for (var i = 0; i < grp.nodes.length; i++) {
            var n = grp.nodes[i];
            if (!n.textContent || !n.textContent.trim()) continue;
            var r = n.getBoundingClientRect();
            if (!r.width) continue;
            // the span is a line box; the em box sits inside it with half-leading above
            g.fillText(n.textContent,
              (r.left - base.left) * SC,
              (r.top - base.top + (r.height - fs) / 2) * SC);
          }
        }
        try { return g.getImageData(0, 0, cvW, cvH).data; } catch (e) { return null; }
      }
      var pxA = raster(A.groups), pxB = raster(B.groups);

      function cover(px, cols, rows) {
        var cov = [], cnt = [], i;
        for (i = 0; i < cols * rows; i++) { cov.push(0); cnt.push(0); }
        if (!px) return cov;
        for (var y = 0; y < cvH; y++) {
          var rr = Math.min(rows - 1, Math.floor(y / cvH * rows));
          for (var x = 0; x < cvW; x++) {
            var idx = rr * cols + Math.min(cols - 1, Math.floor(x / cvW * cols));
            cov[idx] += px[(y * cvW + x) * 4 + 3];
            cnt[idx]++;
          }
        }
        for (i = 0; i < cov.length; i++) cov[i] = cnt[i] ? cov[i] / (cnt[i] * 255) : 0;
        return cov;
      }

      var levels = [];
      [Hp / 8, Hp / 13, Hp / 20].forEach(function (cell) {
        var cols = Math.max(2, Math.round(Wp / cell)), rows = Math.max(2, Math.round(Hp / cell));
        var cw = Wp / cols, ch = Hp / rows;
        var ca = cover(pxA, cols, rows), cb = cover(pxB, cols, rows);
        var host = FX.el('div', null, {
          position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
          opacity: '0', pointerEvents: 'none', willChange: 'opacity',
        });
        var cells = [];
        for (var r = 0; r < rows; r++) {
          for (var c = 0; c < cols; c++) {
            var i = r * cols + c;
            if (ca[i] < 0.012 && cb[i] < 0.012) continue;     // blank paper stays blank
            var box = FX.el('div', null, {
              position: 'absolute',
              left: (c * cw).toFixed(2) + 'px', top: (r * ch).toFixed(2) + 'px',
              width: (cw + 0.5).toFixed(2) + 'px', height: (ch + 0.5).toFixed(2) + 'px',
              background: 'currentColor', opacity: '0', willChange: 'opacity',
            });
            host.appendChild(box);
            cells.push({ el: box, a: ca[i], b: cb[i], last: -1 });
          }
        }
        w.appendChild(host);
        levels.push({ host: host, cells: cells });
      });

      stage._g = { A: A.box, B: B.box, L: levels };
    },
    frame: function (stage, t) {
      var G = stage._g;
      // 3 = sharp text, 2 = fine blocks, 1 = medium, 0 = coarse. One trip down and back.
      var D = lerp(3, 0.8, bump(t, 1.4));
      var m = E.inOutCubic(span(t, 0.32, 0.68));            // old coverage -> new coverage
      var txt = clamp(D - 2, 0, 1);

      var early = t < 0.5;
      G.A.style.opacity = String(early ? txt : 0);
      G.B.style.opacity = String(early ? 0 : txt);
      var soft = (1 - txt) * 1.6;
      G.A.style.filter = early ? 'blur(' + soft.toFixed(2) + 'px)' : 'none';
      G.B.style.filter = early ? 'none' : 'blur(' + soft.toFixed(2) + 'px)';

      for (var j = 0; j < 3; j++) {
        var lv = G.L[j];
        var o = clamp(1 - Math.abs(D - j), 0, 1);
        lv.host.style.opacity = String(o);
        if (o <= 0) continue;                               // invisible: don't touch its cells
        for (var i = 0; i < lv.cells.length; i++) {
          var c = lv.cells[i];
          var v = clamp(lerp(c.a, c.b, m) * 2.35, 0, 1);
          v = Math.round(Math.pow(v, 0.8) * 6) / 6;         // quantised, so it bands
          if (v !== c.last) { c.last = v; c.el.style.opacity = String(v); }
        }
      }
    },
    rest: function (stage) {
      var G = stage._g;
      G.A.style.opacity = '0';
      G.B.style.opacity = '1';
      G.A.style.filter = G.B.style.filter = 'none';
      G.L.forEach(function (lv) { lv.host.style.opacity = '0'; });
    },
  });
})();
