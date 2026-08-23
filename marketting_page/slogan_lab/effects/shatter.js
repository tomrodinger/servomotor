/* Shatter family — break apart, then reassemble.
 *
 * Six effects, all PURE FUNCTIONS OF PROGRESS: no timers, no CSS animations, no wall clock.
 * Everything is derived from t, and every pseudo-random quantity comes from hash(index, salt),
 * so frame(i, t) is byte-identical every time it is drawn.
 *
 *   shatter-glass      per-character fracture: launch, spin, gravity; new line flies back together
 *   shatter-slices     the line splits into horizontal strips that blast sideways
 *   shatter-particles  glyphs dissolve into a drifting particle field that re-condenses
 *   shatter-sand       letters erode from the top and pour away; new ones fill from the floor up
 *   shatter-crumble    one wavefront sweeps the line: rubble behind it, new text standing after it
 *   shatter-implode    old line vortexes into a point; new one detonates back out of it
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;

  /* ------------------------------------------------------------------ helpers */

  /* deterministic hash noise in [0,1) — never Math.random() */
  function rnd(i, s) {
    var x = Math.sin((i + 1) * 127.1 + (s + 1) * 311.7) * 43758.5453;
    return x - Math.floor(x);
  }
  function srnd(i, s) { return rnd(i, s) * 2 - 1; }

  /* damped spring settle: 1 at p=0, decays through a small overshoot to exactly 0 at p>=1 */
  function damp(p, freq, decay) {
    if (p >= 1) return 0;
    if (p <= 0) return 1;
    return Math.exp(-decay * p) * Math.cos(freq * p);
  }

  /* Two absolutely-stacked lines (old over new), each split into per-character spans that live
     inside inline-block words, so wrapping never breaks a word. */
  function makeLine(wrap, s) {
    var d = FX.el('div', 'fx-line', {
      position: 'absolute', left: '0', right: '0', textAlign: 'center',
      willChange: 'transform,opacity',
    });
    /* Two ROWS, never a <br>. A <br> sitting between two display:block spans (which the engine
       forces on .fx-light/.fx-bold) generates its own empty line box, so the headline grew a
       third, blank line and the whole block rode ~35px higher than the canonical rest state —
       a hard vertical snap at both ends of every Shatter transition. */
    var a = FX.el('span', 'fx-light', { display: 'block' });
    var ca = FX.charWords(s.light); a.appendChild(ca.frag);
    var b = FX.el('span', 'fx-bold', { display: 'block' });
    var cb = FX.charWords(s.bold); b.appendChild(cb.frag);
    d.appendChild(a); d.appendChild(b);
    wrap.appendChild(d);
    return { el: d, nodes: ca.nodes.concat(cb.nodes) };
  }

  function two(stage, ctx) {
    stage.innerHTML = '';
    var wrap = FX.el('div', 'fx-wrap', {
      position: 'relative', width: '100%', height: '100%',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    });
    var g = { wrap: wrap, out: makeLine(wrap, ctx.from), in: makeLine(wrap, ctx.to), done: false };
    stage.appendChild(wrap);
    stage._g = g;
    return g;
  }

  /* Measure every character once (and again if the tile is resized): offset from its own line's
     centre, horizontal fraction, row index, and a reading-order 0..1 position `u` used by the
     sweeping effects. Transforms are cleared first so the rects are the untransformed layout. */
  function measure(stage) {
    var g = stage._g;
    var w = stage.clientWidth, h = stage.clientHeight;
    if (g.done && g.mw === w && g.mh === h) return g;
    g.mw = w; g.mh = h; g.done = true;
    g.fs = parseFloat(getComputedStyle(stage).fontSize) || 26;

    [g.out, g.in].forEach(function (L) {
      L.nodes.forEach(function (n) { n.style.transform = 'none'; n.style.filter = 'none'; });
      if (L.glyphs) L.glyphs.forEach(function (n) { n.style.transform = 'none'; n.style.filter = 'none'; });
    });
    [g.out, g.in].forEach(function (L) {
      var lr = L.el.getBoundingClientRect();
      var cx = lr.left + lr.width / 2, cy = lr.top + lr.height / 2;
      var tops = [];
      L.m = L.nodes.map(function (n) {
        var r = n.getBoundingClientRect();
        var top = Math.round(r.top - lr.top);
        var ri = -1;
        for (var k = 0; k < tops.length; k++) { if (Math.abs(tops[k] - top) < 6) { ri = k; break; } }
        if (ri < 0) { tops.push(top); ri = tops.length - 1; }
        return {
          dx: r.left + r.width / 2 - cx,
          dy: r.top + r.height / 2 - cy,
          ux: lr.width > 1 ? (r.left + r.width / 2 - lr.left) / lr.width : 0.5,
          w: r.width, h: r.height, row: ri,
        };
      });
      L.rows = Math.max(1, tops.length);
      L.box = { w: lr.width || 1, h: lr.height || 1 };
      L.m.forEach(function (m) {
        m.u = (m.row + m.ux) / L.rows;                       // reading-order sweep parameter
        m.d = clamp(Math.abs(m.dx) / (L.box.w / 2 + 1), 0, 1); // 0 centre .. 1 outer edge
      });
    });
    return g;
  }

  /* Give every character an inner glyph span (so it can fade/blur on its own) plus n absolutely
     positioned debris bits that cost nothing in layout. */
  function bits(L, n, sizeEm, round) {
    L.glyphs = []; L.bits = [];
    for (var i = 0; i < L.nodes.length; i++) {
      var node = L.nodes[i];
      var txt = node.textContent;
      node.textContent = '';
      node.style.position = 'relative';
      var gl = FX.el('span', null, { display: 'inline-block' });
      gl.textContent = txt;
      node.appendChild(gl);
      var arr = [];
      for (var k = 0; k < n; k++) {
        var b = FX.el('span', null, {
          position: 'absolute', left: '50%', top: '50%',
          width: sizeEm + 'em', height: sizeEm + 'em',
          marginLeft: (-sizeEm / 2) + 'em', marginTop: (-sizeEm / 2) + 'em',
          background: 'currentColor', borderRadius: round ? '50%' : '1px',
          opacity: '0', pointerEvents: 'none',
        });
        node.appendChild(b);
        arr.push(b);
      }
      L.glyphs.push(gl); L.bits.push(arr);
    }
  }

  function hide(L) {
    L.nodes.forEach(function (n) { n.style.opacity = '0'; });
  }
  function settle(L) {
    for (var i = 0; i < L.nodes.length; i++) {
      var n = L.nodes[i];
      n.style.opacity = '1'; n.style.transform = 'none'; n.style.filter = 'none';
      n.style.webkitMaskImage = n.style.maskImage = 'none';
      if (L.glyphs) {
        L.glyphs[i].style.opacity = '1';
        L.glyphs[i].style.transform = 'none';
        L.glyphs[i].style.filter = 'none';
        L.glyphs[i].style.webkitMaskImage = L.glyphs[i].style.maskImage = 'none';
      }
      if (L.bits) L.bits[i].forEach(function (b) { b.style.opacity = '0'; });
    }
  }

  /* =========================================================== 1. Glass Fracture ============ */
  FX.register({
    id: 'shatter-glass',
    name: 'Glass Fracture',
    family: 'Shatter',
    blurb: 'Every letter breaks loose, spins and falls; the new line flies back together.',
    duration: 1350,
    dwell: 4200,
    setup: function (stage, ctx) { two(stage, ctx); },
    frame: function (stage, t) {
      var g = measure(stage), fs = g.fs, O = g.out, I = g.in, i, m, n;

      /* outgoing: the fracture starts at the centre of the line and runs outward, each shard
         thrown up a little before gravity takes it. */
      for (i = 0; i < O.nodes.length; i++) {
        n = O.nodes[i]; m = O.m[i];
        var lead = 0.11 * (1 - m.d) + 0.04 * rnd(i, 1);
        var p = span(t, lead, lead + 0.48);
        if (p <= 0) {
          n.style.transform = 'none'; n.style.opacity = '1'; n.style.filter = 'none';
          continue;
        }
        var ax = E.inQuad(p);
        var dir = (m.dx >= 0 ? 1 : -1) * (0.5 + 0.9 * rnd(i, 2));
        var x = dir * (0.35 + 1.5 * m.d) * fs * ax;
        var y = (-0.85 - 1.1 * rnd(i, 3)) * fs * p + 4.6 * fs * p * p;
        /* Spin on the same inQuad clock as the launch. Driven by raw p it was already up to 6.5deg
           at t=0.02 for the shards whose lead hashes near zero — a visible tilt one frame after a
           perfectly upright canonical rest state. */
        var rot = srnd(i, 4) * 165 * ax;
        var sc = 1 - 0.28 * p;
        n.style.transform = 'translate(' + x.toFixed(2) + 'px,' + y.toFixed(2) + 'px) rotate(' +
          rot.toFixed(2) + 'deg) scale(' + sc.toFixed(3) + ')';
        n.style.opacity = String(1 - span(t, lead + 0.11, lead + 0.44));
        n.style.filter = 'blur(' + (2.6 * p).toFixed(2) + 'px)';
      }

      /* incoming: shards converge from the outside in, so the middle of the line locks first. */
      for (i = 0; i < I.nodes.length; i++) {
        n = I.nodes[i]; m = I.m[i];
        var lead2 = 0.34 + 0.22 * m.d + 0.06 * rnd(i, 5);
        var p2 = span(t, lead2, lead2 + 0.36);
        var q = 1 - E.outQuart(p2);
        var sx = srnd(i, 6) * (0.5 + 1.1 * m.d) * fs;
        var sy = (0.8 + 1.2 * rnd(i, 7)) * fs * (rnd(i, 8) > 0.72 ? -0.55 : 1);
        var rot2 = srnd(i, 9) * 95;
        n.style.transform = 'translate(' + (sx * q).toFixed(2) + 'px,' + (sy * q * q).toFixed(2) +
          'px) rotate(' + (rot2 * q).toFixed(2) + 'deg) scale(' + (1 - 0.35 * q).toFixed(3) + ')';
        n.style.opacity = String(span(t, lead2, lead2 + 0.14));
        n.style.filter = q > 0.002 ? 'blur(' + (1.9 * q).toFixed(2) + 'px)' : 'none';
      }
    },
    rest: function (stage) { var g = measure(stage); hide(g.out); settle(g.in); },
  });

  /* =========================================================== 2. Strip Blast =============== */
  var STRIPS = 9;
  FX.register({
    id: 'shatter-slices',
    name: 'Strip Blast',
    family: 'Shatter',
    blurb: 'The line splits into horizontal strips that shear apart and slam back together.',
    duration: 1200,
    dwell: 4200,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'fx-wrap', {
        position: 'relative', width: '100%', height: '100%',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
      });
      function content(s) {
        var f = document.createDocumentFragment();
        var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
        var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
        f.appendChild(a); f.appendChild(b);   // no <br>: it would add a third, empty line box
        return f;
      }
      function line(s) {
        var d = FX.el('div', 'fx-line', {
          position: 'absolute', left: '0', right: '0', textAlign: 'center',
        });
        var ghost = FX.el('div', null, { visibility: 'hidden' });   // holds the height open
        ghost.appendChild(content(s));
        d.appendChild(ghost);
        var strips = [];
        for (var k = 0; k < STRIPS; k++) {
          var st = FX.el('div', null, {
            position: 'absolute', left: '0', right: '0', top: '0', textAlign: 'center',
            willChange: 'transform,opacity',
          });
          st.appendChild(content(s));
          var a = Math.max(0, (k / STRIPS) * 100 - 0.45);
          var b = Math.max(0, ((STRIPS - k - 1) / STRIPS) * 100 - 0.45);
          st.style.webkitClipPath = st.style.clipPath =
            'inset(' + a.toFixed(3) + '% 0 ' + b.toFixed(3) + '% 0)';
          d.appendChild(st);
          strips.push(st);
        }
        wrap.appendChild(d);
        return strips;
      }
      stage.appendChild(wrap);
      stage._s = { out: line(ctx.from), in: line(ctx.to) };
    },
    frame: function (stage, t) {
      var S = stage._s, W = stage.clientWidth || 600, k, st;
      var last = STRIPS - 1;
      var fs = parseFloat(getComputedStyle(stage).fontSize) || 26;

      /* Every strip owns its own horizontal band of the line box, so strips can never overlap
         each other — the whole picture stays one sliced image however far they travel.
         Outgoing: alternating strips fire left and right in a top-down cascade, shearing. */
      for (k = 0; k < STRIPS; k++) {
        st = S.out[k];
        var lead = 0.09 * (k / last);
        var p = span(t, lead, lead + 0.46);
        if (p <= 0) { st.style.transform = 'none'; st.style.opacity = '1'; st.style.filter = 'none'; continue; }
        var dir = (k % 2 ? 1 : -1);
        /* p^1.6 still moved the first band ~13px at t=0.02 — a visible shear the instant the
           transition starts, against a canonical rest state that is perfectly registered.
           A steeper launch keeps the first frames indistinguishable from the canon. */
        var e = Math.pow(p, 2.4);
        var x = dir * (0.55 + 0.50 * rnd(k, 11)) * W * e;
        st.style.transform = 'translateX(' + x.toFixed(2) + 'px) skewX(' +
          (-dir * 15 * Math.pow(p, 1.8)).toFixed(2) + 'deg)';
        st.style.opacity = String(1 - span(t, lead + 0.12, lead + 0.46));
        st.style.filter = 'blur(' + (2 * p).toFixed(2) + 'px)';
      }

      /* Incoming: a counter-cascade — the bottom bands land first, the top last. Travel is kept
         short and the ease is outQuint, so by three-quarters the comb has closed to a hairline
         instead of leaving two half-registered copies of the line on screen. A small opposite
         kick at the end makes each band snap into place rather than glide. */
      for (k = 0; k < STRIPS; k++) {
        st = S.in[k];
        var lead2 = 0.34 + 0.08 * (1 - k / last);
        var p2 = span(t, lead2, lead2 + 0.42);
        var dir2 = (k % 2 ? -1 : 1);
        var q = 1 - E.outQuint(p2);
        var amp = dir2 * (0.26 + 0.09 * rnd(k, 13)) * W;
        /* Snap, not misregistration. The kick was a flat 11px riding a sin(pi*p2^3) envelope, so
           it was still ~6px per band at p2=0.94 — from t~0.6 all the way to t~0.9 the line sat
           essentially assembled with every other band offset by 12px, which reads as a printing
           error rather than as motion. Scale it to the type size and squeeze the envelope into
           the last fifth of the landing: one flick, then clean registration. */
        var kick = -dir2 * 0.05 * fs * Math.sin(Math.PI * Math.pow(p2, 9));
        st.style.transform = 'translateX(' + (amp * q + kick).toFixed(2) + 'px) skewX(' +
          (dir2 * 13 * q).toFixed(2) + 'deg)';
        st.style.opacity = String(span(t, lead2, lead2 + 0.10));
        st.style.filter = q > 0.004 ? 'blur(' + (1.5 * q).toFixed(2) + 'px)' : 'none';
      }
    },
    rest: function (stage) {
      var S = stage._s;
      S.out.forEach(function (st) { st.style.opacity = '0'; });
      S.in.forEach(function (st) { st.style.opacity = '1'; st.style.transform = 'none'; st.style.filter = 'none'; });
    },
  });

  /* =========================================================== 3. Particle Field ============ */
  FX.register({
    id: 'shatter-particles',
    name: 'Particle Field',
    family: 'Shatter',
    blurb: 'Glyphs disperse into a drifting particle cloud that condenses into the new words.',
    duration: 1600,
    dwell: 4600,
    setup: function (stage, ctx) {
      var g = two(stage, ctx);
      bits(g.out, 4, 0.13, true);
      bits(g.in, 4, 0.13, true);
    },
    frame: function (stage, t) {
      var g = measure(stage), fs = g.fs, O = g.out, I = g.in, i, k;

      for (i = 0; i < O.nodes.length; i++) {
        /* glyph crumbles into blur, its particles are flung out on a slow gravity arc */
        var gs = 0.04 + 0.16 * rnd(i, 1);
        var gp = span(t, gs, gs + 0.34);
        var gl = O.glyphs[i];
        gl.style.opacity = String(1 - gp);
        gl.style.filter = gp > 0.002 ? 'blur(' + (4 * gp).toFixed(2) + 'px)' : 'none';
        gl.style.transform = 'scale(' + (1 - 0.12 * gp).toFixed(3) + ')';
        for (k = 0; k < 4; k++) {
          var b = O.bits[i][k];
          var ds = gs + 0.02 + 0.06 * k;
          var dp = span(t, ds, ds + 0.52);
          if (dp <= 0 || dp >= 1) { b.style.opacity = '0'; continue; }
          var ang = rnd(i, 20 + k) * Math.PI * 2;
          var rad = (0.35 + 1.35 * rnd(i, 30 + k)) * fs;
          var ee = E.outCubic(dp);
          var x = Math.cos(ang) * rad * ee;
          var y = Math.sin(ang) * rad * 0.6 * ee + 1.5 * fs * dp * dp;
          b.style.opacity = (Math.sin(Math.PI * dp) * 0.85).toFixed(3);
          b.style.transform = 'translate(' + x.toFixed(2) + 'px,' + y.toFixed(2) +
            'px) scale(' + (1 - 0.55 * dp).toFixed(3) + ')';
        }
      }

      for (i = 0; i < I.nodes.length; i++) {
        /* particles rush in from the field first — the cloud already spells the new line — and
           the glyph resolves out of them at the end */
        /* This used to start at 0.60..0.76 with a 0.24 window, which had two consequences: the
           stage held nothing but an unreadable cloud from t~0.42 to t~0.62, and the slowest
           glyphs only finished resolving AT t=1 — so t=0.98 still sat at 92% opacity with a
           quarter-pixel blur and snapped when the engine swapped in the canon. Pulling the
           resolve forward closes the wordless gap and lands the line well before the edge. */
        var gi0 = 0.46 + 0.13 * rnd(i, 2);
        var gi = span(t, gi0, gi0 + 0.26);
        var gl2 = I.glyphs[i];
        gl2.style.opacity = String(gi);
        gl2.style.filter = gi < 0.998 ? 'blur(' + (3 * (1 - gi)).toFixed(2) + 'px)' : 'none';
        gl2.style.transform = 'scale(' + lerp(0.72, 1, E.outBack(gi)).toFixed(3) + ')';
        for (k = 0; k < 4; k++) {
          var b2 = I.bits[i][k];
          var ds2 = 0.18 + 0.12 * rnd(i, 40 + k) + 0.03 * k;
          var dp2 = span(t, ds2, ds2 + 0.42);
          var vis = Math.min(1, dp2 * 6) * (1 - span(t, gi0, gi0 + 0.18));
          if (vis <= 0.002) { b2.style.opacity = '0'; continue; }
          var ang2 = rnd(i, 50 + k) * Math.PI * 2;
          var rad2 = (0.4 + 1.6 * rnd(i, 60 + k)) * fs;
          var q = 1 - E.outQuart(dp2);
          var x2 = Math.cos(ang2) * rad2 * q;
          var y2 = Math.sin(ang2) * rad2 * 0.65 * q - 1.1 * fs * q * q;
          b2.style.opacity = (vis * 0.9).toFixed(3);
          b2.style.transform = 'translate(' + x2.toFixed(2) + 'px,' + y2.toFixed(2) +
            'px) scale(' + (0.55 + 0.45 * dp2).toFixed(3) + ')';
        }
      }
    },
    rest: function (stage) {
      var g = measure(stage);
      g.out.nodes.forEach(function (n) { n.style.opacity = '0'; });
      g.out.bits.forEach(function (a) { a.forEach(function (b) { b.style.opacity = '0'; }); });
      settle(g.in);
    },
  });

  /* =========================================================== 4. Sand Pour ================= */
  FX.register({
    id: 'shatter-sand',
    name: 'Sand Pour',
    family: 'Shatter',
    blurb: 'Letters erode from the top and pour away; the new ones fill up from the floor.',
    duration: 1700,
    dwell: 4600,
    theme: { bg: '#F7F4EE', fg: '#2A2622', accent: '#C08A3E' },
    setup: function (stage, ctx) {
      var g = two(stage, ctx);
      bits(g.out, 3, 0.085, false);
      bits(g.in, 3, 0.085, false);
    },
    frame: function (stage, t) {
      var g = measure(stage), fs = g.fs, O = g.out, I = g.in, i, k;

      for (i = 0; i < O.nodes.length; i++) {
        /* erosion line sweeps down the glyph, left to right along the line */
        var m = O.m[i];
        var lead = 0.30 * m.u + 0.04 * rnd(i, 1);
        var p = span(t, lead, lead + 0.46);
        var a = -10 + 122 * E.inOutQuad(p);
        var gl = O.glyphs[i];
        gl.style.webkitMaskImage = gl.style.maskImage =
          'linear-gradient(to bottom, rgba(0,0,0,0) ' + (a - 7).toFixed(1) + '%, rgba(0,0,0,1) ' +
          (a + 4).toFixed(1) + '%)';
        gl.style.transform = 'translateY(' + (0.10 * fs * p).toFixed(2) + 'px)';
        gl.style.opacity = String(1 - 0.15 * p);
        for (k = 0; k < 3; k++) {
          var b = O.bits[i][k];
          var ds = lead + 0.03 + 0.16 * rnd(i, 10 + k);
          var dp = span(t, ds, ds + 0.44);
          if (dp <= 0 || dp >= 1) { b.style.opacity = '0'; continue; }
          var x = srnd(i, 20 + k) * 0.30 * fs * dp;
          var y = -0.35 * fs + 0.5 * fs * dp + 3.2 * fs * dp * dp;
          b.style.opacity = (0.9 * Math.min(1, (1 - dp) * 3)).toFixed(3);
          b.style.transform = 'translate(' + x.toFixed(2) + 'px,' + y.toFixed(2) + 'px)';
        }
      }

      for (i = 0; i < I.nodes.length; i++) {
        /* the new glyph builds up from its baseline as grains land on it */
        var m2 = I.m[i];
        var lead2 = 0.30 + 0.32 * m2.u;
        var p2 = span(t, lead2, lead2 + 0.34);
        var bb = -10 + 122 * E.outCubic(p2);
        var gl2 = I.glyphs[i];
        gl2.style.webkitMaskImage = gl2.style.maskImage =
          'linear-gradient(to top, rgba(0,0,0,1) ' + (bb - 4).toFixed(1) + '%, rgba(0,0,0,0) ' +
          (bb + 7).toFixed(1) + '%)';
        gl2.style.opacity = String(Math.min(1, 0.55 + p2));
        gl2.style.transform = 'translateY(' + (0.12 * fs * (1 - E.outCubic(p2))).toFixed(2) + 'px)';
        for (k = 0; k < 3; k++) {
          var b2 = I.bits[i][k];
          var ds2 = lead2 - 0.14 + 0.14 * rnd(i, 30 + k);
          var dp2 = span(t, ds2, ds2 + 0.30);
          if (dp2 <= 0 || dp2 >= 1) { b2.style.opacity = '0'; continue; }
          var x2 = srnd(i, 40 + k) * 0.28 * fs * (1 - dp2);
          var y2 = -2.9 * fs * (1 - dp2) * (1 - dp2) + 0.35 * fs;
          b2.style.opacity = (0.9 * Math.min(1, dp2 * 4) * Math.min(1, (1 - dp2) * 4)).toFixed(3);
          b2.style.transform = 'translate(' + x2.toFixed(2) + 'px,' + y2.toFixed(2) + 'px)';
        }
      }
    },
    rest: function (stage) {
      var g = measure(stage);
      g.out.nodes.forEach(function (n) { n.style.opacity = '0'; });
      g.out.bits.forEach(function (a) { a.forEach(function (b) { b.style.opacity = '0'; }); });
      settle(g.in);
    },
  });

  /* =========================================================== 5. Crumble Sweep ============= */
  FX.register({
    id: 'shatter-crumble',
    name: 'Crumble Sweep',
    family: 'Shatter',
    blurb: 'A wavefront runs the line: letters topple into rubble, new ones stand up behind it.',
    duration: 1500,
    dwell: 4400,
    setup: function (stage, ctx) {
      var g = two(stage, ctx);
      bits(g.out, 2, 0.10, false);
      g.out.nodes.forEach(function (n) { n.style.transformOrigin = '0% 100%'; });
      g.in.nodes.forEach(function (n) { n.style.transformOrigin = '50% 100%'; });
    },
    frame: function (stage, t) {
      var g = measure(stage), fs = g.fs, O = g.out, I = g.in, i, k;

      for (i = 0; i < O.nodes.length; i++) {
        var n = O.nodes[i], m = O.m[i];
        var lead = 0.55 * m.u;
        var p = span(t, lead, lead + 0.40);
        if (p <= 0) {
          n.style.transform = 'none'; n.style.opacity = '1'; n.style.filter = 'none';
          O.bits[i][0].style.opacity = '0'; O.bits[i][1].style.opacity = '0';
          continue;
        }
        var tip = E.inQuad(p);
        var rot = (55 + 55 * rnd(i, 1)) * tip;
        var y = 0.25 * fs * p + 2.8 * fs * p * p * (0.5 + 0.8 * rnd(i, 2));
        var x = 0.35 * fs * srnd(i, 3) * p;
        n.style.transform = 'translate(' + x.toFixed(2) + 'px,' + y.toFixed(2) + 'px) rotate(' +
          rot.toFixed(2) + 'deg)';
        n.style.opacity = String(1 - span(t, lead + 0.14, lead + 0.38));
        n.style.filter = 'blur(' + (1.8 * p).toFixed(2) + 'px)';
        for (k = 0; k < 2; k++) {
          var b = O.bits[i][k];
          var dp = span(t, lead + 0.02 + 0.10 * rnd(i, 10 + k), lead + 0.42);
          if (dp <= 0 || dp >= 1) { b.style.opacity = '0'; continue; }
          var bx = srnd(i, 20 + k) * 0.55 * fs * dp;
          var by = -0.2 * fs * dp + 3.1 * fs * dp * dp;
          b.style.opacity = (0.85 * Math.min(1, (1 - dp) * 2.5)).toFixed(3);
          b.style.transform = 'translate(' + bx.toFixed(2) + 'px,' + by.toFixed(2) + 'px) rotate(' +
            (srnd(i, 30 + k) * 220 * dp).toFixed(1) + 'deg)';
        }
      }

      for (i = 0; i < I.nodes.length; i++) {
        var n2 = I.nodes[i], m2 = I.m[i];
        var lead2 = 0.30 + 0.48 * m2.u;
        var p2 = span(t, lead2, lead2 + 0.22);
        var e = E.outBack(p2);
        var q = 1 - e;
        n2.style.transform = 'translateY(' + (0.75 * fs * q).toFixed(2) + 'px) rotate(' +
          (-16 * q).toFixed(2) + 'deg) scale(' + (1 - 0.10 * q).toFixed(3) + ')';
        n2.style.opacity = String(span(t, lead2, lead2 + 0.15));
        n2.style.filter = p2 < 0.4 ? 'blur(' + (1.4 * (1 - E.outCubic(span(t, lead2, lead2 + 0.16)))).toFixed(2) + 'px)' : 'none';
      }
    },
    rest: function (stage) {
      var g = measure(stage);
      g.out.nodes.forEach(function (n) { n.style.opacity = '0'; });
      g.out.bits.forEach(function (a) { a.forEach(function (b) { b.style.opacity = '0'; }); });
      settle(g.in);
    },
  });

  /* =========================================================== 6. Implosion ================= */
  FX.register({
    id: 'shatter-implode',
    name: 'Implosion',
    family: 'Shatter',
    blurb: 'The old line vortexes down to a single spark; the new one detonates back out of it.',
    duration: 1500,
    dwell: 4400,
    theme: { bg: '#0B0C0F', fg: '#F2F4F7', accent: '#7AB648' },
    setup: function (stage, ctx) {
      var g = two(stage, ctx);
      /* The spark is a garnish, not the subject. It used to be 190px at up to 2.3x with a hot
         core — a 400px green sun that owned the middle of the transition while the text under it
         was down at 10% opacity. Half the size, softer core, and it never outshines the letters. */
      var core = FX.el('div', null, {
        position: 'absolute', left: '50%', top: '50%', width: '110px', height: '110px',
        borderRadius: '50%', pointerEvents: 'none', opacity: '0',
        background: 'radial-gradient(circle, rgba(255,255,255,.85) 0%, rgba(122,182,72,.55) 30%, rgba(122,182,72,0) 70%)',
        mixBlendMode: 'screen', willChange: 'transform,opacity',
      });
      g.wrap.appendChild(core);
      g.core = core;
    },
    frame: function (stage, t) {
      var g = measure(stage), O = g.out, I = g.in, i;

      /* Outgoing: a beat of outward anticipation, then an accelerating vortex into the centre.
         The vortex reads through scale and spin, so the letters hold FULL opacity nearly all the
         way in and only cut out in the last tenth before the singularity. Fading them early left
         the middle of the transition with nothing in it but the spark. */
      /* Squared so the breath starts flat: a bare sine is steepest at t=0, which pushed the outer
         letters ~5px out of the canonical position within the first 2% of the transition. */
      var antic = Math.pow(Math.sin(Math.PI * span(t, 0, 0.20)), 2) * 0.06;
      var col = E.inCubic(span(t, 0.08, 0.55));
      for (i = 0; i < O.nodes.length; i++) {
        var n = O.nodes[i], m = O.m[i];
        var s = (1 + antic) * (1 - col);
        var r = Math.sqrt(m.dx * m.dx + m.dy * m.dy);
        var a0 = Math.atan2(m.dy, m.dx) + 1.15 * col;
        var x = Math.cos(a0) * r * s - m.dx;
        var y = Math.sin(a0) * r * s - m.dy;
        n.style.transform = 'translate(' + x.toFixed(2) + 'px,' + y.toFixed(2) + 'px) rotate(' +
          (srnd(i, 1) * 60 * col).toFixed(2) + 'deg) scale(' + (1 - 0.82 * col).toFixed(3) + ')';
        n.style.opacity = String(1 - span(t, 0.47, 0.58));
        n.style.filter = col > 0.002 ? 'blur(' + (2.2 * col).toFixed(2) + 'px)' : 'none';
      }

      /* the spark at the singularity — brief, and it peaks in the one instant the stage is empty */
      var flash = Math.pow(Math.sin(Math.PI * span(t, 0.42, 0.80)), 2.1) * 0.85;
      g.core.style.opacity = flash.toFixed(3);
      g.core.style.transform = 'translate(-50%,-50%) scale(' +
        lerp(0.18, 1.5, E.outQuart(span(t, 0.44, 0.95))).toFixed(3) + ')';

      /* Incoming: blown outward from the same point, settling on a damped overshoot. It starts
         while the old line is still cutting out, so the two overlap by a few frames and the stage
         is never bare. */
      for (i = 0; i < I.nodes.length; i++) {
        var n2 = I.nodes[i], m2 = I.m[i];
        var p2 = span(t, 0.50, 1);
        var s2 = 1 - damp(p2, 4.6, 5.4);
        var r2 = Math.sqrt(m2.dx * m2.dx + m2.dy * m2.dy);
        var a2 = Math.atan2(m2.dy, m2.dx) - 0.95 * (1 - E.outQuart(p2));
        var x2 = Math.cos(a2) * r2 * s2 - m2.dx;
        var y2 = Math.sin(a2) * r2 * s2 - m2.dy;
        var sc = lerp(0.15, 1, E.outQuart(span(t, 0.50, 0.92)));
        n2.style.transform = 'translate(' + x2.toFixed(2) + 'px,' + y2.toFixed(2) + 'px) rotate(' +
          (srnd(i, 2) * 45 * (1 - E.outQuart(p2))).toFixed(2) + 'deg) scale(' + sc.toFixed(3) + ')';
        n2.style.opacity = String(span(t, 0.50, 0.60));
        n2.style.filter = p2 < 0.9 ? 'blur(' + (2.4 * (1 - E.outQuart(p2))).toFixed(2) + 'px)' : 'none';
      }
    },
    rest: function (stage) {
      var g = measure(stage);
      hide(g.out);
      settle(g.in);
      g.core.style.opacity = '0';
    },
  });
})();
