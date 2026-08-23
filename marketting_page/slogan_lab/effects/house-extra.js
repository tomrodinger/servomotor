/* House effects — Light, Print, Glitch and Kinetic families.
 *
 * Written directly rather than by a subagent. Same contract as everything else: each effect is a
 * pure function of progress t, so it can be seeked, screenshotted and exported to GIF. Any
 * "randomness" is derived from a hash of the element index, never Math.random, so the same (i,t)
 * always renders the identical frame.
 *
 * ids are prefixed `hx-` so they cannot collide with the agent-authored family files.
 */
(function () {
  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;

  /* deterministic pseudo-random in [0,1) from two integers */
  function rnd(a, b) {
    var x = Math.sin(a * 127.1 + b * 311.7) * 43758.5453;
    return x - Math.floor(x);
  }

  /* Two absolutely-positioned lines, old over new, neither affecting the other's layout. */
  function twoLayer(stage, ctx) {
    stage.innerHTML = '';
    var wrap = FX.el('div', 'fx-wrap', {
      position: 'relative', width: '100%', height: '100%',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    });
    function line(s) {
      var d = FX.el('div', 'fx-line', {
        position: 'absolute', left: '0', right: '0', textAlign: 'center',
        willChange: 'transform,opacity,filter',
      });
      var a = FX.el('span', 'fx-light'); a.textContent = s.light;
      var br = FX.el('br');
      var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
      d.appendChild(a); d.appendChild(br); d.appendChild(b);
      wrap.appendChild(d);
      return d;
    }
    var out = line(ctx.from), inn = line(ctx.to);
    stage.appendChild(wrap);
    return { out: out, in: inn, wrap: wrap };
  }

  /* Per-word layers for both lines, returned as flat node arrays for staggering. */
  function wordLayers(stage, ctx) {
    stage.innerHTML = '';
    var wrap = FX.el('div', 'fx-wrap', {
      position: 'relative', width: '100%', height: '100%',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    });
    function line(s) {
      var d = FX.el('div', 'fx-line', {
        position: 'absolute', left: '0', right: '0', textAlign: 'center',
      });
      var l = FX.el('span', 'fx-light');
      var lw = FX.words(s.light); l.appendChild(lw.frag);
      var b = FX.el('span', 'fx-bold');
      var bw = FX.words(s.bold); b.appendChild(bw.frag);
      d.appendChild(l); d.appendChild(FX.el('br')); d.appendChild(b);
      wrap.appendChild(d);
      return { el: d, nodes: lw.nodes.concat(bw.nodes) };
    }
    var o = line(ctx.from), i = line(ctx.to);
    stage.appendChild(wrap);
    return { out: o, in: i };
  }

  /* ================================================================= LIGHT */

  FX.register({
    id: 'hx-spotlight', name: 'Spotlight Pan', family: 'Light',
    blurb: 'A spotlight sweeps the dark; the new line is lit as it passes.',
    duration: 1500,
    theme: { bg: '#0b0c0d', fg: '#f3f3f5', accent: '#8FD05A' },
    setup: function (stage, ctx) { stage._l = twoLayer(stage, ctx); },
    frame: function (stage, t) {
      var L = stage._l;
      var x = E.inOutCubic(t) * 140 - 20;           // beam centre, -20%..120%
      var beam = function (pct) {
        return 'radial-gradient(60% 200% at ' + pct + '% 50%, #000 0%, #000 18%, ' +
               'rgba(0,0,0,.35) 34%, transparent 52%)';
      };
      // old line is lit everywhere the beam has NOT yet reached
      L.out.style.webkitMaskImage = L.out.style.maskImage =
        'linear-gradient(to right, #000 ' + clamp(x - 14, 0, 100) + '%, transparent ' + clamp(x + 6, 0, 100) + '%)';
      L.out.style.opacity = '1';
      // new line only exists inside the beam, and stays lit behind it
      L.in.style.webkitMaskImage = L.in.style.maskImage =
        'linear-gradient(to right, transparent ' + clamp(x - 26, -30, 130) + '%, #000 ' + clamp(x - 6, -30, 130) + '%)';
      L.in.style.opacity = '1';
      L.in.style.filter = 'brightness(' + lerp(1.5, 1, E.outCubic(t)) + ')';
      L.wrap.style.backgroundImage = t > 0 && t < 1 ? beam(x) : 'none';
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.filter = 'none';
      L.in.style.webkitMaskImage = L.in.style.maskImage = 'none';
      L.wrap.style.backgroundImage = 'none';
    },
  });

  FX.register({
    id: 'hx-neon-flicker', name: 'Neon Flicker', family: 'Light',
    blurb: 'The old line stutters out like a failing tube; the new one strikes on.',
    duration: 1400,
    theme: { bg: '#0a0b0e', fg: '#eaf3ff', accent: '#8FD05A' },
    setup: function (stage, ctx) { stage._l = twoLayer(stage, ctx); },
    frame: function (stage, t) {
      var L = stage._l;
      // deterministic stutter: quantise t, hash the step. Real tubes flicker unevenly.
      function stutter(p, seed) {
        var step = Math.floor(p * 14);
        var r = rnd(step, seed);
        return r < 0.34 ? 0.15 + r : 1;
      }
      var a = span(t, 0, .46), b = span(t, .4, 1);
      var glow = function (v) {
        return 'drop-shadow(0 0 ' + (7 * v) + 'px rgba(160,220,255,.85)) ' +
               'drop-shadow(0 0 ' + (20 * v) + 'px rgba(122,182,72,.45))';
      };
      var oa = a >= 1 ? 0 : (1 - a * .5) * stutter(a, 3);
      L.out.style.opacity = String(oa);
      L.out.style.filter = glow(oa * .9);
      var ib = b <= 0 ? 0 : b * stutter(b, 11);
      L.in.style.opacity = String(ib);
      L.in.style.filter = glow(ib);
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.filter = 'drop-shadow(0 0 7px rgba(160,220,255,.85)) drop-shadow(0 0 20px rgba(122,182,72,.45))';
    },
  });

  FX.register({
    id: 'hx-bloom', name: 'Overexpose Bloom', family: 'Light',
    blurb: 'The line blows out to white, then the new one resolves out of the glare.',
    duration: 1250,
    setup: function (stage, ctx) { stage._l = twoLayer(stage, ctx); },
    frame: function (stage, t) {
      var L = stage._l;
      var up = E.inQuad(span(t, 0, .48));
      var dn = E.outQuart(span(t, .42, 1));
      L.out.style.opacity = String(1 - span(t, .40, .60));
      L.out.style.filter = 'brightness(' + (1 + 1.9 * up) + ') blur(' + (3.4 * up) + 'px) contrast(' + (1 - .3 * up) + ')';
      L.out.style.transform = 'scale(' + (1 + .035 * up) + ')';
      L.in.style.opacity = String(span(t, .34, .56));
      L.in.style.filter = 'brightness(' + (1 + 1.9 * (1 - dn)) + ') blur(' + (3.4 * (1 - dn)) + 'px)';
      L.in.style.transform = 'scale(' + (1 + .035 * (1 - dn)) + ')';
    },
  });

  /* ================================================================= PRINT */

  FX.register({
    id: 'hx-letterpress', name: 'Letterpress', family: 'Print',
    blurb: 'The plate presses in, lifts, and the new line is left impressed in the paper.',
    duration: 1250,
    theme: { bg: '#F4F1EA', fg: '#22201C', font: 'Charter, Georgia, "Iowan Old Style", serif' },
    setup: function (stage, ctx) { stage._l = twoLayer(stage, ctx); },
    frame: function (stage, t) {
      var L = stage._l;
      var press = Math.sin(Math.PI * clamp(t, 0, 1));       // 0 -> 1 -> 0, the plate travel
      var swap = t < .5 ? 0 : 1;
      var deboss = function (v) {
        return 'drop-shadow(0 ' + (1.2 * v) + 'px 0 rgba(255,255,255,.9)) ' +
               'drop-shadow(0 -' + (1.1 * v) + 'px ' + (1.6 * v) + 'px rgba(0,0,0,.42))';
      };
      L.out.style.opacity = swap ? '0' : '1';
      L.in.style.opacity = swap ? '1' : '0';
      var live = swap ? L.in : L.out;
      live.style.filter = deboss(press);
      live.style.transform = 'scale(' + (1 - .012 * press) + ')';
      live.style.opacity = String(1 - .12 * press);
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.filter = 'none';
      L.in.style.transform = 'none';
    },
  });

  FX.register({
    id: 'hx-riso', name: 'Riso Registration', family: 'Print',
    blurb: 'Two ink layers drift apart, then pull into register as the new line.',
    duration: 1350,
    theme: { bg: '#F7F4EC', fg: '#1d1d1f' },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'fx-wrap', {
        position: 'relative', width: '100%', height: '100%',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
      });
      function ply(s, colour) {
        var d = FX.el('div', 'fx-line', {
          position: 'absolute', left: '0', right: '0', textAlign: 'center',
          color: colour, mixBlendMode: 'multiply', willChange: 'transform,opacity',
        });
        var a = FX.el('span', 'fx-light'); a.textContent = s.light;
        var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
        d.appendChild(a); d.appendChild(FX.el('br')); d.appendChild(b);
        wrap.appendChild(d);
        return d;
      }
      stage._oG = ply(ctx.from, '#7AB648');
      stage._oK = ply(ctx.from, '#1d1d1f');
      stage._nG = ply(ctx.to, '#7AB648');
      stage._nK = ply(ctx.to, '#1d1d1f');
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      var sep = Math.sin(Math.PI * clamp(t, 0, 1));         // registration error
      var a = span(t, .34, .62);
      stage._oG.style.transform = 'translate(' + (-7 * sep) + 'px,' + (3 * sep) + 'px)';
      stage._oK.style.transform = 'translate(' + (5 * sep) + 'px,' + (-2 * sep) + 'px)';
      stage._nG.style.transform = 'translate(' + (7 * sep) + 'px,' + (-3 * sep) + 'px)';
      stage._nK.style.transform = 'translate(' + (-5 * sep) + 'px,' + (2 * sep) + 'px)';
      stage._oG.style.opacity = stage._oK.style.opacity = String(1 - a);
      stage._nG.style.opacity = stage._nK.style.opacity = String(a);
    },
    rest: function (stage) {
      [stage._oG, stage._oK].forEach(function (n) { n.style.opacity = '0'; });
      [stage._nG, stage._nK].forEach(function (n) { n.style.opacity = '1'; n.style.transform = 'none'; });
    },
  });

  FX.register({
    id: 'hx-tear', name: 'Paper Tear', family: 'Print',
    blurb: 'The old line tears away along a ragged edge, revealing the new one beneath.',
    duration: 1300,
    theme: { bg: '#FBFAF7', fg: '#26231F' },
    setup: function (stage, ctx) {
      stage._l = twoLayer(stage, ctx);
      // one fixed ragged edge, built once, so the tear is identical every time it is seeked
      var pts = [];
      for (var k = 0; k <= 18; k++) {
        pts.push([k / 18 * 100, 46 + (rnd(k, 5) - .5) * 17]);
      }
      stage._edge = pts;
    },
    frame: function (stage, t) {
      var L = stage._l, pts = stage._edge;
      var p = E.inOutQuart(t);
      // polygon keeps the part of the old line above the tear line, which travels downward
      var top = pts.map(function (q) { return q[0] + '% ' + (q[1] - 120 + p * 190) + '%'; }).join(',');
      L.out.style.webkitClipPath = L.out.style.clipPath =
        'polygon(-5% -60%, 105% -60%, ' + top + ')';
      L.out.style.transform = 'translate(' + (-16 * p) + 'px,' + (-26 * p) + 'px) rotate(' + (-2.2 * p) + 'deg)';
      L.out.style.opacity = String(1 - E.inQuad(span(t, .55, 1)));
      L.in.style.opacity = String(span(t, .12, .5));
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.webkitClipPath = L.in.style.clipPath = 'none';
    },
  });

  /* ================================================================= GLITCH */

  FX.register({
    id: 'hx-rgb-split', name: 'Channel Split', family: 'Glitch',
    blurb: 'Colour channels tear apart horizontally, then lock back together as the new line.',
    duration: 1100,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'fx-wrap', {
        position: 'relative', width: '100%', height: '100%',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
      });
      function ch(s, colour, blend) {
        var d = FX.el('div', 'fx-line', {
          position: 'absolute', left: '0', right: '0', textAlign: 'center',
          color: colour, mixBlendMode: blend, willChange: 'transform,opacity',
        });
        var a = FX.el('span', 'fx-light'); a.textContent = s.light;
        var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
        d.appendChild(a); d.appendChild(FX.el('br')); d.appendChild(b);
        wrap.appendChild(d);
        return d;
      }
      stage._or = ch(ctx.from, '#ff2d55', 'multiply');
      stage._oc = ch(ctx.from, '#00c2d1', 'multiply');
      stage._nr = ch(ctx.to, '#ff2d55', 'multiply');
      stage._nc = ch(ctx.to, '#00c2d1', 'multiply');
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      var burst = Math.sin(Math.PI * clamp(t, 0, 1));
      var step = Math.floor(t * 11);
      var j = (rnd(step, 2) - .5) * 22 * burst;
      var d = 16 * burst;
      var a = span(t, .38, .66);
      stage._or.style.transform = 'translateX(' + (-d + j) + 'px)';
      stage._oc.style.transform = 'translateX(' + (d - j) + 'px)';
      stage._nr.style.transform = 'translateX(' + (d - j) + 'px)';
      stage._nc.style.transform = 'translateX(' + (-d + j) + 'px)';
      stage._or.style.opacity = stage._oc.style.opacity = String(1 - a);
      stage._nr.style.opacity = stage._nc.style.opacity = String(a);
    },
    rest: function (stage) {
      [stage._or, stage._oc].forEach(function (n) { n.style.opacity = '0'; });
      [stage._nr, stage._nc].forEach(function (n) { n.style.opacity = '1'; n.style.transform = 'none'; });
    },
  });

  FX.register({
    id: 'hx-crt', name: 'CRT Collapse', family: 'Glitch',
    blurb: 'The line collapses to a bright scanline and blooms back as the new one.',
    duration: 1200,
    theme: { bg: '#0a0a0b', fg: '#e9ffe4', accent: '#7AB648' },
    setup: function (stage, ctx) {
      var L = twoLayer(stage, ctx);
      // The collapsed state has to be something you can SEE. Rather than relying on a separate
      // element (an undefined reference there threw inside frame() and froze the whole effect at
      // its setup state), the scanline is a child of the wrap created here and always present.
      var scan = FX.el('div', 'fx-scan', {
        position: 'absolute', left: '5%', right: '5%', top: '50%', height: '2px',
        marginTop: '-1px', background: '#d9ffd0', opacity: '0',
        boxShadow: '0 0 16px 3px rgba(150,255,140,.85)', pointerEvents: 'none',
      });
      L.wrap.appendChild(scan);
      L.scan = scan;
      stage._l = L;
    },
    frame: function (stage, t) {
      var L = stage._l;
      if (!L) return;
      var close = E.inQuart(span(t, 0, .44));
      var open = E.outQuart(span(t, .52, 1));
      if (L.scan) L.scan.style.opacity = String(Math.sin(Math.PI * clamp(t, 0, 1)));
      L.out.style.opacity = String(t < .5 ? 1 - close * .15 : 0);
      L.out.style.transform = 'scaleY(' + Math.max(.02, 1 - close) + ') scaleX(' + (1 + .12 * close) + ')';
      L.out.style.filter = 'brightness(' + (1 + 2.2 * close) + ')';
      L.in.style.opacity = String(t < .5 ? 0 : 1);
      L.in.style.transform = 'scaleY(' + Math.max(.02, open) + ') scaleX(' + (1 + .12 * (1 - open)) + ')';
      L.in.style.filter = 'brightness(' + (1 + 2.2 * (1 - open)) + ')';
    },
    rest: function (stage) {
      var L = stage._l;
      if (!L) return;
      if (L.scan) L.scan.style.opacity = '0';
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.transform = 'none';
      L.in.style.filter = 'none';
    },
  });

  FX.register({
    id: 'hx-signal-lock', name: 'Signal Lock', family: 'Glitch',
    blurb: 'The line jitters and slips like an unlocked signal, then snaps stable.',
    duration: 1150,
    setup: function (stage, ctx) { stage._l = twoLayer(stage, ctx); },
    frame: function (stage, t) {
      var L = stage._l;
      if (!L) return;
      var unrest = 1 - E.outCubic(t);              // settles as t advances
      var step = Math.floor(t * 20);
      var dx = (rnd(step, 7) - .5) * 26 * unrest;
      var dy = (rnd(step, 13) - .5) * 9 * unrest;
      var slip = rnd(step, 21) < .3 ? (rnd(step, 31) - .5) * 40 * unrest : 0;
      var a = span(t, .3, .6);
      L.out.style.opacity = String(1 - a);
      L.out.style.transform = 'translate(' + (dx + slip) + 'px,' + dy + 'px)';
      L.out.style.filter = 'blur(' + (1.6 * unrest) + 'px)';
      L.in.style.opacity = String(a);
      L.in.style.transform = 'translate(' + (dx - slip) + 'px,' + (-dy) + 'px)';
      L.in.style.filter = 'blur(' + (1.6 * unrest) + 'px)';
    },
    rest: function (stage) {
      var L = stage._l;
      if (!L) return;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.transform = 'none';
      L.in.style.filter = 'none';
    },
  });

  /* ================================================================= KINETIC */

  FX.register({
    id: 'hx-spring-stagger', name: 'Spring Stagger', family: 'Kinetic',
    blurb: 'Words leave and arrive from alternating sides, each on its own spring.',
    duration: 1400,
    setup: function (stage, ctx) { stage._w = wordLayers(stage, ctx); },
    frame: function (stage, t) {
      var W = stage._w;
      var on = W.out.nodes, inn = W.in.nodes;
      on.forEach(function (n, k) {
        var p = E.inCubic(span(t, k / Math.max(1, on.length) * .3, .55));
        var dir = k % 2 ? 1 : -1;
        n.style.transform = 'translateX(' + (dir * 90 * p) + 'px) translateY(' + (12 * p) + 'px)';
        n.style.opacity = String(1 - p);
      });
      inn.forEach(function (n, k) {
        var lead = .34 + (k / Math.max(1, inn.length)) * .34;
        var p = E.outBack(span(t, lead, Math.min(1, lead + .42)));
        var dir = k % 2 ? -1 : 1;
        n.style.transform = 'translateX(' + (dir * 110 * (1 - p)) + 'px)';
        n.style.opacity = String(clamp(p * 1.6, 0, 1));
      });
    },
    rest: function (stage) {
      stage._w.out.nodes.forEach(function (n) { n.style.opacity = '0'; });
      stage._w.in.nodes.forEach(function (n) { n.style.opacity = '1'; n.style.transform = 'none'; });
    },
  });

  FX.register({
    id: 'hx-gravity-drop', name: 'Gravity Drop', family: 'Kinetic',
    blurb: 'The old line falls out of frame; the new one drops in and bounces to rest.',
    duration: 1350,
    setup: function (stage, ctx) { stage._w = wordLayers(stage, ctx); },
    frame: function (stage, t) {
      var W = stage._w;
      W.out.nodes.forEach(function (n, k) {
        var p = E.inQuad(span(t, k * .022, .5));
        n.style.transform = 'translateY(' + (120 * p) + 'px) rotate(' + ((rnd(k, 2) - .5) * 22 * p) + 'deg)';
        n.style.opacity = String(1 - span(t, .3, .55));
      });
      W.in.nodes.forEach(function (n, k) {
        var lead = .30 + k * .022;
        var p = E.outBounce(span(t, lead, Math.min(1, lead + .5)));
        n.style.transform = 'translateY(' + (-78 * (1 - p)) + 'px)';
        n.style.opacity = String(span(t, lead, lead + .12));
      });
    },
    rest: function (stage) {
      stage._w.out.nodes.forEach(function (n) { n.style.opacity = '0'; });
      stage._w.in.nodes.forEach(function (n) { n.style.opacity = '1'; n.style.transform = 'none'; });
    },
  });

  FX.register({
    id: 'hx-magnet', name: 'Magnetic Snap', family: 'Kinetic',
    blurb: 'Letters scatter, hang, then snap hard into their places.',
    duration: 1500,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'fx-wrap', {
        position: 'relative', width: '100%', height: '100%',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
      });
      function line(s) {
        var d = FX.el('div', 'fx-line', {
          position: 'absolute', left: '0', right: '0', textAlign: 'center',
        });
        var l = FX.el('span', 'fx-light');
        var lc = FX.charWords(s.light); l.appendChild(lc.frag);
        var b = FX.el('span', 'fx-bold');
        var bc = FX.charWords(s.bold); b.appendChild(bc.frag);
        d.appendChild(l); d.appendChild(FX.el('br')); d.appendChild(b);
        wrap.appendChild(d);
        return { el: d, nodes: lc.nodes.concat(bc.nodes) };
      }
      stage._o = line(ctx.from);
      stage._n = line(ctx.to);
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      stage._o.nodes.forEach(function (n, k) {
        var p = E.inQuad(span(t, 0, .45));
        n.style.transform = 'translate(' + ((rnd(k, 1) - .5) * 200 * p) + 'px,' +
          ((rnd(k, 2) - .5) * 130 * p) + 'px) rotate(' + ((rnd(k, 3) - .5) * 90 * p) + 'deg)';
        n.style.opacity = String(1 - p);
      });
      stage._n.nodes.forEach(function (n, k) {
        var p = E.outQuint(span(t, .42, 1));
        n.style.transform = 'translate(' + ((rnd(k, 4) - .5) * 220 * (1 - p)) + 'px,' +
          ((rnd(k, 5) - .5) * 150 * (1 - p)) + 'px) rotate(' + ((rnd(k, 6) - .5) * 100 * (1 - p)) + 'deg)';
        n.style.opacity = String(clamp(p * 2.2, 0, 1));
      });
    },
    rest: function (stage) {
      stage._o.nodes.forEach(function (n) { n.style.opacity = '0'; });
      stage._n.nodes.forEach(function (n) { n.style.opacity = '1'; n.style.transform = 'none'; });
    },
  });
})();
