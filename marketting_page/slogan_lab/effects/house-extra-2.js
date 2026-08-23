/* House effects, second batch — Technical, Fold, Optical, Organic and Data families.
 * Same contract: pure functions of progress, deterministic, seekable.
 * ids prefixed `hy-`.
 */
(function () {
  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;

  function rnd(a, b) {
    var x = Math.sin(a * 91.7 + b * 47.3) * 24634.6345;
    return x - Math.floor(x);
  }

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
      var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
      d.appendChild(a); d.appendChild(FX.el('br')); d.appendChild(b);
      wrap.appendChild(d);
      return d;
    }
    var out = line(ctx.from), inn = line(ctx.to);
    stage.appendChild(wrap);
    return { out: out, in: inn, wrap: wrap };
  }

  function charLayers(stage, ctx) {
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
    var o = line(ctx.from), i = line(ctx.to);
    stage.appendChild(wrap);
    return { out: o, in: i, wrap: wrap };
  }

  /* --------------------------------------------------------- TECHNICAL */

  FX.register({
    id: 'hy-dimension-line', name: 'Dimension Line', family: 'Technical',
    blurb: 'A drafting leader line draws across and the new text appears along it.',
    duration: 1450,
    theme: { bg: '#FCFCFA', fg: '#1A1D21', accent: '#2B4C7E' },
    setup: function (stage, ctx) {
      stage._l = twoLayer(stage, ctx);
      var rule = FX.el('div', null, {
        position: 'absolute', left: '0', top: '50%', height: '1px',
        background: '#2B4C7E', transformOrigin: 'left center', pointerEvents: 'none',
      });
      var tick = FX.el('div', null, {
        position: 'absolute', top: '50%', width: '1px', height: '13px',
        marginTop: '-6px', background: '#2B4C7E', pointerEvents: 'none',
      });
      stage._l.wrap.appendChild(rule);
      stage._l.wrap.appendChild(tick);
      stage._rule = rule; stage._tick = tick;
    },
    frame: function (stage, t) {
      var L = stage._l;
      var draw = E.inOutCubic(span(t, 0, .55));
      var back = E.inOutCubic(span(t, .5, 1));
      stage._rule.style.width = (draw * 100) + '%';
      stage._rule.style.opacity = String(1 - back);
      stage._tick.style.left = (draw * 100) + '%';
      stage._tick.style.opacity = String(draw > 0 ? 1 - back : 0);
      // old line is erased behind the travelling tick; new line is drawn in after it
      L.out.style.webkitClipPath = L.out.style.clipPath =
        'inset(0 ' + (draw * 100) + '% 0 0)';
      L.in.style.webkitClipPath = L.in.style.clipPath =
        'inset(0 0 0 ' + ((1 - draw) * 100) + '%)';
      L.in.style.opacity = '1';
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.webkitClipPath = L.in.style.clipPath = 'none';
      stage._rule.style.opacity = '0';
      stage._tick.style.opacity = '0';
    },
  });

  FX.register({
    id: 'hy-vernier', name: 'Vernier Slide', family: 'Technical',
    blurb: 'The line slides along a measuring scale and locks onto its mark.',
    duration: 1350,
    theme: { bg: '#FAFAF7', fg: '#1d1d1f', accent: '#7AB648' },
    setup: function (stage, ctx) {
      stage._l = twoLayer(stage, ctx);
      var scale = FX.el('div', null, {
        position: 'absolute', left: '0', right: '0', bottom: '10%', height: '9px',
        backgroundImage: 'repeating-linear-gradient(to right,#9aa3ad 0 1px,transparent 1px 12px)',
        opacity: '.5', pointerEvents: 'none',
      });
      stage._l.wrap.appendChild(scale);
      stage._scale = scale;
    },
    frame: function (stage, t) {
      var L = stage._l;
      var p = E.outQuint(t);
      var over = Math.sin(Math.PI * clamp(t, 0, 1)) * 5;   // slight travel past the mark
      L.out.style.transform = 'translateX(' + (-p * 100 - over) + 'px)';
      L.out.style.opacity = String(1 - E.inQuad(span(t, .1, .55)));
      L.in.style.transform = 'translateX(' + ((1 - p) * 110 + over) + 'px)';
      L.in.style.opacity = String(E.outCubic(span(t, .3, .8)));
      stage._scale.style.backgroundPosition = (-p * 90) + 'px 0';
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1'; L.in.style.transform = 'none';
    },
  });

  /* --------------------------------------------------------- FOLD */

  FX.register({
    id: 'hy-origami', name: 'Origami Panels', family: 'Fold',
    blurb: 'Vertical panels fold away in sequence and unfold as the new line.',
    duration: 1400,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'fx-wrap', {
        position: 'relative', width: '100%', height: '100%',
        perspective: '900px', display: 'flex', alignItems: 'center', justifyContent: 'center',
      });
      var N = 8;
      stage._panels = [];
      for (var k = 0; k < N; k++) {
        var slot = FX.el('div', null, {
          position: 'absolute', top: '0', bottom: '0',
          left: (k * 100 / N) + '%', width: (100 / N) + '%',
          overflow: 'hidden', transformStyle: 'preserve-3d',
          transformOrigin: k % 2 ? 'left center' : 'right center',
          willChange: 'transform',
        });
        function face(s, off) {
          var f = FX.el('div', 'fx-line', {
            position: 'absolute', top: '0', bottom: '0',
            left: (-off) + 'px', width: '0', textAlign: 'center',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
          });
          var a = FX.el('span', 'fx-light'); a.textContent = s.light;
          var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
          var inner = FX.el('div');
          inner.appendChild(a); inner.appendChild(FX.el('br')); inner.appendChild(b);
          f.appendChild(inner);
          return f;
        }
        var o = face(ctx.from, 0), n = face(ctx.to, 0);
        slot.appendChild(o); slot.appendChild(n);
        wrap.appendChild(slot);
        stage._panels.push({ slot: slot, o: o, n: n, k: k, N: N });
      }
      stage.appendChild(wrap);
      stage._sized = false;
    },
    frame: function (stage, t) {
      var W = stage.clientWidth;
      stage._panels.forEach(function (p) {
        // the faces must be full stage width and offset, so the text lines up across panels
        p.o.style.width = W + 'px';
        p.n.style.width = W + 'px';
        p.o.style.left = (-p.k * W / p.N) + 'px';
        p.n.style.left = (-p.k * W / p.N) + 'px';
        var lead = (p.k / p.N) * .42;
        var q = E.inOutCubic(span(t, lead, lead + .5));
        var ang = q * 180;
        p.slot.style.transform = 'rotateY(' + (p.k % 2 ? -ang : ang) + 'deg)';
        p.o.style.opacity = String(q < .5 ? 1 : 0);
        p.n.style.opacity = String(q < .5 ? 0 : 1);
        // counter-rotate the revealed face so the new text reads the right way round
        p.n.style.transform = 'rotateY(180deg)';
        var shade = 1 - Math.abs(Math.cos(ang * Math.PI / 180)) * .18;
        p.slot.style.filter = 'brightness(' + shade + ')';
      });
    },
    rest: function (stage) {
      stage._panels.forEach(function (p) {
        p.slot.style.transform = 'none';
        p.slot.style.filter = 'none';
        p.o.style.opacity = '0';
        p.n.style.opacity = '1';
        p.n.style.transform = 'none';
      });
    },
  });

  /* --------------------------------------------------------- OPTICAL */

  FX.register({
    id: 'hy-lens-focus', name: 'Lens Focus', family: 'Optical',
    blurb: 'Focus racks past the plane, chromatic fringes and all, onto the new line.',
    duration: 1300,
    setup: function (stage, ctx) { stage._l = twoLayer(stage, ctx); },
    frame: function (stage, t) {
      var L = stage._l;
      var a = span(t, 0, .55), b = span(t, .4, 1);
      var blurA = 14 * E.inQuad(a);
      var blurB = 14 * (1 - E.outQuad(b));
      L.out.style.opacity = String(1 - E.inQuad(span(t, .3, .62)));
      L.out.style.filter = 'blur(' + blurA + 'px)';
      L.out.style.transform = 'scale(' + (1 + .05 * a) + ')';
      L.out.style.textShadow = blurA > .5
        ? (blurA * .22) + 'px 0 rgba(255,0,80,.5), ' + (-blurA * .22) + 'px 0 rgba(0,190,220,.5)' : 'none';
      L.in.style.opacity = String(E.outQuad(span(t, .34, .72)));
      L.in.style.filter = 'blur(' + blurB + 'px)';
      L.in.style.transform = 'scale(' + (1 - .04 * (1 - E.outQuad(b))) + ')';
      L.in.style.textShadow = blurB > .5
        ? (blurB * .22) + 'px 0 rgba(255,0,80,.5), ' + (-blurB * .22) + 'px 0 rgba(0,190,220,.5)' : 'none';
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1'; L.in.style.filter = 'none';
      L.in.style.transform = 'none'; L.in.style.textShadow = 'none';
    },
  });

  /* --------------------------------------------------------- ORGANIC */

  FX.register({
    id: 'hy-smoke', name: 'Smoke Drift', family: 'Organic',
    blurb: 'Letters lift and disperse like smoke; the new line condenses out of the haze.',
    duration: 1700,
    setup: function (stage, ctx) { stage._c = charLayers(stage, ctx); },
    frame: function (stage, t) {
      var C = stage._c;
      C.out.nodes.forEach(function (n, k) {
        var lead = (k / Math.max(1, C.out.nodes.length)) * .22;
        var p = E.inOutQuad(span(t, lead, lead + .55));
        n.style.transform = 'translate(' + ((rnd(k, 1) - .5) * 26 * p) + 'px,' + (-58 * p) + 'px) ' +
          'rotate(' + ((rnd(k, 2) - .5) * 26 * p) + 'deg) scale(' + (1 + .5 * p) + ')';
        n.style.opacity = String(1 - p);
        n.style.filter = 'blur(' + (5 * p) + 'px)';
      });
      C.in.nodes.forEach(function (n, k) {
        var lead = .34 + (k / Math.max(1, C.in.nodes.length)) * .3;
        var p = E.outCubic(span(t, lead, Math.min(1, lead + .42)));
        n.style.transform = 'translate(' + ((rnd(k, 3) - .5) * 22 * (1 - p)) + 'px,' +
          (26 * (1 - p)) + 'px) scale(' + (1 + .3 * (1 - p)) + ')';
        n.style.opacity = String(p);
        n.style.filter = 'blur(' + (5 * (1 - p)) + 'px)';
      });
    },
    rest: function (stage) {
      stage._c.out.nodes.forEach(function (n) { n.style.opacity = '0'; });
      stage._c.in.nodes.forEach(function (n) {
        n.style.opacity = '1'; n.style.transform = 'none'; n.style.filter = 'none';
      });
    },
  });

  /* --------------------------------------------------------- ARCHITECTURE */

  FX.register({
    id: 'hy-louvre', name: 'Louvre Turn', family: 'Architecture',
    blurb: 'Horizontal louvres pivot; the new line is on their backs.',
    duration: 1300,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'fx-wrap', {
        position: 'relative', width: '100%', height: '100%', perspective: '1000px',
      });
      var N = 9;
      stage._slats = [];
      for (var k = 0; k < N; k++) {
        var slat = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0',
          top: (k * 100 / N) + '%', height: (100 / N) + '%',
          overflow: 'hidden', transformStyle: 'preserve-3d',
          transformOrigin: 'center center', willChange: 'transform',
        });
        function face(s, k2) {
          var f = FX.el('div', null, {
            position: 'absolute', left: '0', right: '0', top: '0',
            height: (N * 100) + '%', textAlign: 'center',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            transform: 'translateY(' + (-k2 * 100 / N) + '%)',
          });
          var box = FX.el('div', 'fx-line');
          var a = FX.el('span', 'fx-light'); a.textContent = s.light;
          var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
          box.appendChild(a); box.appendChild(FX.el('br')); box.appendChild(b);
          f.appendChild(box);
          return f;
        }
        var o = face(ctx.from, k), n = face(ctx.to, k);
        n.style.transform += ' rotateX(180deg)';
        slat.appendChild(o); slat.appendChild(n);
        wrap.appendChild(slat);
        stage._slats.push({ slat: slat, o: o, n: n, k: k });
      }
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      stage._slats.forEach(function (s) {
        var lead = (s.k / stage._slats.length) * .38;
        var q = E.inOutCubic(span(t, lead, lead + .55));
        var ang = q * 180;
        s.slat.style.transform = 'rotateX(' + ang + 'deg)';
        s.o.style.opacity = String(q < .5 ? 1 : 0);
        s.n.style.opacity = String(q < .5 ? 0 : 1);
        s.slat.style.filter = 'brightness(' + (1 - Math.sin(q * Math.PI) * .3) + ')';
      });
    },
    rest: function (stage) {
      stage._slats.forEach(function (s) {
        s.slat.style.transform = 'none';
        s.slat.style.filter = 'none';
        s.o.style.opacity = '0';
        s.n.style.opacity = '1';
      });
    },
  });

  /* --------------------------------------------------------- DATA */

  FX.register({
    id: 'hy-bar-morph', name: 'Bar Morph', family: 'Data',
    blurb: 'Letters collapse into data bars, the bars re-sort, and the new line grows out of them.',
    duration: 1600,
    theme: { accent: '#7AB648' },
    setup: function (stage, ctx) { stage._c = charLayers(stage, ctx); },
    frame: function (stage, t) {
      var C = stage._c;
      // phase 1: old chars squash to bars. phase 2: bars grow into new chars.
      C.out.nodes.forEach(function (n, k) {
        var p = E.inOutCubic(span(t, 0, .46));
        n.style.transform = 'scaleY(' + lerp(1, .26, p) + ') translateY(' + (p * 8) + 'px)';
        n.style.opacity = String(1 - span(t, .42, .54));
        n.style.color = p > .3 ? '#7AB648' : '';
      });
      C.in.nodes.forEach(function (n, k) {
        var lead = .44 + (k / Math.max(1, C.in.nodes.length)) * .2;
        var p = E.outQuart(span(t, lead, Math.min(1, lead + .45)));
        n.style.transform = 'scaleY(' + lerp(.26, 1, p) + ') translateY(' + ((1 - p) * 8) + 'px)';
        n.style.opacity = String(span(t, .36, .50));
        n.style.color = p < .75 ? '#7AB648' : '';
      });
    },
    rest: function (stage) {
      stage._c.out.nodes.forEach(function (n) { n.style.opacity = '0'; });
      stage._c.in.nodes.forEach(function (n) {
        n.style.opacity = '1'; n.style.transform = 'none'; n.style.color = '';
      });
    },
  });

  FX.register({
    id: 'hy-flip-clock', name: 'Flip Clock', family: 'Mechanical',
    blurb: 'The whole line flips over on a centre hinge, departure-board style.',
    duration: 1150,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'fx-wrap', {
        position: 'relative', width: '100%', height: '100%', perspective: '1200px',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
      });
      function card(s) {
        var d = FX.el('div', 'fx-line', {
          position: 'absolute', left: '0', right: '0', textAlign: 'center',
          backfaceVisibility: 'hidden', transformOrigin: 'center center',
          willChange: 'transform',
        });
        var a = FX.el('span', 'fx-light'); a.textContent = s.light;
        var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
        d.appendChild(a); d.appendChild(FX.el('br')); d.appendChild(b);
        wrap.appendChild(d);
        return d;
      }
      stage._o = card(ctx.from);
      stage._n = card(ctx.to);
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      // Two asymmetric halves rather than one symmetric sweep: the outgoing card falls to edge-on
      // by t=0.40, the incoming card opens from edge-on with an outQuart so it is already well
      // past 40 degrees by t=0.5. A single inOutQuart put dead-on-edge exactly at the midpoint,
      // which rendered an empty frame.
      var a = E.inQuad(span(t, 0, .40));
      var b = E.outQuart(span(t, .40, 1));
      stage._o.style.transform = 'rotateX(' + (-90 * a) + 'deg)';
      stage._n.style.transform = 'rotateX(' + (90 - 90 * b) + 'deg)';
      stage._o.style.opacity = String(t < .40 ? 1 : 0);
      stage._n.style.opacity = String(t < .40 ? 0 : 1);
      var shade = 1 - (t < .40 ? a : 1 - b) * .3;
      stage._o.style.filter = 'brightness(' + shade + ')';
      stage._n.style.filter = 'brightness(' + shade + ')';
    },
    rest: function (stage) {
      stage._o.style.transform = 'rotateX(-90deg)';
      stage._o.style.opacity = '0';
      stage._n.style.opacity = '1';
      stage._n.style.transform = 'rotateX(0deg)';
      stage._o.style.filter = 'none';
      stage._n.style.filter = 'none';
    },
  });
})();
