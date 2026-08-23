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
      var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
      var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
      d.appendChild(a); d.appendChild(b);
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
      var l = FX.el('span', 'fx-light', { display: 'block' });
      var lw = FX.words(s.light); l.appendChild(lw.frag);
      var b = FX.el('span', 'fx-bold', { display: 'block' });
      var bw = FX.words(s.bold); b.appendChild(bw.frag);
      d.appendChild(l); d.appendChild(b);
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
    setup: function (stage, ctx) { stage._l = twoLayer(stage, ctx); stage._sc = ctx.sc; },
    frame: function (stage, t) {
      var L = stage._l, sc = stage._sc;
      var x = E.inOutCubic(t) * 140 - 20;           // beam centre, -20%..120%

      /* The two masks used to be the wrong way round, and their stops were clamped INTO the box.
         At t=0.008 the beam sits at -20%, so the old line's two stops both collapsed onto 0% and
         masked the whole line away, while the new line's mask had already opened completely: the
         transition opened on the INCOMING text (47px of ink-box jump at pair 0, 179px at pair 12)
         and closed on the outgoing one. Stops are allowed to sit outside the box, and that is what
         keeps a sweep honest at its ends — so clamp only far enough out to stay sane. */
      var sweep = function (aCol, aPct, bCol, bPct) {
        return 'linear-gradient(to right, ' + aCol + ' ' + clamp(aPct, -60, 160).toFixed(2) + '%, ' +
               bCol + ' ' + clamp(bPct, -60, 160).toFixed(2) + '%)';
      };
      // old line is lit AHEAD of the beam, erased everywhere the beam has already swept
      L.out.style.webkitMaskImage = L.out.style.maskImage = sweep('transparent', x - 6, '#000', x + 14);
      L.out.style.opacity = '1';
      // new line exists only BEHIND the beam, and stays lit once it has passed
      L.in.style.webkitMaskImage = L.in.style.maskImage = sweep('#000', x - 14, 'transparent', x + 6);
      L.in.style.opacity = '1';
      L.in.style.filter = 'brightness(' + lerp(1.5, 1, E.outCubic(t)) + ')';

      /* The pool of light was hard-coded black-on-black: invisible on the dark hero and a dark
         smudge on white paper, which is the opposite of a spotlight. Take it from the stage
         instead, and fade it up and away with the sweep so neither endpoint carries a wash. */
      var pool = sc && !sc.dark ? '0,0,0' : '255,255,255';
      var lit = Math.sin(Math.PI * clamp(t, 0, 1)) * 0.13;
      L.wrap.style.backgroundImage =
        'radial-gradient(58% 190% at ' + x.toFixed(2) + '% 50%, ' +
        'rgba(' + pool + ',' + lit.toFixed(4) + ') 0%, ' +
        'rgba(' + pool + ',' + (lit * 0.45).toFixed(4) + ') 30%, transparent 56%)';
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
      /* Everything the tube does has to grow out of, and fall back into, a plain lit line.
         Both the halo and the stutter used to run at full strength from the very first frame:
         the 20px drop-shadow alone pushed the ink box 11px past the canonical text and lit 8.5%
         of the stage, and the stutter could open the transition on a dimmed line. This envelope
         is 0 at both endpoints, so t=0.008 is the outgoing line exactly as the engine parked it,
         and the tube only starts misbehaving once it is on screen. */
      var env = Math.sin(Math.PI * clamp(t, 0, 1));
      // deterministic stutter: quantise t, hash the step. Real tubes flicker unevenly.
      function stutter(p, seed) {
        var step = Math.floor(p * 14);
        var r = rnd(step, seed);
        var dip = r < 0.34 ? 0.15 + r : 1;
        return 1 - (1 - dip) * env;
      }
      // the new tube reaches full burn just before the end, not exactly at it
      var a = span(t, 0, .46), b = span(t, .4, .96);
      var glow = function (v) {
        var g = v * env;
        return 'drop-shadow(0 0 ' + (7 * g) + 'px rgba(160,220,255,.85)) ' +
               'drop-shadow(0 0 ' + (20 * g) + 'px rgba(122,182,72,.45))';
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
        /* the sheet is whatever the stage already is; isolate so the plies blend against it
           and not against whatever happens to be behind the stage */
        background: ctx.sc.paper, isolation: 'isolate',
      });
      function ply(s, colour, blend) {
        var d = FX.el('div', 'fx-line', {
          position: 'absolute', left: '0', right: '0', textAlign: 'center',
          color: colour, mixBlendMode: blend, willChange: 'transform,opacity',
        });
        var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
        var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
        d.appendChild(a); d.appendChild(b);
        wrap.appendChild(d);
        return d;
      }
      /* A riso is SUBTRACTIVE: spot inks multiplying down onto white paper. Both plies were
         multiplied, and #7AB648 or #1d1d1f multiplied into the dark hero's #0A0B0C is #0A0B0C —
         so the effect drew literally nothing, at every frame including both endpoints. Same two
         drums, opposite direction: on a dark ground the ink is light and screens up. */
      var blend = ctx.sc.dark ? 'screen' : 'multiply';
      stage._oK = ply(ctx.from, ctx.sc.ink, 'normal');
      stage._oG = ply(ctx.from, '#7AB648', blend);
      stage._nK = ply(ctx.to, ctx.sc.ink, 'normal');
      stage._nG = ply(ctx.to, '#7AB648', blend);
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      var sep = Math.sin(Math.PI * clamp(t, 0, 1));         // registration error
      var a = span(t, .34, .62);
      /* The colour drum is only on press in the middle. Even in perfect register a spot ink
         laid over the key tints every glyph, so the endpoints have to be the key layer alone,
         in the stage's own ink, exactly where the engine parked the canonical text. */
      var cv = Math.min(span(t, .03, .17), span(1 - t, .03, .17));
      stage._oG.style.transform = 'translate(' + (-7 * sep) + 'px,' + (3 * sep) + 'px)';
      stage._oK.style.transform = 'translate(' + (5 * sep) + 'px,' + (-2 * sep) + 'px)';
      stage._nG.style.transform = 'translate(' + (7 * sep) + 'px,' + (-3 * sep) + 'px)';
      stage._nK.style.transform = 'translate(' + (-5 * sep) + 'px,' + (2 * sep) + 'px)';
      stage._oK.style.opacity = String(1 - a);
      stage._nK.style.opacity = String(a);
      stage._oG.style.opacity = String((1 - a) * cv);
      stage._nG.style.opacity = String(a * cv);
    },
    rest: function (stage) {
      [stage._oG, stage._oK, stage._nG].forEach(function (n) { n.style.opacity = '0'; });
      stage._nK.style.opacity = '1';
      stage._nK.style.transform = 'none';
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
      /* inOutQuart put the whole rip into the middle fifth of the transition, so the flap was
         gone by t=0.5 and the second half was a still frame. A tear is a steady pull: ease it
         gently and let it run from just after the start to well before the end. */
      var p = E.inOutQuad(span(t, .02, .80));
      /* The polygon keeps the flap ABOVE the ragged edge, so the edge has to START below the
         whole line and rise through it. It used to start at roughly -74% — above the top of the
         line box — which clipped every last pixel away: the transition opened on an empty stage
         instead of on the outgoing text, and only became visible once the tear was underway.
         Now p=0 puts the edge at ~124% (nothing clipped, the first frame IS the outgoing line)
         and p=1 at ~-56% (the flap is gone), while the flap lifts up and to the left.
         The points also run right-to-left here: appended in reading order after the two top
         corners they made a self-intersecting bowtie, whose fill was at the mercy of the
         nonzero rule rather than of the tear. */
      var edge = pts.map(function (q) { return q[0] + '% ' + (q[1] + 78 - p * 180).toFixed(2) + '%'; });
      var top = edge.reverse().join(',');
      L.out.style.webkitClipPath = L.out.style.clipPath =
        'polygon(-8% -220%, 108% -220%, ' + top + ')';
      L.out.style.transform = 'translate(' + (-16 * p) + 'px,' + (-26 * p) + 'px) rotate(' + (-2.2 * p) + 'deg)';
      L.out.style.opacity = String(1 - E.inQuad(span(t, .55, 1)));
      // the sheet beneath comes up as the flap clears it, and settles onto the canonical line
      var lift = E.outCubic(span(t, .26, .92));
      L.in.style.opacity = String(span(t, .22, .74));
      L.in.style.transform = 'translateY(' + (7 * (1 - lift)).toFixed(2) + 'px)';
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
        /* isolate over the stage's own colour, so the channels blend against this sheet */
        background: ctx.sc.paper, isolation: 'isolate',
      });
      function ch(s, colour, blend) {
        var d = FX.el('div', 'fx-line', {
          position: 'absolute', left: '0', right: '0', textAlign: 'center',
          color: colour, mixBlendMode: blend, willChange: 'transform,opacity',
        });
        var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
        var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
        d.appendChild(a); d.appendChild(b);
        wrap.appendChild(d);
        return d;
      }
      /* A channel tear is ADDITIVE on a lit display and subtractive only in print. Both tinted
         copies were multiplied, and #ff2d55 or #00c2d1 multiplied into the dark hero's #0A0B0C
         is #0A0B0C — the effect drew nothing at all, at both endpoints and everywhere between.
         Screen on a dark ground, multiply on paper.
         The plain ink copy underneath is new: two tinted copies recolour every glyph even when
         they are in perfect register, so the endpoints need an untinted line to land on. */
      var blend = ctx.sc.dark ? 'screen' : 'multiply';
      stage._ob = ch(ctx.from, ctx.sc.ink, 'normal');
      stage._or = ch(ctx.from, '#ff2d55', blend);
      stage._oc = ch(ctx.from, '#00c2d1', blend);
      stage._nb = ch(ctx.to, ctx.sc.ink, 'normal');
      stage._nr = ch(ctx.to, '#ff2d55', blend);
      stage._nc = ch(ctx.to, '#00c2d1', blend);
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      var burst = Math.sin(Math.PI * clamp(t, 0, 1));
      var step = Math.floor(t * 11);
      var j = (rnd(step, 2) - .5) * 22 * burst;
      var d = 16 * burst;
      var a = span(t, .38, .66);
      // the channels are only torn off the signal in the middle; the ends are locked
      var cv = Math.min(span(t, .04, .18), span(1 - t, .04, .18));
      var out = 'translateX(' + (-d + j).toFixed(2) + 'px)';
      var back = 'translateX(' + (d - j).toFixed(2) + 'px)';
      stage._ob.style.transform = stage._nb.style.transform =
        'translateX(' + (j * .25).toFixed(2) + 'px)';
      stage._or.style.transform = stage._nc.style.transform = out;
      stage._oc.style.transform = stage._nr.style.transform = back;
      // the ink copy steps back while the channels are out, so the split reads as a split
      stage._ob.style.opacity = String((1 - a) * (1 - .45 * cv));
      stage._nb.style.opacity = String(a * (1 - .45 * cv));
      stage._or.style.opacity = stage._oc.style.opacity = String((1 - a) * cv);
      stage._nr.style.opacity = stage._nc.style.opacity = String(a * cv);
    },
    rest: function (stage) {
      [stage._ob, stage._or, stage._oc, stage._nr, stage._nc].forEach(function (n) {
        n.style.opacity = '0';
      });
      stage._nb.style.opacity = '1';
      stage._nb.style.transform = 'none';
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
      /* outCubic is already ~0.98 at t=0.008, so the very first frame arrived with the jitter
         at nearly full amplitude: the outgoing line opened displaced by up to a dozen px and
         blurred, which is 5-6% of the stage differing from the canonical text the engine had
         just been showing. The signal has to LOSE lock, not start unlocked — so ramp the
         amplitude up over the first tenth, then let it decay exactly as before. */
      var unrest = (1 - E.outCubic(t)) * E.inQuad(span(t, 0, .10));
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
        var l = FX.el('span', 'fx-light', { display: 'block' });
        var lc = FX.charWords(s.light); l.appendChild(lc.frag);
        var b = FX.el('span', 'fx-bold', { display: 'block' });
        var bc = FX.charWords(s.bold); b.appendChild(bc.frag);
        d.appendChild(l); d.appendChild(b);
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
