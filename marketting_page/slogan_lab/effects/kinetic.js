/* Family: Kinetic — physics and choreography.
 *
 * Six transitions that read as mass being moved: a spring stagger, a gravity drop with bounce,
 * a magnetic snap, a pendulum swing, an inertial whip and a domino chain.
 *
 * Every one is a PURE FUNCTION OF PROGRESS t. There is no timer, no CSS transition and no
 * @keyframes anywhere in this file; the "randomness" in the scatter effects is a deterministic
 * hash of the character index, so (i, t) always renders the identical picture and any frame can
 * be seeked, screenshotted or exported.
 *
 * House rules honoured throughout:
 *   - t = 0 is the FROM slogan exactly at rest, t = 1 the TO slogan exactly at rest.
 *   - No dead frames. Every departure window overlaps the matching arrival window, so at no t
 *     is the stage empty: either you are reading a line, or you are watching one become another.
 *   - Words are inline-blocks joined by real text-node spaces, and per-character effects keep
 *     their characters inside a nowrap word box, so a long slogan wraps by word and never
 *     breaks mid-word.
 *   - Motion blur is derived from the actual derivative of the position curve and gated to zero
 *     at the ends, so both resting frames are crisp.
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, clamp = FX.clamp, el = FX.el;
  var GREEN = '#7AB648';

  /* ---------------------------------------------------------------- maths */

  function u(k, n) { return n > 1 ? k / (n - 1) : 0; }

  /* The engine's ease table has no inQuart — a hard, late-breaking acceleration is exactly what
     "the pole switches on and throws you" wants, so it lives here. */
  function inQuart(s) { return s * s * s * s; }

  /* Deterministic stand-in for randomness: same k always gives the same value. */
  function rnd(k, s) {
    var x = Math.sin((k + 1) * 12.9898 + s * 78.233) * 43758.5453;
    return x - Math.floor(x);
  }

  /* Damped spring: 0 at s=0, 1 at s=1, with a real overshoot in between.
     The (1-s)^2 envelope is what forces the tail to land exactly on 1 — without it the curve
     ends a fraction of a percent off and the "at rest" frame is subtly wrong. */
  function damped(s, w, z) {
    if (s <= 0) return 0;
    if (s >= 1) return 1;
    return 1 - Math.exp(-z * s) * Math.cos(w * s) * (1 - s) * (1 - s);
  }

  /* Magnetic approach: creeps, then is yanked in, overshoots the pole by 6% and settles. */
  function magnet(s) {
    if (s <= 0) return 0;
    if (s >= 1) return 1;
    if (s <= 0.82) return Math.pow(s / 0.82, 2.8) * 1.06;
    return 1.06 - 0.06 * E.outCubic((s - 0.82) / 0.18);
  }

  /* |d/ds| of a progress curve — the only source of motion blur below. */
  function speed(fn, t) {
    var h = 0.012, a = clamp(t - h, 0, 1), b = clamp(t + h, 0, 1);
    if (b - a < 1e-6) return 0;
    return Math.abs(fn(b) - fn(a)) / (b - a);
  }
  function mb(px) { return px > 0.25 ? 'blur(' + px.toFixed(2) + 'px)' : 'none'; }

  /* ---------------------------------------------------------------- plumbing */

  /* Words as inline-blocks joined by plain text nodes, so the browser keeps its normal break
     opportunities (a white-space:pre spacer suppresses them and overflows the stage). */
  function addWords(parent, text, cls, sink) {
    var parts = String(text || '').split(/\s+/).filter(Boolean);
    for (var i = 0; i < parts.length; i++) {
      var s = el('span', cls, { display: 'inline-block', willChange: 'transform,opacity,filter' });
      s.textContent = parts[i];
      parent.appendChild(s);
      sink.push(s);
      if (i < parts.length - 1) parent.appendChild(document.createTextNode(' '));
    }
  }

  /* Per-character spans, grouped inside a nowrap word box so lines never break mid-word. */
  function addChars(parent, text, cls, sink) {
    var parts = String(text || '').split(/\s+/).filter(Boolean);
    for (var i = 0; i < parts.length; i++) {
      var box = el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
      for (var j = 0; j < parts[i].length; j++) {
        var c = el('span', cls, { display: 'inline-block', willChange: 'transform,opacity,filter' });
        c.textContent = parts[i][j];
        box.appendChild(c);
        sink.push(c);
      }
      parent.appendChild(box);
      if (i < parts.length - 1) parent.appendChild(document.createTextNode(' '));
    }
  }

  /* Two absolutely-positioned, full-width lines (old first, new on top) plus their item lists.
     Absolute + left/right:0 inside a centred flex box gives both lines the same centred static
     position, so they wrap identically and never disturb each other's layout. */
  function build(stage, ctx, o) {
    o = o || {};
    stage.innerHTML = '';
    var wrap = el('div', 'fx-wrap', {
      position: 'relative', width: '100%', height: '100%',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    });
    var add = o.chars ? addChars : addWords;
    function mk(s) {
      var d = el('div', 'fx-line', {
        position: 'absolute', left: '0', right: '0', textAlign: 'center',
        willChange: 'transform,opacity',
      });
      var items = [];
      add(d, s.light, 'fx-light', items);
      if (o.split) d.appendChild(el('br'));
      else d.appendChild(document.createTextNode(' '));
      add(d, s.bold, 'fx-bold', items);
      wrap.appendChild(d);
      return { el: d, items: items };
    }
    var a = mk(ctx.from), b = mk(ctx.to);
    stage.appendChild(wrap);
    return { wrap: wrap, out: a.el, in: b.el, outN: a.items, inN: b.items };
  }

  function origin(nodes, v) {
    for (var i = 0; i < nodes.length; i++) nodes[i].style.transformOrigin = v;
  }

  /* Shared settled state: the new line untouched, the old one gone. */
  function settle(L) {
    L.out.style.opacity = '0';
    L.in.style.opacity = '1';
    var i;
    for (i = 0; i < L.outN.length; i++) { L.outN[i].style.opacity = '0'; L.outN[i].style.filter = 'none'; }
    for (i = 0; i < L.inN.length; i++) {
      L.inN[i].style.transform = 'none';
      L.inN[i].style.opacity = '1';
      L.inN[i].style.filter = 'none';
    }
  }

  /* ---- 1. Spring Stagger --------------------------------------------------
     Words are flung off alternating sides, and each replacement is thrown in from the opposite
     side on a spring that overshoots the centre and rings down. The arrival stagger runs a beat
     behind the departure stagger, so the incoming word is already on screen before its
     predecessor has cleared: the stage is never empty. */
  FX.register({
    id: 'kinetic-spring',
    name: 'Spring Stagger',
    family: 'Kinetic',
    blurb: 'Words are flung off alternate sides; their replacements spring back in and ring down.',
    duration: 1150,
    dwell: 4400,
    setup: function (stage, ctx) { stage._l = build(stage, ctx, { split: true }); },
    frame: function (stage, t, ctx) {
      var L = stage._l, W = Math.max(240, ctx.W);
      var no = L.outN.length, ni = L.inN.length, k, d, p, s, q, x;
      L.out.style.opacity = '1'; L.in.style.opacity = '1';

      for (k = 0; k < no; k++) {
        d = (k % 2) ? 1 : -1;
        p = span(t, u(k, no) * 0.26, u(k, no) * 0.26 + 0.5);
        var e = E.inCubic(p);
        x = d * W * 0.85 * e;
        L.outN[k].style.transform = 'translate(' + x.toFixed(1) + 'px,' + (-12 * e).toFixed(1) + 'px)'
          + ' rotate(' + (d * 8 * e).toFixed(2) + 'deg)';
        L.outN[k].style.opacity = String(clamp(1 - p * p * 1.4, 0, 1));
        L.outN[k].style.filter = mb(speed(E.inCubic, p) * 1.5);
      }

      for (k = 0; k < ni; k++) {
        d = (k % 2) ? -1 : 1;
        var lead = 0.20 + u(k, ni) * 0.26;
        s = span(t, lead, 1);
        q = damped(s, 8.6, 4.6);
        x = d * W * 0.72 * (1 - q);
        L.inN[k].style.transform = 'translate(' + x.toFixed(1) + 'px,0)'
          + ' rotate(' + (d * -6 * (1 - q)).toFixed(2) + 'deg)';
        L.inN[k].style.opacity = String(clamp(span(t, lead, lead + 0.10) * 1.3, 0, 1));
        L.inN[k].style.filter = mb(speed(function (v) { return damped(v, 8.6, 4.6); }, s) * 1.1);
      }
    },
    rest: function (stage) { settle(stage._l); },
  });

  /* ---- 2. Gravity Drop ----------------------------------------------------
     The old line lets go word by word and falls out of frame under constant acceleration,
     tumbling as it goes. The new one is dropped from just above the stage and lands on
     outBounce, squashing on each contact. The drop starts while the old line is still in
     frame, so the two overlap for a third of the transition and nothing is ever blank. */
  FX.register({
    id: 'kinetic-gravity',
    name: 'Gravity Drop',
    family: 'Kinetic',
    blurb: 'The line lets go and falls out of frame; the new one drops in, bounces and settles.',
    duration: 1250,
    dwell: 4400,
    setup: function (stage, ctx) {
      var L = build(stage, ctx, { split: true });
      origin(L.inN, '50% 100%');
      stage._l = L;
    },
    frame: function (stage, t, ctx) {
      var L = stage._l, H = Math.max(120, ctx.H);
      var no = L.outN.length, ni = L.inN.length, k, p, s;
      L.out.style.opacity = '1'; L.in.style.opacity = '1';

      for (k = 0; k < no; k++) {
        var lead = u(k, no) * 0.16;
        p = span(t, lead, lead + 0.46);
        var f = p * p;                                   // s = ½at², constant g
        var tilt = (rnd(k, 1) * 2 - 1) * 22 * f;
        L.outN[k].style.transform = 'translateY(' + (f * H * 1.35).toFixed(1) + 'px)'
          + ' rotate(' + tilt.toFixed(2) + 'deg)';
        L.outN[k].style.opacity = String(clamp(1 - span(p, 0.62, 1), 0, 1));
        L.outN[k].style.filter = mb(p * p * 5.5 * (1 - span(p, 0.7, 1)));
      }

      for (k = 0; k < ni; k++) {
        var lead2 = 0.14 + u(k, ni) * 0.16;
        s = span(t, lead2, 1);
        var b = E.outBounce(s);
        // squash exactly at the contacts: b touches 1 at each landing, and the window closes
        // as s -> 1 so the settled frame is never squashed
        var near = Math.max(0, 1 - Math.abs(1 - b) * 26) * Math.min(1, (1 - s) * 3);
        L.inN[k].style.transform = 'translateY(' + (-H * 0.72 * (1 - b)).toFixed(1) + 'px)'
          + ' scale(' + (1 + 0.09 * near).toFixed(3) + ',' + (1 - 0.14 * near).toFixed(3) + ')';
        L.inN[k].style.opacity = String(clamp(span(t, lead2, lead2 + 0.08), 0, 1));
        L.inN[k].style.filter = mb(speed(E.outBounce, s) * 1.6);
      }
    },
    rest: function (stage) { settle(stage._l); },
  });

  /* ---- 3. Magnetic Snap ---------------------------------------------------
     A pole switches on at the centre of the stage. The old characters are repelled and thrown
     clear; the new ones are already scattered around the frame and are drawn in — creeping,
     then yanked, overshooting the pole and settling. Each character has its own hashed delay,
     so at mid-transition roughly half the line has already landed and is readable while the
     rest is still flying in. */
  FX.register({
    id: 'kinetic-magnet',
    name: 'Magnetic Snap',
    family: 'Kinetic',
    blurb: 'A pole switches on: old letters are thrown clear, new ones are pulled in and snap home.',
    duration: 1300,
    dwell: 4600,
    theme: { bg: '#0E1013', fg: '#EFF2EC' },
    setup: function (stage, ctx) {
      var L = build(stage, ctx, { split: true, chars: true });
      var glow = el('div', null, {
        position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
        pointerEvents: 'none', opacity: '0', willChange: 'opacity',
        background: 'radial-gradient(46% 62% at 50% 50%, rgba(122,182,72,.30), rgba(122,182,72,.07) 55%, rgba(122,182,72,0) 78%)',
      });
      L.wrap.appendChild(glow);
      L.glow = glow;
      stage._l = L;
    },
    frame: function (stage, t, ctx) {
      var L = stage._l, H = Math.max(120, ctx.H);
      var no = L.outN.length, ni = L.inN.length, k, a, r, p, s, m;
      L.out.style.opacity = '1'; L.in.style.opacity = '1';
      L.glow.style.opacity = (Math.sin(Math.PI * clamp(t, 0, 1)) * 0.9).toFixed(3);

      for (k = 0; k < no; k++) {
        a = rnd(k, 4) * Math.PI * 2;
        r = (0.45 + 0.55 * rnd(k, 5)) * H;
        p = span(t, rnd(k, 6) * 0.30, rnd(k, 6) * 0.30 + 0.40);
        var e = inQuart(p);
        L.outN[k].style.transform =
          'translate(' + (Math.cos(a) * r * e).toFixed(1) + 'px,' + (Math.sin(a) * r * e).toFixed(1) + 'px)'
          + ' rotate(' + ((rnd(k, 7) * 2 - 1) * 90 * e).toFixed(1) + 'deg)'
          + ' scale(' + (1 - 0.5 * e).toFixed(3) + ')';
        L.outN[k].style.opacity = String(clamp(1 - span(p, 0.45, 0.95), 0, 1));
      }

      for (k = 0; k < ni; k++) {
        a = rnd(k, 1) * Math.PI * 2;
        r = (0.35 + 0.45 * rnd(k, 2)) * H;
        var lead = 0.08 + rnd(k, 3) * 0.42;
        s = span(t, lead, lead + 0.45);
        m = magnet(s);
        L.inN[k].style.transform =
          'translate(' + (Math.cos(a) * r * (1 - m)).toFixed(1) + 'px,' + (Math.sin(a) * r * (1 - m)).toFixed(1) + 'px)'
          + ' rotate(' + ((rnd(k, 8) * 2 - 1) * 40 * (1 - m)).toFixed(1) + 'deg)'
          + ' scale(' + (1 - 0.45 * (1 - m)).toFixed(3) + ')';
        L.inN[k].style.opacity = String(clamp(0.35 + s * 2.2, 0, 1) * (s > 0 ? 1 : 0));
        L.inN[k].style.filter = mb(speed(magnet, s) * 1.1);
      }
    },
    rest: function (stage) { settle(stage._l); stage._l.glow.style.opacity = '0'; },
  });

  /* ---- 4. Pendulum Swing --------------------------------------------------
     Both lines hang from one pivot at the top of the stage. The old line is pushed away and
     swings up and out of frame; the new one falls in from the far side, crosses the bottom of
     its arc and rings to a stop. They are always on opposite sides of the pivot, so the frame
     where both are visible reads as an exchange, not as a double exposure. */
  FX.register({
    id: 'kinetic-pendulum',
    name: 'Pendulum Swing',
    family: 'Kinetic',
    blurb: 'The old line swings up and away on its rod; the new one falls in and rings to a stop.',
    duration: 1350,
    dwell: 4600,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = el('div', 'fx-wrap', { position: 'relative', width: '100%', height: '100%' });
      function arm(s) {
        var a = el('div', null, {
          position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          transformOrigin: '50% 0%', willChange: 'transform,opacity',
        });
        var rod = el('div', null, {
          position: 'absolute', top: '0', left: '50%', width: '1.5px', height: '38%',
          marginLeft: '-0.75px',
          background: 'linear-gradient(180deg, rgba(122,182,72,.75), rgba(122,182,72,.10))',
        });
        var d = el('div', 'fx-line', { position: 'absolute', left: '0', right: '0', textAlign: 'center' });
        var lt = el('span', 'fx-light'); lt.textContent = s.light;
        var bd = el('span', 'fx-bold'); bd.textContent = s.bold;
        d.appendChild(lt); d.appendChild(el('br')); d.appendChild(bd);
        a.appendChild(rod); a.appendChild(d);
        wrap.appendChild(a);
        return a;
      }
      var out = arm(ctx.from), inn = arm(ctx.to);
      var pin = el('div', null, {
        position: 'absolute', top: '-3px', left: '50%', width: '9px', height: '9px',
        marginLeft: '-4.5px', borderRadius: '50%', background: GREEN,
        boxShadow: '0 0 10px rgba(122,182,72,.7)', pointerEvents: 'none',
      });
      wrap.appendChild(pin);
      stage.appendChild(wrap);
      stage._p = { out: out, in: inn, pin: pin };
    },
    frame: function (stage, t) {
      var P = stage._p;
      // a short anticipation the wrong way, then the push
      var back = 7 * Math.sin(Math.PI * span(t, 0, 0.20));
      var swing = 64 * E.inCubic(span(t, 0.05, 0.62));
      var ao = back - swing;
      P.out.style.transform = 'rotate(' + ao.toFixed(2) + 'deg)';
      P.out.style.opacity = String(clamp(1 - span(t, 0.30, 0.64), 0, 1));

      var s = span(t, 0.20, 1);
      var q = damped(s, 7.4, 4.0);
      var ai = 58 * (1 - q);
      P.in.style.transform = 'rotate(' + (-ai).toFixed(2) + 'deg)';
      P.in.style.opacity = String(clamp(span(t, 0.20, 0.36), 0, 1));
      P.pin.style.opacity = String(0.25 + 0.75 * Math.sin(Math.PI * clamp(t, 0, 1)));
    },
    rest: function (stage) {
      var P = stage._p;
      P.out.style.opacity = '0';
      P.in.style.transform = 'rotate(0deg)';
      P.in.style.opacity = '1';
      P.pin.style.opacity = '0.25';
    },
  });

  /* ---- 5. Inertial Whip ---------------------------------------------------
     The line behaves like a rope: the head goes first and every character behind it lags a
     little more, so the text stretches, cracks off to one side, and the replacement is thrown
     in with the tail arriving last and overshooting hardest. Mid-transition the new line is
     already legible at the head while its tail is still catching up — the stretch IS the
     picture, so there is nothing to hide. */
  FX.register({
    id: 'kinetic-whip',
    name: 'Inertial Whip',
    family: 'Kinetic',
    blurb: 'The line stretches like a rope and cracks away; the new one snaps in tail-last.',
    duration: 1200,
    dwell: 4400,
    setup: function (stage, ctx) { stage._l = build(stage, ctx, { split: true, chars: true }); },
    frame: function (stage, t, ctx) {
      var L = stage._l, W = Math.max(240, ctx.W), H = Math.max(120, ctx.H);
      var no = L.outN.length, ni = L.inN.length, k, uu, p, s, q;
      L.out.style.opacity = '1'; L.in.style.opacity = '1';

      for (k = 0; k < no; k++) {
        uu = u(k, no);
        p = span(t, uu * 0.17, uu * 0.17 + 0.42);
        var e = E.inQuad(p);
        L.outN[k].style.transform =
          'translate(' + (-W * 1.15 * e).toFixed(1) + 'px,' + (-H * 0.16 * uu * Math.sin(Math.PI * p)).toFixed(1) + 'px)'
          + ' rotate(' + (12 * uu * e).toFixed(2) + 'deg)';
        L.outN[k].style.opacity = String(clamp(1 - span(p, 0.55, 1), 0, 1));
        L.outN[k].style.filter = mb(speed(E.inQuad, p) * 2.0);
      }

      for (k = 0; k < ni; k++) {
        uu = u(k, ni);
        var lead = 0.22 + uu * 0.22;
        s = span(t, lead, 1);
        var w = 7.6 + 3.2 * uu, z = 5.4 - 1.8 * uu;      // the tip rings longer and wider
        q = damped(s, w, z);
        L.inN[k].style.transform =
          'translate(' + (W * 0.95 * (1 - q)).toFixed(1) + 'px,'
          + (-H * 0.14 * uu * Math.sin(Math.PI * s)).toFixed(1) + 'px)'
          + ' rotate(' + (-10 * uu * (1 - q)).toFixed(2) + 'deg)';
        L.inN[k].style.opacity = String(clamp(span(t, lead, lead + 0.09) * 1.3, 0, 1));
        L.inN[k].style.filter = mb(speed(function (v) { return damped(v, w, z); }, s) * 1.3);
      }
    },
    rest: function (stage) { settle(stage._l); },
  });

  /* ---- 6. Domino Chain ----------------------------------------------------
     Each word is a domino standing on its bottom-left corner. The first is tipped, and the
     fall runs along the line — but every word that goes down has already been replaced by the
     new one standing up behind it, half a beat later. The result is one travelling wavefront
     with old text ahead of it and new text behind it, and no point on the line is ever bare. */
  FX.register({
    id: 'kinetic-domino',
    name: 'Domino Chain',
    family: 'Kinetic',
    blurb: 'A toppling wave runs the line; each word knocks the next as its replacement stands up.',
    duration: 1400,
    dwell: 4600,
    setup: function (stage, ctx) {
      var L = build(stage, ctx, { split: true });
      origin(L.outN, '0% 100%');
      origin(L.inN, '0% 100%');
      stage._l = L;
    },
    frame: function (stage, t) {
      var L = stage._l;
      var no = L.outN.length, ni = L.inN.length, k, p, q;
      L.out.style.opacity = '1'; L.in.style.opacity = '1';

      for (k = 0; k < no; k++) {
        var lead = u(k, no) * 0.55;
        p = span(t, lead, lead + 0.30);
        var f = E.inQuad(p);                             // tips slowly, then falls
        L.outN[k].style.transform = 'rotate(' + (86 * f).toFixed(2) + 'deg)'
          + ' scale(' + (1 - 0.10 * f).toFixed(3) + ')';
        L.outN[k].style.opacity = String(clamp(1 - span(p, 0.45, 0.85), 0, 1));
      }

      for (k = 0; k < ni; k++) {
        var lead2 = 0.12 + u(k, ni) * 0.52;
        q = damped(span(t, lead2, lead2 + 0.34), 6.6, 5.2);
        L.inN[k].style.transform = 'rotate(' + (-76 * (1 - q)).toFixed(2) + 'deg)'
          + ' scale(' + (1 - 0.08 * (1 - q)).toFixed(3) + ')';
        L.inN[k].style.opacity = String(clamp(span(t, lead2, lead2 + 0.10) * 1.4, 0, 1));
      }
    },
    rest: function (stage) { settle(stage._l); },
  });
})();
