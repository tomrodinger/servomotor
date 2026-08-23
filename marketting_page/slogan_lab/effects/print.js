/* Family: Print — paper, ink, and the machines that put one on the other.
 *
 * Six transitions, all PURE FUNCTIONS OF PROGRESS: no timers, no CSS animation, no wall clock,
 * no Math.random. Every "random" quantity is hashed from an index, so frame(i, t) draws the
 * identical picture every time and can be seeked, screenshotted and exported to GIF.
 *
 *   print-letterpress  every letter is struck into the paper and released as its replacement
 *   print-riso         two spot-colour layers pull out of register, swap, and pull back in
 *   print-stamp        a rubber stamp descends, lands crooked, and lifts off leaving new ink
 *   print-inkjet       a carriage lays the line down in horizontal passes, band by band
 *   print-tear         the printed sheet tears along a ragged edge; the next line is underneath
 *   print-fold         each row creases away on a hinge and comes back printed differently
 *
 * The house rule this file was written against: NO DEAD FRAMES. At every t the viewer sees
 * legible text, or text plainly in the act of becoming other text. Where a moment is fully
 * obscured (the stamp at impact) something deliberate and solid occupies the frame.
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;

  var GREEN = '#7AB648';                        // Gearotons green
  var PINK = '#E4497F';                         // the other riso drum
  var SERIF = '"Iowan Old Style", "Palatino Linotype", Palatino, Georgia, serif';
  var SANS = '"Avenir Next", "Segoe UI", Inter, system-ui, sans-serif';

  /* ------------------------------------------------------------------ tiny utils */

  /* deterministic pseudo-random in [0,1) — never Math.random() */
  function rnd(i, s) {
    var x = Math.sin((i + 1) * 127.1 + (s + 1) * 311.7) * 43758.5453;
    return x - Math.floor(x);
  }
  function srnd(i, s) { return rnd(i, s) * 2 - 1; }

  /* 0 at the ends, 1 in the middle — the shape of a press stroke or a crease */
  function bump(p) { return Math.sin(Math.PI * clamp(p, 0, 1)); }
  /* the engine's ease table has no in-quart; a press needs one, so here it is locally */
  function inQuart(p) { p = clamp(p, 0, 1); return p * p * p * p; }
  function n2(v) { return Math.round(v * 100) / 100; }
  function n3(v) { return Math.round(v * 1000) / 1000; }

  /* ------------------------------------------------------------------ builders */

  /* Full-stage relative wrapper. Everything else is absolutely positioned inside it, so two
     copies of the line can occupy exactly the same place without disturbing each other. */
  function wrapper(stage, extra) {
    stage.innerHTML = '';
    var css = {
      position: 'relative', width: '100%', height: '100%', overflow: 'hidden',
    };
    if (extra) for (var k in extra) css[k] = extra[k];
    var w = FX.el('div', 'pr-wrap', css);
    stage.appendChild(w);
    return w;
  }

  /* Two rows, exactly as the engine's canonical layer draws them.
     NEVER put a <br> between the halves: the engine forces a lone fx-light/fx-bold pair to
     display:block, so a <br> as well leaves an EMPTY third line between the rows — the effect's
     layer then sits looser and higher than the canon, and every transition snaps at both ends.
     Own the block display here instead, and the two layers line up to the pixel. */
  function twoRows(inner, s) {
    var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light;
    var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
    inner.appendChild(a); inner.appendChild(b);
    return { light: a, bold: b };
  }

  /* One full-stage centred copy of a slogan: light line over bold line. The returned handles
     let an effect re-text the layer in place (a hard swap while the layer is hidden or far out
     of register is how several of these change what is printed without a cross-fade smear). */
  function sheet(wrap, s, css) {
    var d = FX.el('div', 'pr-sheet', {
      position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
      willChange: 'transform, opacity',
    });
    if (css) for (var k in css) d.style[k] = css[k];
    var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
    var r = twoRows(inner, s);
    d.appendChild(inner);
    wrap.appendChild(d);
    return {
      el: d, inner: inner, light: r.light, bold: r.bold,
      set: function (t2) { r.light.textContent = t2.light; r.bold.textContent = t2.bold; },
    };
  }

  /* --------------------------------------------------- 1. letterpress ----------- */
  /* A press strikes one character at a time, left to right across both rows. The glyph sinks,
     the paper takes the impression (a highlight below the stroke, a shadow above it), and it
     comes back up carrying the new letter. The swap happens at the bottom of the stroke, where
     the type is deepest in the paper — so the change is *caused* by the press, and no frame is
     ever a cross-fade of two alphabets.
     Each cell is one glyph in a box whose width is measured for BOTH letters at setup and then
     interpolated across the strike. Padding every cell to the wider glyph instead (the obvious
     fix for jitter) visibly loosens the tracking at rest — "Inside" prints as "I nside" — and
     rest has to be exact, so the width travels with the press. */
  FX.register({
    id: 'print-letterpress',
    name: 'Letterpress',
    family: 'Print',
    blurb: 'A press strikes each letter into the paper and releases it as its replacement.',
    duration: 1250,
    dwell: 4400,
    theme: { bg: '#F1EDE3', fg: '#241F18', font: SERIF },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', null, {
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        width: '100%', height: '100%',
      });
      var col = FX.el('div', null, { width: '100%', textAlign: 'center' });
      stage._cells = [];

      function row(a, b, cls) {
        var line = FX.el('div', cls, null);
        var groups = FX.alignWords(a, b);
        groups.forEach(function (g, gi) {
          var word = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
          g.pairs.forEach(function (pr) {
            var cell = FX.el('span', null, {
              display: 'inline-block', whiteSpace: 'pre', textAlign: 'center',
              willChange: 'transform, width',
            });
            cell.textContent = pr[0];
            word.appendChild(cell);
            stage._cells.push({
              c: cell, a: pr[0], b: pr[1], blank: !(pr[0] || '').trim() && !(pr[1] || '').trim(),
            });
          });
          line.appendChild(word);
          if (gi < groups.length - 1) {
            var sp = FX.el('span', null, { whiteSpace: 'pre' });
            sp.textContent = ' ';
            line.appendChild(sp);
          }
        });
        col.appendChild(line);
      }
      row(ctx.from.light, ctx.to.light, 'fx-light');
      row(ctx.from.bold, ctx.to.bold, 'fx-bold');
      wrap.appendChild(col);
      stage.appendChild(wrap);

      /* Two measuring passes, two layouts total: read every "before" width, re-text the whole
         line, read every "after" width. Reading and writing in bulk keeps this off the
         layout-thrash path even for a 60-character slogan.
         Widths are stored in EM, not px. Glyph advance scales linearly with font size, so an em
         width stays correct if the stage is later resized — the lab's focus button enlarges the
         type without rebuilding the effect, and frozen px widths would collide every glyph into
         its neighbour and leave the RESTING line an unreadable smear. */
      var C = stage._cells, k;
      var fs = parseFloat(getComputedStyle(stage).fontSize) || 26;
      for (k = 0; k < C.length; k++) C[k].wa = C[k].c.getBoundingClientRect().width / fs;
      for (k = 0; k < C.length; k++) C[k].c.textContent = C[k].b;
      for (k = 0; k < C.length; k++) C[k].wb = C[k].c.getBoundingClientRect().width / fs;
      /* alignWords pads the shorter word with spaces, and a padding cell that keeps a space's
         advance loosens the tracking: "Bolts in like" would rest as "Bolts   in    like". Real
         inter-word gaps are separate spans, so any space INSIDE a group is padding — give it zero
         width at the end it is padding on. Both ends then set the two slogans exactly, and a
         letter with no counterpart grows out of nothing under the press, which is what a press
         does anyway. */
      for (k = 0; k < C.length; k++) {
        if (!(C[k].a || '').trim()) C[k].wa = 0;
        if (!(C[k].b || '').trim()) C[k].wb = 0;
      }
      for (k = 0; k < C.length; k++) {
        C[k].c.textContent = C[k].a;
        C[k].c.style.width = n3(C[k].wa) + 'em';
      }
    },
    frame: function (stage, t) {
      var C = stage._cells, n = C.length;
      for (var k = 0; k < n; k++) {
        var c = C[k];
        var lead = (k / Math.max(1, n)) * 0.66;
        var p = span(t, lead, lead + 0.34);
        var d = bump(p);                                   // press depth 0..1..0
        var down = p >= 0.5;
        if (c.c.textContent !== (down ? c.b : c.a)) c.c.textContent = down ? c.b : c.a;
        // the cell breathes from one glyph's width to the other's across the strike
        c.c.style.width = n3(lerp(c.wa, c.wb, E.inOutQuad(span(p, 0.34, 0.66)))) + 'em';
        c.c.style.transform = 'translateY(' + n3(0.11 * d) + 'em) scale(' + n3(1 - 0.13 * d) + ')';
        if (c.blank) continue;
        // the impression: paper lifts on the underside of the stroke, shadow gathers above it
        c.c.style.textShadow = d > 0.004
          ? '0 ' + n3(0.06 * d) + 'em 0 rgba(255,255,255,' + n2(0.98 * d) + '),'
            + ' 0 ' + n3(-0.045 * d) + 'em ' + n3(0.06 * d) + 'em rgba(0,0,0,' + n2(0.5 * d) + ')'
          : 'none';
        // a whisper of blur only at the very bottom of the stroke, where the swap happens
        c.c.style.filter = d > 0.8 ? 'blur(' + n2(2.2 * (d - 0.8)) + 'px)' : 'none';
      }
    },
    rest: function (stage) {
      stage._cells.forEach(function (c) {
        c.c.textContent = c.b;
        c.c.style.width = n3(c.wb) + 'em';
        c.c.style.transform = 'none'; c.c.style.textShadow = 'none'; c.c.style.filter = 'none';
      });
    },
  });

  /* ------------------------------------------------------ 2. risograph ---------- */
  /* A riso prints one spot colour per pass, and the paper never comes back through in quite the
     same place — hence the fringe. Here registration collapses on purpose: the black key layer
     drops out, the green and pink drums pull apart, each one re-inks itself with the new line
     while it is far out of register (so the two copies never smear over each other), and then
     they walk back into register with one small settling wobble before the key returns.
     Both colour layers always carry the SAME line at any instant, so an offset pair reads as
     misregistration, which is the whole point, rather than as two competing texts. */
  FX.register({
    id: 'print-riso',
    name: 'Risograph',
    family: 'Print',
    blurb: 'Two spot-colour layers slip out of register, re-ink, and walk back into alignment.',
    duration: 1500,
    dwell: 4600,
    theme: { bg: '#F7F3E8', fg: '#1B1A18', font: SANS },
    setup: function (stage, ctx) {
      // isolate so mixBlendMode multiplies against this paper and nothing outside it
      var wrap = wrapper(stage, { background: '#F7F3E8', isolation: 'isolate' });
      stage._key = sheet(wrap, ctx.from);
      stage._g = sheet(wrap, ctx.from, { color: GREEN, mixBlendMode: 'multiply' });
      stage._p = sheet(wrap, ctx.from, { color: PINK, mixBlendMode: 'multiply' });
      stage._ctx = ctx;
    },
    frame: function (stage, t, ctx) {
      var key = stage._key, G = stage._g, P = stage._p;

      /* key layer: the crisp black print. Present only at the two ends; it re-texts itself
         at the midpoint while it is completely invisible. */
      var kOut = 1 - span(t, 0.02, 0.16);
      var kIn = span(t, 0.86, 0.99);
      key.set(t < 0.5 ? ctx.from : ctx.to);
      key.el.style.opacity = String(Math.max(kOut, kIn));

      /* colour layers: alive for everything in between */
      var cv = Math.min(span(t, 0.03, 0.15), span(1 - t, 0.02, 0.13));
      G.el.style.opacity = String(cv);
      P.el.style.opacity = String(cv);

      /* separation: out of register at the middle, dead-on at both ends */
      var s = Math.pow(bump(t), 0.62);
      // the green drum re-inks first, the pink one a beat later — both while far apart
      G.set(t < 0.40 ? ctx.from : ctx.to);
      P.set(t < 0.58 ? ctx.from : ctx.to);

      // one small damped settling wobble as the paper finds its stops again
      var w = t > 0.72 ? Math.exp(-9 * (t - 0.72)) * Math.cos(19 * (t - 0.72)) * 0.07 : 0;

      var gx = -1.30 * s - w, gy = -0.30 * s - w * 0.4;
      var px = 1.30 * s + w, py = 0.30 * s + w * 0.4;
      G.el.style.transform = 'translate(' + n3(gx) + 'em,' + n3(gy) + 'em) rotate(' + n3(-0.8 * s) + 'deg)';
      P.el.style.transform = 'translate(' + n3(px) + 'em,' + n3(py) + 'em) rotate(' + n3(0.7 * s) + 'deg)';
    },
    rest: function (stage, ctx) {
      stage._key.set(ctx.to);
      stage._key.el.style.opacity = '1';
      stage._g.el.style.opacity = '0';
      stage._p.el.style.opacity = '0';
      stage._g.el.style.transform = 'none';
      stage._p.el.style.transform = 'none';
    },
  });

  /* --------------------------------------------------------- 3. rubber stamp ---- */
  /* The stamp looms in out of focus, carrying the new line on its face. It lands crooked at the
     midpoint — that impact is the only fully-obscured instant, and what occupies it is a solid
     inked pad, not a hole — the paper jolts, ink spatters out of the edges, and the pad lifts
     away leaving the new line printed slightly off-square before it settles. */
  FX.register({
    id: 'print-stamp',
    name: 'Rubber Stamp',
    family: 'Print',
    blurb: 'A rubber stamp drops out of focus, lands crooked with a spatter, and lifts away.',
    duration: 1150,
    dwell: 4200,
    theme: { bg: '#F3EFE5', fg: '#211D17', font: SANS },
    setup: function (stage, ctx) {
      var wrap = wrapper(stage);

      function die(s) {
        var box = FX.el('div', null, {
          position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          willChange: 'transform, opacity, filter',
        });
        var inner = FX.el('div', null, {
          position: 'relative', display: 'inline-block', maxWidth: '100%',
          textAlign: 'center', padding: '0.16em 0.34em',
        });
        var pad = FX.el('div', null, {
          position: 'absolute', left: '0', top: '0', right: '0', bottom: '0',
          background: 'rgba(40,34,28,.16)', border: '2px solid rgba(40,34,28,.30)',
          borderRadius: '5px', opacity: '0', pointerEvents: 'none',
        });
        inner.appendChild(pad);
        twoRows(inner, s);
        box.appendChild(inner);
        wrap.appendChild(box);
        return { el: box, inner: inner, pad: pad };
      }

      stage._old = die(ctx.from);
      stage._new = die(ctx.to);
      stage._new.el.style.opacity = '0';

      // ink spatter: hashed once, laid out in an ellipse that hugs the line
      stage._dots = [];
      for (var k = 0; k < 20; k++) {
        var ang = rnd(k, 1) * Math.PI * 2;
        var rr = 0.35 + 0.65 * rnd(k, 2);
        var sz = 0.045 + 0.10 * rnd(k, 3) * rnd(k, 3);
        var d = FX.el('div', null, {
          position: 'absolute', left: '50%', top: '50%',
          width: sz + 'em', height: sz + 'em', borderRadius: '50%',
          background: 'currentColor', opacity: '0', pointerEvents: 'none',
          willChange: 'transform, opacity',
        });
        wrap.appendChild(d);
        stage._dots.push({
          el: d,
          x: Math.cos(ang) * (1.9 + 4.4 * rr),
          y: Math.sin(ang) * (0.85 + 1.5 * rr),
          lag: 0.06 * rnd(k, 4),
        });
      }
    },
    frame: function (stage, t) {
      var O = stage._old, N = stage._new, D = stage._dots;

      // approach: big, soft and faint, coming down onto the sheet
      var app = E.inQuad(span(t, 0, 0.5));
      var land = span(t, 0.5, 1);

      if (t < 0.5) {
        N.el.style.transform = 'scale(' + n3(lerp(1.85, 1.04, app)) + ') rotate(' + n2(lerp(-7, -3.4, app)) + 'deg)';
        N.el.style.filter = 'blur(' + n2(lerp(7, 0.2, app)) + 'px)';
        N.el.style.opacity = String(lerp(0, 0.42, E.inQuad(app)));
        N.pad.style.opacity = String(0.7 * app);
      } else {
        // impact, then the settle: crooked and squashed, ringing down to square
        var st = E.outCubic(land);
        var ring = Math.exp(-6 * land) * Math.cos(11 * land);
        N.el.style.transform =
          'scale(' + n3(1 + 0.035 * ring) + ',' + n3(1 - 0.05 * ring) + ')'
          + ' rotate(' + n2(-3.4 * (1 - st) - 0.9 * ring) + 'deg)';
        N.el.style.filter = 'none';
        N.el.style.opacity = '1';
        // the pad peels off in the first third of the settle
        var lift = span(t, 0.5, 0.78);
        N.pad.style.opacity = String(0.7 * (1 - E.outQuad(lift)));
        N.pad.style.transform = 'scale(' + n3(1 + 0.10 * lift) + ')';
      }

      // the old line survives right up to the strike, then is buried under the ink
      var crush = inQuart(span(t, 0.30, 0.5));
      O.el.style.opacity = String(t >= 0.5 ? 0 : 1 - 0.55 * crush);
      O.el.style.transform = 'scale(' + n3(1 - 0.02 * crush) + ')';
      O.el.style.filter = crush > 0 ? 'blur(' + n2(1.4 * crush) + 'px)' : 'none';

      // spatter flies out of the impact and dries
      for (var k = 0; k < D.length; k++) {
        var d = D[k];
        var q = span(t, 0.5 + d.lag, 0.94 + d.lag);
        var f = E.outQuint(q);
        d.el.style.transform = 'translate(' + n3(d.x * f) + 'em,' + n3(d.y * f) + 'em)';
        d.el.style.opacity = String(q <= 0 || q >= 1 ? 0 : (1 - q) * 0.75);
      }
    },
    rest: function (stage) {
      var N = stage._new;
      stage._old.el.style.opacity = '0';
      N.el.style.opacity = '1';
      N.el.style.transform = 'none';
      N.el.style.filter = 'none';
      N.pad.style.opacity = '0';
      stage._dots.forEach(function (d) { d.el.style.opacity = '0'; });
    },
  });

  /* ------------------------------------------------------------ 4. inkjet ------- */
  /* The carriage lays the line down one horizontal band at a time, snaking left-to-right then
     right-to-left the way a real head does. Behind the nozzles the new line, ahead of them the
     old one — so mid-transition the reader sees a single line that is part old and part new,
     which is exactly "in the act of becoming".
     The band stack covers only the union of the two lines' boxes, measured at setup, so every
     pass does real work instead of crossing blank paper. */
  FX.register({
    id: 'print-inkjet',
    name: 'Inkjet Passes',
    family: 'Print',
    blurb: 'A print head lays the new line down band by band, snaking back and forth.',
    duration: 1600,
    dwell: 4400,
    theme: { bg: '#FBFAF6', fg: '#1B1B1D', font: SANS },
    setup: function (stage, ctx) {
      var wrap = wrapper(stage);
      var H = Math.max(60, stage.clientHeight || ctx.H || 160);

      function face(s, y0) {
        var f = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0', top: (-y0) + 'px', height: H + 'px',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          willChange: 'clip-path',
        });
        var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
        var a = FX.el('span', 'fx-light'); a.textContent = s.light + ' ';
        var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
        inner.appendChild(a); inner.appendChild(FX.el('br')); inner.appendChild(b);
        f.appendChild(inner);
        return { el: f, inner: inner };
      }

      // measure the union of both lines so the bands only cover inked territory
      var m1 = face(ctx.from, 0), m2 = face(ctx.to, 0);
      wrap.appendChild(m1.el); wrap.appendChild(m2.el);
      var wr = wrap.getBoundingClientRect();
      var r1 = m1.inner.getBoundingClientRect(), r2 = m2.inner.getBoundingClientRect();
      var top = Math.min(r1.top, r2.top) - wr.top - 5;
      var bot = Math.max(r1.bottom, r2.bottom) - wr.top + 5;
      wrap.removeChild(m1.el); wrap.removeChild(m2.el);
      if (!(bot > top)) { top = H * 0.25; bot = H * 0.75; }
      top = Math.max(0, Math.round(top));
      bot = Math.min(H, Math.round(bot));

      var band = bot - top;
      var n = clamp(Math.round(band / 26), 4, 9);
      stage._bands = [];
      for (var k = 0; k < n; k++) {
        var y0 = top + Math.round(k * band / n);
        var y1 = top + Math.round((k + 1) * band / n);
        var slot = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0',
          top: y0 + 'px', height: (y1 - y0) + 'px', overflow: 'hidden',
        });
        var fo = face(ctx.from, y0), fi = face(ctx.to, y0);
        slot.appendChild(fo.el); slot.appendChild(fi.el);
        wrap.appendChild(slot);
        stage._bands.push({ out: fo.el, in: fi.el, y0: y0, y1: y1, dir: k % 2 === 0 ? 1 : -1 });
      }

      // the carriage: a dark block with a lit nozzle strip on its leading face
      var head = FX.el('div', null, {
        position: 'absolute', left: '0', top: '0', width: '1.55em', height: '10px',
        background: 'linear-gradient(180deg,#3A3A40,#1C1C21)',
        borderRadius: '3px', boxShadow: '0 2px 7px rgba(0,0,0,.30)',
        opacity: '0', pointerEvents: 'none', willChange: 'transform, opacity',
      });
      var noz = FX.el('div', null, {
        position: 'absolute', top: '14%', bottom: '14%', width: '3px',
        background: GREEN, boxShadow: '0 0 7px rgba(122,182,72,.85)', borderRadius: '2px',
      });
      head.appendChild(noz);
      // the damp stripe of ink just laid down
      var wet = FX.el('div', null, {
        position: 'absolute', left: '0', top: '0', width: '3.4em', height: '10px',
        opacity: '0', pointerEvents: 'none', willChange: 'transform, opacity',
      });
      wrap.appendChild(wet); wrap.appendChild(head);
      stage._head = head; stage._noz = noz; stage._wet = wet;
      stage._n = n;
    },
    frame: function (stage, t, ctx) {
      var B = stage._bands, n = B.length;
      var W = Math.max(1, stage.clientWidth || ctx.W);
      var p = span(t, 0.03, 0.99);
      var raw = p * n;
      var bi = Math.min(n - 1, Math.floor(raw));
      var u = clamp(raw - bi, 0, 1);

      for (var k = 0; k < n; k++) {
        var b = B[k];
        var done = k < bi ? 1 : (k > bi ? 0 : u);
        var q = E.inOutQuad(done) * 0.25 + done * 0.75;     // slight ease at the turnarounds
        var pct = q * 100;
        if (b.dir > 0) {
          b.in.style.clipPath = 'inset(0 ' + n2(100 - pct) + '% 0 0)';
          b.out.style.clipPath = 'inset(0 0 0 ' + n2(pct) + '%)';
        } else {
          b.in.style.clipPath = 'inset(0 0 0 ' + n2(100 - pct) + '%)';
          b.out.style.clipPath = 'inset(0 ' + n2(pct) + '% 0 0)';
        }
      }

      var cur = B[bi];
      var qq = E.inOutQuad(u) * 0.25 + u * 0.75;
      var xf = cur.dir > 0 ? qq : 1 - qq;                    // 0..1 across the stage
      var hw = 0.775;                                        // half the head width, em
      var fs = parseFloat(getComputedStyle(stage).fontSize) || 26;
      var x = xf * W - hw * fs;
      var hh = (cur.y1 - cur.y0) + 6;
      var vis = Math.min(span(t, 0, 0.05), span(1 - t, 0, 0.05));

      stage._head.style.height = hh + 'px';
      stage._head.style.transform = 'translate(' + n2(x) + 'px,' + (cur.y0 - 3) + 'px)';
      stage._head.style.opacity = String(vis);
      stage._noz.style.left = cur.dir > 0 ? 'auto' : '2px';
      stage._noz.style.right = cur.dir > 0 ? '2px' : 'auto';

      stage._wet.style.height = (cur.y1 - cur.y0) + 'px';
      stage._wet.style.transform = 'translate(' + n2(cur.dir > 0 ? x - 3.4 * fs : x + 1.55 * fs) + 'px,' + cur.y0 + 'px)';
      stage._wet.style.background = cur.dir > 0
        ? 'linear-gradient(90deg, rgba(122,182,72,0), rgba(122,182,72,.20))'
        : 'linear-gradient(270deg, rgba(122,182,72,0), rgba(122,182,72,.20))';
      stage._wet.style.opacity = String(vis);
    },
    rest: function (stage) {
      stage._bands.forEach(function (b) {
        b.in.style.clipPath = 'none';
        b.out.style.clipPath = 'inset(0 100% 0 0)';
      });
      stage._head.style.opacity = '0';
      stage._wet.style.opacity = '0';
    },
  });

  /* --------------------------------------------------------------- 5. tear ------ */
  /* Physically honest layering: the next line is already printed on the sheet UNDERNEATH. The
     top sheet tears along a ragged edge that walks left to right; the torn-off piece is opaque
     paper with the old line on it, so nothing shows through it, and it carries a white fibre
     rim and a drop shadow before it curls away.
     Both halves of the tear share one jitter table, so the two ragged edges are the same edge. */
  var TEAR_PAPER = '#EFEAE0';
  FX.register({
    id: 'print-tear',
    name: 'Paper Tear',
    family: 'Print',
    blurb: 'The printed sheet tears along a ragged edge and lifts away; the next line is beneath.',
    duration: 1350,
    dwell: 4600,
    theme: { bg: TEAR_PAPER, fg: '#221E18', font: SANS },
    setup: function (stage, ctx) {
      var wrap = wrapper(stage);

      // bottom sheet: the new line, always fully printed, simply covered up at first
      sheet(wrap, ctx.to, { background: TEAR_PAPER });

      // the part of the top sheet still attached (right of the tear)
      stage._keep = sheet(wrap, ctx.from, { background: TEAR_PAPER, willChange: 'clip-path' });

      // the torn-off piece (left of the tear): opaque paper that moves as one
      var piece = FX.el('div', null, {
        position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
        background: TEAR_PAPER, willChange: 'transform, clip-path, opacity, filter',
      });
      var pi = FX.el('div', null, {
        position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
      });
      var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
      var a = FX.el('span', 'fx-light'); a.textContent = ctx.from.light + ' ';
      var b = FX.el('span', 'fx-bold'); b.textContent = ctx.from.bold;
      inner.appendChild(a); inner.appendChild(FX.el('br')); inner.appendChild(b);
      pi.appendChild(inner); piece.appendChild(pi);
      // the raw fibre along the torn edge: a sliver of near-white clipped inside the piece
      var fib = FX.el('div', null, {
        position: 'absolute', top: '0', left: '0', right: '0', bottom: '0',
        background: 'rgba(255,255,255,.72)', willChange: 'clip-path', pointerEvents: 'none',
      });
      piece.appendChild(fib);
      wrap.appendChild(piece);
      stage._piece = piece; stage._fib = fib;

      // one jitter table, shared by every polygon so the edges match exactly
      var N = 26;
      stage._jit = [];
      for (var k = 0; k <= N; k++) {
        stage._jit.push(srnd(k, 5) * 1.5 + srnd(k, 11) * 0.85 + srnd(k, 23) * 0.4);
      }
      stage._N = N;
    },
    frame: function (stage, t) {
      var N = stage._N, J = stage._jit;
      var p = E.inOutQuad(t) * 0.35 + t * 0.65;             // the tear travels honestly
      var X = lerp(-4, 104, p);

      var left = ['-25% -12%'], right = [], fibr = [];
      for (var k = 0; k <= N; k++) {
        var y = -10 + (120 * k) / N;
        var x = X + J[k];
        left.push(n2(x) + '% ' + n2(y) + '%');
        right.push(n2(x) + '% ' + n2(y) + '%');
        fibr.push(n2(x - 1.05) + '% ' + n2(y) + '%');
      }
      left.push('-25% 112%');
      right.push('125% 112%', '125% -12%');
      var leftPoly = 'polygon(' + left.join(',') + ')';
      // the fibre sliver: inside the piece, but right of a slightly-retreated copy of the edge
      var fibPoly = 'polygon(' + fibr.join(',') + ',125% 112%,125% -12%)';

      stage._keep.el.style.clipPath = 'polygon(' + right.join(',') + ')';
      stage._piece.style.clipPath = leftPoly;
      stage._fib.style.clipPath = fibPoly;

      // the piece pulls away from the tear, then curls off in the last stretch
      var go = E.inCubic(span(t, 0.52, 1));
      var dx = -(2.4 * p + 46 * go);
      var dy = -(1.2 * p + 16 * go);
      var rot = -(2.4 * p + 11 * go);
      stage._piece.style.transformOrigin = n2(clamp(X, 0, 100)) + '% 50%';
      stage._piece.style.transform =
        'translate(' + n2(dx) + 'px,' + n2(dy) + 'px) rotate(' + n2(rot) + 'deg)';
      stage._piece.style.opacity = String(1 - E.inQuad(span(t, 0.74, 1)));
      stage._piece.style.filter = t > 0.005
        ? 'drop-shadow(' + n2(2 + 5 * go) + 'px ' + n2(3 + 6 * go) + 'px ' + n2(3 + 7 * go) + 'px rgba(60,48,32,.34))'
        : 'none';
    },
    rest: function (stage) {
      stage._keep.el.style.clipPath = 'polygon(125% -12%,125% -12%,125% 112%,125% 112%)';
      stage._piece.style.opacity = '0';
      stage._piece.style.filter = 'none';
    },
  });

  /* --------------------------------------------------------------- 6. fold ------ */
  /* Each row of the slogan is a strip of paper hinged at its top edge. The strip creases away
     from the reader, and while it is at the bottom of the crease — deep in shadow, foreshortened,
     with a bright fold line running along the hinge — the print on it changes. Then it unfolds
     flat again carrying the new words.
     The two rows are offset in time, so one row is always close to flat and plainly readable
     while the other is mid-crease; and the fold stops short of edge-on, so no row ever collapses
     to nothing. */
  FX.register({
    id: 'print-fold',
    name: 'Crease Fold',
    family: 'Print',
    blurb: 'Each row creases away on a paper hinge and unfolds carrying different words.',
    duration: 1450,
    dwell: 4600,
    theme: { bg: '#F4F1E7', fg: '#211E17', font: SANS },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', null, {
        position: 'relative', width: '100%', height: '100%',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        perspective: '760px',
      });
      var col = FX.el('div', null, {
        width: '100%', textAlign: 'center', transformStyle: 'preserve-3d',
      });
      stage._rows = [];

      function row(a, b, cls) {
        var holder = FX.el('div', null, {
          position: 'relative', transformStyle: 'preserve-3d',
        });
        var leaf = FX.el('div', null, {
          position: 'relative', transformOrigin: '50% 0%',
          willChange: 'transform', backfaceVisibility: 'hidden',
        });
        var txt = FX.el('div', cls, { position: 'relative' });
        txt.textContent = a;
        var shade = FX.el('div', null, {
          position: 'absolute', left: '-8%', right: '-8%', top: '0', bottom: '-6%',
          background: 'linear-gradient(180deg, rgba(30,24,14,0) 0%, rgba(30,24,14,.10) 34%,'
            + ' rgba(30,24,14,.42) 100%)',
          opacity: '0', pointerEvents: 'none',
        });
        leaf.appendChild(txt); leaf.appendChild(shade);
        // the crease itself: a lit fold line that lives on the hinge
        var crease = FX.el('div', null, {
          position: 'absolute', left: '-8%', right: '-8%', top: '0', height: '3px',
          background: 'linear-gradient(180deg, rgba(255,255,255,.95), rgba(120,100,70,.42))',
          opacity: '0', pointerEvents: 'none',
        });
        holder.appendChild(leaf); holder.appendChild(crease);
        col.appendChild(holder);
        stage._rows.push({ leaf: leaf, txt: txt, shade: shade, crease: crease, a: a, b: b });
      }
      row(ctx.from.light, ctx.to.light, 'fx-light');
      row(ctx.from.bold, ctx.to.bold, 'fx-bold');
      wrap.appendChild(col);
      stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      var R = stage._rows;
      for (var k = 0; k < R.length; k++) {
        var r = R[k];
        var lead = k * 0.30;
        var p = span(t, lead, lead + 0.70);
        // one crease down and back; sharpened so the deep part of the fold is brief
        var d = Math.pow(bump(p), 1.35);
        var ang = -62 * d;
        r.txt.textContent = (p > 0 && p < 1 && p >= 0.5) || p >= 1 ? r.b : r.a;
        r.leaf.style.transform = 'rotateX(' + n2(ang) + 'deg)';
        r.shade.style.opacity = String(0.92 * d);
        r.crease.style.opacity = String(Math.min(1, 1.5 * d));
      }
    },
    rest: function (stage) {
      stage._rows.forEach(function (r) {
        r.txt.textContent = r.b;
        r.leaf.style.transform = 'none';
        r.shade.style.opacity = '0';
        r.crease.style.opacity = '0';
      });
    },
  });
})();
