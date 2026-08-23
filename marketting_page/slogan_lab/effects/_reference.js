/* Reference effects — the models new effects should be written against.
 * Each is a pure function of progress t. No timers, no setTimeout, no CSS animations:
 * if it isn't a function of t it can't be seeked, screenshotted, or exported to GIF.
 */
(function () {
  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp;

  /* Two stacked layers is the most common structure: old on top, new underneath.
     `.fx-line` is absolutely positioned so the two never affect each other's layout. */
  function twoLayer(stage, ctx, split) {
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
      var a = FX.el('span', 'fx-light'); a.textContent = s.light + ' ';
      var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
      if (split) { d.appendChild(a); d.appendChild(FX.el('br')); d.appendChild(b); }
      else { d.appendChild(a); d.appendChild(b); }
      wrap.appendChild(d);
      return d;
    }
    var out = line(ctx.from), inn = line(ctx.to);
    stage.appendChild(wrap);
    return { out: out, in: inn };
  }

  /* ---- 1. Keynote-style dissolve with a lift and a blur -------------------- */
  FX.register({
    id: 'keynote-lift',
    name: 'Keynote Lift',
    family: 'Dissolve',
    blurb: 'Outgoing lifts and blurs away; incoming rises into focus.',
    duration: 1000,
    setup: function (stage, ctx) { stage._l = twoLayer(stage, ctx, true); },
    frame: function (stage, t, ctx) {
      var L = stage._l;
      var a = E.inOutCubic(span(t, 0, .55));
      var b = E.outCubic(span(t, .35, 1));
      L.out.style.opacity = String(1 - a);
      L.out.style.transform = 'translateY(' + (-14 * a) + 'px) scale(' + (1 - .012 * a) + ')';
      L.out.style.filter = 'blur(' + (6 * a) + 'px)';
      L.in.style.opacity = String(b);
      L.in.style.transform = 'translateY(' + (16 * (1 - b)) + 'px) scale(' + (1 - .012 * (1 - b)) + ')';
      L.in.style.filter = 'blur(' + (6 * (1 - b)) + 'px)';
    },
  });

  /* ---- 2. Per-character glyph scramble, settling left to right ------------
     Uses alignWords so each word is an inline-block that wraps as a unit — a naive per-character
     split lets the browser break a word in half mid-transition, which looks broken. */
  /* One row of character cells, morphing rowA -> rowB. Used by the per-character effects so the
     headline keeps its two-line shape all the way through the transition. */
  function buildRow(parent, aText, bText, cls, sink) {
    var row = FX.el('div', cls, { display: 'block' });
    var groups = FX.alignWords(aText, bText);
    groups.forEach(function (g, gi) {
      var w = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
      g.pairs.forEach(function (pr) {
        var c = FX.el('span', null, { display: 'inline-block', whiteSpace: 'pre' });
        c.textContent = pr[0];
        c._a = pr[0]; c._b = pr[1];
        w.appendChild(c);
        sink.push(c);
      });
      row.appendChild(w);
      if (gi < groups.length - 1) {
        var sp = FX.el('span', null, { whiteSpace: 'pre' });
        sp.textContent = ' ';
        row.appendChild(sp);
      }
    });
    parent.appendChild(row);
    return row;
  }

  var GLYPHS = '#$%&*+=<>/\\|@01_-';
  FX.register({
    id: 'glyph-scramble',
    name: 'Glyph Scramble',
    family: 'Type',
    blurb: 'Characters churn through symbols and settle left to right.',
    duration: 1300,
    theme: { font: 'ui-monospace, SFMono-Regular, Menlo, monospace' },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'fx-wrap', {
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        height: '100%', width: '100%',
      });
      var d = FX.el('div', null, { textAlign: 'center' });
      stage._cells = [];
      // Two rows, matching the canonical two-line headline. Merging the halves into one string
      // made the line snap 2->1 at the start of the transition and back at the end.
      buildRow(d, ctx.from.light, ctx.to.light, 'fx-light', stage._cells);
      buildRow(d, ctx.from.bold, ctx.to.bold, 'fx-bold', stage._cells);
      wrap.appendChild(d); stage.appendChild(wrap);
      stage._rnd = function (k, s) { var x = Math.sin(k * 12.9898 + s * 78.233) * 43758.5453; return x - Math.floor(x); };
    },
    frame: function (stage, t) {
      var n = stage._cells.length;
      for (var k = 0; k < n; k++) {
        var c = stage._cells[k];
        var lead = (k / n) * 0.55;
        var p = span(t, lead, lead + 0.45);
        if (p >= 1) { c.textContent = c._b; c.style.opacity = '1'; }
        else if (p <= 0) { c.textContent = c._a; c.style.opacity = '1'; }
        else if (!(c._a || '').trim() && !(c._b || '').trim()) { c.textContent = ''; }
        else {
          c.textContent = GLYPHS[Math.floor(stage._rnd(k, Math.floor(p * 9)) * GLYPHS.length)];
          c.style.opacity = String(lerp(0.45, 1, p));
        }
      }
    },
  });

  /* ---- 3. Mechanical odometer roll, per character ------------------------- */
  function buildDrumRow(parent, aText, bText, cls, sink) {
    var row = FX.el('div', cls, { display: 'block' });
    var groups = FX.alignWords(aText, bText);
    groups.forEach(function (g, gi) {
      var w = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
      g.pairs.forEach(function (pr) {
        var cell = FX.el('span', null, {
          display: 'inline-block', overflow: 'hidden', verticalAlign: 'top',
          height: '1.18em', lineHeight: '1.18em', whiteSpace: 'pre',
        });
        var strip = FX.el('span', null, { display: 'block', willChange: 'transform' });
        var top = FX.el('span', null, { display: 'block', height: '1.18em', whiteSpace: 'pre' });
        var bot = FX.el('span', null, { display: 'block', height: '1.18em', whiteSpace: 'pre' });
        top.textContent = pr[0]; bot.textContent = pr[1];
        strip.appendChild(top); strip.appendChild(bot);
        cell.appendChild(strip);
        w.appendChild(cell);
        sink.push(strip);
      });
      row.appendChild(w);
      if (gi < groups.length - 1) {
        var sp = FX.el('span', null, { whiteSpace: 'pre', display: 'inline-block' });
        sp.textContent = ' ';
        row.appendChild(sp);
      }
    });
    parent.appendChild(row);
  }

  FX.register({
    id: 'odometer',
    name: 'Odometer Roll',
    family: 'Mechanical',
    blurb: 'Every character rolls vertically to its replacement, like a counter drum.',
    duration: 1200,
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var wrap = FX.el('div', 'fx-wrap', {
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        height: '100%', width: '100%',
      });
      var d = FX.el('div', null, { textAlign: 'center' });
      stage._drums = [];
      // two rows so the headline keeps its canonical two-line shape mid-transition
      buildDrumRow(d, ctx.from.light, ctx.to.light, 'fx-light', stage._drums);
      buildDrumRow(d, ctx.from.bold, ctx.to.bold, 'fx-bold', stage._drums);
      wrap.appendChild(d); stage.appendChild(wrap);
    },
    frame: function (stage, t) {
      var n = stage._drums.length;
      for (var k = 0; k < n; k++) {
        var lead = (k / n) * 0.45;
        var p = E.outBack(span(t, lead, lead + 0.5));
        stage._drums[k].style.transform = 'translateY(' + (-p * 1.18) + 'em)';
      }
    },
  });

  /* ---- 4. Ink soaking into paper (mask-driven) ---------------------------- */
  FX.register({
    id: 'ink-bleed',
    name: 'Ink Bleed',
    family: 'Reveal',
    blurb: 'The new line soaks up through the paper; the old one blots away.',
    duration: 1400,
    theme: { bg: '#FBFAF7', fg: '#26231F', accent: '#6E9F3F',
             font: 'Optima, "Avenir Next", Palatino, Georgia, serif' },
    setup: function (stage, ctx) { stage._l = twoLayer(stage, ctx, true); },
    frame: function (stage, t, ctx) {
      var L = stage._l;
      var a = E.inOutQuad(span(t, 0, .5));
      var b = E.outCubic(span(t, .3, 1));
      // outgoing: bleeds outward and fades, as if the paper drank it
      L.out.style.opacity = String(1 - a);
      L.out.style.filter = 'blur(' + (5 * a) + 'px) saturate(' + (1 - .5 * a) + ')';
      L.out.style.transform = 'scale(' + (1 + .02 * a) + ')';
      // incoming: an irregular wet edge rising from the baseline
      var pct = Math.round(b * 130) - 15;
      L.in.style.opacity = String(Math.min(1, b * 1.25));
      L.in.style.filter = 'blur(' + (2.2 * (1 - b)) + 'px)';
      L.in.style.webkitMaskImage = L.in.style.maskImage =
        'radial-gradient(120% 78% at 50% 108%, #000 ' + Math.max(0, pct - 22) + '%, ' +
        'rgba(0,0,0,.55) ' + pct + '%, transparent ' + (pct + 16) + '%)';
    },
    rest: function (stage) {
      var L = stage._l;
      L.out.style.opacity = '0';
      L.in.style.opacity = '1';
      L.in.style.filter = 'none';
      L.in.style.webkitMaskImage = L.in.style.maskImage = 'none';
    },
  });
})();
