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
      var a = FX.el('span', 'fx-light', { display: 'block' }); a.textContent = s.light + ' ';
      var b = FX.el('span', 'fx-bold', { display: 'block' }); b.textContent = s.bold;
      if (split) { d.appendChild(a); d.appendChild(b); }
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
        /* The inter-word gap is a CELL too, not a fixed space.
         *
         * alignWords pads the word LIST as well as the letters: turning a four-word slogan into a
         * seven-word one gives three groups that are entirely blank on one side. Those groups
         * collapse to nothing at that end, but the space in front of each of them did not, so the
         * settled line carried up to three phantom spaces. On a centred row that drags the text
         * sideways — measured 8-13px off canonical on the six slogan pairs whose two texts differ
         * in word count. Making the gap a cell whose two sides are '' when the group it introduces
         * is blank on that side lets it collapse with the group it belongs to.
         */
        var nxt = groups[gi + 1].pairs;
        var liveA = nxt.some(function (q) { return (q[0] || '').trim(); });
        var liveB = nxt.some(function (q) { return (q[1] || '').trim(); });
        var sp = FX.el('span', null, { display: 'inline-block', whiteSpace: 'pre' });
        sp.textContent = ' ';
        sp._a = liveA ? ' ' : '';
        sp._b = liveB ? ' ' : '';
        sp._gap = true;
        row.appendChild(sp);
        sink.push(sp);
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
        /* The stagger has to FIT INSIDE the transition with room at both ends.
           It used to run lead 0..0.55 with a 0.45 window, i.e. the first cell started churning at
           t=0 and the last one settled at exactly t=1.0. So the first effect frame already showed
           a random glyph instead of the outgoing character, and the last frame was still showing
           one instead of the incoming character — the headline changed width at both ends because
           the substituted glyphs are not the width of the letters they stand in for. Starting at
           0.04 and finishing by 0.95 leaves both endpoints settled on the real text. */
        var lead = 0.04 + (k / n) * 0.49;
        var p = span(t, lead, lead + 0.42);
        if (c._gap) {
          /* A gap never scrambles — it just has to be the right width at each end, and hold a
             single space in between so the words do not run together mid-transition. */
          c.textContent = p >= 1 ? c._b : p <= 0 ? c._a : (c._a || c._b);
          continue;
        }
        /* A padding cell renders as NOTHING at the end it is padding on, not as a space.
           alignWords pads the shorter of each word pair with blank cells so the two spellings line
           up; a blank cell that keeps a space's advance widens the settled line, so the headline
           breathed out by 8-11px the instant the effect took over and back in at the end. Only the
           pairs whose two slogans differ in word length showed it, which is why the two-pair audit
           missed it and the all-pairs sweep did not. print-letterpress solves the same problem by
           giving those cells an explicit zero width. */
        if (p >= 1) { c.textContent = (c._b || '').trim() ? c._b : ''; c.style.opacity = '1'; }
        else if (p <= 0) { c.textContent = (c._a || '').trim() ? c._a : ''; c.style.opacity = '1'; }
        else if (!(c._a || '').trim() && !(c._b || '').trim()) { c.textContent = ''; }
        else {
          c.textContent = GLYPHS[Math.floor(stage._rnd(k, Math.floor(p * 9)) * GLYPHS.length)];
          c.style.opacity = String(lerp(0.45, 1, p));
        }
      }
    },
  });

  /* ---- 3. Mechanical odometer roll, per character ------------------------- */
  /* Two things about the drum are not obvious, and the naive version got both wrong.
   *
   * WIDTH. alignWords pads the shorter of each word pair with blank cells, and a drum cell sized
   * by shrink-to-fit takes the width of the WIDER of its two glyphs. So the outgoing line carried
   * a permanent blank slot everywhere the incoming word was longer, and vice versa: pair 0 opened
   * 75px wider than the canon and closed 65px wider, pair 12 closed 177px wider. The headline
   * visibly re-spaced itself against the canonical text at BOTH ends of the transition — exactly
   * the snap the canonical rest state exists to prevent. Each cell now carries the measured
   * advance of its top glyph AND of its bottom glyph and tweens between them on the roll's own
   * clock, so the slot is the outgoing advance at t=0 and the incoming advance at t=1.
   *
   * HEIGHT. The clip window has to be taller than one line or the overflow shears the ascenders
   * and descenders off. But a taller inline-block grows the line box with it, which pushed the two
   * rows apart and lifted the whole block ~6px off where the canon draws it. Negative vertical
   * margins pull the MARGIN box back to exactly one canonical line while the border box — and so
   * the clip window — stays tall: line box, and therefore baseline, unchanged.
   */

  /* Drum geometry read off the stage rather than hard-coded, because the lab, the rater and the
     live page do not all run at the same line-height. */
  function drumGeom(stage) {
    var cs = getComputedStyle(stage);
    var fs = parseFloat(cs.fontSize) || 26;
    var lh = parseFloat(cs.lineHeight);
    var lhEm = (lh && isFinite(lh) && fs) ? lh / fs : 1.15;
    var h = Math.max(lhEm + 0.16, 1.2);   // window: one line plus room for ascender + descender
    return { h: h, lh: lhEm, m: (h - lhEm) / 2 };
  }

  function buildDrumRow(parent, aText, bText, cls, geom, sink) {
    var row = FX.el('div', cls, { display: 'block' });
    var groups = FX.alignWords(aText, bText);
    /* which side of each word pair actually has text — pad groups exist when one slogan has
       fewer words than the other, and their separating space must not be rendered on the side
       that has nothing to separate */
    function has(gi, side) {
      return groups[gi].pairs.some(function (pr) { return !!pr[side]; });
    }
    function later(gi, side) {
      for (var j = gi + 1; j < groups.length; j++) if (has(j, side)) return true;
      return false;
    }

    function cell(chA, chB, host) {
      var c = FX.el('span', null, {
        display: 'inline-block', overflow: 'hidden', verticalAlign: 'top', whiteSpace: 'pre',
        height: geom.h + 'em', lineHeight: geom.h + 'em',
        marginTop: (-geom.m) + 'em', marginBottom: (-geom.m) + 'em',
      });
      var strip = FX.el('span', null, { display: 'block', willChange: 'transform' });
      var top = FX.el('span', null, { display: 'block', height: geom.h + 'em', whiteSpace: 'pre' });
      var bot = FX.el('span', null, { display: 'block', height: geom.h + 'em', whiteSpace: 'pre' });
      top.textContent = chA; bot.textContent = chB;
      strip.appendChild(top); strip.appendChild(bot);
      c.appendChild(strip);
      host.appendChild(c);
      sink.push({ cell: c, strip: strip, top: top, bot: bot, wA: 0, wB: 0 });
    }

    groups.forEach(function (g, gi) {
      var w = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
      g.pairs.forEach(function (pr) { cell(pr[0] || '', pr[1] || '', w); });
      row.appendChild(w);
      if (gi < groups.length - 1) {
        // the inter-word gap is a drum too, so a word present on only one side opens and closes
        // its space instead of leaving a permanent blank in the other line
        cell(has(gi, 0) && later(gi, 0) ? ' ' : '', has(gi, 1) && later(gi, 1) ? ' ' : '', row);
      }
    });
    parent.appendChild(row);
  }

  /* Read each slot's two natural advances once, at setup, by showing one glyph at a time. Batched
     into two write/read passes so the whole thing costs two layouts, not one per character.
     Kept in em rather than px: the stage's font-size is a vw clamp, so a px width measured at one
     viewport would freeze the wrong slot size after a resize. */
  function measureDrums(stage, drums) {
    var fs = parseFloat(getComputedStyle(stage).fontSize) || 26;
    var i;
    for (i = 0; i < drums.length; i++) {
      drums[i].cell.style.width = 'auto';
      drums[i].bot.style.display = 'none';
    }
    for (i = 0; i < drums.length; i++) drums[i].wA = drums[i].cell.getBoundingClientRect().width / fs;
    for (i = 0; i < drums.length; i++) {
      drums[i].bot.style.display = 'block';
      drums[i].top.style.display = 'none';
    }
    for (i = 0; i < drums.length; i++) drums[i].wB = drums[i].cell.getBoundingClientRect().width / fs;
    for (i = 0; i < drums.length; i++) {
      drums[i].top.style.display = 'block';
      drums[i].cell.style.width = drums[i].wA + 'em';
    }
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
      stage._geom = drumGeom(stage);
      // two rows so the headline keeps its canonical two-line shape mid-transition
      buildDrumRow(d, ctx.from.light, ctx.to.light, 'fx-light', stage._geom, stage._drums);
      buildDrumRow(d, ctx.from.bold, ctx.to.bold, 'fx-bold', stage._geom, stage._drums);
      wrap.appendChild(d); stage.appendChild(wrap);
      measureDrums(stage, stage._drums);
    },
    frame: function (stage, t) {
      var D = stage._drums, n = D.length, h = stage._geom.h;
      for (var k = 0; k < n; k++) {
        /* Every drum is stationary until t=0.05 and has landed by t=0.90. The old schedule started
           the first drums at t=0, so by the audit's first sampled frame they had already rolled 7%
           and the incoming glyph was peeking into the window — a headline that was subtly wrong
           before the transition had visibly begun. */
        var lead = 0.05 + (k / n) * 0.40;
        var q = span(t, lead, lead + 0.45);
        D[k].strip.style.transform = 'translateY(' + (-E.outBack(q) * h) + 'em)';
        D[k].cell.style.width = lerp(D[k].wA, D[k].wB, E.inOutQuad(q)) + 'em';
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
