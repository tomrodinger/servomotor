/* Mechanical family — analogue displays and machinery.
 *
 * Six effects, all PURE FUNCTIONS OF PROGRESS: split-flap board, 3D flip cards, a word-level
 * odometer, a dot-matrix LED rebuild, a typewriter carriage return, and a slide-rule vernier lock.
 * No timers, no CSS animations, no wall clock — every pixel is derived from t.
 */
(function () {
  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;

  /* deterministic pseudo-random from (index, salt) — never Math.random() */
  function h1(k, s) {
    var x = Math.sin((k + 1) * 127.1 + (s + 1) * 311.7) * 43758.5453;
    return x - Math.floor(x);
  }

  /* full text of a slogan half-pair */
  function full(s) { return s.light + ' ' + s.bold; }

  function box(stage, css) {
    stage.innerHTML = '';
    var w = FX.el('div', 'fx-wrap', {
      position: 'relative', width: '100%', height: '100%', overflow: 'hidden',
    });
    if (css) for (var k in css) w.style[k] = css[k];
    stage.appendChild(w);
    return w;
  }

  /* an absolutely-positioned, vertically+horizontally centred text box filling its parent */
  function centred(parent) {
    var d = FX.el('div', null, {
      position: 'absolute', left: '0', top: '0', right: '0', bottom: '0',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
    });
    parent.appendChild(d);
    var inner = FX.el('div', null, { textAlign: 'center', maxWidth: '100%' });
    d.appendChild(inner);
    return { box: d, inner: inner };
  }

  function twoWeights(inner, s) {
    var a = FX.el('span', 'fx-light'); a.textContent = s.light + ' ';
    var b = FX.el('span', 'fx-bold'); b.textContent = s.bold;
    inner.appendChild(a); inner.appendChild(b);
  }

  /* The canonical rest state is TWO rows — light half above, bold half below. Any effect that
     builds its own per-character machinery has to lay those rows out itself, otherwise the halves
     reflow into one paragraph and the headline snaps 2->1 the moment a transition starts. */
  function twoRows(inner) {
    var a = FX.el('div', 'fx-light', { whiteSpace: 'normal' });
    var b = FX.el('div', 'fx-bold', { whiteSpace: 'normal' });
    inner.appendChild(a); inner.appendChild(b);
    return [a, b];
  }

  /* How much of an effect's own hardware (cards, panels, rails) is on show. 0 at both endpoints so
     the frames either side of the engine's canonical text are plain text too, 1 across the middle. */
  function chromeAt(t, inA, outA) {
    return clamp(Math.min(t / inA, (1 - t) / outA), 0, 1);
  }

  /* ================================================================= 1. SPLIT-FLAP BOARD ==== */
  /* Solari departure board. Every cell is a real two-flap card: the top flap falls away showing
     the next glyph's top half behind it, then the bottom flap swings up. Each cell riffles
     through its own deterministic number of intermediate glyphs before landing on the target. */

  var CH = 1.16;                 /* card height, em */
  var CM = 0.01;                 /* card margin, em — set so the row pitch equals the canon's */
  var CG = 0.02;                 /* gap between a card and its cell edge, em */

  function flapSeq(a, b, k) {
    var set = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
    if (/[a-z]/.test(b)) set = 'abcdefghijklmnopqrstuvwxyz';
    if (/[0-9]/.test(b)) set = '0123456789';
    var n = 3 + Math.floor(h1(k, 1) * 5);
    var seq = [a];
    for (var i = 0; i < n; i++) seq.push(set.charAt(Math.floor(h1(k, i + 2) * set.length)));
    seq.push(b);
    return seq;
  }

  function flapHalf(isTop) {
    var d = FX.el('div', null, {
      position: 'absolute', left: CG + 'em', right: CG + 'em', height: (CH / 2) + 'em',
      overflow: 'hidden', background: 'transparent',
      borderRadius: isTop ? '.07em .07em 0 0' : '0 0 .07em .07em',
      backfaceVisibility: 'hidden',
    });
    d.style[isTop ? 'top' : 'bottom'] = '0';
    var txt = FX.el('div', null, {
      position: 'absolute', left: '0', right: '0', height: CH + 'em',
      lineHeight: CH + 'em', textAlign: 'center', whiteSpace: 'pre',
    });
    txt.style.top = isTop ? '0' : (-CH / 2) + 'em';
    d.appendChild(txt);
    return { box: d, txt: txt };
  }

  FX.register({
    id: 'mechanical-split-flap',
    name: 'Split-Flap Board',
    family: 'Mechanical',
    blurb: 'Solari departure board: every card riffles through glyphs and clacks into place.',
    duration: 1650,
    dwell: 4200,
    theme: {
      bg: '#0B0C0D', fg: '#F3EEE2',
      font: 'ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace',
    },
    setup: function (stage, ctx) {
      var w = box(stage);
      var c = centred(w);
      stage._cells = [];
      stage._gaps = [];
      var rows = twoRows(c.inner);

      /* Measure the real monospace advance rather than assuming .6em: the cell pitch has to equal
         the canonical text's character pitch to the pixel, or a long line wraps one word earlier
         than the canon does and the headline jumps a whole line at the endpoints. */
      var probe = FX.el('span', null, { position: 'absolute', visibility: 'hidden', whiteSpace: 'pre' });
      probe.textContent = 'MMMMMMMMMM';
      c.inner.appendChild(probe);
      var ADV = probe.getBoundingClientRect().width / 10 || 12;
      c.inner.removeChild(probe);
      stage._adv = ADV;

      function hasSide(g, side) {
        return g.pairs.some(function (p) { return !!(p[side] || '').trim(); });
      }

      /* One board row per half of the headline, so the two lines of the canonical rest state
         survive the whole transition. */
      function buildRow(host, a, b) {
        var groups = FX.alignWords(a, b);
        groups.forEach(function (g, gi) {
          var word = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
          g.pairs.forEach(function (pr) {
            var k = stage._cells.length;
            var cell = FX.el('span', null, {
              position: 'relative', display: 'inline-block', verticalAlign: 'top',
              width: ADV + 'px', height: CH + 'em', margin: CM + 'em 0',
              perspective: '7em', overflow: 'hidden',
            });
            var blank = (!(pr[0] || '').trim() && !(pr[1] || '').trim());
            var st = flapHalf(true), sb = flapHalf(false);
            var ft = flapHalf(true), fb = flapHalf(false);
            ft.box.style.transformOrigin = '50% 100%';
            fb.box.style.transformOrigin = '50% 0%';
            ft.box.style.willChange = fb.box.style.willChange = 'transform';
            cell.appendChild(st.box); cell.appendChild(sb.box);
            cell.appendChild(ft.box); cell.appendChild(fb.box);
            /* seam sits above the flaps, like the axle slot on a real board */
            var seam = FX.el('div', null, {
              position: 'absolute', left: '0', right: '0', top: 'calc(50% - .5px)',
              height: '1px', background: 'transparent',
            });
            cell.appendChild(seam);
            word.appendChild(cell);
            stage._cells.push({
              st: st.txt, sb: sb.txt, ft: ft, fb: fb, blank: blank, seam: seam, cell: cell,
              halves: [st.box, sb.box, ft.box, fb.box],
              /* a cell that only exists on one side of the pair is a card the board is adding or
                 pulling out: its slot closes as it flips, so each endpoint measures exactly the
                 text that is actually there — no ghost width left over from the other slogan */
              wa: !!(pr[0] || '').trim(), wb: !!(pr[1] || '').trim(),
              seq: blank ? [' ', ' '] : flapSeq(pr[0], pr[1], k),
            });
          });
          host.appendChild(word);
          if (gi < groups.length - 1) {
            var sp = FX.el('span', null, { display: 'inline-block', width: ADV + 'px' });
            host.appendChild(sp);
            stage._gaps.push({
              el: sp,
              a: hasSide(g, 0) && hasSide(groups[gi + 1], 0),
              b: hasSide(g, 1) && hasSide(groups[gi + 1], 1),
            });
          }
        });
      }
      buildRow(rows[0], ctx.from.light, ctx.to.light);
      buildRow(rows[1], ctx.from.bold, ctx.to.bold);
    },
    frame: function (stage, t) {
      var cells = stage._cells, n = cells.length, A = stage._adv;
      /* the cards themselves fade up out of plain text and back into it, so the frames either
         side of the engine's canonical endpoints are plain text on the same grid */
      var chrome = chromeAt(t, 0.07, 0.10);
      var cardBg = 'rgba(28,29,32,' + chrome.toFixed(3) + ')';
      var seamBg = 'rgba(8,9,10,' + chrome.toFixed(3) + ')';
      var gp = E.inOutQuad(span(t, 0.08, 0.84));
      for (var gi = 0; gi < stage._gaps.length; gi++) {
        var G = stage._gaps[gi];
        G.el.style.width = (A * lerp(G.a ? 1 : 0, G.b ? 1 : 0, gp)).toFixed(3) + 'px';
      }
      for (var k = 0; k < n; k++) {
        var c = cells[k];
        var lead = 0.08 + (k / Math.max(1, n)) * 0.30 + h1(k, 9) * 0.04;
        var p = E.inOutQuad(span(t, lead, lead + 0.46));
        var wf = c.wa && c.wb ? 1 : (c.wa ? 1 - p : (c.wb ? p : 0));
        c.cell.style.width = (A * wf).toFixed(3) + 'px';
        var steps = c.seq.length - 1;
        var f = Math.min(p * steps, steps);
        var si = Math.min(Math.floor(f), steps - 1);
        var u = clamp(f - si, 0, 1);
        var cur = c.seq[si], nxt = c.seq[si + 1];

        /* with a see-through card the hidden half must not double the glyph: before the flap
           starts falling the static top still reads the OLD glyph, after it lands the static
           bottom already reads the NEW one */
        c.st.textContent = u > 0 ? nxt : cur;
        c.sb.textContent = u >= 1 ? nxt : cur;
        c.ft.txt.textContent = cur;
        c.fb.txt.textContent = nxt;

        if (c.blank) {
          c.ft.box.style.transform = 'rotateX(0deg)';
          c.fb.box.style.transform = 'rotateX(0deg)';
          c.ft.box.style.opacity = '1'; c.fb.box.style.opacity = '1';
          c.ft.box.style.filter = c.fb.box.style.filter = 'none';
          continue;   /* a pad cell holds width only — never a card, never a seam */
        }
        for (var q = 0; q < 4; q++) c.halves[q].style.background = cardBg;
        c.seam.style.background = seamBg;
        /* the fall is fast, the swing-up lands with a little bite */
        if (u < 0.5) {
          var a = E.inQuad(u / 0.5);
          c.ft.box.style.opacity = '1';
          c.ft.box.style.transform = 'rotateX(' + (-90 * a) + 'deg)';
          c.ft.box.style.filter = 'brightness(' + (1 - 0.55 * a) + ')';
          c.fb.box.style.opacity = '0';
          c.fb.box.style.transform = 'rotateX(90deg)';
        } else {
          var b = E.outQuart((u - 0.5) / 0.5);
          c.ft.box.style.opacity = '0';
          c.ft.box.style.transform = 'rotateX(-90deg)';
          c.fb.box.style.opacity = '1';
          c.fb.box.style.transform = 'rotateX(' + (90 - 90 * b) + 'deg)';
          c.fb.box.style.filter = 'brightness(' + (0.45 + 0.55 * b) + ')';
        }
      }
    },
    rest: function (stage) {
      stage._cells.forEach(function (c) {
        var last = c.seq[c.seq.length - 1];
        c.st.textContent = last; c.sb.textContent = last;
        for (var q = 0; q < 4; q++) c.halves[q].style.background = 'transparent';
        c.seam.style.background = 'transparent';
        c.ft.txt.textContent = last; c.fb.txt.textContent = last;
        c.ft.box.style.transform = 'rotateX(0deg)';
        c.ft.box.style.opacity = '1'; c.ft.box.style.filter = 'none';
        c.fb.box.style.transform = 'rotateX(0deg)';
        c.fb.box.style.opacity = '1'; c.fb.box.style.filter = 'none';
      });
    },
  });

  /* ==================================================================== 2. FLIP CARDS ======= */
  /* A wall of hinged tiles. Each character is a physical card on a vertical hinge; the wave runs
     left to right, cards dip back into the board as they spin and pop forward as they land. */

  FX.register({
    id: 'mechanical-flip-cards',
    name: 'Flip Cards',
    family: 'Mechanical',
    blurb: 'Hinged tiles spin on their vertical axis in a wave, dipping back into the board.',
    duration: 1400,
    dwell: 4200,
    theme: { bg: '#E7E4DC', fg: '#191A1C' },
    setup: function (stage, ctx) {
      var w = box(stage, { perspective: '1600px' });
      var c = centred(w);
      c.box.style.transformStyle = 'preserve-3d';
      stage._board = c.box;
      stage._cards = [];
      stage._gaps = [];
      var rows = twoRows(c.inner);

      /* one word-space, measured in the page font — the cards have to add up to the same width
         as the canonical line or the row wraps somewhere the canon does not */
      var probe = FX.el('span', null, { position: 'absolute', visibility: 'hidden', whiteSpace: 'pre' });
      probe.textContent = ' ';
      c.inner.appendChild(probe);
      var SPW = probe.getBoundingClientRect().width || 8;
      c.inner.removeChild(probe);

      function hasSide(g, side) {
        return g.pairs.some(function (p) { return !!(p[side] || '').trim(); });
      }

      function buildRow(host, a, b) {
        var groups = FX.alignWords(a, b);
        groups.forEach(function (g, gi) {
          var word = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
          g.pairs.forEach(function (pr) {
            /* The cell owns the layout width; the card floats above it, centred, so a card can
               grow, shrink or spin without dragging the line around with it. The width itself is
               animated from the FROM glyph's width to the TO glyph's, which is what makes the
               first and last frames measure exactly like the engine's canonical text. */
            var cell = FX.el('span', null, {
              position: 'relative', display: 'inline-block', verticalAlign: 'top',
              perspective: '520px', height: '1.18em', width: '0px',
            });
            var card = FX.el('span', null, {
              position: 'absolute', left: '0', top: '0', width: '0', height: '0',
              transformStyle: 'preserve-3d', willChange: 'transform',
            });
            /* NB: no `filter` anywhere on the card or its faces. A filter turns the used value of
               transform-style into `flat`, which kills backface-visibility — the far face then
               shows through mirrored. Shading is an overlay, the shadow is a box-shadow. */
            function face(ch, back) {
              var f = FX.el('span', null, {
                position: 'absolute', left: '0', top: '0', display: 'block', whiteSpace: 'pre',
                textAlign: 'center', lineHeight: '1.18em', padding: '0',
                backfaceVisibility: 'hidden', WebkitBackfaceVisibility: 'hidden',
                borderRadius: '2px',
              });
              var g = FX.el('span', null, { position: 'relative', zIndex: '1' });
              g.textContent = ch || ' ';
              /* A slot the other slogan needs but this one does not: the card simply is not there
                 on this face. Left visible it would measure a space wide and park a blank tile in
                 the middle of a word at the very endpoint the canon renders as clean text. */
              if (!(ch || '').trim()) f.style.visibility = 'hidden';
              f.appendChild(g);
              var shade = FX.el('span', null, {
                position: 'absolute', left: '0', top: '0', right: '0', bottom: '0',
                borderRadius: '2px', background: '#0B0C0E', opacity: '0', zIndex: '2',
                pointerEvents: 'none',
              });
              f.appendChild(shade);
              card.appendChild(f);
              return { el: f, shade: shade, back: back };
            }
            var fa = face(pr[0], false), fb = face(pr[1], true);
            cell.appendChild(card);
            word.appendChild(cell);
            stage._cards.push({
              cell: cell, card: card, a: fa, b: fb,
              ca: (pr[0] || '').trim(), cb: (pr[1] || '').trim(),
              blank: !(pr[0] || '').trim() && !(pr[1] || '').trim(),
            });
          });
          host.appendChild(word);
          if (gi < groups.length - 1) {
            var sp = FX.el('span', null, { display: 'inline-block', width: SPW + 'px' });
            host.appendChild(sp);
            stage._gaps.push({
              el: sp, w: SPW,
              a: hasSide(g, 0) && hasSide(groups[gi + 1], 0),
              b: hasSide(g, 1) && hasSide(groups[gi + 1], 1),
            });
          }
        });
      }
      buildRow(rows[0], ctx.from.light, ctx.to.light);
      buildRow(rows[1], ctx.from.bold, ctx.to.bold);

      /* natural width of every face, measured once — the cells are driven from these */
      stage._cards.forEach(function (C) {
        C.wa = C.ca ? C.a.el.getBoundingClientRect().width : 0;
        C.wb = C.cb ? C.b.el.getBoundingClientRect().width : 0;
        C.a.el.style.transform = 'translateX(-50%)';
        C.b.el.style.transform = 'translateX(-50%) rotateY(180deg)';
      });
    },
    frame: function (stage, t) {
      var cards = stage._cards, n = cards.length;
      /* the whole board leans back a touch while the wave crosses it */
      var tilt = Math.sin(Math.PI * E.inOutQuad(t)) * 3.2;
      stage._board.style.transform = 'rotateX(' + tilt + 'deg)';
      /* the tiles themselves fade up out of plain text and back into it at the endpoints */
      var chrome = chromeAt(t, 0.08, 0.09);
      var faceBg = 'linear-gradient(rgba(255,255,255,' + chrome.toFixed(3) +
        '),rgba(241,238,231,' + chrome.toFixed(3) + '))';
      var gp = E.inOutQuart(span(t, 0.06, 0.90));
      for (var gi = 0; gi < stage._gaps.length; gi++) {
        var G = stage._gaps[gi];
        G.el.style.width = (G.w * lerp(G.a ? 1 : 0, G.b ? 1 : 0, gp)).toFixed(2) + 'px';
      }
      for (var k = 0; k < n; k++) {
        var lead = 0.06 + (k / Math.max(1, n)) * 0.40;
        var u = span(t, lead, lead + 0.44);
        var e = E.inOutQuart(u);
        var s = Math.sin(Math.PI * u);            /* 0 at the ends, 1 mid-flip */
        var ang = -180 * e;
        var z = -34 * s;
        var sc = 1 + 0.07 * s;
        var C = cards[k];
        C.cell.style.width = lerp(C.wa, C.wb, e).toFixed(2) + 'px';
        C.card.style.left = (lerp(C.wa, C.wb, e) / 2).toFixed(2) + 'px';
        C.card.style.transform =
          'translateZ(' + z.toFixed(2) + 'px) rotateY(' + ang.toFixed(2) + 'deg) scale(' +
          sc.toFixed(3) + ')';
        if (C.blank) continue;
        var sh = '0 ' + (1.5 + 7 * s).toFixed(1) + 'px ' + (3 + 10 * s).toFixed(1) +
          'px rgba(0,0,0,' + (chrome * (0.10 + 0.20 * s)).toFixed(3) +
          '), inset 0 0 0 1px rgba(0,0,0,' + (chrome * 0.07).toFixed(3) + ')';
        C.a.el.style.boxShadow = C.b.el.style.boxShadow = sh;
        C.a.el.style.background = C.b.el.style.background = faceBg;
        /* light rakes across the face as it turns: the leading edge stays bright, the far side
           of the swing sinks into the board */
        C.a.shade.style.opacity = (0.34 * s).toFixed(3);
        C.b.shade.style.opacity = (0.34 * s).toFixed(3);
      }
    },
    rest: function (stage) {
      stage._board.style.transform = 'rotateX(0deg)';
      stage._gaps.forEach(function (G) { G.el.style.width = (G.b ? G.w : 0) + 'px'; });
      stage._cards.forEach(function (C) {
        C.cell.style.width = C.wb.toFixed(2) + 'px';
        C.card.style.left = (C.wb / 2).toFixed(2) + 'px';
        C.card.style.transform = 'translateZ(0px) rotateY(-180deg) scale(1)';
        C.a.el.style.boxShadow = C.b.el.style.boxShadow = 'none';
        C.a.el.style.background = C.b.el.style.background = 'none';
        C.a.shade.style.opacity = '0';
        C.b.shade.style.opacity = '0';
      });
    },
  });

  /* ================================================================ 3. WORD ODOMETER ======== */
  /* A counter that rolls WORDS, not letters. Each word sits on its own drum; only the words that
     actually change roll over — the ones that survive simply glide to their new slot, exactly
     like an odometer where the unchanged digits stay put. Both layouts are measured at setup so
     t=0 is the FROM line's true typography and t=1 is the TO line's. */

  /* The roll has to LEAVE at zero speed and ARRIVE at zero speed: the frame just after t=0 sits
     next to the engine's canonical text, so a drum that has already snapped a tenth of a turn (and
     smeared itself with the motion blur that comes off this curve) reads as a jump cut. Body of
     the move is a smooth in-out; the counter ring only starts once the drum is under way. */
  function detent(u) {
    if (u <= 0) return 0;
    if (u >= 1) return 1;
    var v = span(u, 0.45, 1);
    var ring = v > 0 ? Math.sin(v * Math.PI * 3) * Math.pow(1 - v, 2) * 0.07 : 0;
    return E.inOutCubic(span(u, 0, 0.72)) + ring;
  }

  FX.register({
    id: 'mechanical-word-odometer',
    name: 'Word Odometer',
    family: 'Mechanical',
    blurb: 'Whole words roll over on counter drums, least-significant first; unchanged words glide.',
    duration: 1500,
    dwell: 4400,
    theme: { bg: '#FAF8F3', fg: '#191A1C' },
    setup: function (stage, ctx) {
      var w = box(stage);
      stage._drums = [];

      /* Measured against the CANONICAL shape — two rows, page line-height — so a drum's slot is
         the very pixel the engine parks that word on at rest. */
      function measureLine(s) {
        var b = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0', top: '50%',
          transform: 'translateY(-50%)', textAlign: 'center',
        });
        var parts = [];
        function add(row, text, cls) {
          var first = true;
          text.split(/\s+/).filter(Boolean).forEach(function (t) {
            if (!first) {
              var g = FX.el('span', null, { whiteSpace: 'pre' });
              g.textContent = ' ';
              row.appendChild(g);
            }
            first = false;
            var sp = FX.el('span', cls, { display: 'inline-block', whiteSpace: 'nowrap' });
            sp.textContent = t;
            row.appendChild(sp);
            parts.push({ el: sp, text: t, cls: cls });
          });
        }
        var rows = twoRows(b);
        add(rows[0], s.light, 'fx-light');
        add(rows[1], s.bold, 'fx-bold');
        return { box: b, parts: parts };
      }

      var A = measureLine(ctx.from), B = measureLine(ctx.to);
      w.appendChild(A.box); w.appendChild(B.box);
      var wr = w.getBoundingClientRect();
      function meas(L) {
        return L.parts.map(function (p) {
          var r = p.el.getBoundingClientRect();
          return {
            cx: r.left - wr.left + r.width / 2, cy: r.top - wr.top + r.height / 2,
            h: r.height, text: p.text, cls: p.cls,
          };
        });
      }
      var ma = meas(A), mb = meas(B);
      w.removeChild(A.box); w.removeChild(B.box);

      var n = Math.max(ma.length, mb.length);
      for (var i = 0; i < n; i++) {
        var a = ma[i], b = mb[i];
        var ref = a || b;
        var x0 = (a || b).cx, y0 = (a || b).cy;
        var x1 = (b || a).cx, y1 = (b || a).cy;
        var same = !!(a && b && a.text === b.text && a.cls === b.cls);
        var h = ref.h || 20;

        var holder = FX.el('div', null, {
          position: 'absolute', left: '0', top: '0', width: '0', height: '0',
          perspective: (h * 9) + 'px', willChange: 'transform,filter',
        });
        var drum = FX.el('div', null, {
          position: 'absolute', left: '0', top: '0', width: '0', height: '0',
          transformStyle: 'preserve-3d',
        });
        var R = h * 0.54;
        /* A face parked at radius R sits R nearer the eye than the drum's axis, so the perspective
           on the holder magnifies it by P/(P-R). Left uncorrected the word renders ~6% large and
           slightly overlapping its neighbours — visible as jammed-together text the instant the
           transition starts. Scale it back down by exactly that factor. */
        var SC = (h * 9 - R) / (h * 9);
        function mkFace(m, back) {
          if (!m) return null;
          var f = FX.el('span', m.cls, {
            position: 'absolute', left: '0', top: '0', whiteSpace: 'nowrap',
            backfaceVisibility: 'hidden',
            /* Origin at the drum's axis, not the glyph's own middle. With the default 50% 50%
               origin the face's own centring translation is re-rotated by the drum, which parks
               the landing face half a line height nearer the eye: it renders 6% oversized and
               floats above its slot, so the words end the roll jammed together and too high. */
            transformOrigin: '0 0',
          });
          f.textContent = m.text;
          f.style.transform = (back ? 'rotateX(-90deg) ' : '') +
            'translateZ(' + R + 'px) scale(' + SC.toFixed(4) + ') translate(-50%,-50%)';
          drum.appendChild(f);
          return f;
        }
        var fa = mkFace(a, false);
        var fb = same ? null : mkFace(b, true);
        holder.appendChild(drum);
        w.appendChild(holder);
        stage._drums.push({
          holder: holder, drum: drum, fa: fa, fb: fb, same: same,
          x0: x0, y0: y0, x1: x1, y1: y1, i: i, n: n,
        });
      }
    },
    frame: function (stage, t) {
      var D = stage._drums;
      for (var k = 0; k < D.length; k++) {
        var d = D[k];
        /* rightmost first — the least significant word rolls before the rest */
        var lead = ((d.n - 1 - d.i) / Math.max(1, d.n)) * 0.32;
        var u = span(t, lead, lead + 0.60);
        var pos = E.inOutCubic(u);
        var x = lerp(d.x0, d.x1, pos), y = lerp(d.y0, d.y1, pos);

        if (d.same) {
          d.holder.style.transform = 'translate(' + x + 'px,' + y + 'px)';
          d.holder.style.filter = 'none';
          d.drum.style.transform = 'rotateX(0deg)';
          continue;
        }
        var p = detent(u);
        var ang = 90 * p;
        d.drum.style.transform = 'rotateX(' + ang + 'deg)';
        /* motion blur straight off the derivative of the roll — analytic, so still pure in t */
        var dp = Math.abs(detent(clamp(u + 0.02, 0, 1)) - detent(clamp(u - 0.02, 0, 1)));
        var bl = Math.min(2.4, dp * 26);
        d.holder.style.transform = 'translate(' + x + 'px,' + y + 'px)';
        d.holder.style.filter = bl > 0.05 ? 'blur(' + bl.toFixed(2) + 'px)' : 'none';
        var ra = clamp(Math.cos(ang * Math.PI / 180), 0, 1);
        var rb = clamp(Math.cos((ang - 90) * Math.PI / 180), 0, 1);
        if (d.fa) d.fa.style.opacity = String(Math.pow(ra, 0.65));
        if (d.fb) d.fb.style.opacity = String(Math.pow(rb, 0.65));
      }
    },
    rest: function (stage) {
      stage._drums.forEach(function (d) {
        d.holder.style.transform = 'translate(' + d.x1 + 'px,' + d.y1 + 'px)';
        d.holder.style.filter = 'none';
        d.drum.style.transform = d.same ? 'rotateX(0deg)' : 'rotateX(90deg)';
        if (d.fa) d.fa.style.opacity = d.same ? '1' : '0';
        if (d.fb) d.fb.style.opacity = '1';
      });
    },
  });

  /* ================================================================= 4. DOT-MATRIX SIGN ===== */
  /* An industrial LED panel. Unlit dots fill the whole board; a refresh bar sweeps across,
     extinguishing the old message behind it and igniting the new one, with phosphor persistence
     and a settling flash when the sweep clears the edge. */

  var PITCH = '0.155em';

  function dotBg(node, color, r) {
    node.style.backgroundImage =
      'radial-gradient(circle at 50% 50%, ' + color + ' 0 ' + r + '%, transparent ' + (r + 10) + '%)';
    node.style.backgroundSize = PITCH + ' ' + PITCH;
    node.style.backgroundRepeat = 'repeat';
  }
  function dotMask(node, r) {
    var img = 'radial-gradient(circle at 50% 50%, #000 0 ' + r + '%, transparent ' + (r + 10) + '%)';
    node.style.webkitMaskImage = node.style.maskImage = img;
    node.style.webkitMaskSize = node.style.maskSize = PITCH + ' ' + PITCH;
    node.style.webkitMaskRepeat = node.style.maskRepeat = 'repeat';
  }
  function sweepMask(node, css) {
    node.style.webkitMaskImage = node.style.maskImage = css;
    node.style.webkitMaskSize = node.style.maskSize = '100% 100%';
    node.style.webkitMaskRepeat = node.style.maskRepeat = 'no-repeat';
  }

  FX.register({
    id: 'mechanical-dot-matrix',
    name: 'Dot-Matrix Sign',
    family: 'Mechanical',
    blurb: 'An LED board repaints itself: a refresh bar sweeps across, old dots die, new dots fire.',
    duration: 1500,
    dwell: 4200,
    theme: {
      bg: '#0A0D0E', fg: '#FFB324',
      font: '"Helvetica Neue", Inter, Arial, sans-serif',
    },
    setup: function (stage, ctx) {
      var w = box(stage);

      var panel = FX.el('div', null, { position: 'absolute', left: 0, top: 0, right: 0, bottom: 0 });
      dotBg(panel, '#1D2428', 38);
      w.appendChild(panel);

      function sign(s, cls) {
        var glow = FX.el('div', null, {
          position: 'absolute', left: 0, top: 0, right: 0, bottom: 0,
          willChange: 'filter,opacity',
        });
        var outer = FX.el('div', null, {
          position: 'absolute', left: 0, top: 0, right: 0, bottom: 0, willChange: 'mask-image',
        });
        var inner = FX.el('div', null, {
          position: 'absolute', left: 0, top: 0, right: 0, bottom: 0,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
        });
        dotMask(inner, 42);
        var txt = FX.el('div', null, {
          textAlign: 'center', maxWidth: '100%', lineHeight: '1.26em',
          WebkitTextStroke: '0.014em currentColor',
        });
        twoWeights(txt, s);
        inner.appendChild(txt); outer.appendChild(inner); glow.appendChild(outer);
        w.appendChild(glow);
        return { glow: glow, outer: outer, inner: inner };
      }

      stage._ghost = sign(ctx.from);
      stage._out = sign(ctx.from);
      stage._in = sign(ctx.to);

      var scan = FX.el('div', null, { position: 'absolute', left: 0, top: 0, right: 0, bottom: 0 });
      dotBg(scan, '#FFF2D2', 44);
      w.appendChild(scan);
      stage._scan = scan;
    },
    frame: function (stage, t) {
      var e = E.inOutCubic(t);
      var P = -12 + e * 124;                       /* refresh-bar position, % of board */
      var soft = 5;

      sweepMask(stage._out.outer, 'linear-gradient(90deg, transparent ' + (P - 1).toFixed(2) +
        '%, #000 ' + (P + soft).toFixed(2) + '%)');
      sweepMask(stage._in.outer, 'linear-gradient(90deg, #000 ' + (P - soft).toFixed(2) +
        '%, transparent ' + (P + 1).toFixed(2) + '%)');

      /* phosphor persistence: the whole old message hangs on faintly, then decays */
      stage._ghost.glow.style.opacity = String(0.16 * (1 - E.outQuad(span(t, 0.10, 0.92))));
      sweepMask(stage._ghost.outer, 'linear-gradient(90deg, #000 0%, #000 100%)');

      /* the bar itself: a band of every LED on the board driven hard */
      var band = 'linear-gradient(90deg, transparent ' + (P - 7).toFixed(2) +
        '%, rgba(0,0,0,.30) ' + (P - 3.2).toFixed(2) +
        '%, #000 ' + P.toFixed(2) + '%, rgba(0,0,0,.35) ' + (P + 2.4).toFixed(2) +
        '%, transparent ' + (P + 5.5).toFixed(2) + '%)';
      sweepMask(stage._scan, band);
      stage._scan.style.opacity = String(0.85 * Math.min(1, span(t, 0, 0.06) * 1) *
        (1 - E.inQuad(span(t, 0.88, 1))));

      /* glow, and a two-beat settle flash once the bar clears the right edge */
      var flash = 1 + 0.34 * Math.sin(span(t, 0.86, 1) * Math.PI * 2) * (1 - span(t, 0.86, 1));
      stage._out.glow.style.filter = 'drop-shadow(0 0 .07em rgba(255,179,36,.55))';
      stage._in.glow.style.filter =
        'drop-shadow(0 0 .07em rgba(255,179,36,.6)) brightness(' + flash.toFixed(3) + ')';
      stage._ghost.glow.style.filter = 'blur(.4px)';
    },
    rest: function (stage) {
      sweepMask(stage._out.outer, 'linear-gradient(90deg, transparent 0%, transparent 100%)');
      sweepMask(stage._in.outer, 'linear-gradient(90deg, #000 0%, #000 100%)');
      stage._ghost.glow.style.opacity = '0';
      stage._scan.style.opacity = '0';
      stage._in.glow.style.filter = 'drop-shadow(0 0 .07em rgba(255,179,36,.6))';
    },
  });

  /* ================================================================== 5. TYPEWRITER ========= */
  /* Platen roll, carriage shunt, then the new line is struck one glyph at a time. Each strike
     kicks the paper (a damped impulse summed over the last few keys) and every letter keeps a
     permanent hair of misalignment, the way a real typebar never quite lands square. */

  FX.register({
    id: 'mechanical-typewriter',
    name: 'Carriage Return',
    family: 'Mechanical',
    blurb: 'The platen rolls the old line away, the carriage slams left, the new line is struck out.',
    duration: 2100,
    dwell: 4200,
    theme: {
      bg: '#F6F3EA', fg: '#241F1A',
      font: '"American Typewriter", "Courier New", ui-monospace, monospace',
    },
    setup: function (stage, ctx) {
      var w = box(stage, { perspective: '900px' });
      stage._wrap = w;

      function line(s, sz) {
        var b = FX.el('div', null, {
          position: 'absolute', left: 0, top: 0, right: 0, bottom: 0,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          willChange: 'transform,opacity',
        });
        var inner = FX.el('div', null, {
          textAlign: 'center', maxWidth: '100%', fontSize: sz, lineHeight: '1.32em',
        });
        var nodes = [];
        function add(text, cls) {
          var cw = FX.charWords(text, cls);
          inner.appendChild(cw.frag);
          cw.nodes.forEach(function (nd) { nd.classList.add(cls); nodes.push(nd); });
        }
        add(s.light, 'fx-light');
        var gap = FX.el('span', null, { whiteSpace: 'pre' }); gap.textContent = ' ';
        inner.appendChild(gap);
        add(s.bold, 'fx-bold');
        b.appendChild(inner);
        w.appendChild(b);
        return { box: b, inner: inner, nodes: nodes };
      }

      stage._old = line(ctx.from, '.9em');
      stage._new = line(ctx.to, '.9em');

      var caret = FX.el('div', null, {
        position: 'absolute', left: '0', top: '0', width: '.09em', height: '1.1em',
        background: 'currentColor', opacity: '.5', willChange: 'transform',
      });
      w.appendChild(caret);
      stage._caret = caret;

      /* cache geometry once — reading layout every frame would be both slow and, once the paper
         has a transform on it, wrong */
      var wr = w.getBoundingClientRect();
      function pt(node) {
        var r = node.getBoundingClientRect();
        return { l: r.left - wr.left, r: r.right - wr.left, t: r.top - wr.top, h: r.height };
      }
      stage._endOld = stage._old.nodes.length
        ? pt(stage._old.nodes[stage._old.nodes.length - 1]) : { r: 0, t: 0, h: 20 };
      stage._pos = stage._new.nodes.map(pt);
      stage._startNew = stage._pos.length ? stage._pos[0] : { l: 0, r: 0, t: 0, h: 20 };

      /* permanent per-glyph misalignment, fixed for the life of this transition */
      stage._new.nodes.forEach(function (nd, k) {
        nd._rot = (h1(k, 4) - 0.5) * 1.7;
        nd._dy = (h1(k, 5) - 0.5) * 0.055;
      });
    },
    frame: function (stage, t) {
      var O = stage._old, N = stage._new, pos = stage._pos, n = N.nodes.length;

      /* ---- platen: the old line rolls up and over the roller */
      var roll = E.inOutCubic(span(t, 0.03, 0.30));
      O.box.style.transformOrigin = '50% 0%';
      O.box.style.transform = 'translateY(' + (-2.1 * roll) + 'em) rotateX(' + (-46 * roll) + 'deg)';
      O.box.style.opacity = String(1 - E.inQuad(span(t, 0.10, 0.30)));

      /* ---- carriage shunt: hard travel left, hits the stop, rebounds once */
      var sh = span(t, 0.08, 0.30);
      var shunt = sh <= 0 ? 0 : (sh >= 1 ? 1 : E.outBack(sh));

      /* ---- strike schedule */
      var T0 = 0.26, T1 = 0.985;
      function strikeAt(k) { return n <= 1 ? T0 : T0 + (k / (n - 1)) * (T1 - T0); }
      var last = -1;
      for (var k = 0; k < n; k++) {
        var s0 = strikeAt(k);
        var u = span(t, s0, s0 + Math.min(0.075, (T1 - T0) / Math.max(1, n) * 3 + 0.02));
        var nd = N.nodes[k];
        if (u <= 0) {
          nd.style.opacity = '0';
          nd.style.transform = 'scale(1.55)';
        } else {
          last = k;
          var e = E.outBack(u);
          nd.style.opacity = String(Math.min(1, u * 3));
          nd.style.transform =
            'translateY(' + (nd._dy + (1 - e) * -0.10) + 'em) ' +
            'scale(' + lerp(1.55, 1, e) + ') ' +
            'rotate(' + (nd._rot + (h1(k, 6) - 0.5) * 7 * (1 - e)) + 'deg)';
        }
        nd.style.display = 'inline-block';
      }

      /* ---- paper jolt: damped impulse from the last few strikes, plus the carriage slam */
      var jy = 0, jx = 0;
      for (var m = Math.max(0, last - 3); m <= last; m++) {
        var dt = t - strikeAt(m);
        if (dt < 0) continue;
        var d = Math.exp(-42 * dt);
        jy += 1.5 * d * Math.sin(dt * 150);
        jx += 0.5 * d * Math.sin(dt * 110);
      }
      var slam = Math.exp(-16 * Math.max(0, t - 0.255)) * Math.sin(Math.max(0, t - 0.255) * 130);
      if (t > 0.24) jx += 3.4 * slam;
      N.box.style.transform = 'translate(' + jx.toFixed(2) + 'px,' + jy.toFixed(2) + 'px)';
      N.box.style.opacity = '1';

      /* ---- carriage caret */
      var cx, cy, ch;
      if (last < 0) {
        cx = lerp(stage._endOld.r, stage._startNew.l, shunt);
        cy = lerp(stage._endOld.t, stage._startNew.t, E.inOutCubic(sh));
        ch = stage._startNew.h;
      } else {
        cx = pos[last].r + jx; cy = pos[last].t + jy; ch = pos[last].h;
      }
      stage._caret.style.height = ch + 'px';
      stage._caret.style.transform = 'translate(' + cx.toFixed(2) + 'px,' + cy.toFixed(2) + 'px)';
      stage._caret.style.opacity = String(last < 0 ? 0.58 : 0.5);
    },
    rest: function (stage) {
      var N = stage._new, pos = stage._pos;
      stage._old.box.style.opacity = '0';
      N.box.style.transform = 'translate(0px,0px)';
      N.nodes.forEach(function (nd) {
        nd.style.opacity = '1';
        nd.style.transform = 'translateY(' + nd._dy + 'em) scale(1) rotate(' + nd._rot + 'deg)';
      });
      var l = pos.length - 1;
      if (l >= 0) {
        stage._caret.style.height = pos[l].h + 'px';
        stage._caret.style.transform = 'translate(' + pos[l].r + 'px,' + pos[l].t + 'px)';
      }
      stage._caret.style.opacity = '.38';
    },
  });

  /* ================================================================== 6. SLIDE RULE ========= */
  /* The line rides a slide-rule cursor. The whole rail — text and its engraved scale — travels
     under a fixed hairline, and instead of easing politely to a stop it overshoots and is walked
     back in three shrinking vernier corrections until the mark sits dead on the line. */

  function slideBase(u) { return E.outQuint(span(u, 0, 0.66)); }
  function slideWobble(u) {
    var v = span(u, 0.46, 1);
    if (v <= 0 || v >= 1) return 0;
    return 26 * Math.pow(1 - v, 1.7) * Math.sin(v * Math.PI * 3);
  }
  function slideX(u, D) { return -D * slideBase(u) + slideWobble(u); }

  FX.register({
    id: 'mechanical-slide-rule',
    name: 'Vernier Lock',
    family: 'Mechanical',
    blurb: 'The line rides an engraved rail past a hairline, overshoots, and is walked into lock.',
    duration: 1700,
    dwell: 4400,
    theme: { bg: '#EBE7DC', fg: '#1F1F1E' },
    setup: function (stage, ctx) {
      var w = box(stage);
      var W = Math.max(240, ctx.W || stage.clientWidth || 400);
      var D = W + Math.round(W * 0.16);
      stage._D = D;

      function scale(host, cssTop, flip, label) {
        var s = FX.el('div', null, {
          position: 'absolute', left: '0', width: '340%', height: '1.05em',
          opacity: '.55',
        });
        s.style[flip ? 'bottom' : 'top'] = cssTop;
        var minor = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0', height: '.34em',
          backgroundImage: 'repeating-linear-gradient(90deg, currentColor 0 1px, transparent 1px .42em)',
        });
        var major = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0', height: '.62em',
          backgroundImage: 'repeating-linear-gradient(90deg, currentColor 0 1px, transparent 1px 2.1em)',
        });
        minor.style[flip ? 'bottom' : 'top'] = '0';
        major.style[flip ? 'bottom' : 'top'] = '0';
        s.appendChild(minor); s.appendChild(major);
        var tag = FX.el('div', null, {
          position: 'absolute', left: '.25em', fontSize: '.34em', letterSpacing: '.18em',
          fontWeight: '700', opacity: '.8',
        });
        tag.style[flip ? 'top' : 'bottom'] = '0';
        tag.textContent = label;
        s.appendChild(tag);
        host.appendChild(s);
        return s;
      }

      /* fixed stator scale */
      scale(w, '2px', false, 'D');

      /* the rail: masked at both ends so text slides in and out through a soft edge */
      var railBox = FX.el('div', null, {
        position: 'absolute', left: 0, top: 0, right: 0, bottom: 0,
      });
      railBox.style.webkitMaskImage = railBox.style.maskImage =
        'linear-gradient(90deg, transparent 0%, #000 7%, #000 93%, transparent 100%)';
      railBox.style.webkitMaskRepeat = railBox.style.maskRepeat = 'no-repeat';
      railBox.style.webkitMaskSize = railBox.style.maskSize = '100% 100%';
      w.appendChild(railBox);

      var rail = FX.el('div', null, {
        position: 'absolute', left: 0, top: 0, right: 0, bottom: 0,
        willChange: 'transform,filter',
      });
      railBox.appendChild(rail);

      /* moving slide scale, offset so its engraving reads as travel */
      var sl = scale(rail, '2px', true, 'C');
      sl.style.left = (-W * 0.6) + 'px';

      function line(s, dx) {
        var b = FX.el('div', null, {
          position: 'absolute', left: 0, top: 0, right: 0, bottom: 0,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
        });
        var inner = FX.el('div', null, {
          textAlign: 'center', maxWidth: '100%', lineHeight: '1.26em', padding: '0 .4em',
        });
        twoWeights(inner, s);
        b.appendChild(inner);
        b.style.transform = 'translateX(' + dx + 'px)';
        rail.appendChild(b);
        return b;
      }
      line(ctx.from, 0);
      line(ctx.to, D);
      stage._rail = rail;

      /* the glass cursor: fixed hairline the whole rail is measured against */
      var glass = FX.el('div', null, {
        position: 'absolute', left: '50%', top: '6%', bottom: '6%', width: '2.6em',
        transform: 'translateX(-50%)', border: '1px solid rgba(0,0,0,.18)',
        borderRadius: '2px', background: 'linear-gradient(90deg,' +
          'rgba(255,255,255,0) 0%, rgba(255,255,255,.35) 45%, rgba(255,255,255,.35) 55%,' +
          'rgba(255,255,255,0) 100%)', pointerEvents: 'none',
      });
      var hair = FX.el('div', null, {
        position: 'absolute', left: 'calc(50% - .5px)', top: '0', bottom: '0', width: '1px',
        background: '#C0392B',
      });
      glass.appendChild(hair);
      w.appendChild(glass);
      stage._glass = glass; stage._hair = hair;
    },
    frame: function (stage, t) {
      var D = stage._D;
      var x = slideX(t, D);
      var v = Math.abs(slideX(clamp(t + 0.012, 0, 1), D) - slideX(clamp(t - 0.012, 0, 1), D));
      var bl = Math.min(3.4, v * 0.09);
      var tilt = 0.34 * (1 - slideBase(t));
      stage._rail.style.transform =
        'translateX(' + x.toFixed(2) + 'px) rotate(' + tilt.toFixed(3) + 'deg)';
      stage._rail.style.filter = bl > 0.06 ? 'blur(' + bl.toFixed(2) + 'px)' : 'none';

      /* the hairline brightens as the mark walks into lock */
      var lock = span(t, 0.5, 0.98);
      stage._hair.style.background = lock > 0 ? '#D0402C' : '#C0392B';
      stage._hair.style.boxShadow = '0 0 ' + (7 * E.outQuad(lock)).toFixed(2) +
        'px rgba(208,64,44,' + (0.55 * E.outQuad(lock)).toFixed(2) + ')';
      stage._glass.style.transform = 'translateX(-50%) scaleX(' +
        (1 + 0.05 * Math.sin(Math.PI * span(t, 0.42, 1))).toFixed(3) + ')';
    },
    rest: function (stage) {
      stage._rail.style.transform = 'translateX(' + (-stage._D) + 'px) rotate(0deg)';
      stage._rail.style.filter = 'none';
      stage._hair.style.boxShadow = '0 0 6px rgba(208,64,44,.5)';
      stage._glass.style.transform = 'translateX(-50%) scaleX(1)';
    },
  });
})();
