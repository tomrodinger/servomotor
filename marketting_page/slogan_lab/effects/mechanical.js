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

  /* 'rgb(r,g,b)' -> 'rgba(r,g,b,a)'. The sheet colours ctx.sc hands out are opaque by design, but
     every piece of machinery in this file has to be able to fade to literally nothing at the
     endpoints, so it needs them with an alpha channel. */
  function fade(rgb, a) {
    var m = /rgba?\(([^)]+)\)/.exec(rgb || '');
    if (!m) return rgb;
    var p = m[1].split(',');
    return 'rgba(' + p[0].trim() + ',' + p[1].trim() + ',' + p[2].trim() + ',' + a.toFixed(3) + ')';
  }

  /* ================================================================= 1. SPLIT-FLAP BOARD ==== */
  /* Solari departure board. Every cell is a real two-flap card: the top flap falls away showing
     the next glyph's top half behind it, then the bottom flap swings up. Each cell riffles
     through its own deterministic number of intermediate glyphs before landing on the target. */

  /* Row pitch has to be the canon's line box exactly. The cells are inline-blocks aligned to the
     top of their line, so their MARGIN box is what sets the pitch: CH + 2*CM must equal the
     stage's 1.08 line-height. At the old 1.16 + 2*0.01 every board row stood 0.10em taller than
     the canonical one, which walked the second line down and stretched the ink box at both ends. */
  var CH = 1.06;                 /* card height, em */
  var CM = 0.01;                 /* card margin, em — CH + 2*CM == the canon's 1.08 line box */
  var CGX = 0.09;                /* how far a flap's clipper reaches past its cell, em */

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

  /* One half of a card. Two jobs, and they want different widths, so they are two elements:
       - the CLIPPER shows only the top or the bottom of a full-height glyph, and it reaches CGX
         past the cell on both sides;
       - the FACE, the painted card itself, stays exactly on the cell.
     A cell is one glyph advance wide and the stage runs -0.03em letter-spacing, so a clipper on
     the cell bounds shaved the right-hand edge off every glyph. Invisible while the cards are lit,
     obvious at t=0.008 and t=0.992 where the cards have faded out and the glyphs are all that is
     left to compare against the canonical line. The card must line up with the grid; the letter
     must not be touched. */
  function flapHalf(isTop) {
    var d = FX.el('div', null, {
      position: 'absolute', left: (-CGX) + 'em', right: (-CGX) + 'em', height: (CH / 2) + 'em',
      overflow: 'hidden', background: 'transparent',
      backfaceVisibility: 'hidden',
    });
    d.style[isTop ? 'top' : 'bottom'] = '0';
    var face = FX.el('div', null, {
      position: 'absolute', left: CGX + 'em', right: CGX + 'em', top: '0', bottom: '0',
      background: 'transparent',
      borderRadius: isTop ? '.07em .07em 0 0' : '0 0 .07em .07em',
    });
    d.appendChild(face);
    var txt = FX.el('div', null, {
      position: 'absolute', left: CGX + 'em', right: CGX + 'em', height: CH + 'em',
      lineHeight: CH + 'em', textAlign: 'center', whiteSpace: 'pre',
    });
    txt.style.top = isTop ? '0' : (-CH / 2) + 'em';
    d.appendChild(txt);
    return { box: d, face: face, txt: txt };
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

      function hasSide(g, side) {
        return g.pairs.some(function (p) { return !!(p[side] || '').trim(); });
      }

      /* One board row per half of the headline, so the two lines of the canonical rest state
         survive the whole transition. */
      function buildRow(host, a, b) {
        /* Per-glyph natural advances, measured in THIS row's weight.
           The board used to be one monospace pitch, measured off an 'M', on the assumption that
           the stage was running the effect's own monospace theme. The rater deliberately never
           applies an effect's theme — the page owns the font — so every cell came out an 'M' wide
           in a proportional face and the line rendered hundreds of px wider than the canonical
           text it has to start and end on (425px out at t=0.008, 373px at t=0.992).
           Cells are now one glyph wide and animate from the FROM glyph's advance to the TO
           glyph's, the way the flip-card board further down does it. The cards, the riffle and
           the two-flap fall are unchanged; only the grid stops pretending to be monospace. */
        var probe = FX.el('span', null, {
          position: 'absolute', visibility: 'hidden', whiteSpace: 'pre', left: '-9999px', top: '0',
        });
        host.appendChild(probe);
        var cache = {};
        function adv(ch) {
          if (!ch) return 0;
          if (cache[ch] == null) {
            probe.textContent = ch;
            cache[ch] = probe.getBoundingClientRect().width;
          }
          return cache[ch];
        }
        var SPW = adv(' ');
        var groups = FX.alignWords(a, b);
        groups.forEach(function (g, gi) {
          var word = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
          g.pairs.forEach(function (pr) {
            var k = stage._cells.length;
            var cell = FX.el('span', null, {
              position: 'relative', display: 'inline-block', verticalAlign: 'top',
              width: adv(pr[0]).toFixed(3) + 'px', height: CH + 'em', margin: CM + 'em 0',
              perspective: '7em',
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
              faces: [st.face, sb.face, ft.face, fb.face],
              /* a cell that only exists on one side of the pair is a card the board is adding or
                 pulling out: its slot closes to zero as it flips, so each endpoint measures
                 exactly the text that is there — no ghost width left over from the other slogan */
              wa: adv(pr[0]), wb: adv(pr[1]),
              seq: blank ? [' ', ' '] : flapSeq(pr[0], pr[1], k),
            });
          });
          host.appendChild(word);
          if (gi < groups.length - 1) {
            var sp = FX.el('span', null, { display: 'inline-block', width: SPW.toFixed(3) + 'px' });
            host.appendChild(sp);
            stage._gaps.push({
              el: sp, w: SPW,
              a: hasSide(g, 0) && hasSide(groups[gi + 1], 0),
              b: hasSide(g, 1) && hasSide(groups[gi + 1], 1),
            });
          }
        });
        host.removeChild(probe);
      }
      buildRow(rows[0], ctx.from.light, ctx.to.light);
      buildRow(rows[1], ctx.from.bold, ctx.to.bold);
    },
    frame: function (stage, t, ctx) {
      var cells = stage._cells, n = cells.length, sc = ctx.sc;
      /* the cards themselves fade up out of plain text and back into it, so the frames either
         side of the engine's canonical endpoints are plain text on the same grid.
         The card was a hard-coded near-black rgba(28,29,32) — a black card carrying dark ink the
         moment the stage is light. It is now the stage's own sheet, lifted. */
      var chrome = chromeAt(t, 0.08, 0.10);
      var cardBg = fade(sc.raised, chrome);
      var seamBg = fade(sc.sunken, chrome);
      var gp = E.inOutQuad(span(t, 0.08, 0.84));
      for (var gi = 0; gi < stage._gaps.length; gi++) {
        var G = stage._gaps[gi];
        G.el.style.width = lerp(G.a ? G.w : 0, G.b ? G.w : 0, gp).toFixed(3) + 'px';
      }
      for (var k = 0; k < n; k++) {
        var c = cells[k];
        var lead = 0.08 + (k / Math.max(1, n)) * 0.30 + h1(k, 9) * 0.04;
        var p = E.inOutQuad(span(t, lead, lead + 0.46));
        c.cell.style.width = lerp(c.wa, c.wb, p).toFixed(3) + 'px';
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
        for (var q = 0; q < 4; q++) c.faces[q].style.background = cardBg;
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
        c.cell.style.width = c.wb.toFixed(3) + 'px';
        for (var q = 0; q < 4; q++) c.faces[q].style.background = 'transparent';
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
            /* Height is the canon's line box, not a roomier 1.18em. The cells are top-aligned
               inline-blocks, so their height IS the row pitch: at 1.18em every row stood 0.10em
               taller than the canonical one, which pushed the bold line down and stretched the
               ink box ~13px at the start and ~20px at the end. */
            var cell = FX.el('span', null, {
              position: 'relative', display: 'inline-block', verticalAlign: 'top',
              perspective: '520px', height: '1.08em', width: '0px',
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
                textAlign: 'center', lineHeight: '1.08em', padding: '0',
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
              /* the raking shadow is plain black at low alpha, which darkens correctly whatever
                 the sheet under it happens to be — it is a shadow, not a sheet */
              var shade = FX.el('span', null, {
                position: 'absolute', left: '0', top: '0', right: '0', bottom: '0',
                borderRadius: '2px', background: '#000', opacity: '0', zIndex: '2',
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
    frame: function (stage, t, ctx) {
      var cards = stage._cards, n = cards.length, sc = ctx.sc;
      /* the whole board leans back a touch while the wave crosses it */
      var tilt = Math.sin(Math.PI * E.inOutQuad(t)) * 3.2;
      stage._board.style.transform = 'rotateX(' + tilt + 'deg)';
      /* The tiles fade up out of plain text and back into it at the endpoints — but a linear ramp
         still left 9-10% of a tile painted over every glyph at t=0.008 and t=0.992, which is
         ~9% of the stage reading differently from the canonical frame. Squaring the ramp puts the
         endpoints at half a percent while the tiles are still fully solid across the middle. */
      var chrome = Math.pow(chromeAt(t, 0.12, 0.12), 2);
      /* The faces were a hard-coded white-to-#F1EEE7 gradient: right on this effect's own beige
         theme, a full white card wall on the dark stage the rater actually uses. The card is the
         stage's own sheet, lifted off it. */
      var faceBg = 'linear-gradient(' + fade(sc.raised, chrome) + ',' + fade(sc.paper, chrome) + ')';
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
        /* cast shadow stays plain black — a shadow darkens whatever it falls on — but the hairline
           round the tile has to READ, so it comes off ctx.sc.edge, which leans light on a dark
           stage and dark on a light one */
        var sh = '0 ' + (1.5 + 7 * s).toFixed(1) + 'px ' + (3 + 10 * s).toFixed(1) +
          'px rgba(0,0,0,' + (chrome * (0.10 + 0.20 * s)).toFixed(3) +
          '), inset 0 0 0 1px ' + fade(sc.edge, chrome * 0.55);
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
      stage._panel = panel;

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
        /* No line-height of its own. At 1.26em the sign's two rows sat 0.18em further apart than
           the canonical pair, so even with the LEDs off the headline was the wrong shape at both
           endpoints. The stage's 1.08 is the shape the engine parks on. */
        var txt = FX.el('div', null, { textAlign: 'center', maxWidth: '100%' });
        twoWeights(txt, s);
        inner.appendChild(txt); outer.appendChild(inner); glow.appendChild(outer);
        w.appendChild(glow);
        return { glow: glow, outer: outer, inner: inner, txt: txt };
      }

      stage._ghost = sign(ctx.from);
      stage._out = sign(ctx.from);
      stage._in = sign(ctx.to);
      stage._signs = [stage._ghost, stage._out, stage._in];

      var scan = FX.el('div', null, { position: 'absolute', left: 0, top: 0, right: 0, bottom: 0 });
      dotBg(scan, '#FFF2D2', 44);
      w.appendChild(scan);
      stage._scan = scan;
    },
    frame: function (stage, t) {
      var e = E.inOutCubic(t);
      var P = -12 + e * 124;                       /* refresh-bar position, % of board */
      var soft = 5;

      /* THE BOARD IS NOT ALWAYS A BOARD.
         The panel of unlit dots, the mask that chops the glyphs into pixels, the LED bloom and the
         hairline text stroke were all on for the whole transition, so ~29% of the stage differed
         from the canonical frame at t=0.008 AND at t=0.992 — the endpoints were an LED sign where
         the engine parks plain text. All of it now ramps up out of, and back into, an ordinary
         rendered line: at the ends the dots literally grow until they swallow their own cells and
         the message resolves into solid type, which is also how a real sign looks when you walk
         far enough away from it. */
      var chrome = chromeAt(t, 0.09, 0.11);
      stage._panel.style.opacity = chrome.toFixed(3);
      /* 42% of the cell's half-diagonal is a round LED; past 100% the circle swallows the whole
         cell, which is the same thing as having no mask at all */
      var rad = lerp(112, 42, chrome);
      for (var i = 0; i < stage._signs.length; i++) {
        dotMask(stage._signs[i].inner, rad);
        stage._signs[i].txt.style.WebkitTextStroke =
          (0.014 * chrome).toFixed(4) + 'em currentColor';
      }

      sweepMask(stage._out.outer, 'linear-gradient(90deg, transparent ' + (P - 1).toFixed(2) +
        '%, #000 ' + (P + soft).toFixed(2) + '%)');
      sweepMask(stage._in.outer, 'linear-gradient(90deg, #000 ' + (P - soft).toFixed(2) +
        '%, transparent ' + (P + 1).toFixed(2) + '%)');

      /* phosphor persistence: the whole old message hangs on faintly, then decays */
      stage._ghost.glow.style.opacity =
        String(0.16 * chrome * (1 - E.outQuad(span(t, 0.10, 0.92))));
      sweepMask(stage._ghost.outer, 'linear-gradient(90deg, #000 0%, #000 100%)');

      /* the bar itself: a band of every LED on the board driven hard */
      var band = 'linear-gradient(90deg, transparent ' + (P - 7).toFixed(2) +
        '%, rgba(0,0,0,.30) ' + (P - 3.2).toFixed(2) +
        '%, #000 ' + P.toFixed(2) + '%, rgba(0,0,0,.35) ' + (P + 2.4).toFixed(2) +
        '%, transparent ' + (P + 5.5).toFixed(2) + '%)';
      sweepMask(stage._scan, band);
      stage._scan.style.opacity = String(chrome * 0.85 * Math.min(1, span(t, 0, 0.06) * 1) *
        (1 - E.inQuad(span(t, 0.88, 1))));

      /* glow, and a two-beat settle flash once the bar clears the right edge. The bloom is ink
         spread outside the glyph, so it has to shrink to nothing with the rest of the hardware or
         the ink box measures wider than the canon at the ends. */
      var flash = 1 + 0.34 * Math.sin(span(t, 0.86, 1) * Math.PI * 2) * (1 - span(t, 0.86, 1));
      var bloom = (0.07 * chrome).toFixed(4) + 'em';
      stage._out.glow.style.filter = 'drop-shadow(0 0 ' + bloom + ' rgba(255,179,36,.55))';
      stage._in.glow.style.filter =
        'drop-shadow(0 0 ' + bloom + ' rgba(255,179,36,.6)) brightness(' + flash.toFixed(3) + ')';
      stage._ghost.glow.style.filter = 'blur(' + (0.4 * chrome).toFixed(3) + 'px)';
    },
    rest: function (stage) {
      sweepMask(stage._out.outer, 'linear-gradient(90deg, transparent 0%, transparent 100%)');
      sweepMask(stage._in.outer, 'linear-gradient(90deg, #000 0%, #000 100%)');
      stage._ghost.glow.style.opacity = '0';
      stage._scan.style.opacity = '0';
      stage._panel.style.opacity = '0';
      /* settled == the sign has resolved into ordinary type: dots off, mask open, no bloom */
      stage._signs.forEach(function (S) {
        dotMask(S.inner, 112);
        S.txt.style.WebkitTextStroke = '0em currentColor';
      });
      stage._in.glow.style.filter = 'none';
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

      /* The sheet in the machine is the SAME sheet the engine parks on: two rows, page type size,
         page line height. This used to set fontSize .9em, lineHeight 1.32em and run both halves
         into one flowing paragraph separated by a space — so the headline shrank 10%, re-wrapped
         wherever it liked and collapsed from two lines to one the instant the transition started
         (316px out at t=0.008, 250px at t=0.992). Only the striking is per character now. */
      function line(s) {
        var b = FX.el('div', null, {
          position: 'absolute', left: 0, top: 0, right: 0, bottom: 0,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          willChange: 'transform,opacity',
        });
        var inner = FX.el('div', null, { textAlign: 'center', maxWidth: '100%' });
        var rows = twoRows(inner);
        var nodes = [];
        function add(row, text, cls) {
          var cw = FX.charWords(text, cls);
          row.appendChild(cw.frag);
          cw.nodes.forEach(function (nd) { nodes.push(nd); });
        }
        add(rows[0], s.light, 'fx-light');
        add(rows[1], s.bold, 'fx-bold');
        b.appendChild(inner);
        w.appendChild(b);
        return { box: b, inner: inner, nodes: nodes };
      }

      stage._old = line(ctx.from);
      stage._new = line(ctx.to);

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

      /* the carriage, the caret and nothing else: hardware fades out before the endpoints */
      var chrome = chromeAt(t, 0.06, 0.06);

      /* ---- strike schedule.
         The last key used to land at 0.985 and take another 0.066 to settle, so at t=0.992 the
         final four glyphs were still 30% opaque, 1.3x oversized and rotated — the frame the engine
         hands over to the canonical text. Everything is struck and settled by ~0.95 now. */
      var T0 = 0.26, T1 = 0.88;
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
      /* The caret is a full-line-height bar of ink sitting outside the text, so at the endpoints
         it both widens and heightens the ink box against a canonical frame that has no caret at
         all. Squared so it is gone, not merely faint, by the time the engine takes over. */
      /* ...and while the carriage is CROSSING it is a lone tick in empty space: the old line has
         already rolled away and the first key has not struck yet, so a bar of ink slides across
         the middle of a blank stage attached to nothing. Fade it through the crossing, so it
         leaves the end of the old line and arrives at the head of the new one without being
         visible in between — which is also what you see on a real machine, where the thing your
         eye follows is the carriage, not the print point. */
      var crossing = last < 0 ? Math.sin(Math.PI * clamp(sh, 0, 1)) : 0;
      stage._caret.style.opacity =
        String((last < 0 ? 0.58 : 0.5) * chrome * chrome * (1 - crossing));
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

  /* The shove has to START FROM REST. Bare outQuint dumps 6% of the journey into the first 1% of
     the transition, so at t=0.008 the rail — and with it the whole headline — had already jumped
     75px sideways out of the canonical position the engine had just been showing. Gating it with a
     short ease-in keeps the punch (three quarters of the travel is still done by t=0.17) and puts
     the frame next to the canonical one within a quarter of a pixel of it. */
  function slideBase(u) {
    var s = span(u, 0, 0.66);
    return E.outQuint(s) * E.inQuad(clamp(s / 0.22, 0, 1));
  }
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
      var stator = scale(w, '2px', false, 'D');

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
        /* No line-height or padding of its own: the rail carries the canonical two-line block,
           not a looser copy of it. At 1.26em the two halves sat 0.18em further apart than the
           text the engine shows either side of this transition. */
        var inner = FX.el('div', null, { textAlign: 'center', maxWidth: '100%' });
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
      stage._scales = [stator, sl];
    },
    frame: function (stage, t) {
      var D = stage._D;
      /* THE INSTRUMENT IS NOT PART OF THE HEADLINE.
         Both engraved scales run 340% of the stage wide and the glass cursor is a lit panel down
         the middle of it, so at t=0.008 and t=0.992 the ink on screen spanned the whole stage
         while the canonical frame either side was two centred lines of text — 659px and 706px out,
         the worst pair of endpoints in the file. The rail's hardware now racks up out of nothing
         and back into it; only the text is left at the ends. */
      var chrome = chromeAt(t, 0.12, 0.12);
      stage._scales[0].style.opacity = (0.55 * chrome).toFixed(3);
      stage._scales[1].style.opacity = (0.55 * chrome).toFixed(3);
      stage._glass.style.opacity = chrome.toFixed(3);

      var x = slideX(t, D);
      var v = Math.abs(slideX(clamp(t + 0.012, 0, 1), D) - slideX(clamp(t - 0.012, 0, 1), D));
      var bl = Math.min(3.4, v * 0.09);
      /* the rail's lean off true goes with the hardware — a rotation about the stage centre moves
         the text too, and at the endpoints the text must not move */
      var tilt = 0.34 * (1 - slideBase(t)) * chrome;
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
