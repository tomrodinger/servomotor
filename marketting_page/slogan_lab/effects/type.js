/* Type family — typewriter and text-manipulation transitions.
 *
 * Six effects, all pure functions of progress t:
 *   type-backspace   classic backspace-then-retype with a blinking block cursor
 *   type-decode      cipher churn that locks in from BOTH ENDS of the line, inward
 *   type-find        editor search-and-replace: swap each word in its slot, then re-justify
 *   type-shuffle     letters shared by both lines travel to their new seats (measured FLIP)
 *   type-redact      marker bars black the line out, re-shape underneath, then wipe back
 *   type-stencil     ink drains out of the letters, the cut-outs swap, new ink floods up
 *
 * No timers, no CSS animation, no wall clock: everything is placed from t.
 *
 * Shared trick: `measure()` lays each slogan out for real — naturally wrapped, centred, with
 * the light/bold split — reads every word/character box, and then rebuilds those boxes as
 * absolutely positioned spans. That buys three things at once:
 *   1. frame(0) and frame(1) are pixel-exact rest states of the two slogans (no padded cells
 *      leaving gaps, which is what a naive A/B character alignment does to the settled line),
 *   2. nothing reflows during a transition — only transform/opacity/clip-path move,
 *   3. long slogans wrap to 2-3 lines exactly as they would at rest, and never overflow.
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, lerp = FX.lerp, clamp = FX.clamp;

  var MONO = 'ui-monospace, SFMono-Regular, Menlo, "DejaVu Sans Mono", monospace';

  /* deterministic pseudo-random in [0,1) from an integer key — never Math.random() */
  function rnd(k, s) {
    var x = Math.sin(k * 12.9898 + (s || 0) * 78.233) * 43758.5453;
    return x - Math.floor(x);
  }

  /* ------------------------------------------------------------------ layout measurement */

  /* Lay the slogan out invisibly inside the stage exactly as the ENGINE'S CANON lays it out —
     a flex-centred box holding the light half on its own row and the bold half on the next —
     and return one box per word (gran 'word') or per character (gran 'char'):
       { text, weight, row, x, y, w, h, i }   x/y in stage-relative CSS px.

     The two rows are not cosmetic. The engine draws the canonical two-line headline at t=0 and
     t=1; if this measurement flowed "<light> <bold>" as one paragraph (it used to) every effect
     built on it wrapped at a different point and sat at a different height than the canon, so
     the headline jumped at both ends of every transition. */
  function measure(stage, light, bold, gran) {
    var holder = FX.el('div', null, {
      position: 'absolute', left: '0', top: '0', right: '0', bottom: '0',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
      textAlign: 'center', visibility: 'hidden', pointerEvents: 'none',
    });
    var box = FX.el('div');
    holder.appendChild(box);
    var recs = [];
    [[light, 300], [bold, 650]].forEach(function (part, pi) {
      var seg = FX.el('div', null, { fontWeight: String(part[1]), whiteSpace: 'normal' });
      var ws = String(part[0]).split(/\s+/).filter(Boolean);
      ws.forEach(function (wtext, wi) {
        var w = FX.el('span', null, { display: 'inline-block', whiteSpace: 'nowrap' });
        if (gran === 'char') {
          for (var i = 0; i < wtext.length; i++) {
            var c = FX.el('span', null, { display: 'inline-block' });
            c.textContent = wtext[i];
            w.appendChild(c);
            recs.push({ el: c, text: wtext[i], weight: part[1], row: pi });
          }
        } else {
          w.textContent = wtext;
          recs.push({ el: w, text: wtext, weight: part[1], row: pi });
        }
        seg.appendChild(w);
        if (wi < ws.length - 1) seg.appendChild(document.createTextNode(' '));
      });
      box.appendChild(seg);
    });
    stage.appendChild(holder);
    var sr = stage.getBoundingClientRect();
    /* Coordinates are relative to the CENTRE of the stage, not its top-left corner. The lab
       calls seek() before the focused tile has finished growing to its final height, so a
       top-left origin freezes a stale height into every position and the whole headline paints
       ~45px above the canon. Measured from the centre — and painted inside a layer anchored at
       the centre — the block re-centres itself for free, exactly as the canon's flex box does. */
    var cx = sr.left + sr.width / 2, cy = sr.top + sr.height / 2;
    var out = recs.map(function (r, k) {
      var b = r.el.getBoundingClientRect();
      return {
        text: r.text, weight: r.weight, row: r.row, i: k,
        x: b.left - cx, y: b.top - cy, w: b.width, h: b.height,
      };
    });
    stage.removeChild(holder);
    return out;
  }

  /* A zero-size box pinned to the centre of the stage: every measured coordinate is an offset
     from that point, so the whole composition stays centred if the stage is resized after the
     measurement (see measure()). */
  function mkLayer(stage) {
    var L = FX.el('div', null, { position: 'absolute', left: '50%', top: '50%', width: '0', height: '0' });
    stage.appendChild(L);
    return L;
  }

  /* an absolutely positioned span sitting exactly on a measured box */
  function place(L, it, css) {
    var s = FX.el('span', null, {
      position: 'absolute', left: it.x + 'px', top: it.y + 'px',
      display: 'inline-block', whiteSpace: 'pre', fontWeight: String(it.weight),
      willChange: 'transform,opacity',
    });
    if (css) for (var k in css) s.style[k] = css[k];
    s.textContent = it.text;
    L.appendChild(s);
    return s;
  }

  function fontSizeOf(stage) { return parseFloat(getComputedStyle(stage).fontSize) || 26; }

  /* ================================================================= 1. BACKSPACE ===== */
  /* The old line is rubbed out right-to-left with accelerating key-repeat, the caret sits and
     blinks for a beat, then the new line is typed with a human cadence: longer pauses after
     punctuation, per-key jitter from a hash so it is never robotic. */

  function typeUnits(host, text, weight) {
    var units = [];
    var parts = String(text).split(/(\s+)/);
    parts.forEach(function (p) {
      if (!p) return;
      if (/^\s+$/.test(p)) {
        /* an ordinary collapsible space: at a line break it must disappear, otherwise the
           wrapped line starts with a stray indent */
        var g = FX.el('span', null, { display: 'inline' });
        g.textContent = p;
        host.appendChild(g);
        units.push({ el: g, ch: ' ', disp: 'inline' });
        return;
      }
      var w = FX.el('span', null, {
        display: 'inline-block', whiteSpace: 'nowrap', fontWeight: String(weight),
      });
      for (var i = 0; i < p.length; i++) {
        var c = FX.el('span', null, { display: 'inline-block', whiteSpace: 'pre' });
        c.textContent = p[i];
        w.appendChild(c);
        units.push({ el: c, ch: p[i], disp: 'inline-block' });
      }
      host.appendChild(w);
    });
    return units;
  }

  /* normalised moment (0..1 of the typing window) at which each unit lands */
  function cadence(units) {
    var acc = 0, w = [], i;
    for (i = 0; i < units.length; i++) {
      var prev = i > 0 ? units[i - 1].ch : '';
      var c = 1 + 0.55 * rnd(i, 3);
      if (/[,;:]/.test(prev)) c += 1.7;
      if (/[.!?]/.test(prev)) c += 2.8;
      if (units[i].ch === ' ') c += 0.25;
      acc += c;
      w.push(acc);
    }
    /* squeeze into the first 88% of the window: the last keystroke must land WELL before t=1,
       otherwise the final character is still fading in at rest and the loop stutters */
    for (i = 0; i < w.length; i++) w[i] = (w[i] / (acc || 1)) * 0.88;
    return w;
  }

  FX.register({
    id: 'type-backspace',
    name: 'Backspace & Retype',
    family: 'Type',
    blurb: 'Rubs the old line out at key-repeat speed, then types the new one with a human cadence.',
    duration: 2600,
    dwell: 3400,
    theme: { font: MONO },
    setup: function (stage, ctx) {
      stage.innerHTML = '';

      /* One canonically-centred two-row block per slogan: light half on row 1, bold half on
         row 2, exactly where the engine's canon puts them. Each row's text lives in an
         inline-block whose width and height are LOCKED to the settled measurement, so
         characters appearing or vanishing never re-centre the line, never collapse the row and
         never move the wrap — while the fully typed state is pixel-identical to the canon. */
      function block(light, bold) {
        var holder = FX.el('div', null, {
          position: 'absolute', left: '0', top: '0', right: '0', bottom: '0',
          display: 'flex', alignItems: 'center', justifyContent: 'center', textAlign: 'center',
        });
        var box = FX.el('div');
        holder.appendChild(box);
        stage.appendChild(holder);
        var units = [], inners = [];
        [[light, 300], [bold, 650]].forEach(function (part) {
          var row = FX.el('div', null, { textAlign: 'center' });
          /* Characters are hidden with VISIBILITY, not display, so the block always occupies the
             fully-typed layout: same width, same height, same wrap points, from the first frame to
             the last. That is what lets the row stay text-align:center like the canonical layer.
             It used to toggle display and left-align inside a width locked to the measured text —
             which held still at 1100px, where none of the slogans wrap, and came apart at 680px,
             where the long one does: the effect left-aligned each wrapped line while the canon
             centred it, 53px out at the start and 66px at the end. Revealing rather than inserting
             costs nothing and is correct at every width. */
          var inner = FX.el('span', null, {
            display: 'inline-block', textAlign: 'center', verticalAlign: 'top',
            whiteSpace: 'normal', fontWeight: String(part[1]),
          });
          units = units.concat(typeUnits(inner, part[0], part[1]));
          row.appendChild(inner);
          box.appendChild(row);
          inners.push(inner);
        });
        // No width/height lock: with visibility-hiding the block is already at its settled size.

        return { holder: holder, units: units, first: inners[0] };
      }

      var a = block(ctx.from.light, ctx.from.bold);
      var b = block(ctx.to.light, ctx.to.bold);

      /* zero-width block caret: takes no layout space, so it can never push a wrap */
      var cur = FX.el('span', null, {
        display: 'inline-block', width: '0.58em', height: '1.04em', marginRight: '-0.58em',
        background: 'currentColor', verticalAlign: '-0.19em', borderRadius: '1px',
        willChange: 'opacity',
      });

      stage._hA = a.holder; stage._hB = b.holder;
      stage._A = a.units; stage._B = b.units;
      stage._flA = a.first; stage._flB = b.first;
      stage._cur = cur;
      stage._wB = cadence(b.units);
    },
    frame: function (stage, t) {
      var A = stage._A, B = stage._B, wB = stage._wB, nA = A.length;
      /* The retype starts the instant the erase finishes. There used to be a 0.05 gap between
         them — 0.13s on a 2.6s transition — during which the whole headline was gone and the only
         thing on the hero was a caret blinking on an empty stage. It reads as the page having
         broken rather than as a pause. The blocks still swap while nothing is showing, but that
         window is now a single frame instead of eight. */
      var tp = span(t, 0.02, 0.34);          // erase, done at 0.34
      var tq = span(t, 0.35, 0.97);          // type, starts immediately after
      var typing = t >= 0.345;               // the two blocks swap in the one frame between
      var last = null, i;

      stage._hA.style.visibility = typing ? 'hidden' : 'visible';
      stage._hB.style.visibility = typing ? 'visible' : 'hidden';

      for (i = 0; i < nA; i++) {
        /* unit i is the (nA - i)-th keystroke of the backspace run; the exponent makes the
           key-repeat accelerate the way a held delete key does */
        var tau = 0.12 + 0.88 * Math.pow((nA - i) / nA, 0.85);
        var on = tp < tau;
        A[i].el.style.visibility = on ? 'visible' : 'hidden';
        A[i].el.style.opacity = '1';
        if (!typing && on && A[i].disp === 'inline-block') last = A[i].el;
      }
      for (i = 0; i < B.length; i++) {
        var born = wB[i];
        if (tq < born) {
          B[i].el.style.visibility = 'hidden';
        } else {
          var age = clamp((tq - born) / 0.030, 0, 1);
          B[i].el.style.visibility = 'visible';
          B[i].el.style.opacity = String(lerp(0.15, 1, age));
          if (B[i].disp === 'inline-block') {
            B[i].el.style.transform = 'translateY(' + (0.11 * (1 - age)) + 'em)';
            if (typing) last = B[i].el;
          }
        }
      }

      var cur = stage._cur;
      if (last) last.parentNode.insertBefore(cur, last.nextSibling);
      else {
        var fl = typing ? stage._flB : stage._flA;
        fl.insertBefore(cur, fl.firstChild);
      }
      var busy = (t > 0.025 && t < 0.34) || (t > 0.39 && t < 0.94);
      var ph = (t * 5) % 1;                       // blink is a function of t, not of the clock
      /* The canon carries no caret, so it fades in after the start and out before the end instead
         of popping into existence on top of a settled line. The old window opened at t=0, which
         is not "after the start": the block caret hangs 0.58em past the last glyph, so by t=0.008
         it was already legible and the headline measured 25px wider than the canonical FROM text —
         a visible bar appearing out of nowhere at the moment the transition starts. Nothing is
         erased before t=0.02 either, so hold the caret back until the delete key does something,
         and have it gone well before the last keystroke settles. */
      var edge = span(t, 0.02, 0.10) * (1 - span(t, 0.90, 0.97));
      cur.style.opacity = String((busy ? 1 : (ph < 0.55 ? 1 : 0.05)) * edge);
    },
  });

  /* ==================================================================== 2. DECODE ===== */
  /* Reads like a cipher resolving: both ENDS of the line lock first and the churn front closes
     on the middle. The old characters dissolve just ahead of the front, so the churn never
     sits on top of legible text — at any t you get [new][churn][old][churn][new]. */

  var GLY = '0123456789ABCDEF#%&*+=<>/\\|_-[]{}?$!';

  FX.register({
    id: 'type-decode',
    name: 'Decode Inward',
    family: 'Type',
    blurb: 'Cipher churn locks in at both ends of the line and closes on the middle.',
    duration: 1600,
    dwell: 4200,
    theme: { bg: '#07100B', fg: '#D6F7DC', accent: '#57F287', font: MONO },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var A = measure(stage, ctx.from.light, ctx.from.bold, 'char');
      var B = measure(stage, ctx.to.light, ctx.to.bold, 'char');
      var L = mkLayer(stage);

      function depth(i, n) { return n > 1 ? Math.min(i, n - 1 - i) / ((n - 1) / 2) : 1; }

      stage._old = A.map(function (it, i) {
        var e = place(L, it);
        /* ends dissolve first, middle last */
        return { el: e, go: 0.08 + 0.55 * depth(i, A.length), k: i };
      });
      stage._new = B.map(function (it, i) {
        var e = place(L, it, { willChange: 'transform,opacity,text-shadow' });
        e.style.opacity = '0';
        var d = depth(i, B.length);
        return { el: e, ch: it.text, churn: 0.14 + 0.55 * d, k: i };
      });
    },
    frame: function (stage, t) {
      var o = stage._old, nw = stage._new, i;
      for (i = 0; i < o.length; i++) {
        var c = o[i];
        var p = E.inQuad(span(t, c.go - 0.06, c.go + 0.06));
        c.el.style.opacity = String(1 - p);
        c.el.style.transform = 'scale(' + (1 - 0.28 * p).toFixed(3) + ')'
          + ' translateY(' + ((rnd(c.k, 5) - 0.5) * 0.5 * p).toFixed(3) + 'em)';
      }
      for (i = 0; i < nw.length; i++) {
        var b = nw[i], el = b.el;
        var lock = b.churn + 0.16;
        if (t < b.churn) {
          el.style.opacity = '0';
          el.textContent = b.ch;
          continue;
        }
        if (t < lock) {
          var q = (t - b.churn) / 0.16;
          var step = Math.floor(q * 7);
          el.textContent = GLY[Math.floor(rnd(b.k * 7 + step, b.k) * GLY.length)];
          el.style.color = '#57F287';
          el.style.opacity = String(lerp(0.35, 0.9, q));
          el.style.transform = 'translateY(' + ((rnd(b.k, step + 1) - 0.5) * 0.16).toFixed(3) + 'em)'
            + ' scale(' + (0.9 + 0.1 * q).toFixed(3) + ')';
          el.style.textShadow = 'none';
        } else {
          /* locked: a short flare as it snaps to the real glyph, then plain text */
          var f = 1 - E.outQuart(span(t, lock, lock + 0.14));
          el.textContent = b.ch;
          el.style.opacity = '1';
          el.style.color = f > 0.02 ? '#BFFFD0' : '';
          el.style.transform = 'scale(' + (1 + 0.12 * f).toFixed(3) + ')';
          el.style.textShadow = f > 0.02
            ? '0 0 ' + (12 * f).toFixed(1) + 'px rgba(87,242,135,' + (0.85 * f).toFixed(2) + ')'
            : 'none';
        }
      }
    },
    rest: function (stage) {
      stage._old.forEach(function (c) { c.el.style.opacity = '0'; });
      stage._new.forEach(function (b) {
        b.el.textContent = b.ch;
        b.el.style.opacity = '1'; b.el.style.color = '';
        b.el.style.transform = 'none'; b.el.style.textShadow = 'none';
      });
    },
  });

  /* ============================================================== 3. FIND & REPLACE ===== */
  /* A yellow selection sweeps left to right. Each selected word rolls up out of its slot and
     its replacement rolls in — fitted to the old word's slot with a scaleX, so the line keeps
     the old geometry and nothing ever collides. When the sweep is done the whole line
     re-justifies to its natural setting and any added words drop into the gaps. */

  FX.register({
    id: 'type-find',
    name: 'Find & Replace',
    family: 'Type',
    blurb: 'Selection sweeps word by word swapping each in place; the line then re-justifies.',
    duration: 2100,
    dwell: 4000,
    theme: { bg: '#FFFFFF', fg: '#16181D', accent: '#FFD84A' },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var A = measure(stage, ctx.from.light, ctx.from.bold, 'word');
      var B = measure(stage, ctx.to.light, ctx.to.bold, 'word');
      var L = mkLayer(stage);
      var fs = fontSizeOf(stage);
      var padX = 0.13 * fs, padY = 0.05 * fs;
      var n = Math.max(A.length, B.length), ops = [], k;

      for (k = 0; k < n; k++) {
        var a = A[k] || null, b = B[k] || null;
        var slot = a || b;
        var op = { k: k, a: a, b: b, slot: slot, ins: !a, padX: padX, padY: padY };

        if (a) {
          var ca = FX.el('div', null, {
            position: 'absolute', left: a.x + 'px', top: a.y + 'px',
            width: Math.ceil(a.w + 2) + 'px', height: Math.ceil(a.h) + 'px',
            overflow: 'hidden', zIndex: '2',
          });
          var ia = FX.el('span', null, {
            position: 'absolute', left: '0', top: '0', whiteSpace: 'pre',
            fontWeight: String(a.weight), willChange: 'transform',
          });
          ia.textContent = a.text;
          ca.appendChild(ia); L.appendChild(ca);
          op.ca = ca; op.ia = ia; op.ah = a.h;
        }
        if (b) {
          /* The replacement lives in the OLD word's slot until the re-justify — but at its own
             natural width, CENTRED in that slot, not stretched to fill it.
             It used to scale horizontally by clamp(a.w / b.w, 0.52, 1.85), which keeps the line's
             geometry perfectly stable and mangles the type doing it: an 85% stretch turns
             "one motor." into a different, much heavier face, and a 48% squeeze makes "The" look
             like a condensed cut. Half the transition was set in fonts the page does not own.
             Letting each word keep its own width costs a little raggedness mid-sweep, which is
             what a real find-and-replace looks like before it re-justifies, and is the thing this
             effect is imitating. */
          op.sx = 1;
          op.cx = a ? (a.w - b.w) / 2 : 0;              // centred in the old slot until it moves
          var cb = FX.el('div', null, {
            position: 'absolute', left: slot.x + 'px', top: slot.y + 'px',
            width: Math.ceil(b.w + 2) + 'px', height: Math.ceil(Math.max(b.h, slot.h)) + 'px',
            overflow: 'hidden', zIndex: '2', transformOrigin: '0 50%',
            willChange: 'transform',
          });
          var ib = FX.el('span', null, {
            position: 'absolute', left: '0', top: '0', whiteSpace: 'pre',
            fontWeight: String(b.weight), willChange: 'transform',
          });
          ib.textContent = b.text;
          cb.appendChild(ib); L.appendChild(cb);
          op.cb = cb; op.ib = ib; op.bh = Math.max(b.h, slot.h);
          op.dx = b.x - slot.x; op.dy = b.y - slot.y;
        }
        var hl = FX.el('div', null, {
          position: 'absolute', left: '0', top: '0', width: '1px',
          height: Math.ceil(slot.h + 2 * padY) + 'px', background: '#FFD84A',
          transformOrigin: '0 0', zIndex: '1', willChange: 'transform',
        });
        L.appendChild(hl);
        op.hl = hl;
        ops.push(op);
      }
      stage._ops = ops;
    },
    frame: function (stage, t) {
      var ops = stage._ops, n = ops.length;
      for (var k = 0; k < n; k++) {
        var o = ops[k], f = k / n;
        /* the re-justify has to FINISH clearly before t=1: while it runs each word is still
           scaled into its old slot, and a word left at scaleX 0.8 at t=0.98 snaps to the canon */
        var rs = 0.66 + f * 0.05;                 // re-justify window for this word
        var grow, roll, coll, hx, hw, hy;

        if (o.ins) {
          /* a word the new line adds: it has no slot to sit in during the sweep, so it lands
             during the re-justify, as the line opens up a space for it */
          grow = E.outQuart(span(t, rs, rs + 0.05));
          roll = E.outCubic(span(t, rs + 0.04, rs + 0.14));
          coll = E.inOutCubic(span(t, rs + 0.11, rs + 0.20));
        } else {
          /* the sweep starts a beat after t=0 so the first selection does not appear already
             half-drawn on top of the settled line the engine just handed over */
          var l = 0.03 + f * 0.36;
          grow = E.outQuart(span(t, l, l + 0.10));
          roll = E.inOutCubic(span(t, l + 0.07, l + 0.21));
          coll = E.inOutCubic(span(t, l + 0.23, l + 0.34));
        }
        /* Each word RE-JUSTIFIES AS IT SWAPS, rather than the whole line re-justifying at the
           end. Holding every replacement in its predecessor's slot until t=0.66 is what forced
           the old horizontal scaling: without it, words of different length pile into each other
           for two thirds of the transition. Moving each word to its final position on its own
           roll means the line is only ever ragged at the boundary between the part already
           replaced and the part not yet reached — which is exactly what an editor doing a
           find-and-replace looks like, and is the thing this effect is imitating. */
        var rf = o.ins ? 1 : roll;

        /* selection rectangle: grows from the left, collapses to the right */
        hx = o.slot.x - o.padX; hw = (o.slot.w + 2 * o.padX) * grow; hy = o.slot.y - o.padY;
        if (!o.ins && o.b) {                       // it rides along with the re-justify
          hx = lerp(hx, o.b.x - o.padX, rf);
          hy = lerp(hy, o.b.y - o.padY, rf);
          hw = lerp(hw, (o.b.w + 2 * o.padX) * grow, rf);
        }
        var vis = hw * (1 - coll);
        o.hl.style.transform = 'translate(' + (hx + hw - vis).toFixed(2) + 'px,' + hy.toFixed(2) + 'px)'
          + ' scaleX(' + Math.max(0.0001, vis).toFixed(3) + ')';
        o.hl.style.opacity = vis > 0.4 ? '1' : '0';

        /* Old rolls out as the new rolls in — and the two must not both be legible in the same
           slot, or the swap reads as one word printed on top of another. So they cross-fade
           across the roll: whatever is in the slot, one of them owns it. */
        if (o.ia) {
          o.ia.style.transform = 'translateY(' + (-o.ah * roll).toFixed(2) + 'px)';
          o.ia.style.opacity = (1 - span(roll, 0.15, 0.55)).toFixed(3);
        }
        if (o.ib) {
          o.ib.style.opacity = span(roll, 0.50, 0.90).toFixed(3);
          o.ib.style.transform = 'translateY(' + (o.bh * (1 - roll)).toFixed(2) + 'px)';
          var sx = lerp(o.sx, 1, rf);
          var tx = lerp(o.cx, o.dx, rf), ty = lerp(0, o.dy, rf);
          o.cb.style.transform = 'translate(' + tx.toFixed(2) + 'px,' + ty.toFixed(2) + 'px)'
            + ' scaleX(' + sx.toFixed(4) + ')';
        }
      }
    },
    rest: function (stage) {
      stage._ops.forEach(function (o) {
        o.hl.style.opacity = '0';
        if (o.ia) { o.ia.style.transform = 'translateY(' + (-o.ah) + 'px)'; o.ia.style.opacity = '0'; }
        if (o.ib) {
          o.ib.style.opacity = '1';
          o.ib.style.transform = 'translateY(0px)';
          o.cb.style.transform = 'translate(' + (o.dx || 0) + 'px,' + (o.dy || 0) + 'px) scaleX(1)';
        }
      });
    },
  });

  /* =================================================================== 4. SHUFFLE ===== */
  /* Every letter the two lines share keeps its identity and physically travels to its new
     seat (measured FLIP, so the wrapped layout is respected); the arc is proportional to the
     distance travelled, so letters that barely move stay readable instead of flailing.
     Orphaned letters drop away, genuinely new letters rise into the gaps. */

  FX.register({
    id: 'type-shuffle',
    name: 'Letter Shuffle',
    family: 'Type',
    blurb: 'Shared letters fly to their new seats; orphans drop away and newcomers rise in.',
    duration: 1700,
    dwell: 4200,
    theme: { font: MONO },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var fs = fontSizeOf(stage);
      var A = measure(stage, ctx.from.light, ctx.from.bold, 'char');
      var B = measure(stage, ctx.to.light, ctx.to.bold, 'char');
      var L = mkLayer(stage);

      /* greedy match on identical glyphs, nearest in relative position */
      var used = [], i, j;
      for (i = 0; i < A.length; i++) used.push(false);
      var from = [];
      for (j = 0; j < B.length; j++) {
        var best = -1, bd = 1e9;
        for (i = 0; i < A.length; i++) {
          if (used[i] || A[i].text !== B[j].text) continue;
          var dd = Math.abs(i / (A.length || 1) - j / (B.length || 1));
          if (dd < bd) { bd = dd; best = i; }
        }
        if (best >= 0) used[best] = true;
        from.push(best);
      }

      var parts = [];
      for (j = 0; j < B.length; j++) {
        var b = B[j], a = from[j] >= 0 ? A[from[j]] : null;
        if (a) {
          var dx = b.x - a.x, dy = b.y - a.y;
          var dist = Math.sqrt(dx * dx + dy * dy);
          parts.push({
            kind: 'move', el: place(L, { text: b.text, weight: a.weight, x: a.x, y: a.y }),
            dx: dx, dy: dy, wa: a.weight, wb: b.weight, k: j,
            arc: Math.min(0.55 * fs, dist * 0.16) * (rnd(j, 5) < 0.5 ? -1 : 1),
            spin: clamp(dist / (6 * fs), 0, 1) * 16 * (rnd(j, 11) - 0.5) * 2,
            lead: 0.26 * (j / (B.length || 1)) + 0.06 * rnd(j, 7),
          });
        } else {
          parts.push({
            /* newcomers used to start landing only at 0.48 and finish at t=1 — which both
               hollowed out the middle of the transition and left the last letters still
               fading in at the moment the engine takes the canon back. They now overlap the
               travellers and are all seated by 0.94. */
            kind: 'in', el: place(L, b),
            lead: 0.42 + 0.28 * (j / (B.length || 1)), k: j,
          });
        }
      }
      for (i = 0; i < A.length; i++) {
        if (used[i]) continue;
        parts.push({ kind: 'out', el: place(L, A[i]), lead: 0.20 * (i / (A.length || 1)), k: i });
      }
      stage._parts = parts;
      stage._fs = fs;
    },
    frame: function (stage, t) {
      var parts = stage._parts, fs = stage._fs;
      for (var n = 0; n < parts.length; n++) {
        var q = parts[n], el = q.el, p;
        if (q.kind === 'move') {
          p = span(t, q.lead, q.lead + 0.48);
          var e = E.inOutQuart(p);
          var lift = Math.sin(Math.PI * p);
          el.style.transform =
            'translate(' + (q.dx * e).toFixed(2) + 'px,' + (q.dy * e - lift * q.arc).toFixed(2) + 'px)'
            + ' rotate(' + (q.spin * lift).toFixed(2) + 'deg)'
            + ' scale(' + (1 + 0.07 * lift).toFixed(3) + ')';
          el.style.opacity = String(1 - 0.22 * lift);
          el.style.fontWeight = String(p < 0.5 ? q.wa : q.wb);
        } else if (q.kind === 'out') {
          p = E.inQuad(span(t, q.lead, q.lead + 0.34));
          el.style.transform = 'translateY(' + (0.5 * fs * p).toFixed(2) + 'px)'
            + ' rotate(' + ((rnd(q.k, 3) - 0.5) * 34 * p).toFixed(2) + 'deg)'
            + ' scale(' + (1 - 0.32 * p).toFixed(3) + ')';
          el.style.opacity = String(1 - p);
          el.style.filter = p > 0.02 ? 'blur(' + (2.4 * p).toFixed(2) + 'px)' : 'none';
        } else {
          p = E.outCubic(span(t, q.lead, q.lead + 0.22));
          el.style.transform = 'translateY(' + (0.42 * fs * (1 - p)).toFixed(2) + 'px)'
            + ' scale(' + (0.88 + 0.12 * p).toFixed(3) + ')';
          el.style.opacity = String(p);
          el.style.filter = p < 0.98 ? 'blur(' + (2.0 * (1 - p)).toFixed(2) + 'px)' : 'none';
        }
      }
    },
  });

  /* ==================================================================== 5. REDACT ===== */
  /* Marker strokes slam across the line word by word (alternating a touch off-square), the
     page goes fully black — that is the t=0.5 picture, on purpose — and while it is covered
     the bars re-shape to the NEW line's word geometry: some stretch, deleted ones shrink to
     nothing, added ones are drawn in. Then the strokes wipe back right-to-left and the new
     line is underneath. Old and new text are never both visible. */

  FX.register({
    id: 'type-redact',
    name: 'Redacted',
    family: 'Type',
    blurb: 'Marker strokes black out every word, re-shape underneath, then wipe back.',
    duration: 1900,
    dwell: 4000,
    theme: {
      bg: '#F3F0E8', fg: '#191A1D', accent: '#111214',
      font: '"Courier New", ui-monospace, Menlo, monospace',
    },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var A = measure(stage, ctx.from.light, ctx.from.bold, 'word');
      var B = measure(stage, ctx.to.light, ctx.to.bold, 'word');
      var L = mkLayer(stage);
      var fs = fontSizeOf(stage);
      var padX = 0.16 * fs, padY = 0.05 * fs;
      var n = Math.max(A.length, B.length), bars = [], k;

      for (k = 0; k < n; k++) {
        var a = A[k] || null, b = B[k] || null;
        var ra = a
          ? { x: a.x - padX, y: a.y - padY, w: a.w + 2 * padX, h: a.h + 2 * padY }
          : { x: b.x + b.w / 2, y: b.y - padY, w: 0, h: b.h + 2 * padY };
        var rb = b
          ? { x: b.x - padX, y: b.y - padY, w: b.w + 2 * padX, h: b.h + 2 * padY }
          : { x: a.x + a.w / 2 + padX, y: a.y - padY, w: 0, h: a.h + 2 * padY };
        var h = Math.max(ra.h, rb.h);
        var bar = FX.el('div', null, {
          position: 'absolute', left: '0', top: '0', width: '1px', height: h + 'px',
          background: 'linear-gradient(96deg,#191A1D 0%,#0C0D0F 38%,#1E1F23 66%,#0F1013 100%)',
          transformOrigin: '0 50%', zIndex: '3', willChange: 'transform',
        });
        L.appendChild(bar);
        bars.push({
          bar: bar, h: h, ra: ra, rb: rb, k: k,
          tilt: (rnd(k, 4) - 0.5) * 1.6,
          ta: a ? place(L, a, { zIndex: '1' }) : null,
          tb: b ? place(L, b, { zIndex: '1' }) : null,
        });
        if (b) bars[bars.length - 1].tb.style.opacity = '0';
      }
      stage._bars = bars;
    },
    frame: function (stage, t) {
      var bars = stage._bars, n = bars.length;
      for (var k = 0; k < n; k++) {
        var o = bars[k], f = k / n;
        /* The first stroke starts just after t=0 and the last one clears just before t=1: a bar
           already half-drawn at t=0.02, or a sliver still standing at t=0.98, shows up as a
           flicker against the canonical line the engine draws either side of the transition. */
        var c0 = 0.04 + f * 0.22, c1 = c0 + 0.18;             // cover, left to right
        var m0 = 0.42 + f * 0.05, m1 = m0 + 0.16;             // re-shape while black
        var u0 = 0.62 + ((n - 1 - k) / n) * 0.18, u1 = u0 + 0.15;  // wipe back, right to left

        var c = Math.max(0, E.outBack(span(t, c0, c1)));      // a marker overshoots a little
        var m = E.inOutCubic(span(t, m0, m1));
        var u = E.inOutCubic(span(t, u0, u1));

        var x = lerp(o.ra.x, o.rb.x, m);
        var y = lerp(o.ra.y, o.rb.y, m);
        var w = lerp(o.ra.w, o.rb.w, m) * c;
        var vis = w * (1 - u);
        o.bar.style.transform =
          'translate(' + (x + w - vis).toFixed(2) + 'px,' + y.toFixed(2) + 'px)'
          + ' rotate(' + o.tilt.toFixed(2) + 'deg)'
          + ' scaleX(' + Math.max(0.0001, vis).toFixed(3) + ')';
        o.bar.style.opacity = vis > 0.5 ? '1' : '0';

        /* the swap happens under full cover, so it is never seen happening */
        if (o.ta) o.ta.style.opacity = t >= c1 ? '0' : '1';
        if (o.tb) o.tb.style.opacity = t >= m1 ? '1' : '0';
      }
    },
    rest: function (stage) {
      stage._bars.forEach(function (o) {
        o.bar.style.opacity = '0';
        if (o.ta) o.ta.style.opacity = '0';
        if (o.tb) o.tb.style.opacity = '1';
      });
    },
  });

  /* =================================================================== 6. STENCIL ===== */
  /* The ink drains out of the old letters from the top down, leaving hollow cut-outs; the
     cut-outs cross-fade to the shape of the new line; then fresh ink floods up into them,
     green at the waterline, settling to white as each letter tops out. */

  FX.register({
    id: 'type-stencil',
    name: 'Stencil Refill',
    family: 'Type',
    blurb: 'Ink drains out of the letters, the cut-outs swap shape, then new ink floods up.',
    duration: 1900,
    dwell: 4200,
    theme: { bg: '#141519', fg: '#F2F3F7', accent: '#7AB648' },
    setup: function (stage, ctx) {
      stage.innerHTML = '';
      var A = measure(stage, ctx.from.light, ctx.from.bold, 'char');
      var B = measure(stage, ctx.to.light, ctx.to.bold, 'char');
      var L = mkLayer(stage);

      function hollow(el) {
        el.style.webkitTextFillColor = 'transparent';
        el.style.webkitTextStrokeWidth = '0.030em';
        el.style.webkitTextStrokeColor = 'currentColor';
        el.style.opacity = '0';
        return el;
      }
      /* `pos` is how far along the line a glyph sits, 0..1. The cut-out swap is driven by it
         rather than by one global crossfade: the old shapes hand over to the new ones as a
         front travelling left to right, so the two alphabets are never both legible on the
         same spot — which is exactly what a straight crossfade produced (a double exposure of
         both slogans at t=0.5, the one frame of this effect anyone would screenshot). */
      stage._old = A.map(function (it, i) {
        return {
          fill: place(L, it, { willChange: 'clip-path' }),
          out: hollow(place(L, it, { willChange: 'opacity,transform' })),
          lead: (i / Math.max(1, A.length)) * 0.20,
          pos: i / Math.max(1, A.length - 1),
        };
      });
      stage._new = B.map(function (it, i) {
        var fill = place(L, it, { willChange: 'clip-path,text-shadow' });
        fill.style.clipPath = 'inset(100% 0 0 0)';
        return {
          fill: fill,
          out: hollow(place(L, it, { willChange: 'opacity,transform' })),
          lead: (i / Math.max(1, B.length)) * 0.16,
          pos: i / Math.max(1, B.length - 1),
        };
      });
    },
    frame: function (stage, t) {
      var o = stage._old, nw = stage._new, i;
      for (i = 0; i < o.length; i++) {
        var c = o[i];
        var drain = E.inOutCubic(span(t, c.lead, c.lead + 0.30));
        c.fill.style.clipPath = 'inset(' + (drain * 100).toFixed(2) + '% 0 0 0)';
        var goes = E.inQuad(span(t, 0.36 + 0.20 * c.pos, 0.46 + 0.20 * c.pos));
        var op = E.outQuad(span(t, c.lead, c.lead + 0.14)) * (1 - goes);
        c.out.style.opacity = op.toFixed(3);
        c.out.style.transform = 'translateY(' + (-0.10 * goes).toFixed(3) + 'em)';
      }
      for (i = 0; i < nw.length; i++) {
        var b = nw[i];
        var appear = E.outCubic(span(t, 0.40 + 0.20 * b.pos, 0.50 + 0.20 * b.pos));
        var flood = E.inOutCubic(span(t, 0.58 + b.lead, 0.82 + b.lead));
        b.fill.style.clipPath = 'inset(' + ((1 - flood) * 100).toFixed(2) + '% 0 0 0)';
        var wave = Math.sin(Math.PI * flood);
        b.fill.style.textShadow = wave > 0.02
          ? '0 0 ' + (12 * wave).toFixed(1) + 'px rgba(122,182,72,' + (0.55 * wave).toFixed(2) + ')'
          : 'none';
        b.out.style.opacity = (appear * (1 - E.inQuad(span(t, 0.72 + b.lead, 0.88 + b.lead)))).toFixed(3);
        b.out.style.transform = 'translateY(' + (0.10 * (1 - appear)).toFixed(3) + 'em)';
        b.fill.style.transform = wave > 0.02 ? 'scale(' + (1 + 0.05 * wave).toFixed(3) + ')' : 'none';
      }
    },
    rest: function (stage) {
      stage._old.forEach(function (c) {
        c.fill.style.clipPath = 'inset(100% 0 0 0)';
        c.out.style.opacity = '0';
      });
      stage._new.forEach(function (b) {
        b.fill.style.clipPath = 'inset(0 0 0 0)';
        b.fill.style.textShadow = 'none';
        b.fill.style.transform = 'none';
        b.out.style.opacity = '0';
      });
    },
  });
})();
