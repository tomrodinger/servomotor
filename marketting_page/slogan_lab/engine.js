/* Slogan FX engine.
 *
 * Every effect is a PURE FUNCTION OF PROGRESS. The engine owns time; an effect only ever answers
 * "what does the transition from A to B look like at progress t?". That buys three things:
 * deterministic screenshots, a scrub slider in the lab, and frame-exact GIF export.
 *
 * An effect is an object:
 *   {
 *     id:      'ink-bleed',                 // unique, kebab-case
 *     name:    'Ink Bleed',                 // shown in the lab
 *     family:  'Reveal',                    // grouping in the lab
 *     blurb:   'New line soaks into paper', // one line
 *     duration: 1100,                       // ms, transition length
 *     dwell:    4500,                       // ms, optional, time at rest before the next one
 *     theme:   { bg, fg, accent, font, weightLight, weightBold, italic }, // optional look for the tile
 *     setup(stage, ctx)                     // build the DOM for one A->B transition
 *     frame(stage, t, ctx)                  // place everything for progress t in [0,1]
 *     rest(stage, ctx)                      // optional: settled state (defaults to frame(...,1))
 *   }
 *
 * ctx = { from:{light,bold}, to:{light,bold}, i, next, W, H, reduced }
 *
 * Helpers on SloganFX (chars(), words(), lerp(), ease.*) are there so effects stay short.
 */
(function (global) {
  'use strict';

  var REGISTRY = [];

  var ease = {
    linear: function (t) { return t; },
    inQuad: function (t) { return t * t; },
    outQuad: function (t) { return t * (2 - t); },
    inOutQuad: function (t) { return t < .5 ? 2 * t * t : -1 + (4 - 2 * t) * t; },
    inCubic: function (t) { return t * t * t; },
    outCubic: function (t) { return (--t) * t * t + 1; },
    inOutCubic: function (t) { return t < .5 ? 4 * t * t * t : (t - 1) * (2 * t - 2) * (2 * t - 2) + 1; },
    inQuart: function (t) { return t * t * t * t; },
    outQuart: function (t) { return 1 - (--t) * t * t * t; },
    inQuint: function (t) { return t * t * t * t * t; },
    inExpo: function (t) { return t === 0 ? 0 : Math.pow(2, 10 * t - 10); },
    inOutQuart: function (t) { return t < .5 ? 8 * t * t * t * t : 1 - 8 * (--t) * t * t * t; },
    outQuint: function (t) { return 1 + (--t) * t * t * t * t; },
    outExpo: function (t) { return t === 1 ? 1 : 1 - Math.pow(2, -10 * t); },
    inOutExpo: function (t) {
      if (t === 0 || t === 1) return t;
      return t < .5 ? Math.pow(2, 20 * t - 10) / 2 : (2 - Math.pow(2, -20 * t + 10)) / 2;
    },
    outBack: function (t) { var c = 1.70158; return 1 + (c + 1) * Math.pow(t - 1, 3) + c * Math.pow(t - 1, 2); },
    outElastic: function (t) {
      if (t === 0 || t === 1) return t;
      return Math.pow(2, -10 * t) * Math.sin((t * 10 - .75) * (2 * Math.PI / 3)) + 1;
    },
    outBounce: function (t) {
      var n = 7.5625, d = 2.75;
      if (t < 1 / d) return n * t * t;
      if (t < 2 / d) return n * (t -= 1.5 / d) * t + .75;
      if (t < 2.5 / d) return n * (t -= 2.25 / d) * t + .9375;
      return n * (t -= 2.625 / d) * t + .984375;
    },
  };

  function lerp(a, b, t) { return a + (b - a) * t; }
  function clamp(v, a, b) { return v < a ? a : v > b ? b : v; }
  /* map t from [a,b] onto [0,1], clamped — the workhorse for staggering */
  function span(t, a, b) { return clamp((t - a) / (b - a), 0, 1); }

  /* Split a line into per-character spans. Spaces become non-collapsing spans so widths hold. */
  function chars(text, cls) {
    var frag = document.createDocumentFragment();
    var out = [];
    for (var i = 0; i < text.length; i++) {
      var s = document.createElement('span');
      s.className = 'fx-ch' + (cls ? ' ' + cls : '');
      s.style.display = 'inline-block';
      s.style.whiteSpace = 'pre';
      s.textContent = text[i];
      frag.appendChild(s);
      out.push(s);
    }
    return { frag: frag, nodes: out };
  }

  /* Split into word spans (spaces kept inside the preceding word span so wrapping behaves). */
  function words(text, cls) {
    var frag = document.createDocumentFragment();
    var out = [];
    var parts = text.split(/(\s+)/);
    for (var i = 0; i < parts.length; i++) {
      if (!parts[i]) continue;
      if (/^\s+$/.test(parts[i])) {
        var sp = document.createElement('span');
        sp.style.whiteSpace = 'pre';
        sp.textContent = parts[i];
        frag.appendChild(sp);
        continue;
      }
      var s = document.createElement('span');
      s.className = 'fx-w' + (cls ? ' ' + cls : '');
      s.style.display = 'inline-block';
      s.textContent = parts[i];
      frag.appendChild(s);
      out.push(s);
    }
    return { frag: frag, nodes: out };
  }

  /* Per-character spans, but grouped inside inline-block words so lines never break mid-word.
     Returns {frag, nodes} where nodes is the flat character list in reading order — so an effect
     can stagger across characters exactly as with chars(), while wrapping stays sane. */
  function charWords(text, cls) {
    var frag = document.createDocumentFragment();
    var nodes = [];
    var parts = text.split(/(\s+)/);
    for (var p = 0; p < parts.length; p++) {
      if (!parts[p]) continue;
      if (/^\s+$/.test(parts[p])) {
        var gap = document.createElement('span');
        gap.style.whiteSpace = 'pre';
        gap.textContent = parts[p];
        frag.appendChild(gap);
        continue;
      }
      var word = document.createElement('span');
      word.className = 'fx-word';
      word.style.display = 'inline-block';
      word.style.whiteSpace = 'nowrap';
      for (var i = 0; i < parts[p].length; i++) {
        var s = document.createElement('span');
        s.className = 'fx-ch' + (cls ? ' ' + cls : '');
        s.style.display = 'inline-block';
        s.textContent = parts[p][i];
        word.appendChild(s);
        nodes.push(s);
      }
      frag.appendChild(word);
    }
    return { frag: frag, nodes: nodes };
  }

  /* For per-character effects that morph A into B (scramble, odometer, flip...).
   *
   * Naively padding both whole strings to equal length breaks words across lines, because every
   * character becomes its own inline-block. This aligns them WORD BY WORD instead: you get a list
   * of word groups, each holding equal-length character pairs, so a group can be an inline-block
   * that wraps as a unit. Shorter side is space-padded; missing words become all-spaces.
   *
   * returns [ { pairs:[[chA,chB],...] }, ... ]
   */
  function alignWords(a, b) {
    var wa = a.split(/\s+/).filter(Boolean);
    var wb = b.split(/\s+/).filter(Boolean);
    var n = Math.max(wa.length, wb.length);
    var groups = [];
    for (var i = 0; i < n; i++) {
      var x = wa[i] || '', y = wb[i] || '';
      var m = Math.max(x.length, y.length);
      var pairs = [];
      // Pad with '' rather than ' '. A pad cell must occupy NO width: padding "One" out to
      // match "controller," with real spaces rendered as eight blank cells, which showed up
      // as badly spaced words at rest. Consumers treat '' and ' ' alike as blank.
      for (var j = 0; j < m; j++) pairs.push([x[j] || '', y[j] || '']);
      groups.push({ pairs: pairs });
    }
    return groups;
  }

  /* ------------------------------------------------------------------ stage palette
   *
   * Several effects are built around a sheet of paper — letterpress, riso misregistration, a torn
   * edge, a folding face. They hard-coded that sheet white, which is correct on a white page and
   * completely wrong on the dark hero: the stage went black -> white -> black on every cycle, a
   * full-screen flash with the headline inverted in the middle of it.
   *
   * The sheet is not white. The sheet is *whatever the stage already is*. Read that off the DOM
   * once, at setup, and hand the effect a matching pair of colours. This also makes the rater's
   * "On white" toggle honest — the same effect re-mounts as dark ink on white paper — instead of
   * needing a second hard-coded palette.
   */
  function parseRGB(str) {
    var m = /rgba?\(([^)]+)\)/.exec(str || '');
    if (!m) return null;
    var p = m[1].split(',').map(parseFloat);
    if (p.length > 3 && p[3] === 0) return null;         // fully transparent: keep looking
    return [p[0], p[1], p[2]];
  }

  function stageColors(node) {
    var rgb = null, n = node;
    while (n && n.nodeType === 1) {
      rgb = parseRGB(getComputedStyle(n).backgroundColor);
      if (rgb) break;
      n = n.parentElement;
    }
    if (!rgb) rgb = [10, 11, 12];
    var lum = (0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]) / 255;
    var dark = lum < 0.5;
    function mix(target, amt) {
      return 'rgb(' + rgb.map(function (c, i) {
        return Math.round(c + (target[i] - c) * amt);
      }).join(',') + ')';
    }
    return {
      dark: dark,
      /* the stage's own colour, opaque — use this for a sheet that must occlude what is under it */
      paper: 'rgb(' + rgb.map(Math.round).join(',') + ')',
      /* text on that sheet */
      ink: dark ? '#F5F5F7' : '#1d1d1f',
      /* a sheet lifted slightly off the stage, for a card that needs an edge without a border */
      raised: mix(dark ? [255, 255, 255] : [0, 0, 0], 0.07),
      sunken: mix(dark ? [0, 0, 0] : [255, 255, 255], 0.35),
      /* a hairline that reads on either ground */
      edge: mix(dark ? [255, 255, 255] : [0, 0, 0], 0.22),
      /* nudge the stage colour toward ink or away from it, 0..1 */
      shade: function (amt) { return mix(dark ? [255, 255, 255] : [0, 0, 0], amt); },
    };
  }

  function el(tag, cls, css) {
    var n = document.createElement(tag);
    if (cls) n.className = cls;
    if (css) for (var k in css) n.style[k] = css[k];
    return n;
  }

  /* ---------------------------------------------------------------- Player */

  /* `effect` may be a single effect OR an array of them. With an array, each transition uses a
   * different effect — the slogans and the transition style are independent choices.
   *
   * The effect for transition i is a PURE FUNCTION OF i (sequential by default, or a hash of i when
   * opts.pick === 'shuffle'). That is deliberate: it keeps seek(i, t) reproducible, so a mixed
   * player is still screenshottable and GIF-exportable exactly like a single-effect one. Picking at
   * random per cycle would break determinism and with it every verification tool here.
   */
  function Player(stage, effect, slogans, opts) {
    opts = opts || {};
    this.stage = stage;
    this.pool = Array.isArray(effect) ? effect.slice() : [effect];
    if (!this.pool.length) throw new Error('SloganFX.mount: no effect given');
    this.pick = opts.pick || 'sequential';
    this.slogans = slogans;
    this.i = opts.start || 0;
    this.playing = false;
    this.raf = null;
    this.t0 = 0;
    this.phase = 'rest';          // 'rest' | 'run'
    this.reduced = !!opts.reduced;
    this.optDwell = opts.dwell || null;
    this.optDuration = opts.duration || null;
    this._selectFx();
    this.onTick = opts.onTick || null;
    stage.classList.add('fx-stage');
    stage.setAttribute('aria-live', 'off');

    /* KERNING OFF, for the whole stage, canonical layer included.
     *
     * Most effects animate per word or per character, which means wrapping each one in its own
     * inline-block. A glyph in its own box cannot kern against its neighbour, so a split line
     * renders slightly WIDER than the same text as one run — measured at a consistent 11-12px on
     * a 797px line, about a quarter of a pixel per glyph. The canonical layer is one plain run and
     * does kern, so the headline breathed outward by ~6px a side the instant an effect took over,
     * and back in when it ended. Twenty-two effects showed it.
     *
     * There is no way to make a split line kern, so the fix is the other direction: stop the plain
     * run kerning too. Both renderings then agree exactly (0-1px), and the cost is a sub-pixel
     * looseness at 46px that is invisible on its own but would be very visible as a jump.
     *
     * This belongs on the stage rather than in each effect: the canonical layer has to agree with
     * whatever the effect does, so the two must be told the same thing from one place.
     */
    stage.style.fontKerning = 'none';
    stage.style.fontVariantLigatures = 'none';

    /* CANONICAL REST STATE.
     *
     * A slogan's parked appearance must be a function of (text, font) ALONE — never of whichever
     * effect happens to be running. Effects build wildly different DOM (per-character inline
     * blocks, drums, stacked layers), so letting them own the endpoints meant the same slogan
     * rendered with different spacing, and sometimes on a different number of lines, depending on
     * what ran last. That is what makes mixed transitions look broken.
     *
     * So the engine owns two layers:
     *   .fx-canon  — plain two-line text, engine-rendered, shown whenever t is 0 or 1
     *   .fx-layer  — handed to the effect as its stage, shown only while 0 < t < 1
     *
     * Every transition therefore starts at canonical A and ends at canonical B, and any effect
     * can follow any other.
     */
    stage.innerHTML = '';
    this.layer = el('div', 'fx-layer');
    this.layer.style.cssText = 'position:absolute;left:0;top:0;right:0;bottom:0';
    this.canon = el('div', 'fx-canon');
    this.canon.style.cssText = 'position:absolute;left:0;top:0;right:0;bottom:0;' +
      'display:flex;align-items:center;justify-content:center;text-align:center';
    var box = el('div');
    this.canonA = el('span', 'fx-light');
    this.canonB = el('span', 'fx-bold');
    this.canonA.style.display = 'block';
    this.canonB.style.display = 'block';
    box.appendChild(this.canonA);
    box.appendChild(this.canonB);
    this.canon.appendChild(box);
    stage.appendChild(this.layer);
    stage.appendChild(this.canon);

    this._build();
    this._render(1);
  }

  /* show the canonical text for one slogan pair and hide the effect's own DOM */
  Player.prototype._showCanon = function (pair) {
    this.canonA.textContent = pair.light;
    this.canonB.textContent = pair.bold;
    this.canon.style.visibility = 'visible';
    /* opacity AS WELL AS visibility, because `visibility: hidden` is the one form of hiding a
       DESCENDANT can override: any element inside that sets `visibility: visible` reappears, even
       though this layer is hidden. type-backspace does exactly that to reveal typed characters
       without disturbing the layout, and the result was its own text drawn on top of the canonical
       frame at t=1 — 0.8% of the stage, including a caret the canon does not have. A parent at
       opacity 0 cannot be overridden from inside, and unlike display:none it keeps the layer laid
       out, so effects that measure their own geometry still can. */
    this.layer.style.visibility = 'hidden';
    this.layer.style.opacity = '0';
  };

  Player.prototype._showEffect = function () {
    this.canon.style.visibility = 'hidden';
    this.layer.style.visibility = 'visible';
    this.layer.style.opacity = '1';
  };

  /* choose the effect for the CURRENT transition, and adopt its own timing */
  Player.prototype._selectFx = function () {
    var n = this.pool.length, k;
    if (n === 1) k = 0;
    else if (this.pick === 'shuffle') {
      var h = Math.sin(this.i * 78.233 + 12.9898) * 43758.5453;
      k = Math.floor((h - Math.floor(h)) * n) % n;
    } else k = ((this.i % n) + n) % n;
    this.fx = this.pool[k];
    this.fxIndex = k;
    this.dwell = this.optDwell || this.fx.dwell || 3200;
    this.duration = this.optDuration || this.fx.duration || 1000;
  };

  Player.prototype._ctx = function () {
    var a = this.slogans[this.i % this.slogans.length];
    var b = this.slogans[(this.i + 1) % this.slogans.length];
    return {
      from: { light: a[0], bold: a[1] },
      to: { light: b[0], bold: b[1] },
      i: this.i,
      next: (this.i + 1) % this.slogans.length,
      W: this.layer.clientWidth || this.stage.clientWidth,
      H: this.layer.clientHeight || this.stage.clientHeight,
      reduced: this.reduced,
      ease: ease, lerp: lerp, span: span, clamp: clamp,
      chars: chars, words: words, charWords: charWords, alignWords: alignWords, el: el,
    stageColors: stageColors,
      /* the ground this transition is running on; see stageColors() */
      sc: stageColors(this.stage),
    };
  };

  Player.prototype._build = function () {
    this._selectFx();
    this.ctx = this._ctx();
    this.layer.innerHTML = '';
    if (this.fx.setup) this.fx.setup(this.layer, this.ctx);
    this._enforceTwoLines();
  };

  /* The canonical layer is always two lines. If the effect's own layer renders one, the headline
   * snaps 2->1 at the start of every transition and 1->2 at the end. Effects keep the two halves
   * as sibling spans, so force those to block — but only where a parent holds EXACTLY one of each.
   * More than one means the effect tagged individual words with the same classes, and forcing
   * those would stack every word on its own line.
   */
  Player.prototype._enforceTwoLines = function () {
    var marked = this.layer.querySelectorAll('.fx-light, .fx-bold');
    var parents = [];
    for (var i = 0; i < marked.length; i++) {
      var par = marked[i].parentElement;
      if (par && parents.indexOf(par) === -1) parents.push(par);
    }
    for (var j = 0; j < parents.length; j++) {
      var kids = Array.prototype.slice.call(parents[j].children);
      var L = [], B = [];
      for (var k = 0; k < kids.length; k++) {
        if (kids[k].classList.contains('fx-light')) L.push(kids[k]);
        if (kids[k].classList.contains('fx-bold')) B.push(kids[k]);
      }
      if (L.length === 1 && B.length === 1) {
        L[0].style.display = 'block';
        B[0].style.display = 'block';
      }
    }
  };

  Player.prototype._render = function (t) {
    // Endpoints belong to the engine, not the effect: t<=0 is canonical A, t>=1 is canonical B.
    // The effect only ever draws the middle. This is what lets any effect follow any other.
    if (t <= 0) {
      this._showCanon(this.ctx.from);
      if (this.onTick) this.onTick(0, this.i);
      return;
    }
    if (t >= 1) {
      this._showCanon(this.ctx.to);
      if (this.onTick) this.onTick(1, this.i);
      return;
    }
    this._showEffect();
    // A throw inside frame() used to leave the stage frozen in its setup state — every frame
    // identical, which looks like a boring effect rather than a crash. One missing easing name
    // silently killed an entire effect that way. Surface it loudly instead, once per effect.
    try {
      this.fx.frame(this.layer, t, this.ctx);
    } catch (err) {
      if (!this._blewUp) {
        this._blewUp = true;
        console.error('[SloganFX] effect "' + this.fx.id + '" threw in frame(): ' + err.message, err);
      }
    }
    if (this.onTick) this.onTick(t, this.i);
  };

  /* Deterministic: show the transition i -> i+1 frozen at progress t. */
  Player.prototype.seek = function (i, t) {
    this.pause();
    this.i = ((i % this.slogans.length) + this.slogans.length) % this.slogans.length;
    this._build();
    this._render(clamp(t, 0, 1));
  };

  Player.prototype.goto = function (i) {
    this.pause();
    this.i = ((i % this.slogans.length) + this.slogans.length) % this.slogans.length;
    this._build();
    this._render(0);
  };

  Player.prototype.play = function () {
    if (this.playing || this.reduced) return;
    this.playing = true;
    this.phase = 'rest';
    this.t0 = performance.now();
    var self = this;
    var step = function (now) {
      if (!self.playing) return;
      var dt = now - self.t0;
      if (self.phase === 'rest') {
        if (dt >= self.dwell) { self.phase = 'run'; self.t0 = now; }
        else self._render(0);
      } else {
        var t = clamp(dt / self.duration, 0, 1);
        self._render(t);
        if (t >= 1) {
          self.i = (self.i + 1) % self.slogans.length;
          self._build();
          self.phase = 'rest';
          self.t0 = now;
        }
      }
      self.raf = requestAnimationFrame(step);
    };
    this.raf = requestAnimationFrame(step);
  };

  Player.prototype.pause = function () {
    this.playing = false;
    if (this.raf) cancelAnimationFrame(this.raf);
    this.raf = null;
  };

  Player.prototype.destroy = function () { this.pause(); this.stage.innerHTML = ''; };

  /* ---------------------------------------------------------------- API */

  var SloganFX = {
    ease: ease, lerp: lerp, clamp: clamp, span: span,
    chars: chars, words: words, charWords: charWords, alignWords: alignWords, el: el,
    register: function (fx) {
      if (!fx || !fx.id || typeof fx.frame !== 'function') {
        console.error('SloganFX.register: effect needs at least {id, frame}', fx);
        return;
      }
      if (REGISTRY.some(function (e) { return e.id === fx.id; })) {
        console.warn('SloganFX: duplicate id ignored:', fx.id);
        return;
      }
      REGISTRY.push(fx);
    },
    all: function () { return REGISTRY.slice(); },
    get: function (id) { return REGISTRY.filter(function (e) { return e.id === id; })[0]; },
    mount: function (stage, effect, slogans, opts) { return new Player(stage, effect, slogans, opts); },
    /* effect that WILL run for transition i of a given pool — same rule the Player uses */
    effectFor: function (pool, i, pick) {
      var n = pool.length;
      if (n <= 1) return pool[0];
      if (pick === 'shuffle') {
        var h = Math.sin(i * 78.233 + 12.9898) * 43758.5453;
        return pool[Math.floor((h - Math.floor(h)) * n) % n];
      }
      return pool[((i % n) + n) % n];
    },
  };

  global.SloganFX = SloganFX;
})(window);
