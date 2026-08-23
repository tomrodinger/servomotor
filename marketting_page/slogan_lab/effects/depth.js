/* Family: 3D — real perspective, real solids.
 *
 * Every effect here is a pure function of progress: the geometry of a cube, a hinge, a folded
 * sheet is computed from t and written straight into transforms. Nothing animates itself, so any
 * frame can be seeked, screenshotted or exported.
 *
 * House rules used throughout:
 *   - one perspective camera per stage (`scene`), everything under it in one preserve-3d chain
 *   - faces that must read as *solid* get the tile background painted on them plus a shading
 *     overlay driven by the face normal — depth without lighting looks like flat text on a skew
 *   - both slogans are measured and forced to the same block height, so hinges and cube edges sit
 *     exactly on the shared centre line no matter how many lines each slogan wraps to
 */
(function () {
  'use strict';

  var FX = window.SloganFX, E = FX.ease, span = FX.span, clamp = FX.clamp;
  var D2R = Math.PI / 180;

  var PAPER = { bg: '#F5F3EE', fg: '#1B1A17' };
  var NIGHT = { bg: '#0C0E12', fg: '#EDF0F4' };
  var WHITE = { bg: '#FFFFFF', fg: '#1D1D1F' };

  /* ---------------------------------------------------------------- scaffolding */

  /* A perspective camera filling the stage, with a centred 3D box inside it. */
  function scene(stage, persp, po) {
    stage.innerHTML = '';
    var cam = FX.el('div', 'fx-cam', {
      position: 'relative', width: '100%', height: '100%',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
      perspective: (persp || 1100) + 'px',
      perspectiveOrigin: po || '50% 50%',
      /* Any full-width solid tilted TOWARD the lens projects wider than the layer that holds it:
         the flip's paper leaf ran right out to the card border mid-fall, and the cube's face by
         about nine pixels a side. Clip the camera to the layer, which is the frame the whole family
         is supposed to live inside. Text never reaches the edge, so only the solids are trimmed —
         and a sheet of paper cut off by the frame is what a sheet of paper does. */
      overflow: 'hidden',
    });
    var box = FX.el('div', 'fx-box', {
      position: 'relative', width: '100%',
      transformStyle: 'preserve-3d', willChange: 'transform',
    });
    cam.appendChild(box);
    stage.appendChild(cam);
    return { cam: cam, box: box };
  }

  /* Fill a container with one slogan: light line over bold line.
   *
   * The two halves MUST be blocks, not spans separated by a <br>. The engine forces any parent
   * holding exactly one .fx-light and one .fx-bold to display them as blocks — so a <br> as well
   * left an empty third line box between them. That inflated every measurement here: `equalize`
   * read a two-line height while the text actually rendered three lines tall, so the ink overflowed
   * its own face. On the cube it merely pushed the lines apart; on the flip, the unfold and the
   * hinge (a -webkit-mask clips to the element box) the top of the light line and the bottom of the
   * bold line were sliced clean off. Blocks, no <br>: the layer now measures what it draws and
   * matches the canonical two-line rest state exactly.
   */
  function fill(inner, s, split) {
    var a = FX.el('span', 'fx-light'), b = FX.el('span', 'fx-bold');
    a.textContent = s.light;
    b.textContent = s.bold;
    if (split === false) {
      inner.appendChild(a);
      inner.appendChild(document.createTextNode(' '));
    } else {
      a.style.display = 'block';
      b.style.display = 'block';
      inner.appendChild(a);
    }
    inner.appendChild(b);
    return inner;
  }

  /* One slogan as a full-width, vertically centred layer of the 3D box. */
  function layer(box, s, opt) {
    opt = opt || {};
    var face = FX.el('div', 'fx-line', {
      position: 'absolute', left: '0', right: '0', top: '0',
      display: 'flex', alignItems: 'center', justifyContent: 'center',
      backfaceVisibility: 'hidden',
      willChange: 'transform,opacity,filter',
    });
    if (opt.bg) face.style.background = opt.bg;
    if (opt.p3d) face.style.transformStyle = 'preserve-3d';
    var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
    if (opt.p3d) inner.style.transformStyle = 'preserve-3d';
    fill(inner, s, opt.split);
    face.appendChild(inner);
    box.appendChild(face);
    return { face: face, inner: inner };
  }

  /* Same, but split into per-word inline-blocks that can each be posed in 3D. */
  function wordLayer(box, s, opt) {
    var L = layer(box, { light: '', bold: '' }, opt);
    L.inner.innerHTML = '';
    var w1 = FX.words(s.light, 'fx-light'), w2 = FX.words(s.bold, 'fx-bold');
    L.inner.appendChild(w1.frag);
    L.inner.appendChild(FX.el('br'));
    L.inner.appendChild(w2.frag);
    L.nodes = w1.nodes.concat(w2.nodes);
    L.nodes.forEach(function (n) {
      n.style.backfaceVisibility = 'hidden';
      n.style.willChange = 'transform';
    });
    return L;
  }

  /* Measure both slogans, give the box and every layer one shared height. Hinges, cube edges and
     fold lines all key off this, so a 1-line and a 3-line slogan still meet on the same axis. */
  function equalize(box, faces) {
    var h = 0, i;
    for (i = 0; i < faces.length; i++) h = Math.max(h, faces[i].offsetHeight);
    h = Math.max(Math.round(h), 40);              // hidden tiles measure 0 — keep the maths sane
    box.style.height = h + 'px';
    for (i = 0; i < faces.length; i++) { faces[i].style.height = h + 'px'; faces[i].style.top = '0'; }
    return h;
  }

  /* Width of the actual ink inside a container, not of the full-width block that holds it. Both
     halves are block-level so offsetWidth is always the layer width; a range over the contents
     reports the union of the line boxes, which is what "how wide is this slogan" means. */
  function inkWidth(node) {
    var r = document.createRange();
    r.selectNodeContents(node);
    var w = r.getBoundingClientRect().width;
    r.detach && r.detach();
    return w || node.offsetWidth || 0;
  }

  /* A shading plane over a face: cheap, convincing lighting for a solid. */
  function shade(parent, color) {
    var s = FX.el('div', null, {
      position: 'absolute', left: '0', top: '0', right: '0', bottom: '0',
      background: color || '#000', opacity: '0', pointerEvents: 'none',
    });
    parent.appendChild(s);
    return s;
  }

  /* Lambert-ish key light: surface normal · light, normalised so a face square to the lamp is
     unshaded and a face turned away sits on the ambient floor. */
  function lit(dot) { return .30 + .70 * clamp(dot / .83, 0, 1); }

  /* ============================================================ 1. Cube Roll ==== */
  /* Old and new are two faces of one solid. The cube makes a quarter turn about its horizontal
     axis, hits the stop and rocks once. Both faces carry the page colour so the corner between
     them reads as a real edge. */
  FX.register({
    id: 'depth-cube',
    name: 'Cube Roll',
    family: '3D',
    blurb: 'Old and new are two faces of one solid; it rolls a quarter turn and rocks to a stop.',
    duration: 1150,
    dwell: 4200,
    setup: function (stage, ctx) {
      var sc = scene(stage, 1500);
      var A = layer(sc.box, ctx.from, { bg: WHITE.bg });
      var B = layer(sc.box, ctx.to, { bg: WHITE.bg });
      var H = equalize(sc.box, [A.face, B.face]);
      A.face.style.transform = 'translateZ(' + (H / 2) + 'px)';
      B.face.style.transform = 'rotateX(90deg) translateZ(' + (H / 2) + 'px)';
      /* Whichever face is toward the camera sits H/2 IN FRONT of the box origin, so with the box
         at z=0 the resting slogan was magnified by 1500/(1500-H/2) — measurably 4% wider than the
         canonical text at both t=0 and t=1, i.e. a visible pop on entry and exit. Park the solid
         half a face deeper so the front face lands exactly on the camera's zero plane. */
      stage._s = { box: sc.box, A: A.face, B: B.face, z: H / 2,
                   shA: shade(A.face), shB: shade(B.face) };
    },
    frame: function (stage, t) {
      var s = stage._s;
      var k = span(t, .62, 1);
      /* quarter turn on a sine ease — a heavy object starting and stopping — then one damped
         rock as it lands on its stop */
      var m = span(t, .03, .92);
      var ang = 90 * (.5 - .5 * Math.cos(Math.PI * m))
              + Math.sin(k * Math.PI * 2) * Math.pow(1 - k, 1.8) * 5.5;
      var r = ang * D2R;
      var dip = Math.sin(Math.PI * t);              // the whole solid pulls back as it turns
      s.box.style.transform = 'translateZ(' + (-s.z - 80 * dip) + 'px) rotateX(' + (-ang) + 'deg)';
      s.A.style.opacity = String(clamp(1 - span(t, .93, 1), 0, 1));
      s.B.style.opacity = '1';
      /* one directional light from above and in front, so the two faces never share a tone:
         the outgoing face tips down into shadow exactly as the incoming face turns into the key */
      s.shA.style.opacity = String(1 - lit(-.55 * Math.sin(r) + .83 * Math.cos(r)));
      s.shB.style.opacity = String(1 - lit(.55 * Math.cos(r) + .83 * Math.sin(r)));
    },
    rest: function (stage) {
      var s = stage._s;
      s.box.style.transform = 'translateZ(' + (-s.z) + 'px) rotateX(-90deg)';
      s.A.style.opacity = '0';
      s.B.style.opacity = '1';
      s.shA.style.opacity = '.70';
      s.shB.style.opacity = '0';
    },
  });

  /* ============================================================ 2. Page Flip ==== */
  /* Split-flap board. The upper leaf is hinged on the centre line: its front carries the top half
     of the old slogan, its back the bottom half of the new one. Behind it sit the new top half and
     the old bottom half, so the mid-transition frame is the classic spliced hybrid. */
  FX.register({
    id: 'depth-flip',
    name: 'Page Flip',
    family: '3D',
    blurb: 'A leaf hinged on the centre line falls through half a turn, split-flap style.',
    duration: 1250,
    dwell: 4300,
    theme: { bg: PAPER.bg, fg: PAPER.fg },
    setup: function (stage, ctx) {
      var sc = scene(stage, 1150);
      /* static backing: new top half above the hinge, old bottom half below it */
      var topNew = layer(sc.box, ctx.to, { bg: PAPER.bg });
      var botOld = layer(sc.box, ctx.from, { bg: PAPER.bg });
      var H = equalize(sc.box, [topNew.face, botOld.face]);
      var half = Math.round(H / 2);
      topNew.face.style.clipPath = 'inset(0 0 ' + (H - half) + 'px 0)';
      botOld.face.style.clipPath = 'inset(' + half + 'px 0 0 0)';
      topNew.face.style.transform = 'translateZ(-2px)';
      botOld.face.style.transform = 'translateZ(-2px)';
      var castShadow = shade(botOld.face);
      castShadow.style.top = half + 'px';
      castShadow.style.background = 'linear-gradient(180deg, rgba(0,0,0,.5), rgba(0,0,0,0) 62%)';

      /* the leaf: half the block tall, hinged along its bottom edge */
      var leaf = FX.el('div', null, {
        position: 'absolute', left: '0', right: '0', top: '0', height: half + 'px',
        transformOrigin: '50% 100%', transformStyle: 'preserve-3d', willChange: 'transform',
      });
      function side(s, offset, flipped) {
        var f = FX.el('div', null, {
          position: 'absolute', left: '0', top: '0', right: '0', bottom: '0',
          backfaceVisibility: 'hidden',
          transform: flipped ? 'rotateX(180deg)' : 'none',
        });
        var clip = FX.el('div', null, {
          position: 'absolute', left: '0', top: '0', right: '0', bottom: '0',
          clipPath: 'inset(0)', background: PAPER.bg,
        });
        var line = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0', top: offset + 'px', height: H + 'px',
          display: 'flex', alignItems: 'center', justifyContent: 'center',
        });
        var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
        fill(inner, s);
        line.appendChild(inner);
        clip.appendChild(line);
        f.appendChild(clip);
        leaf.appendChild(f);
        return { face: f, shade: shade(clip) };
      }
      var front = side(ctx.from, 0, false);         // old, top half
      var back = side(ctx.to, -half, true);         // new, bottom half (mirrored twice = upright)
      sc.box.appendChild(leaf);
      stage._s = { leaf: leaf, front: front, back: back, cast: castShadow };
    },
    frame: function (stage, t) {
      var s = stage._s;
      /* gravity: the leaf accelerates all the way down, hits its stop and rebounds once.
         It falls TOWARD the viewer, so it is always in front of the static halves behind it. */
      var rb = span(t, .86, 1);
      var th = 180 * E.inQuad(span(t, 0, .86)) - Math.sin(rb * Math.PI * 2) * (1 - rb) * 8;
      var c = Math.cos(th * D2R);
      s.leaf.style.transform = 'rotateX(' + (-th) + 'deg)';
      s.front.shade.style.opacity = String(clamp(.58 * (1 - c), 0, .88));
      s.back.shade.style.opacity = String(clamp(.58 * (1 + c), 0, .88));
      s.cast.style.opacity = String(clamp((th - 55) / 115, 0, 1) * .34 * (1 - span(t, .90, 1)));
    },
    rest: function (stage) {
      var s = stage._s;
      s.leaf.style.transform = 'rotateX(180deg)';
      s.front.shade.style.opacity = '.9';
      s.back.shade.style.opacity = '0';
      s.cast.style.opacity = '0';
    },
  });

  /* A gradient that dims the far edge of a door by `k` (0 = untouched, 1 = fully receded). At k=0
     it is a flat opaque black, i.e. exactly no mask at all — which is what the endpoints need. */
  function edgeMask(deg, k) {
    k = clamp(k, 0, 1);
    if (k < .001) return 'none';
    var start = Math.round(100 - 60 * k);
    return 'linear-gradient(' + deg + 'deg,#000 0%,#000 ' + start + '%,rgba(0,0,0,' +
      (1 - .66 * k).toFixed(3) + ') 100%)';
  }

  /* ============================================================ 3. Door Hinge === */
  /* Two doors on opposite jambs. The old one swings shut into the depth of the frame, the new one
     swings out of the depth toward you and settles with a hair of overshoot. A gradient mask pulls
     the receding edge into the page so the depth reads even in a still. */
  FX.register({
    id: 'depth-hinge',
    name: 'Door Hinge',
    family: '3D',
    blurb: 'The old line swings shut on the left jamb; the new one swings open from the right.',
    duration: 1200,
    dwell: 4400,
    setup: function (stage, ctx) {
      var sc = scene(stage, 900);
      var A = layer(sc.box, ctx.from);
      var B = layer(sc.box, ctx.to);
      equalize(sc.box, [A.face, B.face]);
      A.face.style.transformOrigin = '0% 50%';
      B.face.style.transformOrigin = '100% 50%';
      stage._s = { cam: sc.cam, A: A.face, B: B.face };
    },
    frame: function (stage, t) {
      var s = stage._s;
      var o = E.inOutCubic(span(t, 0, .58));                 // old: accelerates shut
      var i = E.outQuart(span(t, .26, 1));                   // new: decelerates open
      var settle = Math.sin(span(t, .70, 1) * Math.PI) * (1 - span(t, .70, 1)) * 7;

      var oa = 80 * o;
      s.A.style.transform = 'rotateY(' + oa + 'deg) translateZ(' + (-14 * o) + 'px)';
      /* Three things used to gang up on the middle of this transition and leave it near-empty:
         a long opacity fade, a mask that erased three quarters of the panel, and an incoming door
         that took until t=.62 to reach full strength. Now the doors hold their ink — the depth is
         carried by the turn and by a mask that DIMS the receding edge rather than deleting it —
         and the handover is a short, hard crossfade in the middle instead of a long dip. */
      s.A.style.opacity = String(clamp(1 - span(t, .38, .60), 0, 1));
      /* The far edge of a turning door sinks into the page, but never to nothing. The gradient has
         to be an IDENTITY when the door is square to the camera: it used to fade the right-hand 40%
         to 34% ink at every t including zero, so "…surrounds it." sat visibly grey at the top of
         every transition and snapped back to black when the engine handed over to the canon. Both
         the depth of the fade and how far it reaches now scale with the turn, so at t->0 (o=0) and
         t->1 (i=1) the mask is flat black. */
      s.A.style.webkitMaskImage = s.A.style.maskImage = edgeMask(90, o);

      var ia = -78 * (1 - i) + settle;
      s.B.style.transform = 'rotateY(' + ia + 'deg) translateZ(' + (-14 * (1 - i)) + 'px)';
      s.B.style.opacity = String(clamp(span(t, .34, .52), 0, 1));
      s.B.style.webkitMaskImage = s.B.style.maskImage = edgeMask(270, 1 - i);

      /* the camera drifts across the doorway, so the two hinges never look symmetrical */
      s.cam.style.perspectiveOrigin = (50 + 16 * (E.inOutCubic(t) - .5) * 2) + '% 50%';
    },
    rest: function (stage) {
      var s = stage._s;
      s.A.style.opacity = '0';
      s.B.style.opacity = '1';
      s.B.style.transform = 'none';
      s.B.style.webkitMaskImage = s.B.style.maskImage = 'none';
      s.cam.style.perspectiveOrigin = '66% 50%';
    },
  });

  /* ============================================================ 4. Depth Dolly == */
  /* A move down a tunnel: the old line rushes the lens and blows past it, the new one comes up out
     of the dark, word by word, with the far words trailing the near ones. */
  FX.register({
    id: 'depth-dolly',
    name: 'Depth Dolly',
    family: '3D',
    blurb: 'The old line rushes past the lens; the new one climbs out of depth, word by word.',
    duration: 1400,
    dwell: 4200,
    theme: { bg: NIGHT.bg, fg: NIGHT.fg },
    setup: function (stage, ctx) {
      var PERSP = 800;
      var sc = scene(stage, PERSP);
      var A = layer(sc.box, ctx.from, { p3d: true });
      var B = wordLayer(sc.box, ctx.to, { p3d: true });
      equalize(sc.box, [A.face, B.face]);
      /* How far the outgoing line may rush the lens depends on how wide it already is. A flat
         640px of travel magnified it 5x, which is fine for "Four systems." and ran the longest
         slogan clean off both edges of the tile — a sharp, half-legible line sliced by the frame.
         Measure the ink and let it grow only until it fills the frame, so the rush is as big as
         each slogan can afford and never a pixel bigger. Blur and the fade carry the speed. */
      var room = Math.max(1, (ctx.W || sc.cam.offsetWidth) - 8);
      var fit = clamp(room / Math.max(24, inkWidth(A.inner)), 1.06, 2.4);
      stage._s = { A: A.face, B: B.face, w: B.nodes, persp: PERSP, fit: fit };
    },
    frame: function (stage, t) {
      var s = stage._s;

      /* outgoing: straight at the camera, accelerating, smeared by its own speed */
      var o = E.inQuad(span(t, 0, .55));
      var grow = 1 + (s.fit - 1) * o;
      var oz = s.persp * (1 - 1 / grow);
      s.A.style.transform = 'translateZ(' + oz + 'px) rotateX(' + (-5 * o) + 'deg)';
      s.A.style.opacity = String(clamp(1 - Math.pow(span(t, .06, .50), 1.3), 0, 1));
      s.A.style.filter = 'blur(' + (22 * o * o) + 'px)';

      /* incoming: out of the dark, decelerating, with a residual tilt that levels off */
      var p = E.outCubic(span(t, .12, 1));
      s.B.style.transform = 'translateZ(' + (-1500 * (1 - p)) + 'px) rotateX(' + (9 * (1 - p)) + 'deg)';
      s.B.style.opacity = String(clamp(span(t, .12, .46), 0, 1));

      /* per-word parallax: each word sits a little deeper and arrives a little later */
      var n = s.w.length;
      for (var k = 0; k < n; k++) {
        var u = n > 1 ? k / (n - 1) : 0;
        var q = E.outCubic(span(t, .12 + u * .22, .74 + u * .26));
        var z = -260 * (1 - q);
        s.w[k].style.transform = 'translateZ(' + z + 'px)';
        s.w[k].style.filter = 'blur(' + (3.4 * (1 - q)) + 'px)';
        s.w[k].style.opacity = String(clamp(.25 + .75 * q, 0, 1));
      }
    },
    rest: function (stage) {
      var s = stage._s;
      s.A.style.opacity = '0';
      s.B.style.opacity = '1';
      s.B.style.transform = 'none';
      s.w.forEach(function (n) { n.style.transform = 'none'; n.style.filter = 'none'; n.style.opacity = '1'; });
    },
  });

  /* Give every word in a wordLayer a wave phase derived from WHERE IT SITS, not from its index.
   *
   * Index-based staggering desynchronises the two slogans whenever they have different word counts:
   * the outgoing sentence's fourth word and the incoming sentence's fourth word then hit their
   * edge-on 90-degree instant at different times, and since both are backface-hidden there, the
   * screen briefly shows NEITHER — a word simply missing from an otherwise settled line (i=0 lost
   * "system." at t=.5 and "motor." at t=.7). Phase by row and horizontal centre instead and the
   * turn becomes a real sweep across the block: whatever old word and whatever new word share a
   * patch of the line turn together, so the pair hands over on the spot and no hole opens.
   */
  function phaseByPlace(L) {
    var w = L.face.offsetWidth || L.inner.offsetWidth || 1;
    L.nodes.forEach(function (n) {
      var row = n.classList.contains('fx-bold') ? 1 : 0;
      var cx = n.offsetLeft + n.offsetWidth / 2;
      n._u = (row + clamp(cx / w, 0, 1)) / 2;
    });
  }

  /* ============================================================ 5. Word Helix === */
  /* Every word is a card on a vertical spindle set behind the line. Cards turn in sequence, so the
     turn travels along the sentence as a wave; the old face leaves as the new face arrives on the
     back of the same rotation. Backfaces are hidden, so exactly one of the pair is ever readable. */
  FX.register({
    id: 'depth-helix',
    name: 'Word Helix',
    family: '3D',
    blurb: 'Each word turns on its own spindle; the turn travels along the line as a wave.',
    duration: 1450,
    dwell: 4600,
    setup: function (stage, ctx) {
      var sc = scene(stage, 1000);
      var A = wordLayer(sc.box, ctx.from, { p3d: true });
      var B = wordLayer(sc.box, ctx.to, { p3d: true });
      var H = equalize(sc.box, [A.face, B.face]);
      phaseByPlace(A);
      phaseByPlace(B);
      stage._s = { A: A.nodes, B: B.nodes, r: clamp(H * .45, 26, 96), rise: H * .22 };
    },
    frame: function (stage, t) {
      var s = stage._s, r = s.r;

      function pose(nodes, incoming) {
        var n = nodes.length;
        for (var k = 0; k < n; k++) {
          var u = nodes[k]._u;
          var lead = u * .40;
          var p = E.inOutCubic(span(t, lead, lead + .60));
          /* both lines turn the same way: old 0 -> -180, new +180 -> 0 */
          var th = incoming ? 180 * (1 - p) : -180 * p;
          var y = incoming ? s.rise * (1 - p) : -s.rise * p;
          nodes[k].style.transform =
            'translate3d(0,' + y + 'px,' + (-r) + 'px) rotateY(' + th + 'deg) translateZ(' + r + 'px)';
          nodes[k].style.opacity = incoming
            ? String(clamp(span(p, .04, .28), 0, 1))
            : String(clamp(1 - span(p, .72, .96), 0, 1));
        }
      }
      pose(s.A, false);
      pose(s.B, true);
    },
    rest: function (stage) {
      var s = stage._s;
      s.A.forEach(function (n) { n.style.opacity = '0'; });
      s.B.forEach(function (n) { n.style.transform = 'none'; n.style.opacity = '1'; });
    },
  });

  /* ============================================================ 6. Paper Unfold = */
  /* The slogans are sheets of paper scored into six horizontal panels. The old sheet concertinas
     shut and drops back; the new one opens out of its fold, panel after panel, with the valleys
     shaded. Panel positions are integrated along the zigzag each frame, so the pack stays centred
     however far it is folded. */
  var PANELS = 6;

  function foldPose(panels, h, phi, H, max) {
    /* Walk the concertina: each panel advances h*cos(phi) down the page and h*sin(phi) in depth,
       alternating direction. Returns per-panel centre offsets, then re-centres the whole pack. */
    var y = 0, z = 0, ys = [], zs = [], k, c, d;
    for (k = 0; k < panels.length; k++) {
      var a = phi[k];
      d = (k % 2 === 0) ? 1 : -1;
      c = h * Math.cos(a);
      var q = h * Math.sin(a) * d;
      ys.push(y + c / 2); zs.push(z + q / 2);
      y += c; z += q;
    }
    var zmin = Math.min.apply(null, zs), zmax = Math.max.apply(null, zs), zc = (zmin + zmax) / 2;
    var pad = (H - y) / 2;
    for (k = 0; k < panels.length; k++) {
      d = (k % 2 === 0) ? 1 : -1;
      var natural = panels[k]._top + panels[k]._h / 2;
      panels[k].style.transform =
        'translate3d(0,' + (ys[k] + pad - natural) + 'px,' + (zs[k] - zc) + 'px) ' +
        'rotateX(' + (d * phi[k] / D2R) + 'deg)';
      /* Lighting that also does the hiding. A closed pack is only ~30% of the block tall, so any
         ink still showing on it is a squashed, unreadable smear — and with two sheets crossfading
         through that beat it was two smears on top of each other. Ramp the shading with the CUBE of
         how far the pack is folded: barely there while the sheet is still readable, near-total at
         full fold, where the alternating white sheen and dark valley read as the banded edge of
         folded paper. Nothing is legible there, so nothing is double-exposed. */
      var ob = Math.pow(clamp(phi[k] / max, 0, 1), 3);
      panels[k]._shade.style.opacity = String((d > 0 ? .10 : .86) * ob);
      panels[k]._sheen.style.opacity = String((d > 0 ? .94 : 0) * ob);
    }
  }

  FX.register({
    id: 'depth-unfold',
    name: 'Paper Unfold',
    family: '3D',
    blurb: 'The old line concertinas shut; the new one opens out of its folds, panel by panel.',
    duration: 1500,
    dwell: 4600,
    theme: { bg: PAPER.bg, fg: PAPER.fg },
    setup: function (stage, ctx) {
      var sc = scene(stage, 1250);
      /* measure both slogans with throwaway layers, then rebuild each as panels */
      var mA = layer(sc.box, ctx.from), mB = layer(sc.box, ctx.to);
      var H = equalize(sc.box, [mA.face, mB.face]);
      sc.box.removeChild(mA.face); sc.box.removeChild(mB.face);

      var h = Math.max(8, Math.round(H / PANELS));
      H = h * PANELS;                       // integer panels: no hairline seams when flat
      sc.box.style.height = H + 'px';

      function sheet(s) {
        var wrap = FX.el('div', null, {
          position: 'absolute', left: '0', right: '0', top: '0', height: H + 'px',
          transformStyle: 'preserve-3d', willChange: 'transform,opacity',
        });
        var panels = [];
        for (var k = 0; k < PANELS; k++) {
          var p = FX.el('div', null, {
            position: 'absolute', left: '0', right: '0', top: (k * h) + 'px', height: h + 'px',
            clipPath: 'inset(0)', background: PAPER.bg, transformOrigin: '50% 50%',
            willChange: 'transform',
          });
          var line = FX.el('div', null, {
            position: 'absolute', left: '0', right: '0', top: (-k * h) + 'px', height: H + 'px',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
          });
          var inner = FX.el('div', null, { width: '100%', textAlign: 'center' });
          fill(inner, s);
          line.appendChild(inner);
          p.appendChild(line);
          p._shade = shade(p);
          p._sheen = shade(p, PAPER.bg);
          p._top = k * h; p._h = h;
          wrap.appendChild(p);
          panels.push(p);
        }
        sc.box.appendChild(wrap);
        return { wrap: wrap, panels: panels };
      }
      var A = sheet(ctx.from), B = sheet(ctx.to);
      stage._s = { A: A, B: B, h: h, H: H, phiA: [], phiB: [] };
    },
    frame: function (stage, t) {
      var s = stage._s, k;
      /* 88 degrees squashed the whole pack to a 3%-tall hairline, and the two sheets used to
         cross-fade slowly THROUGH that hairline — so the middle of the transition was a grey line
         carrying two different slogans printed on top of each other. Three changes: the pack never
         folds past 62 degrees, so a closed pack is a visible banded slab of paper roughly half of
         the block tall rather than a line; foldPose blacks the valleys out and whitens the peaks as
         the fold deepens, so the closed pack carries no legible ink at all; and the handover is a
         short crossfade that happens WHILE both sheets are folded. Either side of that beat exactly
         one sheet is open and readable, and nothing is ever double-exposed. */
      var MAX = 62 * D2R;

      /* closing: a ripple runs down the old sheet, which then sinks away */
      for (k = 0; k < PANELS; k++) {
        var lead = (k / PANELS) * .08;
        s.phiA[k] = MAX * E.inQuad(span(t, lead, lead + .24));
      }
      foldPose(s.A.panels, s.h, s.phiA, s.H, MAX);
      var o = E.inOutCubic(span(t, .04, .36));
      s.A.wrap.style.transform = 'translateZ(' + (-190 * o) + 'px) rotateX(' + (-9 * o) + 'deg)';
      s.A.wrap.style.opacity = String(clamp(1 - span(t, .27, .33), 0, 1));

      /* opening: the same ripple in reverse, arriving from behind */
      for (k = 0; k < PANELS; k++) {
        var lead2 = .30 + (k / PANELS) * .12;
        s.phiB[k] = MAX * (1 - E.outCubic(span(t, lead2, lead2 + .44)));
      }
      foldPose(s.B.panels, s.h, s.phiB, s.H, MAX);
      var i = E.outCubic(span(t, .28, 1));
      s.B.wrap.style.transform = 'translateZ(' + (-180 * (1 - i)) + 'px) rotateX(' + (8 * (1 - i)) + 'deg)';
      s.B.wrap.style.opacity = String(clamp(span(t, .27, .33), 0, 1));
    },
    rest: function (stage) {
      var s = stage._s;
      s.A.wrap.style.opacity = '0';
      s.B.wrap.style.opacity = '1';
      s.B.wrap.style.transform = 'none';
      s.B.panels.forEach(function (p) {
        p.style.transform = 'none'; p._shade.style.opacity = '0'; p._sheen.style.opacity = '0';
      });
    },
  });
})();
