# Slogan transition bug hunt — overnight 2026-08-23/24

Tom's brief: the transitions have bugs. Two named symptoms — the background flashes white when it
should be black, and a transition sometimes fails to start or end on the canonical text position and
size. Fix as many as possible before 06:00; he rates all 96 in the morning.

## How the bugs are found

`audit_frames.py` drives `_frameaudit.html`, whose stage is a byte-for-byte copy of the rater's.
The engine already makes every effect a pure function of progress `t`, so a frame is *asked for*
rather than caught: `_render(t)` for a list of t values, one element screenshot each.

Per effect, per slogan pair, it measures

| metric | what it catches |
|---|---|
| `startSnapBox` / `endSnapBox` | ink bounding box at t=0.008 / t=0.992 vs the canonical frame, in px — Tom's "doesn't start/end at the canonical position and size" |
| `startSnapCov` / `endSnapCov` | fraction of the stage that visibly differs from the canonical frame |
| `flash` | fraction of the stage gone near-white — Tom's white flash |
| `blankFrames` | headline absent mid-transition |
| `errors` | a throw inside `frame()`, which freezes the animation and reads as a dull effect |

The ink box is measured from PIXELS, not the DOM. Asking the DOM is the obvious approach and it is
wrong: effects build two full-stage layers, so the union of their client rects is the whole stage
regardless of what is visible.

Calibration: known-good effects (`wipe-bar`, `light-specular`) score 0.0000 on every endpoint
metric, so the thresholds in `audit_report.py` sit just above anti-aliasing noise.

## Baseline, before any fixes

96 effects, 2 slogan pairs each. **34 clean, 62 with defects.**

    START-SNAP=83  END-SNAP=73  START-DIFF=71  END-DIFF=31
    FLASH=20       START-VANISH=18  END-VANISH=16

## Round 1 — two root causes, 30-odd effects

### 1. The white flash: effects painting a paper sheet over a dark stage

Seven effects painted an opaque light sheet for the *entire* transition, so the stage went
black → white → black every cycle: `glitch-tear`, `glitch-mosh`, `glitch-blocks`,
`liquid-goo-merge`, `print-riso`, `print-tear`, `dissolve-chromatic`. Three more painted white
faces: `depth-cube`, `depth-flip`, `depth-unfold`.

The three glitch effects were the worst of it — they painted a near-white sheet but never set a
text colour, so the headline inherited the stage's light ink and was nearly invisible on its own
paper.

**Fix — the sheet is not white, the sheet is whatever the stage already is.** `engine.js` gained
`stageColors()`, which walks up from the stage to the first non-transparent background, and hands
every effect `ctx.sc` = `{ dark, paper, ink, raised, sunken, edge, shade() }`. Effects now take
their sheet from `ctx.sc.paper`. This also makes the rater's "On white" toggle honest: the same
effect re-mounts as dark ink on white paper instead of needing a second hard-coded palette.

`print-riso` and `dissolve-chromatic` needed more than a colour swap. Both are *subtractive* print
metaphors — spot inks multiplying down onto white paper — and multiply has no meaning on a dark
ground. They now flip to `screen` on a dark stage, and `dissolve-chromatic` also flips the blend
*identity* (multiply's is white, screen's is black), which its two-beat inking depends on. The
misregistration, the re-inking and the walk back into alignment are unchanged; only the direction
of the ink changes.

Result: flash 100% → 3-5%, which is the canonical frame's own coverage. All ten also went to
0-2px on both endpoints as a side effect.

### 2. The 45px jump: a `<br>` between two block halves

17 effects were off by *exactly* 45px at both ends. One cause. The engine forces a lone
`.fx-light`/`.fx-bold` pair to `display:block` for the canonical two-line rest state; a `<br>`
between two blocks then opens an anonymous **third, empty line box**, so the effect's layer sat a
line looser and higher than the canon and the headline jumped a whole line the instant the effect
took over.

Seven effect files already carried a comment warning about exactly this. Seven others still had the
bug — 16 sites across `house-extra.js`, `house-extra-2.js`, `print.js`, `kinetic.js`, `slide.js`,
`depth.js` and `_reference.js`.

Twelve were the simple shape (two spans and a `<br>`) and were fixed by codemod: both halves become
`display:block`, the `<br>` goes. Four were structural — `kinetic`, `slide` and `depth` tag
individual *words* `.fx-light`/`.fx-bold`, so the engine's two-line enforcement deliberately skips
them and the `<br>` was the only row separator. Each half now gets its own block row, the way the
canonical layer does it. Every caller passed `split: true`, so the one-line branch was dead code
and is gone.

### 3. Incidental

`print-tear` built its torn piece with a hand-rolled inner that had both the `<br>` bug and a
trailing space, in a file whose own helper carries the warning comment. Its torn fibre was a
72%-white streak, which is right on paper and a glare on a dark stage; it now uses `ctx.sc.shade()`.

## Round 1 result

**34 clean → 51 clean.** FLASH went from 20 occurrences to zero. START-VANISH 18 → 6.

## Round 2 — the 11-12px breath, and why the obvious fix was wrong

After round 1 the largest remaining group was ~22 effects, every one of them off by 11-12px at the
start and 7-8px at the end, and only on slogan pair 12. Same centre, same y, same height — the line
was simply **wider** while the effect owned the stage, so the headline breathed outward about 6px a
side on entry and back in on exit.

**A wrong turn, recorded because the reasoning looked sound.** Pair 12 is the long slogan, so the
first hypothesis was wrapping: `words()` and `charWords()` separate words with a
`white-space: pre` span, and `pre` cannot collapse at a line break, so a wrapped line would render
one extra space. Changing those to plain collapsible text nodes changed nothing (still 11-12px) and
regressed `type-backspace` at pair 0 from clean to 25px. Reverted. Measuring the DOM afterwards
showed why the premise was false: the canonical box is 797px wide, which is the natural width of
the longest row — **the line never wraps at all.**

**The real cause: kerning.** Almost every effect animates per word or per character, which means
each one gets its own inline-block. A glyph in its own box cannot kern against its neighbour, so a
split line renders wider than the identical text as a single run — a consistent 11-12px on a 797px
line, about a quarter of a pixel per glyph. The canonical layer is one plain run, and it *does*
kern.

There is no way to make a split line kern. So the fix runs the other way: stop the plain run
kerning too. `engine.js` now sets `font-kerning: none; font-variant-ligatures: none` on the stage,
which both layers inherit. Both renderings then agree to within a pixel, and the cost is a
sub-pixel looseness at 46px that nobody can see standing still — where the 12px jump was obvious.

This belongs on the stage, not in each effect: the canonical layer has to agree with whatever the
effect does, so both have to be told the same thing from one place.

Measured across the whole cluster afterwards — shatter, liquid, type, slide, glitch, print, hy-smoke
— **0-2px on every endpoint, 24/24 records under threshold**, and the two calibration effects that
were already perfect (`wipe-bar`, `light-specular`) stayed at 0.0.

## Round 3 — the per-effect geometry bugs, fanned out by file

Thirteen effects still had real geometry faults, from 25px up to 706px, plus three that drew
*nothing* at the endpoint frames. These are individual bugs with individual causes, so they were
fanned out one agent per effect FILE (files, not effects, so no two agents ever edited the same
file), each given the audit tooling, the canonical contract, and the two root causes already fixed
so it would not chase them again. Every group was then re-audited by a second agent that was told
not to trust the first one's numbers.

| file | fixed |
|---|---|
| `house-extra.js` | hx-riso, hx-rgb-split, hx-tear (all three re-implemented), hx-spotlight, hx-neon-flicker, hx-signal-lock |
| `house-extra-2.js` | hy-vernier (650px), hy-dimension-line (330px), hy-origami (re-implemented), hy-louvre |
| `mechanical.js` | mechanical-slide-rule (706px), mechanical-split-flap (425px), mechanical-typewriter (316px), mechanical-dot-matrix, mechanical-flip-cards |
| `light.js` / `print.js` / `kinetic.js` | light-shadow (241px), print-inkjet (302px), kinetic-pendulum (126px) |
| `_reference.js` / `type.js` | odometer (75px), type-backspace (25px) |

All five verifiers reported the claims accurate, `node --check` clean, and **no regressions** — each
re-audited every effect in its files, not only the ones that were touched.

The verification went past the endpoint numbers, which was the right instinct: an effect can satisfy
"looks like canonical at both ends" by simply doing nothing in between. One verifier wrote its own
mid-transition probe measuring colour saturation and stage coverage across t, and confirmed that
`hx-riso`'s spot ink and `hx-rgb-split`'s channels really do render mid-transition (saturation peaks
0.41 and 0.79, falling to 0.01 at the endpoints) rather than having been quietly suppressed.

**Two things a metric cannot judge, flagged for Tom's eye:**

- `hy-origami` is a genuine mechanism change, not a repair. It no longer flips 180° to a back face;
  it folds edge-on and back with the text swapped at the crease. It is contract-clean, but it looks
  different from what it was.
- `hx-spotlight` is contract-clean but its beam easing leaves roughly the first fifth and last fifth
  of the transition as still frames. A rater may read that as a slow start rather than a bug.

## Where round 3 landed

**96 of 96 effects clean** at pairs 0 and 12 — no endpoint snap, no flash, no vanish, no blank
frames, no thrown errors.

Residual deviation across all 192 records: ink box **median 1px, worst 3px**; visible coverage
difference median 0.9%, worst 3.4%. That is anti-aliasing scale, well under what an eye catches on
a 46px headline.

    baseline   34 clean / 62 defective
    round 1    51 clean / 45 defective   (paper palette, <br>)
    round 2    73 clean / 23 defective   (kerning)
    round 3    96 clean /  0 defective   (per-effect geometry)

## Round 4 — three angles the endpoint audit cannot see

Passing the endpoint check is necessary, not sufficient: an effect could satisfy it by doing
nothing in between, or by being different every time it runs. `audit_extra.py` asks three other
questions of all 96 effects.

**INVARIANCE — 0 failures.** This is Tom's actual requirement, stated back in the design phase: "a
given text will always appear the same no matter what". So render the canonical frame for the same
slogan under all 96 effects and demand the images are identical. All 96 agree to within
0.0008 mean absolute difference. Any effect that leaked a style onto the stage, or drew its own
idea of the rest state, would show here. None do.

**DETERMINISM — 0 failures.** Render the same t twice from a fresh mount and demand the same
pixels. Nothing uses `Math.random()` or the clock, so a screenshot, a GIF export and the live page
cannot disagree.

**CONTINUITY — 1 real fault, 1 false positive.** Walk t in 40 steps and flag any step far larger
than that effect's own typical step.

- `light-neon` alternates 0.057 / 0.000 steps in a square wave. That is a neon tube flickering; it
  is the effect, not a bug. Left alone.
- `liquid-goo-merge` had a genuine pop: one step of 0.036 at t=0.157 against a background of
  0.005, a 7x jump in the middle of an otherwise smooth melt. Cause: the filter was switched
  between `'none'` and `blur(...) contrast(...)` at a threshold. Toggling `filter` is not a no-op
  even at near-identity values — it creates and destroys a stacking context, so the box is
  re-rasterised and the type shifts a fraction of a pixel. The filter is now applied for the whole
  transition at identity values when the melt is weak. Step 0.061 -> 0.039, ratio 6.1x -> 3.5x,
  which is unremarkable (the 90th percentile across all effects is 9.2x).

The same `filter = cond ? blur(...) : 'none'` idiom appears about twenty times across the set, but
only this one crossed a visible threshold, so the others were left alone rather than churned.

**A false alarm worth recording.** Three effects — `ink-bleed`, `dissolve-develop`, `hy-lens-focus`
— looked motionless: their largest frame-to-frame step was under 0.004. Measuring total travel
instead showed they cover 82-123% of the distance between the two canonical frames, which is the
same as `wipe-bar` (123%). They are smooth, not dead. Low frame-to-frame delta is a virtue here,
not a symptom, and "fixing" them would have damaged three working effects.

## Round 5 — what only the other twelve slogan pairs could show

Pairs 0 and 12 were chosen for the audit because one is short and one is the longest. That was not
enough. Sweeping all fourteen found three more faults, all of them invisible on the two pairs that
had been tested all night.

**`wipe-columns`, up to 334px off at the start.** The clip helper `pctPad(v)` ramps its padding
compensation from +0.26em at v=0 to **−0.26em** at v=100, so "fully hidden" put the clip edge at
the glyph's bottom rather than at the bottom of the box, leaving the empty bottom padding
unclipped. That was harmless right up until you notice `frame()` also translates the inner span
down by up to 0.16em — which walks real glyph pixels into that supposedly empty strip. The
*incoming* line was showing as a sliver from the very first frame, so the measured ink box spanned
both slogans at once. Worst on the pairs whose two texts differ most in width, which is exactly why
pairs 0 and 12 hid it. The pad now ramps to zero, so hidden means hidden: **334px → 2px** across
all fourteen pairs.

**`glyph-scramble` and `print-letterpress`, 8-13px, on the same six pairs.** Both build their rows
from `alignWords`, which pads the word LIST as well as the letters — a four-word slogan morphing
into a seven-word one leaves groups that are entirely blank at one end. Both effects already
collapsed those groups to zero width. Neither collapsed the **space in front of them**, and on a
centred line two or three phantom spaces drag the whole row sideways. The inter-word gap is now a
cell like any other, with its own two sides, so it collapses with the group it introduces.

That fix alone made `glyph-scramble` worse at the far end, which turned out to be a second,
independent bug hiding behind the first: its stagger ran `lead` 0 → 0.55 with a 0.45 window, so the
first cell began churning at t=0 and the last settled at exactly t=1.0. The first effect frame was
already showing a random glyph instead of the outgoing character, and the last was still showing
one instead of the incoming character — and substitute glyphs are not the width of the letters they
stand in for. The stagger now starts at 0.04 and is done by 0.95.

    wipe-columns       334px -> 2px   (all 14 pairs)
    glyph-scramble      13px -> 1px   (all 14 pairs)
    print-letterpress   13px -> 2px   (all 14 pairs)

The lesson worth keeping: two slogan pairs is not a sample. Every one of these three needed a pair
where the two texts differ in *word count*, and neither pair 0 nor pair 12 does.

## Round 6 — a narrower stage

The rater is a fixed 1100px box and every slogan except the longest fits on one line there. The
hero is responsive, so the same effects have to survive a width where more of them wrap. Re-running
the audit with the stage at 680px found three faults that 1100px cannot show.

**`type-backspace`, 53px at the start and 66px at the end.** It hid untyped characters with
`display:none` and, because that reflows, locked each row to its measured width and left-aligned
the text inside. Correct at 1100px, where nothing wraps. At 680px the long slogan wraps and the
effect left-aligned each wrapped line while the canonical layer centred it. It now hides characters
with `visibility` instead, so the block always occupies its fully-typed layout — same width, same
height, same wrap points, first frame to last — which lets the rows stay `text-align: center` like
the canon, and makes the width lock unnecessary. Typing looks the same; it reveals rather than
inserts. **53/66px → 0px**, and still 0-1px at 1100px.

**`kinetic-gravity`, 6.3% of the stage out at the end.** Its incoming bounce ran `span(t, lead, 1)`
— the window closed at exactly t=1, so at t=0.992 `outBounce` had not quite reached 1 and every
word was still a fraction above its resting place. At 1100px that was a 3px residual, the worst in
the set; at 680px the long slogan runs to more lines and the residual multiplies across them. The
bounce now lands at 0.96. **≤3px at both widths.** (Same shape of bug as `glyph-scramble`'s
stagger in round 5: a schedule that uses the whole of 0..1 leaves no room to be *finished* in.)

**`depth-flip`, 21px at the end — NOT FIXED, and deliberately so.** At 680px the leaf's own copy of
the incoming text wraps differently from the backing layer's copy, so the two halves of the fold
disagree and 1065 pixels of the settled frame sit outside the canonical text. It is contract-clean
at 1100px, the width Tom rates at. Fixing it means restructuring how a 3D fold shares one text
layout across four faces, which is not something to attempt speculatively at 01:30 on an effect
that currently works. Recorded here so it is not mistaken for a passing effect.

## Round 7 — depth-flip, isolated rather than guessed

`depth-flip` was the one defect left, 21px at 680px only, and two rounds of reading the code did not
explain it: the leaf is an opaque full-width sheet hinged on the centre line, it lands at 180
degrees, and it ought to cover the outgoing bottom half completely.

Rather than keep theorising, the four surfaces in the scene were hidden one at a time at t=0.992 and
the ink box measured each time. Hiding the outgoing bottom-half layer — and only that — reproduced
the canonical frame exactly. So the leaf's own back face was already carrying the incoming bottom
half correctly; the old copy underneath simply was not covered, because at 680px the long slogan
wraps, the block is taller, and the fold no longer reaches it at the very end.

The outgoing bottom half now fades out over the last 30 degrees of the fall. It cannot affect any
earlier frame, because until then it sits behind an opaque leaf either way. **21px → 1px at 680px,
unchanged at 1100px.**

The general lesson, twice over tonight: when the code says a thing should be covered and the pixels
say it is not, stop reading and start removing surfaces one at a time.

## Round 7 verification

    canonical invariance   0 failures   all 96 render a pixel-identical rest state
    determinism            0 failures   no Math.random, no clock
    continuity             1 flagged    light-neon, which is a flicker on purpose
    thrown errors          0

---

# Final state

Every number below is from a run made after the last code change, not carried forward.

| sweep | records | result |
|---|---|---|
| 96 effects x all 14 slogan pairs, 1100px (the rater's width) | 1344 | **96 clean, 0 defects** |
| 96 effects x pairs 0 and 12, 680px (a width where the long slogan wraps) | 192 | **96 clean, 0 defects** |
| canonical invariance / determinism / continuity, 96 effects | 288 | 0 / 0 / 1 (`light-neon`, a deliberate flicker) |

Residual deviation across the 1344-record run — this is what "clean" actually means here:

    ink box offset at the endpoints    median 1px   p95 2px   worst 4px
    visible difference from canonical  median 0.7%  p95 1.8%  worst 4.3%
    frames brighter than canonical     worst +8.7%  (mid-transition, no full-stage flash anywhere)
    thrown errors                      0

For scale: the headline is 46px type on a 1100px stage, so a 1-2px endpoint offset is anti-aliasing,
not something an eye catches. At the start of the night the worst offender was 706px.

## The arc

    baseline   34 clean / 62 defective
    round 1    51 clean / 45 defective   paper palette, <br>
    round 2    73 clean / 23 defective   kerning
    round 3    96 clean /  0 defective   per-effect geometry (16 effects, 5 agents)
    round 5    3 more found by sweeping all 14 slogan pairs, all fixed
    round 6    3 more found at 680px, 2 fixed then, 1 in round 7
    round 7    depth-flip, isolated by removing surfaces one at a time

## Four root causes accounted for most of it

Three were single fixes that repaired whole clusters, and are worth remembering because each one
had already been found and fixed *somewhere* in this codebase and then not applied everywhere:

1. **A `<br>` between two `display:block` halves** opens an empty third line box. Seven effect files
   carried a comment warning about it. Seven others still had the bug — 17 effects, all off by
   exactly 45px.
2. **Effects painting their own white paper** onto a stage that is nearly black. The sheet is not
   white; the sheet is whatever the stage already is. `engine.js` now derives it.
3. **Per-character inline-blocks cannot kern**, so split text is wider than the same text as one
   run. Since a split line cannot be made to kern, the plain run stops kerning instead.
4. Everything else was individual: a clip helper computing a negative inset, a schedule that used
   the whole of 0..1 and so had no room to be *finished* in, a filter toggled rather than ramped,
   a phantom space in front of a collapsed word group.

## What is deliberately NOT changed

- `light-neon` flickers in a square wave. That is the effect.
- The 96 GIFs in `gifs/` are from 21 August and predate all of this. `gallery.html` is the only
  page that reads them; `rate.html` mounts the live effects, so the rating tool is current.
- Nothing outside `slogan_lab/` was touched. `final/index.html` and the store are untouched, as
  instructed — the shipped page still carries the engine as it was.

## Two judgement calls a metric cannot make

- `hy-origami` was re-implemented rather than repaired: it no longer flips 180 degrees to a back
  face, it folds edge-on and back with the text swapped at the crease. Contract-clean, but it looks
  different from what it was.
- `hx-spotlight` is contract-clean, but its beam easing leaves roughly the first fifth and last
  fifth of the transition as still frames. That may read as a slow start rather than a bug.

## The tools, if this needs doing again

    python3 audit_frames.py --pairs 0,12            # endpoints, flash, blanks, throws
    python3 audit_frames.py --quick --pairs 0,1,..  # thinned mid-probes, for sweeping many pairs
    python3 audit_frames.py --width 680             # a width where the long slogan wraps
    python3 audit_extra.py                          # invariance, determinism, continuity
    python3 audit_report.py _audit/<file>.json      # ranked defect list

`_frameaudit.html` is the harness; its stage is a copy of `rate.html`'s, so what it measures is what
Tom sees. Change one, change the other.

---

# Round 8 — the rater's "On white" toggle

`rate.html` has an **On white** button. Everything above was measured on the dark stage, so the
light one was an entirely untested surface. Auditing it found the *mirror image* of the night's
first bug.

**First, the metric was wrong.** The initial white run reported all 96 effects flashing, which is
nonsense. "Flash" was defined as the fraction of the stage gone near-white — and on a white stage
the canonical frame is already 97% near-white, so every frame trips it. The measure now asks the
question Tom actually asked: does the **ground** change colour? It compares each frame's median
colour against the canonical frame's median colour. Validated both ways before trusting it — 0.000
on the repaired `glitch-tear`, and 0.939 when the old white sheet was forced back in for one mount,
against a 0.25 threshold.

With that fixed: 88 of 96 clean on white. After the four dark-sheet repairs below, **92 of 96**.

**Four effects hard-coded a DARK sheet** — `glitch-lock`, `glitch-crt`, `glitch-rgb`,
`liquid-mercury` — which is a black flash on a white stage, exactly the same mistake as the white
paper on a dark stage, pointing the other way. All four now take `ctx.sc.paper`. Verified on both
stages: ground shift 0.00-0.04 everywhere, and nothing on the dark stage regressed.

**Three effects legitimately vanish on white, and are deliberately left alone.** `light-neon`,
`light-bloom` and `dissolve-luminance` are built on additive light: `mixBlendMode: 'screen'`, white
halos, `rgba(255,253,244, …)` ink. White is screen's identity, so on a white ground they compose to
nothing. This is the same shape as the subtractive-print problem that `print-riso` and
`dissolve-chromatic` needed solving in round 1, and the fix is the same in reverse — flip to
`multiply` with dark ink when `ctx.sc.dark` is false. That is a genuine re-implementation of three
effects, and they are pixel-perfect on the dark stage, which is the one being rated. Not worth
doing speculatively at 03:30. **Known limitation, written down rather than half-fixed.**

`mechanical-dot-matrix` also differs by 15-20% on white (its dot rendering does not resolve to
plain text there). Same category, same reasoning.

---

# Morning state, 06:00

| check | result |
|---|---|
| 96 effects x 14 slogan pairs, dark stage 1100px (the rater) | **96 clean / 0 defects**, 1344 records |
| 96 effects x 2 pairs, dark stage 680px (wrapping width) | **96 clean / 0 defects** |
| 96 effects x 2 pairs, white stage (the "On white" toggle) | **92 clean / 4 known**, listed below |
| canonical invariance / determinism / thrown errors | 0 / 0 / 0 |
| continuity | 1 flagged: `light-neon`, a deliberate flicker |

Worst residual on the rated stage: ink box 4px, median 1px, on 46px type.

`rate.html` checked end to end at 06:00: 96 effects registered, stage `rgb(10,11,12)`, kerning
disabled, all three buttons present, ratings still cleared, headlines cycling, no console errors.

The four that are not clean on white, all understood and all perfect on the dark stage:

| effect | why |
|---|---|
| `light-neon` | additive light — `mixBlendMode: 'screen'`, and white is screen's identity |
| `light-bloom` | same |
| `dissolve-luminance` | same |
| `mechanical-dot-matrix` | its dot rendering does not resolve to plain text on a light ground |

The fix for all four is the reversal that `print-riso` and `dissolve-chromatic` already got in
round 1: flip to `multiply` with dark ink when `ctx.sc.dark` is false. It is a real
re-implementation of four effects and was not worth doing speculatively overnight.

The 96 GIFs in `gifs/` were re-exported from the fixed effects.

---

# Round 9 — Tom's verdicts, 2026-08-24

All 96 rated: **81 keep, 11 discard, 4 fix**. The four flagged for fixing carried no notes, so the
defect in each had to be found by looking. They all PASSED the frame audit, which is the point —
these are faults the endpoint metrics cannot see.

Looking meant `strip.py`, a new tool that stacks the frames of one transition into a single tall
image **as the rater draws it**. `filmstrip.html` could not be used: it applies each effect's
`theme`, and `rate.html` deliberately does not ("the page owns the palette, an effect owns only the
motion"), so a filmstrip shows an effect in monospace on cream paper that Tom saw in the page's own
sans on near-black.

**`type-find` — the type was being mangled.** Each replacement word was scaled horizontally to fit
the slot of the word it replaced, `clamp(a.w / b.w, 0.52, 1.85)`. An 85% stretch turns "one motor."
into a different, much heavier face; a 48% squeeze makes "The" look like a condensed cut. Half the
transition was set in fonts the page does not own. Removing the scale alone makes words collide,
which is why it was there — so instead each word now **re-justifies as it swaps** rather than the
whole line re-justifying at the end. The line is then only ever ragged at the boundary between the
part already replaced and the part not yet reached, which is what an editor doing a find-and-replace
actually looks like.

**`mechanical-typewriter` — a caret adrift on an empty stage.** During the carriage return the old
line has rolled away and the first key has not struck, and a bar of ink slid across the middle of a
blank stage attached to nothing. It now fades through the crossing, leaving the end of the old line
and arriving at the head of the new one.

**`liquid-mercury` — two faults.** The melt used `sin(pi*t)^1.35`, already at half strength by
t=0.2 and not back under it until t=0.8, so a *global* blur+contrast smeared the whole line for two
thirds of the transition — unreadable white worms. That contradicted the effect's own design note,
which says only a slice is ever molten: the per-cell wave was working, a global filter was melting
everything regardless. A gaussian centred on the swap is spent by t~0.25 and t~0.75. Separately the
contrast was dragging the near-black ground to pure black for the length of the transition; the
ground is now pre-distorted by the inverse of the contrast so the filter lands it exactly back on
the stage colour. Both goo effects got that treatment.

**`type-backspace` — the headline vanished, and a regression of mine.** The erase finished at
t=0.34 and the retype did not begin until t=0.39: 0.13s during which the entire headline was gone
and the only thing on the hero was a blinking caret. Closed to a single frame.

The second fault was mine, from the night before. I had changed it to hide characters with
`visibility` so the block keeps its settled layout — and `visibility: hidden` is the one form of
hiding a DESCENDANT can override. Every typed character setting `visibility: visible` reappeared
through the engine's hidden layer, so at t=1 the effect's own text and caret were drawn on top of
the canonical frame: 0.8% of the stage, including a caret the canon does not have.

**The engine now closes that hole for good.** `_showCanon` sets the layer to `opacity: 0` as well as
`visibility: hidden`. A parent at opacity 0 cannot be overridden from inside, and unlike
`display: none` it keeps the layer laid out, so effects that measure their own geometry still can.
Only one effect was exploiting the gap, but any future one could have.

After all four: every endpoint back to **0.0000% difference from canonical**, and a full re-sweep of
all 96 at pairs 0 and 12 still **96 clean / 0 defects**.

## Shipping Tom's decisions

`build_page_engine.py` rebuilds the engine inlined in `final/index.html` from `slogan_lab/`, reading
the verdicts out of `ratings/transitions.json`. It drops the `FX.register({...})` block of every
discarded effect by brace matching — a regex cannot, since effect bodies contain braces inside
strings, comments and regexes — and rewrites the cycler's POOL to exactly what was kept. Helper
functions of discarded effects stay, because they are often shared with kept ones; only the
registration goes, so the effect can never be picked.

Result: **85 registered, 85 in the pool, none of the 11 discards present**, verified in a browser.
The page's inlined engine grows 118 KB -> 430 KB, which is the honest cost of keeping 85 effects.

---

# Round 10 — Tom's three comments, and a check that would have caught them

His notes came back in the rater's `why` field (not `note`, which is why the first read showed them
empty). `liquid-mercury` passed. Three left:

**`type-find`** — "the yellow highlight, I would like it if it were green instead to go with my page
better." Now Gearotons green `#7AB648`. Switching it made a second thing obvious: a word being
DELETED has no replacement to ride the re-justify with, so its selection stayed parked at its old
slot while the line closed up around it — a lone block of colour past the end of the new line. Pale
yellow hid that; brand green did not. It now fades out with the word it is deleting.

**`type-backspace`** — "the letters drop by about a pixel after the transition." They did, and it
was a real 1px, not a rendering artefact: shifting the last effect frame down one pixel fitted the
canonical frame better than not shifting it. The cause was the caret. It is a zero-WIDTH caret, but
not a zero-HEIGHT one: sitting on the baseline at `vertical-align:-0.19em` it hung below the row's
descender and grew that line box by 1.05px. The block is flex-centred, so one row a pixel taller
lifted the whole headline a pixel above the canonical text. Top-aligned it is shorter than the strut
and cannot grow the box, and `position:relative` puts it back where it looked right — a relative
offset is painted, not laid out. The effect's block is now 115.32/99.36, the canon's 115.32/99.36,
to the hundredth of a pixel.

**`mechanical-typewriter`** — "the letters move by about a pixel... would be better if they just
were typed out one by one accurately in their final position." The per-letter misalignment that
makes a struck line look hand-set rather than typeset was being applied permanently: `_dy` and
`_rot` stayed on every glyph at rest, so each letter finished a fraction off the canonical position
and the line twitched as a whole when the engine took over. The jitter now rolls off to nothing as
each letter settles, exactly as he asked. Residual 0.00427 -> 0.00260, which is *below* `wipe-iris`,
an effect nobody has complained about.

## A new check, because he found this twice and the audit did not

`audit_frames.py` now measures **the settle**: the last effect frame (t=0.999) against the canonical
frame it hands over to, plus a shift test — if the residual drops when the frame is moved a pixel,
it is a real positional error rather than anti-aliasing. That is precisely what he was seeing, and
the endpoint checks at t=0.008/0.992 were too coarse to catch it.

Running it over all 96 found the same fault in **nine more effects**, eight of them a true -1px:

- The whole **depth family** (cube, flip, hinge, dolly, helix) shared one cause: `equalize()` rounded
  the block height to a whole pixel, but two 49.68px rows are 99.36, so the block came out a third
  of a pixel short of the canon and the flex centring put it in the wrong place. It now uses the
  measured fractional height. All five clean.
- **`odometer`** and **`kinetic-gravity`** are still out at pair 12 only. An attempt to snap the
  odometer's drum travel to whole pixels made it *worse* — its cell geometry is in em throughout and
  a px translate fights it — so that was reverted rather than left as a half-fix. Both are recorded
  here rather than shipped as fixed.
- **`mechanical-split-flap`** and **`wipe-blinds`** also show it, and both are effects Tom discarded,
  so they never reach the page.

**92 of 96 clean under the new, stricter check**; of the 85 that actually ship, 83.

## A build guard that misfired, and shipped the wrong thing for one deploy

`build_page_engine.py` treated "the POOL substitution came back identical" as a failure. On a
rebuild where the pool has not changed that is the NORMAL case, so it aborted, left the page
carrying the previous engine — and the deploy that ran straight afterwards happily shipped it. It
now fails only if the POOL line is absent. Caught by diffing the deployed bundle for the fixes it
was supposed to contain, which is a check worth keeping in the habit.
