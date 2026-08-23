# Slogan cycler — shared specification

Every one of the 20 variants gets its hero headline **cycling through Tom's 15 chosen slogans**,
with a transition effect derived from that page's own design direction.

Tom's instruction, verbatim in spirit: the effects should be *sick* — genuinely eye-catching — but he
chose **"keep each in-character"**, so each effect must look like it grew out of that page's design
language, not like a generic library animation bolted on. Blueprint gets drafting. Terminal gets
typing. Gallery gets a slow museum dissolve. An effect that would look equally at home on any of the
20 has failed the brief.

---

## 1. The fifteen slogans (exact — do not reword, do not add, do not drop)

Each has a LIGHT first beat and a BOLD second beat, matching the existing headline pattern.

```js
const SLOGANS = [
  ["A motor, and everything",              "that usually surrounds it."],
  ["The whole drive system.",              "Inside one motor."],
  ["Four systems.",                        "One motor."],
  ["Motion control,",                      "fully integrated."],
  ["You wanted motion.",                   "Not a wiring project."],
  ["A motor that",                         "needs nothing else."],
  ["The whole motion system.",             "One motor."],
  ["Everything that moves it,",            "inside it."],
  ["Motion control.",                      "All in one."],
  ["Stepper, driver, controller, encoder.","One part number."],
  ["Bolts in like a stepper.",             "Runs like a full servo."],
  ["Four bolts. One cable.",               "A complete servo system."],
  ["Servomotor:",                          "stepper, driver, controller and encoder in one."],
  ["No driver board. No encoder to mount.","It's all inside."],
  ["Driver, controller and encoder",       "are no longer your problem."],
];
```

**Starting slogan.** If your page's current hero is one of the fifteen, start there and continue
through the list from that index — the page must look untouched on first paint. Otherwise start at
index 0.

---

## 2. Hard requirements (all 20 pages)

1. **Self-contained.** All CSS and JS inline in the existing file. No CDN, no imports, no fonts.
2. **No layout shift, ever.** The slogans differ a lot in length — #13 is far longer than #3. The
   hero block must not resize as they cycle and must not push the page around. Measure all fifteen
   at load (render them invisibly, take the max height) and lock the container to it, or reserve
   space some equally robust way. Verify at 1440, 900 and 390 — this is the single most common way
   this task goes wrong.
3. **No horizontal overflow at any width**, including the longest slogan at 390px.
4. **`prefers-reduced-motion: reduce` → do not animate and do not cycle.** Show the first slogan
   statically. Test it; do not just write the media query.
5. **Pause when not visible.** `document.hidden` (visibilitychange) pauses; hovering or focusing the
   headline pauses; leaving resumes. Never run the timer on a hidden tab.
6. **Dwell ~4.5 s** on each slogan, transition ~0.7–1.4 s depending on the effect. Slow, confident
   effects (Gallery, Sage & Stone) may dwell longer. It must never feel frantic.
7. **Accessibility.** The `<h1>` keeps real text at all times. Put `aria-live="off"` on the rotating
   element so screen readers are not spammed. Never leave the h1 empty between transitions.
8. **Degrade with JS off.** The first slogan must be present in the HTML source, fully styled and
   visible, before any script runs. No flash of empty hero.
9. **Do not restyle the headline.** Same family, size, weight, colour, tracking and line breaks as
   the page has now. You are animating the change, not redesigning the type.
10. **60fps.** Animate `transform`, `opacity`, `filter`, `clip-path` — never `top`/`left`/`width`/
    `height` in a running animation. Avoid layout thrash inside rAF loops.

---

## 3. Required debug hook (this is how your work gets checked)

A screenshot cannot see a transition in flight, so every page must expose a deterministic seek:

```js
window.__slogan = {
  items: SLOGANS,              // the array
  index: () => currentIndex,   // integer
  pause: () => {},             // stop the auto timer
  resume: () => {},
  goto: (i) => {},             // jump to slogan i, no animation, settle immediately
  // freeze the transition FROM index i TO index i+1 at progress t (0..1) and render that frame.
  // Must be idempotent, must not advance the timer, must work while paused.
  seek: (i, t) => {},
};
```

`seek` is mandatory and must genuinely render the intermediate frame — if your effect is driven by a
CSS animation, drive it from a single custom property or `animation-delay` with
`animation-play-state: paused` so a given `t` always produces the same picture. Effects that cannot
be seeked cannot be reviewed and will be sent back.

---

## 4. Effect assignments

Each is a starting point, not a cage. If you find something better *in that page's idiom*, take it —
but justify it. Push hard on craft: easing, stagger, per-character or per-word timing, secondary
motion, a considered exit as well as an entrance. A plain crossfade is a failure everywhere except
where the direction genuinely calls for restraint.

| # | Design | Effect | Direction |
|---|---|---|---|
| 1 | Director's Cut | **Keynote dissolve + weight morph** | Outgoing lifts a few px, blurs and fades; incoming rises into place as its weight settles light→bold. Apple-calm, immaculate easing. |
| 2 | Ivory & Ink | **Ink bleed** | New line soaks into the paper — an irregular mask spreads from the baseline with soft bleed edges and a faint warm halo. Outgoing blots away. |
| 3 | Editorial Serif | **Hot-metal slug** | Lines swap like a printing slug dropping into a forme: a hard vertical set, tiny registration offset, one bounce, ink settling. |
| 4 | Midnight Inverse | **Light sweep** | A beam travels across the dark; letters are revealed by it and the old ones fall back into black behind it. Green rim-light on the wavefront. |
| 5 | Swiss Grid | **Hairline wipe** | A 1px rule sweeps left→right; text is clipped by its passage, old out and new in on the same edge. Strict, mechanical, no bounce. |
| 6 | Mono Technical | **Glyph scramble** | Per-character cycling through mono glyphs, settling left→right like a data readout. Fixed-width, no reflow, green caret trailing the settle point. |
| 7 | Green Field | **Field wipe** | A brand-green band sweeps through the headline; the text changes inside the band as it passes, white-on-green mid-sweep. |
| 8 | Asymmetric Split | **Axis shear** | The line splits along the page's 55/45 axis; halves slide opposite ways, the new line reassembles from the same split. |
| 9 | Soft Neutral | **Spring squash** | Words squash and stretch out, blur slightly, and the new line springs in with soft overshoot. Friendly, rounded, never rubbery. |
| 10 | Gallery | **Museum dissolve** | Very slow, very long cross-dissolve with a barely perceptible scale drift, like a gallery projection changing. Restraint is the effect. |
| 11 | Blueprint | **Leader-line draw** | A drafting leader line traces across with tick ends; the new text draws on along it. Old text erased by a drafting sweep. Print-blue accents. |
| 12 | Big Numerals | **Odometer roll** | Per-character vertical mechanical roll to the next slogan's character, staggered, with a tiny settle at the end of each drum. |
| 13 | Graphite & Chrome | **Specular sweep** | A hard specular highlight travels across brushed metal; the text under the highlight is the new one, the text outside it is the old. |
| 14 | Whisper Gradient | **Aurora condense** | Letters dissolve into a soft coloured wash that drifts, then re-condenses as the new line. Atmospheric, weightless. |
| 15 | Hairline Catalogue | **Cell flip** | The headline's bordered cell flips/redraws like a catalogue card; the 1px border re-draws around the new content. Systematic. |
| 16 | Sage & Stone | **Horizon slide** | The line slides under a horizon rule and the new one emerges from it, like a shadow crossing a wall. Slow, architectural, calm. |
| 17 | Terminal Minimal | **Typed** | Backspace the old line character by character, then type the new one. Blinking block cursor, realistic irregular key timing, phosphor green. |
| 18 | Magazine | **Page turn** | The headline turns like a page corner lifting and flipping to reveal the next. Paper curl shading, a real fold. |
| 19 | Scroll Choreography | **Kinetic stagger** | Words exit in a choreographed stagger and the new words fly in from alternating directions with spring physics. This page's showpiece. |
| 20 | Statement Type | **Poster slam** | Giant type slams in with an impact shake and a full-bleed band wipe. Loud, poster energy, heavy weight. |

---

## 5. Verification protocol (required before you report done)

The static server is **already running on port 8912** serving the `marketting_page/` root.
Use `./_shot.sh <url> <out.png> <width> <height>` — it renders true sub-500px widths via an iframe
harness and forces reduced motion off/on correctly for stills.

**NEVER run `pkill -f "Google Chrome"` or `killall "Google Chrome"`.** Tom's real browser matches
that pattern and it destroys his open tabs. `_shot.sh` cleans up after itself; you need no cleanup.
If you must call Chrome directly, always pass `--user-data-dir="$(mktemp -d)"`, capture `$!`, and
scope any `pkill` to that unique temp path.

1. **Rest states.** Screenshot the hero at 1440 for several slogans via `__slogan.goto(i)` — check
   every one of the fifteen fits, none overflows, none changes the block height.
2. **Mid-transition frames.** For at least three different slogan pairs, capture
   `t = 0.15, 0.35, 0.5, 0.65, 0.85` with `__slogan.seek(i, t)` and LOOK at them. This is where bad
   effects are exposed: dead frames, both lines illegibly overlapping, clipping, jumps.
3. **390 and 900 widths.** Longest slogan (#13), at rest and mid-transition.
4. **Reduced motion.** Confirm it renders statically and the timer never starts.
5. Iterate until the mid-transition frames genuinely look good, not merely non-broken.

Write a one-paragraph note on what makes your effect belong to this page specifically.
