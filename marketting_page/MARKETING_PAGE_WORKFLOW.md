# Marketing page: hand-authored, with facts under source control

> **New here?** Read [`STATE_OF_PLAY.md`](STATE_OF_PLAY.md) first — it is the resume point
> for the whole marketing page and the slogan transitions. This document is the *rules*;
> that one is the *state*.

**Decided 2026-08-22 by Tom.** The template generator is retired as the way the page is *designed*.
The page is now hand-edited HTML. In exchange, every FACT on it stays tied to a single source of
truth, and no hand edit is allowed to silently diverge from that source.

---

## 1. The page

**`final/index.html`** is the marketing page. It is hand-authored. Edit it directly.

`ARCHIVED_build_final_from_scratch.py` (was `build_final.py`) and `templates/next_index_template.jsx`
are how it was *first assembled* (from the four designs Tom kept). They are now historical, and if
the page ever needs rebuilding from scratch that is a deliberate decision to be discussed first.

**Two scripts can destroy the hand-authored page, and both now refuse to run without the same
flag, `--i-know-this-overwrites-hand-edits`:**

| Script | What it would overwrite | Guarded |
|---|---|---|
| `ARCHIVED_build_final_from_scratch.py` | `final/index.html` — the page itself | 2026-08-22 |
| `generate_webpage.py` | the STORE's `components/MarketingContent.js`, rebuilt from `templates/next_index_template.jsx` and copied into the store | 2026-08-23 |

The guard on `generate_webpage.py` is on `main()` only. `deploy_to_store.py` imports
`copy_to_ecommerce()` from it, and that import stays unguarded on purpose — see section 6.

---

## 2. The source register — who owns which fact

| Fact class | Source of truth | Notes |
|---|---|---|
| Mechanical + electrical specs, operating conditions | `servomotor_datasheets/specs.py` | Renders the customer-facing datasheet PDF. **The page must never disagree with the datasheet.** |
| Model line-up, intro paragraph | `servomotor_datasheets/introduction.txt` | |
| LED + button behaviour | `servomotor_datasheets/indicators.py` | |
| Datasheet version / date | `servomotor_datasheets/versions.txt` | |
| The 16 key features | `marketting_page/features.txt` | |
| RS-485 protocol facts (baud, addressing, firmware update) | `marketting_page/communication.txt` | |
| Company story | `marketting_page/company_profile.txt` | |
| Kit contents / what's in the box | `marketting_page/package.txt` | |
| Unit system (7 quantities and their units) | `python_programs/servomotor/unit_conversions_M3.json` | Generated from the library — never hand-edit. |
| Hardware-measured behaviour (pinout, real PID rate, measured top speed) | `API_documentation/autogeneration/hardware_setup.md` | Written from bench measurement; **beats the datasheet where they disagree on measured reality.** |
| Command set | `python_programs/servomotor/motor_commands.json` | |
| Dimension drawings, product photos | `servomotor_datasheets/*.png`, `marketting_page/*.jpg`, `transparent/` | |

Two convenience documents consolidate the above. They are **derived, not authoritative** — useful
for writing copy, but if they disagree with a source above, the source wins:
`premium_minimal_variations/BRIEF.md` and `premium_minimal_variations/UPDATE_4_MOTORS.md`.

---

## 3. The rule

**Style is free. Facts are gated.**

- **Free — just do it, no need to ask.** Layout, typography, colour, spacing, section order,
  headline wording, transitions, imagery treatment, CTA phrasing, anything about *how* something is
  said or shown.

- **Gated — STOP and ask Tom.** Any edit that changes, adds or removes a *fact* that appears in the
  source register: a number, a spec, a model name, a protocol detail, a behaviour, a company claim.

When a gated edit comes up, bring Tom exactly three options:

1. **Change both** — update the source file *and* the page, so they stay in step.
2. **Don't change the page** — the source is right, the page should match it as-is.
3. **The source is wrong** — fix the source first, then follow it. (This is the case when the page
   is describing reality more accurately than the source does.)

Never pick one silently. A page that quietly disagrees with the datasheet is worse than either
version alone, because the customer sees both.

---

## 4. The check

`python3 check_facts.py` compares the hand-authored page against the sources and reports drift.
Run it after editing the page, and before any deploy. It is deliberately dumb — it looks for the
canonical numbers and flags anything stale — so it cannot catch a *wording* drift, only a factual
one. Human judgement still owns the rest.

---

## 5. Known conflicts — Tom's rulings, 2026-08-22

1. **PID loop rate — RESOLVED, no change.** Datasheet and page say 32 kHz; the bench doc says
   31.25 kHz. Tom: *"it was rounded and that is fine for marketing."* Leave the page at 32 kHz.

2. **Top speed — RESOLVED, keep "up to 560 RPM".** Tom: *"that is what is on the label and I don't
   want to change the label. The label is not a lie — I did at one point measure the top speed to
   be that. I don't know what test now shows that it is lower."* The ~516 RPM figure in
   `hardware_setup.md` is a later bench measurement whose provenance is unexplained. Do not "fix"
   the page to match it, and do not quietly drop the claim.

3. **Rear-label photo — FIXED 2026-08-22 by AI retouch.** Every rear-label photo in the repo showed
   `Model: M3-60` and `Voltage: 12-30 V`, because the physical labels on the hardware still carry the
   old M3 naming. Tom confirmed shipping units will say M17 and chose 12–24 V for the voltage.
   The label was retouched with `google/gemini-3-pro-image` via OpenRouter, in two passes
   (spec block, then the `+12-30V` connector silkscreen), starting from the best available source,
   `Pictures/servomotor3/M17-60 motor/*154554.jpg`. **The QR code was then composited back from the
   real photograph** — the model regenerates the whole frame, so an AI-drawn QR could silently not
   scan. Verified the restore changed 0.00% of the spec-text zone. Torque, speed, current and weight
   are untouched from the original photo.
   Installed as `motor_back.jpg`, `motor_back_small.jpg` and the two `transparent/motor_back_*`
   cutouts; previous versions kept as `.bak`.
   *If the hardware labels are revised in production, re-shoot and drop the retouch.*

4. **Duplicate intro — FIXED.** `marketting_page/introduction.txt` was a byte-identical copy of the
   datasheet's. It is now a **symlink** to `../servomotor_datasheets/introduction.txt`, so there is
   one file and one source, and `generate_webpage.py` still reads it. The pre-four-motor original is
   kept at `introduction.txt.bak`.

5. **`features.txt` — RESOLVED, no change.** Tom: adding the fourth motor does not change the
   feature list.

## 6. Deploying to the e-commerce store

**`python3 build_page_engine.py`** comes first, and only when the effects or the keep list have
changed. It rebuilds the slogan engine inlined in `final/index.html` from `slogan_lab/`, reading
the verdicts out of `ratings/transitions.json` and physically stripping the `FX.register` block
of every discarded effect, so a discarded effect cannot be picked even by accident. Everything
else on the page is hand-edited and this never touches it.

**`python3 deploy_to_store.py`** ships `final/index.html` into the local store working tree at
`../../AI_testing/selling_web_site`. Use `--dry-run` to build into `preview/` and touch nothing else.
It never commits and never pushes — publishing is a separate, deliberate act.

It **replaces half of `generate_webpage.py` and reuses the other half.** That script's `main()` is
two steps:

```
create_unified_preview('./preview')                     # build preview/ from templates/
copy_to_ecommerce('./preview', <store>)                 # copy preview/ -> store
```

Only the *generate* step is obsolete: it rebuilds the component from
`templates/next_index_template.jsx`, the old stacked design, so running `generate_webpage.py` now
would publish the old layout over the new page. The *copy* step is the deploy, it is generic, and it
copies exactly the three paths the new build writes into `preview/` — so `deploy_to_store.py` calls
straight into `generate_webpage.copy_to_ecommerce()` rather than carrying a second copier that can
drift out of step with it.

`copy_to_ecommerce()` `rmtree`s `public/marketing/` before copying, which is safe **only because
`preview/` carries everything the store should end up with**, including `logos/Gearotons_Logo.png`
that the page itself never references. `deploy_to_store.py` checks for that file and refuses to copy
if it is missing from both `preview/` and the store. The rmtree is also what cleans up the previous
page's now-orphaned assets (11 `*_small.jpg` and friends on the 2026-08-23 deploy — all verified
unreferenced first).

### What the deploy changes on the way out

A standalone page and a page embedded in a store are not the same document, so five adaptations are
applied. Everything else — markup, copy, layout, the slogan engine — ships byte-for-byte.

1. **The CSS is scoped under `.mkt-root`.** The page's stylesheet is global: it styles `nav`,
   `footer`, `a`, `img`, `table` and 22 other bare elements, which would restyle the store's own
   Header, SiteFooter and cart. `html`/`body`/`:root` map onto the wrapper; `.js` (which the reveal
   script puts on `<html>`) stays an ancestor.
2. **A reset for the leak in the other direction.** The store's `styles/globals.css` styles bare
   `h1`-`h6`, `p`, `a` and `main`. Where the page never declares that property itself — a heading's
   colour, which the standalone page simply inherits from `<body>` — the store's rule won and the
   hero headline rendered navy `#0F172A` on the near-black hero. A `:where()` reset (zero
   specificity, so it beats bare `h1 {}` and loses to every page rule) restores the page's baseline.
3. **The page's own `<nav>` is removed**, with the script that recolours it on scroll. The store
   renders a global `<Header/>` in `pages/_app.js`; keeping both stacks two navigation bars. The nav
   is `position:sticky`, so removing it costs no layout compensation. `--keep-nav` opts out.
4. **Class names that collide with the store's `globals.css` are renamed** (`card` → `mkt-card`,
   `btn` → `mkt-btn`, `btn-primary` → `mkt-btn-primary`). Scoping and the reset in (2) both miss
   this case: the store styles those class names too, so every property the store declares and the
   page does not was silently inherited — the spec cards had picked up a white background, a drop
   shadow, `overflow:hidden` and a hover shadow, and the hero CTA a hover nudge. Renaming beats
   another reset layer, which would have to enumerate every property the store might ever set. The
   deploy refuses if a colliding name is also referenced as a string in the page's JS, since the
   rename does not touch the lifted script.
5. **Same-site `https://gearotons.com/...` links become root-relative.** Absolute links pin the
   visitor to production, so on Amplify staging the CTA jumped to a different deployment and the
   staging copy could not be reviewed by clicking through it. Only the apex is rewritten —
   `tutorial.gearotons.com` is genuinely another origin. **Beware:** this rewrites the apex
   wherever it appears, so a link that *means* something else must not use the bare apex as its
   target. That already bit once: the two "documentation" CTAs pointed at `https://gearotons.com`
   and were rewritten to `/`, sending readers back to the homepage. They now point at the real
   documentation URL.

The generated component also **tears the engine down when it unmounts**. The store is a single-page
app, so React unmounts the marketing page on every client-side navigation; without a teardown the
slogan engine's self-re-arming `requestAnimationFrame` loop and its `window`/`document` listeners
outlive it, and Home → Store → Home three times leaves four players animating detached DOM at
60 fps with only the newest reachable to stop. The effect records what the lifted script starts and
undoes exactly that. Verified over three round trips: three old players, none still running.

Asset paths become `/marketing/images/...` and the files are copied. The previous component is kept
as `components/MarketingContent.js.bak`. `styles/Marketing.module.css` is deliberately NOT staged
into `preview/`, so `copy_to_ecommerce()` skips it and the store's own copy is left untouched — the
new component ships its CSS inline and does not import the module.

### Verifying a deploy

Run the store locally (`npx next dev -p 3555` in the store dir) and compare against the standalone
page. Last verified 2026-08-23: all 14 sections pixel-identical (19611 px), computed styles matching,
slogan engine live.

**Gotcha that will cost you an hour if you forget it:** Chrome does not run
`requestAnimationFrame` in a background tab, so **Next.js never hydrates a page in a tab that is not
in the foreground** — no `useEffect`, no IntersectionObserver, so the slogan engine looks dead and
every `.reveal` section stays blank. This is not a bug in the page. Check
`document.visibilityState` before concluding anything, and force the tab to the front (take a
screenshot) before reading state. `window.__marketingEffect` in the generated component exists to
tell "the effect never ran" apart from "the engine threw".
