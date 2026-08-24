# Marketing page — state of play, 2026-08-24

Read this first. It is the resume point for the marketing page and the slogan transitions, written
so the work can be picked up with no memory of how it got here.

Two companion documents go deeper and are still current:

- **`MARKETING_PAGE_WORKFLOW.md`** — how the page is edited, which facts are under source control,
  and how it reaches the store. Read it before changing anything on the page.
- **`slogan_lab/FIX_LOG.md`** — the full record of the transition bug hunt, including the wrong
  turns. Read it before touching an effect.

---

## Where things stand

The page is **hand-authored** in `final/index.html` and **deployed** to the e-commerce store. It is
live on Tom's Amplify staging server.

| thing | state |
|---|---|
| The page | `final/index.html`, hand-edited, 14 sections, dark hero + white body |
| Facts on it | checked against their sources by `check_facts.py` — 64 facts, currently clean |
| Slogan transitions | 96 built, **85 shipping**, 11 discarded by Tom. 15 slogans, all audited |
| Transition quality | 92/96 pass (`slogan_lab/_audit/FINAL_settle.json`); of the 85 that ship, **83** |
| Store component | `AI_testing/selling_web_site/components/MarketingContent.js`, ~516 KB |
| Store tests | `tests/unit/marketingContent.test.js` is the one to run after a deploy; `tests/components/MarketingContent.test.js` pins the teardown. Both pass |

## The pipeline, end to end

    slogan_lab/engine.js + slogan_lab/effects/*.js
        │   build_page_engine.py   (reads ratings/transitions.json; drops discarded effects)
        ▼
    final/index.html               ← the page. Hand-edited for everything except the engine block.
        │   deploy_to_store.py     (scopes CSS, strips the nav, rewrites links, lifts the script)
        ▼
    preview/components/MarketingContent.js
        │   generate_webpage.copy_to_ecommerce()   ← called BY deploy_to_store.py
        ▼
    AI_testing/selling_web_site/components/MarketingContent.js

Two commands, in this order:

    python3 build_page_engine.py     # only needed if effects or the keep list changed
    python3 deploy_to_store.py       # build + copy into the store working tree

`deploy_to_store.py` never commits and never pushes. It is **byte-reproducible**: running it against
an unchanged source leaves the store's working tree clean. If it does not, something has been
hand-edited in the store and the two have drifted — find out which before overwriting.

**Do not run `generate_webpage.py`.** It rebuilds the component from the OLD stacked template and
would publish that over the current page. It refuses without
`--i-know-this-overwrites-hand-edits`, as does `ARCHIVED_build_final_from_scratch.py`, which would
overwrite the hand-authored page itself. Its `copy_to_ecommerce()` half is NOT obsolete and is what
`deploy_to_store.py` calls.

## What the deployer changes on the way out, and why

A standalone page and a page inside a store are not the same document. **Five** adaptations, plus a
teardown, all in `deploy_to_store.py` with the reasoning at each one:

1. **CSS scoped under `.mkt-root`** — the page styles bare `nav`, `footer`, `a`, `img`, `table`;
   unscoped it restyles the store's Header, SiteFooter and cart.
2. **A `:where()` reset for the leak the other way** — the store's `globals.css` styles bare
   `h1`–`h6`/`p`/`a`, which beat what the page inherits. Without it the hero headline rendered navy
   on near-black.
3. **Colliding class names renamed** (`card` → `mkt-card`, `btn`, `btn-primary`) — both sides use
   those names, and every property the store declared and the page did not was silently inherited.
4. **The page's own `<nav>` removed** — the store renders a global `<Header/>`.
5. **Same-site links made root-relative** — absolute `https://gearotons.com/...` pinned staging
   visitors to production.
6. **The slogan engine lifted into a `useEffect`** — injected `<script>` never executes — with a
   teardown, because the store is a single-page app and React unmounts the page on navigation.

## Traps that cost real time

- **A background tab never hydrates.** Chrome fires no `requestAnimationFrame` in a hidden tab, so
  Next.js never finishes hydrating: no `useEffect`, no IntersectionObserver, and the page looks
  dead. Check `document.visibilityState` before diagnosing anything client-side. Take a screenshot
  to bring the tab forward first.
- **`window.__marketingEffect`** exists in the deployed component precisely to tell "the effect
  never ran" (a hydration failure) apart from "the engine threw".
- **`cp` is aliased to `cp -i`** here. A scripted overwrite will hang on the prompt. Use `/bin/cp`.
- The rater deliberately does NOT apply an effect's `theme` — "the page owns the palette, an effect
  owns only the motion". `filmstrip.html` does, so a filmstrip is not what Tom saw. Use `strip.py`.

## The transitions

`ratings/transitions.json` is the source of truth for what ships. `build_page_engine.py` reads it
and physically strips the `FX.register` block of every discarded effect, so a discarded effect
cannot be picked even by accident.

Current verdicts: **85 keep, 11 discard.** The eleven discarded are `ink-bleed`, `hx-letterpress`,
`light-shadow`, `mechanical-split-flap`, `mechanical-flip-cards`, `mechanical-dot-matrix`,
`print-riso`, `print-fold`, `type-redact`, `wipe-bar`, `wipe-blinds`.

To re-rate: serve with `python3 serve.py` and open `/slogan_lab/rate.html`. It writes verdicts and
comments back through a small API into `ratings/transitions.json`. **Comments land in the `why`
field, not `note`** — that cost a wrong first read.

### The audit tools

    python3 slogan_lab/audit_frames.py --pairs 0,12          # endpoints, flash, blanks, throws, settle
    python3 slogan_lab/audit_frames.py --quick --pairs 0,1,…  # thinned, for sweeping all 14 pairs
    python3 slogan_lab/audit_frames.py --width 680            # a width where the long slogan wraps
    python3 slogan_lab/audit_frames.py --white                # the rater's "On white" toggle
    python3 slogan_lab/audit_extra.py                         # invariance, determinism, continuity
    python3 slogan_lab/audit_report.py _audit/<file>.json     # ranked defect list
    python3 slogan_lab/strip.py <id> --frames 11              # frames stacked as the RATER draws them

They need the static server up (`python3 serve.py`, port 8912) and use Playwright + Chromium.
Always pass `--out`: the default is `_audit/report.json` and a second run silently overwrites the
first. A 96-effect two-pair `--quick` sweep is about 3-6 minutes; all fourteen pairs is about 40.

**The audit JSONs are gitignored** (`slogan_lab/.gitignore`), so a fresh clone has none of them and
cannot check any quality claim here without re-running a sweep. The ~50 on this disk are the only
evidence behind the numbers in FIX_LOG, and `git clean` would take them.

The **settle** check is the one that matters most and was added last: it compares the final effect
frame against the canonical frame it hands over to, and shift-tests it, because a 1px twitch at the
hand-over is the thing Tom notices and the coarser endpoint checks passed it twice.

## Known and deliberate — do not treat these as new bugs

- **`odometer`** and **`kinetic-gravity`** still settle ~1px out on slogan pair 12 only. Snapping
  the odometer's drum to whole pixels made it worse (its geometry is in `em` throughout) and was
  reverted rather than left half-done.
- **`light-neon`, `light-bloom`, `dissolve-luminance`** are invisible on the rater's "On white"
  toggle. They are additive-light effects (`mixBlendMode: 'screen'`) and white is screen's identity.
  Fixing means flipping them to multiply with dark ink, as `print-riso` and `dissolve-chromatic`
  already were. They are pixel-perfect on the dark stage, which is what ships.
- **`mechanical-dot-matrix`** likewise differs on white. It is discarded anyway.
- **`light-neon` flickers.** That is the effect, not a discontinuity.
- **`hy-origami`** was re-implemented rather than repaired and now folds edge-on instead of
  flipping to a back face. It looks different from before.
- **Payload**: the inlined engine is 433 KB and the store component 510 KB, because 85 effects
  ship. If that needs trimming the lever is the keep list, not the code — shorten it in
  `ratings/transitions.json` and re-run `build_page_engine.py`.
- The 96 GIFs in `slogan_lab/gifs/` are current. Only `gallery.html` reads them; `rate.html` mounts
  the live effects.

## You are not the only one working here

Another AI owns `AI_testing/selling_web_site` and **edits files on the marketing side directly**.
On 2026-08-23/24 it made four changes to this work, all of them correct:

- `deploy_to_store.py` — root-relative same-site links, so Amplify staging can be clicked through.
- `final/index.html` — documentation links pointed at `https://9o.at/M17_1.3`.
- `deploy_to_store.py` — **gated the teardown harness on an `inEngine` flag.** My interception of
  `requestAnimationFrame` and `addEventListener` was global while the component was mounted, so on
  unmount it would cancel a *bystander's* pending frame and remove a bystander's listener — gtag,
  the store Header, a context provider. Pinned by `tests/components/MarketingContent.test.js`.
- It committed both `deploy_to_store.py` and `final/index.html`, which had been untracked.

Practical consequences: **re-`stat` a file rather than trusting an mtime read earlier in the
session**, check `git log` on `marketting_page` before assuming a file is as you left it, and
before committing, attribute every change you are about to stage.

## What is not done

- `premium_minimal_variations/rate.html` has Tom's page verdicts (`ratings/pages.json`); v4 and v17
  carry notes about which sections should be white vs black that have **not** been acted on.
- **A CSP decision is already recorded in the OTHER repo** and constrains this one:
  `AI_testing/selling_web_site/CLAUDE.md` notes that `MARKETING_SCRIPT` runs through
  `new Function(...)`, which needs `'unsafe-eval'`; the CSP is Report-Only today, and the script is
  to be emitted as a real file (`/marketing/marketing.js`) the next time the generator is touched.
  Anyone editing `deploy_to_store.py` from this document alone would never learn that.
- **`components/MarketingContent.js.bak` is written once and never refreshed** — the deployer only
  creates it if it is absent. It is not a rollback point after the second deploy; git is.
- The store's `styles/Marketing.module.css` is stale and unused — the component ships its CSS
  inline. It is left alone deliberately so the deploy cannot clobber a hand edit.
- Nothing here has been pushed to the live production site. Staging only.
