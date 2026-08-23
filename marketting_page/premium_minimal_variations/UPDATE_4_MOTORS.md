# Update: the M17 series is now FOUR motors, and several existing numbers changed

Source of truth: `servomotor_datasheets/` — datasheet **v1.10, 15 Jul 2026**
(`datasheet_latest_en.pdf`, generated from `servomotor_datasheets/specs.py`). Verified against the
rendered PDF, page 9 (mechanical) and page 10 (electrical), not just the generator source.

**This supersedes the spec numbers in `BRIEF.md` §4.4 and §4.5.** Everything else in BRIEF.md still
stands.

---

## 1. The new model: M17-34

A fourth, shorter model joins the range. It is the smallest and lightest: same 42.2 × 42.2 mm NEMA 17
footprint, 33.5 mm tall, 210 g, 0.28 N·m.

The line-up, tallest to shortest: **M17-60 · M17-48 · M17-40 · M17-34**.

## 2. Mechanical specifications (REPLACES BRIEF §4.4)

| Parameter | M17-60 | M17-48 | M17-40 | M17-34 |
|---|---|---|---|---|
| Dimensions (L×W) | 42.2 × 42.2 mm | 42.2 × 42.2 mm | 42.2 × 42.2 mm | 42.2 × 42.2 mm |
| Height | **59.7 mm** | **48.7 mm** | **40.1 mm** | **33.5 mm** |
| Shaft length | **20.6 mm** | **20.6 mm** | **20.6 mm** | **20.6 mm** |
| Shaft diameter | 5 mm | 5 mm | 5 mm | 5 mm |
| Weight | 470 g | 360 g | 285 g | **210 g** |
| Protection class | IP20 | IP20 | IP20 | IP20 |

## 3. Electrical specifications (REPLACES BRIEF §4.5)

| Parameter | M17-60 | M17-48 | M17-40 | M17-34 |
|---|---|---|---|---|
| Operating voltage | 12–24 V | 12–24 V | 12–24 V | 12–24 V |
| Rated torque | 0.65 N·m | 0.55 N·m | 0.42 N·m | **0.28 N·m** |
| Maximum speed | 560 RPM | 560 RPM | 560 RPM | 560 RPM |
| Maximum current | 1.1 A | 1.1 A | 1.1 A | **1.0 A** |
| Rated power | **26.4 W** | **26.4 W** | **26.4 W** | **24.0 W** |

## 4. What CHANGED on numbers you already published

Do not just append a fourth column — these existing values are now different:

| Value | was | now |
|---|---|---|
| M17-60 height | 59.8 mm | **59.7 mm** |
| M17-48 height | 48.6 mm | **48.7 mm** |
| M17-40 height | 41.6 mm | **40.1 mm** |
| M17-60 shaft length | 20.4 mm | **20.6 mm** |
| M17-48 shaft length | 20.4 mm | **20.6 mm** |
| M17-40 shaft length | 18.5 mm | **20.6 mm** |
| M17-60 rated power | 38 W | **26.4 W** |
| M17-48 rated power | 32 W | **26.4 W** |
| M17-40 rated power | 25 W | **26.4 W** |

Rated power is now the same for the three larger models. If a page's copy leans on rated power to
differentiate the models (e.g. "38 W down to 25 W"), rewrite that line — **torque** is what separates
them now: 0.65 → 0.55 → 0.42 → 0.28 N·m.

Unchanged: 42.2 × 42.2 mm footprint, 5 mm shaft, IP20, 12–24 V, 560 RPM, weights of the three
existing models, all operating conditions, the whole protocol section, units, buttons.

## 5. Corrected LED description (REPLACES BRIEF §4.11, first bullet)

The datasheet fixed which LED shows bus traffic. Use exactly this behaviour:

> Two status LEDs, **green** and **red**. The green LED flashes slowly to show a heartbeat, flashes
> quickly to indicate the bootloader is running rather than the application, and lights up briefly
> while a packet is being received — so it flickers with communication on the bus. The **red** LED
> indicates fatal error codes by flashing a certain number of times.

The old text wrongly said the *red* LED shows bus traffic. Fix it wherever it appears.

The button behaviour (Reset / Test, 0.3 s, 2 s, 15 s, remove the load before calibrating) is
unchanged.

## 6. Images

| Purpose | Path | Note |
|---|---|---|
| Four motors side by side | `../M17_series_overview.jpg` | **NEW photo, all four models**, white bg |
| Same, cut out | `../transparent/M17_series_overview_transparent_small.png` | **NEW**, four motors, transparent |
| Same, full res | `../transparent/M17_series_overview_transparent.png` | **NEW**, four motors, transparent |
| M17-60 drawing | `../M17-60_dimensions.png` | **REPLACED** — now titled "M17-60 Model" |
| M17-48 drawing | `../M17-48_dimensions.png` | **REPLACED** — now titled "M17-48 Model" |
| M17-40 drawing | `../M17-40_dimensions.png` | **REPLACED** — now titled "M17-40 Model" |
| M17-34 drawing | `../M17-34_dimensions.png` | **NEW** |

Two important notes:

1. **Use `../M17-nn_dimensions.png`, NOT `../preview/public/marketing/images/M17-nn_dimensions.png`.**
   The old `preview/...` copies are the stale, wrongly-titled "M3-nn Model" artwork. Every variant
   that currently points at the `preview/` path must be repointed at the top-level file.
2. The new four-motor photo carries **no printed model labels** (the old three-motor one did), so if
   your layout relied on labels being baked into the image, add real HTML labels underneath. In the
   photo the motors run **tallest on the left to shortest on the right: M17-60, M17-48, M17-40,
   M17-34.**

The cutout is wide (2893 × 1416, ≈2:1). Give it room; don't crop it to a square.

## 7. Copy changes

- "Three models" / "three sizes" / "three torque options" → **four**, everywhere including alt text,
  eyebrows, headlines and the `<title>` if it mentions a count.
- The intro line becomes: *"Available in four models — M17-60, M17-48, M17-40 and M17-34 — the series
  offers flexible options to match specific torque and size requirements while maintaining consistent
  control characteristics."* (rephrase in your page's voice; keep the facts).
- Any "three motors" alt text on the overview photo → "four".
- `kit_with_three_motors` is a **different** photo (the kit flat-lay) and genuinely contains three
  motors — leave its filename and its caption alone. Do not describe the kit as containing four.
- Wherever the model comparison is a 3-up grid, it becomes a 4-up. Check it still works at tablet
  width — 2×2 is usually better than 4-across below ~1100px, and 1-up on phones.

## 8. Still true, do not "fix"

The rear-label photo (`motor_back*`) shows a unit silk-screened "Model: M3-40 / Voltage: 12-30 V".
That is a real photograph of older hardware and cannot be corrected in HTML. Leave the image in
place and make sure no page copy repeats those two values.
