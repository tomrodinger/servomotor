# Set C — operational notes

Independent third design set (Grok 4.6). Company list matches set A for apples-to-apples comparison. Do not treat A (`design_variations/`) or B (`design_variations_B/`) as a design source unless Tom asks.

## Serve

From `marketting_page/` (image paths are `../…`):

```bash
python3 -m http.server 8912 --bind 127.0.0.1
```

Viewer: http://127.0.0.1:8912/design_variations_C/

8910 = set A, 8911 = set B. Do not kill those servers.

## What’s in this folder

- `v1.html`–`v10.html` — original directions
- `v11.html`–`v40.html` — company pastiches (same 30 names as A; see `RANKING_AND_MAPPING.md`)
- `index.html` — viewer (`#v12` deep-links)
- `_content.py` — shared facts; `build_r1.py` / `build_r2a.py` / `build_r2b.py` / `build_r2c.py` generate the pages
- `_apply_cutouts.py` — post-pass: studio JPEGs → transparent PNGs + cutout CSS

If you regenerate from the `build_*.py` scripts, re-run `_apply_cutouts.py` (or the HTML will lose later polish: overflow-safe CSS, `dark-cut` shadows, a few layout fixes).

## Product photos

Studio shots have white backgrounds. On dark or colored grounds use the cutouts in `../transparent/`:

| JPEG | Cutout (prefer `_small` except huge heroes) |
|---|---|
| `one_motor*.jpg` | `one_motor_transparent[_small].png` |
| `M17_series_overview.jpg` | `M17_series_overview_transparent[_small].png` |
| `kit_with_three_motors*.jpg` | `kit_with_three_motors_transparent[_small].png` |
| `motor_back*.jpg` | `motor_back_transparent[_small].png` |
| `adapter_and_wire*.jpg` | `adapter_and_wire_transparent[_small].png` |

Do **not** swap: `robotics`, `automation`, `test_rack`, `connection_diagram`, dimension drawings, logos, OSI/OSH marks.

Dark pages: `body.dark-cut` (or `img.on-dark`) for a stronger drop-shadow. Light pages: a light shadow only — a heavy one looks like a gray rectangle.

Wordmark `Gearotons_Logo_and_Gearotons_Name.png` is white-backed; only use it on white/cream nav. On dark nav use `Gearotons_Logo.png` and invert if needed.

## Honesty

No invented prices, awards, customer counts, or testimonials. Shop → https://gearotons.com/store. Docs → https://gearotons.com. Source → https://github.com/tomrodinger/servomotor.

## Research gaps (2026-08-14)

Blocked or not fetched live: Tesla, Mastercard, Lilly, AMD, Applied Materials, AbbVie, Cisco, Caterpillar. Those pastiches used known current marketing systems, noted in `RANKING_AND_MAPPING.md`.
