# Instructions: swap studio photos for transparent-background cutouts

## Asset mapping (paths relative to design_variations/, where the vN.html files live)
Old (white background) → New (transparent PNG, subject-cropped):

| Old | New (full) | New (web-sized ≤1200px) |
|---|---|---|
| `../one_motor.jpg` | `../transparent/one_motor_transparent.png` | `../transparent/one_motor_transparent_small.png` |
| `../M17_series_overview.jpg` | `../transparent/M17_series_overview_transparent.png` | `../transparent/M17_series_overview_transparent_small.png` |
| `../kit_with_three_motors.jpg` | `../transparent/kit_with_three_motors_transparent.png` | `../transparent/kit_with_three_motors_transparent_small.png` |
| `../motor_back.jpg` | `../transparent/motor_back_transparent.png` | `../transparent/motor_back_transparent_small.png` |
| `../adapter_and_wire.jpg` | `../transparent/adapter_and_wire_transparent.png` | `../transparent/adapter_and_wire_transparent_small.png` |
| `../preview/public/marketing/images/one_motor_small.jpg` | — | `../transparent/one_motor_transparent_small.png` |
| `../preview/public/marketing/images/M17_series_overview.jpg` | — | `../transparent/M17_series_overview_transparent_small.png` |
| `../preview/public/marketing/images/kit_with_three_motors_small.jpg` | — | `../transparent/kit_with_three_motors_transparent_small.png` |
| `../preview/public/marketing/images/motor_back_small.jpg` | — | `../transparent/motor_back_transparent_small.png` |
| `../preview/public/marketing/images/adapter_and_wire_small.jpg` | — | `../transparent/adapter_and_wire_transparent_small.png` |

Prefer the `_small` versions anywhere the image displays under ~700px wide; full versions for hero-scale imagery.

DO NOT touch: `../robotics.jpg`, `../automation.jpg`, `../test_rack.jpg` (real backgrounds),
`../connection_diagram.jpg`, dimension PNGs, logos, open-source logos, and any `*_small.jpg`
of the application photos. Only the five studio-shot products above have cutouts.

## Design rules (minimal, tasteful changes — do NOT redesign the page)
1. **Dark/colored contexts** (photo previously sat in a white card or on a dark band):
   swap the src AND, where the white box existed purely to host the photo, remove the white
   background/border so the product floats on the section background. Add a soft shadow so it
   doesn't float weightlessly: `filter: drop-shadow(0 18px 30px rgba(0,0,0,.45));` (tune per bg).
   If the white card is a deliberate design element of that variant (e.g. framed "photograph",
   taped polaroid, studio plinth aesthetic), KEEP the card and just swap the src — the cutout
   on white looks identical but cleaner.
2. **`object-fit: cover` full-bleed backgrounds**: cutouts are subject-cropped, so cover-cropping
   may amputate the product. Either keep the original JPG (fine!) or switch that element to
   `object-fit: contain` + a suitable background — whichever preserves the design intent better.
3. **`mix-blend-mode: multiply` hacks** (used to fake transparency on light bg): replace with the
   real cutout and remove the blend hack.
4. **Grayscale/brightness dimming filters** (e.g. SpaceX-style dark vignette treatment): keep the
   filter if it's the design; the cutout just removes the white sweep. Verify it reads well.
5. Aspect ratios changed (cutouts are tighter crops). Check nothing overflows or squishes;
   adjust max-height/width as needed.
6. Keep all pages fully self-contained; don't introduce external resources.

## Verification (required)
After editing each page, screenshot it with headless Chrome at 1280px wide
("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless --disable-gpu
--hide-scrollbars --window-size=1280,2200 --virtual-time-budget=8000 --screenshot=OUT.png
"http://127.0.0.1:8910/design_variations/vN.html"), Read the screenshot, and confirm:
no white rectangles remain around swapped products on non-white backgrounds, nothing is
cropped/distorted, and the page still looks like its design. Iterate until clean.
