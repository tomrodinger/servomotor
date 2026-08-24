#!/usr/bin/env python3
"""Stack the frames of one transition into a single tall PNG, exactly as the RATER draws it.

`filmstrip.html` already does something like this, but it applies each effect's `theme` (font,
bg, fg) — and `rate.html` deliberately does not: "the page owns the palette, an effect owns only
the motion". So a filmstrip can show an effect in monospace on cream paper that Tom saw in the
page's own sans on near-black. For judging what he actually looked at, the frames have to come
from `_frameaudit.html`, whose stage is a copy of the rater's.

  python3 strip.py type-find                       # 12 frames, slogan pair 0
  python3 strip.py type-find --frames 16 --pair 12
  python3 strip.py a,b,c --out /tmp/strips
"""
import argparse
import io
import os
import sys

from PIL import Image

ROOT = os.path.dirname(os.path.abspath(__file__))
URL = "http://localhost:8912/slogan_lab/_frameaudit.html"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("effects")
    ap.add_argument("--frames", type=int, default=12)
    ap.add_argument("--pair", type=int, default=0)
    ap.add_argument("--white", action="store_true")
    ap.add_argument("--width", type=int, default=0,
                    help="stage width in px; audit_frames.py has this and strip.py did not, so a "
                         "defect that only shows at 680px could be measured but not looked at")
    ap.add_argument("--out", default=os.path.join(ROOT, "_audit", "strips"))
    a = ap.parse_args()

    from playwright.sync_api import sync_playwright
    ids = [x.strip() for x in a.effects.split(",") if x.strip()]
    os.makedirs(a.out, exist_ok=True)

    with sync_playwright() as pw:
        b = pw.chromium.launch(args=["--force-device-scale-factor=1"])
        page = b.new_page(viewport={"width": 1400, "height": 520})
        page.goto(URL, wait_until="load")
        page.evaluate("() => window.__ready")
        if a.width:
            page.evaluate("w => setWidth(w)", a.width)
        if a.white:
            page.evaluate("() => setWhite(true)")
        box = page.locator("#box")
        for fid in ids:
            page.evaluate("([i,p]) => mountFx(i,p)", [fid, a.pair])
            tiles = []
            for k in range(a.frames):
                t = k / float(a.frames - 1)
                page.evaluate("tt => setFrame(tt)", t)
                tiles.append(Image.open(io.BytesIO(box.screenshot(type="png"))).convert("RGB"))
            w, h = tiles[0].size
            # a hairline between frames, so a frame that is legitimately empty is still visible
            sheet = Image.new("RGB", (w, h * len(tiles) + len(tiles) - 1), (60, 60, 66))
            for k, im in enumerate(tiles):
                sheet.paste(im, (0, k * (h + 1)))
            p = os.path.join(a.out, "%s_p%d%s%s.png" % (fid, a.pair,
                                                        "_w%d" % a.width if a.width else "",
                                                        "_white" if a.white else ""))
            sheet.save(p)
            print("  %s  (%d frames, %dx%d)" % (p, len(tiles), sheet.width, sheet.height))
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
