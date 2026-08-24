#!/usr/bin/env python3
"""Frame-level audit of every slogan transition.

The engine makes each effect a pure function of progress t, so a frame can be *asked for* rather
than caught mid-animation. This drives `_frameaudit.html` one `_render(t)` at a time and measures:

  START SNAP  does t=0.008 look like the canonical FROM text?   (position, size, the lot)
  END SNAP    does t=0.992 look like the canonical TO text?
  FLASH       does any frame go bright on a #0A0B0C stage?      (the "white flash" bug)
  BLANK       does the headline vanish for a stretch mid-transition?
  ERRORS      did the effect throw? A throw freezes the frame and reads as a dull effect.

Two frames are compared both ways round, because each catches what the other misses:
  * pixels   — mean absolute difference, which sees colour, weight and anti-aliasing
  * ink box  — the union of the client rects of every text-bearing element, which sees a
               position or size jump that a blurred pixel diff can average away

Usage
  python3 audit_frames.py                       # every effect, default pairs
  python3 audit_frames.py --effects wipe-bar,iris-open
  python3 audit_frames.py --pairs 0,12 --save-fails
"""
import argparse
import io
import json
import os
import sys
import time

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "_audit")
URL = "http://localhost:8912/slogan_lab/_frameaudit.html"

# Frames to sample. Dense at the ends because that is where the canonical contract is tested, and
# spread across the middle because a flash can happen anywhere.
EARLY = [0.004, 0.008, 0.02, 0.04]
MID = [round(x, 3) for x in np.arange(0.08, 0.93, 0.06)]
LATE = [0.96, 0.98, 0.992, 0.996]
FRAMES = EARLY + MID + LATE

# A reduced sweep for breadth runs. The endpoint frames are the point of the audit and are all
# kept; the middle is thinned to four probes, which is enough to catch a flash or a blank but not
# enough to characterise an effect. Use the full set when judging one effect, this when sweeping
# every slogan pair.
QUICK = EARLY + [0.2, 0.4, 0.6, 0.8] + LATE


def grab(page, box):
    raw = box.screenshot(type="png")
    im = Image.open(io.BytesIO(raw)).convert("RGB")
    return np.asarray(im, dtype=np.float32) / 255.0


def luma(a):
    return a[..., 0] * 0.2126 + a[..., 1] * 0.7152 + a[..., 2] * 0.0722


def pix_diff(a, b):
    """Mean absolute difference, 0..1. Robust to a few stray pixels, sensitive to a real shift."""
    n = min(a.shape[0], b.shape[0]), min(a.shape[1], b.shape[1])
    return float(np.abs(a[:n[0], :n[1]] - b[:n[0], :n[1]]).mean())


def ink_bbox(img, bg=None):
    """Bounding box of everything drawn on the stage, measured from PIXELS.

    Asking the DOM was the obvious approach and it is wrong: effects build two full-stage layers
    (outgoing and incoming), so the union of their client rects is the whole stage no matter what
    is actually visible. What matters is where the light is, so threshold against the stage
    background and take the extent of what is left.
    """
    if bg is None:
        bg = np.median(img.reshape(-1, 3), axis=0)
    m = np.abs(img - bg).max(axis=2) > 0.10
    if not m.any():
        return None
    ys, xs = np.where(m)
    return (int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1))


def box_delta(p, q):
    """How far apart two ink boxes are, in px: the largest of the edge and size differences."""
    if p is None or q is None:
        return None if (p is None and q is None) else 9999.0
    return float(max(abs(p[0] - q[0]), abs(p[1] - q[1]), abs(p[2] - q[2]), abs(p[3] - q[3])))


def vis_cov(a, b):
    """Fraction of pixels that VISIBLY differ. Mean abs diff can hide a small hard shift under a
    big identical background; this cannot."""
    n = min(a.shape[0], b.shape[0]), min(a.shape[1], b.shape[1])
    return float((np.abs(a[:n[0], :n[1]] - b[:n[0], :n[1]]).max(axis=2) > 0.06).mean())


def audit_effect(page, box, fx, pair, save_dir=None):
    page.evaluate("([id,p]) => mountFx(id,p)", [fx["id"], pair])

    def at(t):
        page.evaluate("t => setFrame(t)", t)
        return grab(page, box), page.evaluate("() => inkBox()")

    canon_a, ink_a = at(0.0)
    canon_b, ink_b = at(1.0)
    base_bright = float((luma(canon_a) > 0.75).mean())
    # The GROUND the transition is running on, i.e. the stage colour with the text ignored. A
    # "flash" is this changing, not the frame being bright: on the rater's white stage the
    # canonical frame is already 97% near-white, so an absolute brightness test flags all 96
    # effects and means nothing. Comparing each frame's ground against the canonical frame's
    # ground asks the question Tom actually asked — does the background change colour?
    base_ground = np.median(canon_a.reshape(-1, 3), axis=0)

    rec = {"id": fx["id"], "family": fx.get("family", ""), "pair": pair,
           "canonInkA": ink_a, "canonInkB": ink_b, "frames": {}}

    worst_flash, worst_flash_t = base_bright, None
    worst_shift, worst_shift_t = 0.0, None
    blanks = []
    for t in FRAMES:
        img, ink = at(t)
        bright = float((luma(img) > 0.75).mean())
        shift = float(np.abs(np.median(img.reshape(-1, 3), axis=0) - base_ground).mean())
        if shift > worst_shift:
            worst_shift, worst_shift_t = shift, t
        cov = float((np.abs(img - canon_a).max(axis=2) > 0.06).mean())
        rec["frames"][str(t)] = {"bright": round(bright, 4), "box": ink_bbox(img)}
        if bright > worst_flash:
            worst_flash, worst_flash_t = bright, t
        # "nothing on screen": almost no ink AND the frame is nearly the empty stage
        if ink is None and bright < 0.005 and cov < 0.02:
            blanks.append(t)

    # THE SETTLE. The endpoint checks above compare t=0.008 and t=0.992 against canonical, which is
    # coarse enough to miss the thing a person notices most: the headline twitching into place as
    # the engine hands over. Tom caught two of those by eye that this audit had passed --
    # type-backspace sitting a pixel high, and the typewriter's letters keeping a per-letter
    # misalignment at rest. So measure the LAST effect frame against the canonical frame it is
    # replaced by, and separately ask whether shifting it a pixel would fit better: a real
    # positional error shows as a lower residual at +-1px, anti-aliasing does not.
    settle_e, _ = at(0.999)
    base = pix_diff(settle_e, canon_b)
    up = pix_diff(np.roll(settle_e, -1, axis=0), canon_b)
    dn = pix_diff(np.roll(settle_e, 1, axis=0), canon_b)
    rec["settle"] = round(base, 5)
    rec["settleShift"] = -1 if up < base * 0.9 else (1 if dn < base * 0.9 else 0)

    e0, ink_e0 = at(EARLY[1])
    e1, _ = at(EARLY[2])
    l0, ink_l0 = at(LATE[2])
    l1, _ = at(LATE[1])

    bb_a, bb_b = ink_bbox(canon_a), ink_bbox(canon_b)
    rec["startSnapPix"] = round(pix_diff(e0, canon_a), 5)
    rec["startSnapPix2"] = round(pix_diff(e1, canon_a), 5)
    rec["endSnapPix"] = round(pix_diff(l0, canon_b), 5)
    rec["endSnapPix2"] = round(pix_diff(l1, canon_b), 5)
    rec["startSnapCov"] = round(vis_cov(e0, canon_a), 5)
    rec["endSnapCov"] = round(vis_cov(l0, canon_b), 5)
    rec["startSnapBox"] = box_delta(ink_bbox(e0), bb_a)
    rec["endSnapBox"] = box_delta(ink_bbox(l0), bb_b)
    rec["canonBoxA"] = bb_a
    rec["canonBoxB"] = bb_b
    rec["flash"] = round(worst_flash, 4)
    rec["flashBase"] = round(base_bright, 4)
    rec["flashAt"] = worst_flash_t
    rec["groundShift"] = round(worst_shift, 4)
    rec["groundShiftAt"] = worst_shift_t
    rec["blankFrames"] = blanks
    rec["errors"] = page.evaluate("() => getErrors()")

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        for nm, arr in (("canonA", canon_a), ("t008", e0), ("t992", l0), ("canonB", canon_b)):
            Image.fromarray((arr * 255).astype("uint8")).save(
                os.path.join(save_dir, "%s_p%d_%s.png" % (fx["id"], pair, nm)))
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--effects", default="", help="comma-separated ids (default: all)")
    ap.add_argument("--pairs", default="0,12", help="comma-separated slogan-pair indices")
    ap.add_argument("--save-fails", action="store_true")
    ap.add_argument("--quick", action="store_true",
                    help="thin the mid-transition probes; for sweeping many slogan pairs")
    ap.add_argument("--width", type=int, default=0,
                    help="stage width in px (default: the harness's own 1100, i.e. the rater's)")
    ap.add_argument("--white", action="store_true",
                    help="stage on white, matching the rater's 'On white' toggle")
    ap.add_argument("--out", default=os.path.join(OUT, "report.json"))
    a = ap.parse_args()

    from playwright.sync_api import sync_playwright
    global FRAMES
    if a.quick:
        FRAMES = QUICK
    pairs = [int(x) for x in a.pairs.split(",") if x.strip() != ""]
    want = set(x.strip() for x in a.effects.split(",") if x.strip())

    os.makedirs(OUT, exist_ok=True)
    t_start = time.time()
    records = []
    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--force-device-scale-factor=1"])
        page = browser.new_page(viewport={"width": 1400, "height": 520})
        page.goto(URL, wait_until="load")
        effects = page.evaluate("() => window.__ready")
        if a.width:
            got = page.evaluate("w => setWidth(w)", a.width)
            print("stage width set to %spx" % got)
        if a.white:
            page.evaluate("() => setWhite(true)")
            print("stage on WHITE (the rater's 'On white' toggle)")
        box = page.locator("#box")
        if want:
            effects = [e for e in effects if e["id"] in want]
        print("auditing %d effect(s) x %d pair(s) x %d frames"
              % (len(effects), len(pairs), len(FRAMES) + 2))
        for n, fx in enumerate(effects, 1):
            for pair in pairs:
                sd = os.path.join(OUT, "frames") if a.save_fails else None
                try:
                    records.append(audit_effect(page, box, fx, pair, sd))
                except Exception as exc:
                    records.append({"id": fx["id"], "pair": pair, "fatal": str(exc)})
            if n % 10 == 0 or n == len(effects):
                print("  %3d/%d  %.0fs" % (n, len(effects), time.time() - t_start))
        browser.close()

    with open(a.out, "w", encoding="utf-8") as f:
        json.dump({"frames": FRAMES, "records": records}, f, indent=1)
    print("wrote %s  (%d records, %.0fs)" % (a.out, len(records), time.time() - t_start))


if __name__ == "__main__":
    sys.exit(main())
