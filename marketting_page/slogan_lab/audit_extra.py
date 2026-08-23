#!/usr/bin/env python3
"""Checks the frame audit cannot make, from three other angles.

`audit_frames.py` asks "does this effect start and end on the canonical text?". These ask
different questions, and each has caught things the endpoint check cannot see.

  INVARIANCE   Tom's actual requirement: "a given text will always appear the same no matter
               what". So render the canonical frame for the SAME slogan under all 96 effects and
               demand they are pixel-identical. If one effect leaves a style on the stage, or
               renders its own idea of the rest state, this is what catches it.

  DETERMINISM  The engine's contract is that a frame is a pure function of t. Render the same t
               twice, from a fresh mount, and demand the same pixels. Catches Math.random(),
               Date.now() and anything else that would make a screenshot, a GIF export and the
               live page disagree.

  CONTINUITY   A transition should not teleport. Walk t across the whole range and compare each
               frame with the one before; a step far larger than the effect's own typical step is
               a jump the eye reads as a glitch, even when both endpoints are perfect.

  python3 audit_extra.py [--effects a,b,c] [--pair 0] [--out _audit/extra.json]
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
URL = "http://localhost:8912/slogan_lab/_frameaudit.html"

STEPS = 40                     # continuity sweep resolution
JUMP_FACTOR = 6.0              # a step this many times the effect's median step is a teleport
JUMP_FLOOR = 0.045             # ...and it has to be visible at all to count


def grab(box):
    im = Image.open(io.BytesIO(box.screenshot(type="png"))).convert("RGB")
    return np.asarray(im, dtype=np.float32) / 255.0


def diff(a, b):
    n = min(a.shape[0], b.shape[0]), min(a.shape[1], b.shape[1])
    return float(np.abs(a[:n[0], :n[1]] - b[:n[0], :n[1]]).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--effects", default="")
    ap.add_argument("--pair", type=int, default=0)
    ap.add_argument("--out", default=os.path.join(ROOT, "_audit", "extra.json"))
    a = ap.parse_args()

    from playwright.sync_api import sync_playwright
    want = set(x.strip() for x in a.effects.split(",") if x.strip())
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    t0 = time.time()

    out = []
    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--force-device-scale-factor=1"])
        page = browser.new_page(viewport={"width": 1240, "height": 520})
        page.goto(URL, wait_until="load")
        effects = page.evaluate("() => window.__ready")
        if want:
            effects = [e for e in effects if e["id"] in want]
        box = page.locator("#box")

        ref = None                       # the canonical frame every effect must agree with
        for n, fx in enumerate(effects, 1):
            page.evaluate("([i,p]) => mountFx(i,p)", [fx["id"], a.pair])
            page.evaluate("t => setFrame(t)", 0.0)
            canon = grab(box)
            if ref is None:
                ref = canon
                inv = 0.0
            else:
                inv = diff(canon, ref)

            # determinism: same t, fresh mount, same pixels?
            page.evaluate("t => setFrame(t)", 0.37)
            d1 = grab(box)
            page.evaluate("([i,p]) => mountFx(i,p)", [fx["id"], a.pair])
            page.evaluate("t => setFrame(t)", 0.37)
            d2 = grab(box)
            det = diff(d1, d2)

            # continuity
            prev, steps = None, []
            for k in range(STEPS + 1):
                t = k / float(STEPS)
                page.evaluate("tt => setFrame(tt)", t)
                img = grab(box)
                if prev is not None:
                    steps.append((round(t, 3), diff(prev, img)))
                prev = img
            vals = sorted(s[1] for s in steps)
            med = vals[len(vals) // 2] if vals else 0.0
            jumps = [(t, round(v, 4)) for t, v in steps
                     if v > JUMP_FLOOR and med > 0 and v > med * JUMP_FACTOR]

            out.append({
                "id": fx["id"], "family": fx.get("family", ""),
                "canonInvariance": round(inv, 6),
                "determinism": round(det, 6),
                "medianStep": round(med, 5),
                "maxStep": round(vals[-1], 5) if vals else 0.0,
                "jumps": jumps,
                "errors": page.evaluate("() => getErrors()"),
            })
            if n % 10 == 0 or n == len(effects):
                print("  %3d/%d  %.0fs" % (n, len(effects), time.time() - t0))
        browser.close()

    with open(a.out, "w", encoding="utf-8") as f:
        json.dump({"pair": a.pair, "records": out}, f, indent=1)

    inv_bad = [r for r in out if r["canonInvariance"] > 0.0008]
    det_bad = [r for r in out if r["determinism"] > 0.0008]
    jump_bad = [r for r in out if r["jumps"]]
    err_bad = [r for r in out if r["errors"]]
    print("\n%d effects, pair %d" % (len(out), a.pair))
    print("  canonical NOT invariant : %d  %s" % (len(inv_bad), [r["id"] for r in inv_bad][:8]))
    print("  non-deterministic       : %d  %s" % (len(det_bad), [r["id"] for r in det_bad][:8]))
    print("  discontinuous           : %d  %s" % (len(jump_bad), [r["id"] for r in jump_bad][:8]))
    print("  threw                   : %d  %s" % (len(err_bad), [r["id"] for r in err_bad][:8]))
    print("wrote %s (%.0fs)" % (a.out, time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
