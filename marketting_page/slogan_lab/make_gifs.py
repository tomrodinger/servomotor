#!/usr/bin/env python3
"""Export every registered slogan effect as an animated GIF.

The trick: filmstrip.html renders all N frames of one transition stacked vertically, so ONE headless
screenshot yields the whole animation. Slicing that tall PNG into equal bands gives frame-exact GIF
export at one browser launch per effect instead of one per frame (~1200 launches otherwise).

Usage:
  python3 make_gifs.py                 # every effect in the manifest
  python3 make_gifs.py ink-bleed slide-push
  python3 make_gifs.py --slogan 12 --frames 24
"""
import argparse
import os
import re
import subprocess
import sys

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SHOT = os.path.join(HERE, "..", "premium_minimal_variations", "_shot.sh")
OUT = os.path.join(HERE, "gifs")
BASE = "http://127.0.0.1:8912/slogan_lab/filmstrip.html"


def registered_ids():
    """Scrape ids straight out of the effect files — no browser needed."""
    ids = []
    edir = os.path.join(HERE, "effects")
    for fn in sorted(os.listdir(edir)):
        if not fn.endswith(".js"):
            continue
        src = open(os.path.join(edir, fn), encoding="utf-8").read()
        ids += re.findall(r"""\bid\s*:\s*['"]([a-z0-9][a-z0-9-]*)['"]""", src)
    seen, out = set(), []
    for i in ids:
        if i not in seen:
            seen.add(i)
            out.append(i)
    return out


def shoot(url, png, w, h):
    r = subprocess.run(["bash", SHOT, url, png, str(w), str(h)],
                       capture_output=True, text=True)
    return os.path.exists(png) and os.path.getsize(png) > 0


def export(fx_id, slogan, frames, w, h, ms):
    png = os.path.join(OUT, "_strip_%s.png" % fx_id)
    url = "%s?id=%s&i=%d&n=%d&w=%d&h=%d" % (BASE, fx_id, slogan, frames, w, h)
    if not shoot(url, png, w, frames * h):
        return None, "screenshot failed"

    sheet = Image.open(png).convert("RGB")
    if sheet.height < frames * h:
        return None, "strip short (%dpx, wanted %d)" % (sheet.height, frames * h)

    imgs = [sheet.crop((0, k * h, w, (k + 1) * h)) for k in range(frames)]
    # hold the settled frame so the loop reads as a pause, not a snap
    durations = [ms] * frames
    durations[-1] = max(ms * 6, 900)
    durations[0] = max(ms * 3, 500)

    gif = os.path.join(OUT, "%s.gif" % fx_id)
    # One shared palette derived from the whole strip, not a per-frame adaptive one: per-frame
    # palettes make the thin light-weight type shimmer between frames, and 128 colours visibly
    # washes out its antialiasing. 256 from a single quantiser keeps the type crisp and stable.
    master = Image.new("RGB", (w, h * len(imgs)))
    for k, im in enumerate(imgs):
        master.paste(im, (0, k * h))
    pal_img = master.quantize(colors=256, method=Image.MEDIANCUT)
    pal = [im.quantize(palette=pal_img, dither=Image.FLOYDSTEINBERG) for im in imgs]
    pal[0].save(gif, save_all=True, append_images=pal[1:], loop=0,
                duration=durations, optimize=True, disposal=2)
    os.remove(png)
    return gif, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ids", nargs="*")
    ap.add_argument("--slogan", type=int, default=9)
    ap.add_argument("--frames", type=int, default=18)
    ap.add_argument("--width", type=int, default=760)
    ap.add_argument("--height", type=int, default=200)
    ap.add_argument("--ms", type=int, default=70)
    a = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    ids = a.ids or registered_ids()
    print("exporting %d effect(s)" % len(ids))
    ok = fail = 0
    for n, fx in enumerate(ids, 1):
        gif, err = export(fx, a.slogan, a.frames, a.width, a.height, a.ms)
        if gif:
            ok += 1
            print("  [%d/%d] %-28s %6.1f KB" % (n, len(ids), fx, os.path.getsize(gif) / 1024))
        else:
            fail += 1
            print("  [%d/%d] %-28s FAILED: %s" % (n, len(ids), fx, err))
        sys.stdout.flush()
    # tell gallery.html what exists (no directory listing available to a static page)
    made = sorted(f[:-4] for f in os.listdir(OUT) if f.endswith(".gif"))
    meta = {}
    edir = os.path.join(HERE, "effects")
    for fn in sorted(os.listdir(edir)):
        if not fn.endswith(".js"):
            continue
        src = open(os.path.join(edir, fn), encoding="utf-8").read()
        for blk in re.split(r"\bFX\.register\(|\bSloganFX\.register\(", src)[1:]:
            gid = re.search(r"""\bid\s*:\s*['"]([^'"]+)['"]""", blk)
            if not gid:
                continue
            nm = re.search(r"""\bname\s*:\s*['"]([^'"]+)['"]""", blk)
            fam = re.search(r"""\bfamily\s*:\s*['"]([^'"]+)['"]""", blk)
            bl = re.search(r"""\bblurb\s*:\s*['"]([^'"]+)['"]""", blk)
            meta[gid.group(1)] = {
                "name": nm.group(1) if nm else gid.group(1),
                "family": fam.group(1) if fam else "Other",
                "blurb": bl.group(1) if bl else "",
            }
    with open(os.path.join(OUT, "list.js"), "w", encoding="utf-8") as fh:
        fh.write("/* generated by make_gifs.py */\nwindow.SLOGAN_GIFS = [\n")
        for gid in made:
            m = meta.get(gid, {"name": gid, "family": "Other", "blurb": ""})
            fh.write('  {id:%r, name:%r, family:%r, blurb:%r},\n'
                     % (gid, m["name"], m["family"], m["blurb"]))
        fh.write("];\n")
    print("done: %d ok, %d failed -> %s" % (ok, fail, OUT))
    print("gifs/list.js: %d entries" % len(made))


if __name__ == "__main__":
    main()
