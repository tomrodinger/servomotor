#!/usr/bin/env python3
"""Cut the white studio background out of M17_series_overview.jpg -> transparent PNGs.

Unlike make_transparent.py this needs no rembg: the studio background is pure white and
uncoloured, so a border flood-fill through near-white pixels isolates it exactly, and a
luminance ramp inside the 1px transition band removes the white fringe. Keeps the silver
faceplates and white connectors that a plain luminance threshold would eat.
"""
import os
from collections import deque
import numpy as np
from PIL import Image, ImageFilter

SRC = "/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page"
OUT = os.path.join(SRC, "transparent")
NEAR_WHITE = 238          # min-channel value at/above which a pixel may be background
FRINGE_KNEE = 250.0       # start of the luminance ramp used inside the edge band
FRINGE_SPAN = 10.0

def cut(src_path, stem):
    im = Image.open(src_path).convert("RGB")
    mn = np.asarray(im).astype(np.int16).min(axis=2)
    H, W = mn.shape
    near = mn >= NEAR_WHITE

    bg = np.zeros_like(near, dtype=bool)
    dq = deque()
    for x in range(W):
        for y in (0, H - 1):
            if near[y, x] and not bg[y, x]:
                bg[y, x] = True; dq.append((y, x))
    for y in range(H):
        for x in (0, W - 1):
            if near[y, x] and not bg[y, x]:
                bg[y, x] = True; dq.append((y, x))
    while dq:
        y, x = dq.popleft()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < H and 0 <= nx < W and near[ny, nx] and not bg[ny, nx]:
                bg[ny, nx] = True; dq.append((ny, nx))

    hard = (~bg).astype(np.float32) * 255.0
    soft = np.asarray(Image.fromarray(hard.astype(np.uint8))
                      .filter(ImageFilter.GaussianBlur(1.0))).astype(np.float32)
    ramp = np.clip((FRINGE_KNEE - mn.astype(np.float32)) / FRINGE_SPAN, 0.0, 1.0) * 255.0
    band = (soft > 0) & (soft < 255)
    alpha = soft.copy()
    alpha[band] = np.minimum(soft[band], ramp[band])
    alpha[bg] = 0.0

    res = Image.fromarray(np.dstack([np.asarray(im), alpha.astype(np.uint8)]), "RGBA")
    bb = res.getbbox()
    w, h = res.size
    mx, my = int(w * 0.02), int(h * 0.02)
    res = res.crop((max(0, bb[0] - mx), max(0, bb[1] - my),
                    min(w, bb[2] + mx), min(h, bb[3] + my)))
    full = os.path.join(OUT, f"{stem}_transparent.png")
    res.save(full)
    small = res.copy()
    small.thumbnail((1200, 1200), Image.LANCZOS)
    small.save(os.path.join(OUT, f"{stem}_transparent_small.png"))
    print(f"{os.path.basename(src_path)}: {res.size}, full={os.path.getsize(full)//1024} KB")

if __name__ == "__main__":
    cut(os.path.join(SRC, "M17_series_overview.jpg"), "M17_series_overview")
    print("DONE")
