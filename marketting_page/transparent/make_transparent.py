#!/usr/bin/env python3
"""Remove white studio backgrounds from Gearotons product photos -> transparent PNGs.
Uses rembg (isnet-general-use) with alpha matting for clean edges, then trims
to the subject bounding box (with margin) and also writes a web-sized version.
"""
import os
from rembg import remove, new_session
from PIL import Image

SRC = "/Users/tom/Documents/Move_the_Needle/Servomotor/marketting_page"
OUT = os.path.join(SRC, "transparent")
os.makedirs(OUT, exist_ok=True)

PHOTOS = [
    ("one_motor.jpg", "one_motor"),
    ("M17_series_overview.jpg", "M17_series_overview"),
    ("kit_with_three_motors.jpg", "kit_with_three_motors"),
    ("motor_back.jpg", "motor_back"),
    ("adapter_and_wire.jpg", "adapter_and_wire"),
]

session = new_session("isnet-general-use")

for fname, stem in PHOTOS:
    src_path = os.path.join(SRC, fname)
    img = Image.open(src_path).convert("RGBA")
    result = remove(
        img,
        session=session,
        alpha_matting=True,
        alpha_matting_foreground_threshold=240,
        alpha_matting_background_threshold=15,
        alpha_matting_erode_size=8,
    )
    # trim to subject + 2% margin
    bbox = result.getbbox()
    if bbox:
        w, h = result.size
        mx, my = int(w * 0.02), int(h * 0.02)
        bbox = (max(0, bbox[0]-mx), max(0, bbox[1]-my),
                min(w, bbox[2]+mx), min(h, bbox[3]+my))
        result = result.crop(bbox)
    full_path = os.path.join(OUT, f"{stem}_transparent.png")
    result.save(full_path)
    # web-size (max 1200px on the long edge)
    small = result.copy()
    small.thumbnail((1200, 1200), Image.LANCZOS)
    small.save(os.path.join(OUT, f"{stem}_transparent_small.png"))
    print(f"{fname}: {result.size}, full={os.path.getsize(full_path)//1024} KB")

print("DONE")
