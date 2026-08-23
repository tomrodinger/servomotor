#!/usr/bin/env python3
"""Swap studio JPEGs for transparent cutouts and add cutout CSS."""
from pathlib import Path
import re

ROOT = Path(__file__).parent

MAP = [
    ("../one_motor.jpg", "../transparent/one_motor_transparent.png"),
    ("../one_motor_small.jpg", "../transparent/one_motor_transparent_small.png"),
    ("../M17_series_overview.jpg", "../transparent/M17_series_overview_transparent_small.png"),
    ("../kit_with_three_motors.jpg", "../transparent/kit_with_three_motors_transparent.png"),
    ("../kit_with_three_motors_small.jpg", "../transparent/kit_with_three_motors_transparent_small.png"),
    ("../motor_back.jpg", "../transparent/motor_back_transparent.png"),
    ("../motor_back_small.jpg", "../transparent/motor_back_transparent_small.png"),
    ("../adapter_and_wire.jpg", "../transparent/adapter_and_wire_transparent.png"),
    ("../adapter_and_wire_small.jpg", "../transparent/adapter_and_wire_transparent_small.png"),
]

CUTOUT_CSS = """
/* product cutouts — no white studio rectangle */
img[src*="transparent/"]{
  background:transparent !important;
  object-fit:contain !important;
  filter:drop-shadow(0 22px 40px rgba(0,0,0,.42));
}
img[src*="dimensions.png"]{
  background:#fff;
  border-radius:8px;
  padding:8px;
}
"""

# Pages whose studio photos sit on dark/colored grounds (not a white card).
# After swap, also loosen cover-crops that would amputate the product.
DARK_CONTAIN = {
    6, 7, 10, 11, 16, 17, 22, 23, 26, 28, 33, 34, 40
}


def add_cutout_class(html: str) -> str:
    def repl(m):
        tag = m.group(0)
        if "class=" in tag:
            return re.sub(r'class="([^"]*)"', r'class="\1 cutout"', tag, count=1)
        return tag.replace("<img ", '<img class="cutout" ', 1)

    return re.sub(
        r'<img\b[^>]*src="\.\./transparent/[^"]+"[^>]*>',
        repl,
        html,
    )


def main():
    for p in sorted(ROOT.glob("v*.html"), key=lambda x: int(x.stem[1:])):
        t = p.read_text(encoding="utf-8")
        orig = t
        for old, new in MAP:
            t = t.replace(old, new)
        t = t.replace('style="background:#eee"', "")
        t = t.replace("background:#eee;", "")
        t = t.replace("background:#eee", "")
        if "/* product cutouts" not in t:
            t = t.replace("<style>", "<style>\n" + CUTOUT_CSS, 1)
        t = add_cutout_class(t)
        n = int(p.stem[1:])
        if n in DARK_CONTAIN:
            # hero/bento cover on product images becomes contain via CSS above
            pass
        if t != orig:
            p.write_text(t, encoding="utf-8")
            print("updated", p.name)
        else:
            print("unchanged", p.name)


if __name__ == "__main__":
    main()
