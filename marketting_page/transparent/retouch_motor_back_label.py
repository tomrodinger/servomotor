"""Retouch the M17-60 rear-label photo.

  Model:   M3-60   -> M17-60
  Voltage: 12-30 V -> 12-24 V
  rotated connector legend: +12-30V -> +12-24V

No synthetic type is drawn. Every replacement glyph is transplanted from elsewhere in
this same photograph, so typeface, print texture, focus and silkscreen sheen match by
construction. Placement comes from a least-squares fit of Arial Bold metrics to the
glyphs already on the label, so new strings land on the label's real baseline and
advance grid, including its ~1.4 deg in-plane tilt.
"""
import os, numpy as np, cv2
from PIL import Image, ImageFont, ImageDraw

SRC = os.path.expanduser("~/Documents/Move_the_Needle/Servomotor/Pictures/servomotor3/M17-60 motor/微信图片_20250714154555.jpg")
S   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_label_work")
os.makedirs(S, exist_ok=True)

img = np.asarray(Image.open(SRC).convert("RGB")).astype(np.float32)
out = img.copy()

FONT = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial Bold.ttf", 200)
_ink = {}
def ink(ch):
    if ch == ' ':
        return (0, 0, 0, 0, FONT.getlength(ch))
    if ch not in _ink:
        im = Image.new("L", (700, 500), 0)
        ImageDraw.Draw(im).text((200, 150), ch, font=FONT, fill=255)
        a = np.asarray(im); ys, xs = np.nonzero(a > 128)
        _ink[ch] = (xs.min()-200, xs.max()+1-200, ys.min()-150, ys.max()+1-150, FONT.getlength(ch))
    return _ink[ch]

def layout(s):
    pen, res = 0.0, []
    for ch in s:
        x0, x1, y0, y1, adv = ink(ch)
        res.append(dict(ch=ch, ix0=pen+x0, ix1=pen+x1, iy0=y0, iy1=y1))
        pen += adv
    return res

def plate_for(box, thresh=105, dil=13, rad=12, blur=9):
    x0, y0, x1, y1 = box
    sub = img[y0:y1, x0:x1]
    m = cv2.dilate(((sub.mean(2) > thresh)).astype(np.uint8), np.ones((dil, dil), np.uint8))
    pl = cv2.inpaint(np.clip(sub, 0, 255).astype(np.uint8), m, rad, cv2.INPAINT_TELEA).astype(np.float32)
    return pl, cv2.GaussianBlur(pl, (0, 0), blur)

def feather(m, r): return np.clip(cv2.GaussianBlur(m.astype(np.float32), (0, 0), r), 0, 1)

class Region:
    def __init__(self, box):
        self.box = box
        self.plate, self.field = plate_for(box)
        x0, y0, x1, y1 = box
        self.buf = img[y0:y1, x0:x1].copy()
    def erase(self, gbox, pad=4, soft=2.5):
        bx, by = self.box[0], self.box[1]
        gx0, gy0, gx1, gy1 = gbox
        m = np.zeros(self.buf.shape[:2], np.float32)
        m[gy0-by-pad:gy1-by+pad, gx0-bx-pad:gx1-bx+pad] = 1.0
        m = feather(m, soft)[..., None]
        self.buf = self.buf*(1-m) + self.plate*m
    def commit(self):
        x0, y0, x1, y1 = self.box
        out[y0:y1, x0:x1] = self.buf

def transplant(src_box, reg, dst_box, rot90=0, margin=10, soft=1.4, thresh=105, hp=3):
    """Move one glyph's pixels from src_box in the ORIGINAL photo into reg at dst_box.

    Matte model  P = a*F + (1-a)*Bsrc  gives the correct result over a new background as
    P + (1-a)*(Bdst - Bsrc), which preserves the print grain and the soft silkscreen edge.
    Any neighbouring glyph that falls inside the sampling margin is scrubbed back to the
    inpainted plate first, so only this glyph travels.
    """
    sx0, sy0, sx1, sy1 = src_box
    ex0, ey0, ex1, ey1 = sx0-margin, sy0-margin, sx1+margin, sy1+margin
    patch = img[ey0:ey1, ex0:ex1].copy()
    pl, sfield = plate_for((ex0-26, ey0-26, ex1+26, ey1+26))
    k = (slice(26, 26+(ey1-ey0)), slice(26, 26+(ex1-ex0)))
    bsrc, plp = sfield[k].copy(), pl[k].copy()

    core = np.zeros(patch.shape[:2], np.uint8)
    core[margin:margin+(sy1-sy0), margin:margin+(sx1-sx0)] = 1
    halo = cv2.dilate(core, np.ones((2*hp+1, 2*hp+1), np.uint8))   # the glyph's own soft edge
    # scrub any neighbouring glyph, and its halo, out of the sampling margin
    nb = (halo == 0) & (patch.mean(2) > np.minimum(bsrc.mean(2) + 10, thresh))
    patch[nb] = plp[nb]

    Lp, Lb = patch.mean(2), bsrc.mean(2)
    Lfg = np.percentile(Lp[core.astype(bool)], 92)
    alpha = np.clip((Lp - Lb) / max(Lfg - float(Lb.mean()), 1e-3), 0, 1)
    alpha[halo == 0] = 0.0
    keep = feather(cv2.dilate(halo, np.ones((9, 9), np.uint8)).astype(np.float32), 1.4)

    if rot90:
        patch, bsrc = np.rot90(patch, rot90), np.rot90(bsrc, rot90)
        alpha, keep = np.rot90(alpha, rot90), np.rot90(keep, rot90)
        cw, ch = (sy1-sy0), (sx1-sx0)
    else:
        cw, ch = (sx1-sx0), (sy1-sy0)
    patch, bsrc = np.ascontiguousarray(patch), np.ascontiguousarray(bsrc)
    alpha, keep = np.ascontiguousarray(alpha), np.ascontiguousarray(keep)

    tx0, ty0, tx1, ty1 = dst_box
    fx, fy = (tx1-tx0)/cw, (ty1-ty0)/ch
    ow, oh = int(round(patch.shape[1]*fx)), int(round(patch.shape[0]*fy))
    R = cv2.INTER_LANCZOS4
    patch = cv2.resize(patch, (ow, oh), interpolation=R)
    bsrc  = cv2.resize(bsrc,  (ow, oh), interpolation=R)
    alpha = np.clip(cv2.resize(alpha, (ow, oh), interpolation=R), 0, 1)
    keep  = np.clip(cv2.resize(keep,  (ow, oh), interpolation=R), 0, 1)

    mx, my = int(round(margin*fx)), int(round(margin*fy))
    dx, dy = int(round(tx0))-mx, int(round(ty0))-my
    lx, ly = dx-reg.box[0], dy-reg.box[1]
    assert lx >= 0 and ly >= 0 and ly+oh <= reg.buf.shape[0] and lx+ow <= reg.buf.shape[1], \
        f"target {dst_box} outside region {reg.box}"
    cur = reg.buf[ly:ly+oh, lx:lx+ow]
    # out = a*F + (1-a)*cur, with a*F = P - (1-a)*Bsrc. Exact, and it reduces to `cur`
    # plus this surface's own grain wherever a = 0 -- so a neighbouring glyph the patch
    # happens to overlap is left standing instead of being painted over with background.
    res = patch + (1-alpha[..., None])*(cur - bsrc)
    pm  = np.ones(cur.shape[:2], np.float32)
    pm[:2] = pm[-2:] = 0; pm[:, :2] = pm[:, -2:] = 0
    pm  = (feather(pm, soft)*keep)[..., None]
    reg.buf[ly:ly+oh, lx:lx+ow] = res*pm + cur*(1-pm)

def lsq(xs, ys):
    X = np.asarray(xs, float); Y = np.asarray(ys, float)
    A = np.vstack([np.ones_like(X), X]).T
    c, s = np.linalg.lstsq(A, Y, rcond=None)[0]
    r = Y - (c + s*X)
    return c, s, float(np.abs(r).max())

TILT = -0.025          # baseline rises to the right by 1.4 deg

# ------------------------------------------------------------------ measured ink boxes
# x0, x1, y0, y1 of every glyph, read off the original photo by thresholding at L>150.
M_MODEL = [(1211,1282,1694,1764), (1288,1333,1694,1762), (1338,1366,1728,1744),
           (1369,1413,1693,1760), (1418,1466,1692,1760)]                        # M 3 - 6 0
M_VOLT  = [(1210,1243,1803,1871), (1257,1304,1803,1870), (1308,1337,1836,1851),
           (1338,1383,1801,1868), (1389,1435,1800,1867), (1463,1523,1798,1864)] # 1 2 - 3 0 V
M_WEIGHT= [(1192,1242,2230,2292), (1245,1291,2229,2291), (1296,1341,2227,2291),
           (1370,1418,2241,2306)]                                               # 4 7 0 g
M_ROT   = [(2594,2635,1298,1342), (2559,2586,1291,1353), (2506,2545,1293,1355),
           (2479,2501,1327,1340), (2437,2475,1296,1358), (2393,2432,1297,1359),
           (2340,2391,1297,1360)]                                # + 1 2 - 3 0 V, as y0,y1,x0,x1

def fit_h(text, measured, skip=()):
    """photo_x = cx + sx*arial_ix ; photo_y = cy + sy*arial_iy + TILT*(x-xref)"""
    lay = [g for g in layout(text) if g['ch'] != ' ']
    assert len(lay) == len(measured), (text, len(lay), len(measured))
    xs, ys = [], []
    xref = measured[0][0]
    for i, (g, m) in enumerate(zip(lay, measured)):
        if i in skip: continue
        xs += [(g['ix0'], m[0]), (g['ix1'], m[1])]
        t = TILT*(m[0]-xref)
        ys += [(g['iy0'], m[2]-t), (g['iy1'], m[3]-t)]
    cx, sx, rx = lsq(*zip(*xs)); cy, sy, ry = lsq(*zip(*ys))
    print(f"  fit {text!r}: sx={sx:.4f} sy={sy:.4f}  max resid x={rx:.1f}px y={ry:.1f}px")
    return dict(cx=cx, sx=sx, cy=cy, sy=sy, xref=xref)

def fit_rot(text, measured):
    """advance runs up the screen: y = cy + sa*ix (sa<0); cap height runs right: x = cx + sc*iy"""
    lay = layout(text); ay, ax = [], []
    for g, m in zip(lay, measured):
        ay += [(g['ix0'], m[1]), (g['ix1'], m[0])]
        if g['ch'] not in '+-':          # '+' measures unreliably here, '-' is vertically centred
            ax += [(g['iy0'], m[2]), (g['iy1'], m[3])]
    cy, sa, ry = lsq(*zip(*ay)); cx, sc, rx = lsq(*zip(*ax))
    print(f"  fit {text!r}: advance={sa:.4f} cap={sc:.4f}  max resid y={ry:.1f}px x={rx:.1f}px")
    return dict(cy=cy, sa=sa, cx=cx, sc=sc)

def place_h(dst, glyphs, src_box, src_fit):
    """Target box for a glyph (or rigid run of glyphs) moved onto row `dst`.

    Size comes from the source's real ink box rescaled by the ratio of the two rows'
    fitted scales -- never from Arial's predicted extent, which is only accurate to a
    couple of px and would visibly shrink the glyph. Position comes from the fit:
    ink-left on the advance grid, ink-bottom on the baseline.
    """
    sw, sh = src_box[2]-src_box[0], src_box[3]-src_box[1]
    w = sw * dst['sx']/src_fit['sx']
    h = sh * dst['sy']/src_fit['sy']
    x0 = dst['cx'] + dst['sx']*min(g['ix0'] for g in glyphs)
    y1 = dst['cy'] + dst['sy']*max(g['iy1'] for g in glyphs) + TILT*(x0-dst['xref'])
    return (x0, y1-h, x0+w, y1)

def place_rot(dst, g, src_box, src_fit, from_horizontal):
    sw, sh = src_box[2]-src_box[0], src_box[3]-src_box[1]
    if from_horizontal:                       # cap height is the source's y, advance its x
        cap, adv, cap_s, adv_s = sh, sw, src_fit['sy'], src_fit['sx']
    else:                                     # already rotated: cap is x, advance is y
        cap, adv, cap_s, adv_s = sw, sh, src_fit['sc'], abs(src_fit['sa'])
    w = cap * dst['sc']/cap_s
    h = adv * abs(dst['sa'])/adv_s
    x1 = dst['cx'] + dst['sc']*g['iy1']       # baseline sits on the right
    y1 = dst['cy'] + dst['sa']*g['ix0']       # ink-left sits at the bottom
    return (x1-w, y1-h, x1, y1)

fW = fit_h("470 g", M_WEIGHT)                 # the row the new 4 and 7 come from

print("EDIT 1  Model:  M3-60 -> M17-60")
f1 = fit_h("M3-60", M_MODEL, skip={0})        # M measures 7px wider than Arial; fit the digits
f2 = fit_h("12-30 V", M_VOLT)                 # the row the new 1 comes from
L1 = layout("M17-60")                         # M 1 7 - 6 0
r1 = Region((1150, 1650, 1620, 1790))
r1.erase((1198, 1683, 1535, 1774), pad=0, soft=2.0)
transplant((1211,1694,1282,1764), r1, (1211,1694,1282,1764))                     # M, put back
transplant((1210,1803,1243,1871), r1, place_h(f1, [L1[1]], (1210,1803,1243,1871), f2))
transplant((1245,2229,1291,2291), r1, place_h(f1, [L1[2]], (1245,2229,1291,2291), fW))
transplant((1338,1692,1466,1760), r1, place_h(f1, L1[3:6], (1338,1692,1466,1760), f1))
r1.commit()

print("EDIT 2  Voltage: 12-30 V -> 12-24 V")
L2 = [g for g in layout("12-24 V") if g['ch'] != ' ']     # 1 2 - 2 4 V
r2 = Region((1150, 1788, 1620, 1900))
r2.erase((1198, 1793, 1535, 1882), pad=0, soft=2.0)
transplant((1210,1803,1337,1871), r2, (1210,1803,1337,1871))                     # "12-", put back
transplant((1463,1798,1523,1864), r2, (1463,1798,1523,1864))                     # "V",   put back
transplant((1257,1803,1304,1870), r2, place_h(f2, [L2[3]], (1257,1803,1304,1870), f2))
transplant((1192,2230,1242,2292), r2, place_h(f2, [L2[4]], (1192,2230,1242,2292), fW))
r2.commit()

print("EDIT 3  connector legend: +12-30V -> +12-24V")
f3 = fit_rot("+12-30V", M_ROT)
L3 = layout("+12-24V")                                    # + 1 2 - 2 4 V
r3 = Region((1230, 2320, 1430, 2660))
r3.erase((1286, 2333, 1368, 2477), pad=0, soft=2.0)
transplant((1297,2340,1360,2391), r3, (1297,2340,1360,2391))                     # V,  put back
transplant((1327,2479,1340,2501), r3, (1327,2479,1340,2501))                     # '-', put back
transplant((1293,2506,1355,2545), r3, place_rot(f3, L3[4], (1293,2506,1355,2545), f3, False))
transplant((1192,2230,1242,2292), r3, place_rot(f3, L3[5], (1192,2230,1242,2292), fW, True),
           rot90=1)
r3.commit()

Image.fromarray(np.clip(out, 0, 255).astype(np.uint8)).save(f"{S}/retouched_full.png")
print("wrote retouched_full.png")
