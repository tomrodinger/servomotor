"""Cut the retouched rear-of-motor photo out of its white studio background.

The background is exactly 255,255,255 and there is no cast shadow worth keeping, so the
matte comes from a flood fill rather than a segmentation model: every white pixel reachable
from a frame corner is background, and white enclosed by the motor (the silkscreen text, the
connector, the screw heads) stays opaque. The boundary band gets a real fractional alpha
recovered from the compositing equation P = a*F + (1-a)*255, and the colour is
un-premultiplied so the edge is correct on any background rather than only on white.
"""
import os, numpy as np, cv2
from PIL import Image

S = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_label_work")
os.makedirs(S, exist_ok=True)
TH = 238                      # white-ness cut; also drops the very faint floor shadow

rgb = np.asarray(Image.open(S + "/retouched_full.png").convert("RGB")).astype(np.float32)
L = rgb.mean(2)
H, W = L.shape

SHADOW_Y  = 2850              # below here the frame holds only the button tab and the floor
SHADOW_LO = 110               # measured: the tab reads 35-116, the floor shadow 140-250,
SHADOW_HI = 150               # so the two separate cleanly with room to spare

def fill_holes(m):
    """anything the background cannot reach from a corner belongs to the object"""
    f = m.copy()
    cv2.floodFill(f, np.zeros((m.shape[0]+2, m.shape[1]+2), np.uint8), (0, 0), 1)
    return m | (f == 0).astype(np.uint8)

bg = (L > TH).astype(np.uint8)
ff, seen = bg.copy(), np.zeros((H+2, W+2), np.uint8)
for seed in [(0, 0), (W-1, 0), (0, H-1), (W-1, H-1)]:
    cv2.floodFill(ff, seen, seed, 2)
inside = (ff != 2).astype(np.uint8)
n0 = int(inside.sum())

# The chamfer on the motor's lower corners throws a specular highlight brighter than TH that
# reaches the outline, so the flood fill runs up it and slits the mask. Close the slits.
inside = fill_holes(cv2.morphologyEx(inside, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8)))
n1 = int(inside.sum())

# The unit was shot on a white floor and casts a soft contact shadow. It is not part of the
# product, and leaving it in would both smudge the page and shrink the motor inside its
# fixed-width slot. Below SHADOW_Y, take the silhouette from the motor's own darkness
# instead, then restore the white silkscreen ("Reset", "Test") the cut punches out.
bot = fill_holes((L[SHADOW_Y:] < SHADOW_LO).astype(np.uint8))
bot = fill_holes(cv2.morphologyEx(bot, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)))
inside[SHADOW_Y:] &= cv2.dilate(bot, np.ones((5, 5), np.uint8))
print(f"object pixels: {n0:,} -> {n1:,} after sealing slits -> {int(inside.sum()):,} after "
      f"dropping the floor shadow  ({inside.mean()*100:.1f}% of frame)")

k5 = np.ones((5, 5), np.uint8)
core = cv2.erode(inside, k5)                       # 2px inside the boundary
edge = cv2.dilate(inside, k5) & ~core              # the band where alpha is fractional

# local object luminance just inside the edge, for the alpha equation
Lm = np.where(core > 0, L, 255.0).astype(np.float32)
Fl = cv2.erode(Lm, np.ones((21, 21), np.float32))
Fl = np.minimum(Fl, 250.0)

a = np.clip((255.0 - L) / np.maximum(255.0 - Fl, 1.0), 0, 1)
alpha = np.where(core > 0, 1.0, np.where(edge > 0, a, 0.0)).astype(np.float32)
alpha = cv2.GaussianBlur(alpha, (0, 0), 0.6)
alpha = np.where(core > 0, 1.0, alpha)
# Below the cut the motor's edge meets grey floor, not white paper, so the white-background
# matte equation does not hold: ramp the alpha across the measured tab-to-shadow gap instead,
# holding the enclosed silkscreen fully opaque.
ramp = np.clip((SHADOW_HI - L[SHADOW_Y:]) / (SHADOW_HI - SHADOW_LO), 0, 1)
ramp = np.maximum(ramp, cv2.erode(inside[SHADOW_Y:], np.ones((5, 5), np.uint8)).astype(np.float32))
alpha[SHADOW_Y:] = ramp * cv2.dilate(inside[SHADOW_Y:], np.ones((3, 3), np.uint8))

# un-premultiply so the soft edge carries the motor's colour, not a white-blended one
den = np.maximum(alpha, 0.20)[..., None]
col = np.clip((rgb - (1.0 - alpha[..., None])*255.0) / den, 0, 255)
col = np.where((alpha[..., None] > 0.02) & (alpha[..., None] < 0.995), col, rgb)

# flood the fully transparent RGB with the nearest object colour, so nothing white can
# bleed back in when the image is resampled
solid = (alpha > 0.02).astype(np.uint8)
col = cv2.inpaint(col.astype(np.uint8), (1-solid)*255, 6, cv2.INPAINT_TELEA).astype(np.float32)

ys, xs = np.nonzero(alpha > 2/255)
x0, x1, y0, y1 = xs.min(), xs.max()+1, ys.min(), ys.max()+1
print(f"trim to {x1-x0} x {y1-y0} at +{x0}+{y0}")

full = np.dstack([col, alpha*255.0])[y0:y1, x0:x1]
full = np.clip(full, 0, 255).astype(np.uint8)
Image.fromarray(full, "RGBA").save(S + "/motor_back_transparent.png")

def downscale(img, width):
    """Lanczos on premultiplied alpha, then un-premultiply: no white halo."""
    a = img[..., 3:4].astype(np.float32)/255.0
    pre = np.dstack([img[..., :3].astype(np.float32)*a, a*255.0])
    h = max(1, int(round(img.shape[0]*width/img.shape[1])))
    sm = cv2.resize(pre, (width, h), interpolation=cv2.INTER_LANCZOS4)
    a2 = np.clip(sm[..., 3:4]/255.0, 0, 1)
    c2 = np.clip(sm[..., :3]/np.maximum(a2, 1e-4), 0, 255)
    return np.clip(np.dstack([c2, a2*255.0]), 0, 255).astype(np.uint8), h

# 2x the CSS slot. The slot goes 310px -> 350px because this photo includes some motor
# body, so the spec label renders ~13% smaller than the old edge-to-edge crop at the same
# width; 350px puts the label back at the size it reads today.
small, h = downscale(full, 700)
Image.fromarray(small, "RGBA").save(S + "/motor_back_transparent_small.png")
print(f"wrote master {full.shape[1]}x{full.shape[0]} and small {small.shape[1]}x{h}")
