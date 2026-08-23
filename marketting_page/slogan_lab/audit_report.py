#!/usr/bin/env python3
"""Turn a raw frame audit into a ranked defect list.

Thresholds are calibrated against effects that are known-good by eye (wipe-bar, light-specular
score ~0 on every endpoint metric), so anything above them is a real departure rather than
anti-aliasing noise.

  python3 audit_report.py [_audit/report_baseline.json] [--csv] [--ids-only]
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))

# A start/end frame may differ from canonical by a hair — sub-pixel layout, a 1px shadow. These
# are the points at which a human sees a jump rather than a fade.
BOX_PX = 6.0        # ink box moved/resized this many px at t=0.008 or t=0.992
COV = 0.060         # this fraction of the stage visibly differs from the canonical frame
FLASH_ABS = 0.30    # this fraction of the stage went near-white
FLASH_REL = 4.0     # ...or this many times the canonical frame's own bright coverage
GROUND_SHIFT = 0.25 # the stage's own colour moved this far from the canonical frame's


def defects(r):
    """Every way this one (effect, pair) departs from the contract, worst first."""
    out = []
    if r.get("fatal"):
        return [("FATAL", 100.0, r["fatal"])]
    if r.get("errors"):
        out.append(("THREW", 90.0, "; ".join(r["errors"])[:160]))

    for end, boxk, covk, pixk in (("START", "startSnapBox", "startSnapCov", "startSnapPix"),
                                  ("END", "endSnapBox", "endSnapCov", "endSnapPix")):
        box, cov = r.get(boxk), r.get(covk)
        if box is not None and box >= 9000:
            out.append((end + "-VANISH", 80.0, "nothing drawn at the endpoint frame"))
        elif box is not None and box > BOX_PX:
            out.append((end + "-SNAP", min(70.0, box), "ink box off by %.0fpx" % box))
        if cov is not None and cov > COV:
            out.append((end + "-DIFF", 40.0 + cov * 50, "%.1f%% of the stage differs" % (cov * 100)))

    # A flash is the GROUND changing colour, not the frame being bright. Absolute brightness is
    # meaningless on a light stage, where the canonical frame is near-white to begin with.
    gs = r.get("groundShift")
    if gs is not None:
        if gs > GROUND_SHIFT:
            out.append(("FLASH", 50.0 + gs * 40,
                        "the stage itself changes colour by %.2f at t=%s" % (gs, r.get("groundShiftAt"))))
    else:
        base, fl = r.get("flashBase", 0.0), r.get("flash", 0.0)   # older reports
        if fl > FLASH_ABS or (base > 0 and fl > base * FLASH_REL and fl > 0.12):
            out.append(("FLASH", 50.0 + fl * 40,
                        "%.0f%% of the stage near-white at t=%s (canon %.0f%%)"
                        % (fl * 100, r.get("flashAt"), base * 100)))

    bl = r.get("blankFrames") or []
    if len(bl) >= 3:
        out.append(("BLANK", 30.0 + len(bl), "headline absent for %d sampled frames" % len(bl)))
    return sorted(out, key=lambda d: -d[1])


def main():
    path = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") \
        else os.path.join(ROOT, "_audit", "report_baseline.json")
    data = json.load(open(path, encoding="utf-8"))
    by_id = {}
    for r in data["records"]:
        by_id.setdefault(r["id"], []).append(r)

    rows = []
    for fid, recs in by_id.items():
        alld = []
        for r in recs:
            for kind, sev, msg in defects(r):
                alld.append((kind, sev, msg, r["pair"]))
        if alld:
            alld.sort(key=lambda d: -d[1])
            rows.append((max(d[1] for d in alld), fid, recs[0].get("family", ""), alld))
    rows.sort(key=lambda x: -x[0])

    if "--ids-only" in sys.argv:
        print(",".join(r[1] for r in rows))
        return 0

    clean = len(by_id) - len(rows)
    print("%d effects audited: %s%d clean%s, %d with defects\n"
          % (len(by_id), "\033[32m", clean, "\033[0m", len(rows)))
    kinds = {}
    for _, _, _, ds in rows:
        for k, _s, _m, _p in ds:
            kinds[k] = kinds.get(k, 0) + 1
    print("by kind: " + ", ".join("%s=%d" % kv for kv in sorted(kinds.items(), key=lambda k: -k[1])))
    print()
    for sev, fid, fam, ds in rows:
        seen = set()
        uniq = [d for d in ds if not (d[0] in seen or seen.add(d[0]))]
        print("\033[33m%-24s\033[0m %-11s %s" % (fid, fam,
              "  ".join("%s(p%d): %s" % (d[0], d[3], d[2]) for d in uniq[:3])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
