#!/usr/bin/env python3
"""Rebuild the slogan engine that ships inside `final/index.html` from `slogan_lab/`.

The lab is where the effects are developed and rated; the page carries a frozen copy of the engine
and the effects inlined into a <script>. Without this script the two drift, and the page keeps
shipping whatever was pasted into it last.

It does two things:

  1. Rebuilds the inlined bundle from `slogan_lab/engine.js` plus the effect files, DROPPING the
     `FX.register({...})` block of every effect Tom discarded. Their helper functions stay — they
     are often shared with effects he kept — but the effect itself is not registered and its code
     path is dead, so it can never be picked.

  2. Rewrites the cycler's POOL to exactly the effects he kept.

The verdicts come from `ratings/transitions.json`, which the rater writes. Nothing here decides
what to keep; it only carries out what is recorded there.

  python3 build_page_engine.py            # rebuild final/index.html in place
  python3 build_page_engine.py --dry-run  # report what it would do
"""
import argparse
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
LAB = os.path.join(ROOT, "slogan_lab")
PAGE = os.path.join(ROOT, "final", "index.html")
RATINGS = os.path.join(ROOT, "ratings", "transitions.json")


def register_blocks(src):
    """Every `FX.register({ ... });` in a file, as (start, end, id), by brace matching.

    A regex cannot do this: the effect bodies contain braces, strings with braces, and regexes
    with braces. Counting depth while skipping strings and comments can.
    """
    out = []
    for m in re.finditer(r"FX\.register\(\s*\{", src):
        i = m.end() - 1                      # at the opening brace
        depth, j, n = 0, i, len(src)
        in_s, q, esc, in_c, in_lc = False, "", False, False, False
        while j < n:
            c = src[j]
            if in_lc:
                if c == "\n":
                    in_lc = False
            elif in_c:
                if c == "*" and src[j + 1:j + 2] == "/":
                    in_c = False
                    j += 1
            elif in_s:
                if esc:
                    esc = False
                elif c == "\\":
                    esc = True
                elif c == q:
                    in_s = False
            elif c == "/" and src[j + 1:j + 2] == "*":
                in_c = True
                j += 1
            elif c == "/" and src[j + 1:j + 2] == "/":
                in_lc = True
                j += 1
            elif c in "'\"`":
                in_s, q = True, c
            elif c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        body = src[i:j + 1]
        idm = re.search(r"\bid:\s*'([^']+)'", body)
        # swallow the trailing `);` and any newline, so removing a block leaves no orphan
        k = j + 1
        while k < n and src[k] in " \t":
            k += 1
        if src[k:k + 2] == ");":
            k += 2
        while k < n and src[k] in " \t":
            k += 1
        if k < n and src[k] == "\n":
            k += 1
        out.append((m.start(), k, idm.group(1) if idm else None))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    verdicts = json.load(open(RATINGS, encoding="utf-8"))
    keep = sorted(k for k, v in verdicts.items() if v.get("verdict") in ("keep", "fix"))
    drop = sorted(k for k, v in verdicts.items() if v.get("verdict") == "discard")
    print("ratings: %d keep+fix, %d discard" % (len(keep), len(drop)))

    files = [l.strip().strip('",') for l in open(os.path.join(LAB, "manifest.js"), encoding="utf-8")
             if l.strip().startswith('"effects/')]
    parts = [open(os.path.join(LAB, "engine.js"), encoding="utf-8").read()]
    removed, kept_ids = [], []
    for rel in files:
        src = open(os.path.join(LAB, rel), encoding="utf-8").read()
        blocks = register_blocks(src)
        cut = [b for b in blocks if b[2] in drop]
        for s, e, fid in sorted(cut, key=lambda b: -b[0]):
            src = src[:s] + src[e:]
            removed.append(fid)
        kept_ids += [b[2] for b in blocks if b[2] not in drop]
        parts.append(src)

    bundle = "\n\n".join(parts)
    print("bundle: %d effect files, %d registrations kept, %d removed (%d bytes)"
          % (len(files), len(kept_ids), len(removed), len(bundle)))

    missing = sorted(set(keep) - set(kept_ids))
    extra = sorted(set(kept_ids) - set(keep))
    if missing:
        print("  !! kept in the ratings but NOT in the bundle: %s" % missing)
        return 2
    if extra:
        print("  !! in the bundle but not rated keep: %s" % extra)
        return 2

    page = open(PAGE, encoding="utf-8").read()
    blocks = list(re.finditer(r"(<script[^>]*>)(.*?)(</script>)", page, re.S))
    # the engine block by its header comment; the cycler by the POOL it declares. Matching the
    # cycler on "SloganFX.mount" also matches the engine, which mentions it in an error string.
    eng = [b for b in blocks if "Slogan FX engine" in b.group(2)]
    cyc = [b for b in blocks if re.search(r"^  var POOL = \[", b.group(2), re.M)]
    if len(eng) != 1 or len(cyc) != 1:
        print("  !! expected exactly one engine block and one cycler block, found %d and %d"
              % (len(eng), len(cyc)))
        return 2

    pool_line = "  var POOL = %s;" % json.dumps(keep)
    pat = re.compile(r"^  var POOL = \[.*?\];", re.M | re.S)
    if not pat.search(cyc[0].group(2)):
        print("  !! could not find the POOL line in the cycler")
        return 2
    # A no-op substitution is the NORMAL case on a rebuild where the pool has not changed. Treating
    # "the text came back identical" as failure aborted the build and left the page carrying the
    # previous engine, while the deploy that followed happily shipped it.
    new_cyc = pat.sub(lambda m: pool_line, cyc[0].group(2), count=1)

    # replace by position, largest offset first, so the earlier offsets stay valid
    out = page
    for b, body in sorted(((cyc[0], new_cyc), (eng[0], "\n" + bundle + "\n")),
                          key=lambda x: -x[0].start(2)):
        out = out[:b.start(2)] + body + out[b.end(2):]

    print("  engine block %d -> %d bytes" % (len(eng[0].group(2)), len(bundle) + 2))
    print("  pool: %d effects" % len(keep))
    if a.dry_run:
        print("\ndry run: final/index.html not written")
        return 0
    open(PAGE, "w", encoding="utf-8").write(out)
    print("\nwrote %s" % PAGE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
