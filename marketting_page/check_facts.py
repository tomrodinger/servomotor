#!/usr/bin/env python3
"""Check the hand-authored marketing page against the sources of truth.

The page is hand-edited now, so nothing structurally stops it drifting away from the datasheet.
This reads the FACTS out of their source files (never hardcoded here — that would just create a
third copy to go stale) and reports anything the page contradicts or is missing.

It is deliberately narrow. It catches numbers and named facts. It cannot judge wording, tone or
whether a claim is fair. See MARKETING_PAGE_WORKFLOW.md §3 for what to do with a hit.

  python3 check_facts.py [page.html]        default: final/index.html
"""
import html
import os
import re
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(ROOT)
DS = os.path.join(REPO, "servomotor_datasheets")

GREEN, RED, YELLOW, DIM, OFF = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


def text_of(path):
    try:
        return open(path, encoding="utf-8").read()
    except Exception:
        return ""


def page_text(path):
    """Visible text of the page: tags stripped, entities decoded, whitespace flattened."""
    s = text_of(path)
    s = re.sub(r"<script\b.*?</script>", " ", s, flags=re.S | re.I)
    s = re.sub(r"<style\b.*?</style>", " ", s, flags=re.S | re.I)
    s = re.sub(r"<[^>]+>", " ", s)
    s = html.unescape(s)
    return re.sub(r"\s+", " ", s)


def specs_from_source():
    """Pull the spec tables straight out of the datasheet generator."""
    src = text_of(os.path.join(DS, "specs.py"))
    rows = {}
    for m in re.finditer(r"label_cell\('([^']+)'\)\s*,\s*((?:'[^']*'\s*,?\s*){2,5})", src):
        label = m.group(1)
        vals = re.findall(r"'([^']*)'", m.group(2))
        rows[label] = vals
    return rows


def units_from_source():
    import json
    p = os.path.join(REPO, "python_programs", "servomotor", "unit_conversions_M3.json")
    try:
        return list(json.load(open(p))["units"].keys())
    except Exception:
        return []


def norm(v):
    """Fold away pure TYPOGRAPHY so only real fact differences survive.

    The sources are written for a PDF generator and the page is written for the web, so the same
    fact legitimately appears as '42.2x42.2 mm' in one and '42.2 × 42.2 mm' in the other, or
    '0.65 N.m' vs '0.65 N·m'. Flagging those would train everyone to ignore this checker.
    """
    v = v.replace("N.m", "N·m")
    v = v.replace("–", "-").replace("—", "-").replace("−", "-")   # dashes
    v = v.replace("×", "x").replace("✕", "x")                      # multiplication sign
    v = v.replace("\u00a0", " ").replace("\u2011", "-")           # nbsp, non-breaking hyphen
    return re.sub(r"\s+", "", v).lower()


def variants(v):
    """Reasonable renderings of the same value that the page might legitimately use."""
    out = {norm(v)}
    out.add(norm(v.replace("N·m", "N.m")))
    out.add(norm(v.replace("-", "–")))
    m = re.match(r"^([\d.]+)\s*(.*)$", v.strip())
    if m:
        num, unit = m.groups()
        out.add(norm(num + unit))
        out.add(norm(num + " " + unit))
        if num.endswith(".0"):
            out.add(norm(num[:-2] + " " + unit))
    return {x for x in out if x}


def main():
    page = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "final", "index.html")
    if not os.path.exists(page):
        print("%sno such page: %s%s" % (RED, page, OFF))
        return 2
    txt = page_text(page)
    flat = norm(txt)
    problems, checked = [], 0

    print("checking %s%s%s against the sources\n" % (DIM, os.path.relpath(page, ROOT), OFF))

    # ---- 1. spec tables, straight from specs.py -------------------------------------------
    rows = specs_from_source()
    if not rows:
        problems.append(("specs.py", "could not parse any spec rows — has its format changed?"))
    missing_specs = []
    for label, vals in sorted(rows.items()):
        for v in vals:
            if not v.strip():
                continue
            checked += 1
            if not any(x in flat for x in variants(v)):
                missing_specs.append("%s = %s" % (label, v))
    if missing_specs:
        problems.append(("specs.py", "%d spec value(s) from the datasheet are not on the page: %s"
                         % (len(missing_specs), "; ".join(sorted(set(missing_specs))[:8]))))

    # ---- 2. model names -------------------------------------------------------------------
    models = sorted(set(re.findall(r"M17-\d{2}", text_of(os.path.join(DS, "specs.py")))))
    for m in models:
        checked += 1
        if norm(m) not in flat:
            problems.append(("specs.py", "model %s is missing from the page" % m))

    # ---- 3. superseded values that must NOT appear ----------------------------------------
    # these were correct before the four-motor update; if one is back, something regressed
    stale = {"38 W": "old M17-60 rated power", "32 W": "old M17-48 rated power",
             "25 W": "old M17-40 rated power", "59.8 mm": "old M17-60 height",
             "48.6 mm": "old M17-48 height", "41.6 mm": "old M17-40 height",
             "18.5 mm": "old M17-40 shaft length", "20.4 mm": "old shaft length"}
    for v, why in stale.items():
        if norm(v) in flat:
            problems.append(("REGRESSION", "page contains superseded value %r (%s)" % (v, why)))
        checked += 1

    # ---- 4. unit system -------------------------------------------------------------------
    for q in units_from_source():
        checked += 1
        if q.replace("_", "") not in flat.replace(" ", ""):
            problems.append(("unit_conversions_M3.json",
                             "unit quantity %r not mentioned on the page" % q))

    # ---- 5. LED behaviour: the green LED shows bus traffic, not the red ---------------------
    ind = text_of(os.path.join(DS, "indicators.py"))
    if "flickers with communication" in ind:
        checked += 1
        if re.search(r"red LED[^.]{0,90}(communication on the bus|shows? communication)", txt, re.I):
            problems.append(("indicators.py",
                             "page says the RED LED shows bus traffic; the source says the GREEN one does"))

    # ---- 6. protocol constants ------------------------------------------------------------
    comm = text_of(os.path.join(ROOT, "communication.txt"))
    for const in re.findall(r"\b(230400|64-bit|0-251|255)\b", comm):
        checked += 1
        if norm(const) not in flat:
            problems.append(("communication.txt", "protocol fact %r is not on the page" % const))

    # ---- report ---------------------------------------------------------------------------
    if not problems:
        print("%sOK%s  %d facts checked, page agrees with every source." % (GREEN, OFF, checked))
        return 0
    print("%s%d issue(s)%s  (%d facts checked)\n" % (RED, len(problems), OFF, checked))
    for src, msg in problems:
        colour = RED if src == "REGRESSION" else YELLOW
        print("  %s[%s]%s %s" % (colour, src, OFF, msg))
    print("\n%sPer MARKETING_PAGE_WORKFLOW.md section 3: a fact difference is Tom's call —"
          "\nchange both, change neither, or fix the source. Do not resolve it silently.%s" % (DIM, OFF))
    return 1


if __name__ == "__main__":
    sys.exit(main())
