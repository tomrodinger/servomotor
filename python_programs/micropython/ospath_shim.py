"""Tiny os.path-compatible shim for MicroPython.

Install with:
    import os, ospath_shim
    os.path = ospath_shim
so test code can call os.path.dirname/basename/abspath/join.
"""

import os as _os


def join(*parts):
    out = ""
    for p in parts:
        if not p:
            continue
        if not out:
            out = p
        elif p.startswith("/"):
            out = p
        else:
            out = out.rstrip("/") + "/" + p
    return out


def dirname(p):
    if not p:
        return ""
    i = p.rfind("/")
    if i < 0:
        return ""
    if i == 0:
        return "/"
    return p[:i]


def basename(p):
    i = p.rfind("/")
    return p[i + 1:] if i >= 0 else p


def split(p):
    return dirname(p), basename(p)


def splitext(p):
    base = basename(p)
    i = base.rfind(".")
    if i <= 0:
        return p, ""
    head = p[: len(p) - len(base) + i]
    return head, base[i:]


def abspath(p):
    if p.startswith("/"):
        return p
    try:
        return getcwd() + "/" + p
    except Exception:
        return p


def getcwd():
    try:
        return _os.getcwd()
    except Exception:
        return ""


def exists(p):
    try:
        _os.stat(p)
        return True
    except OSError:
        return False


def isdir(p):
    try:
        return (_os.stat(p)[0] & 0x4000) != 0
    except OSError:
        return False


def isfile(p):
    try:
        return (_os.stat(p)[0] & 0x8000) != 0
    except OSError:
        return False
