"""
Wrapper that re-exports MicroPython's `collections` plus a `defaultdict`
implementation (which MP 1.28 omits). Install via
`sys.modules['collections'] = mp_collections_shim` before tests run.
"""

import collections as _real


# Re-export real symbols so other code (e.g. argparse's namedtuple) keeps working.
for _name in dir(_real):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_real, _name)


class defaultdict(dict):
    """Minimal CPython-compatible defaultdict."""

    def __init__(self, default_factory=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.default_factory = default_factory

    def __missing__(self, key):
        if self.default_factory is None:
            raise KeyError(key)
        value = self.default_factory()
        self[key] = value
        return value

    def __repr__(self):
        return "defaultdict(%r, %s)" % (self.default_factory, dict.__repr__(self))
