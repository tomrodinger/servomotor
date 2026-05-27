"""
Wrapper module that exposes everything from MicroPython's built-in
`random` plus the CPython-only `choices()` helper. Install via
`sys.modules['random'] = mp_random_shim` BEFORE any test imports random,
so a test's `import random` resolves to this wrapper instead.
"""

import random as _real


# Copy public symbols off the real module so attribute access is fast.
for _name in dir(_real):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_real, _name)


def choices(population, weights=None, cum_weights=None, k=1):
    """Uniform-only random.choices() (weights ignored — sufficient for the
    servomotor tests, which only use this to generate random payloads)."""
    return [_real.choice(population) for _ in range(k)]


_real_getrandbits = _real.getrandbits


def getrandbits(n):
    """MicroPython caps at 32 bits per call; chain 32-bit draws for larger n."""
    if n <= 32:
        return _real_getrandbits(n)
    out = 0
    shift = 0
    remaining = n
    while remaining > 0:
        chunk = 32 if remaining > 32 else remaining
        out |= _real_getrandbits(chunk) << shift
        shift += chunk
        remaining -= chunk
    return out
