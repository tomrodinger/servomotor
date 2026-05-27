"""
Language-level shims so the main modules can use ordinary Python syntax
without sprinkling ``is_micropython()`` checks. Anything that genuinely
differs between CPython and MicroPython at the language level (Enum,
typing) is centralised here.
"""

from .platform_detect import is_micropython


if is_micropython():
    class _EnumValue:
        def __init__(self, name, value):
            self.name = name
            self.value = value

        def __repr__(self):
            return self.name

    class _SimpleEnum:
        def __init__(self, name, members):
            self._name = name
            self._members = {}
            for key, value in members.items():
                enum_val = _EnumValue(key, value)
                setattr(self, key, enum_val)
                self._members[value] = enum_val

        def __call__(self, value):
            if value in self._members:
                return self._members[value]
            raise ValueError("'%s' is not a valid %s" % (value, self._name))

        def __iter__(self):
            return iter(self._members.values())

        def __repr__(self):
            return "<enum '%s'>" % self._name

    def Enum(name, members):
        """MicroPython stand-in for ``enum.Enum(name, members)``."""
        return _SimpleEnum(name, members)

    List = list
    Tuple = tuple
else:
    from enum import Enum  # noqa: F401
    from typing import List, Tuple  # noqa: F401
