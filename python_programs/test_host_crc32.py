#!/usr/bin/env python3
"""
Host-only test: CRC32 implementation.

No motor / serial port required. Validates:
  * servomotor.calculate_crc32 against published CRC-32 (ISO-HDLC) check
    vectors,
  * the pure-Python MicroPython implementation (_micropython_crc32) against
    the same vectors -- this is the code path that actually runs on the
    ESP32-S3 / Pico 2, so it must be verified on the host,
  * that the three CRC32 entry points in the library agree with each other.

Accepts (and ignores) the -p / -P / -a arguments so run_all_tests.py can
invoke it alongside the hardware tests.
"""

from host_test_framework import TestRunner

from servomotor.platform_utils import calculate_crc32, _micropython_crc32
from servomotor import communication
from servomotor import calculate_crc32 as package_crc32


# (input bytes, expected CRC-32) -- standard CRC-32/ISO-HDLC check values.
KNOWN_VECTORS = [
    (b"", 0x00000000),
    (b"123456789", 0xCBF43926),
    (b"The quick brown fox jumps over the lazy dog", 0x414FA339),
    (b"\x00" * 32, 0x190A55AD),
    (bytes(range(256)), 0x29058C73),
]


def main():
    t = TestRunner("test_host_crc32")

    # Known check vectors against the standard (zlib-backed) implementation.
    for data, expected in KNOWN_VECTORS:
        t.check_eq("calculate_crc32(%d bytes)" % len(data),
                   calculate_crc32(data), expected)

    # The pure-Python MicroPython implementation must produce identical
    # results -- this is the code that runs on the microcontroller.
    for data, expected in KNOWN_VECTORS:
        t.check_eq("_micropython_crc32(%d bytes)" % len(data),
                   _micropython_crc32(data), expected)

    # All three CRC32 entry points exposed by the library must agree.
    for data, _ in KNOWN_VECTORS:
        ref = calculate_crc32(data)
        t.check_eq("package calculate_crc32 agrees (%d bytes)" % len(data),
                   package_crc32(data), ref)
        t.check_eq("communication.calculate_crc32 agrees (%d bytes)" % len(data),
                   communication.calculate_crc32(data), ref)

    # Result must always be a non-negative 32-bit unsigned integer.
    for data, _ in KNOWN_VECTORS:
        value = calculate_crc32(data)
        t.check("crc32(%d bytes) is unsigned 32-bit" % len(data),
                0 <= value <= 0xFFFFFFFF)

    # bytearray and bytes inputs must hash identically.
    t.check_eq("bytearray input matches bytes input",
               calculate_crc32(bytearray(b"123456789")),
               calculate_crc32(b"123456789"))

    t.finish()


if __name__ == "__main__":
    main()
