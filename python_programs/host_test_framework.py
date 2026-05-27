"""
Minimal host-only test framework for the servomotor library.

These are *host-only* tests: they exercise the pure-software parts of the
library (command encoding, response decoding, CRC32, imports) with no motor
and no serial port connected. They form the regression safety net that the
hardware suite (run_all_tests.py) cannot provide -- e.g. they would have
caught the `int.to_bytes(..., signed)` regression that once broke the PC
build.

Design constraints:
  * No external dependencies (no pytest / unittest).
  * Runs unchanged on standard Python AND MicroPython, so the same checks
    can validate an ESP32-S3 / Pico 2 build later.
  * The final printed line contains the word 'PASSED' (success) or 'FAILED'
    (failure) so run_all_tests.py can grade the test, and the process exits
    0 / 1 to match.

Usage in a test file:

    from host_test_framework import TestRunner
    t = TestRunner("test_host_example")
    t.check_eq("one plus one", 1 + 1, 2)
    t.finish()        # prints a summary and exits the process
"""

import sys


def _hex(data):
    """Hex-encode a byte sequence (bytes.hex() is not on every MicroPython)."""
    return "".join("%02x" % b for b in bytearray(data))


class TestRunner:
    def __init__(self, name):
        self.name = name
        self.passed = 0
        self.failed = 0
        print("=== %s ===" % name)

    def _ok(self, label):
        self.passed += 1
        print("  ok   : " + label)

    def _fail(self, label, detail):
        self.failed += 1
        print("  FAIL : " + label)
        if detail:
            print("         " + detail)

    def check(self, label, condition):
        """Pass when condition is truthy."""
        if condition:
            self._ok(label)
        else:
            self._fail(label, "expected a truthy value")

    def check_eq(self, label, actual, expected):
        """Pass when actual == expected."""
        if actual == expected:
            self._ok(label)
        else:
            self._fail(label, "expected " + repr(expected) + ", got " + repr(actual))

    def check_bytes_eq(self, label, actual, expected):
        """Pass when two byte sequences are equal; reports hex on failure."""
        a = bytes(bytearray(actual))
        e = bytes(bytearray(expected))
        if a == e:
            self._ok(label)
        else:
            self._fail(label, "expected " + _hex(e) + ", got " + _hex(a))

    def check_raises(self, label, exc_type, func, *args, **kwargs):
        """Pass when calling func(*args, **kwargs) raises exc_type."""
        try:
            func(*args, **kwargs)
        except exc_type:
            self._ok(label)
            return
        except Exception as e:
            self._fail(label, "raised %s instead of %s"
                       % (type(e).__name__, exc_type.__name__))
            return
        self._fail(label, "did not raise " + exc_type.__name__)

    def finish(self):
        """Print the summary and exit the process (0 = all passed, 1 = failure)."""
        total = self.passed + self.failed
        print("")
        print("%s: %d/%d checks passed" % (self.name, self.passed, total))
        if self.failed == 0 and total > 0:
            print(self.name + " PASSED")
            sys.exit(0)
        else:
            reason = "no checks ran" if total == 0 else ("%d failed" % self.failed)
            print("%s FAILED (%s)" % (self.name, reason))
            sys.exit(1)


class FakeSerial:
    """
    Stand-in for a serial port used to exercise the communication module
    without hardware. Captures everything written, and replays a queue of
    bytes for reads. read() returns b'' once the queue is empty, which the
    communication layer treats as a timeout.
    """

    def __init__(self, rx=b""):
        self.written = bytearray()
        self._rx = bytearray(rx)

    def write(self, data):
        self.written += bytearray(data)

    def read(self, n):
        chunk = bytes(self._rx[:n])
        del self._rx[:n]
        return chunk

    def feed(self, data):
        """Queue more bytes to be returned by future read() calls."""
        self._rx += bytearray(data)

    def reset_input_buffer(self):
        self._rx = bytearray()

    def close(self):
        pass


def build_response_packet(body, crc32_enabled=True):
    """
    Build a raw device-response packet for feeding into FakeSerial.

    `body` is the bytes that follow the response-character address byte:
      * b''                  -> a success response (empty payload)
      * b'\\x00' + data       -> a normal data response (0x00 = no error)
      * bytes([code])         -> a fatal-error response (code != 0)

    The framing here mirrors the firmware: size byte(s) | response char |
    body | optional CRC32, with the extended (3-byte) size form when the
    packet exceeds 127 bytes.
    """
    from servomotor.communication import (
        calculate_crc32, encode_first_byte,
        RESPONSE_CHARACTER_CRC32_ENABLED, RESPONSE_CHARACTER_CRC32_DISABLED,
    )

    response_char = (RESPONSE_CHARACTER_CRC32_ENABLED if crc32_enabled
                     else RESPONSE_CHARACTER_CRC32_DISABLED)
    content = bytes([response_char]) + bytes(bytearray(body))

    packet_size = 1 + len(content)
    if crc32_enabled:
        packet_size += 4
    if packet_size > 127:
        packet_size += 2
        packet = bytes([encode_first_byte(127),
                        packet_size & 0xFF, (packet_size >> 8) & 0xFF]) + content
    else:
        packet = bytes([encode_first_byte(packet_size)]) + content

    if crc32_enabled:
        crc = calculate_crc32(packet)
        packet += bytes([crc & 0xFF, (crc >> 8) & 0xFF,
                         (crc >> 16) & 0xFF, (crc >> 24) & 0xFF])
    return packet
