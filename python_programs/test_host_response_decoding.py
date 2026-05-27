#!/usr/bin/env python3
"""
Host-only test: response decoding.

No motor / serial port required. Validates that the library turns raw
device bytes back into Python values:
  * get_response() -- unframes a packet read from the serial port: size
    byte(s), response character, CRC32 verification, the leading error-code
    byte, and the extended (3-byte) size form. Fatal-error and corruption
    paths are checked too.
  * interpret_single_response() -- decodes a payload into typed values
    (signed/unsigned integers of every width, version numbers, strings,
    unique IDs), and rejects payloads that are too short or too long.

Accepts (and ignores) the -p / -P / -a arguments so run_all_tests.py can
invoke it alongside the hardware tests.
"""

from host_test_framework import TestRunner, FakeSerial, build_response_packet

from servomotor import communication
from servomotor.communication import (
    get_response, interpret_single_response, FatalError, CommunicationError,
    TimeoutError as CommTimeoutError,
)

UNIQUE_ID = 0x99856389A2B46555


def le(value, length):
    """Little-endian unsigned encoding helper."""
    return bytes([(value >> (8 * i)) & 0xFF for i in range(length)])


def read_response(body, crc32_enabled=True):
    """Feed a built response packet through get_response; return the payload."""
    communication.ser = FakeSerial(build_response_packet(body, crc32_enabled))
    return get_response(crc32_enabled=crc32_enabled, verbose=0)


def main():
    t = TestRunner("test_host_response_decoding")

    # ---- interpret_single_response: integer widths --------------------
    # Command 42 "Get temperature": signed i16.
    t.check_eq("decode i16 +300",
               interpret_single_response(42, le(300, 2), verbose=0), [300])
    t.check_eq("decode i16 -300",
               interpret_single_response(42, bytes([0xD4, 0xFE]), verbose=0),
               [-300])
    t.check_eq("decode i16 max 32767",
               interpret_single_response(42, bytes([0xFF, 0x7F]), verbose=0),
               [32767])
    t.check_eq("decode i16 min -32768",
               interpret_single_response(42, bytes([0x00, 0x80]), verbose=0),
               [-32768])

    # Command 34 "Get position": signed i64.
    t.check_eq("decode i64 +12345",
               interpret_single_response(34, le(12345, 8), verbose=0), [12345])
    t.check_eq("decode i64 -1",
               interpret_single_response(34, bytes([0xFF] * 8), verbose=0),
               [-1])

    # Command 9 "Get current time": unsigned u64.
    t.check_eq("decode u64 large value",
               interpret_single_response(9, le(0xFEDCBA9876543210, 8),
                                         verbose=0),
               [0xFEDCBA9876543210])

    # Command 11 "Get n queued items": unsigned u8.
    t.check_eq("decode u8 200",
               interpret_single_response(11, bytes([200]), verbose=0), [200])

    # ---- interpret_single_response: multi-field & non-integer ---------
    # Command 16 "Get status": u16 then u8.
    t.check_eq("decode status (u16, u8)",
               interpret_single_response(16, bytes([0x34, 0x12, 0x05]),
                                         verbose=0),
               [0x1234, 5])

    # Command 25 "Get firmware version": u32_version_number then u8.
    t.check_eq("decode firmware version (u32_version_number, u8)",
               interpret_single_response(25, bytes([1, 2, 3, 0, 7]), verbose=0),
               [[1, 2, 3, 0], 7])

    # Command 20 "Detect devices": u64_unique_id then u8_alias.
    t.check_eq("decode detect-devices (unique ID, alias)",
               interpret_single_response(20, le(UNIQUE_ID, 8) + bytes([88]),
                                         verbose=0),
               [UNIQUE_ID, 88])

    # Command 24 "Get product description": null-terminated string.
    t.check_eq("decode null-terminated string",
               interpret_single_response(24, b"Hello\x00", verbose=0),
               ["Hello\x00"])

    # Command 31 "Ping": buf10 echo.
    t.check_eq("decode buf10 echo",
               interpret_single_response(31, b"0123456789", verbose=0),
               [b"0123456789"])

    # Command 22 "Get product info": string8, u8, u24_version, u32,
    # u64_unique_id, u32 -- 28 bytes total.
    product_info = (b"M3      " + bytes([1]) + bytes([10, 20, 30])
                    + le(0x12345678, 4) + le(UNIQUE_ID, 8) + le(100, 4))
    t.check_eq("decode product info (6 mixed fields)",
               interpret_single_response(22, product_info, verbose=0),
               ["M3      ", 1, [10, 20, 30], 0x12345678, UNIQUE_ID, 100])

    # ---- interpret_single_response: malformed payloads ----------------
    t.check_raises("short payload is rejected", CommunicationError,
                   interpret_single_response, 42, bytes([0x00]), verbose=0)
    t.check_raises("trailing extra bytes are rejected", CommunicationError,
                   interpret_single_response, 42, bytes([0x2C, 0x01, 0x99]),
                   verbose=0)

    # ---- get_response: framing & error handling -----------------------
    t.check_bytes_eq("get_response success (empty payload)",
                     read_response(b""), b"")

    # A normal data response carries a leading 0x00 (no-error) byte that
    # get_response strips before returning the data.
    t.check_bytes_eq("get_response strips the no-error byte",
                     read_response(b"\x00" + le(300, 2)), le(300, 2))

    # End-to-end: get_response output feeds straight into interpret.
    payload = read_response(b"\x00" + le(-1234 & 0xFFFF, 2))
    t.check_eq("get_response + interpret round-trip",
               interpret_single_response(42, payload, verbose=0), [-1234])

    # A non-zero error code must raise FatalError carrying that code.
    t.check_raises("fatal error code raises FatalError", FatalError,
                   read_response, bytes([47]))

    # CRC32-disabled responses decode through the no-CRC path.
    t.check_bytes_eq("get_response success without CRC32",
                     read_response(b"", crc32_enabled=False), b"")

    # Extended (3-byte) size form: a response longer than 127 bytes.
    big = bytes(range(256)) * 1  # 256 bytes of body data
    communication.ser = FakeSerial(
        build_response_packet(b"\x00" + big, crc32_enabled=True))
    t.check_bytes_eq("get_response decodes an extended-size packet",
                     get_response(crc32_enabled=True, verbose=0), big)

    # A corrupted CRC32 must be detected.
    bad = bytearray(build_response_packet(b"\x00" + le(42, 2),
                                          crc32_enabled=True))
    bad[-1] ^= 0xFF
    communication.ser = FakeSerial(bytes(bad))
    t.check_raises("corrupted CRC32 raises CommunicationError",
                   CommunicationError, get_response, crc32_enabled=True,
                   verbose=0)

    # A first byte without the LSB set is an invalid frame.
    communication.ser = FakeSerial(bytes([0x08, 0x00, 0x00]))
    t.check_raises("invalid first byte raises CommunicationError",
                   CommunicationError, get_response, crc32_enabled=True,
                   verbose=0)

    # No bytes available at all is a timeout.
    communication.ser = FakeSerial(b"")
    t.check_raises("empty serial read raises TimeoutError", CommTimeoutError,
                   get_response, crc32_enabled=True, verbose=0)

    t.finish()


if __name__ == "__main__":
    main()
