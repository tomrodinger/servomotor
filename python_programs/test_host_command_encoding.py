#!/usr/bin/env python3
"""
Host-only test: command encoding.

No motor / serial port required. Validates that the library turns command
parameters into the exact bytes the protocol expects:
  * gather_inputs() -- packs each parameter data type (u8/u16/u32/i32/i64,
    u8_alias, buf10, list_2d) into the payload, little-endian, signed where
    appropriate. This is the path that the int.to_bytes(..., signed)
    regression broke.
  * send_command() -- wraps the payload in a full packet: size byte(s),
    address byte(s), command byte, optional CRC32, including extended
    addressing and the extended (3-byte) size form.

Accepts (and ignores) the -p / -P / -a arguments so run_all_tests.py can
invoke it alongside the hardware tests.
"""

from host_test_framework import TestRunner, FakeSerial, build_response_packet

from servomotor import communication
from servomotor.communication import (
    gather_inputs, send_command, encode_first_byte, decode_first_byte,
    is_valid_first_byte_format,
)

UNIQUE_ID = 0x99856389A2B46555
UNIQUE_ID_LE = bytes([0x55, 0x65, 0xB4, 0xA2, 0x89, 0x63, 0x85, 0x99])


def run_send(alias, command_id, payload, crc32_enabled=True, response_body=None):
    """Drive send_command with a FakeSerial; return the bytes it wrote."""
    fake = FakeSerial()
    if response_body is not None:
        fake.feed(build_response_packet(response_body, crc32_enabled=crc32_enabled))
    communication.ser = fake
    send_command(alias, command_id, payload, crc32_enabled=crc32_enabled, verbose=0)
    return bytes(fake.written)


def main():
    t = TestRunner("test_host_command_encoding")

    # ---- payload encoding (gather_inputs) -----------------------------
    # Command 0 "Disable MOSFETs": no inputs -> empty payload.
    t.check_bytes_eq("no-input command -> empty payload",
                     gather_inputs(0, [], verbose=0), b"")

    # Command 4 "Go to position": i32 position, u32 time.
    t.check_bytes_eq("go_to_position [1000, 5000] (i32,u32)",
                     gather_inputs(4, [1000, 5000], verbose=0),
                     bytes([0xE8, 0x03, 0x00, 0x00, 0x88, 0x13, 0x00, 0x00]))
    t.check_bytes_eq("go_to_position negative i32 [-1, 0]",
                     gather_inputs(4, [-1, 0], verbose=0),
                     bytes([0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00]))
    t.check_bytes_eq("go_to_position i32/u32 extremes",
                     gather_inputs(4, [-2147483648, 4294967295], verbose=0),
                     bytes([0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF]))

    # Command 28 "Set maximum motor current": two u16 values.
    t.check_bytes_eq("set_maximum_motor_current [390, 390] (u16,u16)",
                     gather_inputs(28, [390, 390], verbose=0),
                     bytes([0x86, 0x01, 0x86, 0x01]))

    # Command 43 "Set PID constants": three u32 values.
    t.check_bytes_eq("set_pid_constants [1, 256, 65536] (u32 x3)",
                     gather_inputs(43, [1, 256, 65536], verbose=0),
                     bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
                            0x00, 0x00, 0x01, 0x00]))

    # Command 30 "Set safety limits": two signed i64 values.
    t.check_bytes_eq("set_safety_limits [-1, 1] (i64,i64)",
                     gather_inputs(30, [-1, 1], verbose=0),
                     bytes([0xFF] * 8) + bytes([0x01] + [0x00] * 7))

    # Command 36 "Test mode": single u8.
    t.check_bytes_eq("test_mode [5] (u8)", gather_inputs(36, [5], verbose=0),
                     bytes([0x05]))

    # Command 21 "Set device alias": u8_alias accepts an int or a 1-char str.
    t.check_bytes_eq("set_device_alias int 88 (u8_alias)",
                     gather_inputs(21, [88], verbose=0), bytes([0x58]))
    t.check_bytes_eq("set_device_alias str 'X' (u8_alias)",
                     gather_inputs(21, ["X"], verbose=0), bytes([0x58]))

    # Command 31 "Ping": buf10 accepts bytes or a 10-char str.
    t.check_bytes_eq("ping buf10 from bytes",
                     gather_inputs(31, [b"0123456789"], verbose=0),
                     b"0123456789")
    t.check_bytes_eq("ping buf10 from str",
                     gather_inputs(31, ["ABCDEFGHIJ"], verbose=0),
                     b"ABCDEFGHIJ")

    # Command 29 "Multimove": u8, u32, list_2d. list_2d packs alternating
    # signed / unsigned 4-byte values.
    t.check_bytes_eq("multimove list_2d encoding",
                     gather_inputs(29, [2, 1000, "[[100, 200], [300, 400]]"],
                                   verbose=0),
                     bytes([0x02]) + bytes([0xE8, 0x03, 0x00, 0x00])
                     + bytes([0x64, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00,
                              0x2C, 0x01, 0x00, 0x00, 0x90, 0x01, 0x00, 0x00]))

    # ---- size-byte encoding (pure helpers) ----------------------------
    for size in (3, 7, 15, 64, 126):
        t.check_eq("encode/decode first byte round-trip (%d)" % size,
                   decode_first_byte(encode_first_byte(size)), size)
    t.check_eq("encode_first_byte sets the LSB",
               encode_first_byte(7) & 0x01, 0x01)
    t.check("encoded first byte is valid format",
            is_valid_first_byte_format(encode_first_byte(7)))
    t.check("even byte is an invalid first byte",
            not is_valid_first_byte_format(0x08))

    # ---- full packet framing (send_command) ---------------------------
    # Standard addressing, CRC32 enabled: size | alias | cmd | crc32.
    written = run_send(88, 0, b"", crc32_enabled=True, response_body=b"")
    t.check_eq("addressed packet size byte", written[0], encode_first_byte(7))
    t.check_eq("addressed packet length (with CRC32)", len(written), 7)
    t.check_eq("addressed packet alias byte", written[1], 88)
    t.check_eq("addressed packet command byte", written[2], 0)

    # Broadcast: addressed to ALL_ALIAS (255), expects no response.
    written = run_send(255, 0, b"", crc32_enabled=True)
    t.check_bytes_eq("broadcast packet header",
                     written[:3], bytes([encode_first_byte(7), 255, 0]))
    t.check_eq("broadcast packet length", len(written), 7)

    # CRC32 disabled: no trailing 4 bytes, smaller size byte.
    written = run_send(88, 0, b"", crc32_enabled=False, response_body=b"")
    t.check_bytes_eq("no-CRC packet is exactly size|alias|cmd",
                     written, bytes([encode_first_byte(3), 88, 0]))

    # Extended addressing: address byte 254 + 8-byte little-endian unique ID.
    written = run_send(UNIQUE_ID, 0, b"", crc32_enabled=True, response_body=b"")
    t.check_eq("extended-addressing size byte",
               written[0], encode_first_byte(15))
    t.check_eq("extended-addressing marker byte", written[1], 254)
    t.check_bytes_eq("extended-addressing unique ID (little-endian)",
                     written[2:10], UNIQUE_ID_LE)
    t.check_eq("extended-addressing command byte", written[10], 0)

    # Extended size: payload large enough to need the 3-byte size form.
    big_payload = bytes([0x77]) * 200
    written = run_send(255, 0, big_payload, crc32_enabled=False)
    t.check_eq("extended-size first byte is the 127 marker",
               written[0], encode_first_byte(127))
    # packet = 3 size bytes + 1 alias + 1 cmd + 200 payload = 205 bytes.
    t.check_eq("extended-size encodes total length 205",
               written[1] | (written[2] << 8), 205)
    t.check_eq("extended-size total bytes written", len(written), 205)
    t.check_bytes_eq("extended-size payload preserved", written[5:], big_payload)

    t.finish()


if __name__ == "__main__":
    main()
