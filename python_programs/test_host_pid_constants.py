#!/usr/bin/env python3
"""PID readback: compile real firmware handlers/framing, decode through M3.

No hardware required; needs a native C compiler. Like test_pid_differential.sh,
extract the firmware code verbatim so this tests the implementation rather than
a second implementation. Only hardware I/O, IRQs and coefficient recomputation
are stubbed here; the differential test covers reads with the real PID running.
"""

import os
from pathlib import Path
import re
import shlex
import struct
import subprocess
import tempfile
import zlib

from host_test_framework import FakeSerial, TestRunner
from servomotor import M3, communication, get_command_id

REPO = Path(__file__).resolve().parent.parent


def function(source, name):
    match = re.search(r"^\w+ " + name + r"\([^\n]*\)\n\{.*?^\}", source,
                      re.M | re.S)
    if not match:
        raise RuntimeError("Firmware function not found: " + name)
    return match.group(0)


def build_firmware_harness(directory):
    main = (REPO / "firmware/Src/main.c").read_text()
    motor = (REPO / "firmware/Src/motor_control.c").read_text()
    rs485 = (REPO / "common_source_files/RS485.c").read_text()
    case = re.search(r"^    case GET_PID_CONSTANTS_COMMAND:.*?(?=^    case )",
                     main, re.M | re.S)
    if not case:
        raise RuntimeError("Get PID constants handler not found")
    source = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "commands.h"
#define ERROR_COMMAND_SIZE_WRONG 51
#define ERROR_CODE_NO_ERROR 0
#define RESPONSE_CHARACTER_CRC32_ENABLED 253
#define RESPONSE_CHARACTER_CRC32_DISABLED 252
static uint32_t pid_p, pid_i, pid_d;
static uint16_t max_motor_pwm_voltage = 200, max_motor_regen_pwm_voltage = 200;
static int irq_enabled = 1, recomputations, completed, crc32_enabled;
static void __disable_irq(void) { assert(irq_enabled); irq_enabled = 0; }
static void __enable_irq(void) { assert(!irq_enabled); irq_enabled = 1; }
static void recompute_pid_parameters_and_set_pwm_voltage(uint16_t a, uint16_t b)
{ (void)a; (void)b; recomputations++; }
static void fatal_error(uint16_t error) { exit(error); }
static void rs485_done_with_this_packet(void) { completed++; }
static void if_fatal_error_then_dont_respond(void) { }
static uint8_t encode_first_byte(uint16_t size) { return (size << 1) | 1; }
static void rs485_transmit(void *data, uint16_t size)
{ assert(irq_enabled); assert(completed == 1); fwrite(data, 1, size, stdout); }
/* Software replacement for the STM32 CRC peripheral. */
static uint32_t calculate_crc32_buffer(void *data, uint32_t size) {
    uint32_t crc = UINT32_MAX;
    for (uint32_t n = 0; n < size; n++) {
        crc ^= ((uint8_t *)data)[n];
        for (int bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0);
    }
    return ~crc;
}
'''
    source += function(motor, "set_pid_constants") + "\n"
    source += function(motor, "get_pid_constants") + "\n"
    source += function(main, "check_payload_size_is_zero") + "\n"
    source += function(rs485, "rs485_finalize_and_transmit_packet") + "\n"
    source += "static void dispatch(uint8_t command, uint16_t payload_size, int is_broadcast) {\nswitch(command) {\n"
    source += case.group(0) + "default: abort();\n}\n}\n"
    source += r'''
int main(int argc, char **argv) {
    assert(argc == 7);
    uint32_t p = strtoull(argv[1], NULL, 0), i = strtoull(argv[2], NULL, 0);
    uint32_t d = strtoull(argv[3], NULL, 0);
    crc32_enabled = atoi(argv[4]);
    set_pid_constants(p, i, d);
    dispatch(GET_PID_CONSTANTS_COMMAND, atoi(argv[6]), atoi(argv[5]));
    assert(recomputations == 1); /* Getter must not reset controller state. */
    assert(irq_enabled && completed == 1);
    assert(pid_p == p && pid_i == i && pid_d == d);
    return 0;
}
'''
    path = Path(directory) / "pid_readback.c"
    executable = Path(directory) / "pid_readback"
    path.write_text(source)
    subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
        "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I" + str(REPO / "firmware/Src"), str(path), "-o", str(executable),
    ], check=True)
    return executable


def main():
    t = TestRunner("test_host_pid_constants")
    t.check_eq("catalog ID", get_command_id("Get PID constants"), 48)
    with tempfile.TemporaryDirectory(prefix="pid_readback_") as directory:
        firmware = build_firmware_harness(directory)

        def reply(gains, crc=True, broadcast=False, payload_size=0):
            return subprocess.run([str(firmware)] + [str(x) for x in gains] +
                                  [str(int(crc)), str(int(broadcast)),
                                   str(payload_size)], capture_output=True,
                                  check=False)

        gains_to_test = [
            (2000, 5, 175000),  # M17 startup defaults; D must remain unshifted.
            (0, 0, 0), (1, 256, 65536),
            (0x7fffffff, 0x80000000, 0xffffffff),
            (0xffffffff, 0xffffffff, 0xffffffff),
        ]
        for crc in (True, False):
            motor = M3("X", verbose=0)
            motor.set_crc32_enabled(crc)
            for gains in gains_to_test:
                result = reply(gains, crc)
                t.check_eq("firmware success", result.returncode, 0)
                payload = struct.pack("<III", *gains)
                expected = bytes([39 if crc else 31, 253 if crc else 252, 0]) + payload
                if crc:
                    expected += struct.pack("<I", zlib.crc32(expected))
                t.check_bytes_eq("firmware wire bytes CRC=%s gains=%s" % (crc, gains),
                                 result.stdout, expected)
                communication.ser = FakeSerial(result.stdout)
                t.check_eq("generated Python method decodes firmware reply",
                           motor.get_pid_constants(), list(gains))
                request = bytes([15 if crc else 7, ord("X"), 48])
                if crc:
                    request += struct.pack("<I", zlib.crc32(request))
                t.check_bytes_eq("getter sends fresh empty-payload command",
                                 communication.ser.written, request)

            result = reply(gains_to_test[0], crc, broadcast=True)
            t.check_eq("broadcast completes", result.returncode, 0)
            t.check_bytes_eq("broadcast sends no reply", result.stdout, b"")
            for broadcast in (False, True):
                result = reply(gains_to_test[0], crc, broadcast, payload_size=1)
                t.check_eq("non-empty payload raises existing size error",
                           result.returncode, 51)
                t.check_bytes_eq("invalid request sends no gains", result.stdout, b"")

        # A malformed or corrupt reply must not be presented as PID settings.
        for payload in (b"\0" * 11, b"\0" * 13):
            t.check_raises("wrong reply payload length", communication.CommunicationError,
                           communication.interpret_single_response, 48, payload, verbose=0)
        packet = bytearray(reply(gains_to_test[0]).stdout)
        packet[-1] ^= 1
        communication.ser = FakeSerial(packet)
        t.check_raises("bad CRC rejected", communication.CommunicationError,
                       M3("X", verbose=0).get_pid_constants)
    t.finish()


if __name__ == "__main__":
    main()
