#!/usr/bin/env python3
"""Test-only stdio bridge from browser QA to one real UART (no HTTP API).

Chromium's serial chooser cannot be selected through the available browser
automation interface. This bridge tests the production browser transaction and
stream code with real UART bytes; it does not test the native serial chooser.
"""
import argparse
from contextlib import redirect_stdout
import json
from pathlib import Path
import struct
import sys
import threading
import time
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python_programs'))
from servomotor.vendor import serial

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', required=True)
parser.add_argument('--uid', required=True)
parser.add_argument('--firmware-version', choices=('0.17.0.0', '0.17.0.2', '0.17.0.3', '0.17.0.4', '0.17.0.5', '0.17.0.6'), default='0.17.0.0')
args = parser.parse_args()
uid = int(args.uid, 16)
assert len(args.uid) == 16 and uid > 255
link = None
reading = False
reader = None
lock = threading.Lock()


def emit(message):
    with lock:
        print(json.dumps(message), flush=True)


def read_loop():
    while reading:
        try:
            data = link.read(max(1, link.in_waiting))
            if data:
                emit({'event': 'data', 'bytes': list(data)})
        except Exception as error:
            if reading:
                emit({'event': 'error', 'message': str(error)})
            break


def close():
    global link, reading
    reading = False
    if reader:
        reader.join(timeout=1)
    if link:
        link.close()
        link = None


def packet(command):
    data = b'\x1f\xfe' + struct.pack('<Q', uid) + bytes([command])
    # A unique-ID, no-payload request is 15 bytes including CRC.
    return data + struct.pack('<I', zlib.crc32(data))


try:
    for line in sys.stdin:
        message = json.loads(line)
        try:
            operation = message['operation']
            result = None
            if operation == 'prepare':
                assert link is None
                # Keep diagnostic prints out of the bridge's JSON channel.
                with redirect_stdout(sys.stderr):
                    import servomotor
                    servomotor.set_serial_port(args.port)
                    servomotor.open_serial_port()
                    try:
                        motor = servomotor.M3(uid, current_unit='internal_current_units', verbose=0)
                        info = motor.get_product_info()
                        assert info[0].strip() == 'M17' and info[1] == 3 and info[4] == uid
                        expected_version = [int(part) for part in args.firmware_version.split('.')][::-1]
                        assert motor.get_firmware_version() == [expected_version, 0]
                        assert motor.get_status() == [0, 0] and motor.get_n_queued_items() == 0
                        motor.system_reset()
                        time.sleep(1.5)
                        # Preserve the established raw limits: 50 internal
                        # units each, approximately 256 mA in this unit table.
                        motor.set_maximum_motor_current(50, 50)
                    finally:
                        servomotor.close_serial_port()
            elif operation == 'open':
                assert link is None
                link = serial.Serial(args.port, 230400, timeout=0.02)
                link.reset_input_buffer()
                reading = True
                reader = threading.Thread(target=read_loop, daemon=True)
                reader.start()
            elif operation == 'write':
                data = bytes(message['bytes'])
                assert link and data[1] == 254 and struct.unpack('<Q', data[2:10])[0] == uid
                assert (data[0] >> 1) == len(data) and zlib.crc32(data[:-4]) == struct.unpack('<I', data[-4:])[0]
                assert data[10] in (0, 1, 12, 13, 15, 16, 17, 18, 22, 25, 27, 28, 31, 37, 43, 48, 49, 50)
                link.write(data)
                link.flush()
            elif operation == 'close':
                close()
            elif operation == 'cleanup':
                close()
                with serial.Serial(args.port, 230400, timeout=0.02) as cleanup:
                    cleanup.write(packet(12)); cleanup.flush()
                    time.sleep(0.05)
                    cleanup.reset_input_buffer()
                    cleanup.write(packet(27)); cleanup.flush()
                    time.sleep(1.5)
            elif operation == 'shutdown':
                close()
            else:
                raise ValueError('Unknown bridge operation')
            emit({'id': message['id'], 'result': result})
            if operation == 'shutdown':
                break
        except Exception as error:
            emit({'id': message['id'], 'error': f'{type(error).__name__}: {error}'})
finally:
    close()
