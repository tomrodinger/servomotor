#!/usr/bin/env python3
"""Offline library contract tests for PID readback and immediate positions.

Only an in-memory byte stream is used. No serial ports are enumerated/opened.
Covers public M3 methods, not just the low-level pack/unpack helpers.
"""
import contextlib
import io
import json
import math
from pathlib import Path
import random
import struct
import sys
import unittest
import zlib

from host_test_framework import FakeSerial
from servomotor import M3, communication, FatalError, CommunicationError, TimeoutError

UID = 0x0123456789ABCDEF
COUNT = 3276800


def reply(payload=b'', crc=True, error=0, ack=False, extended=False):
    body = bytes([253 if crc else 252])
    if not ack:
        body += bytes([error]) + payload
    size = len(body) + (4 if crc else 0) + (3 if extended else 1)
    packet = (b'\xff' + struct.pack('<H', size) if extended else bytes([(size << 1) | 1])) + body
    return packet + (struct.pack('<I', zlib.crc32(packet)) if crc else b'')


def request(address, command, payload=b'', crc=True):
    body = (bytes([address]) if address <= 255 else b'\xfe' + struct.pack('<Q', address))
    body += bytes([command]) + payload
    packet = bytes([((len(body) + 1 + (4 if crc else 0)) << 1) | 1]) + body
    return packet + (struct.pack('<I', zlib.crc32(packet)) if crc else b'')


class TracedSerial(FakeSerial):
    def __init__(self, rx=b''):
        super().__init__(rx)
        self.writes = 0
        self.reads = 0

    def write(self, data):
        self.writes += 1
        return super().write(data)

    def read(self, size):
        self.reads += 1
        return super().read(size)


class LibraryCommands(unittest.TestCase):
    def setUp(self):
        self.old_serial = communication.ser
        self.link = TracedSerial()
        communication.ser = self.link

    def tearDown(self):
        communication.ser = self.old_serial

    def motor(self, address='X', crc=True, **units):
        units.setdefault('position_unit', 'encoder_counts')
        m = M3(address, verbose=0, **units)
        m.set_crc32_enabled(crc)
        return m

    def load(self, data):
        self.link = TracedSerial(data)
        communication.ser = self.link

    def test_names_and_ids(self):
        for name, number in [('Get PID constants', 48), ('Set immediate position target', 49),
                             ('Set immediate position target and get position', 50)]:
            self.assertEqual(communication.get_command_id(name), number)
        self.assertFalse(hasattr(self.motor(), 'set_immediate_position_target_and_get_state'))

    def test_literal_wire_vectors(self):
        self.load(bytes.fromhex('1ffd0084030000000000007517a255'))
        self.assertEqual(self.motor().set_immediate_position_target_and_get_position(1000), 900)
        self.assertEqual(bytes(self.link.written), bytes.fromhex('1f5832e803000000000000408b509d'))
        self.load(bytes.fromhex('0dfd13e27b37'))
        self.assertEqual(self.motor().set_immediate_position_target(1000), [])
        self.assertEqual(bytes(self.link.written), bytes.fromhex('1f5831e80300000000000085b7dda4'))

    def test_pid_unsigned_triplets_and_all_addressing(self):
        triplets = [(0, 0, 0), (2000, 5, 175000), (2**31, 2**32-1, 2**31-1),
                    (0x01234567, 0x89abcdef, 0xfedcba98)]
        for address, encoded in [('X', 88), (88, 88), (UID, UID), (f'{UID:016X}', UID)]:
            for crc in (False, True):
                for values in triplets:
                    with self.subTest(address=address, crc=crc, values=values):
                        self.load(reply(struct.pack('<III', *values), crc))
                        result = self.motor(address, crc).get_pid_constants()
                        self.assertEqual(result, list(values))
                        self.assertTrue(all(type(v) is int for v in result))
                        self.assertEqual(bytes(self.link.written), request(encoded, 48, crc=crc))
                        self.assertEqual(self.link.writes, 1)

    def test_pid_setter_and_new_readback_use_matching_raw_representation(self):
        rng=random.Random(0x4817)
        triples=[(2000,5,175000),(0,0,0),(2**32-1,2**31,2**32-1)]
        triples += [tuple(rng.randrange(2**32) for _ in range(3)) for _ in range(64)]
        for address in (88,UID):
            for crc in (False,True):
                for values in triples:
                    self.load(reply(crc=crc,ack=True)+reply(struct.pack('<III',*values),crc))
                    motor=self.motor(address,crc)
                    motor.set_pid_constants(**dict(zip(('kD','kI','kP'),reversed(values))))
                    self.assertEqual(motor.get_pid_constants(),list(values))
                    self.assertEqual(bytes(self.link.written),
                                     request(address,43,struct.pack('<III',*values),crc)+request(address,48,crc=crc))
                    self.assertEqual(self.link.writes,2)

    def test_pid_invalid_inputs_never_transmit_a_partial_setting(self):
        for field in ('kP','kI','kD'):
            for bad in (-1,2**32,math.inf,-math.inf,math.nan):
                self.load(reply(ack=True))
                values={'kP':2000,'kI':5,'kD':175000};values[field]=bad
                with self.subTest(field=field,bad=bad):
                    with self.assertRaises((ValueError,OverflowError)):
                        self.motor().set_pid_constants(**values)
                    self.assertEqual(self.link.writes,0)

    def test_raw_i64_exactness_and_scalar_return(self):
        rng = random.Random(0x174950)
        values = [-2**63, -2**63+1, -2**53-1, -10, -1, 0, 10, 2**53+1, 2**63-1]
        values += [rng.randrange(-2**63, 2**63) for _ in range(128)]
        for address in (88, UID):
            for crc in (True, False):
                for target in values:
                    measured = -target-1
                    with self.subTest(address=address, crc=crc, target=target):
                        self.load(reply(struct.pack('<q', measured), crc))
                        result = self.motor(address, crc).set_immediate_position_target_and_get_position(target)
                        self.assertEqual(result, measured)
                        self.assertIs(type(result), int)
                        self.assertEqual(bytes(self.link.written), request(address, 50, struct.pack('<q', target), crc))
                        self.assertEqual(self.link.writes, 1)

    def test_ack_for_standalone_target_both_modes(self):
        for address in (88, UID):
            for crc in (True, False):
                for target in (-2**63, -1, 0, 2**63-1):
                    self.load(reply(crc=crc, ack=True))
                    self.assertEqual(self.motor(address, crc).set_immediate_position_target(target), [])
                    self.assertEqual(bytes(self.link.written), request(address, 49, struct.pack('<q', target), crc))

    def test_position_unit_conversions_both_commands(self):
        for unit, value, expected in [('degrees', 90, COUNT//4), ('shaft_rotations', -0.5, -COUNT//2),
                                      ('radians', math.pi, COUNT//2), ('encoder_counts', 2**53+1, 2**53+1)]:
            for command in (49, 50):
                self.load(reply(struct.pack('<q', -expected)) if command == 50 else reply(ack=True))
                motor = self.motor(position_unit=unit)
                method = motor.set_immediate_position_target_and_get_position if command == 50 else motor.set_immediate_position_target
                result = method(value)
                self.assertEqual(bytes(self.link.written), request(88, command, struct.pack('<q', expected)))
                if command == 50:
                    self.assertAlmostEqual(result, -value, places=6)

    def test_keyword_parameter_matches_positional(self):
        self.load(reply(struct.pack('<q', -123)))
        self.assertEqual(self.motor().set_immediate_position_target_and_get_position(targetPosition=123), -123)
        self.assertEqual(bytes(self.link.written), request(88, 50, struct.pack('<q', 123)))

    def test_unknown_keywords_are_rejected_before_transmission(self):
        with self.assertRaises(TypeError):
            self.motor().set_immediate_position_target(123, targetPostion=456)
        self.assertEqual(self.link.writes, 0)

    def test_duplicate_and_missing_parameters(self):
        for operation in [lambda m: m.set_immediate_position_target(1, targetPosition=2),
                          lambda m: m.set_immediate_position_target(),
                          lambda m: m.set_immediate_position_target(1, 2),
                          lambda m: m.get_pid_constants(1)]:
            self.load(reply(ack=True))
            with self.assertRaises(TypeError):
                operation(self.motor())
            self.assertEqual(self.link.writes, 0)

    def test_nonfinite_and_out_of_range_targets(self):
        for command in (49, 50):
            for value in (-2**63-1, 2**63, math.inf, -math.inf, math.nan):
                self.load(reply(ack=True))
                motor = self.motor()
                method = motor.set_immediate_position_target_and_get_position if command == 50 else motor.set_immediate_position_target
                with self.subTest(command=command, value=value):
                    with self.assertRaises((ValueError, OverflowError)):
                        method(value)
                    self.assertEqual(self.link.writes, 0)

    def test_conversion_overflow_never_transmits(self):
        for unit in ('degrees','radians','shaft_rotations'):
            for value in (-1e308,1e308):
                for combined in (False,True):
                    self.load(reply(struct.pack('<q',0)) if combined else reply(ack=True))
                    motor=self.motor(position_unit=unit)
                    method=motor.set_immediate_position_target_and_get_position if combined else motor.set_immediate_position_target
                    with self.assertRaises((ValueError,OverflowError)):
                        method(value)
                    self.assertEqual(self.link.writes,0)

    def test_broadcast_never_reads_or_claims_a_measurement(self):
        for command, args in [(48, ()), (49, (123,)), (50, (123,))]:
            for crc in (True, False):
                self.load(b'')
                motor = self.motor(255, crc)
                method = {48:motor.get_pid_constants,49:motor.set_immediate_position_target,
                          50:motor.set_immediate_position_target_and_get_position}[command]
                self.assertEqual(method(*args), [])
                self.assertEqual(self.link.reads, 0)
                self.assertEqual(self.link.writes, 1)
                self.assertEqual(bytes(self.link.written), request(255, command, b'' if command == 48 else struct.pack('<q',123), crc))

    def test_all_fatal_guards_propagate_without_retries(self):
        for command in (48,49,50):
            for code in (8,13,19,27,45,51,55):
                for crc in (True,False):
                    self.load(reply(crc=crc,error=code))
                    motor=self.motor(UID,crc)
                    method={48:motor.get_pid_constants,49:motor.set_immediate_position_target,
                            50:motor.set_immediate_position_target_and_get_position}[command]
                    with self.assertRaises(FatalError) as caught: method(*(() if command==48 else (100,)))
                    self.assertEqual(caught.exception.args,(code,))
                    self.assertEqual(self.link.writes,1)

    def test_crc_corruption_rejected_without_retry(self):
        for command, payload in [(48,struct.pack('<III',2000,5,175000)),(49,None),(50,struct.pack('<q',-123))]:
            data=bytearray(reply(payload or b'',ack=payload is None));data[-1]^=0x80
            self.load(data);motor=self.motor()
            method={48:motor.get_pid_constants,49:motor.set_immediate_position_target,
                    50:motor.set_immediate_position_target_and_get_position}[command]
            with self.assertRaises(CommunicationError):method(*(() if command==48 else (123,)))
            self.assertEqual(self.link.writes,1)

    def test_every_single_bit_reply_corruption_is_a_transport_error(self):
        for command, payload in [(48,struct.pack('<III',2000,5,175000)),(49,None),(50,struct.pack('<q',10))]:
            for fault in (False, True):
                complete = reply(error=55) if fault else reply(payload or b'',ack=payload is None)
                for byte in range(len(complete)):
                    for bit in range(8):
                        damaged=bytearray(complete);damaged[byte]^=1<<bit
                        self.load(damaged);motor=self.motor()
                        method={48:motor.get_pid_constants,49:motor.set_immediate_position_target,
                                50:motor.set_immediate_position_target_and_get_position}[command]
                        with self.subTest(command=command,fault=fault,byte=byte,bit=bit):
                            with self.assertRaises((CommunicationError,TimeoutError)):
                                method(*(() if command==48 else (123,)))
                            self.assertEqual(self.link.writes,1)

    def test_timeout_and_every_truncation_do_not_retry(self):
        for command,payload in [(48,struct.pack('<III',1,2,3)),(49,None),(50,struct.pack('<q',42))]:
            complete=reply(payload or b'',ack=payload is None)
            for cut in range(len(complete)):
                self.load(complete[:cut]);motor=self.motor()
                method={48:motor.get_pid_constants,49:motor.set_immediate_position_target,
                        50:motor.set_immediate_position_target_and_get_position}[command]
                with self.subTest(command=command,cut=cut):
                    with self.assertRaises((TimeoutError,CommunicationError)):method(*(() if command==48 else (123,)))
                    self.assertEqual(self.link.writes,1)

    def test_wrong_and_legacy_response_sizes_never_become_positions(self):
        for size in (0,1,7,9,12,16,24):
            self.load(reply(b'\0'*size))
            with self.assertRaises(CommunicationError):self.motor().set_immediate_position_target_and_get_position(123)
            self.assertEqual(self.link.writes,1)

    def test_extended_response_header_is_decoded(self):
        self.load(reply(struct.pack('<q',-9007199254740993),extended=True))
        self.assertEqual(self.motor().set_immediate_position_target_and_get_position(1),-9007199254740993)
        self.assertEqual(self.link.writes,1)

    def test_coalesced_responses_consumed_once_in_order(self):
        self.load(reply(struct.pack('<III',1,2,3))+reply(ack=True)+reply(struct.pack('<q',-42)))
        m=self.motor()
        self.assertEqual(m.get_pid_constants(),[1,2,3])
        self.assertEqual(m.set_immediate_position_target(42),[])
        self.assertEqual(m.set_immediate_position_target_and_get_position(43),-42)
        self.assertEqual(self.link.writes,3)

    def test_legacy_velocity_internal_unit_retains_fixed_point_scaling(self):
        self.load(reply(ack=True))
        m=self.motor(velocity_unit='counts_per_timestep',time_unit='timesteps')
        m.move_with_velocity(1,10)
        self.assertEqual(bytes(self.link.written)[3:11],struct.pack('<iI',1<<20,10))

    def test_legacy_acceleration_internal_unit_retains_fixed_point_scaling(self):
        self.load(reply(ack=True))
        m=self.motor(acceleration_unit='counts_per_timestep_squared',time_unit='timesteps')
        m.move_with_acceleration(1,10)
        self.assertEqual(bytes(self.link.written)[3:11],struct.pack('<iI',1<<24,10))

    def test_reversed_keyword_order_uses_declared_parameter_order(self):
        self.load(reply(ack=True))
        m=self.motor(velocity_unit='counts_per_timestep',time_unit='timesteps')
        m.move_with_velocity(**{'duration':10,'velocity':1})
        self.assertEqual(bytes(self.link.written)[3:11],struct.pack('<iI',1<<20,10))


def main():
    with contextlib.redirect_stdout(io.StringIO()):
        result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(LibraryCommands))
    print('test_host_library_commands '+('PASSED' if result.wasSuccessful() else 'FAILED'))
    return 0 if result.wasSuccessful() else 1

if __name__=='__main__':raise SystemExit(main())
