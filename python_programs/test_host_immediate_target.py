#!/usr/bin/env python3
"""Compile real target handlers, queue logic and control ISR; no motor required.

Hardware acquisition/output and the PID arithmetic are stubbed. The existing
PID differential test exercises the real controller separately. This harness
executes the real atomic foreground update, cached Hall readback, queue and framing
functions verbatim.
It also compiles the generated Arduino methods with a payload-level transport
stand-in and a portable test-only Arduino shim (no serial ports opened);
native C and C++ compilers are required.
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
from test_immediate_position_target import compact_position_reply_supported
from servomotor import M3, communication, get_command_id

ROOT = Path(__file__).resolve().parent.parent


def extract(source, name):
    match = re.search(r'^(?:static )?\w+ ' + name + r'\([^\n]*\)\s*\{.*?^\}', source, re.M | re.S)
    if not match:
        raise RuntimeError('Function not found: ' + name)
    return match[0]


def build(directory):
    motor = (ROOT / 'firmware/Src/motor_control.c').read_text()
    main = (ROOT / 'firmware/Src/main.c').read_text()
    rs485 = (ROOT / 'common_source_files/RS485.c').read_text()
    rs_header = (ROOT / 'common_source_files/RS485.h').read_text()
    source = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#include "commands.h"
#include "error_text.h"
#include "device_status.h"
#define PRODUCT_NAME_M17
#define MOVEMENT_QUEUE_SIZE 32
#define OPEN_LOOP_POSITION_CONTROL 0
#define CLOSED_LOOP_POSITION_CONTROL 1
#define ACCELERATION_SHIFT_LEFT 8
#define VELOCITY_SHIFT_LEFT 12
#define TURN_POINT_CALCULATION_SHIFT 4
#define HALL_TO_POSITION_90_DEGREE_OFFSET 16384
#define PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT 8
#define MAX_HALL_POSITION_DELTA_FATAL_ERROR_THRESHOLD 100000
#define MAX_MOTOR_CONTROL_PERIOD_FATAL_ERROR_THRESHOLD 100
#define ALL_MOTOR_CONTROL_CALULATIONS_PROFILER 0
#define MOTOR_CONTROL_LOOP_PERIOD_PROFILER 0
#define ERROR_CODE_NO_ERROR 0
#define RESPONSE_CHARACTER_CRC32_ENABLED 253
#define RESPONSE_CHARACTER_CRC32_DISABLED 252
#define RESPONSE_CHARACTER_CRC32_ENABLED_TEXT "\xFD"
#define RESPONSE_CHARACTER_CRC32_DISABLED_TEXT "\xFC"
typedef enum { MOVE_WITH_ACCELERATION, MOVE_WITH_VELOCITY } movement_type_t;
'''
    source += re.search(r'typedef struct \{\n    uint32_t low;.*?\} int96_t;', motor, re.S)[0] + '\n'
    source += re.search(r'typedef struct __attribute__\(\(\__packed__\)\) \{.*?\} movement_queue_t;', motor, re.S)[0] + '\n'
    source += '\n'.join(re.findall(r'^#define NO_ERROR_RESPONSE[^\n]+', rs_header, re.M)) + '\n'
    source += r'''
_Static_assert(sizeof(int64_t) == 8, "position wire size");
static int irq_enabled = 1, mosfets = 1, completed, phase_calls, pid_calls;
static int packet_mode, broadcast;
static int check_atomic_update, atomic_checks, irq_restores, commit_irq_restore, tick_at_restore;
static int64_t expected_atomic_target, last_pid_error;
static jmp_buf fault_jump;
static movement_queue_t movement_queue[MOVEMENT_QUEUE_SIZE];
static uint8_t queue_write_position, queue_read_position, n_items_in_queue;
static int96_t current_position_i96, position_after_last_queue_item_i96;
static int64_t hall_position_i64, current_velocity_i64, velocity_after_last_queue_item;
static int64_t position_lower_safety_limit_i64, position_upper_safety_limit_i64;
static int64_t max_allowable_position_deviation, max_velocity, max_acceleration;
static uint8_t motor_control_mode, motor_busy, homing_active, calibration_step;
static uint8_t go_to_closed_loop_step, test_mode, decelerate_to_stop_active;
static struct { uint8_t capture_type; } capture;
static int64_t debug_value1, debug_value2, debug_value3, debug_value4;
static int32_t integral_term, previous_error, low_pass_filtered_error_change;
static int32_t sensor_position, hall_position_delta, max_hall_position_delta, min_hall_position_delta;
static int32_t average_hall_position_delta, average_hall_position_delta_count;
static uint8_t hall_position_delta_violation;
static struct { uint16_t hall1, hall2, hall3, hall_position_16bit; } fast_capture_data[16];
static uint16_t fast_capture_data_index, fast_capture_data_size = 16;
static uint8_t fast_capture_data_active, fast_capture_data_result_ready;
static int32_t commutation_position, commutation_position_offset, desired_motor_pwm_voltage, motor_pwm_voltage;
static uint16_t max_motor_pwm_voltage = 200;
static int32_t sensor_change = 1;
static uint8_t crc32_enabled;
static struct { uint32_t SR; } timer;
#define TIM16 (&timer)
typedef struct { int32_t position, change; } get_sensor_position_return_t;
static get_sensor_position_return_t get_sensor_position(void) { return (get_sensor_position_return_t){0, sensor_change}; }
static void fake_tick(void);
static void __disable_irq(void) { assert(irq_enabled); irq_enabled = 0; }
static void __enable_irq(void) {
    assert(!irq_enabled);
    ++irq_restores;
    if(check_atomic_update && irq_restores == commit_irq_restore) {
        // Validation has already copied Hall separately. At the end of the
        // write mask, both complete position values must be visible together.
        assert(current_position_i96.position == expected_atomic_target);
        assert(position_after_last_queue_item_i96.position == expected_atomic_target);
        assert(current_position_i96.low == 0 && position_after_last_queue_item_i96.low == 0);
        check_atomic_update = 0;
        atomic_checks++;
    }
    irq_enabled = 1;
    if(irq_restores == tick_at_restore) {
        tick_at_restore = 0;
        fake_tick();
    }
}
#define __NOP() assert(!"setter must not wait for a control tick")
static uint8_t is_mosfets_enabled(void) { assert(irq_enabled); return mosfets; }
static void disable_mosfets(void) { mosfets = 0; }
static void rs485_transmit_error_packet(uint8_t code);
static void fatal_error(uint16_t code) {
    assert(irq_enabled);
    disable_mosfets();
    if(packet_mode) { if(!broadcast) rs485_transmit_error_packet(code); exit(code); }
    longjmp(fault_jump, code);
}
static void rs485_done_with_this_packet(void) { completed++; }
static void if_fatal_error_then_dont_respond(void) { }
static uint8_t encode_first_byte(uint16_t size) { return (size << 1) | 1; }
static void rs485_transmit(const void *data, uint16_t size) { fwrite(data, 1, size, stdout); }
static uint32_t calculate_crc32_buffer(void *data, uint32_t size) {
    uint32_t crc = UINT32_MAX;
    for(uint32_t i = 0; i < size; i++) {
        crc ^= ((uint8_t *)data)[i];
        for(int b = 0; b < 8; b++) crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0);
    }
    return ~crc;
}
static void profiler_start_time(int i) { (void)i; }
static void profiler_end_time(int i) { (void)i; }
static uint16_t profiler_period(int i) { (void)i; return 32; }
static int get_hall_sensor1_voltage(void) { return 0; }
static int get_hall_sensor2_voltage(void) { return 0; }
static int get_hall_sensor3_voltage(void) { return 0; }
static void compute_velocity(void) { }
static void handle_homing_logic(void) { }
static void handle_calibration_logic(void) { }
static void capture_logic(void) { }
static int32_t PID_controller(int64_t error) {
    last_pid_error = error;
    pid_calls++;
    return 0;
}
static void motor_phase_calculations(void) { phase_calls++; }
uint16_t get_motor_status_flags_without_disable_enable_irq(void);
'''
    names = ['get_hall_position', 'validate_immediate_position_target', 'set_immediate_position_target',
             'checked_position_error', 'check_if_actual_vs_desired_position_deviated_too_much',
             'add_int96', 'check_queue_item_prediction_overflow', 'add_to_queue',
             'handle_queued_movements', 'motor_movement_calculations',
             'get_motor_status_flags_without_disable_enable_irq', 'TIM16_IRQHandler']
    source += '\n'.join(extract(motor, name) for name in names) + '\n'
    source += r'''
static void fake_tick(void) {
    assert(irq_enabled);
    TIM16_IRQHandler();
}
'''
    source += '\n'.join(extract(main, name) for name in ['copy_input_parameters_and_check_size']) + '\n'
    source += '\n'.join(extract(rs485, name) for name in ['rs485_transmit_no_error_packet', 'rs485_transmit_error_packet', 'rs485_finalize_and_transmit_packet']) + '\n'
    cases = re.search(r'^    case SET_IMMEDIATE_POSITION_TARGET_COMMAND:.*?(?=^    case SET_MAX_ALLOWABLE_POSITION_DEVIATION:)', main, re.M | re.S)[0]
    source += 'static void dispatch(uint8_t command, void *payload, uint16_t payload_size, uint8_t is_broadcast) { switch(command) {\n' + cases + 'default: abort();\n} }\n'
    source += r'''
static void init(void) {
    memset(movement_queue, 0, sizeof(movement_queue));
    queue_write_position = queue_read_position = 3; n_items_in_queue = 0;
    current_position_i96 = (int96_t){0}; current_position_i96.position = 11;
    position_after_last_queue_item_i96 = current_position_i96;
    hall_position_i64 = 10; sensor_change = 1;
    current_velocity_i64 = velocity_after_last_queue_item = 0;
    position_lower_safety_limit_i64 = INT64_MIN; position_upper_safety_limit_i64 = INT64_MAX;
    max_allowable_position_deviation = INT64_MAX;
    max_velocity = max_acceleration = INT64_C(1) << 52;
    motor_control_mode = CLOSED_LOOP_POSITION_CONTROL; mosfets = irq_enabled = 1;
    motor_busy = homing_active = calibration_step = go_to_closed_loop_step = test_mode = 0;
    capture.capture_type = fast_capture_data_active = decelerate_to_stop_active = 0;
    phase_calls = pid_calls = completed = 0;
    check_atomic_update = atomic_checks = irq_restores = commit_irq_restore = tick_at_restore = 0;
    integral_term = 123; previous_error = -78; low_pass_filtered_error_change = 9;
}
static void reject(int code) {
    int96_t before = current_position_i96;
    int96_t planner = position_after_last_queue_item_i96;
    uint8_t queued = n_items_in_queue, read = queue_read_position, write = queue_write_position;
    int actual = setjmp(fault_jump);
    if(actual == 0) { set_immediate_position_target(100); assert(!"expected fatal error"); }
    assert(actual == code && !mosfets && irq_enabled);
    assert(memcmp(&before, &current_position_i96, sizeof(before)) == 0);
    assert(memcmp(&planner, &position_after_last_queue_item_i96, sizeof(planner)) == 0);
    assert(n_items_in_queue == queued && queue_read_position == read && queue_write_position == write);
}
static void expect_position_fault(int run_tick) {
    int actual = setjmp(fault_jump);
    if(actual == 0) {
        if(run_tick) fake_tick();
        else check_if_actual_vs_desired_position_deviated_too_much();
        assert(!"expected position fault");
    }
    assert(actual == ERROR_POSITION_DEVIATION_TOO_LARGE && !mosfets);
    if(run_tick) assert(pid_calls == 0);
}
static void expect_atomic_commit(int64_t target) {
    expected_atomic_target = target;
    commit_irq_restore = irq_restores + 2; // validation Hall read, then write
    check_atomic_update = 1;
}
static void selftest(void) {
    init();
    current_position_i96.low = 12345; position_after_last_queue_item_i96.low = 67890;
    expect_atomic_commit(100);
    set_immediate_position_target(100);
    int64_t measured_position = get_hall_position();
    assert(measured_position == 10 && hall_position_i64 == 10 && atomic_checks == 1);
    assert(irq_restores == 3 && !check_atomic_update); // independent read/write/read masks
    assert(current_position_i96.position == 100 && motor_control_mode == 1 && n_items_in_queue == 0);
    assert(phase_calls == 0 && pid_calls == 0);
    assert(position_after_last_queue_item_i96.position == 100 && current_velocity_i64 == 0);
    assert(integral_term == 123 && previous_error == -78 && low_pass_filtered_error_change == 9);
    assert(queue_read_position == 3 && queue_write_position == 3 && mosfets);
    for(int i = 0; i < 1000; i++) {
        int64_t target = (i & 1) ? -1000 : 1000;
        expect_atomic_commit(target);
        set_immediate_position_target(target);
        measured_position = get_hall_position();
        assert(!check_atomic_update && measured_position == 10 + i && n_items_in_queue == 0);
        assert(phase_calls == i && pid_calls == i);
        fake_tick();
        assert(last_pid_error == target - hall_position_i64);
        assert(phase_calls == i + 1 && queue_write_position == 3 && queue_read_position == 3);
    }
    assert(atomic_checks == 1001);
    // A controller tick is allowed between the target commit and Hall read.
    init(); expect_atomic_commit(100); tick_at_restore = commit_irq_restore;
    set_immediate_position_target(100);
    measured_position = get_hall_position();
    assert(atomic_checks == 1 && phase_calls == 1 && pid_calls == 1);
    assert(measured_position == 11 && hall_position_i64 == 11 && irq_enabled);
    // It may also run after validation's Hall copy but before the write.
    init(); expect_atomic_commit(100); tick_at_restore = 1;
    set_immediate_position_target(100);
    assert(get_hall_position() == 11 && atomic_checks == 1 && pid_calls == 1);
    assert(current_position_i96.position == 100 && position_after_last_queue_item_i96.position == 100);
    // Each Hall copy itself is coherent across a changing high/low word.
    init(); hall_position_i64 = UINT32_MAX; tick_at_restore = 1;
    measured_position = get_hall_position();
    assert(measured_position == UINT32_MAX && hall_position_i64 == INT64_C(0x100000000));
    // Queue timed motion before any tick after the setter.
    init(); set_immediate_position_target(1000);
    assert(phase_calls == 0 && pid_calls == 0);
    sensor_change = 0;
    add_to_queue(1 << 20, 2, MOVE_WITH_VELOCITY); add_to_queue(0, 1, MOVE_WITH_VELOCITY);
    assert(n_items_in_queue == 2);
    fake_tick(); fake_tick(); fake_tick(); fake_tick();
    assert(n_items_in_queue == 0 && current_position_i96.position == 1002);
    assert(position_after_last_queue_item_i96.position == 1002);
    set_immediate_position_target(-25); fake_tick();
    assert(current_position_i96.position == -25 && n_items_in_queue == 0);
    int64_t edges[] = { INT64_MIN, INT64_MIN + 1, -(INT64_C(1) << 53) - 1,
                       (INT64_C(1) << 53) + 1, INT64_MAX - 1, INT64_MAX };
    for(size_t i = 0; i < sizeof(edges)/sizeof(edges[0]); i++) {
        init(); sensor_change = 0; hall_position_i64 = edges[i];
        expect_atomic_commit(edges[i]);
        set_immediate_position_target(edges[i]);
        assert(get_hall_position() == edges[i] && current_position_i96.position == edges[i]);
        assert(atomic_checks == 1 && !check_atomic_update);
    }
    init(); n_items_in_queue = 1; reject(ERROR_QUEUE_NOT_EMPTY);
    init(); motor_control_mode = OPEN_LOOP_POSITION_CONTROL; reject(ERROR_NOT_IN_CLOSED_LOOP);
    init(); motor_control_mode = 2; reject(ERROR_NOT_IN_CLOSED_LOOP);
    init(); mosfets = 0; reject(ERROR_MOSFETS_NOT_ENABLED);
    uint8_t *busy[] = { &motor_busy, &homing_active, &calibration_step, &go_to_closed_loop_step,
                       &capture.capture_type, &fast_capture_data_active, &test_mode };
    for(size_t i = 0; i < sizeof(busy)/sizeof(busy[0]); i++) { init(); *busy[i] = 1; reject(ERROR_MOTOR_BUSY); }
    init(); current_velocity_i64 = 1; reject(ERROR_MOTOR_BUSY);
    init(); velocity_after_last_queue_item = 1; reject(ERROR_MOTOR_BUSY);
    init(); position_upper_safety_limit_i64 = 99; reject(ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE);
    init(); position_lower_safety_limit_i64 = 101; reject(ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE);
    init(); max_allowable_position_deviation = 89; reject(ERROR_POSITION_DEVIATION_TOO_LARGE);
    init(); hall_position_i64 = INT64_MIN; reject(ERROR_POSITION_DEVIATION_TOO_LARGE);
    assert(checked_position_error(INT64_MAX, 0) == INT64_MAX);
    assert(checked_position_error(INT64_MIN, -1) == INT64_MIN + 1);
    assert(checked_position_error(-123, 456) == -579);
    init(); max_allowable_position_deviation = 90; sensor_change = -1;
    set_immediate_position_target(100);
    assert(get_hall_position() == 10 && pid_calls == 0);
    fake_tick(); expect_position_fault(0);
    // A later Hall value must not overflow PID or absolute-value arithmetic.
    init(); hall_position_i64 = 0; sensor_change = -1;
    set_immediate_position_target(INT64_MAX);
    assert(get_hall_position() == 0 && pid_calls == 0);
    expect_position_fault(1);
    init(); hall_position_i64 = -1; sensor_change = 1;
    set_immediate_position_target(INT64_MIN); expect_position_fault(1);
    init(); hall_position_i64 = -1; sensor_change = 2;
    set_immediate_position_target(INT64_MIN); expect_position_fault(1);
    init(); current_position_i96.position = INT64_MAX; hall_position_i64 = -1; expect_position_fault(0);
    init(); current_position_i96.position = INT64_MIN; hall_position_i64 = 0; expect_position_fault(0);
    init(); current_position_i96.position = INT64_MIN; hall_position_i64 = 1; expect_position_fault(0);
    puts("PASSED: separate atomic writes/reads, IRQ-enabled validation, preemption between operations, 1000 retargets, queued followup, i64 edges and fatal guards");
}
int main(int argc, char **argv) {
#ifdef _WIN32
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    init();
    if(argc == 1) { selftest(); return 0; }
    assert(argc == 8); packet_mode = 1;
    uint8_t command = atoi(argv[1]); int64_t target = strtoll(argv[2], NULL, 0);
    hall_position_i64 = strtoll(argv[3], NULL, 0); sensor_change = 0;
    crc32_enabled = atoi(argv[4]); broadcast = atoi(argv[5]);
    if(atoi(argv[7])) n_items_in_queue = 1;
    dispatch(command, &target, atoi(argv[6]), broadcast);
    assert(completed == 1 && current_position_i96.position == target && phase_calls == 0 && pid_calls == 0);
    return 0;
}
'''
    path = Path(directory) / 'immediate_target.c'
    path.write_text(source)
    executable = Path(directory) / ('immediate_target.exe' if os.name == 'nt' else 'immediate_target')
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=gnu11', '-O2', '-Wall', '-Wextra', '-Werror',
        '-I' + str(ROOT / 'firmware/Src'), '-I' + str(ROOT / 'common_source_files'),
        str(path), '-o', str(executable)], check=True)
    return executable


def main():
    t = TestRunner('test_host_immediate_target')
    for numeric, supported in [
        ((0,16,0,0),False), ((0,17,0,0),True), ((0,17,0,1),False),
        ((0,17,0,2),False), ((0,17,0,3),True), ((0,17,0,6),True),
        ((0,17,1,0),True), ((0,18,0,0),True),
    ]:
        t.check_eq('compact reply firmware gate ' + '.'.join(map(str, numeric)),
                   compact_position_reply_supported([list(reversed(numeric)), 0]), supported)
    t.check_eq('bootloader is not a compact-position application',
               compact_position_reply_supported([[0,0,17,0],1]), False)

    t.check_eq('target command ID', get_command_id('Set immediate position target'), 49)
    t.check_eq('combined command ID', get_command_id('Set immediate position target and get position'), 50)
    with tempfile.TemporaryDirectory(prefix='immediate_target_') as directory:
        cpp_executable = Path(directory) / ('arduino_immediate_target.exe' if os.name == 'nt' else 'arduino_immediate_target')
        subprocess.run(shlex.split(os.environ.get('CXX', 'c++')) + [
            '-std=c++17', '-DARDUINO',
            '-I' + str(ROOT / 'Arduino_library/test_support'),
            '-I' + str(ROOT / 'Arduino_library'),
            *[str(ROOT / 'Arduino_library' / name) for name in (
                'test_host_immediate_target.cpp', 'Servomotor.cpp',
                'DataTypes.cpp', 'AutoGeneratedUnitConversions.cpp')],
            '-o', str(cpp_executable)], check=True)
        cpp = subprocess.run([str(cpp_executable)], capture_output=True, text=True)
        t.check_eq('generated Arduino raw/converted results, i64, addressing and errors: ' +
                   cpp.stdout + cpp.stderr, cpp.returncode, 0)
        executable = build(directory)
        native = subprocess.run([str(executable)], capture_output=True, text=True)
        t.check_eq('native ISR/state/queue self-tests: ' + native.stdout + native.stderr, native.returncode, 0)
        golden = subprocess.run([str(executable), '50', '1000', '900', '1', '0', '8', '0'], capture_output=True, check=True)
        t.check_bytes_eq('shared browser/documentation state vector', golden.stdout,
                         bytes.fromhex('1ffd0084030000000000007517a255'))
        communication.ser = FakeSerial(golden.stdout)
        t.check_eq('binding returns measured position, not requested target',
                   M3('X', position_unit='encoder_counts', verbose=0).set_immediate_position_target_and_get_position(1000), 900)
        # Compact clients must reject a legacy reply rather than treating its target as measured.
        legacy = struct.pack('<qqIHBB', 1000, 900, 43, 6, 1, 0)
        data = bytes([63, 253, 0]) + legacy
        communication.ser = FakeSerial(data + struct.pack('<I', zlib.crc32(data)))
        t.check_raises('legacy 24-byte reply rejected', communication.CommunicationError,
                       M3('X', position_unit='encoder_counts', verbose=0).set_immediate_position_target_and_get_position, 1000)
        t.check_eq('schema mismatch sends only one request', len(communication.ser.written), 15)
        for crc in (True, False):
            for command in (49, 50):
                for target in (-123456, 0, 10, 123456, -(1 << 63), (1 << 63) - 1, (1 << 53) + 1):
                    # Same measured position keeps full-width edge targets within the deviation guard.
                    args = [str(executable), str(command), str(target), str(target), str(int(crc)), '0', '8', '0']
                    result = subprocess.run(args, capture_output=True)
                    t.check_eq('native success', result.returncode, 0)
                    payload = b'' if command == 49 else struct.pack('<q', target)
                    expected = (bytes([13 if crc else 5, 253 if crc else 252]) if command == 49 else
                                bytes([31 if crc else 23, 253 if crc else 252, 0]) + payload)
                    if crc:
                        expected += struct.pack('<I', zlib.crc32(expected))
                    t.check_bytes_eq('actual firmware packet', result.stdout, expected)
                    for address in ('X', '04245C407D26CCD0'):
                        communication.ser = FakeSerial(result.stdout)
                        motor = M3(address, position_unit='encoder_counts', verbose=0)
                        motor.set_crc32_enabled(crc)
                        reply = (motor.set_immediate_position_target(target) if command == 49 else
                                 motor.set_immediate_position_target_and_get_position(target))
                        if command == 50:
                            t.check_eq('Python preserves full i64 position', reply, target)
                        written = bytes(communication.ser.written)
                        t.check_bytes_eq('Python sends full i64 target', written[-12:-4] if crc else written[-8:], struct.pack('<q', target))
                    for broadcast in (0, 1):
                        bad = args.copy(); bad[5] = str(broadcast); bad[6] = '7'
                        error = subprocess.run(bad, capture_output=True)
                        t.check_eq('malformed payload fatal 51', error.returncode, 51)
                        if broadcast:
                            t.check_bytes_eq('broadcast failure silent', error.stdout, b'')
                        else:
                            expected_error = bytes([15 if crc else 7, 253 if crc else 252, 51])
                            if crc:
                                expected_error += struct.pack('<I', zlib.crc32(expected_error))
                            t.check_bytes_eq('fatal size error wire packet', error.stdout, expected_error)
                    silent = args.copy(); silent[5] = '1'
                    t.check_bytes_eq('broadcast applies silently', subprocess.run(silent, capture_output=True, check=True).stdout, b'')
                    busy = args.copy(); busy[7] = '1'
                    queue_error = subprocess.run(busy, capture_output=True)
                    t.check_eq('nonempty queue fatal 8', queue_error.returncode, 8)
                    expected_error = bytes([15 if crc else 7, 253 if crc else 252, 8])
                    if crc:
                        expected_error += struct.pack('<I', zlib.crc32(expected_error))
                    t.check_bytes_eq('fatal queue error wire packet', queue_error.stdout, expected_error)
        # Position conversion still applies to the measured result.
        payload = struct.pack('<q', -819200)
        data = bytes([31, 253, 0]) + payload
        communication.ser = FakeSerial(data + struct.pack('<I', zlib.crc32(data)))
        reply = M3('X', position_unit='degrees', verbose=0).set_immediate_position_target_and_get_position(90)
        t.check_eq('measured position degree conversion', reply, -90)
    t.finish()


if __name__ == '__main__':
    main()
