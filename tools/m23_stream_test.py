#!/usr/bin/env python3
"""
M23 Current Streaming Test Tool
===============================

Command-line tool for testing and validating the M23 DMA-based current streaming.

Usage:
  python m23_stream_test.py --port /dev/ttyUSB0 --baud 5000000 --duration 5

Features:
  - Sends 's' to start streaming on connect
  - Captures frames for specified duration
  - Prints real-time stats every second
  - Final summary with statistics
  - Sends 's' to stop streaming before exit
"""

import argparse
import sys
import time
import serial
import serial.tools.list_ports
import numpy as np
from collections import deque

from m23_telemetry_protocol import (
    FrameParser, M23TelemetryFrame,
    TELEMETRY_FRAME_SIZE, DEFAULT_BAUD, DEBUG_BAUD, SAMPLE_RATE_HZ,
    ADC_MAX_VALUE, PWM_PERIOD_TIM1
)


def list_serial_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]


def select_port_interactively():
    """Let user select a port interactively."""
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found!")
        return None

    print("\nAvailable ports:")
    for i, port in enumerate(ports):
        print(f"  {i + 1}. {port}")

    while True:
        try:
            choice = input("\nSelect port number (or 'q' to quit): ").strip()
            if choice.lower() == 'q':
                return None
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                return ports[idx]
            print("Invalid selection, try again.")
        except ValueError:
            print("Invalid input, enter a number.")


def run_stream_test(port: str, baud: int, duration: float, verbose: bool = False):
    """Run the streaming test."""

    # Step 1: Connect at 230400 baud to send initialization command
    print(f"\nConnecting to {port} at {DEBUG_BAUD} baud...")
    try:
        ser = serial.Serial(
            port=port,
            baudrate=DEBUG_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return 1

    # Clear any pending data
    ser.reset_input_buffer()
    time.sleep(0.1)

    # Step 2: Send '5' to switch firmware to 5Mbps mode
    print("Sending '5' to switch to 5Mbps mode...")
    ser.write(b'5')
    time.sleep(0.1)  # Wait for firmware to switch baud rate

    # Step 3: Change our baud rate to 5Mbps
    print(f"Switching to {baud} baud...")
    ser.baudrate = baud
    time.sleep(0.05)  # Brief settle time

    # Clear any garbage from baud rate transition
    ser.reset_input_buffer()
    time.sleep(0.05)

    # Step 4: Send 's' to start streaming
    print("Sending 's' to start streaming...")
    ser.write(b's')
    time.sleep(0.05)

    parser = FrameParser()
    frames_collected = []
    start_time = time.time()
    last_print_time = start_time
    frames_since_last_print = 0

    print(f"\nCapturing for {duration} seconds...")
    print("-" * 60)

    try:
        while (time.time() - start_time) < duration:
            # Read available data
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                frames = parser.feed(data)
                frames_collected.extend(frames)
                frames_since_last_print += len(frames)

            # Print stats every second
            current_time = time.time()
            if current_time - last_print_time >= 1.0:
                elapsed = current_time - start_time
                total_frames = len(frames_collected)
                print(f"[{elapsed:.1f}s] {total_frames} frames, "
                      f"{frames_since_last_print} frames/sec, "
                      f"{parser.sync_errors} sync errors")
                frames_since_last_print = 0
                last_print_time = current_time

            time.sleep(0.001)  # Small sleep to prevent busy waiting

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    # Stop streaming
    print("\nSending 's' to stop streaming...")
    ser.write(b's')
    time.sleep(0.1)
    ser.close()

    # Calculate statistics
    elapsed = time.time() - start_time
    total_frames = len(frames_collected)

    print("\n" + "=" * 60)
    print(f"SUMMARY ({elapsed:.2f} seconds)")
    print("=" * 60)

    print(f"\nFrame Statistics:")
    print(f"  Total frames:    {total_frames}")
    print(f"  Frame rate:      {total_frames / elapsed:.1f} Hz")
    print(f"  Expected rate:   {SAMPLE_RATE_HZ} Hz")
    print(f"  Sync errors:     {parser.sync_errors}")
    print(f"  Discarded (sync): {parser.discarded_frames}")

    if total_frames > 0:
        # Extract channel data
        current_a = np.array([f.current_a for f in frames_collected], dtype=np.int16)
        current_b = np.array([f.current_b for f in frames_collected], dtype=np.int16)
        pwm_a = np.array([f.pwm_a for f in frames_collected], dtype=np.int16)
        pwm_b = np.array([f.pwm_b for f in frames_collected], dtype=np.int16)

        print(f"\nChannel Statistics:")
        print(f"  {'Channel':<12} {'Min':>8} {'Max':>8} {'Mean':>10} {'Std':>10}")
        print(f"  {'-' * 48}")
        print(f"  {'current_a':<12} {current_a.min():>8} {current_a.max():>8} "
              f"{current_a.mean():>10.1f} {current_a.std():>10.1f}")
        print(f"  {'current_b':<12} {current_b.min():>8} {current_b.max():>8} "
              f"{current_b.mean():>10.1f} {current_b.std():>10.1f}")
        print(f"  {'pwm_a':<12} {pwm_a.min():>8} {pwm_a.max():>8} "
              f"{pwm_a.mean():>10.1f} {pwm_a.std():>10.1f}")
        print(f"  {'pwm_b':<12} {pwm_b.min():>8} {pwm_b.max():>8} "
              f"{pwm_b.mean():>10.1f} {pwm_b.std():>10.1f}")

        # Data quality checks
        print(f"\nData Quality:")
        current_a_valid = np.sum((current_a >= 0) & (current_a <= ADC_MAX_VALUE))
        current_b_valid = np.sum((current_b >= 0) & (current_b <= ADC_MAX_VALUE))
        pwm_valid = np.sum((pwm_a >= 0) & (pwm_a <= PWM_PERIOD_TIM1) &
                          (pwm_b >= 0) & (pwm_b <= PWM_PERIOD_TIM1))

        print(f"  Current A in range:  {current_a_valid}/{total_frames} "
              f"({100 * current_a_valid / total_frames:.1f}%)")
        print(f"  Current B in range:  {current_b_valid}/{total_frames} "
              f"({100 * current_b_valid / total_frames:.1f}%)")
        print(f"  PWM values in range: {pwm_valid}/{total_frames} "
              f"({100 * pwm_valid / total_frames:.1f}%)")

        # Check for expected frame rate
        actual_rate = total_frames / elapsed
        rate_error = abs(actual_rate - SAMPLE_RATE_HZ) / SAMPLE_RATE_HZ * 100
        if rate_error < 5:
            print(f"\n  Frame rate accuracy: GOOD (within 5% of expected)")
        elif rate_error < 20:
            print(f"\n  Frame rate accuracy: ACCEPTABLE ({rate_error:.1f}% error)")
        else:
            print(f"\n  Frame rate accuracy: POOR ({rate_error:.1f}% error)")

    print("\nDone.")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description='M23 Current Streaming Test Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --port /dev/ttyUSB0 --duration 5
  %(prog)s -P                    # Interactive port selection
  %(prog)s --list                # List available ports
        """
    )

    parser.add_argument('--port', '-p', type=str, default=None,
                        help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD})')
    parser.add_argument('--duration', '-d', type=float, default=5.0,
                        help='Capture duration in seconds (default: 5)')
    parser.add_argument('-P', '--interactive', action='store_true',
                        help='Interactive port selection')
    parser.add_argument('--list', action='store_true',
                        help='List available serial ports')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output')

    args = parser.parse_args()

    if args.list:
        ports = list_serial_ports()
        if ports:
            print("Available serial ports:")
            for port in ports:
                print(f"  {port}")
        else:
            print("No serial ports found.")
        return 0

    port = args.port
    if args.interactive or port is None:
        port = select_port_interactively()
        if port is None:
            return 1

    return run_stream_test(port, args.baud, args.duration, args.verbose)


if __name__ == '__main__':
    sys.exit(main())
