#!/usr/bin/env python3
"""
M23 Current Data Capture & Plot Tool
=====================================

Captures streaming current data from the M23 motor and displays a plot.
Optionally saves the raw data to a file.

Can also plot a previously saved file without capturing.

Usage:
  # Capture 5 seconds and plot immediately:
  python m23_current_capture.py --port /dev/tty.usbserial-110 --duration 5

  # Capture with interactive port selection:
  python m23_current_capture.py -P --duration 10

  # Capture, plot, and also save to file:
  python m23_current_capture.py -P --duration 5 --output capture.bin

  # Plot a previously saved file:
  python m23_current_capture.py plot captured_data.bin

  # Plot only a time range (seconds):
  python m23_current_capture.py plot captured_data.bin --start 1.0 --end 3.0
"""

import argparse
import os
import sys
import time
import struct
import serial
import serial.tools.list_ports

# Protocol constants (matching firmware current_streaming.h)
TELEMETRY_FRAME_SIZE = 9
TELEMETRY_DATA_SIZE = 8  # Bytes before checksum
DEFAULT_BAUD = 5000000
DEBUG_BAUD = 230400
SAMPLE_RATE_HZ = 31250


def compute_checksum(data):
    """Compute checksum: sum of bytes, truncated to uint8."""
    return sum(data) & 0xFF


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


def do_capture(port, baud, duration, output_file):
    """Capture raw streaming data to a buffer. Optionally save to file."""

    # Step 1: Connect at debug baud
    print(f"Connecting to {port} at {DEBUG_BAUD} baud...")
    ser = serial.Serial(
        port=port,
        baudrate=DEBUG_BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1
    )
    ser.reset_input_buffer()
    time.sleep(0.1)

    # Step 2: Switch to 5Mbps
    print("Sending '5' to switch firmware to 5Mbps...")
    ser.write(b'5')
    time.sleep(0.2)

    print(f"Switching serial to {baud} baud...")
    ser.baudrate = baud
    time.sleep(0.05)
    ser.reset_input_buffer()
    time.sleep(0.05)

    # Step 3: Start streaming
    print("Sending 's' to start streaming...")
    ser.write(b's')
    time.sleep(0.05)

    # Step 4: Capture raw bytes to buffer
    print(f"Capturing for {duration} seconds...")
    print("-" * 50)

    chunks = []
    total_bytes = 0
    start_time = time.time()
    last_print_time = start_time

    try:
        while (time.time() - start_time) < duration:
            n = ser.in_waiting
            if n > 0:
                data = ser.read(n)
                chunks.append(data)
                total_bytes += len(data)

            # Print progress every second (minimal overhead)
            now = time.time()
            if now - last_print_time >= 1.0:
                elapsed = now - start_time
                rate_mbps = (total_bytes * 8) / (elapsed * 1e6)
                est_frames = total_bytes // TELEMETRY_FRAME_SIZE
                print(f"  [{elapsed:.0f}s] {total_bytes:,} bytes ({rate_mbps:.2f} Mbps), ~{est_frames:,} frames")
                last_print_time = now
    except KeyboardInterrupt:
        print("\nCapture interrupted by user")

    elapsed = time.time() - start_time

    # Step 5: Stop streaming
    print("\nSending 's' to stop streaming...")
    ser.write(b's')
    time.sleep(0.1)
    ser.close()

    raw = b''.join(chunks)

    est_frames = total_bytes // TELEMETRY_FRAME_SIZE
    print(f"\nCapture complete:")
    print(f"  Size:      {total_bytes:,} bytes")
    print(f"  Duration:  {elapsed:.2f} s")
    print(f"  Est frames: ~{est_frames:,}")
    print(f"  Throughput: {(total_bytes * 8) / (elapsed * 1e6):.2f} Mbps")

    # Optionally save to file
    if output_file is not None:
        with open(output_file, 'wb') as f:
            f.write(raw)
        print(f"  Saved to:  {output_file}")

    return raw


def parse_frames(raw):
    """Parse telemetry frames from raw binary data using checksum validation."""
    frames = []
    pos = 0
    checksum_errors = 0
    consecutive_good = 0
    synchronized = False
    REQUIRED_CONSECUTIVE = 3

    while pos + TELEMETRY_FRAME_SIZE <= len(raw):
        frame_data = raw[pos:pos + TELEMETRY_FRAME_SIZE]
        expected_cksum = compute_checksum(frame_data[:TELEMETRY_DATA_SIZE])

        if frame_data[TELEMETRY_DATA_SIZE] == expected_cksum:
            consecutive_good += 1
            if synchronized or consecutive_good >= REQUIRED_CONSECUTIVE:
                synchronized = True
                cur_a, cur_b, pwm_a, pwm_b = struct.unpack_from('<hhhh', frame_data)
                frames.append((cur_a, cur_b, pwm_a, pwm_b))
            pos += TELEMETRY_FRAME_SIZE
        else:
            checksum_errors += 1
            consecutive_good = 0
            synchronized = False
            pos += 1

    print(f"Parsed {len(frames):,} valid frames ({checksum_errors} checksum errors)")
    return frames


def do_plot(frames, title, start_time, end_time):
    """Plot parsed frames."""
    import numpy as np
    import matplotlib.pyplot as plt

    if not frames:
        print("No valid frames found!")
        return

    data = np.array(frames, dtype=np.int16)
    current_a = data[:, 0].astype(np.float64)
    current_b = data[:, 1].astype(np.float64)
    pwm_a = data[:, 2].astype(np.float64)
    pwm_b = data[:, 3].astype(np.float64)

    t = np.arange(len(frames)) / SAMPLE_RATE_HZ

    mask = np.ones(len(t), dtype=bool)
    if start_time is not None:
        mask &= t >= start_time
    if end_time is not None:
        mask &= t <= end_time

    t = t[mask]
    current_a = current_a[mask]
    current_b = current_b[mask]
    pwm_a = pwm_a[mask]
    pwm_b = pwm_b[mask]

    if len(t) == 0:
        print("No data in specified time range!")
        return

    print(f"\nPlotting {len(t):,} frames ({t[0]:.3f}s to {t[-1]:.3f}s)")
    print(f"\n{'Channel':<12} {'Min':>8} {'Max':>8} {'Mean':>10} {'Std':>10} {'P-P':>8}")
    print(f"{'-'*58}")
    for name, arr in [('current_a', current_a), ('current_b', current_b),
                       ('pwm_a', pwm_a), ('pwm_b', pwm_b)]:
        print(f"  {name:<12} {arr.min():>8.0f} {arr.max():>8.0f} "
              f"{arr.mean():>10.1f} {arr.std():>10.1f} {arr.max()-arr.min():>8.0f}")

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f'M23 Current Capture: {title}', fontsize=14)

    # Plot 1: Filtered current (signed, relative to baseline)
    ax = axes[0]
    ax.plot(t, current_a, label='Phase A current', color='#2ecc71', linewidth=0.5, alpha=0.8)
    ax.plot(t, current_b, label='Phase B current', color='#3498db', linewidth=0.5, alpha=0.8)
    ax.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax.set_ylabel('Current (ADC counts from baseline)')
    ax.set_title('Motor Current (filtered, baseline-subtracted)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Plot 2: PWM duty values
    ax = axes[1]
    ax.plot(t, pwm_a, label='Phase A PWM (CCR1)', color='#f39c12', linewidth=0.5, alpha=0.8)
    ax.plot(t, pwm_b, label='Phase B PWM (CCR2)', color='#9b59b6', linewidth=0.5, alpha=0.8)
    ax.axhline(y=512, color='gray', linestyle='--', linewidth=0.5, label='Midpoint (512)')
    ax.set_ylabel('CCR value')
    ax.set_title('PWM Duty Cycle')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Plot 3: Current vs PWM overlay (normalized for comparison)
    ax = axes[2]
    pwm_mid = 512
    cur_max = max(abs(current_a).max(), abs(current_b).max(), 1)
    pwm_max = max(abs(pwm_a - pwm_mid).max(), 1)
    scale = cur_max / pwm_max if pwm_max > 0 else 1
    ax.plot(t, current_a, label='Phase A current', color='#2ecc71', linewidth=0.5, alpha=0.8)
    ax.plot(t, (pwm_a - pwm_mid) * scale, label=f'Phase A PWM (scaled)', color='#f39c12', linewidth=0.5, alpha=0.6)
    ax.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    ax.set_ylabel('Current / scaled PWM')
    ax.set_xlabel('Time (seconds)')
    ax.set_title('Phase A: Current vs PWM Overlay')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='M23 Current Data Capture & Plot Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest='mode', help='Operating mode')

    # Plot subcommand (for previously saved files)
    plt_cmd = subparsers.add_parser('plot', help='Plot a previously captured file')
    plt_cmd.add_argument('file', type=str, help='Path to captured .bin file')
    plt_cmd.add_argument('--start', '-s', type=float, default=None, help='Start time in seconds')
    plt_cmd.add_argument('--end', '-e', type=float, default=None, help='End time in seconds')

    # Capture arguments (default mode when no subcommand given)
    parser.add_argument('--port', '-p', type=str, default=None, help='Serial port')
    parser.add_argument('-P', '--interactive', action='store_true', help='Interactive port selection')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD, help=f'Streaming baud rate (default: {DEFAULT_BAUD})')
    parser.add_argument('--duration', '-d', type=float, default=5.0, help='Capture duration in seconds (default: 5)')
    parser.add_argument('--output', '-o', type=str, default=None, help='Also save raw data to this file')

    args = parser.parse_args()

    if args.mode == 'plot':
        # Plot a previously saved file
        if not os.path.exists(args.file):
            print(f"Error: file '{args.file}' not found")
            return 1
        with open(args.file, 'rb') as f:
            raw = f.read()
        print(f"Read {len(raw):,} bytes from '{args.file}'")
        frames = parse_frames(raw)
        do_plot(frames, os.path.basename(args.file), args.start, args.end)
        return 0

    # Default mode: capture and plot
    port = args.port
    if args.interactive or port is None:
        port = select_port_interactively()
        if port is None:
            return 1
    raw = do_capture(port, args.baud, args.duration, args.output)
    frames = parse_frames(raw)
    title = os.path.basename(args.output) if args.output else 'live capture'
    do_plot(frames, title, None, None)
    return 0


if __name__ == '__main__':
    sys.exit(main())
