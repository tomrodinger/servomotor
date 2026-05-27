#!/usr/bin/env python3

import servomotor
from servomotor.device_detection import detect_devices_iteratively
import time
import argparse
import matplotlib.pyplot as plt
import numpy as np
import os

# Number of detection iterations for autodetection
N_DETECTION_ITERATIONS = 6

# Output directory for hall sensor data files
HALL_DATA_DIR = "hall_sensor_data_no_avg"

OUT_BIN_FILENAME = "hall_calibration_raw_data_no_avg.bin"
OUT_TEXT_FILENAME = "hall_calibration_one_rotation_no_avg.txt"

# Link & firmware constants (must match firmware).
RS485_BAUD_RATE = 230400               # RS485.c:159 sets BRR=278 at 64MHz SYSCLK -> 230400 bps
UART_BITS_PER_BYTE = 10                # 8N1: 1 start + 8 data + 1 stop
# The capture command's timeStepsPerSample counts TIM16 ISR ticks (= PWM_FREQUENCY),
# NOT motor_control.h's TIME_STEPS_PER_SECOND (which is PWM_FREQUENCY >> 1 = 15625 Hz).
# capture_logic is called from motor_movement_calculations -> TIM16_IRQHandler, which
# fires at PWM_FREQUENCY = PWM_CLOCK_FREQUENCY / PWM_PERIOD_TIM16 = 64MHz / 2048 = 31250 Hz.
# This matches the runtime value returned by get_update_frequency() in motor_control.c.
TIM16_ISR_RATE_HZ = 31250
HALL_PEAK_FIND_THRESHOLD = 2000        # motor_control.c (M17/M23): threshold used by the calibration peak-finder

def plot_data(data, save_png=False, png_filename=None):
    col1, col2, col3 = zip(*data)

    X = list(range(len(data)))

    fig, axs = plt.subplots(4, 1, figsize=(8, 12))

    axs[0].plot(X, col1)
    axs[0].set_title('Column 1')
    axs[0].set_ylabel('Y-axis value')

    axs[1].plot(X, col2)
    axs[1].set_title('Column 2')
    axs[1].set_ylabel('Y-axis value')

    axs[2].plot(X, col3)
    axs[2].set_title('Column 3')
    axs[2].set_ylabel('Y-axis value')

    axs[3].plot(X, col1, label="Column 1")
    axs[3].plot(X, col2, label="Column 2")
    axs[3].plot(X, col3, label="Column 3")
    axs[3].set_title('Hall Calibration Data (no averaging)')
    axs[3].set_xlabel('Row number')
    axs[3].set_ylabel('Y-axis value')

    plt.tight_layout()

    if save_png and png_filename:
        # Create hall_sensor_scans directory if it doesn't exist
        os.makedirs('hall_sensor_scans', exist_ok=True)

        # Save the plot as PNG
        png_path = os.path.join('hall_sensor_scans', png_filename)
        plt.savefig(png_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved as: {png_path}")

    if not save_png:
        plt.show()
    else:
        plt.close()  # Close the figure to free memory when saving

def calculate_crc32(data):
    """Calculate CRC32 checksum for a byte array"""
    import zlib
    return zlib.crc32(data) & 0xffffffff


def parse_hall_samples(data):
    """Parse the raw 6-byte-per-point payload into an (N, 3) uint16 numpy array."""
    n_points = len(data) // 6
    arr = np.frombuffer(data[:n_points * 6], dtype=np.uint16).reshape(n_points, 3)
    return arr


def print_capture_statistics(samples, total_bytes, elapsed_s,
                              n_points_requested, time_steps_per_sample,
                              n_samples_to_sum, division_factor,
                              channels_bitmask, bytes_per_point):
    """Print communication, link-utilization and per-channel noise statistics."""

    n_points = samples.shape[0]
    expected_capture_s = (n_points * time_steps_per_sample) / TIM16_ISR_RATE_HZ
    configured_sample_rate_hz = TIM16_ISR_RATE_HZ / time_steps_per_sample
    measured_sample_rate_hz = n_points / elapsed_s if elapsed_s > 0 else float('nan')
    max_uart_bytes_per_s = RS485_BAUD_RATE / UART_BITS_PER_BYTE
    payload_bytes_per_s = total_bytes / elapsed_s if elapsed_s > 0 else float('nan')
    payload_bits_per_s = payload_bytes_per_s * UART_BITS_PER_BYTE
    pct_of_baud = (payload_bits_per_s / RS485_BAUD_RATE) * 100.0

    print()
    print("=" * 68)
    print("CAPTURE STATISTICS")
    print("=" * 68)
    print("Capture parameters:")
    print(f"  captureType                = 1 (raw hall sensor readings)")
    print(f"  nPointsToRead              = {n_points_requested}")
    print(f"  channelsToCaptureBitmask   = 0x{channels_bitmask:02X} ({bin(channels_bitmask)})")
    print(f"  timeStepsPerSample         = {time_steps_per_sample}")
    print(f"  nSamplesToSum              = {n_samples_to_sum}  (1 = NO averaging)")
    print(f"  divisionFactor             = {division_factor}")
    print()
    print("Timing:")
    print(f"  Configured sample rate     = {configured_sample_rate_hz:9.3f} Hz "
          f"(TIM16 ISR rate {TIM16_ISR_RATE_HZ} Hz / timeStepsPerSample={time_steps_per_sample})")
    print(f"  Theoretical capture time   = {expected_capture_s:9.3f} s")
    print(f"  Measured elapsed time      = {elapsed_s:9.3f} s")
    if expected_capture_s > 0:
        overhead_s = elapsed_s - expected_capture_s
        print(f"  Overhead vs theoretical    = {overhead_s:+9.3f} s "
              f"({(overhead_s / expected_capture_s) * 100:+.2f} %)")
    print(f"  Measured point rate        = {measured_sample_rate_hz:9.3f} Hz")
    print()
    print("Link utilization:")
    print(f"  RS485 baud rate            = {RS485_BAUD_RATE} bps "
          f"({UART_BITS_PER_BYTE} bits/byte 8N1 -> max {max_uart_bytes_per_s:.0f} B/s payload)")
    print(f"  Points received            = {n_points}")
    print(f"  Bytes per point            = {bytes_per_point}")
    print(f"  Total payload bytes        = {total_bytes}")
    print(f"  Effective payload rate     = {payload_bytes_per_s:9.1f} B/s "
          f"= {payload_bits_per_s:9.0f} bps  ({pct_of_baud:5.2f} % of baud)")
    print()

    n_channels = samples.shape[1]
    channel_labels = [f"Hall{i+1}" for i in range(n_channels)]

    print("Per-channel value statistics:")
    print(f"  {'':12s}" + "".join(f"{lbl:>12s}" for lbl in channel_labels))
    for name, fn in [
        ("min",      lambda c: int(c.min())),
        ("max",      lambda c: int(c.max())),
        ("peak-pk",  lambda c: int(c.max() - c.min())),
        ("mean",     lambda c: float(c.mean())),
        ("median",   lambda c: float(np.median(c))),
        ("std dev",  lambda c: float(c.std())),
    ]:
        row = f"  {name:12s}"
        for ch in range(n_channels):
            v = fn(samples[:, ch])
            if isinstance(v, float):
                row += f"{v:12.2f}"
            else:
                row += f"{v:12d}"
        print(row)
    print()

    # First-difference statistics: during motion, consecutive samples differ by
    # (deterministic signal change + noise). A noise spike that exceeds the peak-find
    # threshold could fire a phantom extremum during calibration.
    deltas = np.diff(samples.astype(np.int32), axis=0)
    abs_deltas = np.abs(deltas)

    print("Sample-to-sample |Δ| statistics (signal change + noise per step):")
    print(f"  {'':16s}" + "".join(f"{lbl:>12s}" for lbl in channel_labels))
    for name, fn in [
        ("max |Δ|",      lambda c: int(np.abs(c).max())),
        ("mean |Δ|",     lambda c: float(np.abs(c).mean())),
        ("99th-pct |Δ|", lambda c: float(np.percentile(np.abs(c), 99))),
        ("std of Δ",     lambda c: float(c.std())),
    ]:
        row = f"  {name:16s}"
        for ch in range(n_channels):
            v = fn(deltas[:, ch])
            if isinstance(v, float):
                row += f"{v:12.2f}"
            else:
                row += f"{v:12d}"
        print(row)
    print()

    print(f"Comparison vs HALL_PEAK_FIND_THRESHOLD = {HALL_PEAK_FIND_THRESHOLD} "
          "(firmware calibration peak-finder):")
    print(f"  {'threshold':16s}" + "".join(f"{lbl:>12s}" for lbl in channel_labels))
    for thr in (HALL_PEAK_FIND_THRESHOLD, HALL_PEAK_FIND_THRESHOLD // 2,
                HALL_PEAK_FIND_THRESHOLD // 4, HALL_PEAK_FIND_THRESHOLD // 10):
        row = f"  |Δ| > {thr:<9d}"
        for ch in range(n_channels):
            row += f"{int((abs_deltas[:, ch] > thr).sum()):12d}"
        print(row)
    print()

    worst_channel = int(np.argmax(abs_deltas.max(axis=0)))
    worst_value   = int(abs_deltas[:, worst_channel].max())
    pct_of_thr    = 100.0 * worst_value / HALL_PEAK_FIND_THRESHOLD
    print(f"Worst single-step deviation across all channels: {worst_value} counts "
          f"on Hall{worst_channel + 1}  ({pct_of_thr:.1f} % of peak-find threshold)")
    if worst_value > HALL_PEAK_FIND_THRESHOLD:
        print("  *** WARNING: at least one Δ exceeds the calibration peak-find threshold. "
              "This could spuriously fire ERROR_CALIBRATION_OVERFLOW. ***")
    print("=" * 68)
    print()


arg_parser = argparse.ArgumentParser(description="Capture and Plot Hall Sensor Data (no averaging)")
arg_parser.add_argument('-a', '--alias', metavar='ALIAS', help='Alias of the device to capture data from (for example: X, Y, Z, E, 0, 1, etc.)', required=False)
arg_parser.add_argument('-p', '--port', help='serial port device', default=None)
arg_parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
arg_parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
arg_parser.add_argument('--no-graph', action='store_true', help="Disable the plotting of the graph")
arg_parser.add_argument('--save-png', action='store_true', help="Save the plot as PNG file in hall_sensor_scans directory instead of displaying it")

args = arg_parser.parse_args()

# Check if alias is provided or if we should autodetect
if not args.alias:
    print("No alias specified. Will autodetect all devices and capture data from each.")

def capture_hall_sensor_data_for_device(device_identifier, verbose_level, args, filename_identifier=None):
    """Capture hall sensor data for a single device with NO averaging.

    Each returned 16-bit value is one raw get_hall_sensor*_voltage() reading
    (a sum of 4 ADC samples from the DMA buffer) - the exact same number the
    firmware calibration peak-finder compares against the 2000-count threshold.

    Args:
        device_identifier: The identifier to use for addressing the device (alias or unique_id)
        verbose_level: Verbosity level for motor communication
        args: Command line arguments
        filename_identifier: Identifier to use for filenames (defaults to device_identifier)
    """
    if filename_identifier is None:
        filename_identifier = device_identifier

    print(f"\n{'='*60}")
    print(f"Capturing hall sensor data (no averaging) for device: {device_identifier}")
    print(f"{'='*60}")

    motor = servomotor.M3(device_identifier, verbose=verbose_level)

    motor.system_reset()
    time.sleep(1.5)

    motor.set_max_allowable_position_deviation(2000000000)

    # Set the motor current to maximum (390) before enabling MOSFETs and moving.
    # Param 1 is drive current, param 2 is regen current; the latter is unused per the
    # command description, but we set it to the same value to be explicit.
    motor.set_maximum_motor_current(390, 390)

    motor.enable_mosfets()
    time.sleep(0.05)

    motor.set_position_unit("shaft_rotations")
    motor.set_time_unit("seconds")
    motor.trapezoid_move(0.06, 10.0)  # very slow: ~0.006 rev/s avg, ~5x slower than 0.03 rev/s
    time.sleep(0.3) # let the motor to ramp up to a steady velocity

    # No-averaging capture parameters, pushed for higher temporal resolution:
    #   timeStepsPerSample = 12 -> 31250/12 = 2604.2 pts/s
    #                              6 B/pt -> 15.6 kB/s (~67% of 230400 baud,
    #                              ~68% of the 23040 B/s 8N1 ceiling).
    #                              Sample interval 384us vs ~260us UART for 6B -> ~32% margin.
    #   nSamplesToSum     = 1  -> NO averaging
    #   divisionFactor    = 1  -> raw single get_hall_sensor*_voltage() reading per point
    #   nPointsToRead     = 10000 -> ~3.84 s of capture; total payload 60000 bytes
    #                                (well under the firmware's ~65526-byte cap)
    captureType = 1
    nPointsToRead = 10000
    channelsToCaptureBitmask = 7
    timeStepsPerSample = 12
    nSamplesToSum = 1
    divisionFactor = 1

    bytes_per_point = 6  # 3 channels * 2 bytes; the bitmask is 7 so all three are enabled

    t_start = time.monotonic()
    data = motor.capture_hall_sensor_data(captureType,
                                          nPointsToRead,
                                          channelsToCaptureBitmask,
                                          timeStepsPerSample,
                                          nSamplesToSum,
                                          divisionFactor)
    elapsed_s = time.monotonic() - t_start

    print(f"Received {len(data)} bytes from the device.")

    if len(data) % 6 != 0:
        print("Error: The length of the data is not a multiple of 6")
        print("The length of the data is:", len(data))
        return False

    samples = parse_hall_samples(data)
    print_capture_statistics(samples,
                             total_bytes=len(data),
                             elapsed_s=elapsed_s,
                             n_points_requested=nPointsToRead,
                             time_steps_per_sample=timeStepsPerSample,
                             n_samples_to_sum=nSamplesToSum,
                             division_factor=divisionFactor,
                             channels_bitmask=channelsToCaptureBitmask,
                             bytes_per_point=bytes_per_point)

    # Create hall_sensor_data_no_avg directory if it doesn't exist
    os.makedirs(HALL_DATA_DIR, exist_ok=True)

    # Create device-specific filenames in the data directory
    bin_filename = os.path.join(HALL_DATA_DIR, f"hall_calibration_raw_data_no_avg_{filename_identifier}.bin")
    text_filename = os.path.join(HALL_DATA_DIR, f"hall_calibration_one_rotation_no_avg_{filename_identifier}.txt")

    with open(bin_filename, "wb") as fh:
        fh.write(data)

    i = 0
    with open(text_filename, "w") as fh:
        while(i < len(data)):
            hall1 = int.from_bytes(data[i + 0: i + 2], byteorder = "little", signed=False)
            hall2 = int.from_bytes(data[i + 2: i + 4], byteorder = "little", signed=False)
            hall3 = int.from_bytes(data[i + 4: i + 6], byteorder = "little", signed=False)
            if (hall1 != 0) or (hall2 != 0) or (hall3 != 0):
                s = "%5d %5d %5d\n" % (hall1, hall2, hall3)
                fh.write(s)
            i = i + 6

    # Reading the generated text file and preparing the data
    if not args.no_graph:
        file_data = []

        with open(text_filename, 'r') as fh:
            for line in fh:
                values = list(map(int, line.split()))
                file_data.append(values)

        if args.save_png:
            # Generate PNG filename based on filename_identifier
            png_filename = f"{filename_identifier}_no_avg.png"
            plot_data(file_data, save_png=True, png_filename=png_filename)
        else:
            plot_data(file_data)

    print(f"Hall sensor data capture (no averaging) completed for device: {device_identifier}")
    return True


if args.verbose:
    verbose_level = 2
else:
    verbose_level = 0

servomotor.set_serial_port_from_args(args)
servomotor.open_serial_port()

try:
    if args.alias:
        # Single device mode - original behavior using alias
        print(f"Capturing hall sensor data (no averaging) for specified device: {args.alias}")
        capture_hall_sensor_data_for_device(args.alias, verbose_level, args)

        # Write the alias to MY_AXIS file for backward compatibility
        with open("MY_AXIS", "w") as fh:
            fh.write(args.alias)
    else:
        # Autodetection mode - capture data from all devices using unique IDs
        print("Starting device autodetection...")

        # Detect devices
        devices = detect_devices_iteratively(n_detections=N_DETECTION_ITERATIONS, verbose=args.verbose)

        if not devices:
            print("No devices were detected. Exiting.")
            exit(1)

        print(f"\nDetected {len(devices)} device(s):")
        for i, device in enumerate(devices, 1):
            alias_str = servomotor.get_human_readable_alias_or_unique_id(device.alias)
            print(f"  {i}. Unique ID: {device.unique_id:016X}, Alias: {alias_str}")

        # Capture data from each device using unique ID
        successful_captures = 0
        for device in devices:
            unique_id_hex = f"{device.unique_id:016X}"
            try:
                if capture_hall_sensor_data_for_device(device.unique_id, verbose_level, args, unique_id_hex):
                    successful_captures += 1
            except Exception as e:
                print(f"Error capturing data from device {unique_id_hex}: {e}")
                continue

        print(f"\n{'='*60}")
        print(f"SUMMARY: Successfully captured data from {successful_captures}/{len(devices)} devices")
        print(f"{'='*60}")

        # Write the last device unique ID to MY_AXIS file for backward compatibility
        if devices:
            last_unique_id = f"{devices[-1].unique_id:016X}"
            with open("MY_AXIS", "w") as fh:
                fh.write(last_unique_id)

finally:
    servomotor.close_serial_port()
