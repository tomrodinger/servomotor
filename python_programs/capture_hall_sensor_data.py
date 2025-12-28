#!/usr/bin/env python3

import servomotor
from servomotor.device_detection import detect_devices_iteratively
import time
import argparse
import matplotlib.pyplot as plt
import os

# Number of detection iterations for autodetection
N_DETECTION_ITERATIONS = 6

# Output directory for hall sensor data files
HALL_DATA_DIR = "hall_sensor_data"

OUT_BIN_FILENAME = "hall_calibration_raw_data.bin"
OUT_TEXT_FILENAME = "hall_calibration_one_rotation.txt"

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
    axs[3].set_title('Hall Calibration Data')
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


arg_parser = argparse.ArgumentParser(description="Capture and Plot Hall Sensor Data")
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
    """Capture hall sensor data for a single device.
    
    Args:
        device_identifier: The identifier to use for addressing the device (alias or unique_id)
        verbose_level: Verbosity level for motor communication
        args: Command line arguments
        filename_identifier: Identifier to use for filenames (defaults to device_identifier)
    """
    if filename_identifier is None:
        filename_identifier = device_identifier
        
    print(f"\n{'='*60}")
    print(f"Capturing hall sensor data for device: {device_identifier}")
    print(f"{'='*60}")
    
    motor = servomotor.M3(device_identifier, verbose=verbose_level)
    
    motor.system_reset()
    time.sleep(1.5)

    motor.set_max_allowable_position_deviation(2000000000)
    
    motor.enable_mosfets()
    time.sleep(0.05)

    motor.set_position_unit("shaft_rotations")
    motor.set_time_unit("seconds")
    motor.trapezoid_move(1.2, 8.5)
    time.sleep(0.2) # let the motor to speed up to a steady velocity

    captureType = 1
    nPointsToRead = 4000
    channelsToCaptureBitmask = 7
    timeStepsPerSample = 1
    nSamplesToSum = 64
    divisionFactor = 16

    data = motor.capture_hall_sensor_data(captureType,
                                          nPointsToRead,
                                          channelsToCaptureBitmask,
                                          timeStepsPerSample,
                                          nSamplesToSum,
                                          divisionFactor)

    print("The data in the response payload:", data)
    print("The length of the data:", len(data))

    if len(data) % 6 != 0:
        print("Error: The length of the data is not a multiple of 6")
        print("The length of the data is:", len(data))
        return False

    # Create hall_sensor_data directory if it doesn't exist
    os.makedirs(HALL_DATA_DIR, exist_ok=True)
    
    # Create device-specific filenames in the data directory
    bin_filename = os.path.join(HALL_DATA_DIR, f"hall_calibration_raw_data_{filename_identifier}.bin")
    text_filename = os.path.join(HALL_DATA_DIR, f"hall_calibration_one_rotation_{filename_identifier}.txt")

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
                print(s)
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
            png_filename = f"{filename_identifier}.png"
            plot_data(file_data, save_png=True, png_filename=png_filename)
        else:
            plot_data(file_data)
    
    print(f"Hall sensor data capture completed for device: {device_identifier}")
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
        print(f"Capturing hall sensor data for specified device: {args.alias}")
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
