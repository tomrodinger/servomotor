#!/usr/bin/env python3

import servomotor
import time
import argparse
import matplotlib.pyplot as plt

OUT_BIN_FILENAME = "hall_calibration_raw_data.bin"
OUT_TEXT_FILENAME = "hall_calibration_one_rotation.txt"

def plot_data(data):
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
    plt.show()

def calculate_crc32(data):
    """Calculate CRC32 checksum for a byte array"""
    import zlib
    return zlib.crc32(data) & 0xffffffff


arg_parser = argparse.ArgumentParser(description="Capture and Plot Hall Sensor Data")
arg_parser.add_argument('-a', '--alias', metavar='ALIAS', help='Alias of the device to capture data from (for example: X, Y, Z, E, 0, 1, etc.)')
arg_parser.add_argument('-p', '--port', help='serial port device', default=None)
arg_parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
arg_parser.add_argument('-v', '--verbose', help='print verbose messages', action='store_true')
arg_parser.add_argument('--no-graph', action='store_true', help="Disable the plotting of the graph")
args = arg_parser.parse_args()

if args.verbose:
    verbose_level = 2
else:
    verbose_level = 0

motor = servomotor.M3(args.alias, verbose=verbose_level)
servomotor.set_serial_port_from_args(args)
servomotor.open_serial_port()

motor.system_reset()
time.sleep(1.5)

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
    exit(1)

with open(OUT_BIN_FILENAME, "wb") as fh:
    fh.write(data)

i = 0
with open(OUT_TEXT_FILENAME, "w") as fh:
    while(i < len(data)):
        hall1 = int.from_bytes(data[i + 0: i + 2], byteorder = "little", signed=False)
        hall2 = int.from_bytes(data[i + 2: i + 4], byteorder = "little", signed=False)
        hall3 = int.from_bytes(data[i + 4: i + 6], byteorder = "little", signed=False)
        if (hall1 != 0) or (hall2 != 0) or (hall3 != 0):
            s = "%5d %5d %5d\n" % (hall1, hall2, hall3)
            fh.write(s)
            print(s)
        i = i + 6
    
servomotor.close_serial_port()

with open("MY_AXIS", "w") as fh:
    fh.write(args.alias)

# Reading the generated text file and preparing the data
if not args.no_graph:
    file_data = []

    with open(OUT_TEXT_FILENAME, 'r') as fh:
        for line in fh:
            values = list(map(int, line.split()))
            file_data.append(values)

    plot_data(file_data)
