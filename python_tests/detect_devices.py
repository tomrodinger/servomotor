#!/usr/local/bin/python3

import sys
import argparse
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import time
import serial_functions


DETECT_DEVICES_COMMAND = 20
SET_DEVICE_ALIAS_COMMAND = 21
GET_PRODUCT_INFO_COMMAND = 22

DETECT_DEVICES_MAX_TIME = 1.2



def send_detect_devices_command(ser):
    command = bytearray([255, DETECT_DEVICES_COMMAND, 0])
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)


def get_response(ser):
    response = ser.read(3)
    if len(response) != 0:
        print("Receive some bytes:", response)
    if len(response) != 3:
        return None, "Error: didn't receive an expected three bytes"
    print("Received a response: ", response)
    if response[0] != 'R' and response[1] != 1:
        return None, "Error: the first two bytes are not as expected. Expecting R 1"
    payload_size = response[2]
    response = ser.read(payload_size)
    if len(response) != payload_size:
        return None, "Error: didn't receive the right length payload. Expecting length %d, got %d" % (payload_size, len(response))
    print("Got a valid response:", response)
    return response, ""


def parse_response(response):
    if len(response) != 13:
        print("The length of the response if not right. Expecting 13 bytes, got %d bytes" % (len(response)))
        return None, None
    unique_id = int.from_bytes(response[0:8], "little")
    alias = int(response[8])
    crc32 = int.from_bytes(response[9:13], "little")
    if crc32 != 0x04030201:
        print("The crc 32 check failed")
        return None, None
    return unique_id, alias

def print_data(data):
    print(data)
    for d in data:
        print("0x%02X %d" % (d, d))


def print_device_data(unique_id, alias):
    if alias >= 33 and alias <= 126:
        alias_str = "%c" % (alias)
    else:
        alias_str = "%d" % (alias)
    print("unique id: %016X and alias %s" % (unique_id, alias_str))


# Define the arguments for this program. This program takes in an optional -p option to specify the serial port device
# and it also takes a mandatory firmware file name
parser = argparse.ArgumentParser(description='Detect all devices on the RS485 bus')
parser.add_argument('-p', '--port', help='serial port device', default=None)
args = parser.parse_args()

serial_port = args.port

ser = serial_functions.open_serial_port(serial_port, 230400, 0.05)

device_dict = {}

for i in range(3):
    print("=================================================================================================================")
    send_detect_devices_command(ser)
    start_time = time.time()
    while time.time() < start_time + DETECT_DEVICES_MAX_TIME:
        response, error_message = get_response(ser)
        if response == None:
            time.sleep(0.01)
            continue
        device_unique_id, device_alias = parse_response(response)
        if device_unique_id in device_dict:
            print("Found a device that is already known")
        else:
            device_dict[device_unique_id] = device_alias
            print("Detected a new dvice:")
            print_device_data(device_unique_id, device_alias)
    print("Detection finished")

print("=================================================================================================================")
print("Here is the final device list:")
for key in device_dict.keys():
    print_device_data(key, device_dict[key])

ser.close()
