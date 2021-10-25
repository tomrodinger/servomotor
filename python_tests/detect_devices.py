#!/usr/local/bin/python3

import sys
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import time

#SERIAL_PORT = "/dev/tty.SLAB_USBtoUART"
SERIAL_PORT = "/dev/tty.usbserial-1410"

DETECT_DEVICES_COMMAND = 20
SET_DEVICE_ALIAS_COMMAND = 21
GET_PRODUCT_INFO_COMMAND = 22

DETECT_DEVICES_MAX_TIME = 1.2

def open_serial_port(serial_device, baud_rate, timeout = 0.05):
    print("Opening serial device:", serial_device)
    try:
        ser = serial.Serial(serial_device, baud_rate, timeout = timeout)
    except:
        print("Could not open the serial port")
        print("Most likely, this is because the hardware is not connected properly")
        print("So, make sure you plugged in your USB to serial adapeter")
        print("Otherwise, make sure thet the correct serial port is defined in this program")
        print("Here are the current serial ports detected on your computer:")
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            print(p[0])
        exit(1)
    print("Opened:", ser.name)
    return ser


def send_detect_devices_command(ser):
    command = bytearray([255, DETECT_DEVICES_COMMAND, 0])
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)


def get_response(ser):
    response = ser.read(3)
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


ser = open_serial_port(SERIAL_PORT, 230400, 0.05)

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
