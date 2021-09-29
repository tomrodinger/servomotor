#!/usr/local/bin/python3

import sys
import serial
from serial.serialutil import to_bytes
import serial.tools.list_ports
import time

SERIAL_PORT = "/dev/tty.SLAB_USBtoUART"
#SERIAL_PORT = "/dev/tty.usbserial-1420"

DETECT_DEVICES_COMMAND = 20
SET_DEVICE_ALIAS_COMMAND = 21
GET_PRODUCT_INFO_COMMAND = 22
MOVE_WITH_ACCELERATION_COMMAND = 19

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


def send_move_with_acceleration_command(ser, acceleration, time_steps):
    command = bytearray([ord('X'), MOVE_WITH_ACCELERATION_COMMAND, 8])
    command = command + acceleration.to_bytes(4, byteorder = "little", signed = True)
    command = command + time_steps.to_bytes(4, "little")
    print("Writing %d bytes" % (len(command)))
    print_data(command)
    ser.write(command)


def get_response(ser):
    response = ser.read(3)
    if len(response) != 3:
        print("Error: didn't receive enough bytes in the response")
        exit(1)
    print("Received a response: ", response)
    if response[0] != ord('R'):
        print("Error: the first is not the expected R")
        exit(1)
    payload_size = response[2]
    if payload_size == 0:
        if response[1] != 0:
            print("Error: the second byte should be 0 if there is no payload")
            exit(1)
    else:
        if response[1] != 1:
            print("Error: the second byte should be 1 if there is a payload")
            exit(1)

    payload = ser.read(payload_size)
    if len(payload) != payload_size:
        print("Error: didn't receive the right length payload")
        print("Received this payload: ", payload)
        exit(1)
    print("Got a valid payload:", payload)
    return payload


def parse_response(response):
    if len(response) != 13:
        return None, None
    unique_id = int.from_bytes(response[0:8], "little")
    alias = int(response[4])
    crc32 = int.from_bytes(response[5:9], "little")
    if crc32 != 0x04030201:
        return None, None
    return unique_id, alias


def print_data(data):
    print(data)
    for d in data:
        print("0x%02X %d" % (d, d))


def print_usage():
    print("Usage: %s acceleration time-steps" % (sys.argv[0]))
    print("For example, this is a sensible command: %s 100000 1000" % (sys.argv[0]))
    exit(1)

if len(sys.argv) != 3:
    print_usage()
    
acceleration = int(sys.argv[1])
time_steps = int(sys.argv[2])
if time_steps < 0:
    print("The time steps cannot be negative")
    exit(1)


ser = open_serial_port(SERIAL_PORT, 230400, 0.05)
send_move_with_acceleration_command(ser, acceleration, time_steps)
payload = get_response(ser)
if (payload == None) or (len(payload) != 0):
    print("Received an invalid response")
    exit(1)
print("Command succeeded")
ser.close()
