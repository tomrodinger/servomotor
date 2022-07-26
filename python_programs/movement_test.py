#!/usr/local/bin/python3

import sys
import serial
import math
import time
import random

#SERIAL_DEVICE = "/dev/tty.SLAB_USBtoUART"
SERIAL_DEVICE = "/dev/tty.usbserial-1410"
MY_AXIS = 'X'

MOVEMENT_TIME = 2.0

DISABLE_MOSFETS_COMMAND = 0
ENABLE_MOSFETS_COMMAND = 1
SET_POSITION_AND_MOVE_COMMAND = 2
SET_VELOCITY_COMMAND = 3
SET_POSITION_AND_FINISH_TIME_COMMAND = 4
SET_ACCELERATION_COMMAND = 5
START_CALIBRATION_COMMAND = 6
CAPTURE_HALL_SENSOR_DATA_COMMAND = 7
RESET_TIME_COMMAND = 8
GET_CURRENT_TIME_COMMAND = 9
TIME_SYNC_COMMAND = 10
GET_N_ITEMS_IN_QUEUE_COMMAND = 11

MOVEMENT_QUEUE_SIZE = 16

def read_and_print(ser):
    data = ser.read(1000)
    print("Received %d bytes" % (len(data)))
    print(data.decode())

    for d in data:
        print("0x%02X %d" % (d, d))

def wait_for_ok(ser):
    data = ser.read(2)
    if data.decode() == "Ok":
        print("Ok")
    else:
        print("Error: didn't receive Ok")
        data2 = ser.read(1000)
        print("Instead, received: [%s]" % (data.decode() + data2.decode()))
        exit(1)

def send_set_position_and_move_command(position):
    position_bytes = int(position).to_bytes(4, 'little')
    command = [ord(MY_AXIS), SET_POSITION_AND_MOVE_COMMAND, 4]
    command.extend(position_bytes)
    print("Sending command:", command)
    ser.write(bytearray(command))
    wait_for_ok(ser)


def send_set_position_and_finish_time_command(position, finish_time):
    position_bytes = position.to_bytes(4, 'little', signed = True)
    time_bytes = int(finish_time).to_bytes(4, 'little')
    command = [ord(MY_AXIS), SET_POSITION_AND_FINISH_TIME_COMMAND, 8]
    command.extend(position_bytes)
    command.extend(time_bytes)
    print("Sending command:", command)
    ser.write(bytearray(command))
    wait_for_ok(ser)


def send_reset_time_command():
    global master_start_time
    master_start_time = time.time()
    ser.write(bytearray([ord(MY_AXIS), RESET_TIME_COMMAND, 4, 0, 0, 0, 0]))
    wait_for_ok(ser)


def send_clock_sync_command():
    master_time = round((time.time() - master_start_time) * 1000000)
    print("Current time in microseconds:", master_time)
    master_time = int(master_time).to_bytes(4, 'little')
    command = [ord(MY_AXIS), TIME_SYNC_COMMAND, 4]
    command.extend(master_time)
    print("Sending command:", command)
    ser.write(bytearray(command))

    data = ser.read(6)
    if len(data) != 6:
        print("Did not receive the 6 byte response")
        exit(1)
    time_error = int.from_bytes(data[0:4], byteorder='little', signed=True)
    register_value = int.from_bytes(data[4:6], byteorder='little', signed=False)
    hsitrim = int.from_bytes(data[5:6], byteorder='little', signed=False)
    print("time_error: %d   register_value: %d  HSITRIM: %d" % (time_error, register_value, hsitrim))


def send_n_items_in_queue_command():
    command = [ord(MY_AXIS), GET_N_ITEMS_IN_QUEUE_COMMAND, 0]
    print("Sending command:", command)
    ser.write(bytearray(command))

    data = ser.read(1)
    if len(data) != 1:
        print("Did not receive the 1 byte response")
        exit(1)
    n_items_in_queue = int.from_bytes(data[0:1], byteorder='little', signed=False)
    return n_items_in_queue


if len(sys.argv) != 2:
    print("Usage: %s delay-between-commands" % (sys.argv[0]))
    exit(1)
    
try:
    delay_between_commands = float(sys.argv[1])
except:
    print("Error: an invalid delay between commands was specified: %s" % (sys.argv[1]))
    exit(1)

ser = serial.Serial(SERIAL_DEVICE, 230400, timeout = 0.5)  # open serial port
print("Opened this serial port:", ser.name)         # check which port was really used

print("Sending a bunch of zeros to the bus to clear out any possible corrupted commands")
ser.write(bytearray(256))

send_reset_time_command()

finish_time = 0.0

position = 0
while(1):
    random_position_change = 1000000
    if random.randint(0, 1) == 0:
        random_position_change = -random_position_change
    position = position + random_position_change
    print("Position:", position)
    #    send_set_position_and_move_command(position)
    n_items_in_queue = send_n_items_in_queue_command()
    print("n_items_in_queue:", n_items_in_queue)

    finish_time = round((time.time() - master_start_time + MOVEMENT_TIME) * 1000000)
    send_set_position_and_finish_time_command(position, finish_time)

    time.sleep(MOVEMENT_TIME + delay_between_commands)


ser.close()
