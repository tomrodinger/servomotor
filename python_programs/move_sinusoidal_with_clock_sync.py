#!/usr/local/bin/python3

import sys
import serial
import math
import time

#SERIAL_DEVICE = "/dev/tty.SLAB_USBtoUART"
SERIAL_DEVICE = "/dev/tty.usbserial-1410"
MY_AXIS = 'Y'

STEPS_PER_CYCLE = 500
AMPLITUDE_OF_MOVEMENT = 3000000

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
TIME_SYNC_PERIOD = 100000  # in microseconds

previous_time_sync_time = 0

def read_and_print(ser):
    data = ser.read(1000)
    print("Received %d bytes" % (len(data)))
    print(data.decode())

    for d in data:
        print("0x%02X %d" % (d, d))

def wait_for_command_success(ser):
    data = ser.read(3)
    if data == b'\xFE\x00\x00':
        print("Command success")
    else:
        print("Error: didn't receive command success reply")
        data2 = ser.read(1000)
        print("Instead, received:", data + data2)
        exit(1)

def send_set_position_and_move_command(position):
    position_bytes = int(position).to_bytes(4, 'little')
    command = [ord(MY_AXIS), SET_POSITION_AND_MOVE_COMMAND, 4]
    command.extend(position_bytes)
    print("Sending command:", command)
    ser.write(bytearray(command))
    wait_for_command_success(ser)


def send_set_position_and_finish_time_command(position, finish_time):
    position_bytes = position.to_bytes(4, 'little', signed=True)
    time_bytes = int(finish_time).to_bytes(4, 'little')
    command = [ord(MY_AXIS), SET_POSITION_AND_FINISH_TIME_COMMAND, 8]
    command.extend(position_bytes)
    command.extend(time_bytes)
    print("Sending command:", command)
    ser.write(bytearray(command))
    wait_for_command_success(ser)


def send_reset_time_command():
    global master_start_time
    master_start_time = time.time()
    ser.write(bytearray([ord(MY_AXIS), RESET_TIME_COMMAND, 4, 0, 0, 0, 0]))
    wait_for_command_success(ser)


def send_clock_sync_command():
    master_time = round((time.time() - master_start_time) * 1000000)
    print("Current time in microseconds:", master_time)
    master_time = int(master_time).to_bytes(4, 'little')
    command = [ord(MY_AXIS), TIME_SYNC_COMMAND, 4]
    command.extend(master_time)
    print("Sending command:", command)
    ser.write(bytearray(command))

    data = ser.read(9)
    if len(data) != 9:
        print("Did not receive the 9 byte response. Got this instead:", data)
        exit(1)
    if data[0:3] != b'\xFE\x01\x06':
        print("The first three bytes in the response were not right. The full response is this:", data)
    time_error = int.from_bytes(data[3:7], byteorder='little', signed=True)
    register_value = int.from_bytes(data[7:8], byteorder='little', signed=False)
    hsitrim = int.from_bytes(data[8:9], byteorder='little', signed=False)
    print("time_error: %d   register_value: %d  HSITRIM: %d" % (time_error, register_value, hsitrim))


def send_n_items_in_queue_command():
    command = [ord(MY_AXIS), GET_N_ITEMS_IN_QUEUE_COMMAND, 0]
    print("Sending command:", command)
    ser.write(bytearray(command))

    data = ser.read(4)
    if len(data) != 4:
        print("Did not receive the 4 byte response. Instead, received this:", data)
        exit(1)
    if data[0:3] != b'\xFE\x01\x01':
        print("The first three bytes in the response were not right. The full response is this:", data)
    n_items_in_queue = int.from_bytes(data[3:4], byteorder='little', signed=False)
    return n_items_in_queue


def is_it_time_to_send_a_time_sync():
    global previous_time_sync_time
    master_time = round((time.time() - master_start_time) * 1000000)
    if master_time > previous_time_sync_time + TIME_SYNC_PERIOD:
        send_clock_sync_command()
        previous_time_sync_time = master_time

if len(sys.argv) != 2:
    print("Usage: %s one-cycle-seconds" % (sys.argv[0]))
    exit(1)
    
try:
    one_cycle_seconds = float(sys.argv[1])
except:
    print("Error: an invalid position was specified: %s" % (sys.argv[1]))
    exit(1)

ser = serial.Serial(SERIAL_DEVICE, 230400, timeout = 0.5)  # open serial port
print("Opened this serial port:", ser.name)         # check which port was really used

print("Sending a bunch of zeros to the bus to clear out any possible corrupted commands")
ser.write(bytearray(256))

send_reset_time_command()

finish_time = 0.0
time_step_microseconds = one_cycle_seconds / STEPS_PER_CYCLE * 1000000
while(1):
    for i in range(STEPS_PER_CYCLE):
        position = AMPLITUDE_OF_MOVEMENT * math.sin(float(i) / STEPS_PER_CYCLE * 2.0 * math.pi)
        position = round(position)
        print("Position:", i, position)
    #    send_set_position_and_move_command(position)
        while(1):
            n_items_in_queue = send_n_items_in_queue_command()
            print("n_items_in_queue:", n_items_in_queue)
            if(n_items_in_queue < MOVEMENT_QUEUE_SIZE):
                break
            time.sleep(min(one_cycle_seconds/STEPS_PER_CYCLE, TIME_SYNC_PERIOD))
            is_it_time_to_send_a_time_sync()

        finish_time = finish_time + time_step_microseconds
        send_set_position_and_finish_time_command(position, finish_time)
        is_it_time_to_send_a_time_sync()


ser.close()
