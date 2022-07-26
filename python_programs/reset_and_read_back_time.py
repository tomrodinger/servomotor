#!/usr/local/bin/python3

import serial

#SERIAL_DEVICE = "/dev/tty.SLAB_USBtoUART"
SERIAL_DEVICE = "/dev/tty.usbserial-1420"

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

def read_and_print(ser):
    data = ser.read(1000)
    print("Received %d bytes" % (len(data)))
    print(data)

    for d in data:
        print("0x%02X %d" % (d, d))


ser = serial.Serial(SERIAL_DEVICE, 230400, timeout = 0.5)  # open serial port
print(ser.name)         # check which port was really used

ser.write(bytearray([ord('E'), RESET_TIME_COMMAND, 0]))
read_and_print(ser)

ser.write(bytearray([ord('E'), GET_CURRENT_TIME_COMMAND, 0]))
read_and_print(ser)

ser.close()
