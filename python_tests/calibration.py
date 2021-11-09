#!/usr/local/bin/python3

import serial
import time

MY_AXIS = 'X'
CALIBRATION_DONE_TEXT = bytearray(b'Calibration done\n')

ser = serial.Serial('/dev/tty.SLAB_USBtoUART', 230400, timeout = 0.1)  # open serial port
#ser = serial.Serial('/dev/tty.usbserial-1410', 230400, timeout = 0.1)  # open serial port

print(ser.name)         # check which port was really used
print("Sending the command to enable the motor")
command = bytearray([ord(MY_AXIS), 1, 0])
ser.write(command)
time.sleep(0.1)

print("Sending the command to start the calibration motor movements")
command = bytearray([ord(MY_AXIS), 6, 0])
ser.write(command)

data = bytearray()
while 1:
    some_data = ser.read(1000000)
    data = data + some_data
    end_index = data.find(CALIBRATION_DONE_TEXT, 0)
    if end_index >= 0:
        break

ser.close()
