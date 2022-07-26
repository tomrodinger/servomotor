#!/usr/local/bin/python3

import serial

#ser = serial.Serial('/dev/tty.SLAB_USBtoUART', 230400, timeout = 0.1)  # open serial port
ser = serial.Serial('/dev/tty.usbserial-1420', 230400, timeout = 0.05)  # open serial port

print(ser.name)         # check which port was really used
ser.write(bytearray([255, 27, 0]))


data = ser.read(1000)
print("Received %d bytes" % (len(data)))
print(data)

for d in data:
    print("0x%02X %d" % (d, d))

ser.close()
