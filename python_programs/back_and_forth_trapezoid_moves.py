#!/usr/local/bin/python3

import serial
import platform
import time

PORT = ".usbserial-5"

if platform.system() == 'Windows':
    PORT_PREFIX = '\\\\.\\'
else:
    PORT_PREFIX = '/dev/tty'

try:
    serialPort = serial.Serial(PORT_PREFIX+PORT,
                               230400,
                               timeout=1)
except:
    # print(NameError)
    print('Could not open serial port.')
    exit()

print(serialPort.name)

while True:
    serialPort.write(bytearray([0x58, 0x02, 0x08, 0x00, 0x28, 0xF6, 0xFF, 0x09, 0x3D, 0x00, 0x00]))
    data = serialPort.read(1000)
    print("Received %d bytes" % (len(data)))
    print(data)
    for d in data:
        print("0x%02X %d" % (d, d))
    time.sleep(0.5)
    serialPort.write(bytearray([0x58, 0x02, 0x08, 0x00, 0xD8, 0x09, 0x00, 0x09, 0x3D, 0x00, 0x00]))
    data = serialPort.read(1000)
    print("Received %d bytes" % (len(data)))
    print(data)
    for d in data:
        print("0x%02X %d" % (d, d))
    time.sleep(0.5)


serialPort.close()
