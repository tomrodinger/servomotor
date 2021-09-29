import serial

ser = serial.Serial('/dev/tty.SLAB_USBtoUART', 230400, timeout = 10)  # open serial port
print(ser.name)         # check which port was really used

data = ser.read(1000)
print("Received %d bytes" % (len(data)))
for d in data:
    print("%c %d" % (d, ord(d)))

ser.close()
