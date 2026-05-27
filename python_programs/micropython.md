I need my Python library to also work in micropython on an ESP32-S3 board. The library controls a servomotor by communicating with it over a serial port. The Python library that currently runs on my computer needs to also run on the ESP32-S3. You can find all the code to the library here. At the moment, I especial need ball_juggling_demo.py to run on that ESP32-S3. The main thing that is different is the way in which we communicate on the serial port. In the normal Python library we use a thing called pyserial I think. In Micropython, we use code like this:

from machine import UART, Pin
import time

# Configure the secondary UART interface.
uart1 = UART(
    1,                     # UART(1) is the “secondary” port on most boards
    baudrate=230400,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(4),             # Adjust pins to match your wiring
    rx=Pin(5)
)

payload = b'\xDE\xAD\xBE\xEF'  # Example byte sequence to transmit

while True:
    uart1.write(payload)
    time.sleep(1)
    print("sent")


I need the Python library to be portable across to a Mac / PC / Raspberry Pi / Micropython. It needs to detect what it is running on and act accordingly. Please help me update this Python library so that it runs across all these platforms. 

This project will be done when the ball_juggling_demo.py program runs successfully on the ESP32-S3 and on my Mac computer. You need to run it and test it. I am just here to help you see if the motor moved correctly or not. I expect you to self run and debug the program.