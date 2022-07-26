#!/usr/local/bin/python3

import sys

FIRMWARE_SIZE = 4096
FILENAME = "fake_firmware.bin"

print("Writing generated firmware file:", FILENAME)
print("The size will be:", FIRMWARE_SIZE)
with open(FILENAME, "wb") as fh:
    for i in range(FIRMWARE_SIZE >> 1):
        fh.write(i.to_bytes(2, "little"))
    

