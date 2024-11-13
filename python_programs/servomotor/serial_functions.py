import os
from .vendor import serial
from .vendor.serial.tools import list_ports

SAVED_SERIAL_DEVICE_FILENAME = "serial_device.txt"


def select_serial_port_from_menu():
    while(1):
        print("Here are the current serial ports detected on your computer:")
        print("Now getting the list")
        ports = list(list_ports.comports())
        for i in range(len(ports)):
            p = ports[i][0]
            print("%d. %s" % (i + 1, p))
        # get an integer input from the user
        print("Enter the number of the serial port you want to use,\nor press enter to select the last one listed,\nor type q <ENTER> to exit this program,\nor type r <ENTER> to refresh the serial port list")
        user_input_string = input("> ").strip()
        # check if the input is an integer between 1 and the number of ports
        if user_input_string == "q":
            print("Quiting")
            exit(1)
        elif user_input_string == "r":
            continue
        elif user_input_string == "":
            user_input = len(ports)
        else:
            try:
                user_input = int(user_input_string)
            except ValueError:
                print("ERROR: You did not enter an integer")
                exit(1)
            if user_input < 1 or user_input > len(ports):
                print("ERROR: You did not enter an integer between 1 and %d" % len(ports))
                exit(1)
        user_input -= 1
        break
    # print out the serial port device that the user has selected
    print("You have selected:", ports[user_input][0])
    return ports[user_input][0]


# We will attempt to open the serial port and if we fail to open it then we will print out a detailed error message
# and/or troubleshooting tips
def open_serial_port_or_print_detailed_error(device_name = None, baud_rate = 230400, timeout = 0.1):
    print("Opening the serial device:", device_name)
    try:
        serial_port = serial.Serial(device_name, baud_rate, timeout=timeout)
    except serial.SerialException as e:
        # Print out the specific error message
        print("Error:", e)
        errno = e.errno
        # check for a resource busy error
        if errno == 16:
            print("Error: The serial port is busy. This could be because the port is already open by another program.")
        else:
            print("*** Troubleshooting steps: ***")
            print("   Make sure that the hardware is connected properly and powered on")
            print("   Make sure that the serial port name is correct. This changes sometimes (especially if plugged into a different USB port)")
            print("   You can run this program with the -P option to list available serial ports on the system and then select a serial port from a menu")
            exit(1)
        serial_port = None
    return serial_port

# Open a serial port with the given device name and baud rate and timeout
# If the port is already opened or if it cannot be opened then give an error and exit
# If the device name is None then get the device name from a file called "serial_device.txt" from
# the current directory
def open_serial_port(device_name = None, baud_rate = 230400, timeout = 0.1):
    # get the full path of the python program that is executing right now
    current_dir = os.path.dirname(os.path.realpath(__file__))
    device_file_path = current_dir + "/" + SAVED_SERIAL_DEVICE_FILENAME
    device_name_from_file = None
#    device_file_path = open_serial_port.__code__.co_filename.replace("serial_functions.py", SAVED_SERIAL_DEVICE_FILENAME)

    if device_name == None:
        try:
            print("You have not specified a serial port using the -p opton. So, getting the serial port name from this file:", device_file_path)
            device_name = open(device_file_path, "r").read().strip()
            device_name_from_file = device_name
        except FileNotFoundError:
            print("Could not open that file")
            device_name = None

    if (device_name == None) or (device_name == "MENU"):
        print("Will let the user select the serial port from a menu")
        device_name = select_serial_port_from_menu()

    serial_port = open_serial_port_or_print_detailed_error(device_name, baud_rate, timeout)
    if serial_port == None:
        device_name = select_serial_port_from_menu()
        # now that we have gotten a serial port, let's try to open it
        serial_port = open_serial_port_or_print_detailed_error(device_name, baud_rate, timeout)
        if serial_port == None:
            exit(1)
    print("Successfully opened:", serial_port.name)

    if(device_name != device_name_from_file):
        # we have successfully opened the serial port at this point, and now we want to save the device name
        # to the file in the current directory with the name specified in SAVED_SERIAL_DEVICE_FILENAME
        try:
            open(device_file_path, "w").write(serial_port.name)
            print("Saved the serial device name to the file at:", device_file_path)
            print("This serial port name will be used by default from now on.")
            print("To set a new default, you need to run this program with the -p option to specify the port you want")
            print("or run this program with the -P option to select a new default port from a menu.")
        except FileNotFoundError:
            print("ERROR: Could not open the file for writing (could not save the serial port name):", device_file_path)
            
    return serial_port
