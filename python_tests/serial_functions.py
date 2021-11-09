import os
import serial

DEFAULT_SERIAL_DEVICE = "serial_device.txt"

# Open a serial port with the given device name and baud rate and timeout
# If the port is already opened or if it cannot be opened then give an error and exit
# If the device name is None then get the device name from a file called "serial_device.txt" from
# the current directory
def open_serial_port(device_name = None, baud_rate = 230400, timeout = 0.1):
    # get the full path of the python program that is executing right now
    current_dir = os.path.dirname(os.path.realpath(__file__))
    device_file_path = current_dir + "/" + DEFAULT_SERIAL_DEVICE
    need_to_save_device_name = True
#    device_file_path = open_serial_port.__code__.co_filename.replace("serial_functions.py", DEFAULT_SERIAL_DEVICE)

    if device_name == None:
        try:
            print("You have not specified a serial port using the -p opton. So, getting the serial port name from this file:", device_file_path)
            device_name = open(device_file_path, "r").read().strip()
            need_to_save_device_name = False
        except FileNotFoundError:
            print("Could not open that file")
            device_name = "None" # this will cause an error below (it's not a bug to put quotes around None here)
    print("Opening the serial device:", device_name)
    try:
        serial_port = serial.Serial(device_name, baud_rate, timeout=timeout)
    except serial.SerialException:
        print("Could not open the serial port")
        print("Most likely, this is because the hardware is not connected properly")
        print("or that the hardware is connected but the device name has changed. It does that sometimes,")
        print("like when you have multiple serial devices connected and the kernel assigns something")
        print("like random numbers to the end of device names")
        print("Here are the current serial ports detected on your computer:")
        ports = list(serial.tools.list_ports.comports())
        for i in range(len(ports)):
            p = ports[i][0]
            print("%d. %s" % (i + 1, p))
        # get an integer input from the user
        user_input_string = input("Enter the number of the serial port you want to use, or press enter to select the last one listed: ").strip()
        # check if the input is an integer between 1 and the number of ports
        if user_input_string == "":
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
        # print out the serial port device that the user has selected
        print("You have selected:", ports[user_input][0])
        # now that we have gotten a serial port, let's try to open it
        try:
            serial_port = serial.Serial(ports[user_input][0], baud_rate, timeout=timeout)
        except serial.SerialException:
            print("ERROR: Could not open the serial port")
            exit(1)

    print("Successfully opened:", serial_port.name)

    if(need_to_save_device_name):
        # we have successfully opened the serial port at this point, and now we want to save the device name
        # to the file in the current directory with the name specified in DEFAULT_SERIAL_DEVICE
        try:
            open(device_file_path, "w").write(serial_port.name)
            print("Saved the serial device name to the file at:", device_file_path)
            print("This serial port name will be used by default from now on.")
            print("To set a new default, you need to run this program with the -p option to specify the port you want")
        except FileNotFoundError:
            print("ERROR: Could not open the file for writing (could not save the serial port name):", device_file_path)
            
    return serial_port


def open_serial_port_old(serial_device, baud_rate, timeout = 0.05):
    print("Opening serial device:", serial_device)
    try:
        ser = serial.Serial(serial_device, baud_rate, timeout = timeout)
    except:
        print("Could not open the serial port")
        print("Most likely, this is because the hardware is not connected properly")
        print("So, make sure you plugged in your USB to serial adapeter")
        print("Otherwise, make sure thet the correct serial port is defined in this program")
        print("Here are the current serial ports detected on your computer:")
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            print(p[0])
        exit(1)
    print("Opened:", ser.name)
    return ser
