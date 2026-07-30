import os
import sys
from .vendor import serial
from .vendor.serial.tools import list_ports

SAVED_SERIAL_DEVICE_FILENAME = "serial_device.txt"

# Environment variable that names the serial port to use. It outranks the saved
# default but is outranked by an explicit -p / -P on the command line.
SERIAL_PORT_ENV_VAR = "SERVOMOTOR_PORT"


def get_config_dir():
    """Return the per-user directory where this library keeps its settings.

    The saved port used to live inside the installed package directory, which is
    wrong in three ways: it is not writable on a normal system-wide install, it is
    wiped by every upgrade, and it is shared by every user of the machine. Use the
    conventional per-user location for each platform instead.

    On Windows this deliberately uses LOCALAPPDATA rather than APPDATA: APPDATA
    roams between machines in a domain, and a COM port number means nothing on a
    different machine.
    """
    # Honour XDG_CONFIG_HOME wherever it is set, including on macOS -- people who
    # set it have done so on purpose.
    xdg = os.environ.get("XDG_CONFIG_HOME")
    if xdg:
        return os.path.join(xdg, "servomotor")
    if sys.platform.startswith("win"):
        base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
        return os.path.join(base, "servomotor")
    if sys.platform == "darwin":
        return os.path.join(os.path.expanduser("~"), "Library", "Application Support", "servomotor")
    return os.path.join(os.path.expanduser("~"), ".config", "servomotor")


def get_saved_port_path():
    """Full path of the file that remembers the last successfully opened port."""
    return os.path.join(get_config_dir(), SAVED_SERIAL_DEVICE_FILENAME)


def _legacy_saved_port_path():
    """The pre-0.12.2 location: inside the installed package directory."""
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), SAVED_SERIAL_DEVICE_FILENAME)


def read_saved_port():
    """Return the remembered port name, or None.

    Falls back to the old in-package location so that users upgrading from an
    earlier version do not silently lose their saved default.
    """
    for path in (get_saved_port_path(), _legacy_saved_port_path()):
        try:
            with open(path, "r") as f:
                name = f.read().strip()
            if name:
                return name
        except OSError:
            continue
    return None


def save_port(port_name):
    """Remember port_name for next time. Returns True on success.

    Never raises: remembering the port is a convenience, and the caller already
    has a working port in hand. A read-only or unwritable location must not turn
    into a failed run.
    """
    path = get_saved_port_path()
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(port_name)
        return True
    except OSError as e:
        print("WARNING: Could not save the serial port name to:", path)
        print("         Reason:", e)
        print("         This is harmless. The port is open and this program will continue.")
        print("         Pass -p to name the port each time, or set %s." % SERIAL_PORT_ENV_VAR)
        return False


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
    try:
        serial_port = serial.Serial(device_name, baud_rate, timeout=timeout)
        print("Successfully opened the serial port:", serial_port.name)
    except serial.SerialException as e:
        print("Failed to open the serial port:", device_name)
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
            if (errno == 13) and not sys.platform.startswith("win"):
                # By far the most common first-run problem on Linux: serial devices are
                # owned by the dialout (Debian/Ubuntu) or uucp (Arch/Fedora) group.
                print("   Permission was denied on the device. On Linux you usually need to be in the")
                print("   'dialout' group (or 'uucp' on some distributions). To fix this, run:")
                print("       sudo usermod -a -G dialout $USER")
                print("   then log out and back in (a reboot also works) for the change to take effect.")
            exit(1)
        serial_port = None
    return serial_port

def open_serial_port(device_name = None, baud_rate = 230400, timeout = 0.1):
    """Open the serial port, working out which port to use.

    Precedence, highest first -- the more explicit and more recent the intent,
    the higher it wins:

      1. device_name  -- an explicit -p on the command line, or "MENU" for -P
      2. $SERVOMOTOR_PORT
      3. the saved default from the last successful run
      4. ask the user to pick from a menu

    -P ("MENU") deliberately jumps straight to the menu: the user asked to choose,
    so neither the environment variable nor the saved file may pre-empt that.
    """
    device_name_already_saved = None

    if device_name is None:
        env_port = os.environ.get(SERIAL_PORT_ENV_VAR)
        if env_port:
            device_name = env_port.strip()
            print("Using the serial port named by %s: %s" % (SERIAL_PORT_ENV_VAR, device_name))

    if device_name is None:
        device_name = read_saved_port()
        device_name_already_saved = device_name
        if device_name is None:
            print("No serial port specified with -p, no %s set, and no saved default." % SERIAL_PORT_ENV_VAR)
        else:
            print("Using the saved serial port:", device_name)
            print("(saved at %s)" % get_saved_port_path())

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

    # The port is open and working, so remember it as the new default. Skip the write
    # when it already matches what is stored, so a normal run does no disk I/O.
    if serial_port.name != device_name_already_saved:
        if save_port(serial_port.name):
            print("Saved the serial device name to:", get_saved_port_path())
            print("This serial port name will be used by default from now on.")
            print("To override it: pass -p, or -P to choose from a menu, or set %s." % SERIAL_PORT_ENV_VAR)

    return serial_port
