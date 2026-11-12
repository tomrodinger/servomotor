
## Usage

Here's a basic example of how to use the servomotor package:

```python
import servomotor

# Open the intended adapter explicitly. Use a motor unique ID on shared racks.
servomotor.set_serial_port("/dev/ttyUSB0")
servomotor.open_serial_port()
motor = servomotor.M3("0123456789ABCDEF", position_unit="encoder_counts")
try:
    kp, ki, kd = motor.get_pid_constants()  # firmware 0.16.0.0 or later
    print(kp, ki, kd)
finally:
    servomotor.close_serial_port()
```

Firmware 0.17.0.0 adds `set_immediate_position_target(targetPosition)` and
`set_immediate_position_target_and_get_position(targetPosition)`. Both require
closed-loop position mode, enabled MOSFETs, an empty trajectory queue and zero
commanded/planned velocity. Targets must satisfy the configured position and
deviation limits. These commands step the reference immediately; trajectory
velocity/acceleration limits do not profile the resulting catch-up motion.

The combined method returns the latest stored measured Hall position, which may
come from the preceding control tick. It does not wait for arrival or sample the
ADC again. With `position_unit="encoder_counts"`, Python integer targets and
responses preserve the full signed 64-bit value. For other units, conversions use
floating point. The measured position may differ from the requested target.

Broadcast calls return an empty list because motors do not reply. Use a unique
ID to obtain a measured position or PID constants. Invalid keyword arguments,
nonfinite numeric inputs and out-of-range integer fields raise exceptions before
transmitting. Device fatal errors raise `servomotor.FatalError` and require a
motor reset. Do not retry motion automatically after a communication failure.

For more detailed usage instructions, please refer to the documentation.

## Choosing the serial port

The command line tools work out which serial port to use in this order, highest
priority first:

1. `-p PORT` on the command line (`-P` instead pops up a menu of detected ports)
2. the `SERVOMOTOR_PORT` environment variable
3. the port saved from the last successful run
4. an interactive menu, if none of the above apply

So a port can be set once for a whole shell session:

```bash
export SERVOMOTOR_PORT=/dev/ttyUSB0        # Linux
export SERVOMOTOR_PORT=/dev/cu.usbserial-110   # macOS
set SERVOMOTOR_PORT=COM3                   # Windows (cmd)
$env:SERVOMOTOR_PORT = "COM3"              # Windows (PowerShell)
```

`-p` still overrides it for a single command, and `-P` always shows the menu.

After a port is opened successfully it is remembered, in a per-user location:

| Platform | Saved port file |
|----------|-----------------|
| Linux    | `$XDG_CONFIG_HOME/servomotor/serial_device.txt`, else `~/.config/servomotor/serial_device.txt` |
| macOS    | `~/Library/Application Support/servomotor/serial_device.txt` |
| Windows  | `%LOCALAPPDATA%\servomotor\serial_device.txt` |

Delete that file to forget the remembered port.

### Serial port permissions on Linux

Serial devices usually belong to the `dialout` group (`uucp` on some
distributions). If you get a permission error, add yourself to it:

```bash
sudo usermod -a -G dialout $USER
```

then log out and back in for the change to take effect.

## Features

- Control Gearotons servomotors via serial communication
- Easy-to-use interface for sending commands and receiving responses
- Automatic handling of serial port selection and management

## Requirements

- Python 3.8+

That is the whole list. pyserial is BUNDLED inside this package
(`servomotor/vendor/serial/`) and imported from there, so `pip install servomotor`
pulls in no other dependencies and you do not need to install pyserial yourself.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENCE](LICENCE) file for details.

## Contact

Tom Rodinger - tom.rodinger@alumni.utoronto.ca

Project Link: [https://gearotons.com](https://gearotons.com)
