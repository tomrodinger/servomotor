
## Usage

Here's a basic example of how to use the servomotor package:

TODO

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

- Python 3.6+
- pyserial 3.5+

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

Tom Rodinger - tom.rodinger@alumni.utoronto.ca

Project Link: [https://gearotons.com](https://gearotons.com)
