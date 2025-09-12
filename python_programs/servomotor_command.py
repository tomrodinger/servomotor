#!/usr/bin/env python3

import sys
import argparse
import servomotor
from terminal_formatting import format_error, format_info, format_warning, format_success, format_debug

def main() -> int:
    # Define the arguments for this program
    parser = argparse.ArgumentParser(description='This program will let you send any supported command to the motor')
    parser.add_argument('-p', '--port', help='serial port device', default=None)
    parser.add_argument('-P', '--PORT', help='show all ports on the system and let the user select from a menu', action="store_true")
    parser.add_argument('-a', '--alias', help='alias of the device to control, or a 16-character hex string for unique ID (extended addressing)', default=None)
    parser.add_argument('-c', '--commands', help='list all supported commands with detailed descriptions', action="store_true")
    parser.add_argument('--crc32-disabled', help='disable CRC32 checksum transmission', action="store_true")
    parser.add_argument('-v', '--verbose', help='equivalent to --verbose-level 2', action="store_true")
    parser.add_argument('--verbose-level', type=int, choices=[0, 1, 2], help='Set verbosity level (0: no output, 1: minimal output, 2: detailed output)', default=None)
    parser.add_argument('command',
                        help='This is the command to be sent to the motor. For example ENABLE_MOSFETS_COMMAND. Run this program with the -c option to see all supported commands.',
                        nargs='?',
                        default=None)
    parser.add_argument('inputs',
                        help='These are the inputs for the command. The number of inputs depends on the command. Some commands have no inputs.',
                        nargs='*',
                        default=None)

    args = parser.parse_args()

    if args.commands is True:
        print(format_info("=== Protocol and Command Information ==="))
        servomotor.print_protocol_version()
        servomotor.print_data_type_descriptions()
        servomotor.print_registered_commands()
        return 0

    if args.command is None:
        print(format_error("You didn't specify the command to run."))
        print(format_info("Please run this program with the -c option to see all supported commands or run this program with the -h option to see the usage information"))
        return 1

    command_id = servomotor.get_command_id(args.command)
    if command_id is None:
        print(format_error(f"Unknown command: '{args.command}'"))
        print(format_info("Please run this program with the -c option to see all supported commands"))
        return 1

    # Determine verbosity level
    if args.verbose_level is not None:
        verbose_level = args.verbose_level
    elif args.verbose:
        verbose_level = 2
    else:
        verbose_level = 1

    if verbose_level >= 1:
        print(format_info(f"Command ID: {command_id}"))

    servomotor.set_standard_options_from_args(args)  # This will set port and alias/unique_id
    servomotor.open_serial_port()
    try:
        _ = servomotor.execute_command(args.command, args.inputs, crc32_enabled=not args.crc32_disabled, verbose=verbose_level)
        print(format_success("Command executed successfully"))
        return 0
    except servomotor.TimeoutError:
        print(format_error("Timeout Error: The device did not respond"))
        print(format_warning(
            "This may be that your device is not connected and powered on, "
            "or that the serial port is not correct, "
            "or that the alias/unique ID is not correct for your target device, "
            "or your wiring is not right"
        ))
        return 1
    except servomotor.CommunicationError as e:
        print(format_error(f"Error interpreting response: {str(e)}"))
        return 1
    except servomotor.PayloadError as e:
        print(format_error(f"Error interpreting response: {str(e)}"))
        return 1
    except servomotor.NoAliasOrUniqueIdSet as e:
        print(format_error(f"Error interpreting response: {str(e)}"))
        return 1
    except servomotor.FatalError as e:
        print(format_error(f"Error: the device is in a fatal error state. The error code is {e}."))
        return 1
    finally:
        servomotor.close_serial_port()

if __name__ == "__main__":
    sys.exit(main())
