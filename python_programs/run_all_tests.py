#!/usr/bin/env python3

import argparse
import subprocess
import os
import sys
import re
import time
# Attempt to import serial tools, handle if not found
try:
    from serial.tools.list_ports import comports
except ImportError:
    comports = None

# ANSI Color Codes
GREEN = '\033[0;32m'
RED = '\033[0;31m'
NC = '\033[0m' # No Color

# Test runner configuration
TEST_TIMEOUT_SECONDS = 240  # 4 minutes

# No default port initially, will prompt or use provided if -p is given
DEFAULT_PORT = None

def select_serial_port():
    """Lists available serial ports and prompts the user to select one."""
    if comports is None:
        print(f"{RED}Error: pyserial is required for port listing. Please install it (`pip install pyserial`).{NC}")
        sys.exit(1)

    ports = list(comports())
    if not ports:
        print(f"{RED}Error: No serial ports found.{NC}")
        sys.exit(1)

    print("Available serial ports:")
    for i, port in enumerate(ports):
        # Provide more details: description and hwid
        print(f"  {i + 1}: {port.device} - {port.description} [{port.hwid}]")

    while True:
        try:
            selection = input(f"Select port number (1-{len(ports)}): ")
            port_index = int(selection) - 1
            if 0 <= port_index < len(ports):
                selected_port = ports[port_index].device
                print(f"Using port: {selected_port}")
                return selected_port
            else:
                print(f"{RED}Invalid selection. Please enter a number between 1 and {len(ports)}.{NC}")
        except ValueError:
            print(f"{RED}Invalid input. Please enter a number.{NC}")
        except (EOFError, KeyboardInterrupt):
            print("\nOperation cancelled by user.")
            sys.exit(1)


def parse_summary_md(filepath="TEST_SUMMARY.md"):
    """Parses the TEST_SUMMARY.md file to find non-obsolete servomotor tests."""
    tests_to_run = []
    try:
        with open(filepath, 'r') as f:
            in_table = False
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if line.startswith('| Filename'):
                    in_table = True
                    continue
                if line.startswith('| :--'):
                    continue
                if not line.startswith('|'):
                    in_table = False
                    continue

                if in_table:
                    parts = [part.strip() for part in line.split('|')[1:-1]]
                    # Ensure we have enough columns (including the new PASS/FAIL one)
                    if len(parts) >= 5: # Filename, Desc, Module, Obsolete, Prints PASS/FAIL
                        filename_match = re.search(r'`([^`]+)`', parts[0])
                        module_used = parts[2].replace('`', '')
                        obsolete = parts[3].lower()

                        if filename_match and module_used == 'servomotor' and obsolete == 'no':
                             tests_to_run.append(filename_match.group(1))
    except FileNotFoundError:
        print(f"{RED}Error: {filepath} not found.{NC}")
        sys.exit(1)
    except Exception as e:
        print(f"{RED}Error parsing {filepath}: {e}{NC}")
        sys.exit(1)

    # Filter out tests that shouldn't be run automatically (like the runner itself if named similarly)
    tests_to_run = [t for t in tests_to_run if t != os.path.basename(__file__)]
    # Filter out unit tests or simple scripts that don't need port/alias
    tests_to_run = [t for t in tests_to_run if t not in ('test_servomotor_module_get_command_id.py', 'test_servomotor_module.py')]


    print(f"Found {len(tests_to_run)} non-obsolete servomotor tests to run:")
    for test in tests_to_run:
        print(f"  - {test}")
    print("-" * 20)
    time.sleep(1) # Give user time to see which tests will run
    return tests_to_run

def run_tests(port, alias, test_files):
    """Runs the specified test files and reports results."""
    total = 0
    passed = 0
    failed = 0
    test_results = []
 
    print("\nRunning Tests...")
    print("====================")
    print(f"Serial Port: {port}")
    print(f"Device Alias: {alias}")
    print("====================")

    for test_script in test_files:
        total += 1
        print(f"\n--- Running {test_script} ---")
        command = [sys.executable, test_script, "-p", port, "-a", alias]
        start_time = time.time()
        status_message = "FAILED (Unknown)"

        try:
            result = subprocess.run(command, capture_output=True, text=True, check=False, timeout=TEST_TIMEOUT_SECONDS)

            print(result.stdout) # Print stdout regardless of pass/fail

            if result.returncode == 0:
                should_print_pass_fail = True # Assume yes unless known otherwise
                if test_script in ["test_get_comprehensive_position.py",
                                   "test_gradual_speed_up.py",
                                   "test_random_speed_stress.py",
                                   "test_set_velocity_to_specific_values.py"]:
                   should_print_pass_fail = False

                if should_print_pass_fail and (not result.stdout or "PASSED" not in result.stdout.splitlines()[-1]):
                     print(f"{RED}Warning: Test exited successfully but did not print 'PASSED'. Treating as FAIL.{NC}")
                     failed += 1
                     status_message = "FAILED (Missing PASSED message)"
                     print(f"{RED}{test_script} {status_message}{NC}")
                     if result.stderr:
                         print("--- Stderr ---")
                         print(result.stderr)
                         print("--------------")
                else:
                    passed += 1
                    status_message = "PASSED"
                    print(f"{GREEN}{test_script} PASSED{NC}")

            else:
                failed += 1
                status_message = f"FAILED (Exit Code: {result.returncode})"
                print(f"{RED}{test_script} {status_message}{NC}")
                if result.stderr:
                    print("--- Stderr ---")
                    print(result.stderr)
                    print("--------------")

        except subprocess.TimeoutExpired:
            failed += 1
            status_message = f"FAILED (Timeout after {TEST_TIMEOUT_SECONDS} seconds)"
            print(f"{RED}{test_script} {status_message}{NC}")
        except Exception as e:
            failed += 1
            status_message = f"FAILED (Execution Error: {e})"
            print(f"{RED}{test_script} {status_message}{NC}")
        finally:
            end_time = time.time()
            duration = end_time - start_time
            test_results.append((test_script, duration, status_message))

    return total, passed, failed, test_results

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run Python servomotor tests.')
    parser.add_argument('-p', '--port', help='Serial port device. If not specified and -P is used, will prompt.', default=DEFAULT_PORT)
    parser.add_argument('-P', '--PORT', help='Show available ports and prompt for selection if -p is not given.', action="store_true")
    parser.add_argument('-a', '--alias', help='Alias of the device to control (default: X)', default='X')
    # Add verbosity later if needed

    args = parser.parse_args()

    # Determine port
    port_to_use = args.port
    if args.PORT and port_to_use is None:
        port_to_use = select_serial_port()
    elif port_to_use is None:
        # If -P wasn't used and -p wasn't specified, exit with error
        print(f"{RED}Error: Serial port must be specified using -p or selected interactively using -P.{NC}")
        parser.print_help()
        sys.exit(1)


    # Validate alias (simple check)
    if len(args.alias) != 1:
        print(f"{RED}Error: Alias must be a single character.{NC}")
        sys.exit(1)

    test_files_to_run = parse_summary_md()

    if not test_files_to_run:
        print("No tests found to run.")
        sys.exit(0)

    total_run, total_passed, total_failed, test_results = run_tests(port_to_use, args.alias, test_files_to_run)

    # Print runtime summary
    print("\nTest Runtimes")
    print("=============")
    total_duration = 0
    for name, t, status in test_results:
        total_duration += t
        if status == "PASSED":
            status_colored = f"{GREEN}{status}{NC}"
        else:
            status_colored = f"{RED}{status}{NC}"
        print(f"{name:<40} {t:>6.2f}s  {status_colored}")

    # Print final summary
    print("\nTest Summary")
    print("============")
    print(f"Total Test Time: {total_duration:.2f}s")
    print(f"Passed: {GREEN}{total_passed}{NC}")
    print(f"Failed: {RED}{total_failed}{NC}")
    print(f"Total:  {total_run}")

    # Exit with failure if any test failed
    sys.exit(1 if total_failed > 0 else 0)