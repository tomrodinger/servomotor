#!/usr/bin/env python3

from terminal_formatting import (
    STYLE, format_error, format_warning, 
    format_success, format_info, format_debug
)

def main():
    # Important messages with formatting
    print("\n=== Important Messages (With Formatting) ===")
    print(format_warning("WARNING: Critical operation in progress!"))
    print(format_error("ERROR: Connection failed!"))
    print(format_success("SUCCESS: Operation completed!"))
    
    # Informational messages without formatting
    print("\n=== Informational Messages (No Formatting) ===")
    print(format_info("INFO: System status: Online"))
    print(format_info("INFO: Processing request..."))
    print(format_debug("DEBUG: Connection established on port 8080"))
    
    # Practical example
    print("\n=== Practical Example ===")
    try:
        # Simulate an operation
        print(format_info("Starting important operation..."))
        # Simulate progress
        print(format_info("Step 1: Initializing system"))
        print(format_info("Step 2: Loading configuration"))
        # Simulate an error
        raise Exception("Network connection timeout")
    except Exception as e:
        print(format_error(f"ERROR: {str(e)}"))
    
    # Print terminal capabilities information
    print("\n=== Terminal Capabilities ===")
    print(format_info(f"Color Support: {'Yes' if STYLE.has_color else 'No'}"))
    print(format_info(f"Unicode Support: {'Yes' if STYLE.has_unicode else 'No'}"))

if __name__ == "__main__":
    main()
