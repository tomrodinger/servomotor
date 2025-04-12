// ArduinoEmulator.cpp

#include "ArduinoEmulator.h"

// forward declarations from test_one_move.cpp
extern void setup();
extern void loop();

// Define the global 'Serial' and 'Serial1' objects
ConsoleSerial Serial;  // Debug output to console
HardwareSerial Serial1;  // Hardware communication

// Global variables to store the identifier (alias or uniqueID)
char g_motorAlias = 'X'; // Default alias
uint64_t g_uniqueId = 0; // Default uniqueID 
bool g_useUniqueId = false; // Flag to indicate if uniqueID should be used

#if !defined(ARDUINO)
int main(int argc, char* argv[]) {
#if defined(REQUIRE_SERIAL_PORT)
    // Check if enough arguments are provided
    if (argc < 3) {
        std::cerr << "Error: Serial port and motor identifier must be specified.\n";
        std::cerr << "Usage: " << argv[0] << " <serial_port> <motor_identifier>\n";
        std::cerr << "Examples:\n";
        std::cerr << "  " << argv[0] << " /dev/ttys003 X            # Using alias 'X'\n";
        std::cerr << "  " << argv[0] << " /dev/ttys003 0123456789ABCDEF  # Using 16-char hex uniqueID\n";
        return 1;
    }

    // Initialize Serial1 with the specified port
    Serial1 = HardwareSerial(argv[1]);
    if (!Serial1.begin(230400)) {
        std::cerr << "Failed to initialize serial port. Exiting.\n";
        return 1;  // Exit with error
    }

    // Process the motor identifier (2nd argument)
    std::string identifier = argv[2];
    
    // Check if it's a uniqueID (16 hex characters) or an alias (single character)
    if (identifier.length() == 16) {
        // Convert hex string to uint64_t
        try {
            g_uniqueId = std::stoull(identifier, nullptr, 16);
            #ifdef VERBOSE
            std::cout << "[DEBUG] g_uniqueId after stoull: 0x" << std::hex << g_uniqueId << std::dec << std::endl;
            #endif
            g_useUniqueId = true;
            #ifdef VERBOSE
            std::cout << "Using extended addressing with uniqueID: 0x" << identifier << std::endl;
            #endif
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid uniqueID format. Must be 16 hex characters.\n";
            return 1;
        }
    } else if (identifier.length() == 1) {
        // It's an alias
        g_motorAlias = identifier[0];
        g_useUniqueId = false;
        #ifdef VERBOSE
        std::cout << "Using standard addressing with alias: " << g_motorAlias << std::endl;
        #endif
    } else {
        std::cerr << "Error: Invalid identifier format. Must be either:\n";
        std::cerr << "  - A single character alias (e.g., 'X')\n";
        std::cerr << "  - A 16-character hex uniqueID\n";
        return 1;
    }
#endif

    setup();
    while (true) {
        loop();
        // small delay so we don't spin too fast
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}
#endif
