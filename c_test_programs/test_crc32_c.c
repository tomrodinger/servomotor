#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

// CRC32 polynomial and initial value
#define CRC32_POLYNOMIAL 0xEDB88320
static uint32_t crc32_value;

// Function declarations
void crc32_init(void);
uint32_t calculate_crc32_buffer_without_reinit(void *buffer, uint32_t length);
uint32_t calculate_crc32_buffer(void *buffer, uint32_t length);
uint32_t calculate_crc32_u8(uint8_t value);

// Function to parse a line of hex bytes
int parse_hex_line(const char *line, uint8_t *buffer, int max_size) {
    int count = 0;
    char hex[3] = {0}; // To hold 2 hex chars + null terminator
    int i = 0;
    
    while (line[i] && count < max_size) {
        // Skip spaces and other non-hex characters
        if (isspace(line[i])) {
            i++;
            continue;
        }
        
        // Get 2 hex characters
        if (isxdigit(line[i]) && isxdigit(line[i+1])) {
            hex[0] = line[i];
            hex[1] = line[i+1];
            buffer[count++] = (uint8_t)strtol(hex, NULL, 16);
            i += 2;
        } else {
            // Not a valid hex pair, move to next character
            i++;
        }
    }
    
    return count;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <input_file> <output_file>\n", argv[0]);
        return 1;
    }
    
    const char *input_file = argv[1];
    const char *output_file = argv[2];
    
    FILE *in_fp = fopen(input_file, "r");
    if (!in_fp) {
        printf("Error: Cannot open input file %s\n", input_file);
        return 1;
    }
    
    FILE *out_fp = fopen(output_file, "w");
    if (!out_fp) {
        printf("Error: Cannot open output file %s\n", output_file);
        fclose(in_fp);
        return 1;
    }
    
    char line[1024];
    uint8_t buffer[512];
    
    while (fgets(line, sizeof(line), in_fp)) {
        // Skip comments
        if (line[0] == '#') {
            continue;
        }
        
        // Handle empty line as empty buffer test case
        int byte_count = 0;
        if (line[0] == '\n') {
            // Empty buffer test case
            printf("Testing empty buffer (0 bytes)\n");
        } else {
            // Parse the hex bytes
            byte_count = parse_hex_line(line, buffer, sizeof(buffer));
        }
        
        // Calculate CRC32 using calculate_crc32_buffer
        uint32_t crc32_result = calculate_crc32_buffer(buffer, byte_count);
        
        // Format the output
        if (byte_count == 0) {
            fprintf(out_fp, "[empty buffer] => %08X\n", crc32_result);
        } else {
            for (int i = 0; i < byte_count; i++) {
                fprintf(out_fp, "%02X ", buffer[i]);
            }
            fprintf(out_fp, "=> %08X\n", crc32_result);
        }
        
        // Print to console for debugging
        printf("Buffer: ");
        for (int i = 0; i < byte_count; i++) {
            printf("%02X ", buffer[i]);
        }
        printf("\n");
        printf("calculate_crc32_buffer: %08X\n", crc32_result);
        
        // Test calculate_crc32_buffer_without_reinit
        crc32_init();
        uint32_t crc32_without_reinit = calculate_crc32_buffer_without_reinit(buffer, byte_count);
        printf("calculate_crc32_buffer_without_reinit: %08X\n", crc32_without_reinit);
        
        // Test byte-by-byte calculation
        crc32_init();
        for (int i = 0; i < byte_count; i++) {
            calculate_crc32_u8(buffer[i]);
        }
        uint32_t crc32_byte_by_byte = ~crc32_value;
        printf("byte-by-byte calculation: %08X\n\n", crc32_byte_by_byte);
    }
    
    fclose(in_fp);
    fclose(out_fp);
    
    printf("CRC32 test completed. Results written to %s\n", output_file);
    return 0;
}

// Implementations from settings_sim.c
void crc32_init(void) {
    crc32_value = 0xFFFFFFFF;
}

uint32_t calculate_crc32_buffer_without_reinit(void *buffer, uint32_t length) {
    uint32_t i, j;
    for (i = 0; i < length; i++) {
        crc32_value ^= ((uint8_t *)buffer)[i];
        for (j = 0; j < 8; j++) {
            if (crc32_value & 1)
                crc32_value = (crc32_value >> 1) ^ CRC32_POLYNOMIAL;
            else
                crc32_value = crc32_value >> 1;
        }
    }
    return ~crc32_value;
}

uint32_t calculate_crc32_buffer(void *buffer, uint32_t length) {
    crc32_init();
    return calculate_crc32_buffer_without_reinit(buffer, length);
}

uint32_t calculate_crc32_u8(uint8_t value) {
    uint32_t j;
    crc32_value ^= value;
    for (j = 0; j < 8; j++) {
        if (crc32_value & 1)
            crc32_value = (crc32_value >> 1) ^ CRC32_POLYNOMIAL;
        else
            crc32_value = crc32_value >> 1;
    }
    return ~crc32_value;  // Return inverted final value
}
