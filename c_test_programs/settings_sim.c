#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "settings.h"
#include "global_variables.h"
#include "error_handling.h"
#include "motor_control.h"

#define SETTINGS_FILE "servo_settings.bin"
#define CRC32_POLYNOMIAL 0xEDB88320

static uint32_t crc32_value;

void crc32_init(void)
{
    crc32_value = 0xFFFFFFFF;
}

uint32_t calculate_crc32_buffer(uint8_t *buffer, uint32_t length)
{
    uint32_t i, j;
    for (i = 0; i < length; i++) {
        crc32_value ^= buffer[i];
        for (j = 0; j < 8; j++) {
            if (crc32_value & 1)
                crc32_value = (crc32_value >> 1) ^ CRC32_POLYNOMIAL;
            else
                crc32_value = crc32_value >> 1;
        }
    }
    return crc32_value;
}

uint32_t calculate_crc32_u8(uint8_t value)
{
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

void load_global_settings(void)
{
    printf("DEBUG - Loading settings\n"); // DEBUG - temporarily added to aid in debugging
    FILE *fp = fopen(SETTINGS_FILE, "rb");
    if (fp) {
        // File exists, read settings
        size_t read = fread(&global_settings, 1, GLOBAL_SETTINGS_STRUCT_SIZE, fp);
        fclose(fp);
        if (read != GLOBAL_SETTINGS_STRUCT_SIZE) {
            // If we couldn't read the full structure, initialize with defaults
            printf("DEBUG - Failed to read settings, using defaults\n"); // DEBUG - temporarily added to aid in debugging
            memset(&global_settings, 0, GLOBAL_SETTINGS_STRUCT_SIZE);
            global_settings.commutation_position_offset = DEFAULT_COMMUTATION_POSITION_OFFSET;
            global_settings.my_alias = 1;
        } else {
            printf("DEBUG - Loaded settings, alias=%d\n", global_settings.my_alias); // DEBUG - temporarily added to aid in debugging
            printf("Hall sensor midlines: %u, %u, %u\n",
                   global_settings.hall1_midline,
                   global_settings.hall2_midline,
                   global_settings.hall3_midline);
        }
    } else {
        // No settings file, initialize with defaults
        printf("DEBUG - No settings file, using defaults\n"); // DEBUG - temporarily added to aid in debugging
        memset(&global_settings, 0, GLOBAL_SETTINGS_STRUCT_SIZE);
        global_settings.commutation_position_offset = DEFAULT_COMMUTATION_POSITION_OFFSET;
        global_settings.my_alias = 1;
    }
}

extern volatile int gResetProgress;

void save_global_settings(void)
{
    printf("Saving settings, new alias=%d\n", global_settings.my_alias);
    FILE *fp = fopen(SETTINGS_FILE, "wb");
    if (fp) {
        size_t written = fwrite(&global_settings, 1, GLOBAL_SETTINGS_STRUCT_SIZE, fp);
        fclose(fp);
        if (written != GLOBAL_SETTINGS_STRUCT_SIZE) {
            // Write failed - don't reset
            printf("Error: Failed to save settings (wrote %zu of %zu bytes)\n",
                   written, (size_t)GLOBAL_SETTINGS_STRUCT_SIZE);
            return;
        }
        printf("Settings saved successfully\n");
    } else {
        // Failed to open file - don't reset
        printf("Error: Failed to open settings file for writing\n");
        return;
    }
    // Only reset if settings were saved successfully
    printf("Requesting reset\n");
    gResetProgress = 1;
}
