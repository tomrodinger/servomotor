#include <string.h>
#include <stdio.h>
#include "stm32g0xx_hal.h"
#include "settings.h"
#include "global_variables.h"
#include "error_handling.h"
#include "motor_control.h"

#define SETTINGS_FILE "servo_settings.bin"

void load_global_settings(void)
{
    FILE *fp = fopen(SETTINGS_FILE, "rb");
    if (fp) {
        // File exists, read settings
        size_t read = fread(&global_settings, 1, GLOBAL_SETTINGS_STRUCT_SIZE, fp);
        fclose(fp);
        if (read != GLOBAL_SETTINGS_STRUCT_SIZE) {
            // If we couldn't read the full structure, initialize with defaults
            memset(&global_settings, 0, GLOBAL_SETTINGS_STRUCT_SIZE);
            global_settings.commutation_position_offset = DEFAULT_COMMUTATION_POSITION_OFFSET;
            global_settings.my_alias = 1;
        }
    } else {
        // No settings file, initialize with defaults
        memset(&global_settings, 0, GLOBAL_SETTINGS_STRUCT_SIZE);
        global_settings.commutation_position_offset = DEFAULT_COMMUTATION_POSITION_OFFSET;
        global_settings.my_alias = 1;
    }
}

void save_global_settings(void)
{
    FILE *fp = fopen(SETTINGS_FILE, "wb");
    if (fp) {
        fwrite(&global_settings, 1, GLOBAL_SETTINGS_STRUCT_SIZE, fp);
        fclose(fp);
    }
    // Note: In simulation we don't reset after saving settings
}
