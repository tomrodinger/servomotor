#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <assert.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>
#include "common_source_files/product_info.h"

#define MAX_MODEL_CODE_LENGTH 8
#define SERIAL_NUMBER_FILENAME ".motor_serial_number"

#define UPDATE_MODEL_CODE_FLAG                  0x01
#define UPDATE_FIRMWARE_COMPATIBILITY_CODE_FLAG 0x02
#define UPDATE_HARDWARE_VERSION_FLAG            0x04
#define UPDATE_SERIAL_NUMBER_FLAG               0x08
#define UPDATE_UNIQUE_ID_FLAG                   0x10

void read_file(char *filename, char** data, uint32_t *data_length)
{
    ssize_t n_bytes;
    
    printf("Reading from file: %s\n", filename);

    int fh = open(filename, O_RDONLY);

    if(fh < 0) {
        perror("Error: could not open file for reading");
        exit(1);
    }

    struct stat st;
    if (fstat(fh, &st) != 0) {
        perror("Error: could not stat input file");
        close(fh);
        exit(1);
    }
    *data_length = st.st_size;
    printf("The file size is %u bytes\n", *data_length);

//    fseek(fp, 0L, SEEK_END);
//    sz = ftell(fp);

    *data = malloc(*data_length);
    if(*data == NULL) {
        fprintf(stderr, "Error: could not allocate enough memory to hold the input file\n");
        close(fh);
        exit(1);
    }

    n_bytes = read(fh, *data, *data_length);

    if(n_bytes < 0) {
        perror("Error: reading from the input file failed");
        close(fh);
        exit(1);
    }

    if(n_bytes != *data_length) {
        fprintf(stderr, "Error: failed to read the required number of bytes: %u\n", *data_length);
        close(fh);
        exit(1);
    }

    close(fh);

    printf("Successfully read %u bytes from the file\n", *data_length);
}

void write_file(char *filename, char* data, uint32_t data_length)
{
    ssize_t n_bytes;

    printf("Writing to file: %s\n", filename);

    int fh = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);

    if(fh < 0) {
        perror("Error: could not open file for writing");
        exit(1);
    }

    n_bytes = write(fh, data, data_length);

    if(n_bytes < 0) {
        perror("Error: writing to the header file failed with the error");
        close(fh);
        exit(1);
    }

    if(n_bytes != data_length) {
        fprintf(stderr, "Error: failed to write the required number of bytes: %u\n", data_length);
        close(fh);
        exit(1);
    }

    close(fh);

    printf("Successfully wrote %u bytes to the file\n", data_length);
}


uint32_t read_serial_number(char *filename)
{
    char data[20];
    ssize_t n_bytes;
    unsigned int serial_number;
    int n_converted;

    printf("Reading the file: %s\n", filename);

    int fh = open(filename, O_RDONLY);

    if(fh < 0) {
        perror("Error: could not open serial number file for reading");
        exit(1);
    }

    n_bytes = read(fh, data, sizeof(data) - 1);

    if(n_bytes < 0) {
        perror("Error: reading of the file failed");
        close(fh);
        exit(1);
    }

    close(fh);

    assert((size_t)n_bytes <= sizeof(data) - 1);
    data[n_bytes] = 0;
    if(n_bytes == 0) {
        printf("Error: read in zero bytes. Seems that the file is empty. Please make sure there is an ascii representation of an unsigned integer contained in the file.\n");
        exit(1);
    }
//    printf("Read in: [%s]\n", data);

    n_converted = sscanf(data, "%u", &serial_number);
    if(n_converted != 1) {
        fprintf(stderr, "Error: conversion of the data in the file to an unsigned integer failed\n");
        fprintf(stderr, "Check what value is in the file. It must be an ascii text representation of an unsigned integer followed by nothing or white space\n");
        exit(1);
    }

    printf("Read in the serial number %u\n", serial_number);

    return (uint32_t)serial_number;
}


void write_serial_number(char *filename, uint32_t serial_number)
{
    char data[100];
    printf("Saving the serial number %u\n", serial_number);
    sprintf(data, "%u\n", serial_number);
    write_file(filename, data, strlen(data));
}


void save_bin_file(char *input_file, char *output_file, char model_code[8], uint8_t firmware_compatibility_code,
                      uint8_t hardware_version_bugfix, uint8_t hardware_version_minor, uint8_t hardware_version_major,
                      uint32_t serial_number, uint64_t unique_id, uint32_t update_flags)
{
    struct product_info_struct product_info;
    char *data = NULL;
    uint32_t data_len = 0;
    read_file(input_file, &data, &data_len);

    const uint32_t product_info_offset = (uint32_t)(PRODUCT_INFO_MEMORY_LOCATION - 0x08000000u);
    if ((uint64_t)product_info_offset + sizeof(product_info) > (uint64_t)data_len) {
        fprintf(stderr,
                "Error: input file too small (%u bytes) to contain product info at offset 0x%08X\n",
                data_len,
                product_info_offset);
        free(data);
        exit(1);
    }

    memcpy(&product_info, data + product_info_offset, sizeof(product_info));
    if (update_flags & UPDATE_MODEL_CODE_FLAG) {
        memcpy(product_info.model_code, model_code, sizeof(product_info.model_code));
    }
    if (update_flags & UPDATE_FIRMWARE_COMPATIBILITY_CODE_FLAG) {
        product_info.firmware_compatibility_code = firmware_compatibility_code;
    }
    if (update_flags & UPDATE_HARDWARE_VERSION_FLAG) {
        product_info.hardware_version_bugfix = hardware_version_bugfix;
        product_info.hardware_version_minor = hardware_version_minor;
        product_info.hardware_version_major = hardware_version_major;
    }
    if (update_flags & UPDATE_SERIAL_NUMBER_FLAG) {
        product_info.serial_number = serial_number;
    }
    if (update_flags & UPDATE_UNIQUE_ID_FLAG) {
        product_info.unique_id = unique_id;
    }
    memcpy(data + product_info_offset, &product_info, sizeof(product_info));

    write_file(output_file, data, data_len);
    free(data);
}


uint8_t string_to_firmware_compatibility_code(const char *number_str)
{
    char *end;
    errno = 0;
    long int value = strtol(number_str, &end, 10); 
    if ((end == number_str) || (*end != '\0') || (errno == ERANGE)) {
        printf("Error: not a valid integer number");
        exit(1);
    }
    if(value < 0) {
        printf("Error: the firmware compatibility code cannot be a negative number\n");
        printf("You have specified the invalid input [%s]\n", number_str);
        exit(1);
    }
    if(value > 255) {
        printf("Error: the firmware compatibility code cannot be larger than 255\n");
        printf("You have specified the invalid input [%s]\n", number_str);
        exit(1);
    }
    return (uint8_t)value;
}


void decompose_hardware_string(const char *hardware_version_string, uint8_t *hardware_version_bugfix, uint8_t *hardware_version_minor, uint8_t *hardware_version_major)
{
    const char *ptr = hardware_version_string;
    uint32_t decompose_order = 3;
    uint32_t number = 0;

    *hardware_version_major = 0;
    *hardware_version_minor = 0;
    *hardware_version_bugfix = 0;

    printf("Decomposing the hardware string: %s\n", hardware_version_string);

    while(1) {
        if((*ptr >= '0') && (*ptr <= '9')) {
            number = number * 10 + ((*ptr) - '0');
        }
        else if((*ptr == '.') || (*ptr == 13) || (*ptr == 10) || (*ptr == 8) || (*ptr == 0)) {
            switch(decompose_order) {
            case 3:
                *hardware_version_major = number;
                printf("Assigning the hardware version major number the value of %hhu\n", *hardware_version_major);
                break;
            case 2:
                *hardware_version_minor = number;
                printf("Assigning the hardware version minor number the value of %hhu\n", *hardware_version_minor);
                break;
            case 1:
                *hardware_version_bugfix = number;
                printf("Assigning the hardware version bugfix number the value of %hhu\n", *hardware_version_bugfix);
                break;
            default:
                printf("Error: failed to decompose the hardware revision string into the components of major, minor, and bugfix numbers\n");
                printf("The format must be inthe form like this: 1.2.3\n");
                exit(1);
            }
            if(*ptr == 0) {
                break;
            }
            number = 0;
            decompose_order--;
        }
        else {
            fprintf(stderr, "Error: encountered an invalid character: %c (ascii number %hhu)\n", *ptr, *ptr);
            printf("The format must be in the form like this: 1.2.3\n");
            exit(1);
        }
        ptr++;
    }
}


void print_usage(char *executable_name)
{
    printf("Usage: %s input_file.bin output_file.bin model_code|SKIP firmware_compatibility_code|SKIP hardware_version|SKIP serial_number|AUTO|SKIP unique_id|AUTO|SKIP\n", executable_name);
    printf("       the serial number can be taken from a file in the user's home directory called %s if AUTO is specified\n", SERIAL_NUMBER_FILENAME);
    printf("       the serial number can be specified in decimal notation or in hex notation with 0x prefix\n");
    printf("       the unique ID is randomly generated if AUTO is specified, otherwise specify it in hex notation with 0x prefix or no prefix\n");
    printf("       some of the data items can be skipped from updating by specifying SKIP for that item\n");
    exit(1);
}


int main(int argc, char **argv)
{
    char *input_file;
    char *output_file;
    char model_code[9] = "        ";
    uint8_t firmware_compatibility_code;
    char *hardware_version_string;
    uint8_t hardware_version_bugfix;
    uint8_t hardware_version_minor;
    uint8_t hardware_version_major;
    uint32_t serial_number;
    uint64_t unique_id = 0xFFFFFFFFFFFFFFFF;
    char serial_number_full_filename[2000];
    uint32_t update_flags = 0;

    if(argc != 8) {
        print_usage(argv[0]);
    }

    input_file = argv[1];
    output_file = argv[2];

    if (strcmp(argv[3], "SKIP") != 0) {
        update_flags |= UPDATE_MODEL_CODE_FLAG;
        if(strlen(argv[3]) > MAX_MODEL_CODE_LENGTH) {
            fprintf(stderr, "Error: the model code is too long. The length must be %d characters or less.\n", MAX_MODEL_CODE_LENGTH);
            exit(1);
        }
        strncpy(model_code, argv[3], strlen(argv[3])); // do the copy without copying the null terminator
        printf("The model code will be updated to [%s]\n", model_code);
    }

    if (strcmp(argv[4], "SKIP") != 0) {
        update_flags |= UPDATE_FIRMWARE_COMPATIBILITY_CODE_FLAG;
        firmware_compatibility_code = string_to_firmware_compatibility_code(argv[4]);
        printf("The firmware compatibility code will be updated to %hhu\n", firmware_compatibility_code);
    }

    if (strcmp(argv[5], "SKIP") != 0) {
        update_flags |= UPDATE_HARDWARE_VERSION_FLAG;
        hardware_version_string = argv[5];
        printf("The hardware version string to decompose is [%s]\n", hardware_version_string);
        decompose_hardware_string(hardware_version_string, &hardware_version_bugfix, &hardware_version_minor, &hardware_version_major);
        printf("The hardware version will be updated to %hhu.%hhu.%hhu\n", hardware_version_major, hardware_version_minor, hardware_version_bugfix);
    }

    // let's read the serial number from the command line (unless AUTO or SKIP is specified)
    // the serial number can be in decimal or hex format. if hex then it is preceded by 0x
    if(strcmp(argv[6], "SKIP") != 0) {
        update_flags |= UPDATE_SERIAL_NUMBER_FLAG;
        if(strcmp(argv[6], "AUTO") == 0) {
            struct passwd *pw = getpwuid(getuid());
            const char *homedir = pw->pw_dir;
            sprintf(serial_number_full_filename, "%s/%s", homedir, SERIAL_NUMBER_FILENAME);
            serial_number = read_serial_number(serial_number_full_filename);
            serial_number++; // we will increament the serial number each time we run this
            printf("Incremented the serial number. This device will have serial number %u.\n", serial_number);
            write_serial_number(serial_number_full_filename, serial_number);
        }
        else {
            if (strncmp(argv[6], "0x", 2) == 0) {
                char *end;
                errno = 0;
                long int value = strtol(argv[6], &end, 16); 
                if ((end == argv[6]) || (*end != '\0') || (errno == ERANGE)) {
                    printf("Error: not a valid hex number for the serial number\n");
                    exit(1);
                }
                if(value < 0) {
                    printf("Error: the serial number cannot be a negative number\n");
                    printf("You have specified the invalid input [%s]\n", argv[6]);
                    exit(1);
                }
                serial_number = (uint32_t)value;
                printf("Using the specified serial number after decoding from hex: %u\n", serial_number);
            }
            else {
                char *end;
                errno = 0;
                long int value = strtol(argv[6], &end, 10); 
                if ((end == argv[6]) || (*end != '\0') || (errno == ERANGE)) {
                    printf("Error: not a valid integer number for the serial number\n");
                    exit(1);
                }
                if(value < 0) {
                    printf("Error: the serial number cannot be a negative number\n");
                    printf("You have specified the invalid input [%s]\n", argv[6]);
                    exit(1);
                }
                serial_number = (uint32_t)value;
                printf("Using the specified serial number %u\n", serial_number);
            }
        }
    }

    // let's read the unique ID from the command line if AUTO is not specified
    // the unique ID is always specified in hexadecimal format preceded by 0x or not preceded by 0x
    if (strcmp(argv[7], "SKIP") != 0) {
        update_flags |= UPDATE_UNIQUE_ID_FLAG;
        if(strcmp(argv[7], "AUTO") == 0) {
            printf("Generating a random unique ID\n");

            // Prefer OS entropy via /dev/urandom for portability across macOS/Linux.
            // Fallback to libc PRNG if /dev/urandom is unavailable.
            int urandom_fh = open("/dev/urandom", O_RDONLY);
            if (urandom_fh >= 0) {
                ssize_t got = read(urandom_fh, &unique_id, sizeof(unique_id));
                close(urandom_fh);
                if (got != (ssize_t)sizeof(unique_id)) {
                    fprintf(stderr, "Error: failed to read enough bytes from /dev/urandom\n");
                    exit(1);
                }
            } else {
                // Best-effort fallback (not cryptographically strong).
                srandom((unsigned int)getpid() ^ (unsigned int)time(NULL));
                unique_id = ((uint64_t)(uint32_t)random() << 32) | (uint32_t)random();
            }

            printf("Generated the unique ID %llX\n", (unsigned long long)unique_id);
        }
        else {
            char *end;
            errno = 0;
            unsigned long long value = strtoull(argv[7], &end, 16);
            if ((end == argv[7]) || (*end != '\0') || (errno == ERANGE)) {
                printf("Error: not a valid number for the unique ID\n");
                exit(1);
            }
            unique_id = (uint64_t)value;
            printf("Using the specified unique ID %llX\n", (unsigned long long)unique_id);
        }
    }

    save_bin_file(input_file, output_file, model_code, firmware_compatibility_code, hardware_version_bugfix, hardware_version_minor, hardware_version_major, serial_number, unique_id, update_flags);

    return 0;
}
