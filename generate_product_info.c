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
#include <sys/errno.h>
#include "../product_info.h"

#define MAX_MODEL_CODE_LENGTH 8

#define SERIAL_NUMBER_FILENAME ".motor_serial_number"


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
    fstat(fh, &st);
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
        perror("Error: writing to the header file failed with the error");
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
        perror("Error: could not open file for writing");
        exit(1);
    }

    n_bytes = read(fh, data, sizeof(data) - 1);

    if(n_bytes < 0) {
        perror("Error: reading of the file failed");
        close(fh);
        exit(1);
    }

    close(fh);

    assert(n_bytes <= sizeof(data) - 1);
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
                      uint32_t serial_number, uint64_t unique_id)
{
    struct product_info_struct product_info;
    char *data = NULL;
    uint32_t data_len = 0;
    read_file(input_file, &data, &data_len);

    memset(&product_info, 0, sizeof(product_info));
    memcpy(product_info.model_code, model_code, sizeof(product_info.model_code));
    product_info.firmware_compatibility_code = firmware_compatibility_code;
    product_info.hardware_version_bugfix = hardware_version_bugfix;
    product_info.hardware_version_minor = hardware_version_minor;
    product_info.hardware_version_major = hardware_version_major;
    product_info.serial_number = serial_number;
    product_info.unique_id = unique_id;
    memcpy(data + PRODUCT_INFO_MEMORY_LOCATION - 0x8000000, &product_info, sizeof(product_info));

    write_file(output_file, data, data_len);
    free(data);
}


uint8_t string_to_firmware_compatibility_code(const char *number_str)
{
    char *end;
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
    printf("Usage: %s input_file.bin output_file.bin model_code firmware_compatibility_code hardware_version\n", executable_name);
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
    uint64_t unique_id;
    char serial_number_full_filename[2000];

    if(argc != 6) {
        print_usage(argv[0]);
    }

    input_file = argv[1];
    output_file = argv[2];

    if(strlen(argv[3]) > MAX_MODEL_CODE_LENGTH) {
        fprintf(stderr, "Error: the model code is too long. The length must be %d characters or less.\n", MAX_MODEL_CODE_LENGTH);
        exit(1);
    }
    strncpy(model_code, argv[3], strlen(argv[3])); // do the copy without copying the null terminator

    firmware_compatibility_code = string_to_firmware_compatibility_code(argv[4]);

    hardware_version_string = argv[5];
    decompose_hardware_string(hardware_version_string, &hardware_version_bugfix, &hardware_version_minor, &hardware_version_major);

    printf("The model code is [%s]\n", model_code);
    printf("The firmware compatibility code is %hhu\n", firmware_compatibility_code);

    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    sprintf(serial_number_full_filename, "%s/%s", homedir, SERIAL_NUMBER_FILENAME);
    serial_number = read_serial_number(serial_number_full_filename);
    serial_number++; // we will increament the serial number each time we run this
    printf("Incremented the serial number. This device will have serial number %u.\n", serial_number);

    srandomdev();
    unique_id = (random() << 32) | random();

    save_bin_file(input_file, output_file, model_code, firmware_compatibility_code, hardware_version_bugfix, hardware_version_minor, hardware_version_major, serial_number, unique_id);

    write_serial_number(serial_number_full_filename, serial_number);
}