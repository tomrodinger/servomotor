#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/errno.h>

#define MODEL_CODE_LENGTH 8
#define FIRMWARE_COMPATIBILITY_CODE_LENGTH 1

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


void save_bin_file(char *input_file, char *output_file, char model_code[8], uint8_t firmware_compatibility_code)
{
    char *data = NULL;
    uint32_t data_len = 0;
    read_file(input_file, &data, &data_len);

    char *output_data = malloc(data_len + MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH);
    if(output_data == NULL) {
        printf("Error: cannot allocate the memory to store the output data\n");
        exit(1);
    }

    memcpy(output_data, model_code, MODEL_CODE_LENGTH);
    memcpy(output_data + MODEL_CODE_LENGTH, &firmware_compatibility_code, FIRMWARE_COMPATIBILITY_CODE_LENGTH);
    memcpy(output_data + MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH, data, data_len);

    write_file(output_file, output_data, MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH + data_len);

    free(data);
    free(output_data);
}


void print_usage(char *executable_name)
{
    printf("Usage: %s input_file.bin output_file.firmware model_code firmware_compatibility_code\n", executable_name);
    exit(1);
}


int main(int argc, char **argv)
{
    char *input_file;
    char *output_file;
    char model_code[9] = "        ";
    uint8_t firmware_compatibility_code;

    if(argc != 5) {
        print_usage(argv[0]);
    }

    input_file = argv[1];
    output_file = argv[2];

    if(strlen(argv[3]) > MODEL_CODE_LENGTH) {
        fprintf(stderr, "Error: the product code is too long. The length must be %d characters or less.\n", MODEL_CODE_LENGTH);
        exit(1);
    }
    strncpy(model_code, argv[3], strlen(argv[3])); // do the copy without copying the null terminator

    firmware_compatibility_code = string_to_firmware_compatibility_code(argv[4]);

    printf("The model code is [%s]\n", model_code);
    printf("The firmware compatibility code is %hhu\n", firmware_compatibility_code);

    save_bin_file(input_file, output_file, model_code, firmware_compatibility_code);
}