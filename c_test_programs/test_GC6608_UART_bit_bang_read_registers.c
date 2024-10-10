#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define GPIOA_GC6609_UART_PIN 15
#define GPIOA_GC6609_UART_MODER_POS GPIO_MODER_MODE15_Pos // set this correspondingly to whatever GPIOA_GC6609_UART_PIN is set to

#define SIMULATION
uint32_t time_counter = 0;
#define MAX_ITERATIONS 100000
uint8_t simulated_input_bit_sequence[] = {
    1, 1, 1, 1, 1,      // simulate chip not responding yet
    0,0,0,              // start bit
    1,1,1,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0, // the bits for the first byte, which is the sync byte
    1,1,1,              // stop bit
    0,0,0,              // start bit
    1,1,1,  1,1,1,  1,1,1,  1,1,1,  1,1,1,  1,1,1,  1,1,1,  1,1,1, // the bits for the second byte, which is should always be 0xff (it is the master address)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,   // the bits for the third byte, which is the registered address that is being returned
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  1,1,1,  0,0,0,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,   // and now the contents of the register we are reading (4 bytes, most significant byte first)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  1,1,1,  0,0,0,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,   // and now the contents of the register we are reading (4 bytes, most significant byte first)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  1,1,1,  0,0,0,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,   // and now the contents of the register we are reading (4 bytes, most significant byte first)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  1,1,1,  0,0,0,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,   // and now the contents of the register we are reading (4 bytes, most significant byte first)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,   // finally, the crc byte. we will not bother to make it right in this case
    1,1,1,              // stop bit

    1, 1, 1, 1, 1,      // simulate chip not responding yet
    0,0,0,              // start bit
    1,1,1,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0, // the bits for the first byte, which is the sync byte
    1,1,1,              // stop bit
    0,0,0,              // start bit
    1,1,1,  1,1,1,  1,1,1,  1,1,1,  1,1,1,  1,1,1,  1,1,1,  1,1,1, // the bits for the second byte, which is should always be 0xff (it is the master address)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  0,0,0,  1,1,1,  1,1,1,  0,0,0,  1,1,1,  1,1,1,  0,0,0,   // the bits for the third byte, which is the registered address that is being returned
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  1,1,1,  0,0,0,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,   // and now the contents of the register we are reading (4 bytes, most significant byte first)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  1,1,1,  0,0,0,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,   // and now the contents of the register we are reading (4 bytes, most significant byte first)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  1,1,1,  0,0,0,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,   // and now the contents of the register we are reading (4 bytes, most significant byte first)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  1,1,1,  0,0,0,  0,0,0,  1,1,1,  0,0,0,  0,0,0,  0,0,0,   // and now the contents of the register we are reading (4 bytes, most significant byte first)
    1,1,1,              // stop bit
    0,0,0,              // start bit
    0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,   // finally, the crc byte. we will not bother to make it right in this case
    1,1,1,              // stop bit
};
uint32_t simulated_input_bit_index = 0;
FILE *output_bit_stream_fd = NULL;
FILE *input_bit_stream_fd = NULL;
FILE *input_output_mode_fd = NULL;

void set_UART_pin_as_output(void)
{
    #ifndef SIMULATION
    GPIOA->MODER |= (MODER_DIGITAL_OUTPUT << GPIOA_GC6609_UART_MODER_POS); // set the pin to output mode by setting the two bits corresponding to the port to a value of 0x1
    #else
    printf("Setting UART pin as output at time_counter %u\n", time_counter);
    fprintf(input_output_mode_fd, "%u 1\n", time_counter); // also, log to the file
    #endif
}

void set_UART_pin_as_input(void)
{
    #ifndef SIMULATION
    GPIOA->MODER &= ~(0x3 << GPIOA_GC6609_UART_MODER_POS); // set the pin to input mode by clearing the two bits in the MODER register
    #else
    printf("Setting UART pin as input at time_counter %u\n", time_counter);
    fprintf(input_output_mode_fd, "%u 0\n", time_counter); // also, log to the file
    #endif
}

void set_UART_pin_high(void)
{
    #ifndef SIMULATION
    GPIOA->BSRR = (1 << GPIOA_GC6609_UART_PIN);   
    #else
    printf("Setting UART pin high at time_counter %u\n", time_counter);
    fprintf(output_bit_stream_fd, "%u 1\n", time_counter); // also, log to the file
    #endif
}

void set_UART_pin_low(void)
{
    #ifndef SIMULATION
    GPIOA->BSRR = ((1 << GPIOA_GC6609_UART_PIN) << 16);
    #else
    printf("Setting UART pin low at time_counter %u\n", time_counter);
    fprintf(output_bit_stream_fd, "%u 0\n", time_counter); // also, log to the file
    #endif
}

uint8_t read_UART_pin(void)
{
    #ifndef SIMULATION
    return GPIOA->IDR & (1 << GPIOA_GC6609_UART_PIN);
    #else
    if (simulated_input_bit_index >= sizeof(simulated_input_bit_sequence)) {
        printf("Error: ran out of simulated input bits\n");
        return 1;
    }
    uint8_t bit = simulated_input_bit_sequence[simulated_input_bit_index++];
    printf("Reading UART pin at time_counter %u and got bit %hhu\n", time_counter, bit);
    fprintf(input_bit_stream_fd, "%u %hhu\n", time_counter, bit); // also, log to the file
    return bit;
    #endif
}

#define SYNC_PLUS_RESERVED 0x05
uint8_t registers_to_read[] = {0x00, 0x6c};
uint8_t register_index = 0;
bool rx_phase = false;
uint8_t byte_index = 0;
uint8_t bit_index = 0;
uint8_t crc = 0;
uint8_t time_divider = 0; // for writing operations, the baud rate be set by dividing the frequency of the function call by 3. this will give a baud rate of about 10700. for reading operations, we will sample at the full rate.
bool waiting_for_start = 0;
uint8_t n_sampled_bits = 0;
uint8_t rx_byte = 0;
uint8_t end_of_read_delay = 0;

bool GC6608_UART_bit_bang_read_registers(uint8_t *returned_data, uint8_t *returned_data_size)
{
    uint8_t bit;
    if (end_of_read_delay > 0) {
        end_of_read_delay--;
        return false;
    }
    if (!rx_phase) {
        set_UART_pin_as_output();
        if (time_divider == 0) {
            if (bit_index == 0) { // is it a start bit?
                bit = 0;
            }
            else if (bit_index == 9) { // is it a stop bit?
                bit = 1;
            }
            else { // otherwise it is a bit from the byte we need to send
                uint8_t byte;
                if (byte_index == 0) {
                    byte = SYNC_PLUS_RESERVED;
                }
                else if (byte_index == 1) {
                    byte = 0x00;
                }
                else if (byte_index == 2) {
                    byte = registers_to_read[register_index];
                }
                else {
                    byte = crc;
                }
                bit = (byte >> (bit_index - 1)) & 1;
                if (byte_index <= 2) {
                    if ((crc >> 7) ^ bit) { // update CRC based result of XOR operation
                        crc = (crc << 1) ^ 0x07;
                    }
                    else {
                        crc = (crc << 1);
                    }
                }
            }
            if (bit) {
                set_UART_pin_high();
            }
            else {
                set_UART_pin_low();
            }
            bit_index++;
            if (bit_index == 10) {
                bit_index = 0;
                byte_index++;
                if (byte_index == 4) {
                    rx_phase = true;
                    waiting_for_start = true;
                    n_sampled_bits = 0;
                    rx_byte = 0;
                    byte_index = 0;
                }
            }
        }
        time_divider++;
        if (time_divider == 3) {
            time_divider = 0;
        }
    }
    else {
        set_UART_pin_as_input();
        uint8_t sampled_bit = read_UART_pin();
        if (waiting_for_start == true) {
            if (sampled_bit == 0) {
                waiting_for_start = false;
                n_sampled_bits = 0;
            }
        }
        else {
            if (n_sampled_bits == 0) {
                if (bit_index == 0) { // is it a start byte?
                    rx_byte = 0;
                }
                else if (bit_index == 9) { // is it a stop byte?
                    returned_data[(*returned_data_size)++] = rx_byte;
                }
                else {
                    rx_byte >>= 1;
                    rx_byte |= (sampled_bit << 7);
                }
                bit_index++;
                if (bit_index == 10) {
                    bit_index = 0;
                    byte_index++;
                    if (byte_index == 8) {
                        rx_phase = false;
                        byte_index = 0;
                        register_index++;
                        time_divider = 0;
                        end_of_read_delay = 255;
                        if (register_index >= sizeof(registers_to_read)) {
                            register_index = 0;
                            return true;
                        }
                        else {
                            return false;
                        }
                    }
                }
            }
            n_sampled_bits++;
            if(n_sampled_bits == 3) {
                n_sampled_bits = 0;
            }
        }
    }
    return false;
}

int main(void)
{
    uint8_t returned_data[1000];
    uint8_t returned_data_size = 0;
    uint32_t complete_iterations = 0;

    // open three text files for logging the output bit stream, the input bit stream, and the input/output mode of the gpio pin
    output_bit_stream_fd = fopen("output_bit_stream.txt", "w");
    if (output_bit_stream_fd == NULL) {
        printf("Error: could not open output_bit_stream.txt\n");
        exit(1);
    }
    input_bit_stream_fd = fopen("input_bit_stream.txt", "w");
    if (input_bit_stream_fd == NULL) {
        printf("Error: could not open input_bit_stream.txt\n");
        exit(1);
    }
    input_output_mode_fd = fopen("input_output_mode.txt", "w");
    if (input_output_mode_fd == NULL) {
        printf("Error: could not open input_output_mode.txt\n");
        exit(1);
    }

    while(1) {
        printf("Start of next iteration at time_counter: %d\n", time_counter);
        bool finished = GC6608_UART_bit_bang_read_registers(returned_data, &returned_data_size);
        if (finished) {
            break;
        }
        time_counter++;
        complete_iterations++;
        if (complete_iterations >= MAX_ITERATIONS) {
            printf("Error: did not complete all operations in the given maximum iterations (%d)\n", MAX_ITERATIONS);
            exit(1);
        }
    }
    printf("Finished reading registers. Now, let's print out the data:\n");
    for (int i = 0; i < returned_data_size; i++) {
        printf("0x%02x ", returned_data[i]);
    }
    printf("\n");

    // close the files
    fclose(output_bit_stream_fd);
    fclose(input_bit_stream_fd);
    fclose(input_output_mode_fd);

    return 0;
}