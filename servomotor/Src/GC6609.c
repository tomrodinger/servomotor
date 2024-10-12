#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "gpio.h"
#include "GC6609.h"

#define GPIOA_GC6609_UART_PIN 15
#define GPIOA_GC6609_UART_MODER_POS GPIO_MODER_MODE15_Pos // set this correspondingly to whatever GPIOA_GC6609_UART_PIN is set to

void delay_us(uint32_t us)
{
    // This function will delay for the specified number of microseconds
    uint32_t delay = us * 4;
    for (volatile int i = 0; i < delay; i++) {
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
    }
}


void Delay_ms(uint32_t ms)
{
    // This function will delay for the specified number of milliseconds
    for (int i = 0; i < ms; i++) {
        delay_us(1000);
    }
}


// We will send a but, which involves setting the state of the GPIO pin and then waiting for the correct amount of time for the baud rate that we want
// Interrupts are disabled in this operation, so there is no risk that the time delay will be interrupted and extended.
// The UART interface of the GC6609 is connected to the pin stated in GPIOA_GC6609_UART_PIN, so we will use this pin to send the data.
void send_bit(uint8_t bit)
{
    if (bit) {
        GPIOA->BSRR = (1 << GPIOA_GC6609_UART_PIN);   
    }
    else {
        GPIOA->BSRR = ((1 << GPIOA_GC6609_UART_PIN) << 16);
    }
    delay_us(104); // 9600 baud
}


// This function sends a byte over the UART interface. Each byte is 8 bts ovbiously, but we also need a start bit before and a stop bit after.
void send_byte(uint8_t byte)
{
    // Send start bit
    send_bit(0);

    // Send data bits
    for (int i = 0; i < 8; i++) {
        send_bit((byte >> i) & 1);
    }

    // Send stop bit
    send_bit(1);
}


// This function sends several bytes from a buffer over the UART interface
void Send_char(uint8_t *buffer, uint8_t length)
{
    // Send the bytes one by one by calling the send_byte function
    for (int i = 0; i < length; i++) {
        send_byte(buffer[i]);
    }
}


// 6609 Serial communication verification algorithm (colied from their reference manual)
// CRC calculation
// CRC = (CRC << 1) OR (CRC.7 XOR CRC.1 XOR CRC.0 XOR [new incoming bit]) //CRC = x8 + x2 + x1 + x0
void swuart_calcCRC(uint8_t* datagram, uint8_t datagramLength)
{
    int i, j;
    uint8_t *crc = datagram + (datagramLength - 1); // CRC located in last byte of message
    uint8_t currentByte;
    *crc = 0;

    for (i = 0; i < (datagramLength - 1); i++) { // Execute for all bytes of a message
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++) {
            if ((*crc >> 7) ^ (currentByte&0x01)) { // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            }
            else {
                *crc = (*crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
}


// Write 6609 register
void Write_6609_REG(uint8_t WReg_addrr, uint32_t Write_data)
{
    uint8_t Write_data_temp[8];
    uint8_t Write_data3, Write_data2, Write_data1, Write_data0;
    uint8_t Syn;
    uint8_t Slave;
    uint8_t CRC_data;

    Syn = 0x05; // Synchronization + reservation
    Slave = 0x00; // 8-bit slave address
    CRC_data = 0x00; // CRC check initialization 0
    Write_data3 = Write_data >> 24;
    Write_data2 = Write_data >> 16;
    Write_data1 = Write_data >> 8;
    Write_data0 = Write_data;

    Write_data_temp[0] = Syn;
    Write_data_temp[1] = Slave;
    Write_data_temp[2] = WReg_addrr | 0x80;
    Write_data_temp[3] = Write_data3; // Write data high 4 bits
    Write_data_temp[4] = Write_data2; // Write data high 3 bits
    Write_data_temp[5] = Write_data1; // Write data high 2 bits
    Write_data_temp[6] = Write_data0; // Write data high 1 bits
    Write_data_temp[7] = CRC_data;    //CRC Check value
     
    swuart_calcCRC(Write_data_temp, 8); // Calculate CRC check value
    Delay_ms(1); //delay 1ms
    Send_char(Write_data_temp, 8); // Send 8 bytes of data to 6609through serial port
}


void enable_16_microstepping(void)
{
    #define EN_SC 1 // 1 means use SCC mode always, 0 means use SCC at high speed and SC2 at low speed
    #define MULTISTEP_FILT 0 // 0 = no filtering, 1 = filtering
    Write_6609_REG(0x00, 0x000000c1 | (EN_SC << 2) | (MULTISTEP_FILT << 8)); // mstep_reg_select set to 1 (to choose the microstepping resolution using the MRES bits) and disable HOLDEN pin function (use that pin as UART)
//    Write_6609_REG(0x6c, 0x14010053); // intpol set to 0
//    Write_6609_REG(0x6c, 0x14010053); // intpol set to 0, set MRES
    // Default: 0xc80d0e24: 1100 1000 0000 1101 0000 1110 0010 0100                 Write_6609_REG(0x70, 0xc80d0e24); // set some PWM parameters
    //         change:      1100 1000 0000 1101 0000 1110 0010 0100
    //                                     ^^--here
//    Write_6609_REG(0x70, 0xcf010e24); // set some PWM parameters

    #define INTPOL 0 // interpolate to 256 microsteps enable flag
    #define TBL 2 // 2 bits, default is 2, blanking time 16, 24, 32 or 40 clocks
    #define MRES 4 // 1/16 microstepping
    #define HEND 9 // default 0 (maps to -3)  0 to 15 maps to -3 to 12
    #define HSTRT 7 // default 5 (maps to 6)  0 to 7 maps to 1 to 8    HEND+HSTRT=16
    #define TOFF 1 // default 3 (don't use 0 because that will disable the driver. if using 1 then TBL should be >= 2), a lower value here seems to decrease high pitched sounds
    Write_6609_REG(0x6c, (HSTRT << 4) | (HEND << 7) | (INTPOL << 28) | (TBL << 15) | (MRES << 24) | (TOFF << 0)); // intpol set to 0, set MRES to 1/16 microstepping

    #define PWM_LIM 12      // 4 bits (default 12)
    #define PWM_REG 8       // 4 bits (default 8)
    #define PWM_AUTOGRAD 1  // 1 bit  (default 1)
    #define PWM_AUTOSCALE 1 // 1 bit  (default 1)
    #define PWM_FREQ 1      // 2 bits (default 1)
    #define PWM_GRAD 14     // 8 bits (default 14)
    #define PWM_OFS 36      // 8 bits (default 36)
    Write_6609_REG(0x70, (PWM_LIM << 28) | (PWM_REG << 24) | (PWM_AUTOGRAD << 19) | (PWM_AUTOSCALE << 18) | (PWM_FREQ << 16) | (PWM_GRAD << 8) | (PWM_OFS << 0)); // intpol set to 0, set MRES to 1/16 microstepping

    #define IHOLD 31
    #define IRUN 31
    #define IHOLDDELAY 15
    #define TPOWERDOWN 256
//    #define TPWMTHRS 1048575
    #define TPWMTHRS 0
    Write_6609_REG(0x10, (IHOLD << 0) | (IRUN << 8) | (IHOLDDELAY << 16));
    Write_6609_REG(0x11, (TPOWERDOWN << 0));
    Write_6609_REG(0x13, (TPWMTHRS << 0));
} 


void reset_chip(void)
{
    Write_6609_REG(0x00, 0x00000141); // HOLDEN input function disabled. Set this bit when using the UART interface!
    Write_6609_REG(0x01, 0x00000001); // reset the chip
}

// We will bit-bang out some initialization data for the GC6609 motor driver chip, which has a UART interface.
// The minimum baud rate is around 9600 baud. It can be any baud rate above this minimum. The chip will autodetect
// it using a sync byte at the beginning of each transmission.
void init_GC6609_through_UART(void)
{
    enable_16_microstepping();
//    reset_chip();
//    Write_6609_REG(0x22, 0x000000ff); // cause the motor to turn by itself without pulses to the step pin
//    Write_6609_REG(0x00, 0x00000141); // HOLDEN input function disabled. Set this bit when using the UART interface!
//    Write_6609_REG(0x10, (31 << 0) | (31 << 5) | (15 << 10)); // set the power down delay to maximum, which is about 5.6 seconds
//    Write_6609_REG(0x11, 0x000000ff); // set the power down delay to maximum, which is about 5.6 seconds
}

void power_on_GC6609(void)
{
    GPIOB->BSRR = (1 << 4); // set VIO high
    GPIOB->BSRR = (1 << 5); // set other VIO high
    GPIOB->BSRR = (1 << 0); // set MENABLE high to disable the motor for now
    GPIOA->BSRR = (1 << 15); // set HOLDEN high (we will use this for communication over the UART)
}

void power_off_GC6609(void)
{
    TIM1->CCR1 = 0; // set the PWM duty cycle to 0, which controls the current control reference
    GPIOA->BSRR = ((1 << 0) << 16); // set DIR low
    GPIOA->BSRR = ((1 << 1) << 16); // set STEP low
    GPIOA->BSRR = ((1 << 15) << 16); // set HOLDEN low
    GPIOB->BSRR = ((1 << 4) << 16); // set VIO low
    GPIOB->BSRR = ((1 << 5) << 16); // set other VIO low
    GPIOB->BSRR = ((1 << 0) << 16); // set MENABLE low so that VIO is not driven high through the clamping diodes
}

void reset_GC6609(void)
{
    power_off_GC6609();
    Delay_ms(100);
    power_on_GC6609();
    Delay_ms(10);
}

inline void set_UART_pin_as_output(void)
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
    return (GPIOA->IDR & (1 << GPIOA_GC6609_UART_PIN)) != 0;
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
uint8_t registers_to_read[] = {0x00, 0x01, 0x6a, 0x6b, 0x6c, 0x6c, 0x6f, 0x70, 0x71, 0x72};
//uint8_t registers_to_read[] = {0x6c};
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

bool GC6608_UART_bit_bang_read_registers(uint8_t *returned_data, uint16_t *returned_data_size)
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
                    crc = 0;
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

void test_motor_stepping(void)
{
    uint32_t acceleration_delay = 1000;
    GPIOA->BSRR = (1 << 0); // set DIR high
    for (int i = 0; i < 100000; i++) {
        GPIOA->BSRR = (1 << 1); // set STEP high
        delay_us(5);
        GPIOA->BSRR = ((1 << 1) << 16); // set STEP low
        delay_us(acceleration_delay);
        if (acceleration_delay > 13 * 2) {
            acceleration_delay--;
        }
    }
}