#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32g0xx_hal.h"
#include "coin_cell_control_and_calculations.h"
#include "ADC.h"
#include "debug_uart.h"
#include "error_handling.h"

#define COIN_CELL_MILLIVOLTS_CHANGE_HYSTERESIS 5
#define CALIBRATION_FACTOR ((((uint32_t)(4096 - 1) * 256) >> 4) * DMA_ADC_BUFFER_REPEAT_CYCLES * DMA_ADC_BUFFER_VALUES_PER_CYCLE * 256 / 3208)

static volatile uint16_t ADC_buffer[DMA_ADC_BUFFER_SIZE];

#define GPIO_LOAD_MASK_OUT_MASK (~GPIO_ALL_LOADS_OFF) // This is the mask to use to clear (with &= operation) the load control pins in the GPIOx->ODR register

#define N_INTERVALS 200 // keep this to a maximum of 255
#define DEFAULT_INTERVAL_DURATION 1
#define DEFAULT_INTERVAL_LOAD_STATE 0xff
#define SHUNT_RESISTANCE_OHMS 5 // this is the shunt resistor in series with the load

typedef struct __attribute__((__packed__)) {
    uint8_t load_state;
    uint32_t duration;
} interval_t;
static volatile interval_t intervals[N_INTERVALS];
static volatile uint16_t n_intervals = 0;
static volatile uint16_t interval_index = 0;
static volatile uint16_t sequence_count = 0;
static volatile interval_run_mode_t interval_run_mode = STOPPED;

#define COMPRESSED_MILLIVOLTS_REPRESENTATION_BITS 9
#define COMPRESSED_MILLIVOLTS_REPRESENTATION_MAX_VALUE ((1 << COMPRESSED_MILLIVOLTS_REPRESENTATION_BITS) - 1)
typedef struct __attribute__((__packed__)) {
    uint16_t max_millivolts:COMPRESSED_MILLIVOLTS_REPRESENTATION_BITS;
    uint16_t min_millivolts:COMPRESSED_MILLIVOLTS_REPRESENTATION_BITS;
    uint16_t sequence_count:14;
    uint8_t interval_index;
} compressed_data_record_t;

#define N_DATA_RECORDS 400
static volatile compressed_data_record_t compressed_data_record[N_DATA_RECORDS];
static volatile uint16_t data_record_add_index = 0;
static volatile uint16_t data_record_take_index = 0;
static volatile uint16_t n_data_records = 0;
static volatile uint8_t data_record_overflow = 0;

static volatile uint16_t manual_load_state = 0;
static volatile uint32_t manual_pulse_duration = 0;
static volatile uint16_t load_state = 0;
static volatile uint64_t microsecond_time = 0;
static volatile uint32_t pulse_duration_counter = 0;
#define DATA_RECORD_VOLTAGE_DIVIDE 10
static volatile uint32_t sample_counter = 0;
static volatile uint32_t coin_cell_millivolts = 0;
static volatile uint64_t picoamp_seconds_count = 0;

static volatile uint16_t time_difference1 = 0;
static volatile uint16_t max_time_difference1 = 0;
static volatile uint16_t start_time2 = 0;
static volatile uint16_t time_difference2 = 0;
static volatile uint16_t min_time_difference2 = 0xffff;
static volatile uint16_t max_time_difference2 = 0;
static volatile uint16_t time_difference3 = 0;
static volatile uint16_t max_time_difference3 = 0;

static const uint32_t load_resistances_ohms[] = { 300000, 60000, 15000, 3000, 600, 300, 150, 75 };


static volatile uint32_t value_min = 0xffffffff;
static volatile uint32_t value_max = 0;


void reset_min_and_max(void)
{
    value_min = 0xffffffff;
    value_max = 0;
}


void calculate_min_and_max(uint32_t value)
{
    if (value < value_min) {
        value_min = value;
    }
    if (value > value_max) {
        value_max = value;
    }
}


uint32_t get_min(void)
{
    return value_min;
}


uint32_t get_max(void)
{
    return value_max;
}


void coin_cell_control_and_calculations_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_TIM16EN; // enable the clock to TIM16
    TIM16->PSC = 0; // prescale is 1, so the timer will count at 64 MHz
    TIM16->ARR = 64000;  // we want an overflow once every millisecond
    TIM16->DIER = TIM_DIER_UIE; // enable the interrupt for the update event (overflow)
    TIM16->CR1 = TIM_CR1_CEN; // enable the timer
    TIM16->EGR |= TIM_EGR_UG; // do an update (load and clear the prescaler)
    NVIC_EnableIRQ(TIM16_IRQn); // enable the interrupt to this timer
}


// Turn on the specified load
void load_on(uint8_t load_number)
{
    if (load_number >= N_LOADS) {
        return;
    }
    manual_load_state |= (1 << load_number);
    manual_pulse_duration = 0;
    interval_run_mode = HOLD_LOAD_STATE;
}


// Turn off the specified load
void load_off(uint8_t load_number)
{
    if (load_number >= N_LOADS) {
        return;
    }
    manual_load_state &= ~(1 << load_number);
    manual_pulse_duration = 0;
    interval_run_mode = HOLD_LOAD_STATE;
}


// Toggle the on/off state of the specified load
void load_toggle_on_or_off(uint8_t load_number)
{
    if (load_number >= N_LOADS) {
        return;
    }
    manual_load_state ^= (1 << load_number);
    manual_pulse_duration = 0;
    interval_run_mode = HOLD_LOAD_STATE;
}


// Get the state of the load (0 = off, 1 = on)
uint8_t get_load_state(uint8_t load_number)
{
    if (load_number >= N_LOADS) {
        return 0;
    }
    return ((load_state >> load_number) & 1);
}


uint8_t get_device_status_flags(void)
{
    return (((uint8_t)interval_run_mode & 0xf) << 0) | ((data_record_overflow & 1) << 4) | (((n_data_records > 0) & 1) << 5);
}


void apply_pulse(uint32_t pulse_duration_ms, uint8_t load_number)
{
    if (load_number >= N_LOADS) {
        return;
    }
    NVIC_DisableIRQ(DMA1_Channel1_IRQn); // disable the interrupt for the DMA channel 1
    if((interval_run_mode != STOPPED) && (interval_run_mode != HOLD_LOAD_STATE)) {
        fatal_error(ERROR_CANT_PULSE_WHEN_INTERVALS_ACTIVE); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
    }
    manual_pulse_duration = pulse_duration_ms;
    manual_load_state = (1 << load_number);
    interval_run_mode = MANUAL_PULSE;
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable the interrupt for the DMA channel 1
}


void print_pulse_stats(void)
{
    char buf[60];
    sprintf(buf, "Minimum voltage during pulse: %lu\n", get_min());
    transmit(buf, strlen(buf));
    sprintf(buf, "Maximum voltage before pulse: %lu\n", get_max());
    transmit(buf, strlen(buf));
}


void set_interval_run_mode(interval_run_mode_t run_mode)
{
    NVIC_DisableIRQ(DMA1_Channel1_IRQn); // disable the interrupt for the DMA channel 1
    interval_run_mode = run_mode;
    if(run_mode == STOP_IMMEDIATELY_AND_DELETE_ALL_INTERVALS) {
        for(interval_index = 0; interval_index < N_INTERVALS; interval_index++) {
            intervals[interval_index].duration = DEFAULT_INTERVAL_DURATION;
            intervals[interval_index].load_state = DEFAULT_INTERVAL_LOAD_STATE;
        }
        n_intervals = 0;
        interval_index = 0;
        run_mode = STOPPED;
    }
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable the interrupt for the DMA channel 1
}


void add_new_interval(uint16_t interval_add_index, uint8_t load_state, uint32_t duration)
{
    if(interval_add_index >= N_INTERVALS) {
        if(n_intervals >= N_INTERVALS) {
            fatal_error(ERROR_QUEUE_IS_FULL); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
        }
        interval_add_index = n_intervals;
    }
    NVIC_DisableIRQ(DMA1_Channel1_IRQn); // disable the interrupt for the DMA channel 1
    intervals[interval_add_index].load_state = load_state;
    intervals[interval_add_index].duration = duration;
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable the interrupt for the DMA channel 1
    if(interval_add_index >= n_intervals) {
        n_intervals = interval_add_index + 1;
    }
}


void get_current_interval_status(uint16_t *interval_index_out, uint16_t *n_intervals_out, interval_run_mode_t *interval_run_mode_out,
                                 uint8_t *load_state_out, uint32_t *remaining_duration_out)
{
    NVIC_DisableIRQ(DMA1_Channel1_IRQn); // disable the interrupt for the DMA channel 1
    *interval_index_out = interval_index;
    *n_intervals_out = n_intervals;    
    *interval_run_mode_out = interval_run_mode;
    *load_state_out = load_state;
    *remaining_duration_out = pulse_duration_counter;
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable the interrupt for the DMA channel 1
}


void set_current_interval_index(uint16_t new_interval_index)
{
    if(new_interval_index >= n_intervals) {
        fatal_error(ERROR_INDEX_OUT_OF_RANGE); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
    }
    interval_index = new_interval_index;
}


// This is called in the case of a fatal error. All we need to do is disable all the laods.
void all_loads_off(void)
{
    NVIC_DisableIRQ(DMA1_Channel1_IRQn); // disable the interrupt for the DMA channel 1, which is the only place where loads can be turned on
    GPIOA->ODR = GPIOA->ODR |= GPIO_ALL_LOADS_OFF; // loads are active low, so we set the bits to turn them off
    manual_load_state = 0;
    load_state = 0;
}


volatile uint16_t *get_ADC_buffer_ptr(void)
{
    return ADC_buffer;
}


void print_ADC_values(void)
{
    char buf[100];
    sprintf(buf, "Coin cell voltage: %lu at time: %lu   sample_counter: %lu\n", coin_cell_millivolts, (uint32_t)microsecond_time, sample_counter);
    transmit(buf, strlen(buf));
}


void save_data_record(void)
{
    if(n_data_records < N_DATA_RECORDS) {
        n_data_records++;
    }
    else {
        data_record_take_index++;
        if(data_record_take_index >= N_DATA_RECORDS) {
            data_record_take_index = 0;
        }
        data_record_overflow = 1;
    }

    int32_t coin_cell_max_millivolts_adjusted = (get_max() + (DATA_RECORD_VOLTAGE_DIVIDE >> 1)) / DATA_RECORD_VOLTAGE_DIVIDE;
    if(coin_cell_max_millivolts_adjusted > COMPRESSED_MILLIVOLTS_REPRESENTATION_MAX_VALUE) {
        coin_cell_max_millivolts_adjusted = COMPRESSED_MILLIVOLTS_REPRESENTATION_MAX_VALUE;
    }
    int32_t coin_cell_min_millivolts_adjusted = (get_min() + (DATA_RECORD_VOLTAGE_DIVIDE >> 1)) / DATA_RECORD_VOLTAGE_DIVIDE;
    if(coin_cell_min_millivolts_adjusted > COMPRESSED_MILLIVOLTS_REPRESENTATION_MAX_VALUE) {
        coin_cell_min_millivolts_adjusted = COMPRESSED_MILLIVOLTS_REPRESENTATION_MAX_VALUE;
    }
    compressed_data_record[data_record_add_index].max_millivolts = coin_cell_max_millivolts_adjusted;
    compressed_data_record[data_record_add_index].min_millivolts = coin_cell_min_millivolts_adjusted;
    compressed_data_record[data_record_add_index].interval_index = interval_index;
    compressed_data_record[data_record_add_index].sequence_count = sequence_count;
    data_record_add_index++;
    if(data_record_add_index >= N_DATA_RECORDS) {
        data_record_add_index = 0;
    }
}


void get_data_record(data_record_t *data_record_out)
{
    if(n_data_records > 0) {
        NVIC_DisableIRQ(DMA1_Channel1_IRQn); // disable the interrupt for the DMA channel 1
        data_record_out->max_millivolts = (compressed_data_record[data_record_take_index].max_millivolts) * DATA_RECORD_VOLTAGE_DIVIDE;
        data_record_out->min_millivolts = (compressed_data_record[data_record_take_index].min_millivolts) * DATA_RECORD_VOLTAGE_DIVIDE;
        data_record_out->interval_index = compressed_data_record[data_record_take_index].interval_index;
        data_record_out->sequence_count = compressed_data_record[data_record_take_index].sequence_count;
        data_record_out->n_records_left = n_data_records - 1;
        data_record_out->data_record_overflow = data_record_overflow;
        data_record_overflow = 0;
        data_record_take_index++;
        if(data_record_take_index >= N_DATA_RECORDS) {
            data_record_take_index = 0;
        }
        n_data_records--;
        NVIC_EnableIRQ(DMA1_Channel1_IRQn); // enable the interrupt for the DMA channel 1
    }
    else {
        data_record_out->max_millivolts = 0;
        data_record_out->min_millivolts = 0;
        data_record_out->interval_index = 0;
        data_record_out->sequence_count = 0;
        data_record_out->n_records_left = 0;
        data_record_out->data_record_overflow = 0;
    }
}


void get_data_record_and_print_it(void)
{
    data_record_t data_record;
    get_data_record(&data_record);
    char buf[100];

    if((data_record.max_millivolts != 0) || (data_record.min_millivolts != 0)) {
        sprintf(buf, "Data record: max_millivolts: %hu\n", data_record.max_millivolts);
        transmit(buf, strlen(buf));
        sprintf(buf, "             min_millivolts: %hu\n", data_record.min_millivolts);
        transmit(buf, strlen(buf));
        sprintf(buf, "             interval_index: %hu\n", (uint16_t)data_record.interval_index);
        transmit(buf, strlen(buf));
        sprintf(buf, "             sequence_count: %hu\n", data_record.sequence_count);
        transmit(buf, strlen(buf));
        sprintf(buf, "             n_records_left: %hu\n", data_record.n_records_left);
        transmit(buf, strlen(buf));
        sprintf(buf, "             data_record_overflow: %hu\n", (uint16_t)data_record.data_record_overflow);
        transmit(buf, strlen(buf));
    }
    else {
        transmit("No data record available\n", 25);
    }
}


void DMA1_Channel1_IRQHandler(void)
{
    uint16_t start_time;
    uint16_t end_time;
    uint32_t voltage = 0;
    int32_t i;
    int32_t j;

    start_time = TIM14->CNT;

    time_difference2 = TIM14->CNT - start_time2;
    if(time_difference2 < min_time_difference2) {
        min_time_difference2 = time_difference2;
    }
    if(time_difference2 > max_time_difference2) {
        max_time_difference2 = time_difference2;
    }

    for(i = 0; i < DMA_ADC_BUFFER_SIZE; i += DMA_ADC_BUFFER_VALUES_PER_CYCLE) {
        for(j = 0; j < DMA_ADC_BUFFER_VALUES_PER_CYCLE; j++) {
            voltage += ADC_buffer[i + j];
        }
    }
    coin_cell_millivolts = voltage * 256 / CALIBRATION_FACTOR;
    calculate_min_and_max(coin_cell_millivolts);
/*
    uint8_t load_state_tmp = 0x80;
    if(load_state_tmp & ((1 << N_LOADS) - 1)) { // only if one or more loads are on will we do the following calculation and accumulate the picoamp-seconds
        uint32_t coin_cell_nanovolts = coin_cell_millivolts * 1000000;
        uint32_t shunt_resistance_adjustment_factor = 0;
        uint32_t nanoamps = 0;
        for(i = 0; i < N_LOADS; i++) {
            if(load_state_tmp & (1 << i)) {
                nanoamps += coin_cell_nanovolts / load_resistances_ohms[i];
                #define SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR 1000  // this is here so that the calculation is more precice. if you look at the math, this just cancels out.
                shunt_resistance_adjustment_factor += SHUNT_RESISTANCE_OHMS * SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR / load_resistances_ohms[i];
            }
        }
        shunt_resistance_adjustment_factor += SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR;
        while(nanoamps > (0xffffffff / (SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR + 1))) { // Let's make sure that we don't overflow the 32-bit nanoamps variable in the calculation below where we multiply by SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR.
            nanoamps >>= 1;
            shunt_resistance_adjustment_factor >>= 1;
        }
        nanoamps = nanoamps * SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR / shunt_resistance_adjustment_factor;
        picoamp_seconds_count += nanoamps; // Note that the units stated are right. The thing to know is that this operation happens 1000 times per second, which effectively converts the unit from nanoamps to picoamps.
    }
*/

    if(load_state & ((1 << N_LOADS) - 1)) { // only if one or more loads are on will we do the following calculation and accumulate the picoamp-seconds
        uint32_t coin_cell_nanovolts = coin_cell_millivolts * 1000000;
        uint32_t shunt_resistance_adjustment_factor = 0;
        uint64_t nanoamps = 0;
        for(i = 0; i < N_LOADS; i++) {
            if(load_state & (1 << i)) {
                nanoamps += coin_cell_nanovolts / load_resistances_ohms[i];
                #define SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR 1000000  // this is here so that the calculation is more precise. if you look at the math, this just cancels out.
                shunt_resistance_adjustment_factor += SHUNT_RESISTANCE_OHMS * SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR / load_resistances_ohms[i];
            }
        }
        shunt_resistance_adjustment_factor += SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR;
        nanoamps = nanoamps * SHUNT_RESISTANCE_ADJUSTMENT_FACTOR_SCALE_FACTOR / shunt_resistance_adjustment_factor;
        picoamp_seconds_count += nanoamps; // Note that the units stated are right. The thing to know is that this operation happens 1000 times per second, which effectively converts the unit from nanoamps to picoamps.
    }


    sample_counter++;

    end_time = TIM14->CNT;
    time_difference3 = end_time - start_time;
    if(time_difference3 > max_time_difference3) {
        max_time_difference3 = time_difference3;
    }

    DMA1->IFCR = 0xf; // clear all interrupt flags for DMA channel 1
}


void TIM16_IRQHandler(void)
{
    uint16_t start_time;
    uint16_t end_time;

    start_time = TIM14->CNT;

    if(DMA1->ISR & 0xf) { // if any of the interrupt flags for DMA channel 1 are set, it means that we are still in the process of processing the previous data
        fatal_error(ERROR_CONTROL_LOOP_TOOK_TOO_LONG); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
    }

    if(pulse_duration_counter > 0) {
        pulse_duration_counter--;
    }

    switch(interval_run_mode) {
    case STOP_IMMEDIATELY_AND_DELETE_ALL_INTERVALS:
    case STOPPED:
        pulse_duration_counter = 0;
        load_state = 0;
        break;
    case HOLD_LOAD_STATE:
        pulse_duration_counter = 0;
        load_state = manual_load_state;
        break;
    case MANUAL_PULSE:
        pulse_duration_counter = manual_pulse_duration;
        load_state = manual_load_state;
        manual_load_state = 0;
        interval_run_mode = ALLOW_ONLY_CURRENT_INTERVAL_TO_FINISH_THEN_STOP;
        break;
    case ALLOW_ONLY_CURRENT_INTERVAL_TO_FINISH_THEN_STOP:
        if(pulse_duration_counter == 0) {
            save_data_record();
            reset_min_and_max();
            interval_run_mode = STOPPED;
            load_state = 0;
        }
        break;
    case ALLOW_ALL_INTERVALS_TO_FINISH_THEN_STOP:
        if(pulse_duration_counter == 0) {
            save_data_record();
            reset_min_and_max();
            interval_index++;
            if(interval_index >= n_intervals) {
                interval_index = 0;
                interval_run_mode = STOPPED;
                load_state = 0;
            }
            else {
                pulse_duration_counter = intervals[interval_index].duration;
                load_state = intervals[interval_index].load_state;
            }
        }
        break;
    case RUN_CONTINUOUSLY:
        if(pulse_duration_counter == 0) {
            save_data_record();
            reset_min_and_max();
            interval_index++;
            if(interval_index >= n_intervals) {
                interval_index = 0;
                sequence_count++;
            }
            pulse_duration_counter = intervals[interval_index].duration;
            load_state = intervals[interval_index].load_state;
        }
        break;
    }

    ADC1->CR |= ADC_CR_ADSTART; // start to do the conversions one by one all over again until the DMA buffer is full again
    start_time2 = TIM14->CNT;

    GPIOA->ODR = (GPIOA->ODR & GPIO_LOAD_MASK_OUT_MASK) | (~(load_state << LOAD_CONTROL_PIN_START_INDEX));

    end_time = TIM14->CNT;
    time_difference1 = end_time - start_time;
    if(time_difference1 > max_time_difference1) {
        max_time_difference1 = time_difference1;
    }

    TIM16->SR = 0; // set all interrupt flags for the timer to zero
}


uint64_t get_picoamp_seconds_count(void)
{
    return picoamp_seconds_count;
}


void print_picoamp_seconds_count(void)
{
    char buf[80];
    uint32_t microamp_seconds_count_decimal = (uint32_t)((uint64_t)picoamp_seconds_count / (uint64_t)1000000);
    sprintf(buf, "%lu microamp seconds\r", microamp_seconds_count_decimal);
    transmit(buf, strlen(buf));
}


void print_intervals(void)
{
    char buf[80];
    int32_t i;
    if(n_intervals == 0) {
        transmit("No intervals defined\n", 21);
    }
    else {
        for(i = 0; i < n_intervals; i++) {
            sprintf(buf, "Interval %ld: load_state: %hu   duration: %lu\r", i, (uint16_t)intervals[i].load_state, intervals[i].duration);
            transmit(buf, strlen(buf));
        }
    }
}

void print_time_difference(void)
{
    char buf[150];

    sprintf(buf, "time_difference1: %hu   max_time_difference1: %hu\n", time_difference1, max_time_difference1);
    transmit(buf, strlen(buf));
    sprintf(buf, "time_difference3: %hu   max_time_difference3: %hu\n", time_difference3, max_time_difference3);
    transmit(buf, strlen(buf));
    sprintf(buf, "time_difference2: %hu   min_time_difference2: %hu   max_time_difference2: %hu\n", time_difference2, min_time_difference2, max_time_difference2);
    transmit(buf, strlen(buf));
    max_time_difference1 = 0;
    max_time_difference3 = 0;
    min_time_difference2 = 0xffff;
    max_time_difference2 = 0;
}

void print_millivolts_if_changed(void)
{
    static uint32_t previous_coin_cell_millivolts = 0;
    uint8_t print_voltage_flag = 0;
    char buf[50];

    if(coin_cell_millivolts > previous_coin_cell_millivolts) {
        if(coin_cell_millivolts - previous_coin_cell_millivolts >= COIN_CELL_MILLIVOLTS_CHANGE_HYSTERESIS) {
            print_voltage_flag = 1;
            previous_coin_cell_millivolts = coin_cell_millivolts;
        }
    }
    else if(coin_cell_millivolts < previous_coin_cell_millivolts) {
        if(previous_coin_cell_millivolts - coin_cell_millivolts >= COIN_CELL_MILLIVOLTS_CHANGE_HYSTERESIS) {
            print_voltage_flag = 1;
            previous_coin_cell_millivolts = coin_cell_millivolts;
        }
    }

    if(print_voltage_flag) {
        sprintf(buf, "Millivolts: %lu\n", coin_cell_millivolts);
        transmit(buf, strlen(buf));
    }   
}

