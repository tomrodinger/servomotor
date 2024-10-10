#!/usr/bin/env bash

gcc -Wall -ferror-limit=1000 motor_control_simulation.c ../servomotor/Src/motor_control.c ../servomotor/Src/mosfets.c stm32g0xx_hal.c simulated_functions.c debug_uart.c ADC.c -DPRODUCT_NAME_M4 -o motor_control_simulation -I . -I ../common_source_files -I ../servomotor/Src