#!/usr/bin/env bash

gcc -Wall -ferror-limit=1000 motor_control_simulation.c ../firmware/Src/motor_control.c ../firmware/Src/mosfets.c stm32g0xx_hal.c simulated_functions.c debug_uart.c ADC.c -DPRODUCT_NAME_M4 -o motor_control_simulation -I . -I ../common_source_files -I ../firmware/Src
gcc -Wall -ferror-limit=1000 test_add_to_queue.c        ../firmware/Src/motor_control.c ../firmware/Src/mosfets.c stm32g0xx_hal.c simulated_functions.c debug_uart.c ADC.c -DPRODUCT_NAME_M3 -o test_add_to_queue        -I . -I ../common_source_files -I ../firmware/Src
gcc -Wall -ferror-limit=1000 test_PID_controller.c      ../firmware/Src/motor_control.c ../firmware/Src/mosfets.c stm32g0xx_hal.c simulated_functions.c debug_uart.c ADC.c -DPRODUCT_NAME_M3 -o test_PID_controller_M3   -I . -I ../common_source_files -I ../firmware/Src
gcc -Wall -ferror-limit=1000 test_PID_controller.c      ../firmware/Src/motor_control.c ../firmware/Src/mosfets.c stm32g0xx_hal.c simulated_functions.c debug_uart.c ADC.c -DPRODUCT_NAME_M4 -o test_PID_controller_M4   -I . -I ../common_source_files -I ../firmware/Src
gcc -Wall -ferror-limit=1000 test_add_int96.c           ../firmware/Src/motor_control.c ../firmware/Src/mosfets.c stm32g0xx_hal.c simulated_functions.c debug_uart.c ADC.c -DPRODUCT_NAME_M4 -o test_add_int96           -I . -I ../common_source_files -I ../firmware/Src
make
