################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ADC.c \
../Core/Src/PWM.c \
../Core/Src/RS485.c \
../Core/Src/debug_uart.c \
../Core/Src/error_handling.c \
../Core/Src/hall_sensor_calculations.c \
../Core/Src/leds.c \
../Core/Src/main.c \
../Core/Src/microsecond_clock.c \
../Core/Src/mosfets.c \
../Core/Src/motor_control.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c 

OBJS += \
./Core/Src/ADC.o \
./Core/Src/PWM.o \
./Core/Src/RS485.o \
./Core/Src/debug_uart.o \
./Core/Src/error_handling.o \
./Core/Src/hall_sensor_calculations.o \
./Core/Src/leds.o \
./Core/Src/main.o \
./Core/Src/microsecond_clock.o \
./Core/Src/mosfets.o \
./Core/Src/motor_control.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o 

C_DEPS += \
./Core/Src/ADC.d \
./Core/Src/PWM.d \
./Core/Src/RS485.d \
./Core/Src/debug_uart.d \
./Core/Src/error_handling.d \
./Core/Src/hall_sensor_calculations.d \
./Core/Src/leds.d \
./Core/Src/main.d \
./Core/Src/microsecond_clock.d \
./Core/Src/mosfets.d \
./Core/Src/motor_control.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ADC.o: ../Core/Src/ADC.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/ADC.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/PWM.o: ../Core/Src/PWM.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/PWM.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/RS485.o: ../Core/Src/RS485.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/RS485.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/debug_uart.o: ../Core/Src/debug_uart.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/debug_uart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/error_handling.o: ../Core/Src/error_handling.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/error_handling.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/hall_sensor_calculations.o: ../Core/Src/hall_sensor_calculations.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/hall_sensor_calculations.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/leds.o: ../Core/Src/leds.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/leds.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/microsecond_clock.o: ../Core/Src/microsecond_clock.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/microsecond_clock.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/mosfets.o: ../Core/Src/mosfets.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/mosfets.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/motor_control.o: ../Core/Src/motor_control.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/motor_control.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/syscalls.o: ../Core/Src/syscalls.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/sysmem.o: ../Core/Src/sysmem.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS
Core/Src/system_stm32g0xx.o: ../Core/Src/system_stm32g0xx.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32g0xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" -DZ_AXIS

