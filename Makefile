HEADERS = Src/global_variables.h   Src/ADC.h   Src/RS485.h   Src/error_handling.h   Src/mosfets.h   Src/step_direction_input.h   Src/CommutationTable.h   Src/clock_calibration.h   Src/hall_sensor_calculations.h   Src/motor_control.h   Src/stm32g0xx_hal_conf.h   Src/LookupTableE.h   Src/LookupTableZ.h   Src/commands.h   Src/leds.h   Src/product_info.h   Src/stm32g0xx_it.h   Src/LookupTableSmallZ.h   Src/PWM.h   Src/debug_uart.h   Src/microsecond_clock.h   Src/settings.h   Src/unique_id.h   Src/error_text.h   Src/overvoltage.h   Src/device_status.h
OBJECTS = build/startup_stm32g030c8tx.o   build/global_variables.o   build/ADC.o   build/PWM.o   build/RS485.o   build/clock_calibration.o   build/debug_uart.o   build/error_handling.o   build/hall_sensor_calculations.o   build/leds.o   build/main.o   build/microsecond_clock.o   build/mosfets.o   build/motor_control.o   build/step_direction_input.o   build/syscalls.o   build/sysmem.o   build/system_stm32g0xx.o   build/unique_id.o   build/settings.o   build/error_text.o   build/overvoltage.o   build/device_status.o
GCC_DIR=toolchain_essentials/bin/

default: build/firmware_STM32G030.firmware

build/startup_stm32g030c8tx.o: Src/startup_stm32g030c8tx.s
	$(GCC_DIR)/arm-none-eabi-gcc -mcpu=cortex-m0plus -c -x assembler-with-cpp -MMD -MP -MF"build/startup_stm32g030c8tx.d" -MT"build/startup_stm32g030c8tx.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/startup_stm32g030c8tx.o" "Src/startup_stm32g030c8tx.s"

build/global_variables.o: Src/global_variables.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/global_variables.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/global_variables.d" -MT"build/global_variables.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/global_variables.o" -DZ_AXIS

build/ADC.o: Src/ADC.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/ADC.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/ADC.d" -MT"build/ADC.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/ADC.o" -DZ_AXIS

build/PWM.o: Src/PWM.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/PWM.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/PWM.d" -MT"build/PWM.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/PWM.o" -DZ_AXIS

build/RS485.o: Src/RS485.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/RS485.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/RS485.d" -MT"build/RS485.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/RS485.o" -DZ_AXIS

build/clock_calibration.o: Src/clock_calibration.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/clock_calibration.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/clock_calibration.d" -MT"build/clock_calibration.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/clock_calibration.o" -DZ_AXIS

build/debug_uart.o: Src/debug_uart.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/debug_uart.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/debug_uart.d" -MT"build/debug_uart.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/debug_uart.o" -DZ_AXIS

build/error_handling.o: Src/error_handling.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/error_handling.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/error_handling.d" -MT"build/error_handling.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/error_handling.o" -DZ_AXIS

build/hall_sensor_calculations.o: Src/hall_sensor_calculations.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/hall_sensor_calculations.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/hall_sensor_calculations.d" -MT"build/hall_sensor_calculations.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/hall_sensor_calculations.o" -DZ_AXIS

build/leds.o: Src/leds.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/leds.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/leds.d" -MT"build/leds.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/leds.o" -DZ_AXIS

build/main.o: Src/main.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/main.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/main.d" -MT"build/main.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/main.o" -DZ_AXIS

build/microsecond_clock.o: Src/microsecond_clock.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/microsecond_clock.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/microsecond_clock.d" -MT"build/microsecond_clock.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/microsecond_clock.o" -DZ_AXIS

build/mosfets.o: Src/mosfets.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/mosfets.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/mosfets.d" -MT"build/mosfets.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/mosfets.o" -DZ_AXIS

build/motor_control.o: Src/motor_control.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/motor_control.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/motor_control.d" -MT"build/motor_control.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/motor_control.o" -DZ_AXIS

build/step_direction_input.o: Src/step_direction_input.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/step_direction_input.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/step_direction_input.d" -MT"build/step_direction_input.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/step_direction_input.o" -DZ_AXIS

build/syscalls.o: Src/syscalls.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/syscalls.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/syscalls.d" -MT"build/syscalls.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/syscalls.o" -DZ_AXIS

build/sysmem.o: Src/sysmem.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/sysmem.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/sysmem.d" -MT"build/sysmem.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/sysmem.o" -DZ_AXIS

build/system_stm32g0xx.o: Src/system_stm32g0xx.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/system_stm32g0xx.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/system_stm32g0xx.d" -MT"build/system_stm32g0xx.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/system_stm32g0xx.o" -DZ_AXIS

build/unique_id.o: Src/unique_id.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/unique_id.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/unique_id.d" -MT"build/unique_id.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/unique_id.o" -DZ_AXIS

build/settings.o: Src/settings.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/settings.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/settings.d" -MT"build/settings.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/settings.o" -DZ_AXIS

build/error_text.o: Src/error_text.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/error_text.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/error_text.d" -MT"build/error_text.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/error_text.o" -DZ_AXIS

build/overvoltage.o: Src/overvoltage.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/overvoltage.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/overvoltage.d" -MT"build/overvoltage.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/overvoltage.o" -DZ_AXIS

build/device_status.o: Src/device_status.c $(HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "Src/device_status.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -ISrc -IDrivers/STM32G0xx_HAL_Driver/Inc -IDrivers/STM32G0xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32G0xx/Include -IDrivers/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"build/device_status.d" -MT"build/device_status.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "build/device_status.o" -DZ_AXIS

build/firmware_STM32G030.elf: $(OBJECTS) memory_layout.ld
	$(GCC_DIR)/arm-none-eabi-gcc -o "build/firmware_STM32G030.elf" $(OBJECTS) -mcpu=cortex-m0plus -T"memory_layout.ld" --specs=nosys.specs -Wl,-Map="build/firmware_STM32G030.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	$(GCC_DIR)/arm-none-eabi-size   build/firmware_STM32G030.elf
	$(GCC_DIR)/arm-none-eabi-objdump -h -S  build/firmware_STM32G030.elf  > "build/firmware_STM32G030.list"

build/firmware_STM32G030.bin: build/firmware_STM32G030.elf
	$(GCC_DIR)/arm-none-eabi-objcopy  -O binary  build/firmware_STM32G030.elf  "build/firmware_STM32G030.bin"

build/firmware_STM32G030.firmware: build/firmware_STM32G030.bin
	./bin2firmware build/firmware_STM32G030.bin build/firmware_STM32G030.firmware M1 1


clean:
	-rm -f build/*

program: build/firmware_STM32G030.firmware
	python_tests/upgrade_firmware.py build/firmware_STM32G030.firmware
