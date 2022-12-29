COMMON_SOURCE_FILES_DIR = ../common_source_files
DEVICE_SOURCE_FILES_DIR = Src
DRIVERS_DIR = ../Drivers
BUILD_DIR = build
HEADERS = ${DEVICE_SOURCE_FILES_DIR}/global_variables.h   ${DEVICE_SOURCE_FILES_DIR}/ADC.h   ${DEVICE_SOURCE_FILES_DIR}/commands.h   ${DEVICE_SOURCE_FILES_DIR}/PWM.h   ${DEVICE_SOURCE_FILES_DIR}/device_specific_settings.h
COMMON_HEADERS = ${COMMON_SOURCE_FILES_DIR}/RS485.h   ${COMMON_SOURCE_FILES_DIR}/error_handling.h   ${COMMON_SOURCE_FILES_DIR}/clock_calibration.h   ${COMMON_SOURCE_FILES_DIR}/stm32g0xx_hal_conf.h   ${COMMON_SOURCE_FILES_DIR}/stm32g0xx_it.h   ${COMMON_SOURCE_FILES_DIR}/leds.h   ${COMMON_SOURCE_FILES_DIR}/product_info.h   ${COMMON_SOURCE_FILES_DIR}/debug_uart.h   ${COMMON_SOURCE_FILES_DIR}/microsecond_clock.h   ${COMMON_SOURCE_FILES_DIR}/settings.h   ${COMMON_SOURCE_FILES_DIR}/unique_id.h   ${COMMON_SOURCE_FILES_DIR}/error_text.h   ${COMMON_SOURCE_FILES_DIR}/device_status.h
OBJECTS = ${BUILD_DIR}/startup_stm32g030c8tx.o   ${BUILD_DIR}/global_variables.o   ${BUILD_DIR}/ADC.o   ${BUILD_DIR}/PWM.o   ${BUILD_DIR}/RS485.o   ${BUILD_DIR}/clock_calibration.o   ${BUILD_DIR}/debug_uart.o   ${BUILD_DIR}/error_handling.o   ${BUILD_DIR}/leds.o   ${BUILD_DIR}/main.o   ${BUILD_DIR}/microsecond_clock.o   ${BUILD_DIR}/mosfets.o   ${BUILD_DIR}/coin_cell_control_and_calculations.o   ${BUILD_DIR}/system_stm32g0xx.o   ${BUILD_DIR}/unique_id.o   ${BUILD_DIR}/settings.o   ${BUILD_DIR}/error_text.o   ${BUILD_DIR}/device_status.o
GCC_DIR=../toolchain_essentials/bin/

default: ${BUILD_DIR}/firmware_STM32G030.firmware

${BUILD_DIR}/startup_stm32g030c8tx.o: ${COMMON_SOURCE_FILES_DIR}/startup_stm32g030c8tx.s
	$(GCC_DIR)/arm-none-eabi-gcc -mcpu=cortex-m0plus -c -x assembler-with-cpp -MMD -MP -MF"${BUILD_DIR}/startup_stm32g030c8tx.d" -MT"${BUILD_DIR}/startup_stm32g030c8tx.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/startup_stm32g030c8tx.o" "${COMMON_SOURCE_FILES_DIR}/startup_stm32g030c8tx.s"

${BUILD_DIR}/global_variables.o: ${DEVICE_SOURCE_FILES_DIR}/global_variables.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/global_variables.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/global_variables.d" -MT"${BUILD_DIR}/global_variables.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/global_variables.o"

${BUILD_DIR}/ADC.o: ${DEVICE_SOURCE_FILES_DIR}/ADC.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/ADC.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/ADC.d" -MT"${BUILD_DIR}/ADC.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/ADC.o"

${BUILD_DIR}/PWM.o: ${DEVICE_SOURCE_FILES_DIR}/PWM.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/PWM.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/PWM.d" -MT"${BUILD_DIR}/PWM.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/PWM.o"

${BUILD_DIR}/RS485.o: ${COMMON_SOURCE_FILES_DIR}/RS485.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/RS485.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/RS485.d" -MT"${BUILD_DIR}/RS485.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/RS485.o"

${BUILD_DIR}/clock_calibration.o: ${COMMON_SOURCE_FILES_DIR}/clock_calibration.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/clock_calibration.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/clock_calibration.d" -MT"${BUILD_DIR}/clock_calibration.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/clock_calibration.o"

${BUILD_DIR}/debug_uart.o: ${COMMON_SOURCE_FILES_DIR}/debug_uart.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/debug_uart.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/debug_uart.d" -MT"${BUILD_DIR}/debug_uart.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/debug_uart.o"

${BUILD_DIR}/error_handling.o: ${COMMON_SOURCE_FILES_DIR}/error_handling.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/error_handling.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/error_handling.d" -MT"${BUILD_DIR}/error_handling.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/error_handling.o"

${BUILD_DIR}/leds.o: ${COMMON_SOURCE_FILES_DIR}/leds.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/leds.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/leds.d" -MT"${BUILD_DIR}/leds.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/leds.o"

${BUILD_DIR}/main.o: ${DEVICE_SOURCE_FILES_DIR}/main.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/main.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/main.d" -MT"${BUILD_DIR}/main.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/main.o"

${BUILD_DIR}/microsecond_clock.o: ${COMMON_SOURCE_FILES_DIR}/microsecond_clock.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/microsecond_clock.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/microsecond_clock.d" -MT"${BUILD_DIR}/microsecond_clock.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/microsecond_clock.o"

${BUILD_DIR}/mosfets.o: ${DEVICE_SOURCE_FILES_DIR}/mosfets.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/mosfets.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/mosfets.d" -MT"${BUILD_DIR}/mosfets.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/mosfets.o"

${BUILD_DIR}/coin_cell_control_and_calculations.o: ${DEVICE_SOURCE_FILES_DIR}/coin_cell_control_and_calculations.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/coin_cell_control_and_calculations.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/coin_cell_control_and_calculations.d" -MT"${BUILD_DIR}/coin_cell_control_and_calculations.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/coin_cell_control_and_calculations.o"

${BUILD_DIR}/system_stm32g0xx.o: ${COMMON_SOURCE_FILES_DIR}/system_stm32g0xx.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/system_stm32g0xx.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/system_stm32g0xx.d" -MT"${BUILD_DIR}/system_stm32g0xx.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/system_stm32g0xx.o"

${BUILD_DIR}/unique_id.o: ${COMMON_SOURCE_FILES_DIR}/unique_id.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/unique_id.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/unique_id.d" -MT"${BUILD_DIR}/unique_id.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/unique_id.o"

${BUILD_DIR}/settings.o: ${COMMON_SOURCE_FILES_DIR}/settings.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/settings.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/settings.d" -MT"${BUILD_DIR}/settings.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/settings.o"

${BUILD_DIR}/error_text.o: ${COMMON_SOURCE_FILES_DIR}/error_text.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/error_text.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/error_text.d" -MT"${BUILD_DIR}/error_text.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/error_text.o"

${BUILD_DIR}/device_status.o: ${COMMON_SOURCE_FILES_DIR}/device_status.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/device_status.c" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"${BUILD_DIR}/device_status.d" -MT"${BUILD_DIR}/device_status.o" --specs=nano.specs -mfloat-abi=soft -mthumb -o "${BUILD_DIR}/device_status.o"

${BUILD_DIR}/firmware_STM32G030.elf: $(OBJECTS) ${COMMON_SOURCE_FILES_DIR}/memory_layout.ld
	$(GCC_DIR)/arm-none-eabi-gcc -o "${BUILD_DIR}/firmware_STM32G030.elf" $(OBJECTS) -mcpu=cortex-m0plus -T"${COMMON_SOURCE_FILES_DIR}/memory_layout.ld" --specs=nosys.specs -Wl,-Map="${BUILD_DIR}/firmware_STM32G030.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	$(GCC_DIR)/arm-none-eabi-size   ${BUILD_DIR}/firmware_STM32G030.elf
	$(GCC_DIR)/arm-none-eabi-objdump -h -S  ${BUILD_DIR}/firmware_STM32G030.elf  > "${BUILD_DIR}/firmware_STM32G030.list"

${BUILD_DIR}/firmware_STM32G030.bin: ${BUILD_DIR}/firmware_STM32G030.elf
	$(GCC_DIR)/arm-none-eabi-objcopy  -O binary  ${BUILD_DIR}/firmware_STM32G030.elf  "${BUILD_DIR}/firmware_STM32G030.bin"

${BUILD_DIR}/firmware_STM32G030.firmware: ${BUILD_DIR}/firmware_STM32G030.bin
	../bin2firmware ${BUILD_DIR}/firmware_STM32G030.bin ${BUILD_DIR}/firmware_STM32G030.firmware B1 1


clean:
	-rm -f ${BUILD_DIR}/*

program: ${BUILD_DIR}/firmware_STM32G030.firmware
	/usr/bin/python3 ../python_programs/upgrade_firmware.py ${BUILD_DIR}/firmware_STM32G030.firmware

