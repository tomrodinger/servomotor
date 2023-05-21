ifeq ($(PRODUCT_NAME),)
	PRODUCT_OPTIONS := M1 M2
else
	PRODUCT_OPTIONS := $(PRODUCT_NAME)
endif

VERSION=$(shell awk -v product="$(PRODUCT_NAME)" '$$1 == product { print $$2 }' VERSIONS)

SOFTWARE_COMPATIBILITY_CODE = 1
COMMON_SOURCE_FILES_DIR = ../common_source_files
DEVICE_SOURCE_FILES_DIR = Src
DRIVERS_DIR = ../Drivers
BUILD_DIR = build
HEADERS = ${DEVICE_SOURCE_FILES_DIR}/global_variables.h   ${DEVICE_SOURCE_FILES_DIR}/ADC.h   ${DEVICE_SOURCE_FILES_DIR}/mosfets.h   ${DEVICE_SOURCE_FILES_DIR}/step_direction_input.h   ${DEVICE_SOURCE_FILES_DIR}/CommutationTable.h   ${DEVICE_SOURCE_FILES_DIR}/hall_sensor_calculations.h   ${DEVICE_SOURCE_FILES_DIR}/motor_control.h   ${DEVICE_SOURCE_FILES_DIR}/LookupTable_M1.h   ${DEVICE_SOURCE_FILES_DIR}/LookupTable_M2.h   ${DEVICE_SOURCE_FILES_DIR}/commands.h   ${DEVICE_SOURCE_FILES_DIR}/PWM.h   ${DEVICE_SOURCE_FILES_DIR}/device_specific_settings.h   ${DEVICE_SOURCE_FILES_DIR}/overvoltage.h
COMMON_HEADERS = ${COMMON_SOURCE_FILES_DIR}/RS485.h   ${COMMON_SOURCE_FILES_DIR}/error_handling.h   ${COMMON_SOURCE_FILES_DIR}/clock_calibration.h   ${COMMON_SOURCE_FILES_DIR}/stm32g0xx_hal_conf.h   ${COMMON_SOURCE_FILES_DIR}/leds.h   ${COMMON_SOURCE_FILES_DIR}/product_info.h   ${COMMON_SOURCE_FILES_DIR}/stm32g0xx_it.h   ${COMMON_SOURCE_FILES_DIR}/debug_uart.h   ${COMMON_SOURCE_FILES_DIR}/microsecond_clock.h   ${COMMON_SOURCE_FILES_DIR}/settings.h   ${COMMON_SOURCE_FILES_DIR}/unique_id.h   ${COMMON_SOURCE_FILES_DIR}/error_text.h   ${COMMON_SOURCE_FILES_DIR}/device_status.h
OBJECTS = ${BUILD_DIR}/startup_stm32g030c8tx.o   ${BUILD_DIR}/global_variables.o   ${BUILD_DIR}/ADC.o   ${BUILD_DIR}/PWM.o   ${BUILD_DIR}/RS485.o   ${BUILD_DIR}/clock_calibration.o   ${BUILD_DIR}/debug_uart.o   ${BUILD_DIR}/error_handling.o   ${BUILD_DIR}/hall_sensor_calculations.o   ${BUILD_DIR}/leds.o   ${BUILD_DIR}/main.o   ${BUILD_DIR}/microsecond_clock.o   ${BUILD_DIR}/mosfets.o   ${BUILD_DIR}/motor_control.o   ${BUILD_DIR}/step_direction_input.o   ${BUILD_DIR}/system_stm32g0xx.o   ${BUILD_DIR}/unique_id.o   ${BUILD_DIR}/settings.o   ${BUILD_DIR}/error_text.o   ${BUILD_DIR}/overvoltage.o   ${BUILD_DIR}/device_status.o
BUILD_FLAGS = -D PRODUCT_NAME_$(PRODUCT_NAME) -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx
MORE_BUILD_FLAGS = -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP --specs=nano.specs -mfloat-abi=soft -mthumb
GCC_DIR=../toolchain_essentials/bin

default:
ifeq ($(PRODUCT_NAME),)
	@echo "Please provide a PRODUCT_NAME. Acceptable options are: $(PRODUCT_OPTIONS)"
	@echo "Example: make PRODUCT_NAME=M1"
else
	${MAKE} ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware
endif

${BUILD_DIR}/startup_stm32g030c8tx.o: ${COMMON_SOURCE_FILES_DIR}/startup_stm32g030c8tx.s
	$(GCC_DIR)/arm-none-eabi-gcc  -mcpu=cortex-m0plus -c -x assembler-with-cpp $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/startup_stm32g030c8tx.d" -MT"${BUILD_DIR}/startup_stm32g030c8tx.o" -o "${BUILD_DIR}/startup_stm32g030c8tx.o" "${COMMON_SOURCE_FILES_DIR}/startup_stm32g030c8tx.s"

${BUILD_DIR}/global_variables.o: ${DEVICE_SOURCE_FILES_DIR}/global_variables.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/global_variables.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/global_variables.d" -MT"${BUILD_DIR}/global_variables.o" -o "${BUILD_DIR}/global_variables.o"

${BUILD_DIR}/ADC.o: ${DEVICE_SOURCE_FILES_DIR}/ADC.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/ADC.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/ADC.d" -MT"${BUILD_DIR}/ADC.o" -o "${BUILD_DIR}/ADC.o"

${BUILD_DIR}/PWM.o: ${DEVICE_SOURCE_FILES_DIR}/PWM.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/PWM.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/PWM.d" -MT"${BUILD_DIR}/PWM.o" -o "${BUILD_DIR}/PWM.o"

${BUILD_DIR}/RS485.o: ${COMMON_SOURCE_FILES_DIR}/RS485.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/RS485.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/RS485.d" -MT"${BUILD_DIR}/RS485.o" -o "${BUILD_DIR}/RS485.o"

${BUILD_DIR}/clock_calibration.o: ${COMMON_SOURCE_FILES_DIR}/clock_calibration.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/clock_calibration.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/clock_calibration.d" -MT"${BUILD_DIR}/clock_calibration.o" -o "${BUILD_DIR}/clock_calibration.o"

${BUILD_DIR}/debug_uart.o: ${COMMON_SOURCE_FILES_DIR}/debug_uart.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/debug_uart.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/debug_uart.d" -MT"${BUILD_DIR}/debug_uart.o" -o "${BUILD_DIR}/debug_uart.o"

${BUILD_DIR}/error_handling.o: ${COMMON_SOURCE_FILES_DIR}/error_handling.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/error_handling.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/error_handling.d" -MT"${BUILD_DIR}/error_handling.o" -o "${BUILD_DIR}/error_handling.o"

${BUILD_DIR}/hall_sensor_calculations.o: ${DEVICE_SOURCE_FILES_DIR}/hall_sensor_calculations.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/hall_sensor_calculations.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/hall_sensor_calculations.d" -MT"${BUILD_DIR}/hall_sensor_calculations.o" -o "${BUILD_DIR}/hall_sensor_calculations.o"

${BUILD_DIR}/leds.o: ${COMMON_SOURCE_FILES_DIR}/leds.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/leds.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/leds.d" -MT"${BUILD_DIR}/leds.o" -o "${BUILD_DIR}/leds.o"

${BUILD_DIR}/main.o: ${DEVICE_SOURCE_FILES_DIR}/main.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/main.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/main.d" -MT"${BUILD_DIR}/main.o" -o "${BUILD_DIR}/main.o"

${BUILD_DIR}/microsecond_clock.o: ${COMMON_SOURCE_FILES_DIR}/microsecond_clock.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/microsecond_clock.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/microsecond_clock.d" -MT"${BUILD_DIR}/microsecond_clock.o" -o "${BUILD_DIR}/microsecond_clock.o"

${BUILD_DIR}/mosfets.o: ${DEVICE_SOURCE_FILES_DIR}/mosfets.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/mosfets.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/mosfets.d" -MT"${BUILD_DIR}/mosfets.o" -o "${BUILD_DIR}/mosfets.o"

${BUILD_DIR}/motor_control.o: ${DEVICE_SOURCE_FILES_DIR}/motor_control.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/motor_control.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/motor_control.d" -MT"${BUILD_DIR}/motor_control.o" -o "${BUILD_DIR}/motor_control.o"

${BUILD_DIR}/step_direction_input.o: ${DEVICE_SOURCE_FILES_DIR}/step_direction_input.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/step_direction_input.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/step_direction_input.d" -MT"${BUILD_DIR}/step_direction_input.o" -o "${BUILD_DIR}/step_direction_input.o"

${BUILD_DIR}/system_stm32g0xx.o: ${COMMON_SOURCE_FILES_DIR}/system_stm32g0xx.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/system_stm32g0xx.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/system_stm32g0xx.d" -MT"${BUILD_DIR}/system_stm32g0xx.o" -o "${BUILD_DIR}/system_stm32g0xx.o"

${BUILD_DIR}/unique_id.o: ${COMMON_SOURCE_FILES_DIR}/unique_id.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/unique_id.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/unique_id.d" -MT"${BUILD_DIR}/unique_id.o" -o "${BUILD_DIR}/unique_id.o"

${BUILD_DIR}/settings.o: ${COMMON_SOURCE_FILES_DIR}/settings.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/settings.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/settings.d" -MT"${BUILD_DIR}/settings.o" -o "${BUILD_DIR}/settings.o"

${BUILD_DIR}/error_text.o: ${COMMON_SOURCE_FILES_DIR}/error_text.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/error_text.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/error_text.d" -MT"${BUILD_DIR}/error_text.o" -o "${BUILD_DIR}/error_text.o"

${BUILD_DIR}/overvoltage.o: ${DEVICE_SOURCE_FILES_DIR}/overvoltage.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${DEVICE_SOURCE_FILES_DIR}/overvoltage.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/overvoltage.d" -MT"${BUILD_DIR}/overvoltage.o" -o "${BUILD_DIR}/overvoltage.o"

${BUILD_DIR}/device_status.o: ${COMMON_SOURCE_FILES_DIR}/device_status.c $(HEADERS) $(COMMON_HEADERS)
	$(GCC_DIR)/arm-none-eabi-gcc "${COMMON_SOURCE_FILES_DIR}/device_status.c" $(BUILD_FLAGS) -c -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/device_status.d" -MT"${BUILD_DIR}/device_status.o" -o "${BUILD_DIR}/device_status.o"

${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf: $(OBJECTS) ${COMMON_SOURCE_FILES_DIR}/memory_layout.ld
	$(GCC_DIR)/arm-none-eabi-gcc -o "${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf" $(OBJECTS) -mcpu=cortex-m0plus -T"${COMMON_SOURCE_FILES_DIR}/memory_layout.ld" --specs=nosys.specs -Wl,-Map="${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	$(GCC_DIR)/arm-none-eabi-size   ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf
	$(GCC_DIR)/arm-none-eabi-objdump -h -S  ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf  > "${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.list"

${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.bin: ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf
	$(GCC_DIR)/arm-none-eabi-objcopy  -O binary  ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf  "${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.bin"

${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware: ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.bin
	../bin2firmware ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.bin ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware $(PRODUCT_NAME) $(SOFTWARE_COMPATIBILITY_CODE)
	cp ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware ../firmware_releases/

clean:
	-rm -f ${BUILD_DIR}/*

program: ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware
	../python_programs/upgrade_firmware.py ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}_${VERSION}.firmware

