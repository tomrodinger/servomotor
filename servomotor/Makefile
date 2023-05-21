COMMON_SOURCE_FILES_DIR = ../common_source_files
PRODUCT_OPTIONS := M1 M2

# Find out the product name
ifeq ($(PRODUCT_NAME),)
	PRODUCT_OPTIONS := M1 M2
else
	PRODUCT_OPTIONS := $(PRODUCT_NAME)
endif

# Find out what version of the firmware we are building
VERSION=$(shell while read product version; do if [ "$$product" = "$(PRODUCT_NAME)" ]; then echo $$version; break; fi; done < VERSIONS)

# Determine what platform we are building on
HOST_OS := $(shell uname -s)
ifeq ($(HOST_OS),Linux)
    TOOLCHAIN_DIR ?= ../toolchain_essentials_linux
else ifeq ($(HOST_OS),Darwin)  # macOS
    TOOLCHAIN_DIR ?= ../toolchain_essentials_mac
else ifeq ($(OS),Windows_NT)  # Windows
    TOOLCHAIN_DIR ?= ../toolchain_essentials_windows
else
    $(error Unsupported platform: $(HOST_OS))
endif

GCC_DIR ?= ${TOOLCHAIN_DIR}/bin

# Compiler and linker settings
SOFTWARE_COMPATIBILITY_CODE = 1
DEVICE_SOURCE_FILES_DIR = Src
DRIVERS_DIR = ../Drivers
BUILD_DIR = build
HEADERS = $(wildcard ${DEVICE_SOURCE_FILES_DIR}/*.h)
COMMON_HEADERS = $(wildcard ${COMMON_SOURCE_FILES_DIR}/*.h)
DEVICE_OBJECTS = $(addprefix ${BUILD_DIR}/,$(notdir $(patsubst %.c, %.o,$(wildcard ${DEVICE_SOURCE_FILES_DIR}/*.c))))
COMMON_OBJECTS = $(addprefix ${BUILD_DIR}/,$(notdir $(patsubst %.c, %.o,$(wildcard ${COMMON_SOURCE_FILES_DIR}/*.c))))
OBJECTS = $(DEVICE_OBJECTS) $(COMMON_OBJECTS) ${BUILD_DIR}/startup_stm32g030c8tx.o
BUILD_FLAGS = -D PRODUCT_NAME_$(PRODUCT_NAME) -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx
MORE_BUILD_FLAGS = -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP --specs=nano.specs -mfloat-abi=soft -mthumb
INCLUDE_DIRS = \
	-I${DEVICE_SOURCE_FILES_DIR} \
	-I${COMMON_SOURCE_FILES_DIR} \
	-I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc \
	-I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc/Legacy \
	-I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include \
	-I${DRIVERS_DIR}/CMSIS/Include

.PHONY: default
default:
ifeq ($(PRODUCT_NAME),)
	@echo "Please provide a PRODUCT_NAME. Acceptable options are: $(PRODUCT_OPTIONS)"
	@echo "Example: make PRODUCT_NAME=M1"
else
	${MAKE} ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware
endif

${BUILD_DIR}:
	mkdir -p ${BUILD_DIR}

${BUILD_DIR}/startup_stm32g030c8tx.o: ${COMMON_SOURCE_FILES_DIR}/startup_stm32g030c8tx.s | ${BUILD_DIR}
	$(GCC_DIR)/arm-none-eabi-gcc -mcpu=cortex-m0plus -c -x assembler-with-cpp $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/startup_stm32g030c8tx.d" -MT"${BUILD_DIR}/startup_stm32g030c8tx.o" -o "${BUILD_DIR}/startup_stm32g030c8tx.o" "${COMMON_SOURCE_FILES_DIR}/startup_stm32g030c8tx.s"

${BUILD_DIR}/%.o: ${DEVICE_SOURCE_FILES_DIR}/%.c ${HEADERS} ${COMMON_HEADERS} | ${BUILD_DIR}
	$(GCC_DIR)/arm-none-eabi-gcc $< $(BUILD_FLAGS) -c $(INCLUDE_DIRS) $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/$*.d" -MT"${BUILD_DIR}/$*.o" -o "${BUILD_DIR}/$*.o"

${BUILD_DIR}/%.o: ${COMMON_SOURCE_FILES_DIR}/%.c ${HEADERS} ${COMMON_HEADERS} | ${BUILD_DIR}
	$(GCC_DIR)/arm-none-eabi-gcc $< $(BUILD_FLAGS) -c $(INCLUDE_DIRS) $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/$*.d" -MT"${BUILD_DIR}/$*.o" -o "${BUILD_DIR}/$*.o"

.PHONY: ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware
${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf: $(OBJECTS) ${COMMON_SOURCE_FILES_DIR}/memory_layout.ld
	$(GCC_DIR)/arm-none-eabi-gcc -o "${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf" $(OBJECTS) -mcpu=cortex-m0plus -T"${COMMON_SOURCE_FILES_DIR}/memory_layout.ld" --specs=nosys.specs -Wl,-Map="${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	$(GCC_DIR)/arm-none-eabi-size ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf
	$(GCC_DIR)/arm-none-eabi-objdump -h -S ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf > "${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.list"

${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.bin: ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf
	$(GCC_DIR)/arm-none-eabi-objcopy -O binary ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.elf "${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.bin"

${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware: ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.bin
	${TOOLCHAIN_DIR}/bin2firmware ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.bin ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware $(PRODUCT_NAME) $(SOFTWARE_COMPATIBILITY_CODE)
	cp ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware firmware_releases/

.PHONY: clean
clean:
	$(RM) ${BUILD_DIR}/*

.PHONY: program
program: ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}.firmware
	../python_programs/upgrade_firmware.py ${BUILD_DIR}/motor_firmware_${PRODUCT_NAME}_${VERSION}_${VERSION}.firmware
