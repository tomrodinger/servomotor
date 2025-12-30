COMMON_SOURCE_FILES_DIR = ../common_source_files
PRODUCT_OPTIONS := M1 M2 M17 M23

# Define the directory of source files before using it
DEVICE_SOURCE_FILES_DIR = Src

# Find out the product name
ifeq ($(PRODUCT_NAME),)
	PRODUCT_OPTIONS := M1 M2 M17 M23
else
	PRODUCT_OPTIONS := $(PRODUCT_NAME)
endif

# Extract hardware version and software compatibility code from VERSIONS file
SOFTWARE_COMPATIBILITY_CODE := $(shell awk '$$1 == "$(PRODUCT_NAME)" {print $$3}' VERSIONS)
HARDWARE_VERSION := $(shell awk '$$1 == "$(PRODUCT_NAME)" {print $$2}' VERSIONS)

# Extract version information from main.c
MAJOR := $(shell awk '/^\#define[[:space:]]+MAJOR_FIRMWARE_VERSION[[:space:]]+[0-9]+/{print $$3}' $(DEVICE_SOURCE_FILES_DIR)/main.c)
MINOR := $(shell awk '/^\#define[[:space:]]+MINOR_FIRMWARE_VERSION[[:space:]]+[0-9]+/{print $$3}' $(DEVICE_SOURCE_FILES_DIR)/main.c)
BUGFIX := $(shell awk '/^\#define[[:space:]]+BUGFIX_FIRMWARE_VERSION[[:space:]]+[0-9]+/{print $$3}' $(DEVICE_SOURCE_FILES_DIR)/main.c)
DEVELOPMENT := $(shell awk '/^\#define[[:space:]]+DEVELOPMENT_FIRMWARE_VERSION[[:space:]]+[0-9]+/{print $$3}' $(DEVICE_SOURCE_FILES_DIR)/main.c)
FIRMWARE_VERSION := $(MAJOR).$(MINOR).$(BUGFIX).$(DEVELOPMENT)

# Define a version string that contains all the versioning information for using to name the output file
VERSION := $(PRODUCT_NAME)_fw$(FIRMWARE_VERSION)_scc$(SOFTWARE_COMPATIBILITY_CODE)_hw$(HARDWARE_VERSION)

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
DEVICE_SOURCE_FILES_DIR = Src
DRIVERS_DIR = ../Drivers
BUILD_DIR = build
HEADERS = $(wildcard ${DEVICE_SOURCE_FILES_DIR}/*.h)
COMMON_HEADERS = $(wildcard ${COMMON_SOURCE_FILES_DIR}/*.h)
DEVICE_OBJECTS = $(addprefix ${BUILD_DIR}/,$(notdir $(patsubst %.c, %.o,$(wildcard ${DEVICE_SOURCE_FILES_DIR}/*.c))))
COMMON_OBJECTS = $(addprefix ${BUILD_DIR}/,$(notdir $(patsubst %.c, %.o,$(wildcard ${COMMON_SOURCE_FILES_DIR}/*.c))))
OBJECTS = $(DEVICE_OBJECTS) $(COMMON_OBJECTS) ${BUILD_DIR}/startup_stm32g030c8tx.o

# Additional preprocessor defines can be provided on the command line.
#
# Examples:
#   make PRODUCT_NAME=M17 PROGRAMMING_TEST_JIG_MODE
#   make PRODUCT_NAME=M17 EXTRA_DEFINES="PROGRAMMING_TEST_JIG_MODE SOME_FEATURE=1"
EXTRA_DEFINES ?=

BUILD_FLAGS = -DPRODUCT_NAME_$(PRODUCT_NAME) -DSOFTWARE_COMPATIBILITY_CODE=$(SOFTWARE_COMPATIBILITY_CODE) $(addprefix -D,$(strip $(EXTRA_DEFINES))) -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G030xx
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
	@echo ""
	@echo "Usage:"
	@echo "  make PRODUCT_NAME=<one of: $(PRODUCT_OPTIONS)> [PROGRAMMING_TEST_JIG_MODE] [EXTRA_DEFINES=\"DEF1 DEF2=VALUE\"]"
	@echo ""
	@echo "Examples:"
	@echo "  make PRODUCT_NAME=M17"
	@echo "  make PRODUCT_NAME=M17 PROGRAMMING_TEST_JIG_MODE"
	@echo "  make PRODUCT_NAME=M17 EXTRA_DEFINES=\"PROGRAMMING_TEST_JIG_MODE SOME_FEATURE=1\""
else
	# Pass EXTRA_DEFINES through to the recursive make so they affect all compilation commands.
	${MAKE} ${BUILD_DIR}/servomotor_${VERSION}.firmware EXTRA_DEFINES="$(EXTRA_DEFINES)" && echo "* * * SUCCESS * * *" || echo "* * * BUILD FAILED * * *"
endif

# Convenience option-goal: allow specifying PROGRAMMING_TEST_JIG_MODE as a bare token
# on the command line (i.e. as a goal), while still building the normal default target.
.PHONY: PROGRAMMING_TEST_JIG_MODE
PROGRAMMING_TEST_JIG_MODE: EXTRA_DEFINES += PROGRAMMING_TEST_JIG_MODE
PROGRAMMING_TEST_JIG_MODE: default

${BUILD_DIR}:
	mkdir -p ${BUILD_DIR}

${BUILD_DIR}/startup_stm32g030c8tx.o: ${COMMON_SOURCE_FILES_DIR}/startup_stm32g030c8tx.s | ${BUILD_DIR}
	$(GCC_DIR)/arm-none-eabi-gcc -mcpu=cortex-m0plus -c -x assembler-with-cpp $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/startup_stm32g030c8tx.d" -MT"${BUILD_DIR}/startup_stm32g030c8tx.o" -o "${BUILD_DIR}/startup_stm32g030c8tx.o" "${COMMON_SOURCE_FILES_DIR}/startup_stm32g030c8tx.s"

${BUILD_DIR}/%.o: ${DEVICE_SOURCE_FILES_DIR}/%.c ${HEADERS} ${COMMON_HEADERS} | ${BUILD_DIR}
	$(GCC_DIR)/arm-none-eabi-gcc $< $(BUILD_FLAGS) -c $(INCLUDE_DIRS) $(MORE_BUILD_FLAGS) \
	-Wa,-adhlns=${BUILD_DIR}/$*.lst -g \
	-MF"${BUILD_DIR}/$*.d" -MT"${BUILD_DIR}/$*.o" -o "${BUILD_DIR}/$*.o"

${BUILD_DIR}/%.o: ${COMMON_SOURCE_FILES_DIR}/%.c ${HEADERS} ${COMMON_HEADERS} | ${BUILD_DIR}
	$(GCC_DIR)/arm-none-eabi-gcc $< $(BUILD_FLAGS) -c $(INCLUDE_DIRS) $(MORE_BUILD_FLAGS) -MF"${BUILD_DIR}/$*.d" -MT"${BUILD_DIR}/$*.o" -o "${BUILD_DIR}/$*.o"

.PHONY: ${BUILD_DIR}/servomotor_${VERSION}.firmware
${BUILD_DIR}/servomotor_${VERSION}.elf: $(OBJECTS) ${COMMON_SOURCE_FILES_DIR}/memory_layout.ld
	$(GCC_DIR)/arm-none-eabi-gcc -o "${BUILD_DIR}/servomotor_${VERSION}.elf" $(OBJECTS) -mcpu=cortex-m0plus -T"${COMMON_SOURCE_FILES_DIR}/memory_layout.ld" --specs=nosys.specs -Wl,-Map="${BUILD_DIR}/servomotor_${VERSION}.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	$(GCC_DIR)/arm-none-eabi-size ${BUILD_DIR}/servomotor_${VERSION}.elf
	$(GCC_DIR)/arm-none-eabi-objdump -h -S ${BUILD_DIR}/servomotor_${VERSION}.elf > "${BUILD_DIR}/servomotor_${VERSION}.list"

${BUILD_DIR}/servomotor_${VERSION}.bin: ${BUILD_DIR}/servomotor_${VERSION}.elf
	$(GCC_DIR)/arm-none-eabi-objcopy -O binary ${BUILD_DIR}/servomotor_${VERSION}.elf "${BUILD_DIR}/servomotor_${VERSION}.bin"

${BUILD_DIR}/servomotor_${VERSION}.firmware: ${BUILD_DIR}/servomotor_${VERSION}.bin
	${TOOLCHAIN_DIR}/bin2firmware ${BUILD_DIR}/servomotor_${VERSION}.bin ${BUILD_DIR}/servomotor_${VERSION}.firmware $(PRODUCT_NAME) $(SOFTWARE_COMPATIBILITY_CODE)
	cp ${BUILD_DIR}/servomotor_${VERSION}.firmware firmware_releases/

.PHONY: clean
clean:
	$(RM) ${BUILD_DIR}/*

.PHONY: program
program: ${BUILD_DIR}/servomotor_${VERSION}.firmware
	../python_programs/upgrade_firmware.py ${BUILD_DIR}/servomotor_${VERSION}.firmware
