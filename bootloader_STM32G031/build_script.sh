#!/usr/bin/env bash

set -x

COMMON_SOURCE_FILES_DIR=../common_source_files
DEVICE_SOURCE_FILES_DIR=Src
DRIVERS_DIR=../Drivers
BUILD_DIR=build
GCC_DIR=../toolchain_essentials_mac/bin/

C_FLAGS="-DPRODUCT_NAME_M3 -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G031xx -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -MMD -MP -mfloat-abi=soft -mthumb"

LINKER_FLAGS="-mcpu=cortex-m0plus -TSTM32G031G8UX_FLASH.ld --specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group"

OBJECT_FILES="${DEVICE_SOURCE_FILES_DIR}/startup_stm32g031g8ux.s \
    ${DEVICE_SOURCE_FILES_DIR}/global_variables.c \
    ${COMMON_SOURCE_FILES_DIR}/RS485.c \
    ${COMMON_SOURCE_FILES_DIR}/debug_uart.c \
    ${COMMON_SOURCE_FILES_DIR}/error_handling.c \
    ${COMMON_SOURCE_FILES_DIR}/error_text.c \
    ${DEVICE_SOURCE_FILES_DIR}/mosfets.c \
    ${DEVICE_SOURCE_FILES_DIR}/heater.c \
    ${COMMON_SOURCE_FILES_DIR}/leds.c \
    ${DEVICE_SOURCE_FILES_DIR}/main.c \
    ${DEVICE_SOURCE_FILES_DIR}/syscalls.c \
    ${DEVICE_SOURCE_FILES_DIR}/sysmem.c \
    ${DEVICE_SOURCE_FILES_DIR}/system_stm32g0xx.c \
    ${COMMON_SOURCE_FILES_DIR}/unique_id.c \
    ${COMMON_SOURCE_FILES_DIR}/settings.c \
    ${COMMON_SOURCE_FILES_DIR}/product_info.c \
    ${COMMON_SOURCE_FILES_DIR}/device_status.c"


mkdir -p ${BUILD_DIR}

$GCC_DIR/arm-none-eabi-gcc -o ${BUILD_DIR}/"bootloader.elf" $OBJECT_FILES $C_FLAGS $LINKER_FLAGS
$GCC_DIR/arm-none-eabi-size   ${BUILD_DIR}/bootloader.elf 
$GCC_DIR/arm-none-eabi-objdump -h -S  ${BUILD_DIR}/bootloader.elf  > "${BUILD_DIR}/bootloader.list"
$GCC_DIR/arm-none-eabi-objcopy  -O binary  ${BUILD_DIR}/bootloader.elf  "${BUILD_DIR}/bootloader.bin"





