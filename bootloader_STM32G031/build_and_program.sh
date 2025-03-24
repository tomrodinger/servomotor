#!/usr/bin/env bash

# Function to print large messages
print_large_message() {
    echo ""
    echo "****************************"
    echo "*                          *"
    echo "*      $1"
    echo "*                          *"
    echo "****************************"
    echo ""
}

# Function to build bootloader
build_bootloader() {
    local product_name=$1
    local software_compatibility_code=$2
    
    # Enable command echoing for build process
    set -x
    
    COMMON_SOURCE_FILES_DIR=../common_source_files
    DEVICE_SOURCE_FILES_DIR=Src
    DRIVERS_DIR=../Drivers
    BUILD_DIR=build
    GCC_DIR=../toolchain_essentials_mac/bin/

    # Note: PRODUCT_NAME_${product_name} will expand to e.g. PRODUCT_NAME_M1
    C_FLAGS="-DPRODUCT_NAME_${product_name} -DSOFTWARE_COMPATIBILITY_CODE=${software_compatibility_code} -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G031xx -I${DEVICE_SOURCE_FILES_DIR} -I${COMMON_SOURCE_FILES_DIR} -I${DRIVERS_DIR}/STM32G0xx_HAL_Driver/Inc -I${DRIVERS_DIR}/CMSIS/Device/ST/STM32G0xx/Include -I${DRIVERS_DIR}/CMSIS/Include -O3 -ffunction-sections -fdata-sections -Wall -MMD -MP -mfloat-abi=soft -mthumb"

    LINKER_FLAGS="-mcpu=cortex-m0plus -TSTM32G031G8UX_FLASH.ld --specs=nosys.specs -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group"

    OBJECT_FILES="${DEVICE_SOURCE_FILES_DIR}/startup_stm32g031g8ux.s \
        ${DEVICE_SOURCE_FILES_DIR}/global_variables.c \
        ${COMMON_SOURCE_FILES_DIR}/RS485.c \
        ${COMMON_SOURCE_FILES_DIR}/crc32.c \
        ${COMMON_SOURCE_FILES_DIR}/debug_uart.c \
        ${COMMON_SOURCE_FILES_DIR}/error_handling.c \
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
        ${COMMON_SOURCE_FILES_DIR}/device_status.c \
        ${COMMON_SOURCE_FILES_DIR}/gpio_M1.c \
        ${COMMON_SOURCE_FILES_DIR}/gpio_M2.c \
        ${COMMON_SOURCE_FILES_DIR}/gpio_M3.c \
        ${COMMON_SOURCE_FILES_DIR}/gpio_M4.c"

    # Create build directory if it doesn't exist
    mkdir -p ${BUILD_DIR}

    # Build the project
    if $GCC_DIR/arm-none-eabi-gcc -o ${BUILD_DIR}/"bootloader.elf" $OBJECT_FILES $C_FLAGS $LINKER_FLAGS &&
       $GCC_DIR/arm-none-eabi-size ${BUILD_DIR}/bootloader.elf &&
       $GCC_DIR/arm-none-eabi-objdump -h -S ${BUILD_DIR}/bootloader.elf > "${BUILD_DIR}/bootloader.list" &&
       $GCC_DIR/arm-none-eabi-objcopy -O binary ${BUILD_DIR}/bootloader.elf "${BUILD_DIR}/bootloader.bin"; then
        
        # Disable command echoing before printing message
        set +x
        print_large_message "Build Success"
        return 0
    else
        # Disable command echoing before printing message
        set +x
        print_large_message "Build Error"
        return 1
    fi
}

# Function to program device
program_device() {
    local bin_file=$1
    local product_name=$2
    local software_compatibility_code=$3
    local hardware_version=$4

    # Detect which platform (OS) is being used
    platform='unknown'
    unamestr=$(uname)
    case "$unamestr" in
        Linux*) platform='linux' ;;
        Darwin*) platform='mac' ;;
        CYGWIN*|MINGW*) platform='windows' ;;
    esac

    # Call generate_product_info with the appropriate arguments to get a bin file with product specific info
    if [ "$platform" = linux ]; then
    ../toolchain_essentials_linux/generate_product_info "$bin_file" build/bootloader_with_product_info.bin "$product_name" "$software_compatibility_code" "$hardware_version"
    elif [ "$platform" = mac ]; then
    ../toolchain_essentials_mac/generate_product_info "$bin_file" build/bootloader_with_product_info.bin "$product_name" "$software_compatibility_code" "$hardware_version"
    elif [ "$platform" = windows ]; then
    ../toolchain_essentials_windows/generate_product_info "$bin_file" build/bootloader_with_product_info.bin "$product_name" "$software_compatibility_code" "$hardware_version"
    else
    echo "Platform not supported."
    exit 2
    fi

    # Flash the device
    if ../programmer_essentials/STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst -e all -w build/bootloader_with_product_info.bin 0x08000000 -g; then
        print_large_message "Programming Success"
        return 0
    else
        print_large_message "Programming Failed"
        return 1
    fi
}

# Function to build new bootloader with user input
build_new_bootloader() {
    echo "Building new bootloader..."
    echo "Please enter the following information:"
    
    # Get product name
    while true; do
        echo "Product name (e.g., M1, M2, M3, M4, C1):"
        read product_name
        if [[ "$product_name" =~ ^[A-Za-z][0-9]$ ]]; then
            break
        else
            echo "Invalid product name format. Please use a letter followed by a number (e.g., M1)"
        fi
    done

    # Get hardware version
    while true; do
        echo "Hardware version (e.g., 0.9.0):"
        read hw_version
        if [[ "$hw_version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
            break
        else
            echo "Invalid version format. Please use X.Y.Z format (e.g., 0.9.0)"
        fi
    done

    # Get software compatibility code
    while true; do
        echo "Software compatibility code (0-255):"
        read sw_compat
        if [[ "$sw_compat" =~ ^[0-9]+$ ]] && [ "$sw_compat" -ge 0 ] && [ "$sw_compat" -le 255 ]; then
            break
        else
            echo "Invalid compatibility code. Please enter a number between 0 and 255"
        fi
    done

    # Get current time as epoch timestamp
    local timestamp=$(date +%s)

    # Build the bootloader
    if build_bootloader "$product_name" "$sw_compat"; then
        # Create release filename with timestamp
        release_file="bootloader_releases/bootloader_${product_name}_hw${hw_version}_scc${sw_compat}_${timestamp}.bin"
        
        # Copy the built bootloader to releases directory
        cp build/bootloader.bin "$release_file"
        echo "Bootloader copied to: $release_file"
        return 0
    else
        echo "Failed to build bootloader"
        return 1
    fi
}

# Main program loop
while true; do
    echo
    echo "Available bootloaders:"
    
    # Create bootloader_releases directory if it doesn't exist
    mkdir -p bootloader_releases

    # Get unique list of bootloader files sorted by date
    readarray -t bootloader_files < <(find bootloader_releases -name "bootloader_*.bin" -type f -exec stat -f "%m %N" {} \; | sort -n | cut -d' ' -f2-)
    
    # Print existing bootloaders with numbers
    for i in "${!bootloader_files[@]}"; do
        file="${bootloader_files[i]}"
        # Get file creation date/time in human readable format
        creation_datetime=$(stat -f "%Sm" -t "%Y-%m-%d %H:%M:%S" "$file")
        
        # Extract info from filename
        filename=$(basename "$file")
        if [[ $filename =~ bootloader_(.+)_hw(.+)_scc([0-9]+)(_[0-9]+)?.bin ]]; then
            product="${BASH_REMATCH[1]}"
            hw_ver="${BASH_REMATCH[2]}"
            sw_compat="${BASH_REMATCH[3]}"
            printf "%2d) Product name: %s, Hardware Version: %s, Software Compatibility Code: %s\n    Build Date/Time: %s\n" \
                   $((i+1)) "$product" "$hw_ver" "$sw_compat" "$creation_datetime"
        fi
    done
    printf "%2d) Build new bootloader\n" "$((${#bootloader_files[@]}+1))"
    
    # Get user selection
    echo -e "\nPlease select an option (1-$((${#bootloader_files[@]}+1))) or 'q' to quit:"
    read selection

    # Handle quit option
    if [ "$selection" = "q" ]; then
        echo "Exiting..."
        exit 0
    fi

    # Validate input is a number
    if ! [[ "$selection" =~ ^[0-9]+$ ]]; then
        echo "Please enter a valid number"
        continue
    fi

    # Handle build new option
    if [ "$selection" -eq "$((${#bootloader_files[@]}+1))" ]; then
        build_new_bootloader
        continue
    fi

    # Adjust for zero-based array indexing
    index=$((selection-1))

    # Validate selection is in range
    if [ "$index" -lt 0 ] || [ "$index" -ge "${#bootloader_files[@]}" ]; then
        echo "Please select a number between 1 and $((${#bootloader_files[@]}+1))"
        continue
    fi

    # Get selected file and its creation date/time
    selected_file="${bootloader_files[index]}"
    creation_datetime=$(stat -f "%Sm" -t "%Y-%m-%d %H:%M:%S" "$selected_file")
    
    # Show selected bootloader details
    filename=$(basename "$selected_file")
    if [[ $filename =~ bootloader_(.+)_hw(.+)_scc([0-9]+)(_[0-9]+)?.bin ]]; then
        echo -e "\nSelected bootloader:"
        echo "  Product name: ${BASH_REMATCH[1]}"
        echo "  Hardware Version: ${BASH_REMATCH[2]}"
        echo "  Software Compatibility Code: ${BASH_REMATCH[3]}"
        echo "  Build Date/Time: $creation_datetime"
    fi

    while true; do
        echo -e "\nPress ENTER to program the device with the selected bootloader"
        echo "Type 'b' and press ENTER to go back to bootloader selection"
        echo "Type 'q' and press ENTER to quit"
        read choice

        if [ "$choice" = "q" ]; then
            echo "Exiting..."
            exit 0
        elif [ "$choice" = "b" ]; then
            break
        elif [ "$choice" = "" ]; then
            program_device "$selected_file" "${BASH_REMATCH[1]}" "${BASH_REMATCH[3]}" "${BASH_REMATCH[2]}"
        else
            echo "Invalid choice. Please try again."
        fi
    done
done
