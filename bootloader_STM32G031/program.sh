#!/usr/bin/env bash

# Check for the number of command line arguments
if [[ $# -eq 0 ]]; then
  echo "Please provide a product name."
  echo "Available products: M1, M2, M3, C1"
  exit 1
fi

# Get the product name from the command line argument
product_name=$1

# Validate product name and set software compatibility code and hardware_version
case "$product_name" in
  M1)
    software_compatibility_code=1
    hardware_version="0.9.0"
    ;;
  M2)
    software_compatibility_code=1
    hardware_version="0.8.0"
    ;;
  M3)
    software_compatibility_code=0
    hardware_version="0.0.1"
    ;;
  C1)
    software_compatibility_code=1
    hardware_version="0.8.0"
    ;;
  *)
    echo "Invalid product name. Please choose a valid product."
    echo "Available products: M1, M2, M3, C1"
    exit 1
    ;;
esac

# Detect which platform (OS) is being used
platform='unknown'
unamestr=$(uname)
case "$unamestr" in
  Linux*) platform='linux' ;;
  Darwin*) platform='mac' ;;
  CYGWIN*|MINGW*) platform='windows' ;;
esac

# Call generate_product_info with the appropriate arguments
if [ "$platform" = linux ]; then
  ../toolchain_essentials_linux/generate_product_info build/bootloader.bin build/bootloader_with_product_info.bin "$product_name" "$software_compatibility_code" "$hardware_version"
elif [ "$platform" = mac ]; then
  ../toolchain_essentials_mac/generate_product_info build/bootloader.bin build/bootloader_with_product_info.bin "$product_name" "$software_compatibility_code" "$hardware_version"
elif [ "$platform" = windows ]; then
  ../toolchain_essentials_windows/generate_product_info build/bootloader.bin build/bootloader_with_product_info.bin "$product_name" "$software_compatibility_code" "$hardware_version"
else
  echo "Platform not supported."
  exit 2
fi

# Flash the device
../programmer_essentials/STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst -e all
../programmer_essentials/STM32_Programmer_CLI -c port=SWD -w build/bootloader_with_product_info.bin 0x08000000 
../programmer_essentials/STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst
