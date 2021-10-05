./generate_product_info build/servomotor_bootloader.bin build/bootloader_with_product_info.bin M1 0.8.0

../../programmer_essentials/STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst -e all
../../programmer_essentials/STM32_Programmer_CLI -c port=SWD -w build/bootloader_with_product_info.bin 0x08000000 
../../programmer_essentials//STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst



#/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_2.0.0.202105311346/tools/bin/STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst -e all
##/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_2.0.0.202105311346/tools/bin/STM32_Programmer_CLI -c port=SWD -w build/servomotor_bootloader.elf 
#/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_2.0.0.202105311346/tools/bin/STM32_Programmer_CLI -c port=SWD -w build/bootloader_with_product_info.bin 0x08000000 
#/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_2.0.0.202105311346/tools/bin/STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst
