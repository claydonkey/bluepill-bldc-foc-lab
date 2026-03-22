################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c 

OBJS += \
./Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.o 

C_DEPS += \
./Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/%.o Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/%.su Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/%.cyclo: ../Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/%.c Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32F1xx-2f-Source-2f-Templates

clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32F1xx-2f-Source-2f-Templates:
	-$(RM) ./Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.cyclo ./Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.d ./Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.o ./Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.su

.PHONY: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32F1xx-2f-Source-2f-Templates

