################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f103cbtx.s 

OBJS += \
./Core/Startup/startup_stm32f103cbtx.o 

S_DEPS += \
./Core/Startup/startup_stm32f103cbtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -c -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/antho/Documents/MCU Electronics Tutorial/stm32f103cbt6_foc/Drivers/CMSIS/Include" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f103cbtx.d ./Core/Startup/startup_stm32f103cbtx.o

.PHONY: clean-Core-2f-Startup

