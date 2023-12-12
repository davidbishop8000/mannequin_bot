################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/printf/printf.c 

C_DEPS += \
./Core/Src/printf/printf.d 

OBJS += \
./Core/Src/printf/printf.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/printf/%.o Core/Src/printf/%.su: ../Core/Src/printf/%.c Core/Src/printf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-printf

clean-Core-2f-Src-2f-printf:
	-$(RM) ./Core/Src/printf/printf.d ./Core/Src/printf/printf.o ./Core/Src/printf/printf.su

.PHONY: clean-Core-2f-Src-2f-printf

