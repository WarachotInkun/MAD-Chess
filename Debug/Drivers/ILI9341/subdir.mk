################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ILI9341/ILI9341_GFX.c \
../Drivers/ILI9341/ILI9341_STM32_Driver.c \
../Drivers/ILI9341/ILI9341_Touchscreen.c 

OBJS += \
./Drivers/ILI9341/ILI9341_GFX.o \
./Drivers/ILI9341/ILI9341_STM32_Driver.o \
./Drivers/ILI9341/ILI9341_Touchscreen.o 

C_DEPS += \
./Drivers/ILI9341/ILI9341_GFX.d \
./Drivers/ILI9341/ILI9341_STM32_Driver.d \
./Drivers/ILI9341/ILI9341_Touchscreen.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ILI9341/%.o Drivers/ILI9341/%.su Drivers/ILI9341/%.cyclo: ../Drivers/ILI9341/%.c Drivers/ILI9341/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ssupa/Downloads/stepMotor_v2/stepMotor_v2/Drivers/ILI9341" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ILI9341

clean-Drivers-2f-ILI9341:
	-$(RM) ./Drivers/ILI9341/ILI9341_GFX.cyclo ./Drivers/ILI9341/ILI9341_GFX.d ./Drivers/ILI9341/ILI9341_GFX.o ./Drivers/ILI9341/ILI9341_GFX.su ./Drivers/ILI9341/ILI9341_STM32_Driver.cyclo ./Drivers/ILI9341/ILI9341_STM32_Driver.d ./Drivers/ILI9341/ILI9341_STM32_Driver.o ./Drivers/ILI9341/ILI9341_STM32_Driver.su ./Drivers/ILI9341/ILI9341_Touchscreen.cyclo ./Drivers/ILI9341/ILI9341_Touchscreen.d ./Drivers/ILI9341/ILI9341_Touchscreen.o ./Drivers/ILI9341/ILI9341_Touchscreen.su

.PHONY: clean-Drivers-2f-ILI9341

