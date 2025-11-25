################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Micro_Max.c \
../Core/Src/adc.c \
../Core/Src/cli.c \
../Core/Src/gpio.c \
../Core/Src/homing.c \
../Core/Src/main.c \
../Core/Src/motion.c \
../Core/Src/rng.c \
../Core/Src/spi.c \
../Core/Src/stepper.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/usb_otg.c 

OBJS += \
./Core/Src/Micro_Max.o \
./Core/Src/adc.o \
./Core/Src/cli.o \
./Core/Src/gpio.o \
./Core/Src/homing.o \
./Core/Src/main.o \
./Core/Src/motion.o \
./Core/Src/rng.o \
./Core/Src/spi.o \
./Core/Src/stepper.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/usb_otg.o 

C_DEPS += \
./Core/Src/Micro_Max.d \
./Core/Src/adc.d \
./Core/Src/cli.d \
./Core/Src/gpio.d \
./Core/Src/homing.d \
./Core/Src/main.d \
./Core/Src/motion.d \
./Core/Src/rng.d \
./Core/Src/spi.d \
./Core/Src/stepper.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/usb_otg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ssupa/Downloads/stepMotor_v2/stepMotor_v2/Drivers/ILI9341" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Micro_Max.cyclo ./Core/Src/Micro_Max.d ./Core/Src/Micro_Max.o ./Core/Src/Micro_Max.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/cli.cyclo ./Core/Src/cli.d ./Core/Src/cli.o ./Core/Src/cli.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/homing.cyclo ./Core/Src/homing.d ./Core/Src/homing.o ./Core/Src/homing.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motion.cyclo ./Core/Src/motion.d ./Core/Src/motion.o ./Core/Src/motion.su ./Core/Src/rng.cyclo ./Core/Src/rng.d ./Core/Src/rng.o ./Core/Src/rng.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stepper.cyclo ./Core/Src/stepper.d ./Core/Src/stepper.o ./Core/Src/stepper.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/usb_otg.cyclo ./Core/Src/usb_otg.d ./Core/Src/usb_otg.o ./Core/Src/usb_otg.su

.PHONY: clean-Core-2f-Src

