################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/oem/Desktop/STM32Cube/algorytm/Src/main.c \
C:/Users/oem/Desktop/STM32Cube/algorytm/Src/stm32f0xx_hal_msp.c \
C:/Users/oem/Desktop/STM32Cube/algorytm/Src/stm32f0xx_it.c 

OBJS += \
./Application/User/main.o \
./Application/User/stm32f0xx_hal_msp.o \
./Application/User/stm32f0xx_it.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/stm32f0xx_hal_msp.d \
./Application/User/stm32f0xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: C:/Users/oem/Desktop/STM32Cube/algorytm/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DUSE_HAL_DRIVER -DSTM32F051x8 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Inc" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/STM32F0xx_HAL_Driver/Inc" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/CMSIS/Include" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f0xx_hal_msp.o: C:/Users/oem/Desktop/STM32Cube/algorytm/Src/stm32f0xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DUSE_HAL_DRIVER -DSTM32F051x8 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Inc" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/STM32F0xx_HAL_Driver/Inc" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/CMSIS/Include" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f0xx_it.o: C:/Users/oem/Desktop/STM32Cube/algorytm/Src/stm32f0xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DUSE_HAL_DRIVER -DSTM32F051x8 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Inc" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/STM32F0xx_HAL_Driver/Inc" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Drivers/CMSIS/Include" -I"C:/Users/oem/Desktop/STM32Cube/algorytm/Inc"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


