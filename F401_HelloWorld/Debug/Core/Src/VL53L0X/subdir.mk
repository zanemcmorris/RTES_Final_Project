################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/VL53L0X/vl53l0x_api.c \
../Core/Src/VL53L0X/vl53l0x_api_calibration.c \
../Core/Src/VL53L0X/vl53l0x_api_core.c \
../Core/Src/VL53L0X/vl53l0x_api_ranging.c \
../Core/Src/VL53L0X/vl53l0x_api_strings.c \
../Core/Src/VL53L0X/vl53l0x_platform_stm32.c 

OBJS += \
./Core/Src/VL53L0X/vl53l0x_api.o \
./Core/Src/VL53L0X/vl53l0x_api_calibration.o \
./Core/Src/VL53L0X/vl53l0x_api_core.o \
./Core/Src/VL53L0X/vl53l0x_api_ranging.o \
./Core/Src/VL53L0X/vl53l0x_api_strings.o \
./Core/Src/VL53L0X/vl53l0x_platform_stm32.o 

C_DEPS += \
./Core/Src/VL53L0X/vl53l0x_api.d \
./Core/Src/VL53L0X/vl53l0x_api_calibration.d \
./Core/Src/VL53L0X/vl53l0x_api_core.d \
./Core/Src/VL53L0X/vl53l0x_api_ranging.d \
./Core/Src/VL53L0X/vl53l0x_api_strings.d \
./Core/Src/VL53L0X/vl53l0x_platform_stm32.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/VL53L0X/%.o Core/Src/VL53L0X/%.su Core/Src/VL53L0X/%.cyclo: ../Core/Src/VL53L0X/%.c Core/Src/VL53L0X/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Core/Inc/VL53L0X -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-VL53L0X

clean-Core-2f-Src-2f-VL53L0X:
	-$(RM) ./Core/Src/VL53L0X/vl53l0x_api.cyclo ./Core/Src/VL53L0X/vl53l0x_api.d ./Core/Src/VL53L0X/vl53l0x_api.o ./Core/Src/VL53L0X/vl53l0x_api.su ./Core/Src/VL53L0X/vl53l0x_api_calibration.cyclo ./Core/Src/VL53L0X/vl53l0x_api_calibration.d ./Core/Src/VL53L0X/vl53l0x_api_calibration.o ./Core/Src/VL53L0X/vl53l0x_api_calibration.su ./Core/Src/VL53L0X/vl53l0x_api_core.cyclo ./Core/Src/VL53L0X/vl53l0x_api_core.d ./Core/Src/VL53L0X/vl53l0x_api_core.o ./Core/Src/VL53L0X/vl53l0x_api_core.su ./Core/Src/VL53L0X/vl53l0x_api_ranging.cyclo ./Core/Src/VL53L0X/vl53l0x_api_ranging.d ./Core/Src/VL53L0X/vl53l0x_api_ranging.o ./Core/Src/VL53L0X/vl53l0x_api_ranging.su ./Core/Src/VL53L0X/vl53l0x_api_strings.cyclo ./Core/Src/VL53L0X/vl53l0x_api_strings.d ./Core/Src/VL53L0X/vl53l0x_api_strings.o ./Core/Src/VL53L0X/vl53l0x_api_strings.su ./Core/Src/VL53L0X/vl53l0x_platform_stm32.cyclo ./Core/Src/VL53L0X/vl53l0x_platform_stm32.d ./Core/Src/VL53L0X/vl53l0x_platform_stm32.o ./Core/Src/VL53L0X/vl53l0x_platform_stm32.su

.PHONY: clean-Core-2f-Src-2f-VL53L0X

