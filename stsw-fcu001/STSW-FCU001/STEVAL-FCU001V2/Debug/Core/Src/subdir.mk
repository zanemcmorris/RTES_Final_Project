################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/PID.c \
../Core/Src/ahrs.c \
../Core/Src/basic_math.c \
../Core/Src/console.c \
../Core/Src/custom_bus.c \
../Core/Src/debug.c \
../Core/Src/flight_control.c \
../Core/Src/main.c \
../Core/Src/motor.c \
../Core/Src/quaternion.c \
../Core/Src/rc.c \
../Core/Src/sensor_service.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/timer.c 

OBJS += \
./Core/Src/PID.o \
./Core/Src/ahrs.o \
./Core/Src/basic_math.o \
./Core/Src/console.o \
./Core/Src/custom_bus.o \
./Core/Src/debug.o \
./Core/Src/flight_control.o \
./Core/Src/main.o \
./Core/Src/motor.o \
./Core/Src/quaternion.o \
./Core/Src/rc.o \
./Core/Src/sensor_service.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/timer.o 

C_DEPS += \
./Core/Src/PID.d \
./Core/Src/ahrs.d \
./Core/Src/basic_math.d \
./Core/Src/console.d \
./Core/Src/custom_bus.d \
./Core/Src/debug.d \
./Core/Src/flight_control.d \
./Core/Src/main.d \
./Core/Src/motor.d \
./Core/Src/quaternion.d \
./Core/Src/rc.d \
./Core/Src/sensor_service.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xC -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dsr -I../Drivers/BSP/Components/lps22hh -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../X-CUBE-MEMS1/Target -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Include" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Drivers/BSP/Components/BLE" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI/Controller" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Interface" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/PID.cyclo ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/PID.su ./Core/Src/ahrs.cyclo ./Core/Src/ahrs.d ./Core/Src/ahrs.o ./Core/Src/ahrs.su ./Core/Src/basic_math.cyclo ./Core/Src/basic_math.d ./Core/Src/basic_math.o ./Core/Src/basic_math.su ./Core/Src/console.cyclo ./Core/Src/console.d ./Core/Src/console.o ./Core/Src/console.su ./Core/Src/custom_bus.cyclo ./Core/Src/custom_bus.d ./Core/Src/custom_bus.o ./Core/Src/custom_bus.su ./Core/Src/debug.cyclo ./Core/Src/debug.d ./Core/Src/debug.o ./Core/Src/debug.su ./Core/Src/flight_control.cyclo ./Core/Src/flight_control.d ./Core/Src/flight_control.o ./Core/Src/flight_control.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/quaternion.cyclo ./Core/Src/quaternion.d ./Core/Src/quaternion.o ./Core/Src/quaternion.su ./Core/Src/rc.cyclo ./Core/Src/rc.d ./Core/Src/rc.o ./Core/Src/rc.su ./Core/Src/sensor_service.cyclo ./Core/Src/sensor_service.d ./Core/Src/sensor_service.o ./Core/Src/sensor_service.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/timer.cyclo ./Core/Src/timer.d ./Core/Src/timer.o ./Core/Src/timer.su

.PHONY: clean-Core-2f-Src

