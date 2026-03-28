################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Utils/ble_list.c \
../Middlewares/Utils/gp_timer.c \
../Middlewares/Utils/osal.c 

OBJS += \
./Middlewares/Utils/ble_list.o \
./Middlewares/Utils/gp_timer.o \
./Middlewares/Utils/osal.o 

C_DEPS += \
./Middlewares/Utils/ble_list.d \
./Middlewares/Utils/gp_timer.d \
./Middlewares/Utils/osal.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Utils/%.o Middlewares/Utils/%.su Middlewares/Utils/%.cyclo: ../Middlewares/Utils/%.c Middlewares/Utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xC -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dsr -I../Drivers/BSP/Components/lps22hh -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../X-CUBE-MEMS1/Target -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Include" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Drivers/BSP/Components/BLE" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI/Controller" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Interface" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Utils

clean-Middlewares-2f-Utils:
	-$(RM) ./Middlewares/Utils/ble_list.cyclo ./Middlewares/Utils/ble_list.d ./Middlewares/Utils/ble_list.o ./Middlewares/Utils/ble_list.su ./Middlewares/Utils/gp_timer.cyclo ./Middlewares/Utils/gp_timer.d ./Middlewares/Utils/gp_timer.o ./Middlewares/Utils/gp_timer.su ./Middlewares/Utils/osal.cyclo ./Middlewares/Utils/osal.d ./Middlewares/Utils/osal.o ./Middlewares/Utils/osal.su

.PHONY: clean-Middlewares-2f-Utils

