################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/HCI/hci.c 

OBJS += \
./Middlewares/HCI/hci.o 

C_DEPS += \
./Middlewares/HCI/hci.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/HCI/%.o Middlewares/HCI/%.su Middlewares/HCI/%.cyclo: ../Middlewares/HCI/%.c Middlewares/HCI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xC -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dsr -I../Drivers/BSP/Components/lps22hh -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../X-CUBE-MEMS1/Target -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Include" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Drivers/BSP/Components/BLE" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI/Controller" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Interface" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-HCI

clean-Middlewares-2f-HCI:
	-$(RM) ./Middlewares/HCI/hci.cyclo ./Middlewares/HCI/hci.d ./Middlewares/HCI/hci.o ./Middlewares/HCI/hci.su

.PHONY: clean-Middlewares-2f-HCI

