################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm6dsr/lsm6dsr.c \
../Drivers/BSP/Components/lsm6dsr/lsm6dsr_reg.c 

OBJS += \
./Drivers/BSP/Components/lsm6dsr/lsm6dsr.o \
./Drivers/BSP/Components/lsm6dsr/lsm6dsr_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lsm6dsr/lsm6dsr.d \
./Drivers/BSP/Components/lsm6dsr/lsm6dsr_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm6dsr/%.o Drivers/BSP/Components/lsm6dsr/%.su Drivers/BSP/Components/lsm6dsr/%.cyclo: ../Drivers/BSP/Components/lsm6dsr/%.c Drivers/BSP/Components/lsm6dsr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xC -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dsr -I../Drivers/BSP/Components/lps22hh -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../X-CUBE-MEMS1/Target -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Include" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Drivers/BSP/Components/BLE" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI/Controller" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Interface" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsr

clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsr:
	-$(RM) ./Drivers/BSP/Components/lsm6dsr/lsm6dsr.cyclo ./Drivers/BSP/Components/lsm6dsr/lsm6dsr.d ./Drivers/BSP/Components/lsm6dsr/lsm6dsr.o ./Drivers/BSP/Components/lsm6dsr/lsm6dsr.su ./Drivers/BSP/Components/lsm6dsr/lsm6dsr_reg.cyclo ./Drivers/BSP/Components/lsm6dsr/lsm6dsr_reg.d ./Drivers/BSP/Components/lsm6dsr/lsm6dsr_reg.o ./Drivers/BSP/Components/lsm6dsr/lsm6dsr_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsr

