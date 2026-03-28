################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/HCI/Controller/bluenrg_gap_aci.c \
../Middlewares/HCI/Controller/bluenrg_gatt_aci.c \
../Middlewares/HCI/Controller/bluenrg_hal_aci.c \
../Middlewares/HCI/Controller/bluenrg_l2cap_aci.c \
../Middlewares/HCI/Controller/bluenrg_utils.c 

OBJS += \
./Middlewares/HCI/Controller/bluenrg_gap_aci.o \
./Middlewares/HCI/Controller/bluenrg_gatt_aci.o \
./Middlewares/HCI/Controller/bluenrg_hal_aci.o \
./Middlewares/HCI/Controller/bluenrg_l2cap_aci.o \
./Middlewares/HCI/Controller/bluenrg_utils.o 

C_DEPS += \
./Middlewares/HCI/Controller/bluenrg_gap_aci.d \
./Middlewares/HCI/Controller/bluenrg_gatt_aci.d \
./Middlewares/HCI/Controller/bluenrg_hal_aci.d \
./Middlewares/HCI/Controller/bluenrg_l2cap_aci.d \
./Middlewares/HCI/Controller/bluenrg_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/HCI/Controller/%.o Middlewares/HCI/Controller/%.su Middlewares/HCI/Controller/%.cyclo: ../Middlewares/HCI/Controller/%.c Middlewares/HCI/Controller/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xC -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dsr -I../Drivers/BSP/Components/lps22hh -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../X-CUBE-MEMS1/Target -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Include" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Drivers/BSP/Components/BLE" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/HCI/Controller" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Interface" -I"C:/Users/Zane/Documents/SP26/RTES/Final Project/stsw-fcu001/STSW-FCU001/STEVAL-FCU001V2/Middlewares/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-HCI-2f-Controller

clean-Middlewares-2f-HCI-2f-Controller:
	-$(RM) ./Middlewares/HCI/Controller/bluenrg_gap_aci.cyclo ./Middlewares/HCI/Controller/bluenrg_gap_aci.d ./Middlewares/HCI/Controller/bluenrg_gap_aci.o ./Middlewares/HCI/Controller/bluenrg_gap_aci.su ./Middlewares/HCI/Controller/bluenrg_gatt_aci.cyclo ./Middlewares/HCI/Controller/bluenrg_gatt_aci.d ./Middlewares/HCI/Controller/bluenrg_gatt_aci.o ./Middlewares/HCI/Controller/bluenrg_gatt_aci.su ./Middlewares/HCI/Controller/bluenrg_hal_aci.cyclo ./Middlewares/HCI/Controller/bluenrg_hal_aci.d ./Middlewares/HCI/Controller/bluenrg_hal_aci.o ./Middlewares/HCI/Controller/bluenrg_hal_aci.su ./Middlewares/HCI/Controller/bluenrg_l2cap_aci.cyclo ./Middlewares/HCI/Controller/bluenrg_l2cap_aci.d ./Middlewares/HCI/Controller/bluenrg_l2cap_aci.o ./Middlewares/HCI/Controller/bluenrg_l2cap_aci.su ./Middlewares/HCI/Controller/bluenrg_utils.cyclo ./Middlewares/HCI/Controller/bluenrg_utils.d ./Middlewares/HCI/Controller/bluenrg_utils.o ./Middlewares/HCI/Controller/bluenrg_utils.su

.PHONY: clean-Middlewares-2f-HCI-2f-Controller

