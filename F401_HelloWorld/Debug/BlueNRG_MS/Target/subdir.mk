################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BlueNRG_MS/Target/hci_tl_interface.c 

OBJS += \
./BlueNRG_MS/Target/hci_tl_interface.o 

C_DEPS += \
./BlueNRG_MS/Target/hci_tl_interface.d 


# Each subdirectory must supply rules for building sources it contributes
BlueNRG_MS/Target/%.o BlueNRG_MS/Target/%.su BlueNRG_MS/Target/%.cyclo: ../BlueNRG_MS/Target/%.c BlueNRG_MS/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../BlueNRG_MS/App -I../BlueNRG_MS/Target -I../Middlewares/ST/BlueNRG-MS/utils -I../Middlewares/ST/BlueNRG-MS/includes -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BlueNRG_MS-2f-Target

clean-BlueNRG_MS-2f-Target:
	-$(RM) ./BlueNRG_MS/Target/hci_tl_interface.cyclo ./BlueNRG_MS/Target/hci_tl_interface.d ./BlueNRG_MS/Target/hci_tl_interface.o ./BlueNRG_MS/Target/hci_tl_interface.su

.PHONY: clean-BlueNRG_MS-2f-Target

