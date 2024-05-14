################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.c \
../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.c \
../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.c \
../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.c \
../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_pedal.c 

OBJS += \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.o \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.o \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.o \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.o \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_pedal.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.d \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.d \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.d \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.d \
./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_pedal.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/%.o Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/%.su Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/%.cyclo: ../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/%.c Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Host_Library/Class/HID/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-STM32_USB_Host_Library-2f-Class-2f-HID-2f-Src

clean-Middlewares-2f-ST-2f-STM32_USB_Host_Library-2f-Class-2f-HID-2f-Src:
	-$(RM) ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.cyclo ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.d ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.o ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid.su ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.cyclo ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.d ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.o ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_keybd.su ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.cyclo ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.d ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.o ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_mouse.su ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.cyclo ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.d ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.o ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_parser.su ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_pedal.cyclo ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_pedal.d ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_pedal.o ./Middlewares/ST/STM32_USB_Host_Library/Class/HID/Src/usbh_hid_pedal.su

.PHONY: clean-Middlewares-2f-ST-2f-STM32_USB_Host_Library-2f-Class-2f-HID-2f-Src

