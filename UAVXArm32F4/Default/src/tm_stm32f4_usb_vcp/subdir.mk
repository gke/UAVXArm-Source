################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/tm_stm32f4_usb_vcp/tm_stm32f4_usb_vcp.c \
../src/tm_stm32f4_usb_vcp/usb_bsp.c \
../src/tm_stm32f4_usb_vcp/usb_core.c \
../src/tm_stm32f4_usb_vcp/usb_dcd.c \
../src/tm_stm32f4_usb_vcp/usb_dcd_int.c \
../src/tm_stm32f4_usb_vcp/usbd_cdc_core.c \
../src/tm_stm32f4_usb_vcp/usbd_cdc_vcp.c \
../src/tm_stm32f4_usb_vcp/usbd_core.c \
../src/tm_stm32f4_usb_vcp/usbd_desc.c \
../src/tm_stm32f4_usb_vcp/usbd_ioreq.c \
../src/tm_stm32f4_usb_vcp/usbd_req.c \
../src/tm_stm32f4_usb_vcp/usbd_usr.c 

OBJS += \
./src/tm_stm32f4_usb_vcp/tm_stm32f4_usb_vcp.o \
./src/tm_stm32f4_usb_vcp/usb_bsp.o \
./src/tm_stm32f4_usb_vcp/usb_core.o \
./src/tm_stm32f4_usb_vcp/usb_dcd.o \
./src/tm_stm32f4_usb_vcp/usb_dcd_int.o \
./src/tm_stm32f4_usb_vcp/usbd_cdc_core.o \
./src/tm_stm32f4_usb_vcp/usbd_cdc_vcp.o \
./src/tm_stm32f4_usb_vcp/usbd_core.o \
./src/tm_stm32f4_usb_vcp/usbd_desc.o \
./src/tm_stm32f4_usb_vcp/usbd_ioreq.o \
./src/tm_stm32f4_usb_vcp/usbd_req.o \
./src/tm_stm32f4_usb_vcp/usbd_usr.o 

C_DEPS += \
./src/tm_stm32f4_usb_vcp/tm_stm32f4_usb_vcp.d \
./src/tm_stm32f4_usb_vcp/usb_bsp.d \
./src/tm_stm32f4_usb_vcp/usb_core.d \
./src/tm_stm32f4_usb_vcp/usb_dcd.d \
./src/tm_stm32f4_usb_vcp/usb_dcd_int.d \
./src/tm_stm32f4_usb_vcp/usbd_cdc_core.d \
./src/tm_stm32f4_usb_vcp/usbd_cdc_vcp.d \
./src/tm_stm32f4_usb_vcp/usbd_core.d \
./src/tm_stm32f4_usb_vcp/usbd_desc.d \
./src/tm_stm32f4_usb_vcp/usbd_ioreq.d \
./src/tm_stm32f4_usb_vcp/usbd_req.d \
./src/tm_stm32f4_usb_vcp/usbd_usr.d 


# Each subdirectory must supply rules for building sources it contributes
src/tm_stm32f4_usb_vcp/%.o: ../src/tm_stm32f4_usb_vcp/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

