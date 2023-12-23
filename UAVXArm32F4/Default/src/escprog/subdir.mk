################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/escprog/serial_4way.c \
../src/escprog/serial_4way_avr.c \
../src/escprog/serial_4way_stk.c 

OBJS += \
./src/escprog/serial_4way.o \
./src/escprog/serial_4way_avr.o \
./src/escprog/serial_4way_stk.o 

C_DEPS += \
./src/escprog/serial_4way.d \
./src/escprog/serial_4way_avr.d \
./src/escprog/serial_4way_stk.d 


# Each subdirectory must supply rules for building sources it contributes
src/escprog/%.o: ../src/escprog/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

