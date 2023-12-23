################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/stm/syscalls.c \
../src/stm/system_stm32f4xx.c 

OBJS += \
./src/stm/syscalls.o \
./src/stm/system_stm32f4xx.o 

C_DEPS += \
./src/stm/syscalls.d \
./src/stm/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/stm/%.o: ../src/stm/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

