################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/optical/optical.c 

OBJS += \
./src/optical/optical.o 

C_DEPS += \
./src/optical/optical.d 


# Each subdirectory must supply rules for building sources it contributes
src/optical/%.o: ../src/optical/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

