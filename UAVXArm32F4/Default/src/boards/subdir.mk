################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/boards/harness.c 

OBJS += \
./src/boards/harness.o 

C_DEPS += \
./src/boards/harness.d 


# Each subdirectory must supply rules for building sources it contributes
src/boards/%.o: ../src/boards/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

