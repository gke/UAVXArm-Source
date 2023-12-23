################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/magvar/magvar.c 

OBJS += \
./src/magvar/magvar.o 

C_DEPS += \
./src/magvar/magvar.d 


# Each subdirectory must supply rules for building sources it contributes
src/magvar/%.o: ../src/magvar/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

