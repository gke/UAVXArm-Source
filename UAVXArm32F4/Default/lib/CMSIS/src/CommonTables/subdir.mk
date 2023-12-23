################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/CMSIS/src/CommonTables/arm_common_tables.c 

OBJS += \
./lib/CMSIS/src/CommonTables/arm_common_tables.o 

C_DEPS += \
./lib/CMSIS/src/CommonTables/arm_common_tables.d 


# Each subdirectory must supply rules for building sources it contributes
lib/CMSIS/src/CommonTables/%.o: ../lib/CMSIS/src/CommonTables/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

