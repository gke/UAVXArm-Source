################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../src/stm/uavxarm_startups/startup_stm32f4xx.s 

OBJS += \
./src/stm/uavxarm_startups/startup_stm32f4xx.o 


# Each subdirectory must supply rules for building sources it contributes
src/stm/uavxarm_startups/%.o: ../src/stm/uavxarm_startups/%.s
	arm-atollic-eabi-gcc -c -g -Wa,--warn -x assembler-with-cpp -specs=nano.specs -o "$@" "$<"

