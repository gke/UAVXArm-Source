################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/oled/SSD1X06.c 

OBJS += \
./src/oled/SSD1X06.o 

C_DEPS += \
./src/oled/SSD1X06.d 


# Each subdirectory must supply rules for building sources it contributes
src/oled/%.o: ../src/oled/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

