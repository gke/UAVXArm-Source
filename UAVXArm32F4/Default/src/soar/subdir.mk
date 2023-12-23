################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/soar/MatrixMath.c \
../src/soar/ekf.c \
../src/soar/soar.c 

OBJS += \
./src/soar/MatrixMath.o \
./src/soar/ekf.o \
./src/soar/soar.o 

C_DEPS += \
./src/soar/MatrixMath.d \
./src/soar/ekf.d \
./src/soar/soar.d 


# Each subdirectory must supply rules for building sources it contributes
src/soar/%.o: ../src/soar/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

