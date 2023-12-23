################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/CMSIS/src/FastMathFuncs/arm_cos_f32.c \
../lib/CMSIS/src/FastMathFuncs/arm_cos_q15.c \
../lib/CMSIS/src/FastMathFuncs/arm_cos_q31.c \
../lib/CMSIS/src/FastMathFuncs/arm_sin_f32.c \
../lib/CMSIS/src/FastMathFuncs/arm_sin_q15.c \
../lib/CMSIS/src/FastMathFuncs/arm_sin_q31.c \
../lib/CMSIS/src/FastMathFuncs/arm_sqrt_q15.c \
../lib/CMSIS/src/FastMathFuncs/arm_sqrt_q31.c 

OBJS += \
./lib/CMSIS/src/FastMathFuncs/arm_cos_f32.o \
./lib/CMSIS/src/FastMathFuncs/arm_cos_q15.o \
./lib/CMSIS/src/FastMathFuncs/arm_cos_q31.o \
./lib/CMSIS/src/FastMathFuncs/arm_sin_f32.o \
./lib/CMSIS/src/FastMathFuncs/arm_sin_q15.o \
./lib/CMSIS/src/FastMathFuncs/arm_sin_q31.o \
./lib/CMSIS/src/FastMathFuncs/arm_sqrt_q15.o \
./lib/CMSIS/src/FastMathFuncs/arm_sqrt_q31.o 

C_DEPS += \
./lib/CMSIS/src/FastMathFuncs/arm_cos_f32.d \
./lib/CMSIS/src/FastMathFuncs/arm_cos_q15.d \
./lib/CMSIS/src/FastMathFuncs/arm_cos_q31.d \
./lib/CMSIS/src/FastMathFuncs/arm_sin_f32.d \
./lib/CMSIS/src/FastMathFuncs/arm_sin_q15.d \
./lib/CMSIS/src/FastMathFuncs/arm_sin_q31.d \
./lib/CMSIS/src/FastMathFuncs/arm_sqrt_q15.d \
./lib/CMSIS/src/FastMathFuncs/arm_sqrt_q31.d 


# Each subdirectory must supply rules for building sources it contributes
lib/CMSIS/src/FastMathFuncs/%.o: ../lib/CMSIS/src/FastMathFuncs/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

