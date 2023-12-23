################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/CMSIS/src/SupFuncs/arm_copy_f32.c \
../lib/CMSIS/src/SupFuncs/arm_copy_q15.c \
../lib/CMSIS/src/SupFuncs/arm_copy_q31.c \
../lib/CMSIS/src/SupFuncs/arm_copy_q7.c \
../lib/CMSIS/src/SupFuncs/arm_fill_f32.c \
../lib/CMSIS/src/SupFuncs/arm_fill_q15.c \
../lib/CMSIS/src/SupFuncs/arm_fill_q31.c \
../lib/CMSIS/src/SupFuncs/arm_fill_q7.c \
../lib/CMSIS/src/SupFuncs/arm_float_to_q15.c \
../lib/CMSIS/src/SupFuncs/arm_float_to_q31.c \
../lib/CMSIS/src/SupFuncs/arm_float_to_q7.c \
../lib/CMSIS/src/SupFuncs/arm_q15_to_float.c \
../lib/CMSIS/src/SupFuncs/arm_q15_to_q31.c \
../lib/CMSIS/src/SupFuncs/arm_q15_to_q7.c \
../lib/CMSIS/src/SupFuncs/arm_q31_to_float.c \
../lib/CMSIS/src/SupFuncs/arm_q31_to_q15.c \
../lib/CMSIS/src/SupFuncs/arm_q31_to_q7.c \
../lib/CMSIS/src/SupFuncs/arm_q7_to_float.c \
../lib/CMSIS/src/SupFuncs/arm_q7_to_q15.c \
../lib/CMSIS/src/SupFuncs/arm_q7_to_q31.c 

OBJS += \
./lib/CMSIS/src/SupFuncs/arm_copy_f32.o \
./lib/CMSIS/src/SupFuncs/arm_copy_q15.o \
./lib/CMSIS/src/SupFuncs/arm_copy_q31.o \
./lib/CMSIS/src/SupFuncs/arm_copy_q7.o \
./lib/CMSIS/src/SupFuncs/arm_fill_f32.o \
./lib/CMSIS/src/SupFuncs/arm_fill_q15.o \
./lib/CMSIS/src/SupFuncs/arm_fill_q31.o \
./lib/CMSIS/src/SupFuncs/arm_fill_q7.o \
./lib/CMSIS/src/SupFuncs/arm_float_to_q15.o \
./lib/CMSIS/src/SupFuncs/arm_float_to_q31.o \
./lib/CMSIS/src/SupFuncs/arm_float_to_q7.o \
./lib/CMSIS/src/SupFuncs/arm_q15_to_float.o \
./lib/CMSIS/src/SupFuncs/arm_q15_to_q31.o \
./lib/CMSIS/src/SupFuncs/arm_q15_to_q7.o \
./lib/CMSIS/src/SupFuncs/arm_q31_to_float.o \
./lib/CMSIS/src/SupFuncs/arm_q31_to_q15.o \
./lib/CMSIS/src/SupFuncs/arm_q31_to_q7.o \
./lib/CMSIS/src/SupFuncs/arm_q7_to_float.o \
./lib/CMSIS/src/SupFuncs/arm_q7_to_q15.o \
./lib/CMSIS/src/SupFuncs/arm_q7_to_q31.o 

C_DEPS += \
./lib/CMSIS/src/SupFuncs/arm_copy_f32.d \
./lib/CMSIS/src/SupFuncs/arm_copy_q15.d \
./lib/CMSIS/src/SupFuncs/arm_copy_q31.d \
./lib/CMSIS/src/SupFuncs/arm_copy_q7.d \
./lib/CMSIS/src/SupFuncs/arm_fill_f32.d \
./lib/CMSIS/src/SupFuncs/arm_fill_q15.d \
./lib/CMSIS/src/SupFuncs/arm_fill_q31.d \
./lib/CMSIS/src/SupFuncs/arm_fill_q7.d \
./lib/CMSIS/src/SupFuncs/arm_float_to_q15.d \
./lib/CMSIS/src/SupFuncs/arm_float_to_q31.d \
./lib/CMSIS/src/SupFuncs/arm_float_to_q7.d \
./lib/CMSIS/src/SupFuncs/arm_q15_to_float.d \
./lib/CMSIS/src/SupFuncs/arm_q15_to_q31.d \
./lib/CMSIS/src/SupFuncs/arm_q15_to_q7.d \
./lib/CMSIS/src/SupFuncs/arm_q31_to_float.d \
./lib/CMSIS/src/SupFuncs/arm_q31_to_q15.d \
./lib/CMSIS/src/SupFuncs/arm_q31_to_q7.d \
./lib/CMSIS/src/SupFuncs/arm_q7_to_float.d \
./lib/CMSIS/src/SupFuncs/arm_q7_to_q15.d \
./lib/CMSIS/src/SupFuncs/arm_q7_to_q31.d 


# Each subdirectory must supply rules for building sources it contributes
lib/CMSIS/src/SupFuncs/%.o: ../lib/CMSIS/src/SupFuncs/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

