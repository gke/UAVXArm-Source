################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/alarms.c \
../src/altfilt.c \
../src/altitude.c \
../src/analog.c \
../src/armflash.c \
../src/as.c \
../src/auto.c \
../src/batt.c \
../src/bb.c \
../src/calib.c \
../src/clocks.c \
../src/control.c \
../src/emu.c \
../src/filters.c \
../src/frsky.c \
../src/gps.c \
../src/i2c.c \
../src/i2ceeprom.c \
../src/imu.c \
../src/inertial.c \
../src/isr.c \
../src/leds.c \
../src/mag.c \
../src/mission.c \
../src/mixer.c \
../src/mpu6xxx.c \
../src/nav.c \
../src/nvmem.c \
../src/outputs.c \
../src/params.c \
../src/rc.c \
../src/serial.c \
../src/sio.c \
../src/spi.c \
../src/spiflash.c \
../src/telem.c \
../src/temp.c \
../src/tests.c \
../src/tune.c \
../src/uavxarm-v3-gke.c 

OBJS += \
./src/alarms.o \
./src/altfilt.o \
./src/altitude.o \
./src/analog.o \
./src/armflash.o \
./src/as.o \
./src/auto.o \
./src/batt.o \
./src/bb.o \
./src/calib.o \
./src/clocks.o \
./src/control.o \
./src/emu.o \
./src/filters.o \
./src/frsky.o \
./src/gps.o \
./src/i2c.o \
./src/i2ceeprom.o \
./src/imu.o \
./src/inertial.o \
./src/isr.o \
./src/leds.o \
./src/mag.o \
./src/mission.o \
./src/mixer.o \
./src/mpu6xxx.o \
./src/nav.o \
./src/nvmem.o \
./src/outputs.o \
./src/params.o \
./src/rc.o \
./src/serial.o \
./src/sio.o \
./src/spi.o \
./src/spiflash.o \
./src/telem.o \
./src/temp.o \
./src/tests.o \
./src/tune.o \
./src/uavxarm-v3-gke.o 

C_DEPS += \
./src/alarms.d \
./src/altfilt.d \
./src/altitude.d \
./src/analog.d \
./src/armflash.d \
./src/as.d \
./src/auto.d \
./src/batt.d \
./src/bb.d \
./src/calib.d \
./src/clocks.d \
./src/control.d \
./src/emu.d \
./src/filters.d \
./src/frsky.d \
./src/gps.d \
./src/i2c.d \
./src/i2ceeprom.d \
./src/imu.d \
./src/inertial.d \
./src/isr.d \
./src/leds.d \
./src/mag.d \
./src/mission.d \
./src/mixer.d \
./src/mpu6xxx.d \
./src/nav.d \
./src/nvmem.d \
./src/outputs.d \
./src/params.d \
./src/rc.d \
./src/serial.d \
./src/sio.d \
./src/spi.d \
./src/spiflash.d \
./src/telem.d \
./src/temp.d \
./src/tests.d \
./src/tune.d \
./src/uavxarm-v3-gke.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	arm-atollic-eabi-gcc -c "$<" -std=gnu11 -O0 -g -fstack-usage -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -specs=nano.specs -o "$@"

