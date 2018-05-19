SHELL=cmd

#V1 = -DV1_BOARD  // Beeper reversed but must be re-wired to 5V


SVN_REV := "$(shell "C:\Program Files (x86)\Subversion\bin\svnversion" -cn . | sed -e 's/[MS]//g' -e 's/^[[:digit:]]*://')" 

#EXPFLAGS =  

INSTALL_DIR = "C:/Flight/gitHub/UAVXArm.git/UAVXArm/firmware"

TARGET=UAVXArm32F4_$(SVN_REV)_OMNIBUSF4V1
CONFIG= -DUSE_ATOLLIC -DOMNIBUSF4V1_BOARD -DARM_MATH_CM4  -DHSE_VALUE=8000000

#TARGET=UAVXArm32F4_$(SVN_REV)_V4
#CONFIG= -DV4_BOARD -DARM_MATH_CM4  -DHSE_VALUE=8000000

#==========================================================================================

#TARGET=UAVXArm32F4_$(SVN_REV)_V3
#CONFIG= -DV3_BOARD -DARM_MATH_CM4 -DHSE_VALUE=8000000

#==========================================================================================

# COMMISSIONING

#TARGET=LevArm32F4_V4_Test
#CONFIG = -DARM_MATH_CM4 -DV4_BOARD -DCOMMISSIONING_TEST $(V1)  -DHSE_VALUE=8000000

#TARGET=LevArm32F4_V3_Test
#CONFIG= -DARM_MATH_CM4 -DCOMMISSIONING_TEST $(V1) -DV3_BOARD -DUSE_UAVP_COMMS -DHSE_VALUE=8000000 

#==========================================================================================


LINK_SCRIPT=stm32f4_eeprom16k.ld

# System configuration - UNCOMMENT AS DESIRED

TCHAIN="C:\gcc\bin\"
#OPT = -O3
#OPT = -Os
OPT = -O3
CC = $(TCHAIN)arm-none-eabi-gcc
OBJCOPY = $(TCHAIN)arm-none-eabi-objcopy

#FPFLAGS= -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mlittle-endian 
FPFLAGS=  -D__FPU_PRESENT -D__VFP_FP__ -mthumb -march=armv7e-m -mtune=cortex-m4 -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard 
WARNINGS= -Wall

#STM32F405RG -nostdinc -Wdouble-promotion -D__FPU_PRESENT -nostdinc -mlittle-endian
#mthumb-interwork
RM = rm -rf

# Define output directory
OBJECT_DIR = obj
BIN_DIR = $(OBJECT_DIR)

# Assembler, Compiler and Linker flags and linker script settings
LINKER_FLAGS= -lm $(FPFLAGS) -Wl,--gc-sections -T$(LINK_SCRIPT) -static \
	-Wl,-cref "-Wl,-Map=$(BIN_DIR)/$(TARGET).map" -Wl,--defsym=malloc_getpagesize_P=0x1000
	
ASSEMBLER_FLAGS=-c -g $(OPT) $(FPFLAGS) -x assembler-with-cpp  \
	-Isrc \
	-Isrc\stm \
	-Ilib\STM32F4xx_StdPeriph_Driver\inc \
	-Ilib\STM32F4\Drivers\CMSIS\Device\ST\STM32F4xx\Include \
	-Isrc \
	-Isrc\stm \
	-Ilib\CMSIS\Include \
	-Ilib\STM32F4\Drivers\CMSIS\Device\ST\STM32F4xx\Include \
	-Ilib\Device\STM32F4xx\Include \
	-Ilib\CMSIS\Include \
	-Ilib\DSP_Lib\Include \
	-Ilib\STM32F4xx_StdPeriph_Driver\inc \
	-Ilib\STM32_USB_OTG_Driver\inc \
	-Ilib\STM32_USB_Device_Library\Class\cdc\inc \
	-Ilib\STM32_USB_Device_Library\Class\hid\inc \
	-Ilib\STM32_USB_Device_Library\Class\hid_cdc_wrapper\inc \
	-Ilib\STM32_USB_Device_Library\Class\msc\inc \
	-Ilib\STM32_USB_Device_Library\Core\inc \
	-Ilib\STM32_USB-FS-Device_Driver\inc \
	-Ilib\Std\inc \
	-Isrc\vcp \
	-Isrc\vcp\vcpf4 \
	-Isrc\vcp\vcp_hal \
	-Isrc\vcp\drivers 

#-Wall 
COMPILER_FLAGS=-c -g  $(OPT) $(FPFLAGS) $(CONFIG) -ffunction-sections -fdata-sections \
	-D"STM32F405xx" -D"USE_STDPERIPH_DRIVER"  \
	-Isrc \
	-Isrc\stm \
	-Ilib\CMSIS\Include \
	-Ilib\STM32F4\Drivers\CMSIS\Device\ST\STM32F4xx\Include \
	-Ilib\Device\STM32F4xx\Include \
	-Ilib\CMSIS\Include \
	-Ilib\DSP_Lib\Include \
	-Ilib\STM32F4xx_StdPeriph_Driver\inc \
	-Ilib\STM32_USB_OTG_Driver\inc \
	-Ilib\STM32_USB_Device_Library\Class\cdc\inc \
	-Ilib\STM32_USB_Device_Library\Class\hid\inc \
	-Ilib\STM32_USB_Device_Library\Class\hid_cdc_wrapper\inc \
	-Ilib\STM32_USB_Device_Library\Class\msc\inc \
	-Ilib\STM32_USB_Device_Library\Core\inc \
	-Ilib\STM32_USB-FS-Device_Driver\inc \
	-Ilib\Std\inc \
	-Isrc\escprog \
	-Isrc\vcp \
	-Isrc\vcp\vcpf4 \
	-Isrc\vcp\vcp_hal \
	-Isrc\vcp\drivers 

# Define sources and objects
SRC := $(wildcard */*/*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*/*.c) \
	$(wildcard */*/*/*/*/*.c) \
	$(wildcard */*/*/*/*.c) \
	$(wildcard */*/*/*.c) \
	$(wildcard */*/*.c) \
	$(wildcard */*.c)
	
MAIN_SRC := $(wildcard src/*/*/*/*/*/*/*.c) \
	$(wildcard src/*/*/*/*/*/*.c) \
	$(wildcard src/*/*/*/*/*.c) \
	$(wildcard src/*/*/*/*.c) \
	$(wildcard src/*/*/*.c) \
	$(wildcard src/*/*.c) \
	$(wildcard src/*.c)
	
	
ASM_SRC := $(wildcard */*/*/startup_stm32f4xx.s) \
#ASM_SRC := $(wildcard */*/*/*/*/*/*/*/*.s) \
#	$(wildcard */*/*/*/*/*/*/*.s) \
#	$(wildcard */*/*/*/*/*/*.s) \
#	$(wildcard */*/*/*/*/*.s) \
#	$(wildcard */*/*/*/*.s) \
#	$(wildcard */*/*/*.s) \
#	$(wildcard */*/*.s) \
#	$(wildcard */*.s)


OBJS := $(SRC:%.c=$(OBJECT_DIR)/%.o) $(ASM_SRC:%.s=$(OBJECT_DIR)/%.o)
OBJS := $(OBJS:%.S=$(OBJECT_DIR)/%.o) 

STD_PERIPHERAL_OBJS := $(wildcard obj/lib/Peripherals/src/*.o)
CMSIS_OBJS := $(wildcard obj/lib/CMSIS/src/*.o)

MAIN_OBJS := $(wildcard obj/src/*/*/*/*/*/*/*.o) \
	$(wildcard obj/src/*/*/*/*/*/*.o) \
	$(wildcard obj/src/*/*/*/*/*.o) \
	$(wildcard obj/src/*/*/*/*.o) \
	$(wildcard obj/src/*/*/*.o) \
	$(wildcard obj/src/*/*.o) \
	$(wildcard obj/src/*.o) \
	$(wildcard lib/peripherals/*/*/*/*/*/*/*.o) \
	$(wildcard lib/peripherals/*/*/*/*/*/*.o) \
	$(wildcard lib/peripherals/*/*/*/*/*.o) \
	$(wildcard lib/peripherals/*/*/*/*.o) \
	$(wildcard lib/peripherals/*/*/*.o) \
	$(wildcard lib/peripherals/*/*.o) \
	$(wildcard lib/peripherals/*.o)
	
all: uavxarm
	$(OBJCOPY) -O ihex "$(BIN_DIR)/$(TARGET).elf" "$(BIN_DIR)/$(TARGET).hex"
	$(RM) $(OBJS) "$(BIN_DIR)/$(TARGET).elf" "$(BIN_DIR)/$(TARGET).map" "$(BIN_DIR)/src" "$(BIN_DIR)/lib"
#	cp "$(BIN_DIR)/$(TARGET).hex" "$(INSTALL_DIR)/$(TARGET).hex"
	
svn: src/UAVXRevisionSVN.h
	SubWCRev . src/UAVXRevisionSVN.h src/UAVXRevision.h

uavxarm: svn $(OBJS) 
	$(CC) -o "$(BIN_DIR)/$(TARGET).elf" $(OBJS)  $(LINKER_FLAGS) 

clean:
#	$(RM) $(OBJS) "$(BIN_DIR)/*.*"
	$(RM) $(OBJS) "$(BIN_DIR)/*.elf" "$(BIN_DIR)/$(TARGET).map" "$(BIN_DIR)/src" "$(BIN_DIR)/lib"
	
peripheral_libs:
	ar  rcs obj/lib/libperipherals.a  $(STD_PERIPHERAL_OBJS)
	
cmsis_libs:
	ar  rcs obj/lib/libcmsis.a  $(CMSIS_OBJS)

$(OBJECT_DIR)/%.o: %.c
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(COMPILER_FLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.s
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.S
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
