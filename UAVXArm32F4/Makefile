SHELL=cmd

#V1 = -DV1_BOARD  // Beeper reversed but must be re-wired to 5V

SVN_REV := "xxxx"
#SVN_REV := 


#"$(shell "C:\Program Files (x86)\Subversion\bin\svnversion" -cn . | sed -e 's/[MS]//g' -e 's/^[[:digit:]]*://')" 
#SVN_REV := "$(shell "SubWCRev" -cn . | sed -e 's/[MS]//g' -e 's/^[[:digit:]]*://')" 
#SVN_REV := "$(shell "SubWCRev" .)" 


INSTALL_DIR = "D:/Flight/gitHub/UAVXArm.git/UAVXArm/firmware"

#-------------------------------------------------------------

##BOARD= OMNIBUSF4V1
#BOARD= OMNIBUSF4NXT
#BOARD= DISCOVERYF4
#BOARD= DEVEBOXF4

#BOARD= UAVXF4V3BBFLASH   
BOARD= UAVXF4V3
#BOARD= UAVXF4V4 


#########BOARD= OMNIBUSF4V3 NOT COMMISSIONED
#########BOARD= UAVXF4V2 RETIRED

#-------------------------------------------------------------
 
TARGET=$(BOARD)$(SVN_REV)
CONFIG= -DARM_MATH_CM4  -DHSE_VALUE=8000000

LINK_SCRIPT=stm32f4_eeprom16k.ld

# System configuration - UNCOMMENT AS DESIRED


TCHAIN="C:\Program Files (x86)\Atollic\TrueSTUDIO for STM32 9.3.0\ARMTools\bin\"
OPT = -O3 
#OPT = -Os

CC = $(TCHAIN)arm-atollic-eabi-gcc
OBJCOPY = $(TCHAIN)arm-atollic-eabi-objcopy
#CC = $(TCHAIN)gcc
#OBJCOPY = $(TCHAIN)objcopy

#FPFLAGS= -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mlittle-endian 
FPFLAGS=  -D__FPU_PRESENT -D__VFP_FP__ -mthumb -march=armv7e-m -mtune=cortex-m4 -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard 
WARNINGS= -Wall

#STM32F405RG -nostdinc -Wdouble-promotion -D__FPU_PRESENT -nostdinc -mlittle-endian
#mthumb-interwork
RM = rm -rf

# Define output directory
OBJECT_DIR = obj
BIN_DIR = $(OBJECT_DIR)


INCLUDES= -Isrc \
	-Isrc\stm \
	-Ilib\CMSIS\inc\ \
	-Ilib\Device\ST\STM32F4xx\Include\ \
	-Ilib\Std\inc \
	-Isrc\vcp \
	-Isrc\vcp\vcpf4 \
	-Isrc\vcp\drivers 

# Assembler, Compiler and Linker flags and linker script settings
LINKER_FLAGS= -lm $(FPFLAGS) -Wl,--gc-sections -T$(LINK_SCRIPT) -static \
	-Wl,-cref "-Wl,-Map=$(BIN_DIR)/$(TARGET).map" -Wl,--defsym=malloc_getpagesize_P=0x1000
	
ASSEMBLER_FLAGS=-c -g $(OPT) $(FPFLAGS) -x assembler-with-cpp  $(INCLUDES)

#-Wall 
COMPILER_FLAGS=-c -g $(OPT) $(FPFLAGS) $(CONFIG) -D$(BOARD) -finline-functions -ffunction-sections -fdata-sections \
	-D"STM32F407xx" -D"USE_STDPERIPH_DRIVER" $(INCLUDES)

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

STMLIB_OBJS := $(wildcard obj/lib/*/*/*/*/*/*/*.o) \
	$(wildcard obj/lib/*/*/*/*/*/*.o) \
	$(wildcard obj/lib/*/*/*/*/*.o) \
	$(wildcard obj/lib/*/*/*/*.o) \
	$(wildcard obj/lib/*/*/*.o) \
	$(wildcard obj/lib/*/*.o) \
	$(wildcard obj/lib/*.o)

MAIN_OBJS := $(wildcard obj/src/*/*/*/*/*/*/*.o) \
	$(wildcard obj/src/*/*/*/*/*/*.o) \
	$(wildcard obj/src/*/*/*/*/*.o) \
	$(wildcard obj/src/*/*/*/*.o) \
	$(wildcard obj/src/*/*/*.o) \
	$(wildcard obj/src/*/*.o) \
	$(wildcard obj/src/*.o)

	
all: uavxarm
	$(OBJCOPY) -O ihex "$(BIN_DIR)/$(TARGET).elf" "$(BIN_DIR)/$(TARGET).hex"
#	$(OBJCOPY) -O binary "$(BIN_DIR)/$(TARGET).elf" "$(BIN_DIR)/$(TARGET).bin"
	$(RM) $(OBJS) "$(BIN_DIR)/$(TARGET).elf" 
	$(RM) "$(BIN_DIR)/$(TARGET).map"
	$(RM) "$(BIN_DIR)/src" "$(BIN_DIR)/lib"
#	cp "$(BIN_DIR)/$(TARGET).hex" "$(INSTALL_DIR)/$(TARGET).hex"
	
svn: src/UAVXRevisionSVN.h
	SubWCRev . src/UAVXRevisionSVN.h src/UAVXRevision.h

uavxarm: svn  $(OBJS) 
	$(CC) -o "$(BIN_DIR)/$(TARGET).elf" $(OBJS)  $(LINKER_FLAGS) 
	
#bootloader: // requires termios! 
#		$(CC) -g -o stmloader -I./ \
#		./stmloader/loader.c ./stmloader/serial.c  ./stmloader/stmbootloader.c -Wall

clean:
#	$(RM) $(OBJS) "$(BIN_DIR)/*.*"
	$(RM) $(OBJS) "$(BIN_DIR)/$(TARGET).map" "$(BIN_DIR)/src" "$(BIN_DIR)/lib"
	
stm_libs:
	ar obj/lib/stmlib.a  $(STMLIB_OBJS)
	
$(OBJECT_DIR)/%.o: %.c
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(COMPILER_FLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.s
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.S
	@mkdir $(subst /,\,$(dir $@)) 2> NUL || echo off
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
