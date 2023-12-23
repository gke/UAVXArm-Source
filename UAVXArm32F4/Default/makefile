################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := del

# All of the sources participating in the build are defined here
-include sources.mk
-include src/tm_stm32f4_usb_vcp/subdir.mk
-include src/stm/uavxarm_startups/subdir.mk
-include src/stm/subdir.mk
-include src/soar/subdir.mk
-include src/optical/subdir.mk
-include src/oled/subdir.mk
-include src/magvar/subdir.mk
-include src/escprog/subdir.mk
-include src/boards/subdir.mk
-include src/subdir.mk
-include obj/src/tm_stm32f4_usb_vcp/subdir.mk
-include obj/src/stm/uavxarm_startups/subdir.mk
-include obj/src/stm/subdir.mk
-include obj/src/soar/subdir.mk
-include obj/src/optical/subdir.mk
-include obj/src/oled/subdir.mk
-include obj/src/magvar/subdir.mk
-include obj/src/escprog/subdir.mk
-include obj/src/boards/subdir.mk
-include obj/src/subdir.mk
-include obj/lib/Std/src/subdir.mk
-include obj/lib/CMSIS/src/SupportFunctions/subdir.mk
-include obj/lib/CMSIS/src/SupFuncs/subdir.mk
-include obj/lib/CMSIS/src/FastMathFunctions/subdir.mk
-include obj/lib/CMSIS/src/FastMathFuncs/subdir.mk
-include obj/lib/CMSIS/src/CommonTables/subdir.mk
-include obj/lib/CMSIS/src/BasicMathFunctions/subdir.mk
-include obj/lib/CMSIS/src/BasMathFuncs/subdir.mk
-include lib/Std/src/subdir.mk
-include lib/CMSIS/src/SupportFunctions/subdir.mk
-include lib/CMSIS/src/SupFuncs/subdir.mk
-include lib/CMSIS/src/FastMathFunctions/subdir.mk
-include lib/CMSIS/src/FastMathFuncs/subdir.mk
-include lib/CMSIS/src/CommonTables/subdir.mk
-include lib/CMSIS/src/BasicMathFunctions/subdir.mk
-include lib/CMSIS/src/BasMathFuncs/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
USER_OBJS.elf \

SECOUTPUTBUILDVAR += \
EXECUTABLES \


# All Target
all: UAVXArm32F4 secondary-outputs

# Tool invocations
	@echo 'No tool found that can build the extension specified with the build artifact name $@'
USER_OBJS.elf: $(OBJS) $(USER_OBJS)
	arm-atollic-eabi-g++ -o "USER_OBJS.elf" $(OBJS) $(USER_OBJS) $(LIBS) -specs=nosys.specs -static -Wl,-cref,-u,Reset_Handler "-Wl,-Map=UAVXArm32F4.map" -Wl,--defsym=malloc_getpagesize_P=0x1000 -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group -specs=nano.specs

EXECUTABLES: UAVXArm32F4 $(EXECUTABLES)
	"C:\Program Files (x86)\Atollic\TrueSTUDIO for STM32 9.3.0\ide\jre\bin\java" -jar "C:\Program Files (x86)\Atollic\TrueSTUDIO for STM32 9.3.0\Tools\arm-atollic-reports.jar" sizeinfo list UAVXArm32F4

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(SECOUTPUTBUILDVAR)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(C_DEPS)$(CPP_DEPS) UAVXArm32F4
	-@echo ' '

secondary-outputs: $(SECOUTPUTBUILDVAR)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
