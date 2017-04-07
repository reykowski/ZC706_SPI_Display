################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./Kentec320x240x16_ssd2119_8bit.obj" \
"./TubeImage.obj" \
"./ZC706_SPI_Display.obj" \
"./blue_moon_320x240.obj" \
"./flower320x240.obj" \
"./harmony320x240.obj" \
"./images.obj" \
"./lightning320x240.obj" \
"./soothe320x240.obj" \
"./stormy320x240.obj" \
"./tm4c123gh6pm_startup_ccs.obj" \
"./touch.obj" \
"./ustdlib.obj" \
"./drivers/rgb.obj" \
"./utils/uartstdio.obj" \
"../tm4c123gh6pm.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"C:/ti/TivaWare_C_Series-1.0/driverlib/ccs/Debug/driverlib.lib" \
-l"C:/ti/TivaWare_C_Series-1.0/sensorlib/ccs/Debug/sensorlib.lib" \
-l"C:/ti/TivaWare_C_Series-1.0/grlib/ccs/Debug/grlib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include drivers/subdir_vars.mk
-include utils/subdir_vars.mk
-include subdir_rules.mk
-include drivers/subdir_rules.mk
-include utils/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Display_Parameters.out \

EXE_OUTPUTS__QUOTED += \
"Display_Parameters.out" \

BIN_OUTPUTS += \
Display_Parameters.hex \

BIN_OUTPUTS__QUOTED += \
"Display_Parameters.hex" \


# All Target
all: Display_Parameters.out

# Tool invocations
Display_Parameters.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 --abi=eabi -me -Ooff -g --gcc --define=TARGET_IS_BLIZZARD_RB1 --define="ccs" --define=ccs="ccs" --define=PART_TM4C123GH6PM --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --ual -z -m"AD7294_ccs.map" --heap_size=0 --stack_size=1024 -i"D:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/lib" -i"D:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/include" --reread_libs --display_error_number --diag_wrap=off --warn_sections --xml_link_info="Display_Parameters_linkInfo.xml" --rom_model -o "Display_Parameters.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

Display_Parameters.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.1.LTS/bin/armhex"  -o "Display_Parameters.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "Kentec320x240x16_ssd2119_8bit.obj" "TubeImage.obj" "ZC706_SPI_Display.obj" "blue_moon_320x240.obj" "flower320x240.obj" "harmony320x240.obj" "images.obj" "lightning320x240.obj" "soothe320x240.obj" "stormy320x240.obj" "tm4c123gh6pm_startup_ccs.obj" "touch.obj" "ustdlib.obj" "drivers\rgb.obj" "utils\uartstdio.obj" 
	-$(RM) "Kentec320x240x16_ssd2119_8bit.d" "TubeImage.d" "ZC706_SPI_Display.d" "blue_moon_320x240.d" "flower320x240.d" "harmony320x240.d" "images.d" "lightning320x240.d" "soothe320x240.d" "stormy320x240.d" "tm4c123gh6pm_startup_ccs.d" "touch.d" "ustdlib.d" "drivers\rgb.d" "utils\uartstdio.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

