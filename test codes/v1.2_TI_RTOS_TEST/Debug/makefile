################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs900/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS

GEN_OPTS__FLAG := --cmd_file="configPkg/compiler.opt"
GEN_CMDS__FLAG := -l"configPkg/linker.cmd"

ORDERED_OBJS += \
"./EK_TM4C1294XL.obj" \
"./main.obj" \
"./FreeRTOS/croutine.obj" \
"./FreeRTOS/event_groups.obj" \
"./FreeRTOS/list.obj" \
"./FreeRTOS/queue.obj" \
"./FreeRTOS/tasks.obj" \
"./FreeRTOS/timers.obj" \
"./FreeRTOS/portable/CCS/ARM_CM4F/port.obj" \
"./FreeRTOS/portable/CCS/ARM_CM4F/portasm.obj" \
"./FreeRTOS/portable/MemMang/heap_2.obj" \
"./drivers/pinout.obj" \
"./utils/uartstdio.obj" \
"../EK_TM4C1294XL.cmd" \
$(GEN_CMDS__FLAG) \
-l"C:/ti/tirtos_tivac_2_16_00_08/products/TivaWare_C_Series-2.1.1.71b/grlib/ccs/Debug/grlib.lib" \
-l"C:/ti/tirtos_tivac_2_16_00_08/products/TivaWare_C_Series-2.1.1.71b/usblib/ccs/Debug/usblib.lib" \
-l"C:/ti/tirtos_tivac_2_16_00_08/products/TivaWare_C_Series-2.1.1.71b/driverlib/ccs/Debug/driverlib.lib" \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include FreeRTOS/subdir_vars.mk
-include FreeRTOS/portable/CCS/ARM_CM4F/subdir_vars.mk
-include FreeRTOS/portable/MemMang/subdir_vars.mk
-include drivers/subdir_vars.mk
-include utils/subdir_vars.mk
-include subdir_rules.mk
-include FreeRTOS/subdir_rules.mk
-include FreeRTOS/portable/CCS/ARM_CM4F/subdir_rules.mk
-include FreeRTOS/portable/MemMang/subdir_rules.mk
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
v1.2.out \

EXE_OUTPUTS__QUOTED += \
"v1.2.out" \

BIN_OUTPUTS += \
v1.2.hex \

BIN_OUTPUTS__QUOTED += \
"v1.2.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "v1.2.out"

# Tool invocations
v1.2.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=ccs --define=TIVAWARE -g --gcc --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi -z -m"v1.2.map" --heap_size=0 --stack_size=512 -i"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/lib" -i"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="v1.2_linkInfo.xml" --rom_model -o "v1.2.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

v1.2.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccs900/ccs/tools/compiler/ti-cgt-arm_18.12.1.LTS/bin/armhex"  -o "v1.2.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "EK_TM4C1294XL.obj" "main.obj" "FreeRTOS\croutine.obj" "FreeRTOS\event_groups.obj" "FreeRTOS\list.obj" "FreeRTOS\queue.obj" "FreeRTOS\tasks.obj" "FreeRTOS\timers.obj" "FreeRTOS\portable\CCS\ARM_CM4F\port.obj" "FreeRTOS\portable\CCS\ARM_CM4F\portasm.obj" "FreeRTOS\portable\MemMang\heap_2.obj" "drivers\pinout.obj" "utils\uartstdio.obj" 
	-$(RM) "EK_TM4C1294XL.d" "main.d" "FreeRTOS\croutine.d" "FreeRTOS\event_groups.d" "FreeRTOS\list.d" "FreeRTOS\queue.d" "FreeRTOS\tasks.d" "FreeRTOS\timers.d" "FreeRTOS\portable\CCS\ARM_CM4F\port.d" "FreeRTOS\portable\MemMang\heap_2.d" "drivers\pinout.d" "utils\uartstdio.d" 
	-$(RM) "FreeRTOS\portable\CCS\ARM_CM4F\portasm.d" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

