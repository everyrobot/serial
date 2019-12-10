################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_6.4.12

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"C:/ti/motorware/motorware_1_01_00_18/sw/modules/fast/lib/32b/f28x/f2806x/2806xRevB_FastSpinROMSymbols.lib" \
"C:/ti/motorware/motorware_1_01_00_18/sw/modules/fast/lib/32b/f28x/f2806x/2806xRevB_IQmath_BootROMSymbols.lib" \
-lrts2800_ml.lib \
"C:/ti/motorware/motorware_1_01_00_18/sw/modules/spintac/lib/32b/SpinTAC.lib" \
"C:/ti/motorware/motorware_1_01_00_18/sw/modules/iqmath/lib/f28x/32b/IQmath.lib" \
"./CodeStartBranch.obj" \
"./adc.obj" \
"./clarke.obj" \
"./clk.obj" \
"./cpu.obj" \
"./ctrlQEP.obj" \
"./drv8305.obj" \
"./enc.obj" \
"./er_comm.obj" \
"./er_serial.obj" \
"./filter_fo.obj" \
"./flash.obj" \
"./gpio.obj" \
"./hal.obj" \
"./ipark.obj" \
"./main.obj" \
"./memCopy.obj" \
"./offset.obj" \
"./osc.obj" \
"./park.obj" \
"./pid.obj" \
"./pie.obj" \
"./pll.obj" \
"./pwm.obj" \
"./pwr.obj" \
"./qep.obj" \
"./sci.obj" \
"./slip.obj" \
"./spi.obj" \
"./svgen.obj" \
"./timer.obj" \
"./traj.obj" \
"./usDelay.obj" \
"./user.obj" \
"./wdog.obj" \
"./src/er_buffer.obj" \
"./src/er_command.obj" \
"./src/er_msg.obj" \
"C:/ti/motorware/motorware_1_01_00_18/sw/ide/ccs/cmd/f2806x/F28069M.cmd" \
$(GEN_CMDS__FLAG) \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include src/subdir_vars.mk
-include subdir_rules.mk
-include src/subdir_rules.mk
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
er_ti_f28069m_drv8305.out \

EXE_OUTPUTS__QUOTED += \
"er_ti_f28069m_drv8305.out" \


# All Target
all: er_ti_f28069m_drv8305.out

# Tool invocations
er_ti_f28069m_drv8305.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_6.4.12/bin/cl2000" -v28 -ml -mt -O2 -g --define=FAST_ROM_V1p6 --define=FLASH --define=QEP --diag_warning=225 --display_error_number -z -m"er_ti_f28069m_drv8305.map" --stack_size=0x3B0 --warn_sections -i"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_6.4.12/lib" -i"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_6.4.12/include" --reread_libs --priority --disable_auto_rts --diag_suppress=16002 --xml_link_info="er_ti_f28069m_drv8305_linkInfo.xml" --rom_model -o "er_ti_f28069m_drv8305.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "CodeStartBranch.obj" "adc.obj" "clarke.obj" "clk.obj" "cpu.obj" "ctrlQEP.obj" "drv8305.obj" "enc.obj" "er_comm.obj" "er_serial.obj" "filter_fo.obj" "flash.obj" "gpio.obj" "hal.obj" "ipark.obj" "main.obj" "memCopy.obj" "offset.obj" "osc.obj" "park.obj" "pid.obj" "pie.obj" "pll.obj" "pwm.obj" "pwr.obj" "qep.obj" "sci.obj" "slip.obj" "spi.obj" "svgen.obj" "timer.obj" "traj.obj" "usDelay.obj" "user.obj" "wdog.obj" "src\er_buffer.obj" "src\er_command.obj" "src\er_msg.obj" 
	-$(RM) "adc.d" "clarke.d" "clk.d" "cpu.d" "ctrlQEP.d" "drv8305.d" "enc.d" "er_comm.d" "er_serial.d" "filter_fo.d" "flash.d" "gpio.d" "hal.d" "ipark.d" "main.d" "memCopy.d" "offset.d" "osc.d" "park.d" "pid.d" "pie.d" "pll.d" "pwm.d" "pwr.d" "qep.d" "sci.d" "slip.d" "spi.d" "svgen.d" "timer.d" "traj.d" "user.d" "wdog.d" "src\er_buffer.d" "src\er_command.d" "src\er_msg.d" 
	-$(RM) "CodeStartBranch.d" "usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

