################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src/%.obj: ../src/%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_6.4.12/bin/cl2000" -v28 -ml -mt -O2 --include_path="C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_6.4.12/include" --include_path="C:/ti/motorware/motorware_1_01_00_18/sw/modules/hal/boards/boostxldrv8305_revA/f28x/f2806x/src" --include_path="C:/Users/src" --include_path="C:/ti/motorware/motorware_1_01_00_18" -g --define=FAST_ROM_V1p6 --define=FLASH --define=QEP --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="src/$(basename $(<F)).d_raw" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


