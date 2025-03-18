################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include DSP_Lib/Source/TransformFunctions/subdir.mk
-include DSP_Lib/Source/SupportFunctions/subdir.mk
-include DSP_Lib/Source/StatisticsFunctions/subdir.mk
-include DSP_Lib/Source/MatrixFunctions/subdir.mk
-include DSP_Lib/Source/FilteringFunctions/subdir.mk
-include DSP_Lib/Source/FastMathFunctions/subdir.mk
-include DSP_Lib/Source/ControllerFunctions/subdir.mk
-include DSP_Lib/Source/ComplexMathFunctions/subdir.mk
-include DSP_Lib/Source/CommonTables/subdir.mk
-include DSP_Lib/Source/BasicMathFunctions/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := MAKET_VID_FINAL
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MAKET_VID_FINAL.elf \

MAP_FILES += \
MAKET_VID_FINAL.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MAKET_VID_FINAL.list \

OBJCOPY_HEX += \
MAKET_VID_FINAL.hex \


# All Target
all: main-build

# Main-build Target
main-build: MAKET_VID_FINAL.elf secondary-outputs

# Tool invocations
MAKET_VID_FINAL.elf MAKET_VID_FINAL.map: $(OBJS) $(USER_OBJS) C:\SOFT\Cube_IDE_pro\MAKET_VID_LAND\STM32F405RGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "MAKET_VID_FINAL.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\SOFT\Cube_IDE_pro\MAKET_VID_LAND\STM32F405RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MAKET_VID_FINAL.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MAKET_VID_FINAL.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MAKET_VID_FINAL.list"
	@echo 'Finished building: $@'
	@echo ' '

MAKET_VID_FINAL.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "MAKET_VID_FINAL.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) MAKET_VID_FINAL.elf MAKET_VID_FINAL.hex MAKET_VID_FINAL.list MAKET_VID_FINAL.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
