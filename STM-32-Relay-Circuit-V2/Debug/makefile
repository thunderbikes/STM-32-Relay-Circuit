################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F2xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := STM-32-Relay-Circuit-V2
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM-32-Relay-Circuit-V2.elf \

MAP_FILES += \
STM-32-Relay-Circuit-V2.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM-32-Relay-Circuit-V2.list \

OBJCOPY_HEX += \
STM-32-Relay-Circuit-V2.hex \

OBJCOPY_BIN += \
STM-32-Relay-Circuit-V2.bin \


# All Target
all: main-build

# Main-build Target
main-build: STM-32-Relay-Circuit-V2.elf secondary-outputs

# Tool invocations
STM-32-Relay-Circuit-V2.elf STM-32-Relay-Circuit-V2.map: $(OBJS) $(USER_OBJS) /home/oared/Projects/STM-32-Relay-Circuit/STM-32-Relay-Circuit-V2/STM32F205RBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "STM-32-Relay-Circuit-V2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/home/oared/Projects/STM-32-Relay-Circuit/STM-32-Relay-Circuit-V2/STM32F205RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM-32-Relay-Circuit-V2.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM-32-Relay-Circuit-V2.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM-32-Relay-Circuit-V2.list"
	@echo 'Finished building: $@'
	@echo ' '

STM-32-Relay-Circuit-V2.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "STM-32-Relay-Circuit-V2.hex"
	@echo 'Finished building: $@'
	@echo ' '

STM-32-Relay-Circuit-V2.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM-32-Relay-Circuit-V2.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) STM-32-Relay-Circuit-V2.bin STM-32-Relay-Circuit-V2.elf STM-32-Relay-Circuit-V2.hex STM-32-Relay-Circuit-V2.list STM-32-Relay-Circuit-V2.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
