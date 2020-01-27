################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include MEMS/App/subdir.mk
-include Drivers/STM32L1xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/Components/hts221/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
guau-fw.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
guau-fw.list \


# All Target
all: guau-fw.elf secondary-outputs

# Tool invocations
guau-fw.elf: $(OBJS) $(USER_OBJS) C:\Users\karen\STM32CubeIDE\workspace_1.1.0\guau-fw\STM32L151RDTX_FLASH.ld
	arm-none-eabi-gcc -o "guau-fw.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\karen\STM32CubeIDE\workspace_1.1.0\guau-fw\STM32L151RDTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="guau-fw.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

guau-fw.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "guau-fw.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
