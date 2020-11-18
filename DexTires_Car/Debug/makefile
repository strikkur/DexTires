################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
DexTires_Car.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
DexTires_Car.list \

OBJCOPY_BIN += \
DexTires_Car.bin \


# All Target
all: DexTires_Car.elf secondary-outputs

# Tool invocations
DexTires_Car.elf: $(OBJS) $(USER_OBJS) C:\Users\Lakshya\ Goyal\STM32CubeIDE\workspace_1.4.0\DexTires_Car\STM32F051R8TX_FLASH.ld
	arm-none-eabi-gcc -o "DexTires_Car.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\Lakshya Goyal\STM32CubeIDE\workspace_1.4.0\DexTires_Car\STM32F051R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="DexTires_Car.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

DexTires_Car.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "DexTires_Car.list"
	@echo 'Finished building: $@'
	@echo ' '

DexTires_Car.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "DexTires_Car.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
