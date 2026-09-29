######################### Project Settings #########################
PRJ_NAME = Teco
# What to include, in form dir1 dir2 dir3...
INCLUDE_DIRS = ./ include libopencm3 libopencm3/cm3 libopencm3/dispatch libopencm3/stm32 libopencm3/stm32/f1 libopencm3/stm32/common libopencm3/usb libopencmsis libopencmsis/dispatch libopencmsis/stm32/f1 platforms platforms/common platforms/common/stm32 target
# What to define in form MYDEF1 MYDEF2=18 MYDEF3... $(MAKECMDGOALS) is name of requested action
DEFINS = __ARM_ACLE STM32F1

######################### Figure out what to do #########################
# Target must be in the following form: build_Release, clean_Debug, flash_Fromboot
# The first word is the action (build, clean, flash);
# the second is the name of the configuration and the name of the OUT_DIR.
INPUT_WORDS = $(subst _, ,$(MAKECMDGOALS)) # Replace '_' with ' '
ACTION = $(word 1,$(INPUT_WORDS))
GOAL_NAME = $(word 2,$(INPUT_WORDS))
OUT_DIR = $(GOAL_NAME)
# Run Action (build_Release: build  clean_Debug: clean  flash_Fromboot: flash)
$(MAKECMDGOALS): $(ACTION)


######################### Common Cfg Settings #########################
MCU = cortex-m3
FLOAT_FLAGS = # -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wno-double-promotion -Wfloat-conversion # -Wfloat-equal
WARNING_FLAGS = -Wall -Wlogical-op # -Werror
DISABLED_WARNINGS = -Wno-address-of-packed-member -Wno-unknown-pragmas -Wno-volatile
COMMON_FLAGS = -mcpu=$(MCU) -mthumb -fmessage-length=0 -ffunction-sections -fdata-sections -ffreestanding $(FLOAT_FLAGS) $(WARNING_FLAGS)
CPP_FLAGS = -std=gnu++20 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics $(DISABLED_WARNINGS) $(OUTPUT_ASM)
C_FLAGS = -std=gnu17 -Wno-unknown-pragmas $(OUTPUT_ASM)
# Add this to build commands. Nothing to change here.
OBJ_FLAGS = -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
ELF_NAME = $(OUT_DIR)/$(PRJ_NAME).elf
LINKER_FLAGS = -Xlinker --gc-sections --specs=nano.specs --specs=nosys.specs -nostartfiles -Wl,-Map,"$(OUT_DIR)/$(PRJ_NAME).map" -o "$(ELF_NAME)"

######################### Individual Cfg Settings #########################
####### Release Cfg Settings #######
ifeq "$(GOAL_NAME)" "Release"
COMMON_FLAGS += -Os
LINKER_FLAGS +=
LD_SCRIPT = stlink.ld
# Comment / uncomment the following lines to produce .hex, .bin, .lst output
HEX_NAME = $(OUT_DIR)/$(PRJ_NAME).hex
# BIN_NAME = $(OUT_DIR)/$(PRJ_NAME).bin
# OUTPUT_ASM = -Wa,-adhlns="$@.lst"

####### Debug Cfg Settings #######
else ifeq "$(GOAL_NAME)" "Debug"
COMMON_FLAGS += -O0 -g3
LINKER_FLAGS +=
LD_SCRIPT = stlink.ld
# Comment / uncomment the following lines to produce .hex, .bin, .lst output
HEX_NAME = $(OUT_DIR)/$(PRJ_NAME).hex
# BIN_NAME = $(OUT_DIR)/$(PRJ_NAME).bin
# OUTPUT_ASM = -Wa,-adhlns="$@.lst"

####### FromBoot Cfg Settings #######
# Put to flash starting from 0x800XXXX, to reserve place for bootloader. All other is same as Release.
else ifeq "$(GOAL_NAME)" "Fromboot"
COMMON_FLAGS += -Os
LINKER_FLAGS +=
LD_SCRIPT = stlink_Boot.ld
# Comment / uncomment the following lines to produce .hex, .bin, .lst output
# HEX_NAME = $(OUT_DIR)/fw$(TIMESTAMP).hex
BIN_NAME = $(OUT_DIR)/fw$(TIMESTAMP).bin
# OUTPUT_ASM = -Wa,-adhlns="$@.lst"
endif

######################### Toolchain #########################
CPP_CMP = arm-none-eabi-g++
C_CMP = arm-none-eabi-gcc
OBJCPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SZ = arm-none-eabi-size
GDB = arm-none-eabi-gdb  # Required for flashing using BMP
# GDB COM port: required for flashing using BMP
GDB_COM = COM5

######################### Do not touch #########################
.PHONY: .FORCE print_size clean flash # "virtual" symbols to "rebuild" them always
# Current datetime
TIMESTAMP = $(shell "date" "+%Y%m%d_%H%M")
# Build include flag string out of INCLUDE_DIRS list
# Keep library subdirectories from shadowing standard headers such as <assert.h>.
INCLUDE_STR = -I. -I./include $(addprefix -iquote ./,$(INCLUDE_DIRS))
# Build define flag string out of DEFINS list, surrounding with double quotes
ToUppercase = $(shell echo $(1) | tr '[:lower:]' '[:upper:]')
# add BUILD_CFG_GOALNAME=1 define, uppercasing GOAL_NAME
DEFINS += BUILD_CFG_$(call ToUppercase,$(GOAL_NAME))=1
DEFINE_STR = $(patsubst %,-D"%",$(DEFINS))
COMMON_FLAGS += $(DEFINE_STR)
# Recursive wildcard to iterate subdirs of any depth
rwildcard = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
# Find all .cpp, .c and .S files in all subfolders of ../
SRCS = $(call rwildcard,./,*.c) $(call rwildcard,./,*.cpp) $(call rwildcard,./,*.S)
# Remove leading ./
SRCS := $(patsubst ./%,%,$(SRCS))
# These fragments are included by libopencm3/cm3/vector.c, not compiled separately.
SRCS := $(filter-out %/vector_chipset.c %/vector_nvic.c,$(SRCS))
# Replace .cpp, .c, .S with .o
OBJS = $(SRCS:.cpp=.o) # Take all srcs replacing .cpp with .o
OBJS := $(OBJS:.c=.o)  # Replace .c with .o
OBJS := $(OBJS:.S=.o)  # Replace .S with .o
# Find all source-containing folders; remove ending /; remove .
SUBDIRS = $(filter-out .,$(patsubst %/,%,$(sort $(dir $(SRCS)))))
# Search src here. Nothing to change here.
VPATH := ../

######################### Build #########################
# Add OUT_DIR prefix to OBJS
OBJS := $(addprefix $(OUT_DIR)/, $(OBJS))
# Include dependents *.d (ignore if not exist) to rebuild what depends on changed .h
-include $(OBJS:.o=.d)

# Build: Require dir tree, .elf file, .hex, .bin, .siz
build: $(OUT_DIR)/out_subdirs $(ELF_NAME) $(HEX_NAME) $(BIN_NAME) print_size

# Construct dir tree: out_dir/dir1, out_dir/dir2, ...
$(OUT_DIR)/out_subdirs:
	@mkdir -p $(addprefix $(OUT_DIR)/, $(SUBDIRS))

# Construct .elf
$(ELF_NAME): $(OBJS)
	@echo 'Linking $@'
	@$(CPP_CMP) $(COMMON_FLAGS) -T $(LD_SCRIPT) $(LINKER_FLAGS) $(OBJS)

### Compile objs ###
# Special rule for version.cpp: always rebuild
VERSION_DEFINS = -D"BUILD_TIME=$(TIMESTAMP)" -D"BUILD_CFG=$(call ToUppercase,$(GOAL_NAME))"
$(OUT_DIR)/version.o: version.cpp .FORCE
	@echo 'Building $<'
	@$(CPP_CMP) $(COMMON_FLAGS) $(VERSION_DEFINS) $(INCLUDE_STR) $(CPP_FLAGS) $(OBJ_FLAGS)
# cpp
$(OUT_DIR)/%.o: %.cpp
	@echo 'Building $<'
	@$(CPP_CMP) $(COMMON_FLAGS) $(INCLUDE_STR) $(CPP_FLAGS) $(OBJ_FLAGS)
# c
$(OUT_DIR)/%.o: %.c
	@echo 'Building $<'
	@$(C_CMP) $(COMMON_FLAGS) $(INCLUDE_STR) $(C_FLAGS) $(OBJ_FLAGS)
# S
$(OUT_DIR)/%.o: %.S
	@echo 'Building $<'
	@$(C_CMP) -x assembler-with-cpp $(COMMON_FLAGS) $(INCLUDE_STR) $(OBJ_FLAGS)

# Output .hex
$(HEX_NAME): $(ELF_NAME)
	@echo 'Constructing $@'
	@$(OBJCPY) -O ihex "$(ELF_NAME)" "$(HEX_NAME)"

# Output .bin
$(BIN_NAME): $(ELF_NAME)
	@echo 'Constructing $@'
	@$(OBJCPY) -O binary "$(ELF_NAME)" "$(BIN_NAME)"

# Output ASM (.lst)
# $(OUTPUT_ASM): $(ELF_NAME)
# 	@echo 'Generating asm $@'
# 	@$(OBJDUMP) -d

# Print size
print_size:
	@echo 'Size:'
	@$(SZ) --format=berkeley "$(ELF_NAME)"


######################### Clean #########################
clean:
	@rm -rf $(OUT_DIR)
	@echo 'Done.'


######################### Flash it ######################### Using BlackMagicProbe
flash:
	@$(GDB) -q -ex "target extended-remote $(GDB_COM)" -ex "mon swdp_scan" -ex "att 1" -ex "load $(HEX_NAME)" -ex "det" -ex "quit"


######################### Test for debugging ########################
.PHONY: test
test:
	@echo $(ACTION)
	@echo $(OUT_DIR)
