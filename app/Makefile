BUILD_DIR = build
TARGET = app
PREFIX = arm-none-eabi-
CC = $(GCC_PATH)$(PREFIX)gcc
AS = $(GCC_PATH)$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)$(PREFIX)objcopy
SZ = $(GCC_PATH)$(PREFIX)size

CPU = -mcpu=cortex-m0plus
FPU =
FLOAT-ABI = -mfloat-abi=soft
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
OPT = -O0 -g3 -gdwarf-5

CFLAGS = $(MCU) $(OPT) -Wall -Wextra -fdata-sections -ffunction-sections

LDSCRIPT = pico.ld
LIBS = -nostartfiles
LIBDIR = 
LDFLAGS = $(MCU) -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

ASM_SOURCES = \
startup_pico.S \
crt0.S

C_SOURCES = \
picosystem.c \
main.c

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(shell test -d $(BUILD_DIR) || mkdir $(BUILD_DIR))

all: $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/%.o: %.S FORCE
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.c FORCE
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@
	$(GCC_PATH)$(PREFIX)objdump $(BUILD_DIR)/$(TARGET).elf -D > $(BUILD_DIR)/myD.s

.PHONY: clean
clean:
	-rm -fR $(BUILD_DIR)

.PHONY: FORCE
FORCE:
