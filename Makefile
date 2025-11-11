# Variables
BUILD_DIR = Build
TARGET = Controller
AXF = $(TARGET).axf
HEX = $(TARGET).hex

# Compiler settings (example, adjust as needed)
CC = arm-none-eabi-gcc
CFLAGS = -mcpu=cortex-m4 -mthumb -O2 -Wall -std=c11
LDFLAGS = -T$(TARGET).ld

SRCS = Src/main.c Src/uart.c
OBJS = $(SRCS:.c=.o)

# Default target
all: $(TARGET).elf $(HEX)

# Build ELF
$(TARGET).elf: $(OBJS)
	@echo " Linking $@..."
	@$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@

# Build HEX from ELF
$(HEX): $(TARGET).elf
	@echo " Creating HEX file $@..."
	@arm-none-eabi-objcopy -O ihex $< $@

# Compile C source files
%.o: %.c
	@echo " Compiling $<..."
	@$(CC) $(CFLAGS) -c $< -o $@

# Clean target
clean:
	@echo " Cleaning build files..."
	@mkdir -p $(BUILD_DIR)
	@mv -f $(AXF) $(HEX) $(BUILD_DIR)/ 2>/dev/null || true
	@rm -f *.htm *.lnp *.map *.sct *.d *.o
	@echo "Moved .axf and .hex to $(BUILD_DIR), removed object, dependency, and intermediate files."

.PHONY: all clean
