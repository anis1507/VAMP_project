# Cross-compiler for RISC-V architecture
CC = ../riscv-gnu-toolchain/nds64le-linux-glibc-v5d/nds64le-linux-glibc-v5d/bin/riscv64-linux-gcc

# Compiler flags:
# -Wall: show all warnings
# -std=c11: use C11 standard
# -Iinclude: include header files from 'include' directory
# -mext-dsp: enable DSP extension
# -static: produce a statically linked binary
CFLAGS = -Wall -std=c11 -Iinclude -mext-dsp -static

# Linker flags
LDFLAGS = -Tldscript -lm

# Source and object directories
SRC_DIR = src
OBJ_DIR = obj

# Sources for vamp executable (exclude main.c and gpio_sender.c)
VAMP_SRC = $(filter-out $(SRC_DIR)/main.c $(SRC_DIR)/gpio_sender.c, $(wildcard $(SRC_DIR)/*.c))
VAMP_OBJ = $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(VAMP_SRC))

# Source for main executable (only main.c)
MAIN_SRC = $(SRC_DIR)/main.c
MAIN_OBJ = $(OBJ_DIR)/main.o

# Required input files for runtime
INPUT_FILES = input/arm_description.txt input/start_end.txt input/obstacles.txt

# Default rule: build everything
all: vamp main inputs

# Link vamp executable
vamp: $(VAMP_OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Link main executable
main: $(MAIN_OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile source files into object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

# Check input files presence
inputs: $(INPUT_FILES)
	@echo "All required input files are present."

# Include dependency files
-include $(OBJ_DIR)/*.d

# Clean build artifacts
clean:
	rm -rf $(OBJ_DIR) vamp main
