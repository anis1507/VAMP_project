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

# Target executable
TARGET = vamp

# Source and object directories
SRC_DIR = src
OBJ_DIR = obj

# Source files, excluding gpio_sender.c (not compatible with this architecture)
SRC = $(filter-out $(SRC_DIR)/gpio_sender.c, $(wildcard $(SRC_DIR)/*.c))

# Object files
OBJ = $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRC))

# Dependency files
DEP = $(OBJ:.o=.d)

# Required input files for runtime
INPUT_FILES = input/arm_description.txt input/start_end.txt input/obstacles.txt

# Default rule: build everything
all: $(TARGET) inputs

# Linking the final executable
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compiling C source files to object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

# Check that input files are present
inputs: $(INPUT_FILES)
	@echo "All required input files are present."

# Include dependency files for incremental builds
-include $(DEP)

# Clean build artifacts
clean:
	rm -rf $(OBJ_DIR) $(TARGET)
