CC = ../../../riscv-gnu-toolchain/nds64le-linux-glibc-v5d/nds64le-linux-glibc-v5d/bin/riscv64-linux-gcc
CFLAGS = -Wall -std=c11 -Iinclude -mext-dsp -static
LDFLAGS = -T../../../ldscript -lm
TARGET = vamp
SRC_DIR = src
OBJ_DIR = obj

SRC = $(filter-out $(SRC_DIR)/gpio_sender.c, $(wildcard $(SRC_DIR)/*.c))
OBJ = $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRC))
DEP = $(OBJ:.o=.d)

INPUT_FILES = input/arm_description.txt input/start_end.txt input/obstacles.txt

all: $(TARGET) inputs

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

inputs: $(INPUT_FILES)
	@echo "✅ Input files are present."

-include $(DEP)

clean:
	rm -rf $(OBJ_DIR) $(TARGET)
