# VAMP_project

This project was specifically written to run on a **Tinker V** board (RISC-V architecture).
It performs motion planning using VAMP and communicates via GPIO with an Arduino.

## Requirements

- A Tinker V board (or any RISC-V architecture)
- An Arduino (to receive GPIO signals)
- A Linux host machine for cross-compiling
- [`nds64le-linux-glibc-v5d`](https://github.com/andestech/nds-gnu-toolchain) for compiling VAMP code

## Compilation Guide

### 1. Compile `gpio_sender.c` for RISC-V 

To compile `gpio_sender.c` directly on your **Tinker V** (or another RISC-V device), run: 

```bash
gcc gpio_sender.c -o gpio_sender -lgpiod
```

This will produce the `gpio_sender` binary, which can be executed on your RISC-V device.

### 2. Compile `vamp.c` and `main.c`

On your **host machine**, compile the main VAMP application:

```bash
make
```
This will generate the required executables using the specified RISC-V cross-compiler.

⚠️ **Important** : The toolchain folder must be placed at the same directory level as this project.

### 3. Upload the Arduino code

Flash the file `receive.ino` onto your Arduino board.
This program listens for GPIO signals sent by the Tinker V and triggers corresponding actions (e.g., movements).

### 4. Deploy the Executables

After successful compilation, copy the following files to your Tinker V board (or RISC-V device):
- `main`
- `vamp`
- `gpio_sender`
- `input/`
- `output/`

### 5. Run the Program on Tinker V

Once the executables are transferred:

1. Ensure the Arduino board is connected and powered.
2. Make sure the `input/` files are in the correct location.
3. On the Tinker V board, run the main executable:

```bash
./main
```
This will:

- Run the VAMP motion planner
- Send the resulting motion commands through GPIO
- Trigger actions on the connected Arduino in real time



