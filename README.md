# VAMP_project

This project was written specifically to run on a **Tinker V** board (RISC-V architecture).

## Requirements

- A Tinker V board (or any RISC-V architecture)
- An Arduino (for receive GPIO signals)
- A Linuxhost machine for cross-compling
- [`nds64le-linux-glibc-v5d`](https://github.com/andestech/nds-gnu-toolchain) for compling VAMP code

## Compilation Guide

### 1. Compile `gpio_sender.c` for RISC-V 

To compile `gpio_sender.c` for RISC-V : 
```bash
gcc gpio_sender.c -o gpio_sender -lgpiod
```

This product the binary `gpio_sender`, which can be executed on your RISC-V device.

### 2. Compile `vamp.c` and `main.c`

To compile the main VAMP application (`main.c`, `vamp.c`), simply run :

```bash
make
```
This will generate the required executables using the specified RISC-V cross-compiler.

⚠️ **Important:** Make sure the `Makefile` is configured to use the correct binaries from the `nds64le-linux-glibc-v5d` toolchain (e.g., `nds64le-linux-glibc-v5d-gcc`, etc.).

### 3. Upload the Arduino code

Flash the file `receive.ino` onto your Arduino board.
This program listens for GPIO signals sent by the Tinker V and triggers corrspondig actions (movements).

### 4. Deploy the Executables

After successful compilation, copy the following files to your Tinker V board (or RISC-V device) :
- `main`
- `vamp`
- `gpio_sender`
- `input/`
- `output/`

### 5. Run the Program on Tinker V

Once the executables are transferred :

1. Ensure the Arduino board is connected and powered.
2. Make sure the `input/` files are in the correct location.
3. On the Tinker V board, run the main executable :

```bash
./main
```
This will :

- Run the VAMP motion planner
- Send the resulting motion commands through GPIO
- Trigger actions on the connected Arduino in real time



