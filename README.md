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


