# VAMP_project

This project was written specifically to run on a **Tinker V** board (RISC-V architecture).

## Requirements

- A Tinker V board (or any RISC-V architecture)
- An Arduino (for receive GPIO signals)
- A Linuxhost machine for cross-compling
- [`nds64le-linux-glibc-v5d`](https://github.com/andestech/nds-gnu-toolchain) for compling VAMP code

## Excepted Project Structure

After cloning ans setup, the project directory should like this :

VAMP_project/
├── input/

├── output/

├── src/

│ ├── main.c

│ ├── vamp.c

│ └── Makefile

├── gpio_sender/ ← contains the compiled gpio_sender executable for RISC-V

├── arduino/

│ └── receive.ino

├── nds-gnu-toolchain/ ← required for cross-compiling for the RISC-V
