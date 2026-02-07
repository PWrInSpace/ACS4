# ACS4 - Attitude Control System README

Flight computer firmware for a PWrInSpace rocket active stabilization system using canard fins, targeting the **STM32H743** 

Development is done on a **NUCLEO-H723ZG** dev board (same Cortex-M7 family, pin-compatible).


## Getting Started

### Prerequisites

**OS:** Linux or WSL2 on Windows.

Install the toolchain (Ubuntu/Debian):

```bash
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib

sudo apt install openocd stlink-tools

sudo apt install make cmake
```

### Clone & Build

```bash
git clone --recursive https://github.com/PWrInSpace/ACS4.git
cd ACS4
cmake -B build                        # defaults to NUCLEO_H723 (dev board)
cmake --build build -j$(nproc)
```

#### Target Selection

The build supports two targets via the `ACS4_TARGET` CMake option:

| Value          | Board                         | MCU            |
|----------------|-------------------------------|----------------|
| `NUCLEO_H723`  | NUCLEO-144 dev board (default)| STM32H723ZGT6  |
| `CUSTOM_H743`  | ACS4 production PCB           | STM32H743VIT6  |

```bash
# Dev board (default)
cmake -B build -DACS4_TARGET=NUCLEO_H723

# Production PCB
cmake -B build -DACS4_TARGET=CUSTOM_H743
```

> **Note:** When switching targets, delete the `build/` directory first (`rm -rf build`)
> because CMake caches the target selection.

Each target has its own ChibiOS configuration in `cfg/<target>/` and board BSP.

Output: `build/acs4.elf`, `build/acs4.bin`, `build/acs4.hex`.

> If you cloned without `--recursive`, fetch the ChibiOS submodule manually:
> ```bash
> git submodule update --init --recursive
> ```

### Flash

Connect the board via USB, then:

```bash
cmake --build build --target flash
```

If you get a permission error, add udev rules:
```bash
sudo cp /usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```


### Verify

Connect to the serial port at 115200 baud. On Linux/WSL:

```bash
# Find the port
ls /dev/ttyACM*

# Read output
stty -F /dev/ttyACM0 115200 raw -echo && cat /dev/ttyACM0
```


### Code Style

The project enforces a strict `.clang-format`.

Install clang-format:
```bash
sudo apt install clang-format
```

Format all source files:
```bash
clang-format -i src/**/*.c src/**/*.h
```

Check without modifying:
```bash
clang-format --dry-run --Werror src/**/*.c src/**/*.h
```

> **Rule:** All code pushed to `main` must pass `clang-format --Werror`. Format before committing.