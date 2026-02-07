# ACS4 - Attitude Control System README

Flight computer firmware for a PWrInSpace rocket active stabilization system using canard fins, targeting the **STM32H743** on a custom 4-layer PCB.

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
make -j$(nproc)
```

Output: `build/acs4.elf`, `build/acs4.bin`, `build/acs4.hex`.

> If you cloned without `--recursive`, fetch the ChibiOS submodule manually:
> ```bash
> git submodule update --init --recursive
> ```

### Flash

#### Option A: OpenOCD + ST-Link (native Linux)

Connect the NUCLEO-H723ZG via USB, then:

```bash
make flash
```

This runs:
```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program build/acs4.elf verify reset exit"
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