# ACS4 - Attitude Control System

Flight computer firmware for a PWrInSpace rocket active stabilization system using canard fins.
**Production Target:** Custom PCB (STM32H743)
**Development Target:** NUCLEO-H723ZG

---

## Software Architecture

The firmware is built upon a embedded framework designed for high-performance control loops.

*   **RTOS:** ChibiOS/RT 21.11
*   **Language:** C++17 Embedded
*   **Build System:** CMake + Ninja (managed via Makefile wrapper)
*   **Key Features:**
    *   **Interactive Shell:** CLI available on UART3 (921600 baud) for runtime debugging.
    *   **Runtime Parameters:** Modify control gains (PID) and settings in-flight via `param set`.
    *   **Profiler:** Cycle-accurate execution timing (`perf` command) based on DWT hardware.
    *   **Error Handling:** Centralized fault reporting (`errors` command) with Watchdog protection.

---

## Build & Flash

Prerequisites: `arm-none-eabi-gcc`, `cmake`, `ninja-build`, `openocd`.

### 1. Build

Use the provided `make` wrapper for convenience:

```bash
# Default: Build for NUCLEO-H723ZG
make build

# Build for Custom PCB (STM32H743)
make build TARGET=CUSTOM_H743
```

### 2. Flash

Connect the board via ST-Link (USB):

```bash
make flash
```

### 3. Debug / Terminal

To access the system shell:

```bash
# Connect to Serial Console (Exit with: Ctrl+A, then K, then Y)
screen /dev/ttyACM0 921600
```

**Shell Commands:**
*   `threads` - List active tasks and stack usage.
*   `param list` - Show tunable parameters.
*   `param set <name> <val>` - Change a parameter.
*   `perf` - Show execution time stats.
*   `errors` - Show system error counters.
*   `reboot` - Reset the board.
