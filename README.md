# ACS4 — Attitude Control System

Flight computer firmware for a **PWrInSpace** rocket active stabilization system using **4 canard fins**.

| | Target |
|---|---|
| **Production** | Custom PCB — STM32H725VGT6 |
| **Development** | NUCLEO-144 — STM32H723ZGT6 |

---

## Hardware Overview

ACS4 controls the rocket's roll, pitch, and yaw via 4 independently actuated canard fins mounted near the nosecone.

> *Sensor suite, servo interfaces, and detailed pinout — TBD.*

---

## Software Architecture

The firmware is built upon an embedded framework designed for high-performance control loops.

*   **RTOS:** ChibiOS/RT 21.11
*   **Language:** C++17 (no exceptions, no RTTI)
*   **Build System:** CMake + Ninja (managed via Makefile wrapper)
*   **Math:** Eigen 3 (header-only, single-precision float)

**Key modules:**

| Directory | Description |
|---|---|
| `src/navigation/` | Quaternion algebra (Hamilton, ZYX Euler, body→NED) — Eigen-backed |
| `src/system/` | Shell, error handler, runtime params, watchdog |
| `src/utils/` | DWT timestamp, cycle-accurate profiler |
| `cfg/` | Per-board ChibiOS config (`chconf.h`, `halconf.h`, `mcuconf.h`) |
| `tests/` | Unit tests (Google Test, native x86 build) |

**Runtime features:**
*   **Interactive Shell** — CLI on USB CDC (custom PCB) or UART3 (Nucleo) for debugging.
*   **Runtime Parameters** — Modify PID gains in-flight via `param set`.
*   **Profiler** — Cycle-accurate execution timing (`perf` command).
*   **Error Handling** — Centralized fault counters with watchdog protection.

---

## Prerequisites

| Tool | Minimum version | Install (Ubuntu/Debian) |
|---|---|---|
| `arm-none-eabi-gcc` | ≥ 12 | `sudo apt install gcc-arm-none-eabi` |
| `cmake` | ≥ 3.20 | `sudo apt install cmake` |
| `ninja-build` | any | `sudo apt install ninja-build` |
| `openocd` | any | `sudo apt install openocd` |
| `clang-tidy` | ≥ 18 | `sudo apt install clang-tidy-18` |

**For unit tests only:** a host C++17 compiler (`g++` or `clang++`). Google Test is fetched automatically by CMake.

---

## Getting Started

```bash
# 1. Clone with submodules (ChibiOS + Eigen)
git clone --recurse-submodules https://github.com/PWrInSpace/ACS4.git
cd ACS4

# If already cloned without submodules:
git submodule update --init --recursive
```

---

## Build & Flash

### Build

```bash
# Default: NUCLEO-H723ZG (dev board)
make build

# Production PCB (STM32H725)
make build TARGET=CUSTOM_H725
```

### Flash

Connect the board via ST-Link (USB):

```bash
make flash
```

### Debug (GDB + OpenOCD)

```bash
make debug
```

This starts OpenOCD in the background, launches `arm-none-eabi-gdb`, connects to the target, and loads the ELF.

### Clean / Rebuild

```bash
make clean          # Remove build directory
make rebuild        # Clean + build from scratch
```

---

## Unit Tests

Platform-independent code (quaternion library, etc.) is tested natively on x86 using Google Test:

```bash
make test
```

This configures a separate `build_test/` directory, compiles against the host compiler, and runs `ctest`:

```
100% tests passed, 0 tests failed out of 41
```

Test sources live in `tests/unit/`. Google Test v1.14.0 is fetched automatically at configure time.

---

## Serial Shell

```bash
# Connect to Serial Console (Exit: Ctrl+A → K → Y)
screen /dev/ttyACM0 921600
```

**Commands:**

| Command | Description |
|---|---|
| `version` | Firmware version, build date, git hash |
| `uptime` | Time since boot |
| `threads` | Active tasks and stack usage |
| `param list` | Show all tunable parameters |
| `param set <name> <val>` | Change a parameter at runtime |
| `param get <name>` | Read a parameter value |
| `param defaults` | Reset all parameters to defaults |
| `perf` | Execution time statistics |
| `errors` | System error counters |
| `reboot` | Software reset |

---

## Static Analysis (clang-tidy)

The project uses **clang-tidy 18** for static analysis. Configuration lives in `.clang-tidy` and is tuned for embedded C++17 with ChibiOS + Eigen (suppresses false positives from RTOS macros and cross-compilation).

### Targets

| Command | Scope | Requires ARM toolchain? |
|---|---|---|
| `make lint` | Platform-independent `src/` code compiled in the test build | No |
| `make lint-all` | All `src/*.cpp` (full firmware, ARM compile DB) | Yes |
| `make lint-fix` | Same as `lint`, with `--fix` auto-applied | No |

### Usage

```bash
# Quick check — only modules included in the x86 test build
make lint

# Full check — all application code against the ARM build
make lint-all

# Auto-fix what clang-tidy can fix automatically
make lint-fix
```

Results are saved to `build_test/clang-tidy.log` (lint/lint-fix) or `build/clang-tidy.log` (lint-all).

> **Note:** `make lint` / `make lint-fix` automatically discover which `src/` files the test CMake compiles — no hardcoded paths. When you add a new module to `tests/CMakeLists.txt`, it gets linted automatically.

> **Note:** `make lint-all` injects ARM GCC system include paths so that host clang-tidy can parse ChibiOS headers. This requires `arm-none-eabi-g++` on `$PATH`.

---

## Code Formatting (clang-format)

The project uses **clang-format**.

### Usage

```bash
# Format a single file
clang-format -i src/navigation/quaternion.cpp

# Format all project sources
find src tests -name '*.cpp' -o -name '*.h' -o -name '*.c' | xargs clang-format -i

# Check formatting without modifying (CI-friendly)
find src tests -name '*.cpp' -o -name '*.h' -o -name '*.c' | xargs clang-format --dry-run --Werror
```

> **Note:** Do NOT run clang-format on `cfg/` or `lib/` — those are external (ChibiOS config / Eigen).
