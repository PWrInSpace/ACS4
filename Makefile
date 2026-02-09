# =============================================================================
# ACS4 Flight Computer — Makefile Wrapper for CMake/Ninja
# =============================================================================

BUILD_DIR   ?= build
TARGET      ?= NUCLEO_H723
CMAKE_FLAGS ?=

.PHONY: build flash debug clean test rebuild

# ── Build ─────────────────────────────────────────────────────────────────
build:
	@cmake -B $(BUILD_DIR) -G Ninja -DACS4_TARGET=$(TARGET) $(CMAKE_FLAGS) \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	@cmake --build $(BUILD_DIR)
	@ln -sf $(BUILD_DIR)/compile_commands.json compile_commands.json

# ── Flash via ST-Link / OpenOCD ───────────────────────────────────────────
flash: build
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32h7x.cfg \
		-c "program $(BUILD_DIR)/acs4.elf verify reset exit"

# ── Debug: OpenOCD + GDB attach ──────────────────────────────────────────
debug: build
	@echo "Starting OpenOCD in background..."
	@openocd -f interface/stlink.cfg -f target/stm32h7x.cfg &
	@sleep 1
	arm-none-eabi-gdb $(BUILD_DIR)/acs4.elf \
		-ex "target extended-remote :3333" \
		-ex "monitor reset halt" \
		-ex "load"

# ── Clean ─────────────────────────────────────────────────────────────────
clean:
	rm -rf $(BUILD_DIR)

# ── Rebuild from scratch ─────────────────────────────────────────────────
rebuild: clean build

# ── Unit tests (native x86, future) ──────────────────────────────────────
test:
	@echo "TODO: native x86 test build not yet configured"
