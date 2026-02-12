# =============================================================================
# ACS4 Flight Computer — Makefile Wrapper for CMake/Ninja
# =============================================================================

BUILD_DIR   ?= build
TARGET      ?= NUCLEO_H723
CMAKE_FLAGS ?=

.PHONY: build flash debug clean test rebuild lint lint-all lint-fix

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

# ── Unit tests (native x86, Google Test) ─────────────────────────────────
test:
	@cmake -B build_test -S tests -G Ninja
	@cmake --build build_test
	@cd build_test && ctest --output-on-failure

# ── Static analysis ───────────────────────────────────────────────────────
# lint      — platform-independent code (x86 test build, no ARM toolchain needed)
# lint-all  — all application code (requires ARM build + toolchain)
# lint-fix  — auto-fix platform-independent code
#
# lint-all uses ARM compile_commands.json but runs host clang-tidy, so we
# inject the ARM GCC C++ include paths via --extra-arg=-isystem.

ARM_CXX_INCLUDES := $(shell arm-none-eabi-g++ -mcpu=cortex-m7 -mthumb \
	-mfloat-abi=hard -mfpu=fpv5-d16 -E -x c++ -v /dev/null 2>&1 \
	| sed -n '/search starts here/,/End of search/p' \
	| grep '^ ' | sed 's/^ *//')
EXTRA_ARGS := $(foreach d,$(ARM_CXX_INCLUDES),--extra-arg=-isystem$(d))

# Extract project src/ files from compile_commands.json (auto-tracks test build)
PROJECT_ROOT := $(abspath .)
TEST_SRCS = $$(python3 -c "import json,os; \
	root = '$(PROJECT_ROOT)/src/'; \
	entries = json.load(open('build_test/compile_commands.json')); \
	print(' '.join(e['file'] for e in entries if e['file'].startswith(root)))")

lint:
	@cmake -B build_test -S tests -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	@cmake --build build_test
	@echo "Running clang-tidy (test build sources)..."
	@clang-tidy -p build_test \
		$(TEST_SRCS) \
		2>&1 | tee build_test/clang-tidy.log
	@echo "Results saved to build_test/clang-tidy.log"

lint-all: build
	@echo "Running clang-tidy (all src/)..."
	@clang-tidy -p $(BUILD_DIR) $(EXTRA_ARGS) \
		--extra-arg=-D__ARM_ARCH=7 \
		$$(find src -name '*.cpp') \
		2>&1 | tee $(BUILD_DIR)/clang-tidy.log
	@echo "Results saved to $(BUILD_DIR)/clang-tidy.log"

lint-fix:
	@cmake -B build_test -S tests -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	@cmake --build build_test
	@clang-tidy -p build_test --fix --fix-errors \
		$(TEST_SRCS)
