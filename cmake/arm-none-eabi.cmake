# =============================================================================
# ACS4 Flight Computer — ARM Cortex-M7 cross-compilation toolchain
# Usage: cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
# =============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain prefix
set(TOOLCHAIN_PREFIX arm-none-eabi-)

# Compilers
set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP      ${TOOLCHAIN_PREFIX}objdump)
set(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}size)

# Cortex-M7 with hardware double-precision FPU
set(CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16")

set(CMAKE_C_FLAGS_INIT   "${CPU_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${CPU_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS}")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${CPU_FLAGS} -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs -lm")

# Skip compiler test (cross-compiling, no OS on target)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Search paths — never search host
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
