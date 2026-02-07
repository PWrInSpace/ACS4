# =============================================================================
# ACS4 Flight Computer — CMake Build System
# Target: STM32H723ZG (dev) / STM32H743VIT6 (prod)
# RTOS:   ChibiOS/RT
# =============================================================================
cmake_minimum_required(VERSION 3.20)

# ── Toolchain (must be set before project()) ──────────────────────────────
set(CMAKE_TOOLCHAIN_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake
    CACHE STRING "Toolchain file")

project(acs4
    VERSION   0.1.0
    LANGUAGES C ASM
)

# ── Paths ─────────────────────────────────────────────────────────────────
set(CHIBIOS ${CMAKE_CURRENT_SOURCE_DIR}/lib/chibios)

# ── Target board selection ────────────────────────────────────────────────
# NUCLEO_H723 = NUCLEO-144 dev board (STM32H723ZGT6)
# CUSTOM_H743 = Production PCB       (STM32H743VIT6)
set(ACS4_TARGET "NUCLEO_H723" CACHE STRING "Target board: NUCLEO_H723 | CUSTOM_H743")
set_property(CACHE ACS4_TARGET PROPERTY STRINGS NUCLEO_H723 CUSTOM_H743)

if(ACS4_TARGET STREQUAL "NUCLEO_H723")
    message(STATUS "Target: NUCLEO-144 H723ZG (dev)")
    set(LINKER_SCRIPT
        ${CHIBIOS}/os/common/startup/ARMCMx/compilers/GCC/ld/STM32H723xG_ITCM64k.ld)
    set(BOARD_DIR  ${CHIBIOS}/os/hal/boards/ST_NUCLEO144_H723ZG)
    set(CONFDIR    ${CMAKE_CURRENT_SOURCE_DIR}/cfg/nucleo_h723)
    set(MCU_DEFINE STM32H723xx)
elseif(ACS4_TARGET STREQUAL "CUSTOM_H743")
    message(STATUS "Target: ACS4 custom PCB H743VI (prod)")
    set(LINKER_SCRIPT
        ${CHIBIOS}/os/common/startup/ARMCMx/compilers/GCC/ld/STM32H743xI.ld)
    set(BOARD_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/cfg/boards/ACS4_H743)
    set(CONFDIR    ${CMAKE_CURRENT_SOURCE_DIR}/cfg/custom_h743)
    set(MCU_DEFINE STM32H743xx)
else()
    message(FATAL_ERROR "Unknown ACS4_TARGET '${ACS4_TARGET}'. Use NUCLEO_H723 or CUSTOM_H743.")
endif()

# =============================================================================
# ChibiOS Sources
# Collected from ChibiOS .mk files — keep in sync when enabling new HAL drivers
# =============================================================================

# ── Startup (ASM + C) ────────────────────────────────────────────────────
set(CHIBIOS_STARTUP_ASM
    ${CHIBIOS}/os/common/startup/ARMCMx/compilers/GCC/crt0_v7m.S
    ${CHIBIOS}/os/common/startup/ARMCMx/compilers/GCC/vectors.S
)

set(CHIBIOS_STARTUP_C
    ${CHIBIOS}/os/common/startup/ARMCMx/compilers/GCC/crt1.c
)

# ── Port (ARMv7-M) ───────────────────────────────────────────────────────
set(CHIBIOS_PORT_ASM
    ${CHIBIOS}/os/common/ports/ARMv7-M/compilers/GCC/chcoreasm.S
)

set(CHIBIOS_PORT_C
    ${CHIBIOS}/os/common/ports/ARMv7-M/chcore.c
)

# ── OOP ───────────────────────────────────────────────────────────────────
set(CHIBIOS_OOP_C
    ${CHIBIOS}/os/common/oop/src/oop_base_object.c
)

# ── HAL core ──────────────────────────────────────────────────────────────
set(CHIBIOS_HAL_C
    ${CHIBIOS}/os/hal/src/hal.c
    ${CHIBIOS}/os/hal/src/hal_safety.c
    ${CHIBIOS}/os/hal/src/hal_buffered_serial.c
    ${CHIBIOS}/os/hal/src/hal_buffers.c
    ${CHIBIOS}/os/hal/src/hal_queues.c
    ${CHIBIOS}/os/hal/src/hal_mmcsd.c
    ${CHIBIOS}/os/hal/src/hal_pal.c
    ${CHIBIOS}/os/hal/src/hal_serial.c
    ${CHIBIOS}/os/hal/src/hal_flash.c
    ${CHIBIOS}/os/hal/src/hal_st.c
)

# ── HAL platform (STM32H7xx LLD) ─────────────────────────────────────────
set(CHIBIOS_HAL_PLATFORM_C
    ${CHIBIOS}/os/hal/ports/STM32/STM32H7xx/hal_lld.c
    ${CHIBIOS}/os/hal/ports/STM32/STM32H7xx/stm32_isr.c
    ${CHIBIOS}/os/hal/ports/common/ARMCMx/nvic.c
    ${CHIBIOS}/os/hal/ports/STM32/LLD/BDMAv1/stm32_bdma.c
    ${CHIBIOS}/os/hal/ports/STM32/LLD/DMAv2/stm32_dma.c
    ${CHIBIOS}/os/hal/ports/STM32/LLD/EXTIv1/stm32_exti.c
    ${CHIBIOS}/os/hal/ports/STM32/LLD/GPIOv2/hal_pal_lld.c
    ${CHIBIOS}/os/hal/ports/STM32/LLD/MDMAv1/stm32_mdma.c
    ${CHIBIOS}/os/hal/ports/STM32/LLD/SYSTICKv1/hal_st_lld.c
    ${CHIBIOS}/os/hal/ports/STM32/LLD/USARTv3/hal_serial_lld.c
)

# ── HAL board (selected by ACS4_TARGET) ──────────────────────────────────
set(CHIBIOS_BOARD_C
    ${BOARD_DIR}/board.c
)

# ── HAL OSAL ──────────────────────────────────────────────────────────────
set(CHIBIOS_OSAL_C
    ${CHIBIOS}/os/hal/osal/rt-nil/osal.c
)

# ── HAL streams (chprintf) ───────────────────────────────────────────────
set(CHIBIOS_STREAMS_C
    ${CHIBIOS}/os/hal/lib/streams/chprintf.c
    ${CHIBIOS}/os/hal/lib/streams/chscanf.c
    ${CHIBIOS}/os/hal/lib/streams/memstreams.c
    ${CHIBIOS}/os/hal/lib/streams/nullstreams.c
    ${CHIBIOS}/os/hal/lib/streams/bufstreams.c
)

# ── RT kernel ─────────────────────────────────────────────────────────────
set(CHIBIOS_RT_C
    ${CHIBIOS}/os/rt/src/chsys.c
    ${CHIBIOS}/os/rt/src/chsafety.c
    ${CHIBIOS}/os/rt/src/chrfcu.c
    ${CHIBIOS}/os/rt/src/chdebug.c
    ${CHIBIOS}/os/rt/src/chtrace.c
    ${CHIBIOS}/os/rt/src/chvt.c
    ${CHIBIOS}/os/rt/src/chschd.c
    ${CHIBIOS}/os/rt/src/chinstances.c
    ${CHIBIOS}/os/rt/src/chtm.c
    ${CHIBIOS}/os/rt/src/chthreads.c
    ${CHIBIOS}/os/rt/src/chregistry.c
    ${CHIBIOS}/os/rt/src/chsem.c
    ${CHIBIOS}/os/rt/src/chmtx.c
    ${CHIBIOS}/os/rt/src/chcond.c
    ${CHIBIOS}/os/rt/src/chevents.c
    ${CHIBIOS}/os/rt/src/chmsg.c
    ${CHIBIOS}/os/rt/src/chdynamic.c
)

# ── OS library ────────────────────────────────────────────────────────────
set(CHIBIOS_OSLIB_C
    ${CHIBIOS}/os/oslib/src/chmemchecks.c
    ${CHIBIOS}/os/oslib/src/chmboxes.c
    ${CHIBIOS}/os/oslib/src/chmemheaps.c
    ${CHIBIOS}/os/oslib/src/chmemcore.c
    ${CHIBIOS}/os/oslib/src/chmempools.c
    ${CHIBIOS}/os/oslib/src/chpipes.c
    ${CHIBIOS}/os/oslib/src/chobjcaches.c
    ${CHIBIOS}/os/oslib/src/chdelegates.c
)

# =============================================================================
# Application Sources
# =============================================================================
set(APP_SOURCES
    src/main.c
)

# =============================================================================
# Include Directories
# =============================================================================
set(CHIBIOS_INCLUDE_DIRS
    # Project config
    ${CONFDIR}

    # CMSIS
    ${CHIBIOS}/os/common/ext/ARM/CMSIS/Core/Include
    ${CHIBIOS}/os/common/ext/ST/STM32H7xx

    # ChibiOS internals
    ${CHIBIOS}/os/common/oop/include
    ${CHIBIOS}/os/common/portability/GCC
    ${CHIBIOS}/os/common/ports/ARM-common/include
    ${CHIBIOS}/os/common/ports/ARMv7-M
    ${CHIBIOS}/os/common/startup/ARMCMx/compilers/GCC
    ${CHIBIOS}/os/common/startup/ARMCMx/devices/STM32H7xx
    ${CHIBIOS}/os/license

    # HAL
    ${CHIBIOS}/os/hal/include
    ${BOARD_DIR}
    ${CHIBIOS}/os/hal/osal/rt-nil
    ${CHIBIOS}/os/hal/ports/common/ARMCMx
    ${CHIBIOS}/os/hal/ports/STM32/STM32H7xx
    ${CHIBIOS}/os/hal/ports/STM32/LLD/ADCv4
    ${CHIBIOS}/os/hal/ports/STM32/LLD/BDMAv1
    ${CHIBIOS}/os/hal/ports/STM32/LLD/CRYPv1
    ${CHIBIOS}/os/hal/ports/STM32/LLD/DACv1
    ${CHIBIOS}/os/hal/ports/STM32/LLD/DMAv2
    ${CHIBIOS}/os/hal/ports/STM32/LLD/EXTIv1
    ${CHIBIOS}/os/hal/ports/STM32/LLD/FDCANv2
    ${CHIBIOS}/os/hal/ports/STM32/LLD/GPIOv2
    ${CHIBIOS}/os/hal/ports/STM32/LLD/I2Cv3
    ${CHIBIOS}/os/hal/ports/STM32/LLD/MACv2
    ${CHIBIOS}/os/hal/ports/STM32/LLD/MDMAv1
    ${CHIBIOS}/os/hal/ports/STM32/LLD/OCTOSPIv2
    ${CHIBIOS}/os/hal/ports/STM32/LLD/OTGv1
    ${CHIBIOS}/os/hal/ports/STM32/LLD/QUADSPIv2
    ${CHIBIOS}/os/hal/ports/STM32/LLD/RNGv1
    ${CHIBIOS}/os/hal/ports/STM32/LLD/RTCv2
    ${CHIBIOS}/os/hal/ports/STM32/LLD/SDMMCv2
    ${CHIBIOS}/os/hal/ports/STM32/LLD/SPIv3
    ${CHIBIOS}/os/hal/ports/STM32/LLD/SYSTICKv1
    ${CHIBIOS}/os/hal/ports/STM32/LLD/TIMv1
    ${CHIBIOS}/os/hal/ports/STM32/LLD/USART
    ${CHIBIOS}/os/hal/ports/STM32/LLD/USARTv3
    ${CHIBIOS}/os/hal/ports/STM32/LLD/xWDGv1

    # Streams (chprintf)
    ${CHIBIOS}/os/hal/lib/streams

    # RT kernel
    ${CHIBIOS}/os/rt/include

    # OS library
    ${CHIBIOS}/os/oslib/include

    # Application
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# =============================================================================
# Target: acs4.elf
# =============================================================================
add_executable(${PROJECT_NAME}
    # ASM
    ${CHIBIOS_STARTUP_ASM}
    ${CHIBIOS_PORT_ASM}
    # C — ChibiOS
    ${CHIBIOS_STARTUP_C}
    ${CHIBIOS_PORT_C}
    ${CHIBIOS_OOP_C}
    ${CHIBIOS_HAL_C}
    ${CHIBIOS_HAL_PLATFORM_C}
    ${CHIBIOS_BOARD_C}
    ${CHIBIOS_OSAL_C}
    ${CHIBIOS_STREAMS_C}
    ${CHIBIOS_RT_C}
    ${CHIBIOS_OSLIB_C}
    # C — Application
    ${APP_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CHIBIOS_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE CORTEX_USE_FPU=TRUE)

# ── Compiler flags ────────────────────────────────────────────────────────
target_compile_options(${PROJECT_NAME} PRIVATE
    -O2
    -ggdb
    -fomit-frame-pointer
    -falign-functions=16
    -ffunction-sections
    -fdata-sections
    -fno-common
    -flto=auto
    -Wall -Wextra -Wundef -Wstrict-prototypes
)

# ── ASM flags (suppress C warnings for .S files) ─────────────────────────
set_source_files_properties(
    ${CHIBIOS_STARTUP_ASM} ${CHIBIOS_PORT_ASM}
    PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp"
)

# ── Linker ────────────────────────────────────────────────────────────────
set(LINKER_SCRIPT_DIR
    ${CHIBIOS}/os/common/startup/ARMCMx/compilers/GCC/ld)

target_link_options(${PROJECT_NAME} PRIVATE
    -T${LINKER_SCRIPT}
    -L${LINKER_SCRIPT_DIR}
    -Wl,--defsym=__process_stack_size__=0x800
    -Wl,--defsym=__main_stack_size__=0x400
    -Wl,--gc-sections
    -Wl,--no-warn-rwx-segments
    -flto=auto
    -nostartfiles
)

# ── Output suffix ─────────────────────────────────────────────────────────
set_target_properties(${PROJECT_NAME} PROPERTIES
    SUFFIX ".elf"
    C_STANDARD 17
    C_STANDARD_REQUIRED ON
)

# =============================================================================
# Post-build: Generate .bin, .hex, size report
# =============================================================================
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating .bin, .hex and size report"
)

# =============================================================================
# Custom targets
# =============================================================================

# ── Flash via OpenOCD ─────────────────────────────────────────────────────
add_custom_target(flash
    COMMAND openocd
        -f interface/stlink.cfg
        -f target/stm32h7x.cfg
        -c "program $<TARGET_FILE:${PROJECT_NAME}> verify reset exit"
    DEPENDS ${PROJECT_NAME}
    COMMENT "Flashing ${PROJECT_NAME}.elf via ST-Link"
)

# ── Export compile_commands.json (for IntelliSense) ───────────────────────
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
