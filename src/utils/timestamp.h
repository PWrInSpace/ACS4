/*
 * ACS4 Flight Computer — High-Resolution Timestamp (DWT Cycle Counter)
 *
 * Uses ARM Cortex-M7 DWT->CYCCNT for cycle-accurate timing.
 * Must call timestamp_init() once at boot before any reads.
 */

#ifndef ACS4_TIMESTAMP_H
#define ACS4_TIMESTAMP_H

#include <cstdint>

extern "C" {
#include "hal.h"
}

namespace acs
{

/**
 * @brief Enable the DWT cycle counter. Call once at boot.
 */
inline void timestamp_init()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Read raw cycle count (wraps every ~7.8s @ 550MHz).
 */
inline uint32_t timestamp_cycles()
{
    return DWT->CYCCNT;
}

/**
 * @brief Read time in microseconds since boot (wraps every ~7.8s).
 */
inline uint32_t timestamp_us()
{
    return DWT->CYCCNT / (STM32_SYS_CK / 1000000UL);
}

/**
 * @brief Convert a cycle delta to microseconds.
 */
inline float cycles_to_us(uint32_t cycles)
{
    return static_cast<float>(cycles)
           / static_cast<float>(STM32_SYS_CK / 1000000UL);
}

}  // namespace acs

#endif /* ACS4_TIMESTAMP_H */
