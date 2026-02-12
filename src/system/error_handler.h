/*
 * ACS4 Flight Computer — Central Error Handler
 *
 * Atomic error counters per ErrorCode. Thread-safe.
 * Integrates with shell via `errors` command.
 */

#pragma once

#include <cstdint>

extern "C" {
#include "hal.h"
}

namespace acs
{

/**
 * @brief System-wide error codes.
 */
enum class ErrorCode : uint16_t
{
    NONE = 0,
    IMU_COMM_FAIL,
    IMU_SELF_TEST_FAIL,
    BARO_COMM_FAIL,
    MAG_COMM_FAIL,
    GPS_NO_FIX,
    SD_WRITE_FAIL,
    SD_FULL,
    STACK_OVERFLOW,
    ESKF_DIVERGED,
    PYRO_CONTINUITY_FAIL,
    RADIO_LOST,
    BATTERY_LOW,
    WATCHDOG_TIMEOUT,

    COUNT /* must be last */
};

/**
 * @brief Report an error (increments counter, logs timestamp).
 */
void error_report(ErrorCode code);

/**
 * @brief Get the count for a specific error code.
 */
uint32_t error_count(ErrorCode code);

/**
 * @brief Check if an error is considered flight-critical (abort-worthy).
 */
bool is_critical(ErrorCode code);

/**
 * @brief Get human-readable name for an error code.
 */
const char *error_name(ErrorCode code);

/**
 * @brief Clear all error counters.
 */
void error_clear_all();

/**
 * @brief Print error table to a stream (shell `errors` command).
 */
void error_print(BaseSequentialStream *chp);

}  // namespace acs
