/*
 * ACS4 Flight Computer — Debug Shell
 *
 * Interactive CLI over UART3 (ST-Link VCP) using ChibiOS Shell.
 * Provides commands: version, uptime, threads, reboot, perf, errors, param.
 */

#ifndef ACS4_DEBUG_SHELL_H
#define ACS4_DEBUG_SHELL_H

#include <cstdint>

extern "C" {
#include "ch.h"

#include "hal.h"
}

namespace acs
{

/**
 * @brief Initialize and start the debug shell thread.
 *
 * @param serial_driver Pointer to ChibiOS SerialDriver (e.g. &SD3).
 * @param baudrate      UART baud rate (default 921600).
 */
void shell_start(SerialDriver *serial_driver, uint32_t baudrate = 921600);

}  // namespace acs

#endif /* ACS4_DEBUG_SHELL_H */
