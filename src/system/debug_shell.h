/*
 * ACS4 Flight Computer — Debug Shell
 *
 * Interactive CLI using ChibiOS Shell.
 * Nucleo:     UART3 (ST-Link VCP)
 * Custom PCB: USB CDC (SerialUSBDriver SDU1)
 *
 * Provides commands: version, uptime, threads, reboot, perf, errors, param.
 */

#pragma once

#include <cstdint>

extern "C" {
#include "ch.h"

#include "hal.h"
}

namespace acs
{

/**
 * @brief Start the debug shell on a UART serial port.
 *
 * @param serial_driver Pointer to ChibiOS SerialDriver (e.g. &SD3).
 * @param baudrate      UART baud rate (default 921600).
 */
void shell_start(SerialDriver *serial_driver, uint32_t baudrate = 921600);

/**
 * @brief Start the debug shell on an arbitrary BaseSequentialStream.
 *
 * Use this for USB CDC (SerialUSBDriver) or any other stream.
 * @param stream  Pointer to a BaseSequentialStream (e.g. usb_cdc_stream()).
 */
void shell_start(BaseSequentialStream *stream);

}  // namespace acs
