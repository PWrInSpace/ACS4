/*
 * ACS4 Flight Computer — USB CDC (Virtual COM Port)
 *
 * USB OTG_HS in FS mode on PA11/PA12 (custom PCB only).
 * Provides a SerialUSBDriver (SDU1) usable as BaseSequentialStream.
 *
 * Usage:
 *   acs::usb_cdc_init();          // call after halInit()+chSysInit()
 *   auto *stream = acs::usb_cdc_stream();  // get BaseSequentialStream*
 */

#pragma once

extern "C" {
#include "ch.h"

#include "hal.h"
}

namespace acs
{

/**
 * @brief Initialize USB CDC and connect to host.
 *
 * Initializes SerialUSBDriver SDU1, starts the USB OTG_HS peripheral,
 * and performs bus connect. Blocks ~1.5s for USB re-enumeration.
 */
void usb_cdc_init();

/**
 * @brief Get the USB CDC BaseSequentialStream for I/O.
 * @return Pointer to the SDU1 stream (valid after usb_cdc_init).
 */
BaseSequentialStream *usb_cdc_stream();

}  // namespace acs

