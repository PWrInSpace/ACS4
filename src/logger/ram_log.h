/*
 * ACS4 Flight Computer — RAM Log (Black Box Backup)
 *
 * Circular buffer in RAM holding the last ~10 seconds of log records.
 * Survives SD card failures — data can be extracted via debugger or
 * dumped to SD once the card recovers.
 *
 * Budget: 52 KiB (10s × 1000Hz × ~52 bytes/record average)
 * Located in AXI SRAM.
 *
 * Thread safety: push uses chSysLock (same as flight_logger::log).
 */

#pragma once

#include <cstdint>

extern "C" {
#include "hal.h"
}

namespace acs
{

/**
 * @brief Initialize the RAM log ring buffer.
 */
void ram_log_init();

/**
 * @brief Push a record into the RAM log (circular, overwrites oldest).
 *
 * Must be called from within a chSysLock/chSysUnlock pair, or from
 * a context where interrupts are already masked. flight_logger::log()
 * calls this automatically before writing to the SD buffer.
 *
 * @param data  Pointer to packed log record.
 * @param len   Size of the record in bytes.
 */
void ram_log_push(const void *data, size_t len);

/**
 * @brief Get the RAM log ring buffer contents for dump.
 *
 * @param[out] buf   Pointer set to the internal buffer start.
 * @param[out] size  Total buffer capacity.
 * @param[out] head  Current write head position.
 * @param[out] used  Total bytes currently stored (may wrap).
 */
void ram_log_get(const uint8_t *&buf, size_t &size, size_t &head, size_t &used);

/**
 * @brief Print RAM log summary to a stream (record count estimate).
 */
void ram_log_print_status(BaseSequentialStream *chp);

}  // namespace acs
