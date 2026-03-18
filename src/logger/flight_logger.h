/*
 * ACS4 Flight Computer — Flight Logger (Black Box)
 *
 * Dual-buffer binary logger writing packed log records to SD card via FatFs.
 * Any thread may call log() to append a record; when the active buffer fills
 * up, it is swapped and LoggerThread flushes the inactive buffer to disk.
 *
 * Thread safety: log() uses a short critical section (<1µs) to copy the
 * record into the active buffer and bump the write pointer. The swap and
 * semaphore signal are also inside the critical section.
 *
 * Memory: 2 × 16 KiB buffers in AXI SRAM = 32 KiB total.
 */

#pragma once

#include <cstdint>

#include "logger/log_format.h"

extern "C" {
#include "ch.h"

#include "hal.h"
}

namespace acs
{

/**
 * @brief Logger operating state.
 */
enum class LoggerState : uint8_t
{
    IDLE,      /* not logging */
    LOGGING,   /* actively recording */
    ERROR,     /* SD write error, stopped */
};

/**
 * @brief Runtime statistics for the logger.
 */
struct LoggerStats
{
    LoggerState state;
    uint32_t    records_written;
    uint32_t    bytes_written;
    uint32_t    flush_count;
    uint32_t    flush_errors;
    uint32_t    overflow_count;  /* records dropped because both buffers full */
    char        filename[32];
};

/**
 * @brief Initialize the logger subsystem. Must be called after sdmmc_mount().
 * @return true if the SD card is ready and the logger is in IDLE state.
 */
bool logger_init();

/**
 * @brief Start logging to a new file (auto-generated name LOG_NNN.BIN).
 * @return true if the log file was created and logging began.
 */
bool logger_start();

/**
 * @brief Stop logging, flush remaining data, and close the log file.
 */
void logger_stop();

/**
 * @brief Append a log record (any packed struct from log_format.h).
 *
 * Safe to call from any thread or ISR context (uses chSysLock).
 * The record is memcpy'd into the active buffer. If the buffer is full
 * and the inactive buffer is also not yet flushed, the record is dropped
 * and overflow_count is incremented.
 *
 * @param data   Pointer to packed log record (starts with LogHeader).
 * @param len    Size of the record in bytes (must be <= LOG_MAX_RECORD_SIZE).
 */
void logger_log(const void *data, size_t len);

/**
 * @brief Convenience template — logs any log record struct directly.
 */
template <typename T>
inline void logger_log(const T &record)
{
    logger_log(&record, sizeof(T));
}

/**
 * @brief Get current logger statistics.
 */
[[nodiscard]] LoggerStats logger_stats();

/**
 * @brief Print logger status to a stream (shell command).
 */
void logger_print_status(BaseSequentialStream *chp);

/**
 * @brief LoggerThread entry point. Created by main.cpp.
 *
 * Waits on a binary semaphore; when signalled, flushes the inactive
 * buffer to the open FatFs file and syncs.
 */
void logger_thread(void *arg);

}  // namespace acs
