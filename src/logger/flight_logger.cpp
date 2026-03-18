/*
 * ACS4 Flight Computer — Flight Logger Implementation
 *
 * Dual 16 KiB buffers, swap-on-full, semaphore-driven flush to SD.
 */

#include "logger/flight_logger.h"

#include <cstdio>
#include <cstring>

#include "hal/sdmmc.h"
#include "logger/ram_log.h"
#include "system/error_handler.h"
#include "utils/timestamp.h"

extern "C" {
#include <chprintf.h>

#include "ff.h"
}

namespace acs
{

/* ── Constants ────────────────────────────────────────────────────────────── */

static constexpr size_t BUF_SIZE = 16384;

/* ── State ────────────────────────────────────────────────────────────────── */

static uint8_t s_buf[2][BUF_SIZE] __attribute__((aligned(4)));

static volatile uint8_t  s_active   = 0;
static volatile size_t   s_write_pos = 0;
static volatile bool     s_flush_pending = false;

static binary_semaphore_t s_flush_sem;

static FIL         s_file;
static bool        s_file_open  = false;
static LoggerState s_state      = LoggerState::IDLE;

static uint32_t s_records   = 0;
static uint32_t s_bytes     = 0;
static uint32_t s_flushes   = 0;
static uint32_t s_flush_err = 0;
static uint32_t s_overflows = 0;
static char     s_filename[32] = {};

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static bool find_next_filename()
{
    for (int i = 1; i <= 999; i++)
    {
        chsnprintf(s_filename, sizeof(s_filename), "LOG_%03d.BIN", i);

        FILINFO fno;
        if (f_stat(s_filename, &fno) == FR_NO_FILE)
        {
            return true;
        }
    }
    return false;
}

static bool write_file_header()
{
    LogFileHeader hdr{};
    memcpy(hdr.magic, LOG_MAGIC, 4);
    hdr.version      = LOG_FORMAT_VERSION;
    hdr.sysclk_hz    = STM32_SYS_CK;
    hdr.boot_time_ms = static_cast<uint32_t>(chTimeI2MS(chVTGetSystemTimeX()));

    UINT bw = 0;
    FRESULT res = f_write(&s_file, &hdr, sizeof(hdr), &bw);
    return (res == FR_OK) && (bw == sizeof(hdr));
}

/* ── Public API ───────────────────────────────────────────────────────────── */

bool logger_init()
{
    chBSemObjectInit(&s_flush_sem, true);

    s_active       = 0;
    s_write_pos    = 0;
    s_flush_pending = false;
    s_state        = LoggerState::IDLE;
    s_records      = 0;
    s_bytes        = 0;
    s_flushes      = 0;
    s_flush_err    = 0;
    s_overflows    = 0;

    return sdmmc_is_mounted();
}

bool logger_start()
{
    if (s_state == LoggerState::LOGGING)
    {
        return true;
    }

    if (!sdmmc_is_mounted())
    {
        return false;
    }

    if (!find_next_filename())
    {
        return false;
    }

    FRESULT res = f_open(&s_file, s_filename, FA_WRITE | FA_CREATE_NEW);
    if (res != FR_OK)
    {
        error_report(ErrorCode::SD_WRITE_FAIL);
        return false;
    }

    s_file_open = true;

    if (!write_file_header())
    {
        f_close(&s_file);
        s_file_open = false;
        error_report(ErrorCode::SD_WRITE_FAIL);
        return false;
    }

    s_active        = 0;
    s_write_pos     = 0;
    s_flush_pending = false;
    s_records       = 0;
    s_bytes         = sizeof(LogFileHeader);
    s_flushes       = 0;
    s_flush_err     = 0;
    s_overflows     = 0;
    s_state         = LoggerState::LOGGING;

    return true;
}

void logger_stop()
{
    if (s_state != LoggerState::LOGGING)
    {
        return;
    }

    s_state = LoggerState::IDLE;

    /* Flush whatever is in the active buffer. */
    chSysLock();
    size_t  remaining = s_write_pos;
    uint8_t which     = s_active;
    s_write_pos       = 0;
    chSysUnlock();

    if (remaining > 0 && s_file_open)
    {
        UINT bw = 0;
        f_write(&s_file, s_buf[which], remaining, &bw);
        f_sync(&s_file);
        s_bytes += bw;
    }

    /* Wait for any pending flush to complete. */
    if (s_flush_pending)
    {
        chThdSleepMilliseconds(50);
    }

    if (s_file_open)
    {
        f_close(&s_file);
        s_file_open = false;
    }
}

void logger_log(const void *data, size_t len)
{
    if (s_state != LoggerState::LOGGING)
    {
        return;
    }

    if (len > LOG_MAX_RECORD_SIZE)
    {
        return;
    }

    /* Also push to RAM log for backup. */
    ram_log_push(data, len);

    chSysLock();

    if (s_write_pos + len > BUF_SIZE)
    {
        if (s_flush_pending)
        {
            /* Both buffers occupied — drop this record. */
            s_overflows++;
            chSysUnlock();
            return;
        }

        /* Swap buffers and signal the flush thread. */
        s_flush_pending = true;
        s_active ^= 1;
        s_write_pos = 0;
        chBSemSignalI(&s_flush_sem);
    }

    memcpy(&s_buf[s_active][s_write_pos], data, len);
    s_write_pos += len;
    s_records++;

    chSysUnlock();
}

LoggerStats logger_stats()
{
    LoggerStats st{};

    chSysLock();
    st.state           = s_state;
    st.records_written = s_records;
    st.bytes_written   = s_bytes;
    st.flush_count     = s_flushes;
    st.flush_errors    = s_flush_err;
    st.overflow_count  = s_overflows;
    chSysUnlock();

    memcpy(st.filename, s_filename, sizeof(s_filename));
    return st;
}

void logger_print_status(BaseSequentialStream *chp)
{
    const LoggerStats st = logger_stats();

    const char *state_str = "???";
    switch (st.state)
    {
        case LoggerState::IDLE:    state_str = "IDLE";    break;
        case LoggerState::LOGGING: state_str = "LOGGING"; break;
        case LoggerState::ERROR:   state_str = "ERROR";   break;
    }

    chprintf(chp, "Logger state:   %s\r\n", state_str);
    chprintf(chp, "File:           %s\r\n", st.filename[0] ? st.filename : "(none)");
    chprintf(chp, "Records:        %lu\r\n", st.records_written);
    chprintf(chp, "Bytes written:  %lu\r\n", st.bytes_written);
    chprintf(chp, "Flushes:        %lu\r\n", st.flush_count);
    chprintf(chp, "Flush errors:   %lu\r\n", st.flush_errors);
    chprintf(chp, "Overflows:      %lu\r\n", st.overflow_count);

    if (sdmmc_is_mounted())
    {
        uint32_t total = 0;
        uint32_t free_space = 0;
        if (sdmmc_free_space(total, free_space))
        {
            chprintf(chp, "SD card:        %lu MiB total, %lu MiB free\r\n", total, free_space);
        }
    }
    else
    {
        chprintf(chp, "SD card:        not mounted\r\n");
    }
}

/* ── LoggerThread ─────────────────────────────────────────────────────────── */

void logger_thread(void *arg)
{
    (void)arg;
    chRegSetThreadName("logger");

    while (true)
    {
        chBSemWait(&s_flush_sem);

        if (!s_file_open || s_state != LoggerState::LOGGING)
        {
            s_flush_pending = false;
            continue;
        }

        uint8_t flush_buf = s_active ^ 1;

        UINT    bw  = 0;
        FRESULT res = f_write(&s_file, s_buf[flush_buf], BUF_SIZE, &bw);

        if (res == FR_OK && bw == BUF_SIZE)
        {
            f_sync(&s_file);
            s_bytes += bw;
            s_flushes++;
        }
        else
        {
            s_flush_err++;
            error_report(ErrorCode::SD_WRITE_FAIL);

            if (s_flush_err >= 10)
            {
                s_state = LoggerState::ERROR;
            }
        }

        s_flush_pending = false;
    }
}

}  // namespace acs
