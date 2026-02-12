/*
 * ACS4 Flight Computer — Central Error Handler Implementation
 */

#include "system/error_handler.h"

#include <cstring>

extern "C" {
#include "ch.h"

#include <chprintf.h>
}

namespace acs
{

/* ── Error state ──────────────────────────────────────────────────────── */

struct ErrorEntry
{
    uint32_t count;
    uint32_t last_ms; /* systime of last occurrence */
};

static ErrorEntry s_errors[static_cast<int>(ErrorCode::COUNT)] = {};

/* ── Name lookup ──────────────────────────────────────────────────────── */

// clang-format off
static const char *const s_error_names[] = {
    "NONE",
    "IMU_COMM_FAIL",
    "IMU_SELF_TEST_FAIL",
    "BARO_COMM_FAIL",
    "MAG_COMM_FAIL",
    "GPS_NO_FIX",
    "SD_WRITE_FAIL",
    "SD_FULL",
    "STACK_OVERFLOW",
    "ESKF_DIVERGED",
    "PYRO_CONTINUITY_FAIL",
    "RADIO_LOST",
    "BATTERY_LOW",
    "WATCHDOG_TIMEOUT",
};
// clang-format on

static_assert(sizeof(s_error_names) / sizeof(s_error_names[0])
                  == static_cast<int>(ErrorCode::COUNT),
              "s_error_names out of sync with ErrorCode enum");

/* ── Critical error set ───────────────────────────────────────────────── */

bool is_critical(ErrorCode code)
{
    switch (code)
    {
        case ErrorCode::IMU_COMM_FAIL:
        case ErrorCode::IMU_SELF_TEST_FAIL:
        case ErrorCode::ESKF_DIVERGED:
        case ErrorCode::STACK_OVERFLOW:
        case ErrorCode::WATCHDOG_TIMEOUT:
            return true;
        default:
            return false;
    }
}

/* ── Public API ───────────────────────────────────────────────────────── */

void error_report(ErrorCode code)
{
    auto idx = static_cast<int>(code);
    if (idx <= 0 || idx >= static_cast<int>(ErrorCode::COUNT))
    {
        return;
    }

    chSysLock();
    s_errors[idx].count++;
    s_errors[idx].last_ms =
        static_cast<uint32_t>(chTimeI2MS(chVTGetSystemTimeX()));
    chSysUnlock();
}

uint32_t error_count(ErrorCode code)
{
    auto idx = static_cast<int>(code);
    if (idx <= 0 || idx >= static_cast<int>(ErrorCode::COUNT))
    {
        return 0;
    }
    return s_errors[idx].count;
}

const char *error_name(ErrorCode code)
{
    auto idx = static_cast<int>(code);
    if (idx < 0
        || idx >= static_cast<int>(sizeof(s_error_names)
                                   / sizeof(s_error_names[0])))
    {
        return "???";
    }
    return s_error_names[idx];
}

void error_clear_all()
{
    chSysLock();
    memset(s_errors, 0, sizeof(s_errors));
    chSysUnlock();
}

void error_print(BaseSequentialStream *chp)
{
    chprintf(chp,
             "%-24s %8s %10s %s\r\n",
             "Error",
             "Count",
             "Last(ms)",
             "Critical");
    chprintf(chp, "------------------------------------------------------\r\n");

    bool any = false;
    for (int i = 1; i < static_cast<int>(ErrorCode::COUNT); i++)
    {
        auto code = static_cast<ErrorCode>(i);
        if (s_errors[i].count > 0)
        {
            any = true;
            chprintf(chp,
                     "%-24s %8lu %10lu %s\r\n",
                     error_name(code),
                     s_errors[i].count,
                     s_errors[i].last_ms,
                     is_critical(code) ? "YES" : "no");
        }
    }
    if (!any)
    {
        chprintf(chp, "  (no errors recorded)\r\n");
    }
}

}  // namespace acs
