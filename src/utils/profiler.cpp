/*
 * ACS4 Flight Computer — Execution Time Profiler (Implementation)
 *
 * Cold-path profiler functions: register, print, reset.
 * See profiler.h for hot-path inline functions and API docs.
 */

#include "utils/profiler.h"

#include "utils/timestamp.h"

extern "C" {
#include <chprintf.h>
}

namespace acs
{

int profiler_register(const char *name)
{
    int &n = profiler_slot_count();
    if (n >= PROFILER_MAX_SLOTS)
    {
        return -1;
    }
    const int id   = n++;
    auto     &s    = profiler_slot(id);
    s.name         = name;
    s.last_cycles  = 0;
    s.max_cycles   = 0;
    s.total_cycles = 0;
    s.count        = 0;
    return id;
}

void profiler_print(BaseSequentialStream *chp)
{
    const int n = profiler_slot_count();
    if (n == 0)
    {
        chprintf(chp, "No profiling slots registered.\r\n");
        return;
    }

    chprintf(chp,
             "%-24s %10s %10s %10s %10s\r\n",
             "Slot",
             "Last(us)",
             "Avg(us)",
             "Max(us)",
             "Count");
    chprintf(chp,
             "-----------------------------------------------------------------"
             "-----\r\n");

    for (int i = 0; i < n && i < PROFILER_MAX_SLOTS; i++)
    {
        auto       &s = profiler_slot(i);
        const float avg_us =
            (s.count > 0) ? cycles_to_us(static_cast<uint32_t>(s.total_cycles / s.count)) : 0.0f;
        chprintf(chp,
                 "%-24s %10.1f %10.1f %10.1f %10lu\r\n",
                 s.name,
                 static_cast<double>(cycles_to_us(s.last_cycles)),
                 static_cast<double>(avg_us),
                 static_cast<double>(cycles_to_us(s.max_cycles)),
                 s.count);
    }
}

void profiler_reset()
{
    const int n = profiler_slot_count();
    for (int i = 0; i < n && i < PROFILER_MAX_SLOTS; i++)
    {
        auto &s        = profiler_slot(i);
        s.last_cycles  = 0;
        s.max_cycles   = 0;
        s.total_cycles = 0;
        s.count        = 0;
    }
}

}  // namespace acs
