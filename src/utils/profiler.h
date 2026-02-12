/*
 * ACS4 Flight Computer — Execution Time Profiler
 *
 * Lightweight per-slot profiler based on DWT cycle counter.
 * Each slot tracks: last / avg / max execution time in cycles.
 * Use PROFILE_BEGIN / PROFILE_END macros in hot loops.
 */

#pragma once

#include <cstdint>
#include <cstring>

extern "C" {
#include "hal.h"

#include <chprintf.h>
}

#include "utils/timestamp.h"

namespace acs
{

/* Maximum number of independently tracked profiling slots. */
static constexpr int PROFILER_MAX_SLOTS = 16;

/**
 * @brief Data for a single profiling slot.
 */
struct ProfileSlot
{
    const char *name;
    uint32_t    last_cycles;
    uint32_t    max_cycles;
    uint64_t    total_cycles;
    uint32_t    count;
    uint32_t    _start; /* internal: captured at PROFILE_BEGIN */
};

/**
 * @brief Global profiler state (statically allocated).
 */
inline ProfileSlot &profiler_slot(int id)
{
    static ProfileSlot slots[PROFILER_MAX_SLOTS] = {};
    return slots[id];
}

inline int &profiler_slot_count()
{
    static int n = 0;
    return n;
}

/**
 * @brief Register a named profiling slot. Returns slot ID.
 *        Call at init time (not in hot path).
 */
inline int profiler_register(const char *name)
{
    int &n = profiler_slot_count();
    if (n >= PROFILER_MAX_SLOTS)
    {
        return -1;
    }
    const int id   = n++;
    auto &s        = profiler_slot(id);
    s.name         = name;
    s.last_cycles  = 0;
    s.max_cycles   = 0;
    s.total_cycles = 0;
    s.count        = 0;
    return id;
}

/**
 * @brief Begin timing for a slot.
 */
inline void profiler_begin(int slot_id)
{
    profiler_slot(slot_id)._start = DWT->CYCCNT;
}

/**
 * @brief End timing for a slot. Computes delta and updates stats.
 */
inline void profiler_end(int slot_id)
{
    const uint32_t now   = DWT->CYCCNT;
    auto            &s    = profiler_slot(slot_id);
    const uint32_t   delta = now - s._start; /* handles wrap correctly */

    s.last_cycles = delta;
    s.total_cycles += delta;
    s.count++;
    if (delta > s.max_cycles)
    {
        s.max_cycles = delta;
    }
}

/**
 * @brief Print all profiler slots to a stream (shell `perf` command).
 */
inline void profiler_print(BaseSequentialStream *chp)
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
        auto &s = profiler_slot(i);
        const float avg_us =
            (s.count > 0)
                ? cycles_to_us(static_cast<uint32_t>(s.total_cycles / s.count))
                : 0.0f;
        chprintf(chp,
                 "%-24s %10.1f %10.1f %10.1f %10lu\r\n",
                 s.name,
                 static_cast<double>(cycles_to_us(s.last_cycles)),
                 static_cast<double>(avg_us),
                 static_cast<double>(cycles_to_us(s.max_cycles)),
                 s.count);
    }
}

/**
 * @brief Reset stats for all slots (but keep registrations).
 */
inline void profiler_reset()
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

/* ── Convenience macros ───────────────────────────────────────────────── */

#define PROFILE_BEGIN(slot_id) acs::profiler_begin(slot_id)
#define PROFILE_END(slot_id)   acs::profiler_end(slot_id)
