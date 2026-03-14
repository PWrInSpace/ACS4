/*
 * ACS4 Flight Computer — Execution Time Profiler
 *
 * Lightweight per-slot profiler based on DWT cycle counter.
 * Each slot tracks: last / avg / max execution time in cycles.
 * Use PROFILE_BEGIN / PROFILE_END macros in hot loops.
 *
 * profiler_begin / profiler_end are inline for zero-overhead
 * instrumentation. Cold-path functions live in profiler.cpp.
 */

#pragma once

#include <cstdint>

extern "C" {
#include "hal.h"
}

namespace acs
{

/* Maximum number of independently tracked profiling slots. */
inline constexpr int PROFILER_MAX_SLOTS = 16;

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
    uint32_t    start_cycle; /* internal: captured at PROFILE_BEGIN */
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

/* ── Hot-path functions (inline) ──────────────────────────────────────── */

/**
 * @brief Begin timing for a slot.
 */
inline void profiler_begin(int slot_id)
{
    profiler_slot(slot_id).start_cycle = DWT->CYCCNT;
}

/**
 * @brief End timing for a slot. Computes delta and updates stats.
 */
inline void profiler_end(int slot_id)
{
    const uint32_t now   = DWT->CYCCNT;
    auto          &s     = profiler_slot(slot_id);
    const uint32_t delta = now - s.start_cycle; /* handles wrap correctly */

    s.last_cycles = delta;
    s.total_cycles += delta;
    s.count++;
    if (delta > s.max_cycles)
    {
        s.max_cycles = delta;
    }
}

/* ── Cold-path functions (defined in profiler.cpp) ────────────────────── */

/**
 * @brief Register a named profiling slot. Returns slot ID.
 *        Call at init time (not in hot path).
 */
int profiler_register(const char *name);

/**
 * @brief Print all profiler slots to a stream (shell `perf` command).
 */
void profiler_print(BaseSequentialStream *chp);

/**
 * @brief Reset stats for all slots (but keep registrations).
 */
void profiler_reset();

}  // namespace acs

/* ── Convenience macros ───────────────────────────────────────────────── */

#define PROFILE_BEGIN(slot_id) acs::profiler_begin(slot_id)
#define PROFILE_END(slot_id)   acs::profiler_end(slot_id)
