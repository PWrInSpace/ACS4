/*
 * ACS4 Flight Computer — Watchdog Implementation
 *
 * Software watchdog monitor thread + hardware IWDG.
 */

#include "system/watchdog.h"

#include "system/error_handler.h"

extern "C" {
#include "ch.h"

#include "hal.h"
}

namespace acs
{

/* ── Software watchdog slots ──────────────────────────────────────────── */

struct WdgSlot
{
    const char       *name;
    uint32_t          timeout_ms;
    volatile uint32_t last_feed_ms;
    bool              active;
    volatile bool     timed_out;
};

static WdgSlot s_slots[WDG_MAX_SLOTS] = {};
static int     s_slot_count           = 0;

int watchdog_register(const char *name, uint32_t timeout_ms)
{
    if (s_slot_count >= WDG_MAX_SLOTS)
    {
        return -1;
    }
    const int id           = s_slot_count++;
    s_slots[id].name       = name;
    s_slots[id].timeout_ms = timeout_ms;
    s_slots[id].last_feed_ms =
        static_cast<uint32_t>(chTimeI2MS(chVTGetSystemTimeX()));
    s_slots[id].active    = true;
    s_slots[id].timed_out = false;
    return id;
}

void watchdog_feed(int slot_id)
{
    if (slot_id < 0 || slot_id >= s_slot_count)
    {
        return;
    }
    s_slots[slot_id].last_feed_ms =
        static_cast<uint32_t>(chTimeI2MS(chVTGetSystemTimeX()));
    s_slots[slot_id].timed_out = false;
}

/* ── Monitor thread ───────────────────────────────────────────────────── */

static THD_WORKING_AREA(waWdgMonitor, 512);

static THD_FUNCTION(WdgMonitor, arg)
{
    (void)arg;
    chRegSetThreadName("watchdog");

    while (true)
    {
        const auto now_ms =
            static_cast<uint32_t>(chTimeI2MS(chVTGetSystemTimeX()));

        /* Check each software watchdog slot. */
        for (int i = 0; i < s_slot_count; i++)
        {
            if (!s_slots[i].active)
            {
                continue;
            }

            const uint32_t elapsed = now_ms - s_slots[i].last_feed_ms;
            if (elapsed > s_slots[i].timeout_ms && !s_slots[i].timed_out)
            {
                s_slots[i].timed_out = true;
                error_report(ErrorCode::WATCHDOG_TIMEOUT);
            }
        }

        /* Feed the hardware IWDG — if we reach here, the monitor is alive. */
#if HAL_USE_WDG == TRUE
        wdgReset(&WDGD1);
#endif

        chThdSleepMilliseconds(50);
    }
}

/* ── Init ─────────────────────────────────────────────────────────────── */

void watchdog_init()
{
#if HAL_USE_WDG == TRUE
    /*
     * IWDG configuration: ~500ms timeout.
     * LSI ≈ 32 kHz.  Prescaler /32 → 1 kHz tick.  Reload = 500.
     */
    static const WDGConfig wdg_cfg = {
        STM32_IWDG_PR_32,
        STM32_IWDG_RL(500),
        STM32_IWDG_WIN_DISABLED,
    };
    wdgStart(&WDGD1, &wdg_cfg);
#endif

    /* Start monitor thread at above-normal priority. */
    chThdCreateStatic(waWdgMonitor,
                      sizeof(waWdgMonitor),
                      NORMALPRIO + 20,
                      WdgMonitor,
                      nullptr);
}

}  // namespace acs
