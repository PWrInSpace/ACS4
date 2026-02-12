/*
 * ACS4 Flight Computer — Software + Hardware Watchdog
 *
 * Software watchdog: Each thread calls watchdog_feed(slot).
 *                    A monitor thread checks for timeouts.
 * Hardware watchdog: IWDG with ~500ms timeout as last resort.
 */

#pragma once

#include <cstdint>

namespace acs
{

/**
 * @brief Watchdog slot IDs. Register one per critical thread.
 */
static constexpr int WDG_MAX_SLOTS = 8;

/**
 * @brief Initialize hardware IWDG and start the software watchdog monitor.
 *        Call once after chSysInit().
 */
void watchdog_init();

/**
 * @brief Register a thread for software watchdog monitoring.
 *
 * @param name           Human-readable name (for error reporting).
 * @param timeout_ms     Maximum allowed time between feeds.
 * @return Slot ID (0..WDG_MAX_SLOTS-1), or -1 if table full.
 */
int watchdog_register(const char *name, uint32_t timeout_ms);

/**
 * @brief Feed the software watchdog (call from monitored thread's loop).
 */
void watchdog_feed(int slot_id);

}  // namespace acs
