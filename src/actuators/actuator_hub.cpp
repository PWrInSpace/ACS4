/*
 * ACS4 Flight Computer — Actuator Hub Implementation
 */

#include "actuators/actuator_hub.h"

extern "C" {
#include "ch.h"
}

namespace acs
{

static ActuatorHub g_hub;

ActuatorHub &actuator_hub()
{
    return g_hub;
}

void ActuatorHub::set_aileron_deg(uint8_t idx, float deg, uint32_t timestamp_us)
{
    if (idx >= kAileronCount)
    {
        return;
    }

    chSysLock();
    data_.aileron_cmd_deg[idx] = deg;
    data_.sweep[idx].active    = false;
    data_.cmd_timestamp_us     = timestamp_us;
    chSysUnlock();
}

void ActuatorHub::set_armed_request(bool armed)
{
    chSysLock();
    data_.armed_request = armed;
    chSysUnlock();
}

void ActuatorHub::set_sweep(uint8_t  idx,
                            bool     active,
                            float    min_deg,
                            float    max_deg,
                            uint32_t period_ms,
                            uint32_t start_time_ms)
{
    if (idx >= kAileronCount)
    {
        return;
    }

    chSysLock();
    data_.sweep[idx] = {
        .active        = active,
        .min_deg       = min_deg,
        .max_deg       = max_deg,
        .period_ms     = period_ms,
        .start_time_ms = start_time_ms,
    };
    chSysUnlock();
}

void ActuatorHub::update_current_us(uint8_t idx, uint16_t us)
{
    if (idx >= kAileronCount)
    {
        return;
    }

    chSysLock();
    data_.aileron_current_us[idx] = us;
    chSysUnlock();
}

void ActuatorHub::update_armed(bool armed)
{
    chSysLock();
    data_.armed = armed;
    chSysUnlock();
}

ActuatorSnapshot ActuatorHub::snapshot()
{
    ActuatorSnapshot copy{};
    chSysLock();
    copy = data_;
    chSysUnlock();
    return copy;
}

}  // namespace acs
