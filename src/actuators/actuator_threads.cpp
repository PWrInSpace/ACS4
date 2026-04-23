/*
 * ACS4 Flight Computer — Actuator Thread Implementation
 */

#include "actuators/actuator_threads.h"

#include "actuators/actuator_hub.h"
#include "drivers/servo_t75.h"
#include "system/watchdog.h"
#include "utils/profiler.h"
#include "utils/timestamp.h"

extern "C" {
#include "ch.h"

#include "hal.h"
}

#if defined(STM32H725xx)

namespace acs
{

/* Triangle wave in [min_deg, max_deg] with full period `period_ms`. */
static float triangle_wave_deg(float min_deg, float max_deg, uint32_t period_ms, uint32_t phase_ms)
{
    if (period_ms == 0)
    {
        return 0.5f * (min_deg + max_deg);
    }

    const uint32_t half       = period_ms / 2U;
    const uint32_t in_period  = phase_ms % period_ms;
    const float    amplitude  = max_deg - min_deg;

    if (half == 0)
    {
        return 0.5f * (min_deg + max_deg);
    }

    if (in_period < half)
    {
        /* rising edge: min → max over `half` ms */
        const float frac = static_cast<float>(in_period) / static_cast<float>(half);
        return min_deg + frac * amplitude;
    }

    /* falling edge: max → min over `half` ms */
    const float frac = static_cast<float>(in_period - half) / static_cast<float>(half);
    return max_deg - frac * amplitude;
}

/* =====================================================================
 * ActuatorThread — 100 Hz write-out to ServoBankT75
 * ===================================================================== */

static int g_prof_act = -1;
static int g_wdg_act  = -1;

static THD_WORKING_AREA(waActuatorThread, 1024);

static THD_FUNCTION(ActuatorThread, arg)
{
    (void)arg;
    chRegSetThreadName("actuator");

    g_prof_act = profiler_register("actuator");
    g_wdg_act  = watchdog_register("actuator", 50);

    auto *bank = servo_bank_instance();
    if (bank == nullptr)
    {
        return;
    }

    bool      last_armed_req = false;
    systime_t next           = chVTGetSystemTimeX();

    while (true)
    {
        next += TIME_MS2I(10);

        PROFILE_BEGIN(g_prof_act);

        const ActuatorSnapshot snap = actuator_hub().snapshot();

        /* Synchronize arm state with the producer's request. */
        if (snap.armed_request != last_armed_req)
        {
            if (snap.armed_request)
            {
                bank->arm();
            }
            else
            {
                bank->disarm();
            }
            last_armed_req = snap.armed_request;
            actuator_hub().update_armed(snap.armed_request);
        }

        /* Compute per-fin target angle (sweep overrides static cmd). */
        const auto now_ms = static_cast<uint32_t>(chTimeI2MS(chVTGetSystemTimeX()));
        for (uint8_t i = 0; i < kAileronCount; ++i)
        {
            float deg = snap.aileron_cmd_deg[i];
            if (snap.sweep[i].active)
            {
                const uint32_t phase = now_ms - snap.sweep[i].start_time_ms;
                deg                  = triangle_wave_deg(
                    snap.sweep[i].min_deg,
                    snap.sweep[i].max_deg,
                    snap.sweep[i].period_ms,
                    phase);
            }
            bank->set_angle_deg(i, deg);
        }

        bank->tick(10);

        for (uint8_t i = 0; i < kAileronCount; ++i)
        {
            actuator_hub().update_current_us(i, bank->current_pulse_us(i));
        }

        PROFILE_END(g_prof_act);

        if (g_wdg_act >= 0)
        {
            watchdog_feed(g_wdg_act);
        }

        chThdSleepUntil(next);
    }
}

void start_actuator_threads()
{
    chThdCreateStatic(
        waActuatorThread, sizeof(waActuatorThread), NORMALPRIO + 20, ActuatorThread, nullptr);
}

}  // namespace acs

#else /* NUCLEO_H723 — no PCB, no servos */

namespace acs
{

void start_actuator_threads()
{
}

}  // namespace acs

#endif
