/*
 * ACS4 Flight Computer — SRT Servo T75 HV Niezalezna platformowo matma
 */

#include "drivers/servo_t75_math.h"

#include <algorithm>
#include <cmath>

namespace acs::servo_t75
{

uint16_t angle_to_pulse_us(float deg, const Limits &lim, float max_angle_deg)
{
    /* Clampuj kat wejsciowy do zadeklarowanego zakresu deflection. */
    if (max_angle_deg < 0.0f)
    {
        max_angle_deg = -max_angle_deg;
    }
    if (deg > max_angle_deg)
    {
        deg = max_angle_deg;
    }
    else if (deg < -max_angle_deg)
    {
        deg = -max_angle_deg;
    }

    const float sign      = (lim.direction_sign >= 0) ? 1.0f : -1.0f;
    const float offset_us = sign * deg * lim.us_per_deg;

    float pulse = static_cast<float>(lim.neutral_us) + offset_us;
    pulse       = std::round(pulse);

    /* Hardware safety clamp (wykonywane na koncu, niezaleznie od neutral). */
    if (pulse < static_cast<float>(lim.min_us))
    {
        pulse = static_cast<float>(lim.min_us);
    }
    else if (pulse > static_cast<float>(lim.max_us))
    {
        pulse = static_cast<float>(lim.max_us);
    }

    return static_cast<uint16_t>(pulse);
}

uint16_t slew_step(uint16_t current, uint16_t target, float us_per_ms, uint32_t dt_ms)
{
    if (current == target || dt_ms == 0 || us_per_ms <= 0.0f)
    {
        return current;
    }

    const float max_delta = us_per_ms * static_cast<float>(dt_ms);
    const int   diff      = static_cast<int>(target) - static_cast<int>(current);

    if (static_cast<float>(std::abs(diff)) <= max_delta)
    {
        return target;
    }

    const float step       = (diff > 0) ? max_delta : -max_delta;
    const float new_pulse  = static_cast<float>(current) + step;
    const int   new_pulse_i = static_cast<int>(std::round(new_pulse));

    if (new_pulse_i < 0)
    {
        return 0;
    }
    if (new_pulse_i > 0xFFFF)
    {
        return 0xFFFF;
    }
    return static_cast<uint16_t>(new_pulse_i);
}

}  // namespace acs::servo_t75
