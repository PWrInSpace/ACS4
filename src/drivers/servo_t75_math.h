/*
 * ACS4 Flight Computer — SRT Servo T75 HV Platform-Independent Math
 *
 * Pure computation functions for the servo driver:
 *   - Angle (deg) → pulse width (µs) mapping with trim, direction, clamping
 *   - Slew-rate-limited step toward a target pulse width
 *
 * No RTOS or hardware dependencies — safe for unit tests on host.
 *
 * Servo reference values (SRT Servo T75 HV datasheet):
 *   neutral 1500 µs, 90° span → 1000–2000 µs ⇒ 11.111 µs per degree.
 */

#pragma once

#include <cstdint>

namespace acs::servo_t75
{

struct Limits
{
    uint16_t neutral_us;     /* per-channel mechanical zero pulse */
    uint16_t min_us;         /* hard lower bound (safety stop) */
    uint16_t max_us;         /* hard upper bound (safety stop) */
    int8_t   direction_sign; /* +1 or -1, flips deflection polarity */
    float    us_per_deg;     /* linear gain, datasheet = 11.111 µs/deg */
};

/**
 * @brief Map a commanded angle in degrees to a pulse width in microseconds.
 *
 * Clamps the angle to ±max_angle_deg, applies direction_sign, then
 * clamps the final pulse width to [min_us, max_us]. Guarantees the
 * returned value is always inside the hard safety limits even if
 * neutral_us itself lies outside them (result is clamped last).
 *
 * @param deg             Commanded deflection angle.
 * @param lim             Per-channel limits.
 * @param max_angle_deg   Software deflection cap (before hardware clamp).
 * @return Pulse width in microseconds within [lim.min_us, lim.max_us].
 */
uint16_t angle_to_pulse_us(float deg, const Limits &lim, float max_angle_deg);

/**
 * @brief Move `current` one tick toward `target`, limited by a max rate.
 *
 * Used by ActuatorThread to enforce a maximum rate of change on the
 * commanded pulse width. Avoids overshoot and handles current==target.
 *
 * @param current      Current pulse width (µs).
 * @param target       Target pulse width (µs).
 * @param us_per_ms    Maximum rate of change (µs per millisecond).
 * @param dt_ms        Time elapsed since the previous step.
 * @return New pulse width, never overshooting `target`.
 */
uint16_t slew_step(uint16_t current, uint16_t target, float us_per_ms, uint32_t dt_ms);

}  // namespace acs::servo_t75
