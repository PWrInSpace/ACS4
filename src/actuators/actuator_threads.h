/*
 * ACS4 Flight Computer — Actuator Thread
 *
 * ActuatorThread (100 Hz, prio NORMALPRIO + 20):
 *   Reads commanded angles from ActuatorHub, applies per-fin triangle
 *   sweep (if active), pushes through ServoBankT75's slew limiter,
 *   writes pulse widths to TIM4 PWM channels, feeds back current pulse
 *   widths to the hub.
 *
 * Call start_actuator_threads() once from main() after the servo bank
 * has been initialized. On Nucleo builds (no PCB, no servos) this is
 * a no-op.
 */

#pragma once

namespace acs
{

void start_actuator_threads();

}  // namespace acs
