/*
 * ACS4 Flight Computer — Sensor Acquisition Threads
 *
 * Creates and manages the threads that read hardware sensors and feed
 * data into the SensorHub singleton.
 *
 * Threads created (custom PCB only — Nucleo has no on-board sensors):
 *
 *   ImuThread        prio 255 (HIGHEST)    1 kHz   2 KB stack
 *     Reads IIM-42653 accel+gyro, pushes to SensorHub.
 *     Triggered by DRDY interrupt (PAL_USE_WAIT) or 1 ms polling fallback.
 *
 *   SensorPollThread prio 180 (ABOVE_NORM) 100 Hz  2 KB stack
 *     Drives MS5611 state machine (~50 Hz output).
 *     Reads MMC5983MA magnetometer (100 Hz continuous mode).
 *     Pushes baro + mag data to SensorHub.
 *
 * Call start_sensor_threads() once from main() after all drivers are
 * initialized. On Nucleo builds this is a no-op.
 */

#pragma once

namespace acs
{

/**
 * @brief Start sensor acquisition threads.
 *
 * Must be called after halInit(), chSysInit(), and all sensor driver
 * init/configure calls have completed.
 */
void start_sensor_threads();

}  // namespace acs
