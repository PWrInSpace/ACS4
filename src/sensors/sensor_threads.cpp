/*
 * ACS4 Flight Computer — Sensor Acquisition Threads
 *
 * ImuThread (1 kHz, prio 255):
 *   Reads IIM-42653 via DRDY wait (PAL_USE_WAIT) or 1 ms polling.
 *   Pushes accel+gyro+temp into SensorHub.
 *
 * SensorPollThread (100 Hz, prio 180):
 *   Ticks MS5611 state machine (50 Hz output at OSR 4096).
 *   Reads MMC5983MA in continuous mode (100 Hz).
 *   Pushes baro+mag into SensorHub.
 */

#include "sensors/sensor_threads.h"

#include "drivers/iim42653.h"
#include "drivers/mmc5983ma.h"
#include "drivers/ms5611.h"
#include "sensors/sensor_hub.h"
#include "system/watchdog.h"
#include "utils/profiler.h"

extern "C" {
#include "ch.h"

#include "hal.h"
}

#if defined(STM32H725xx)

namespace acs
{

/* =====================================================================
 * ImuThread — 1 kHz IMU acquisition
 * ===================================================================== */

static int g_prof_imu = -1;
static int g_wdg_imu  = -1;

static THD_WORKING_AREA(waImuThread, 2048);

static THD_FUNCTION(ImuThread, arg)
{
    (void)arg;
    chRegSetThreadName("imu");

    g_prof_imu = profiler_register("imu");
    g_wdg_imu  = watchdog_register("imu", 50);

    auto *imu = imu_instance();
    if (imu == nullptr)
    {
        return;
    }

#if PAL_USE_WAIT == TRUE
    palSetLineMode(LINE_IMU_INT1, PAL_MODE_INPUT);
    palEnableLineEvent(LINE_IMU_INT1, PAL_EVENT_MODE_RISING_EDGE); // NOLINT(cppcoreguidelines-avoid-do-while)
#else
    systime_t next = chVTGetSystemTimeX();
#endif

    while (true)
    {
#if PAL_USE_WAIT == TRUE
        palWaitLineTimeout(LINE_IMU_INT1, TIME_MS2I(5));
#else
        next += TIME_MS2I(1);
#endif

        PROFILE_BEGIN(g_prof_imu);

        ImuSample sample{};
        if (imu->read(sample))
        {
            sensor_hub().update_imu(
                sample.accel_mps2, sample.gyro_rads, sample.temp_degc, sample.timestamp_us);
        }

        PROFILE_END(g_prof_imu);

        if (g_wdg_imu >= 0)
        {
            watchdog_feed(g_wdg_imu);
        }

#if PAL_USE_WAIT != TRUE
        chThdSleepUntil(next);
#endif
    }
}

/* =====================================================================
 * SensorPollThread — 100 Hz baro + mag
 * ===================================================================== */

static int g_prof_spoll = -1;
static int g_wdg_spoll  = -1;

static THD_WORKING_AREA(waSensorPollThread, 2048);

static THD_FUNCTION(SensorPollThread, arg)
{
    (void)arg;
    chRegSetThreadName("sensor_poll");

    g_prof_spoll = profiler_register("sensor_poll");
    g_wdg_spoll  = watchdog_register("sensor_poll", 200);

    auto *baro = baro_instance();
    auto *mag  = mag_instance();

    systime_t next = chVTGetSystemTimeX();

    while (true)
    {
        next += TIME_MS2I(10);

        PROFILE_BEGIN(g_prof_spoll);

        if (baro != nullptr)
        {
            baro->update();
            if (baro->has_new_data())
            {
                const BaroSample s = baro->sample();
                sensor_hub().update_baro(s.pressure_pa, s.temperature_c, s.altitude_m, s.timestamp_us);
            }
        }

        if (mag != nullptr)
        {
            MagSample msample{};
            if (mag->read(msample))
            {
                sensor_hub().update_mag(msample.field_ut, msample.timestamp_us);
            }
        }

        PROFILE_END(g_prof_spoll);

        if (g_wdg_spoll >= 0)
        {
            watchdog_feed(g_wdg_spoll);
        }

        chThdSleepUntil(next);
    }
}

/* =====================================================================
 * Public API
 * ===================================================================== */

void start_sensor_threads()
{
    chThdCreateStatic(waImuThread, sizeof(waImuThread), HIGHPRIO, ImuThread, nullptr);

    chThdCreateStatic(
        waSensorPollThread, sizeof(waSensorPollThread), NORMALPRIO + 52, SensorPollThread, nullptr);
}

}  // namespace acs

#else /* NUCLEO_H723 — no on-board sensors */

namespace acs
{

void start_sensor_threads()
{
}

}  // namespace acs

#endif
