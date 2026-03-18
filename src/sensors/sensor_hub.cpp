/*
 * ACS4 Flight Computer — Sensor Hub Implementation
 */

#include "sensors/sensor_hub.h"

extern "C" {
#include "ch.h"
}

namespace acs
{

/* Globalny singleton */

static SensorHub g_hub;

SensorHub &sensor_hub()
{
    return g_hub;
}

/* IMU (identity transform — board frame == IMU frame) */

void SensorHub::update_imu(const std::array<float, 3> &accel_mps2,
                           const std::array<float, 3> &gyro_rads,
                           float                       temp_c,
                           uint32_t                    timestamp_us)
{
    chSysLock();
    data_.accel_mps2       = accel_mps2;
    data_.gyro_rads        = gyro_rads;
    data_.imu_temp_c       = temp_c;
    data_.imu_timestamp_us = timestamp_us;
    data_.imu_valid        = true;
    chSysUnlock();
}

/* Barometr */

void SensorHub::update_baro(float    pressure_pa,
                            float    temperature_c,
                            float    altitude_m,
                            uint32_t timestamp_us)
{
    chSysLock();
    data_.pressure_pa       = pressure_pa;
    data_.baro_temp_c       = temperature_c;
    data_.altitude_m        = altitude_m;
    data_.baro_timestamp_us = timestamp_us;
    data_.baro_valid        = true;
    data_.baro_fresh        = true;
    chSysUnlock();
}

/* Magnetometr (sensor frame → board frame)
 *
 * Sensor natywnie (MMC5983MA, pin-1 upper-left, after SET):
 *   X+ = backward,  Y+ = right,  Z+ = down
 *
 * Board frame (= IMU frame):
 *   X+ = right,  Y+ = forward,  Z+ = up
 *
 * Rotation:
 *   board_x = +sensor_y
 *   board_y = -sensor_x
 *   board_z = -sensor_z
 *
 */

void SensorHub::update_mag(const std::array<float, 3> &mag_raw_ut,
                           uint32_t                    timestamp_us)
{
    chSysLock();
    data_.mag_ut[0]         = +mag_raw_ut[1];
    data_.mag_ut[1]         = -mag_raw_ut[0];
    data_.mag_ut[2]         = -mag_raw_ut[2];
    data_.mag_timestamp_us  = timestamp_us;
    data_.mag_valid         = true;
    data_.mag_fresh         = true;
    chSysUnlock();
}

/* Atomiczny snapshot (consumer side) */

SensorSnapshot SensorHub::snapshot()
{
    SensorSnapshot copy{};

    chSysLock();
    copy             = data_;
    data_.baro_fresh = false;
    data_.mag_fresh  = false;
    chSysUnlock();

    return copy;
}

}  // namespace acs
