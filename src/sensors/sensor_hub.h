/*
 * ACS4 Flight Computer — Sensor Hub
 *
 * Aggregates data from all on-board sensors into a single timestamped
 * snapshot in a unified board frame, providing thread-safe access for
 * the navigation and FSM layers.
 *
 * Board frame convention (matches IMU, pin-1 at upper-left, top view):
 *   X+ = right
 *   Y+ = forward
 *   Z+ = up
 *
 * Producer: the thread(s) that call update_imu / update_baro / update_mag.
 * Consumer: NavThread (or any reader) calls snapshot() to get an atomic copy.
 */

#pragma once

#include <array>
#include <cstdint>

namespace acs
{

/* =====================================================================
 * SensorSnapshot — all sensor data in one board-frame package
 * ===================================================================== */

struct SensorSnapshot
{
    /* IMU (IIM-42653, up to 1 kHz) — board frame */
    uint32_t             imu_timestamp_us; /* host µs of the most recent IMU sample */
    std::array<float, 3> accel_mps2;       /* m/s² */
    std::array<float, 3> gyro_rads;        /* rad/s */
    float                imu_temp_c;       /* die temperature, °C */
    bool                 imu_valid;        /* true after first successful read */

    /* Barometer (MS5611, ~50 Hz) */
    uint32_t baro_timestamp_us; /* host µs of the most recent baro sample */
    float    pressure_pa;       /* Pa */
    float    baro_temp_c;       /* °C */
    float    altitude_m;        /* barometric altitude (QNH-referenced) */
    bool     baro_valid;        /* true after first compensated sample */
    bool     baro_fresh;        /* true only when a new sample arrived since last snapshot() */

    /* Magnetometer (MMC5983MA, 100 Hz) — board frame */
    uint32_t             mag_timestamp_us; /* host µs of the most recent mag sample */
    std::array<float, 3> mag_ut;           /* µT */
    bool                 mag_valid;
    bool                 mag_fresh;
};

/* =====================================================================
 * SensorHub
 * ===================================================================== */

class SensorHub
{
  public:
    SensorHub()  = default;
    ~SensorHub() = default;

    SensorHub(const SensorHub &)            = delete;
    SensorHub &operator=(const SensorHub &) = delete;

    /**
     * @brief Store a new IMU sample in sensor-native frame.
     *
     * The hub applies the IMU→board rotation internally. The rotation is
     * selected at compile time:
     *   - default (Sigman):    identity (sensor frame == board frame)
     *   - ACS4_LAYOUT_JEDRZEJ: board_x = +sensor_y, board_y = -sensor_x,
     *                          board_z = +sensor_z
     *
     * Called from ImuThread at up to 1 kHz.
     */
    void update_imu(const std::array<float, 3> &accel_mps2,
                    const std::array<float, 3> &gyro_rads,
                    float                       temp_c,
                    uint32_t                    timestamp_us);

    /**
     * @brief Store a new barometer sample.
     *
     * Called from NavThread after Ms5611::update() produces data.
     */
    void update_baro(float    pressure_pa,
                     float    temperature_c,
                     float    altitude_m,
                     uint32_t timestamp_us);

    /**
     * @brief Store a new magnetometer sample in sensor-native frame.
     *
     * The hub applies the MAG→board rotation internally. The rotation is
     * selected at compile time:
     *   - default (Sigman):    board_x = +mag_y, board_y = -mag_x,
     *                          board_z = -mag_z
     *   - ACS4_LAYOUT_JEDRZEJ: board_x = -mag_y, board_y = +mag_x,
     *                          board_z = -mag_z
     *
     * @param mag_raw_ut  XYZ in MMC5983MA sensor frame (µT).
     */
    void update_mag(const std::array<float, 3> &mag_raw_ut,
                    uint32_t                    timestamp_us);

    /**
     * @brief Get an atomic copy of the current sensor state.
     *
     * baro_fresh / mag_fresh are set true only once per new sample and
     * are cleared after this call returns.
     *
     * @note Only one consumer should rely on the _fresh flags; a second
     *       caller would always see them as false.
     */
    SensorSnapshot snapshot();

  private:
    SensorSnapshot data_{};
};

/**
 * @brief Get reference to the global SensorHub singleton.
 */
SensorHub &sensor_hub();

}  // namespace acs
