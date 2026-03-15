/*
 * ACS4 Flight Computer — MMC5983MA Magnetometer Driver
 *
 * MEMSIC MMC5983MA 3-axis AMR magnetic sensor.
 * Communicates over SPI (Mode 3, max 10 MHz).
 *
 * Features:
 *   - ±8 Gauss full-scale range, 18-bit resolution
 *   - Continuous measurement mode up to 1000 Hz
 *   - Built-in SET/RESET for bridge offset elimination
 *   - Automatic and periodic SET/RESET degauss
 *   - On-chip temperature sensor
 *   - Output in SI units: μT
 *
 * Hardware mapping (production PCB):
 *   SPI2: PB13 (SCK) / PB14 (MISO) / PB15 (MOSI)
 *   CS:   PB12 (LINE_MAG_CS)
 *   INT:  PC5  (LINE_MAG_INT)
 *
 * Typical usage:
 *   acs::Mmc5983ma mag;
 *   mag.init(spi_bus, LINE_MAG_CS, spi_config);
 *   mag.configure(Mmc5983maConfig::rocket_default());
 *   // In MagThread (100 Hz, polling or DRDY):
 *   acs::MagSample sample;
 *   mag.read(sample);
 */

#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "hal/spi_bus.h"

namespace acs
{

/* ======================================================
 * MagSample — output data structure
 * ====================================================== */

struct MagSample
{
    /**
     * X, Y, Z in sensor-native frame (LEFT-HANDED).
     * MMC5983MA axes with pin 1 at upper-left:
     *   X+ = backward, Y+ = right, Z+ = down (into PCB)
     *
     * To convert to IMU body frame (right-handed):
     *   body_x = +field_ut[1]   (mag Y)
     *   body_y = -field_ut[0]   (mag -X)
     *   body_z = -field_ut[2]   (mag -Z)
     */

    std::array<float, 3> field_ut;     /* X, Y, Z — body frame, μT */
    uint32_t             timestamp_us; /* host μs (DWT) */
};

/* ==========================
 * MMC5983MA Register Map
 * ========================== */

namespace mmc5983ma_reg
{

/* Data output registers (read-only) */

inline constexpr uint8_t XOUT0   = 0x00; /* Xout[17:10] */
inline constexpr uint8_t XOUT1   = 0x01; /* Xout[9:2]   */
inline constexpr uint8_t YOUT0   = 0x02; /* Yout[17:10] */
inline constexpr uint8_t YOUT1   = 0x03; /* Yout[9:2]   */
inline constexpr uint8_t ZOUT0   = 0x04; /* Zout[17:10] */
inline constexpr uint8_t ZOUT1   = 0x05; /* Zout[9:2]   */
inline constexpr uint8_t XYZOUT2 = 0x06; /* X[1:0], Y[1:0], Z[1:0] in bits [7:2] */
inline constexpr uint8_t TOUT    = 0x07; /* Temperature output */

/* Status register (R/W — write 1 to clear interrupt bits) */

inline constexpr uint8_t STATUS = 0x08;

/* Control registers (write-only) */

inline constexpr uint8_t CTRL0 = 0x09; /* Internal control 0 */
inline constexpr uint8_t CTRL1 = 0x0A; /* Internal control 1 */
inline constexpr uint8_t CTRL2 = 0x0B; /* Internal control 2 */
inline constexpr uint8_t CTRL3 = 0x0C; /* Internal control 3 */

/* Product ID (read-only) */

inline constexpr uint8_t PRODUCT_ID = 0x2F;

/* STATUS (0x08) bit definitions */

inline constexpr uint8_t MEAS_M_DONE = 0x01; /* Magnetic measurement done */
inline constexpr uint8_t MEAS_T_DONE = 0x02; /* Temperature measurement done */
inline constexpr uint8_t OTP_RD_DONE = 0x10; /* OTP read complete */

/* CTRL0 (0x09) bit definitions */

inline constexpr uint8_t TM_M             = 0x01; /* Take magnetic measurement */
inline constexpr uint8_t TM_T             = 0x02; /* Take temperature measurement */
inline constexpr uint8_t INT_MEAS_DONE_EN = 0x04; /* Interrupt on measurement done */
inline constexpr uint8_t DO_SET           = 0x08; /* SET operation (self-clearing) */
inline constexpr uint8_t DO_RESET         = 0x10; /* RESET operation (self-clearing) */
inline constexpr uint8_t AUTO_SR_EN       = 0x20; /* Automatic SET/RESET */
inline constexpr uint8_t OTP_READ         = 0x40; /* Re-read OTP (self-clearing) */

/* CTRL1 (0x0A) bit definitions */

inline constexpr uint8_t BW_MASK = 0x03; /* BW[1:0] filter bandwidth */
inline constexpr uint8_t SW_RST  = 0x80; /* Software reset */

/* CTRL2 (0x0B) bit definitions */

inline constexpr uint8_t CM_FREQ_MASK = 0x07; /* CM_Freq[2:0] */
inline constexpr uint8_t CMM_EN       = 0x08; /* Continuous measurement mode enable */
inline constexpr uint8_t PRD_SET_MASK = 0x70; /* Prd_set[2:0] */
inline constexpr uint8_t EN_PRD_SET   = 0x80; /* Enable periodic SET */

/* Product ID expected value: 0b00110000 */

inline constexpr uint8_t PRODUCT_ID_VALUE = 0x30;

}  // namespace mmc5983ma_reg

/* ==============================
 * Configuration enums
 * ============================== */

/** Measurement bandwidth. Controls decimation filter length and max output rate. */
enum class MagBandwidth : uint8_t
{
    HZ_100 = 0x00, /* 8 ms measurement, max 100 Hz (BW=00) */
    HZ_200 = 0x01, /* 4 ms measurement, max 200 Hz (BW=01) */
    HZ_400 = 0x02, /* 2 ms measurement, max 400 Hz (BW=10) */
    HZ_800 = 0x03, /* 0.5 ms measurement, max 800 Hz (BW=11) */
};

/** Continuous measurement frequency. Requires compatible BW setting. */
enum class MagCmFreq : uint8_t
{
    OFF     = 0x00,
    HZ_1    = 0x01,
    HZ_10   = 0x02,
    HZ_20   = 0x03,
    HZ_50   = 0x04,
    HZ_100  = 0x05,
    HZ_200  = 0x06, /* requires BW >= HZ_200 */
    HZ_1000 = 0x07, /* requires BW = HZ_800 */
};

/** Periodic SET interval (measurements between automatic SET operations). */
enum class MagPeriodicSet : uint8_t
{
    EVERY_1    = 0x00,
    EVERY_25   = 0x01,
    EVERY_75   = 0x02,
    EVERY_100  = 0x03,
    EVERY_250  = 0x04,
    EVERY_500  = 0x05,
    EVERY_1000 = 0x06,
    EVERY_2000 = 0x07,
};

/* =========================
 * Driver Configuration
 * ========================= */

struct Mmc5983maConfig
{
    MagBandwidth   bandwidth;
    MagCmFreq      cm_freq;
    bool           auto_set_reset;
    bool           periodic_set;
    MagPeriodicSet periodic_set_interval;
    bool           enable_int;

    /**
     * @brief Rocket default: 100 Hz continuous, BW=400 Hz filter,
     *        automatic SET/RESET, periodic SET every 100 measurements (~1 s).
     *
     * BW=400 Hz (2 ms measurement) gives 2x timing headroom over 100 Hz
     * with Auto_SR_en (max ~225 Hz). Noise penalty vs BW=200 Hz is
     * 0.8 mG vs 0.6 mG — negligible against Earth's field (25–65 μT).
     */
    static constexpr Mmc5983maConfig rocket_default()
    {
        return {
            .bandwidth             = MagBandwidth::HZ_400,
            .cm_freq               = MagCmFreq::HZ_100,
            .auto_set_reset        = true,
            .periodic_set          = true,
            .periodic_set_interval = MagPeriodicSet::EVERY_100,
            .enable_int            = false,
        };
    }
};

/* ==========================
 * MMC5983MA Driver Class
 * ========================== */

class Mmc5983ma
{
  public:
    Mmc5983ma()                               = default;
    ~Mmc5983ma()                              = default;
    Mmc5983ma(const Mmc5983ma &)              = delete;
    Mmc5983ma &operator=(const Mmc5983ma &)   = delete;
    Mmc5983ma(Mmc5983ma &&)                   = delete;
    Mmc5983ma &operator=(Mmc5983ma &&)        = delete;

    /**
     * @brief Initialize the magnetometer.
     *
     * Performs:
     *   1. Software reset (SW_RST), wait 15 ms
     *   2. Product ID verification (expect 0x30)
     *   3. Initial SET operation for known sensor polarity
     *
     * @param spi        Reference to initialized SpiBus.
     * @param cs_line    PAL line for chip select.
     * @param spi_cfg    SPI configuration (CPOL/CPHA/prescaler).
     * @return true on success, false on comm failure or wrong Product ID.
     */
    [[nodiscard]] bool init(SpiBus &spi, ioline_t cs_line, const SPIConfig &spi_cfg);

    /**
     * @brief Configure measurement parameters and start continuous mode.
     *
     * Must be called after init(). Sets bandwidth, enables auto SET/RESET,
     * and starts continuous measurements at the configured rate.
     *
     * @param cfg  Configuration structure.
     * @return true on success.
     */
    [[nodiscard]] bool configure(const Mmc5983maConfig &cfg);

    /**
     * @brief Read 18-bit XYZ magnetic field data and convert to μT.
     *
     * Checks Meas_M_Done before reading; returns false if no new data
     * is available (stale/duplicate protection). In continuous mode the
     * caller should poll at or below the configured CM_Freq rate.
     *
     * @param[out] sample  Output data.
     * @return true on success, false if data not ready or comm error.
     */
    [[nodiscard]] bool read(MagSample &sample);

    /**
     * @brief Read the on-chip temperature sensor.
     *
     * Triggers a one-shot temperature measurement and waits for completion.
     * Range: −75 to +125 °C, resolution ~0.8 °C/LSB.
     *
     * @param[out] temp_degc  Temperature in °C.
     * @return true on success.
     */
    [[nodiscard]] bool read_temperature(float &temp_degc);

    /**
     * @brief Execute a manual SET/RESET degauss cycle.
     *
     * Eliminates residual magnetization from exposure to strong fields
     * and recalibrates bridge offset. Auto_SR_en is re-asserted after
     * the cycle if it was previously configured.
     *
     * @return true on success.
     */
    [[nodiscard]] bool degauss();

    /**
     * @brief Check if magnetic measurement data is ready.
     *
     * @return true if Meas_M_Done bit is set in Status register.
     */
    [[nodiscard]] bool data_ready();

    [[nodiscard]] bool is_initialized() const
    {
        return initialized_;
    }

    [[nodiscard]] uint32_t error_count() const
    {
        return error_count_;
    }

  private:
    /* Register access helpers */

    [[nodiscard]] bool                   write_reg(uint8_t reg, uint8_t value);
    [[nodiscard]] std::optional<uint8_t> read_reg(uint8_t reg);
    [[nodiscard]] bool                   read_regs(uint8_t start_reg, uint8_t *buf, size_t len);

    /**
     * @brief Compute the CTRL0 base value with persistent mode bits.
     *
     * CTRL0 is write-only; command bits (SET, RESET, TM_M, TM_T) are
     * self-clearing, but mode bits (Auto_SR_en, INT_meas_done_en) must
     * be re-written with every CTRL0 access to remain active.
     */
    [[nodiscard]] uint8_t ctrl0_base() const;

    /* Error handling */

    void report_error();

    /* Constants */

    /** 18-bit null field output (midpoint) as float for conversion */
    static constexpr float kNullField = 131072.0f;

    /** Counts per Gauss at 18-bit resolution */
    static constexpr float kCountsPerGauss = 16384.0f;

    /** Gauss to μT conversion factor */
    static constexpr float kGaussToUt = 100.0f;

    /** Combined scale: (raw - offset) * kScale = μT */
    static constexpr float kScale = kGaussToUt / kCountsPerGauss;

    /** Temperature: T(°C) = raw * 0.8 − 75 */
    static constexpr float kTempScale  = 0.8f;
    static constexpr float kTempOffset = -75.0f;

    /** 18-bit data burst: 7 bytes (Xout0..Zout1 + XYZout2) */
    static constexpr size_t kBurstLen = 7;

    /* State */

    SpiBus          *spi_         = nullptr;
    ioline_t         cs_line_     = 0;
    const SPIConfig *spi_cfg_     = nullptr;
    Mmc5983maConfig  config_      = {};
    bool             initialized_ = false;
    uint32_t         error_count_ = 0;
};

/**
 * @brief Get pointer to the global magnetometer driver instance.
 * @return Pointer to initialized Mmc5983ma, or nullptr if unavailable.
 */
Mmc5983ma *mag_instance();

}  // namespace acs
