/*
 * ACS4 Flight Computer — IIM-42653 IMU Driver
 *
 * TDK InvenSense IIM-42653 6-axis IMU (3-axis accel + 3-axis gyro).
 * Communicates over SPI (Mode 0 or 3, max 24 MHz).
 *
 * Features:
 *   - Configurable ODR (up to 32 kHz) and FSR
 *   - Burst read of accel + gyro + temperature in 14 bytes
 *   - DRDY interrupt on INT1 (push-pull, active-high)
 *   - Built-in self-test with factory reference comparison
 *   - Multi-bank register access
 *   - Output in SI units: m/s², rad/s, °C
 *
 * Hardware mapping (production PCB):
 *   SPI2: PB13 (SCK) / PB14 (MISO) / PB15 (MOSI)
 *   CS:   PD8  (LINE_IMU_CS)
 *   INT1: PD10 (LINE_IMU_INT1)
 *   INT2: PD9  (LINE_IMU_INT2)
 *
 * Typical usage:
 *   acs::Iim42653 imu;
 *   imu.init(spi_bus, LINE_IMU_CS, spi_config);
 *   imu.configure(Iim42653::Config::rocket_default());
 *   // In ImuThread (1 kHz, triggered by DRDY):
 *   acs::ImuSample sample;
 *   imu.read(sample);
 */

#pragma once

#include <cstdint>

#include "hal/spi_bus.h"

namespace acs
{

/* ═══════════════════════════════════════════════════════════════════════════
 * ImuSample — output data structure posted to mailbox
 * ═══════════════════════════════════════════════════════════════════════════ */

struct ImuSample
{
    float    accel_mps2[3]; /* X, Y, Z — body frame, m/s² */
    float    gyro_rads[3];  /* X, Y, Z — body frame, rad/s */
    float    temp_degc;     /* die temperature, °C */
    uint32_t timestamp_us;  /* host µs — DWT (register) or reconstructed (FIFO) */
};

/* ═══════════════════════════════════════════════════════════════════════════
 * IIM-42653 Register Map
 * ═══════════════════════════════════════════════════════════════════════════ */

namespace iim42653_reg
{

/* ── Bank 0 (default) ──────────────────────────────────────────────────── */

inline constexpr uint8_t DEVICE_CONFIG      = 0x11;
inline constexpr uint8_t DRIVE_CONFIG       = 0x13;
inline constexpr uint8_t INT_CONFIG         = 0x14;
inline constexpr uint8_t FIFO_CONFIG        = 0x16;
inline constexpr uint8_t TEMP_DATA1         = 0x1D;
inline constexpr uint8_t TEMP_DATA0         = 0x1E;
inline constexpr uint8_t ACCEL_DATA_X1      = 0x1F;
inline constexpr uint8_t ACCEL_DATA_X0      = 0x20;
inline constexpr uint8_t ACCEL_DATA_Y1      = 0x21;
inline constexpr uint8_t ACCEL_DATA_Y0      = 0x22;
inline constexpr uint8_t ACCEL_DATA_Z1      = 0x23;
inline constexpr uint8_t ACCEL_DATA_Z0      = 0x24;
inline constexpr uint8_t GYRO_DATA_X1       = 0x25;
inline constexpr uint8_t GYRO_DATA_X0       = 0x26;
inline constexpr uint8_t GYRO_DATA_Y1       = 0x27;
inline constexpr uint8_t GYRO_DATA_Y0       = 0x28;
inline constexpr uint8_t GYRO_DATA_Z1       = 0x29;
inline constexpr uint8_t GYRO_DATA_Z0       = 0x2A;
inline constexpr uint8_t INT_STATUS         = 0x2D;
inline constexpr uint8_t FIFO_COUNTH        = 0x2E;
inline constexpr uint8_t FIFO_COUNTL        = 0x2F;
inline constexpr uint8_t FIFO_DATA          = 0x30;
inline constexpr uint8_t SIGNAL_PATH_RESET  = 0x4B;
inline constexpr uint8_t INTF_CONFIG0       = 0x4C;
inline constexpr uint8_t INTF_CONFIG1       = 0x4D;
inline constexpr uint8_t PWR_MGMT0          = 0x4E;
inline constexpr uint8_t GYRO_CONFIG0       = 0x4F;
inline constexpr uint8_t ACCEL_CONFIG0      = 0x50;
inline constexpr uint8_t GYRO_CONFIG1       = 0x51;
inline constexpr uint8_t GYRO_ACCEL_CONFIG0 = 0x52;
inline constexpr uint8_t ACCEL_CONFIG1      = 0x53;
inline constexpr uint8_t TMST_CONFIG        = 0x54;
inline constexpr uint8_t FIFO_CONFIG1       = 0x5F;
inline constexpr uint8_t FIFO_CONFIG2       = 0x60;
inline constexpr uint8_t FIFO_CONFIG3       = 0x61;
inline constexpr uint8_t INT_CONFIG0        = 0x63;
inline constexpr uint8_t INT_CONFIG1        = 0x64;
inline constexpr uint8_t INT_SOURCE0        = 0x65;
inline constexpr uint8_t INT_SOURCE1        = 0x66;
inline constexpr uint8_t INT_SOURCE3        = 0x68;
inline constexpr uint8_t SELF_TEST_CONFIG   = 0x70;
inline constexpr uint8_t WHO_AM_I           = 0x75;
inline constexpr uint8_t REG_BANK_SEL       = 0x76;

/* ── Bank 1 ────────────────────────────────────────────────────────────── */

inline constexpr uint8_t SENSOR_CONFIG0      = 0x03; /* Bank 1 */
inline constexpr uint8_t GYRO_CONFIG_STATIC2  = 0x0B; /* Bank 1 */
inline constexpr uint8_t GYRO_CONFIG_STATIC3  = 0x0C; /* Bank 1 */
inline constexpr uint8_t GYRO_CONFIG_STATIC4  = 0x0D; /* Bank 1 */
inline constexpr uint8_t GYRO_CONFIG_STATIC5  = 0x0E; /* Bank 1 */
inline constexpr uint8_t GYRO_CONFIG_STATIC6  = 0x0F; /* Bank 1 — X NF_COSWZ[7:0] */
inline constexpr uint8_t GYRO_CONFIG_STATIC7  = 0x10; /* Bank 1 — Y NF_COSWZ[7:0] */
inline constexpr uint8_t GYRO_CONFIG_STATIC8  = 0x11; /* Bank 1 — Z NF_COSWZ[7:0] */
inline constexpr uint8_t GYRO_CONFIG_STATIC9  = 0x12; /* Bank 1 — NF_COSWZ[8] + SEL */
inline constexpr uint8_t GYRO_CONFIG_STATIC10 = 0x13; /* Bank 1 — NF_BW_SEL [6:4] */
inline constexpr uint8_t XG_ST_DATA           = 0x5F; /* Bank 1 */
inline constexpr uint8_t YG_ST_DATA          = 0x60; /* Bank 1 */
inline constexpr uint8_t ZG_ST_DATA          = 0x61; /* Bank 1 */
inline constexpr uint8_t INTF_CONFIG4        = 0x7A; /* Bank 1 */
inline constexpr uint8_t INTF_CONFIG5        = 0x7B; /* Bank 1 */
inline constexpr uint8_t INTF_CONFIG6        = 0x7C; /* Bank 1 */

/* ── Bank 2 ────────────────────────────────────────────────────────────── */

inline constexpr uint8_t ACCEL_CONFIG_STATIC2 = 0x03; /* Bank 2 */
inline constexpr uint8_t ACCEL_CONFIG_STATIC3 = 0x04; /* Bank 2 */
inline constexpr uint8_t ACCEL_CONFIG_STATIC4 = 0x05; /* Bank 2 */
inline constexpr uint8_t XA_ST_DATA           = 0x3B; /* Bank 2 */
inline constexpr uint8_t YA_ST_DATA           = 0x3C; /* Bank 2 */
inline constexpr uint8_t ZA_ST_DATA           = 0x3D; /* Bank 2 */

/* ── Bank 4 ────────────────────────────────────────────────────────────── */

inline constexpr uint8_t OFFSET_USER0 = 0x77; /* Bank 4 — gyro X low[7:0] */
inline constexpr uint8_t OFFSET_USER1 = 0x78; /* Bank 4 — gyro X hi[3:0] | gyro Y hi[7:4] */
inline constexpr uint8_t OFFSET_USER2 = 0x79; /* Bank 4 — gyro Y low[7:0] */
inline constexpr uint8_t OFFSET_USER3 = 0x7A; /* Bank 4 — gyro Z low[7:0] */
inline constexpr uint8_t OFFSET_USER4 = 0x7B; /* Bank 4 — gyro Z hi[3:0] | accel X hi[7:4] */
inline constexpr uint8_t OFFSET_USER5 = 0x7C; /* Bank 4 — accel X low[7:0] */
inline constexpr uint8_t OFFSET_USER6 = 0x7D; /* Bank 4 — accel Y low[7:0] */
inline constexpr uint8_t OFFSET_USER7 = 0x7E; /* Bank 4 — accel Y hi[3:0] | accel Z hi[7:4] */
inline constexpr uint8_t OFFSET_USER8 = 0x7F; /* Bank 4 — accel Z low[7:0] */

/* ── Constants ─────────────────────────────────────────────────────────── */

inline constexpr uint8_t WHO_AM_I_VALUE = 0x56;

}  // namespace iim42653_reg

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuration Enumerations
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Gyroscope full-scale range. Bits [7:5] of GYRO_CONFIG0. */
enum class GyroFsr : uint8_t
{
    DPS_4000  = 0 << 5, /* ±4000 °/s,  sensitivity =    8.2 LSB/°/s */
    DPS_2000  = 1 << 5, /* ±2000 °/s,  sensitivity =   16.4 LSB/°/s */
    DPS_1000  = 2 << 5, /* ±1000 °/s,  sensitivity =   32.8 LSB/°/s */
    DPS_500   = 3 << 5, /* ±500  °/s,  sensitivity =   65.5 LSB/°/s */
    DPS_250   = 4 << 5, /* ±250  °/s,  sensitivity =  131.0 LSB/°/s */
    DPS_125   = 5 << 5, /* ±125  °/s,  sensitivity =  262.0 LSB/°/s */
    DPS_62_5  = 6 << 5, /* ±62.5 °/s,  sensitivity =  524.3 LSB/°/s */
    DPS_31_25 = 7 << 5, /* ±31.25°/s,  sensitivity = 1048.6 LSB/°/s */
};

/** Accelerometer full-scale range. Bits [7:5] of ACCEL_CONFIG0. */
enum class AccelFsr : uint8_t
{
    G_32 = 0 << 5, /* ±32 g, sensitivity = 1024 LSB/g */
    G_16 = 1 << 5, /* ±16 g, sensitivity = 2048 LSB/g */
    G_8  = 2 << 5, /* ±8  g, sensitivity = 4096 LSB/g */
    G_4  = 3 << 5, /* ±4  g, sensitivity = 8192 LSB/g */
};

/** Gyroscope output data rate. Bits [3:0] of GYRO_CONFIG0. */
enum class GyroOdr : uint8_t
{
    HZ_32000 = 0x01,
    HZ_16000 = 0x02,
    HZ_8000  = 0x03,
    HZ_4000  = 0x04,
    HZ_2000  = 0x05,
    HZ_1000  = 0x06, /* default */
    HZ_500   = 0x0F,
    HZ_200   = 0x07,
    HZ_100   = 0x08,
    HZ_50    = 0x09,
    HZ_25    = 0x0A,
    HZ_12_5  = 0x0B,
};

/** Accelerometer output data rate. Bits [3:0] of ACCEL_CONFIG0. */
enum class AccelOdr : uint8_t
{
    HZ_32000 = 0x01,
    HZ_16000 = 0x02,
    HZ_8000  = 0x03,
    HZ_4000  = 0x04,
    HZ_2000  = 0x05,
    HZ_1000  = 0x06, /* default */
    HZ_500   = 0x0F,
    HZ_200   = 0x07,
    HZ_100   = 0x08,
    HZ_50    = 0x09,
    HZ_25    = 0x0A,
    HZ_12_5  = 0x0B,
};

/** UI filter order (for both gyro and accel). */
enum class FilterOrder : uint8_t
{
    FIRST  = 0,
    SECOND = 1,
    THIRD  = 2,
};

/** UI filter bandwidth selection (fraction of ODR). LN mode. */
enum class FilterBw : uint8_t
{
    ODR_DIV_2  = 0,
    ODR_DIV_4  = 1,
    ODR_DIV_5  = 2,
    ODR_DIV_8  = 3,
    ODR_DIV_10 = 4,
    ODR_DIV_16 = 5,
    ODR_DIV_20 = 6,
    ODR_DIV_40 = 7,
};

/** Temperature sensor filter bandwidth. Bits [7:5] of GYRO_CONFIG1. */
enum class TempFilterBw : uint8_t
{
    HZ_4000 = 0, /* default — not recommended (noisy) */
    HZ_170  = 1,
    HZ_82   = 2,
    HZ_40   = 3, /* recommended for flight */
    HZ_20   = 4,
    HZ_10   = 5,
    HZ_5    = 6,
};

/** Gyro notch filter bandwidth. Bits [6:4] of GYRO_CONFIG_STATIC10 (Bank 1, 0x13h). */
enum class NotchBw : uint8_t
{
    HZ_1449 = 0,
    HZ_680  = 1,
    HZ_329  = 2,
    HZ_162  = 3,
    HZ_80   = 4,
    HZ_40   = 5,
    HZ_20   = 6,
    HZ_10   = 7,
};

/**
 * Anti-Alias Filter bandwidth preset.
 *
 * Each entry encodes the three register parameters (DELT, DELTSQR, BITSHIFT)
 * needed by the hardware. Values taken from datasheet section 5.3.
 */
struct AafBw
{
    uint8_t  delt;
    uint16_t deltsqr;
    uint8_t  bitshift;

    /** AAF disabled — hardware default (full bandwidth). */
    static constexpr AafBw disabled()
    {
        return {0, 0, 0};
    }

    /* ── Selected presets (sorted by 3 dB BW) ────────────────────────── */

    /** ~213 Hz — heavy filtering for low-vibration environments. */
    static constexpr AafBw bw_213()
    {
        return {5, 25, 10};
    }

    /** ~536 Hz — good for rocket vibration rejection at 1 kHz ODR. */
    static constexpr AafBw bw_536()
    {
        return {12, 144, 8};
    }

    /** ~997 Hz — Nyquist-friendly at 2 kHz ODR. */
    static constexpr AafBw bw_997()
    {
        return {21, 440, 6};
    }

    /** ~1962 Hz — Nyquist-friendly at 4 kHz ODR. */
    static constexpr AafBw bw_1962()
    {
        return {37, 1376, 4};
    }

    bool is_enabled() const
    {
        return delt != 0;
    }
};

/**
 * Gyro notch filter configuration.
 *
 * Suppresses MEMS sense resonance noise (1–3 kHz) per axis.
 * Set freq_hz = 0 to disable (default).
 */
struct GyroNotchConfig
{
    float   freq_hz = 0.0f; /* Notch frequency in Hz (1000–3000), 0 = disabled */
    NotchBw bw      = NotchBw::HZ_329; /* Notch bandwidth */

    bool is_enabled() const { return freq_hz >= 1000.0f && freq_hz <= 3000.0f; }

    static constexpr GyroNotchConfig disabled() { return {0.0f, NotchBw::HZ_329}; }
};

/* ═══════════════════════════════════════════════════════════════════════════
 * Driver Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

struct Iim42653Config
{
    GyroFsr      gyro_fsr;
    GyroOdr      gyro_odr;
    AccelFsr     accel_fsr;
    AccelOdr     accel_odr;
    FilterOrder  gyro_filter_order;
    FilterOrder  accel_filter_order;
    FilterBw     gyro_filter_bw;
    FilterBw     accel_filter_bw;
    TempFilterBw temp_filter_bw;
    AafBw        gyro_aaf;
    AafBw        accel_aaf;
    GyroNotchConfig gyro_notch;
    bool         enable_drdy_int1;
    bool         enable_fifo;        /* FIFO mode with sensor-side timestamps */
    uint16_t     fifo_watermark;     /* watermark in packets (1–130, default 1) */

    /**
     * @brief Default configuration for rocket flight (register-read mode):
     *   - Gyro:  ±2000 dps @ 1 kHz, 2nd-order filter BW = ODR/4
     *   - Accel: ±32 g    @ 1 kHz, 2nd-order filter BW = ODR/4
     *   - AAF:   ~536 Hz on both (rejects motor vibration aliasing)
     *   - Notch: enabled at MEMS resonance (~2.5 kHz), BW = 329 Hz
     *   - Temp filter: 40 Hz (low noise for slow-varying temperature)
     *   - DRDY interrupt on INT1, push-pull, active-high
     */
    static constexpr Iim42653Config rocket_default()
    {
        return {
            .gyro_fsr           = GyroFsr::DPS_2000,
            .gyro_odr           = GyroOdr::HZ_1000,
            .accel_fsr          = AccelFsr::G_32,
            .accel_odr          = AccelOdr::HZ_1000,
            .gyro_filter_order  = FilterOrder::SECOND,
            .accel_filter_order = FilterOrder::SECOND,
            .gyro_filter_bw     = FilterBw::ODR_DIV_4,
            .accel_filter_bw    = FilterBw::ODR_DIV_4,
            .temp_filter_bw     = TempFilterBw::HZ_40,
            .gyro_aaf           = AafBw::bw_536(),
            .accel_aaf          = AafBw::bw_536(),
            .gyro_notch         = {2500.0f, NotchBw::HZ_329},
            .enable_drdy_int1   = true,
            .enable_fifo        = false,
            .fifo_watermark     = 1,
        };
    }

    /**
     * @brief Rocket configuration with FIFO and sensor-side timestamps.
     *
     * Same filtering as rocket_default() but uses FIFO Packet 3
     * (accel + gyro + 8-bit temp + 16-bit ODR timestamp, 16 bytes).
     * INT1 fires on FIFO watermark threshold instead of DRDY.
     *
     * Watermark = 1 keeps latency identical to DRDY mode while
     * providing sensor-side timestamps. For lower CPU usage at the
     * cost of added latency, increase watermark (e.g. 5–10).
     */
    static constexpr Iim42653Config rocket_fifo()
    {
        return {
            .gyro_fsr           = GyroFsr::DPS_2000,
            .gyro_odr           = GyroOdr::HZ_1000,
            .accel_fsr          = AccelFsr::G_32,
            .accel_odr          = AccelOdr::HZ_1000,
            .gyro_filter_order  = FilterOrder::SECOND,
            .accel_filter_order = FilterOrder::SECOND,
            .gyro_filter_bw     = FilterBw::ODR_DIV_4,
            .accel_filter_bw    = FilterBw::ODR_DIV_4,
            .temp_filter_bw     = TempFilterBw::HZ_40,
            .gyro_aaf           = AafBw::bw_536(),
            .accel_aaf          = AafBw::bw_536(),
            .gyro_notch         = {2500.0f, NotchBw::HZ_329},
            .enable_drdy_int1   = true,
            .enable_fifo        = true,
            .fifo_watermark     = 1,
        };
    }
};

/* ═══════════════════════════════════════════════════════════════════════════
 * IIM-42653 Driver Class
 * ═══════════════════════════════════════════════════════════════════════════ */

class Iim42653
{
  public:
    Iim42653()                            = default;
    ~Iim42653()                           = default;
    Iim42653(const Iim42653 &)            = delete;
    Iim42653 &operator=(const Iim42653 &) = delete;
    Iim42653(Iim42653 &&)                 = delete;
    Iim42653 &operator=(Iim42653 &&)      = delete;

    /**
     * @brief Initialize the IMU.
     *
     * Performs:
     *   1. Soft reset
     *   2. WHO_AM_I verification (expect 0x56)
     *   3. Interface configuration (SPI 4-wire, big-endian)
     *   4. Clock source selection (PLL auto)
     *
     * @param spi        Reference to initialized SpiBus.
     * @param cs_line    PAL line for chip select.
     * @param spi_cfg    SPI configuration (CPOL/CPHA/prescaler).
     * @return true on success, false on comm failure or wrong WHO_AM_I.
     */
    [[nodiscard]] bool init(SpiBus &spi, ioline_t cs_line, const SPIConfig &spi_cfg);

    /**
     * @brief Configure ODR, FSR, filters, and interrupts.
     *
     * Must be called after init() and before read(). Sensors are
     * powered on at the end of this call.
     *
     * @param cfg  Configuration structure.
     * @return true on success.
     */
    [[nodiscard]] bool configure(const Iim42653Config &cfg);

    /**
     * @brief Burst-read accel + gyro + temperature.
     *
     * Reads 14 bytes from TEMP_DATA1 (0x1D) through GYRO_DATA_Z0 (0x2A)
     * in a single SPI transaction. Converts to SI units.
     *
     * @param[out] sample  Output data.
     * @return true on success.
     */
    [[nodiscard]] bool read(ImuSample &sample);

    /**
     * @brief Read available FIFO packets into caller-provided buffer.
     *
     * Reads FIFO byte count, then drains up to @p max_count Packet 3
     * frames (16 bytes each: header + accel + gyro + 8-bit temp +
     * 16-bit ODR timestamp). Converts to SI units.
     *
     * Sensor-side timestamps are reconstructed into absolute host µs:
     * the newest packet gets the current DWT time, and earlier packets
     * are offset by the inter-sample sensor timestamp deltas.
     *
     * @param[out] samples  Output buffer (caller-allocated).
     * @param      max_count  Maximum number of samples to read.
     * @return Number of valid samples written (0 on error or FIFO empty).
     *
     * @pre configure() called with enable_fifo = true.
     * @note Timestamp reconstruction is accurate for ODR ≥ 32 Hz.
     */
    [[nodiscard]] size_t read_fifo(ImuSample *samples, size_t max_count);

    /**
     * @brief Flush (reset) the FIFO contents.
     *
     * @return true on success.
     */
    [[nodiscard]] bool flush_fifo();

    /**
     *
     * Measures sensor output with self-test enabled and disabled,
     * compares delta against factory reference values per axis.
     *
     * @return true if all 6 axes pass.
     */
    [[nodiscard]] bool self_test();

    /**
     * @brief Check if data is ready (read INT_STATUS register).
     *
     * Useful for polling mode (when DRDY interrupt is not used).
     *
     * @return true if DATA_RDY_INT bit is set.
     */
    [[nodiscard]] bool data_ready();

    /**
     * @brief Program hardware offset registers (Bank 4, OFFSET_USER0–8).
     *
     * Offsets are applied in hardware before data reaches FIFO/registers,
     * providing zero-CPU-cost bias correction from the first sample.
     *
     * @param gyro_bias_dps   Gyro bias per axis in °/s (±64 dps max, 1/32 dps resolution).
     * @param accel_bias_g    Accel bias per axis in g (±1 g max, 0.5 mg resolution).
     * @return true on success.
     * @pre Sensors must be OFF (call before configure(), or disable sensors first).
     */
    [[nodiscard]] bool set_offsets(const float gyro_bias_dps[3], const float accel_bias_g[3]);

    /**
     * @brief Check if the driver has been successfully initialized.
     */
    [[nodiscard]] bool is_initialized() const
    {
        return initialized_;
    }

    /**
     * @brief Get cumulative communication error count.
     */
    [[nodiscard]] uint32_t error_count() const
    {
        return error_count_;
    }

  private:
    /* ── Register access helpers ─────────────────────────────────────── */

    [[nodiscard]] bool                   write_reg(uint8_t reg, uint8_t value);
    [[nodiscard]] std::optional<uint8_t> read_reg(uint8_t reg);
    [[nodiscard]] bool                   read_regs(uint8_t start_reg, uint8_t *buf, size_t len);

    /** Select register bank (0–4). Caches current bank to avoid
     *  redundant writes. Returns to bank 0 after bank operations. */
    [[nodiscard]] bool select_bank(uint8_t bank);

    /** Write a register in a non-zero bank, then return to bank 0. */
    [[nodiscard]] bool write_bank_reg(uint8_t bank, uint8_t reg, uint8_t value);

    /** Read a register from a non-zero bank, then return to bank 0. */
    [[nodiscard]] std::optional<uint8_t> read_bank_reg(uint8_t bank, uint8_t reg);

    /* ── Scale factor computation ────────────────────────────────────── */

    void compute_scale_factors();

    static float gyro_sensitivity(GyroFsr fsr);
    static float accel_sensitivity(AccelFsr fsr);

    /* ── Init / configure helpers ────────────────────────────────────── */

    [[nodiscard]] bool configure_interface();
    [[nodiscard]] bool configure_odr_fsr(const Iim42653Config &cfg);
    [[nodiscard]] bool configure_ui_filters(const Iim42653Config &cfg);
    [[nodiscard]] bool configure_aaf(const Iim42653Config &cfg);
    [[nodiscard]] bool configure_notch_filter(const Iim42653Config &cfg);
    [[nodiscard]] bool configure_interrupts(const Iim42653Config &cfg);
    [[nodiscard]] bool configure_fifo(const Iim42653Config &cfg);

    /* ── FIFO helpers ────────────────────────────────────────────────── */

    [[nodiscard]] bool    read_fifo_packet(uint8_t *buf);
    [[nodiscard]] int16_t fifo_byte_count();
    bool                  parse_fifo_packet(const uint8_t *pkt, ImuSample &sample) const;

    /* ── Self-test helpers ───────────────────────────────────────────── */

    bool               read_raw(int16_t accel[3], int16_t gyro[3]);
    [[nodiscard]] bool setup_self_test_mode();
    [[nodiscard]] bool collect_st_samples(int32_t accel_sum[3], int32_t gyro_sum[3], int count);
    [[nodiscard]] bool read_st_factory_refs(float gyro_ref[3], float accel_ref[3]);
    static bool        validate_st_results(const float gyro_response[3],
                                           const float accel_response[3],
                                           const float gyro_ref[3],
                                           const float accel_ref[3]);

    /* ── Error handling ──────────────────────────────────────────────── */

    void report_error();

    /* ── State ───────────────────────────────────────────────────────── */

    SpiBus          *spi_          = nullptr;
    ioline_t         cs_line_      = 0;
    const SPIConfig *spi_cfg_      = nullptr;
    Iim42653Config   config_       = {};
    uint8_t          current_bank_ = 0;
    bool             initialized_  = false;
    bool             fifo_enabled_ = false;
    uint32_t         error_count_  = 0;

    /* Scale factors: raw * scale = SI unit */
    float accel_scale_ = 0.0f; /* LSB -> m/s² */
    float gyro_scale_  = 0.0f; /* LSB -> rad/s */

    /* ── Constants ───────────────────────────────────────────────────── */

    static constexpr float kGravity    = 9.80665f;
    static constexpr float kDeg2Rad    = 0.017453292519943295f; /* π/180 */
    static constexpr float kTempScale  = 1.0f / 132.48f;
    static constexpr float kTempOffset = 25.0f;

    /** Sensor data burst: TEMP(2) + ACCEL(6) + GYRO(6) = 14 bytes */
    static constexpr size_t kBurstLen = 14;

    /** FIFO Packet 3: Header(1) + Accel(6) + Gyro(6) + Temp(1) + Timestamp(2) */
    static constexpr size_t kFifoPacketSize = 16;

    /** Max FIFO capacity: 2080 / 16 = 130 packets */
    static constexpr size_t kFifoMaxPackets = 130;

    /** FIFO header bit: FIFO is empty. */
    static constexpr uint8_t kFifoHeaderEmpty = 0x80;

    /** Expected header mask for Packet 3 with ODR timestamp: bits [6:2]. */
    static constexpr uint8_t kFifoHeaderMask    = 0x7C;
    static constexpr uint8_t kFifoHeaderExpected = 0x68; /* 0b0110_1000 */

    /** FIFO 8-bit temperature: T(°C) = raw / 2.07 + 25 */
    static constexpr float kFifoTempScale  = 1.0f / 2.07f;
    static constexpr float kFifoTempOffset = 25.0f;

    /** FIFO 8-bit temperature invalid marker. */
    static constexpr int8_t kFifoTempInvalid = -128;
};

}  // namespace acs
