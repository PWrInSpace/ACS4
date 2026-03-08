/*
 * ACS4 Flight Computer — IIM-42653 IMU Driver (Implementation)
 *
 * TDK InvenSense IIM-42653 6-axis IMU.
 * See iim42653.h for API documentation.
 *
 * Init sequence follows the recommended power-up procedure from the
 * IIM-42653 datasheet, with specific timing constraints:
 *   - 1 ms after soft reset
 *   - 200 µs after sensor enable in PWR_MGMT0
 *   - 30 ms gyro startup time
 *   - INT_ASYNC_RESET must be cleared in INT_CONFIG1
 *
 * All bus access goes through the SpiBus abstraction (DMA, mutex-protected).
 */

#include "drivers/iim42653.h"

#include <cmath>
#include <cstring>

#include "system/error_handler.h"
#include "utils/timestamp.h"

extern "C" {
#include "ch.h"
}

namespace acs
{

using namespace iim42653_reg;

/* ═══════════════════════════════════════════════════════════════════════════
 * Byte Parsing
 * ═══════════════════════════════════════════════════════════════════════════ */

static inline int16_t parse_be16(const uint8_t *p)
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) << 8 | p[1]);
}

/** Raw value 0x8000 (−32768) indicates sensor data is not ready. */
static constexpr int16_t kInvalidRaw = -32768;

/* ═══════════════════════════════════════════════════════════════════════════
 * Register Access Helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Iim42653::write_reg(uint8_t reg, uint8_t value)
{
    return spi_->write_register(cs_line_, reg, value, *spi_cfg_);
}

std::optional<uint8_t> Iim42653::read_reg(uint8_t reg)
{
    return spi_->read_register(cs_line_, reg, *spi_cfg_);
}

bool Iim42653::read_regs(uint8_t start_reg, uint8_t *buf, size_t len)
{
    return spi_->read_registers(cs_line_, start_reg, buf, len, *spi_cfg_);
}

bool Iim42653::select_bank(uint8_t bank)
{
    if (bank == current_bank_)
    {
        return true;
    }

    if (!write_reg(REG_BANK_SEL, bank))
    {
        return false;
    }

    current_bank_ = bank;
    return true;
}

bool Iim42653::write_bank_reg(uint8_t bank, uint8_t reg, uint8_t value)
{
    if (!select_bank(bank))
    {
        return false;
    }

    const bool ok = write_reg(reg, value);

    /* Always return to bank 0. */
    (void)select_bank(0);

    return ok;
}

std::optional<uint8_t> Iim42653::read_bank_reg(uint8_t bank, uint8_t reg)
{
    if (!select_bank(bank))
    {
        return std::nullopt;
    }

    auto val = read_reg(reg);

    /* Always return to bank 0. */
    (void)select_bank(0);

    return val;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Error Handling
 * ═══════════════════════════════════════════════════════════════════════════ */

void Iim42653::report_error()
{
    ++error_count_;
    error_report(ErrorCode::IMU_COMM_FAIL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Scale Factor Computation
 * ═══════════════════════════════════════════════════════════════════════════ */

float Iim42653::gyro_sensitivity(GyroFsr fsr)
{
    switch (fsr)
    {
        case GyroFsr::DPS_4000:
            return 8.2f;
        case GyroFsr::DPS_2000:
            return 16.4f;
        case GyroFsr::DPS_1000:
            return 32.8f;
        case GyroFsr::DPS_500:
            return 65.5f;
        case GyroFsr::DPS_250:
            return 131.0f;
        case GyroFsr::DPS_125:
            return 262.0f;
        case GyroFsr::DPS_62_5:
            return 524.3f;
        case GyroFsr::DPS_31_25:
            return 1048.6f;
        default:
            return 16.4f;
    }
}

float Iim42653::accel_sensitivity(AccelFsr fsr)
{
    switch (fsr)
    {
        case AccelFsr::G_32:
            return 1024.0f;
        case AccelFsr::G_16:
            return 2048.0f;
        case AccelFsr::G_8:
            return 4096.0f;
        case AccelFsr::G_4:
            return 8192.0f;
        default:
            return 1024.0f;
    }
}

void Iim42653::compute_scale_factors()
{
    /*
     * Gyro:  raw / sensitivity_lsb_per_dps * (π/180) = rad/s
     * Accel: raw / sensitivity_lsb_per_g * 9.80665 = m/s²
     */
    gyro_scale_  = kDeg2Rad / gyro_sensitivity(config_.gyro_fsr);
    accel_scale_ = kGravity / accel_sensitivity(config_.accel_fsr);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Initialization
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Iim42653::init(SpiBus &spi, ioline_t cs_line, const SPIConfig &spi_cfg)
{
    spi_          = &spi;
    cs_line_      = cs_line;
    spi_cfg_      = &spi_cfg;
    current_bank_ = 0;

    /*
     * Step 1: Soft reset.
     * Write SOFT_RESET_CONFIG = 1 to DEVICE_CONFIG (0x11).
     * Wait 1 ms for reset to complete.
     */
    if (!write_reg(DEVICE_CONFIG, 0x01))
    {
        report_error();
        return false;
    }

    chThdSleepMilliseconds(2);

    /*
     * Step 2: Verify WHO_AM_I.
     * Expected: 0x56 for IIM-42653.
     */
    auto who = read_reg(WHO_AM_I);
    if (!who.has_value())
    {
        report_error();
        return false;
    }

    if (who.value() != WHO_AM_I_VALUE)
    {
        report_error();
        return false;
    }

    /*
     * Step 3-7: Interface configuration, clock, SPI, I3C.
     */
    if (!configure_interface())
    {
        return false;
    }

    initialized_ = true;
    return true;
}

bool Iim42653::configure_interface()
{
    /*
     * Clear INT_ASYNC_RESET (bit 4) in INT_CONFIG1.
     * Mandatory for proper interrupt pin operation.
     */
    auto int_cfg1 = read_reg(INT_CONFIG1);
    if (!int_cfg1.has_value())
    {
        report_error();
        return false;
    }

    if (!write_reg(INT_CONFIG1, int_cfg1.value() & ~(1U << 4)))
    {
        report_error();
        return false;
    }

    /* Interface configuration: big-endian, FIFO count in bytes, SPI 4-wire. */
    if (!write_reg(INTF_CONFIG0, 0x30))
    {
        report_error();
        return false;
    }

    /* Clock source — PLL auto-select (CLKSEL = 01). */
    auto intf1 = read_reg(INTF_CONFIG1);
    if (!intf1.has_value())
    {
        report_error();
        return false;
    }

    if (!write_reg(INTF_CONFIG1, (intf1.value() & 0xFC) | 0x01))
    {
        report_error();
        return false;
    }

    /* SPI slew rate configuration. */
    if (!write_reg(DRIVE_CONFIG, 0x05))
    {
        report_error();
        return false;
    }

    /* Disable I3C for clean SPI-only operation. */
    if (!write_bank_reg(1, INTF_CONFIG6, 0x00))
    {
        report_error();
        return false;
    }

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Iim42653::configure(const Iim42653Config &cfg)
{
    if (!initialized_)
    {
        return false;
    }

    config_ = cfg;
    compute_scale_factors();

    /* Ensure sensors are off before configuration. */
    if (!write_reg(PWR_MGMT0, 0x00))
    {
        report_error();
        return false;
    }

    chThdSleepMicroseconds(300);

    if (!configure_odr_fsr(cfg))
    {
        return false;
    }

    if (!configure_ui_filters(cfg))
    {
        return false;
    }

    if (!configure_aaf(cfg))
    {
        return false;
    }

    if (!configure_notch_filter(cfg))
    {
        return false;
    }

    if (!configure_fifo(cfg))
    {
        return false;
    }

    if (!configure_interrupts(cfg))
    {
        return false;
    }

    /* Enable sensors: gyro + accel in Low Noise mode, temperature on. */
    if (!write_reg(PWR_MGMT0, 0x0F))
    {
        report_error();
        return false;
    }

    /* Wait for gyro startup (30 ms) + margin. */
    chThdSleepMilliseconds(35);

    return true;
}

bool Iim42653::configure_odr_fsr(const Iim42653Config &cfg)
{
    /* GYRO_CONFIG0: [7:5] = FSR, [3:0] = ODR. */
    const uint8_t gyro_config =
        static_cast<uint8_t>(cfg.gyro_fsr) | static_cast<uint8_t>(cfg.gyro_odr);
    if (!write_reg(GYRO_CONFIG0, gyro_config))
    {
        report_error();
        return false;
    }

    /* ACCEL_CONFIG0: [7:5] = FSR, [3:0] = ODR. */
    const uint8_t accel_config =
        static_cast<uint8_t>(cfg.accel_fsr) | static_cast<uint8_t>(cfg.accel_odr);
    if (!write_reg(ACCEL_CONFIG0, accel_config))
    {
        report_error();
        return false;
    }

    return true;
}

bool Iim42653::configure_ui_filters(const Iim42653Config &cfg)
{
    /* GYRO_CONFIG1: [7:5] = TEMP_FILT_BW, [3:2] = GYRO_UI_FILT_ORD. */
    auto gyro_cfg1 = read_reg(GYRO_CONFIG1);
    if (!gyro_cfg1.has_value())
    {
        report_error();
        return false;
    }

    const uint8_t gyro_val =
        (gyro_cfg1.value() & 0x03)
        | (static_cast<uint8_t>(cfg.temp_filter_bw) << 5)
        | (static_cast<uint8_t>(cfg.gyro_filter_order) << 2);
    if (!write_reg(GYRO_CONFIG1, gyro_val))
    {
        report_error();
        return false;
    }

    /* ACCEL_CONFIG1: [4:3] = ACCEL_UI_FILT_ORD. */
    auto accel_cfg1 = read_reg(ACCEL_CONFIG1);
    if (!accel_cfg1.has_value())
    {
        report_error();
        return false;
    }

    const uint8_t accel_val =
        (accel_cfg1.value() & 0xE7) | (static_cast<uint8_t>(cfg.accel_filter_order) << 3);
    if (!write_reg(ACCEL_CONFIG1, accel_val))
    {
        report_error();
        return false;
    }

    /* GYRO_ACCEL_CONFIG0: [7:4] = accel BW, [3:0] = gyro BW. */
    const uint8_t filter_bw = (static_cast<uint8_t>(cfg.accel_filter_bw) << 4)
                              | static_cast<uint8_t>(cfg.gyro_filter_bw);
    if (!write_reg(GYRO_ACCEL_CONFIG0, filter_bw))
    {
        report_error();
        return false;
    }

    return true;
}

bool Iim42653::configure_aaf(const Iim42653Config &cfg)
{
    /* Gyro AAF — Bank 1. */
    if (cfg.gyro_aaf.is_enabled())
    {
        if (!write_bank_reg(1, GYRO_CONFIG_STATIC3, cfg.gyro_aaf.delt & 0x3F))
        {
            report_error();
            return false;
        }

        if (!write_bank_reg(1,
                            GYRO_CONFIG_STATIC4,
                            static_cast<uint8_t>(cfg.gyro_aaf.deltsqr & 0xFF)))
        {
            report_error();
            return false;
        }

        if (!write_bank_reg(1,
                            GYRO_CONFIG_STATIC5,
                            static_cast<uint8_t>((cfg.gyro_aaf.bitshift << 4)
                                                 | ((cfg.gyro_aaf.deltsqr >> 8) & 0x0F))))
        {
            report_error();
            return false;
        }

        /* Enable gyro AAF (clear GYRO_AAF_DIS bit 1). */
        auto gs2 = read_bank_reg(1, GYRO_CONFIG_STATIC2);
        if (!gs2.has_value())
        {
            report_error();
            return false;
        }

        if (!write_bank_reg(1, GYRO_CONFIG_STATIC2, gs2.value() & ~(1U << 1)))
        {
            report_error();
            return false;
        }
    }

    /* Accel AAF — Bank 2. */
    if (cfg.accel_aaf.is_enabled())
    {
        /* ACCEL_CONFIG_STATIC2: [6:1] = DELT, bit 0 = AAF_DIS = 0 (enabled). */
        if (!write_bank_reg(2,
                            ACCEL_CONFIG_STATIC2,
                            static_cast<uint8_t>((cfg.accel_aaf.delt & 0x3F) << 1)))
        {
            report_error();
            return false;
        }

        if (!write_bank_reg(2,
                            ACCEL_CONFIG_STATIC3,
                            static_cast<uint8_t>(cfg.accel_aaf.deltsqr & 0xFF)))
        {
            report_error();
            return false;
        }

        if (!write_bank_reg(2,
                            ACCEL_CONFIG_STATIC4,
                            static_cast<uint8_t>((cfg.accel_aaf.bitshift << 4)
                                                 | ((cfg.accel_aaf.deltsqr >> 8) & 0x0F))))
        {
            report_error();
            return false;
        }
    }

    return true;
}

bool Iim42653::configure_notch_filter(const Iim42653Config &cfg)
{
    if (!cfg.gyro_notch.is_enabled())
    {
        /* Ensure notch filter is disabled (set GYRO_NF_DIS = bit 0). */
        auto gs2 = read_bank_reg(1, GYRO_CONFIG_STATIC2);
        if (!gs2.has_value())
        {
            report_error();
            return false;
        }

        if (!write_bank_reg(1, GYRO_CONFIG_STATIC2, gs2.value() | 0x01))
        {
            report_error();
            return false;
        }

        return true;
    }

    /*
     * Compute NF_COSWZ and NF_COSWZ_SEL per datasheet §5.2.1.
     * fdesired is in Hz, formula uses kHz: cos(2π·f_kHz/8).
     */
    const float f_khz = cfg.gyro_notch.freq_hz / 1000.0f;
    const float coswz = std::cos(2.0f * 3.14159265358979f * f_khz / 8.0f);

    uint8_t nf_coswz_sel;
    int16_t nf_coswz_raw;

    if (std::fabs(coswz) <= 0.875f)
    {
        nf_coswz_sel = 0;
        nf_coswz_raw = static_cast<int16_t>(std::lround(coswz * 256.0f));
    }
    else
    {
        nf_coswz_sel = 1;
        if (coswz > 0.875f)
        {
            nf_coswz_raw = static_cast<int16_t>(std::lround(8.0f * (1.0f - coswz) * 256.0f));
        }
        else
        {
            nf_coswz_raw = static_cast<int16_t>(std::lround(-8.0f * (1.0f + coswz) * 256.0f));
        }
    }

    /* Encode as unsigned 9-bit value for register packing. */
    const auto coswz_u9 = static_cast<uint16_t>(nf_coswz_raw & 0x01FF);
    const uint8_t coswz_low = static_cast<uint8_t>(coswz_u9 & 0xFF);
    const uint8_t coswz_hi  = static_cast<uint8_t>((coswz_u9 >> 8) & 0x01);

    /* Program same frequency for all 3 axes. */
    if (!write_bank_reg(1, GYRO_CONFIG_STATIC6, coswz_low))
    {
        report_error();
        return false;
    }

    if (!write_bank_reg(1, GYRO_CONFIG_STATIC7, coswz_low))
    {
        report_error();
        return false;
    }

    if (!write_bank_reg(1, GYRO_CONFIG_STATIC8, coswz_low))
    {
        report_error();
        return false;
    }

    /*
     * GYRO_CONFIG_STATIC9 (0x12):
     *   bit 0 = X NF_COSWZ[8]
     *   bit 1 = Y NF_COSWZ[8]
     *   bit 2 = Z NF_COSWZ[8]
     *   bit 3 = X NF_COSWZ_SEL
     *   bit 4 = Y NF_COSWZ_SEL
     *   bit 5 = Z NF_COSWZ_SEL
     */
    const uint8_t static9 =
        (coswz_hi << 0) | (coswz_hi << 1) | (coswz_hi << 2)
        | (nf_coswz_sel << 3) | (nf_coswz_sel << 4) | (nf_coswz_sel << 5);

    if (!write_bank_reg(1, GYRO_CONFIG_STATIC9, static9))
    {
        report_error();
        return false;
    }

    /* GYRO_CONFIG_STATIC10 (0x13): [6:4] = NF_BW_SEL. */
    auto gs10 = read_bank_reg(1, GYRO_CONFIG_STATIC10);
    if (!gs10.has_value())
    {
        report_error();
        return false;
    }

    const uint8_t bw_val =
        (gs10.value() & 0x8F) | (static_cast<uint8_t>(cfg.gyro_notch.bw) << 4);
    if (!write_bank_reg(1, GYRO_CONFIG_STATIC10, bw_val))
    {
        report_error();
        return false;
    }

    /* Enable notch filter: clear GYRO_NF_DIS (bit 0) in GYRO_CONFIG_STATIC2. */
    auto gs2 = read_bank_reg(1, GYRO_CONFIG_STATIC2);
    if (!gs2.has_value())
    {
        report_error();
        return false;
    }

    if (!write_bank_reg(1, GYRO_CONFIG_STATIC2, gs2.value() & ~0x01U))
    {
        report_error();
        return false;
    }

    return true;
}

bool Iim42653::configure_interrupts(const Iim42653Config &cfg)
{
    if (!cfg.enable_drdy_int1)
    {
        return true;
    }

    /* INT_CONFIG: active-high, push-pull, pulsed. */
    if (!write_reg(INT_CONFIG, 0x03))
    {
        report_error();
        return false;
    }

    if (cfg.enable_fifo)
    {
        /*
         * FIFO mode: route FIFO_THS (watermark) to INT1.
         * INT_CONFIG0: FIFO_THS clear on status bit read (default).
         */
        if (!write_reg(INT_CONFIG0, 0x00))
        {
            report_error();
            return false;
        }

        /* INT_SOURCE0: FIFO_THS_INT1_EN (bit 2). */
        if (!write_reg(INT_SOURCE0, 0x04))
        {
            report_error();
            return false;
        }
    }
    else
    {
        /* Register-read mode: DRDY interrupt on INT1. */
        if (!write_reg(INT_CONFIG0, 0x20))
        {
            report_error();
            return false;
        }

        /* INT_SOURCE0: UI_DRDY_INT1_EN (bit 3). */
        if (!write_reg(INT_SOURCE0, 0x08))
        {
            report_error();
            return false;
        }
    }

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * FIFO Configuration
 * ═══════════════════════════════════════════════════════════════════════ */

bool Iim42653::configure_fifo(const Iim42653Config &cfg)
{
    if (!cfg.enable_fifo)
    {
        fifo_enabled_ = false;
        if (!write_reg(FIFO_CONFIG, 0x00)) /* bypass mode */
        {
            report_error();
            return false;
        }
        return true;
    }

    /* FIFO in bypass during configuration. */
    if (!write_reg(FIFO_CONFIG, 0x00))
    {
        report_error();
        return false;
    }

    /*
     * TMST_CONFIG = 0x21: TMST_EN=1, FSYNC_EN=0, 1 µs resolution.
     * Bits 7:5 preserved from reset value 0x23.
     */
    if (!write_reg(TMST_CONFIG, 0x21))
    {
        report_error();
        return false;
    }

    /*
     * FIFO_CONFIG1 = 0x27: FIFO_WM_GT_TH | TEMP_EN | GYRO_EN | ACCEL_EN.
     * Produces Packet 3 (16 B): Header + Accel + Gyro + Temp8 + Timestamp16.
     */
    if (!write_reg(FIFO_CONFIG1, 0x27))
    {
        report_error();
        return false;
    }

    /* Watermark in bytes. */
    const uint16_t wm_packets = (cfg.fifo_watermark > 0) ? cfg.fifo_watermark : 1;
    const uint16_t wm_bytes   = wm_packets * static_cast<uint16_t>(kFifoPacketSize);

    if (!write_reg(FIFO_CONFIG2, static_cast<uint8_t>(wm_bytes & 0xFF)))
    {
        report_error();
        return false;
    }

    if (!write_reg(FIFO_CONFIG3, static_cast<uint8_t>((wm_bytes >> 8) & 0x0F)))
    {
        report_error();
        return false;
    }

    /* Flush FIFO (SIGNAL_PATH_RESET bit 1). */
    auto sig = read_reg(SIGNAL_PATH_RESET);
    if (!sig.has_value())
    {
        report_error();
        return false;
    }

    if (!write_reg(SIGNAL_PATH_RESET, sig.value() | 0x02))
    {
        report_error();
        return false;
    }

    chThdSleepMicroseconds(100);

    /* Stream-to-FIFO mode: FIFO_CONFIG bits [7:6] = 01. */
    if (!write_reg(FIFO_CONFIG, 0x40))
    {
        report_error();
        return false;
    }

    fifo_enabled_ = true;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Data Read
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Iim42653::read(ImuSample &sample)
{
    if (!initialized_)
    {
        return false;
    }

    /*
     * Timestamp at start of read for accurate timing.
     */
    sample.timestamp_us = timestamp_us();

    /*
     * Burst read 14 bytes: TEMP_DATA1 (0x1D) through GYRO_DATA_Z0 (0x2A).
     *
     * Layout (big-endian):
     *   [0]  TEMP_DATA1  (high byte)
     *   [1]  TEMP_DATA0  (low byte)
     *   [2]  ACCEL_X1    (high)
     *   [3]  ACCEL_X0    (low)
     *   [4]  ACCEL_Y1
     *   [5]  ACCEL_Y0
     *   [6]  ACCEL_Z1
     *   [7]  ACCEL_Z0
     *   [8]  GYRO_X1
     *   [9]  GYRO_X0
     *   [10] GYRO_Y1
     *   [11] GYRO_Y0
     *   [12] GYRO_Z1
     *   [13] GYRO_Z0
     */
    uint8_t buf[kBurstLen];

    if (!read_regs(TEMP_DATA1, buf, kBurstLen))
    {
        report_error();
        return false;
    }

    /* Parse big-endian 16-bit signed values. */
    const int16_t raw_temp    = parse_be16(&buf[0]);
    const int16_t raw_accel_x = parse_be16(&buf[2]);
    const int16_t raw_accel_y = parse_be16(&buf[4]);
    const int16_t raw_accel_z = parse_be16(&buf[6]);
    const int16_t raw_gyro_x  = parse_be16(&buf[8]);
    const int16_t raw_gyro_y  = parse_be16(&buf[10]);
    const int16_t raw_gyro_z  = parse_be16(&buf[12]);

    /*
     * 0x8000 (−32768) means the sensor has not yet produced valid data.
     * Reject entire sample to prevent feeding garbage into the nav filter.
     */
    if (raw_temp == kInvalidRaw
        || raw_accel_x == kInvalidRaw || raw_accel_y == kInvalidRaw || raw_accel_z == kInvalidRaw
        || raw_gyro_x == kInvalidRaw || raw_gyro_y == kInvalidRaw || raw_gyro_z == kInvalidRaw)
    {
        return false;
    }

    /*
     * Convert to SI units.
     * Temperature: T(°C) = raw / 132.48 + 25
     * Accel:       a(m/s²) = raw / sensitivity * 9.80665
     * Gyro:        ω(rad/s) = raw / sensitivity * (π/180)
     */
    sample.temp_degc = static_cast<float>(raw_temp) * kTempScale + kTempOffset;

    sample.accel_mps2[0] = static_cast<float>(raw_accel_x) * accel_scale_;
    sample.accel_mps2[1] = static_cast<float>(raw_accel_y) * accel_scale_;
    sample.accel_mps2[2] = static_cast<float>(raw_accel_z) * accel_scale_;

    sample.gyro_rads[0] = static_cast<float>(raw_gyro_x) * gyro_scale_;
    sample.gyro_rads[1] = static_cast<float>(raw_gyro_y) * gyro_scale_;
    sample.gyro_rads[2] = static_cast<float>(raw_gyro_z) * gyro_scale_;

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Data Ready Check
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Iim42653::data_ready()
{
    auto status = read_reg(INT_STATUS);
    if (!status.has_value())
    {
        return false;
    }

    /* DATA_RDY_INT is bit 3 of INT_STATUS (0x2D). */
    return (status.value() & 0x08) != 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FIFO Read
 * ═══════════════════════════════════════════════════════════════════════════ */

int16_t Iim42653::fifo_byte_count()
{
    /*
     * Burst-read FIFO_COUNTH (0x2E) and FIFO_COUNTL (0x2F).
     * Reading FIFO_COUNTL latches both registers for a consistent value.
     * INTF_CONFIG0 bit 5 = 1 → big-endian byte order.
     */
    uint8_t buf[2];
    if (!read_regs(FIFO_COUNTH, buf, 2))
    {
        return -1;
    }

    return parse_be16(buf);
}

bool Iim42653::read_fifo_packet(uint8_t *buf)
{
    return read_regs(FIFO_DATA, buf, kFifoPacketSize);
}

bool Iim42653::parse_fifo_packet(const uint8_t *pkt, ImuSample &sample) const
{
    /*
     * Packet 3 layout (16 bytes, big-endian):
     *   [0]     Header
     *   [1:2]   Accel X
     *   [3:4]   Accel Y
     *   [5:6]   Accel Z
     *   [7:8]   Gyro X
     *   [9:10]  Gyro Y
     *   [11:12] Gyro Z
     *   [13]    Temperature (8-bit signed)
     *   [14:15] Timestamp (16-bit unsigned)
     */
    const uint8_t header = pkt[0];

    /* Check FIFO-empty flag. */
    if ((header & kFifoHeaderEmpty) != 0)
    {
        return false;
    }

    /* Validate header: expect accel + gyro + ODR timestamp (Packet 3). */
    if ((header & kFifoHeaderMask) != kFifoHeaderExpected)
    {
        return false;
    }

    const int16_t raw_accel_x = parse_be16(&pkt[1]);
    const int16_t raw_accel_y = parse_be16(&pkt[3]);
    const int16_t raw_accel_z = parse_be16(&pkt[5]);
    const int16_t raw_gyro_x  = parse_be16(&pkt[7]);
    const int16_t raw_gyro_y  = parse_be16(&pkt[9]);
    const int16_t raw_gyro_z  = parse_be16(&pkt[11]);

    /* Reject sample if any axis reports invalid data (0x8000). */
    if (raw_accel_x == kInvalidRaw || raw_accel_y == kInvalidRaw || raw_accel_z == kInvalidRaw
        || raw_gyro_x == kInvalidRaw || raw_gyro_y == kInvalidRaw || raw_gyro_z == kInvalidRaw)
    {
        return false;
    }

    /* Convert to SI units (same formula as register-read path). */
    sample.accel_mps2[0] = static_cast<float>(raw_accel_x) * accel_scale_;
    sample.accel_mps2[1] = static_cast<float>(raw_accel_y) * accel_scale_;
    sample.accel_mps2[2] = static_cast<float>(raw_accel_z) * accel_scale_;

    sample.gyro_rads[0] = static_cast<float>(raw_gyro_x) * gyro_scale_;
    sample.gyro_rads[1] = static_cast<float>(raw_gyro_y) * gyro_scale_;
    sample.gyro_rads[2] = static_cast<float>(raw_gyro_z) * gyro_scale_;

    /* 8-bit FIFO temperature: T(°C) = raw / 2.07 + 25, range -40 to 85°C. */
    const auto raw_temp = static_cast<int8_t>(pkt[13]);
    if (raw_temp == kFifoTempInvalid)
    {
        sample.temp_degc = 0.0f;
    }
    else
    {
        sample.temp_degc = static_cast<float>(raw_temp) * kFifoTempScale + kFifoTempOffset;
    }

    /* Sensor timestamp stored temporarily in timestamp_us (raw 16-bit µs).
     * The caller (read_fifo) reconstructs absolute host timestamps. */
    sample.timestamp_us = static_cast<uint32_t>(parse_be16(&pkt[14]) & 0xFFFF);

    return true;
}

size_t Iim42653::read_fifo(ImuSample *samples, size_t max_count)
{
    if (!initialized_ || !fifo_enabled_ || samples == nullptr || max_count == 0)
    {
        return 0;
    }

    /*
     * Capture host time — assigned to the newest sample.
     */
    const uint32_t host_time = timestamp_us();

    /* Check for FIFO overflow (bit 1 of INT_STATUS). */
    auto int_status = read_reg(INT_STATUS);
    if (int_status.has_value() && (int_status.value() & 0x02))
    {
        error_report(ErrorCode::IMU_FIFO_OVERFLOW);
        (void)flush_fifo();
        return 0;
    }

    /* Read FIFO byte count and compute number of complete packets. */
    const int16_t byte_count = fifo_byte_count();
    if (byte_count <= 0)
    {
        return 0;
    }

    size_t n_packets = static_cast<size_t>(byte_count) / kFifoPacketSize;
    if (n_packets == 0)
    {
        return 0;
    }

    if (n_packets > max_count)
    {
        n_packets = max_count;
    }

    if (n_packets > kFifoMaxPackets)
    {
        n_packets = kFifoMaxPackets;
    }

    /*
     * Bulk-read all FIFO packets in a single SPI transaction.
     * This avoids per-packet CS toggle + DMA setup overhead.
     */
    const size_t total_bytes = n_packets * kFifoPacketSize;
    uint8_t bulk[kFifoMaxPackets * kFifoPacketSize];

    if (!read_regs(FIFO_DATA, bulk, total_bytes))
    {
        report_error();
        return 0;
    }

    /* Parse packets. FIFO outputs oldest-first. */
    size_t valid = 0;
    for (size_t i = 0; i < n_packets; ++i)
    {
        if (parse_fifo_packet(&bulk[i * kFifoPacketSize], samples[valid]))
        {
            ++valid;
        }
    }

    if (valid == 0)
    {
        return 0;
    }

    /*
     * Reconstruct absolute host timestamps from sensor-side deltas.
     *
     * Each sample.timestamp_us currently holds the raw 16-bit sensor
     * timestamp (µs, wrapping at 65536). The newest sample (last valid)
     * gets the host DWT time. Earlier samples are offset backward by
     * the inter-sample sensor timestamp deltas.
     *
     * Unsigned 16-bit subtraction handles single wraparound correctly
     * as long as consecutive samples are < 65.5 ms apart (ODR ≥ ~16 Hz).
     *
     * Raw deltas must be scaled by 32/30 (datasheet §12.7): without
     * external CLKIN and TMST_RES=0, the sensor's internal 1 µs
     * counter runs at 30/32 of true speed.
     */
    samples[valid - 1].timestamp_us = host_time;

    for (size_t i = valid - 1; i > 0; --i)
    {
        const auto ts_this = static_cast<uint16_t>(samples[i].timestamp_us & 0xFFFF);
        const auto ts_prev = static_cast<uint16_t>(samples[i - 1].timestamp_us & 0xFFFF);
        const auto raw_delta = static_cast<uint16_t>(ts_this - ts_prev);
        const uint32_t delta = static_cast<uint32_t>(raw_delta) * 32U / 30U;

        samples[i - 1].timestamp_us = samples[i].timestamp_us - delta;
    }

    return valid;
}

bool Iim42653::flush_fifo()
{
    if (!initialized_)
    {
        return false;
    }

    auto sig = read_reg(SIGNAL_PATH_RESET);
    if (!sig.has_value())
    {
        report_error();
        return false;
    }

    if (!write_reg(SIGNAL_PATH_RESET, sig.value() | 0x02))
    {
        report_error();
        return false;
    }

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Hardware Offset Calibration
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Iim42653::set_offsets(const float gyro_bias_dps[3], const float accel_bias_g[3])
{
    if (!initialized_)
    {
        return false;
    }

    /*
     * Gyro offset: 12-bit signed, resolution = 1/32 dps → LSB = raw * 32.
     * Accel offset: 12-bit signed, resolution = 0.5 mg → LSB = raw * 2000.
     *
     * Register layout (Bank 4):
     *   USER0 = gyro_x[7:0]
     *   USER1 = gyro_y_hi[7:4] | gyro_x_hi[3:0]
     *   USER2 = gyro_y[7:0]
     *   USER3 = gyro_z[7:0]
     *   USER4 = accel_x_hi[7:4] | gyro_z_hi[3:0]
     *   USER5 = accel_x[7:0]
     *   USER6 = accel_y[7:0]
     *   USER7 = accel_z_hi[7:4] | accel_y_hi[3:0]
     *   USER8 = accel_z[7:0]
     */
    auto to_gyro_lsb = [](float dps) -> int16_t {
        float clamped = (dps < -64.0f) ? -64.0f : (dps > 64.0f) ? 64.0f : dps;
        return static_cast<int16_t>(std::lround(clamped * 32.0f));
    };

    auto to_accel_lsb = [](float g) -> int16_t {
        float clamped = (g < -1.0f) ? -1.0f : (g > 1.0f) ? 1.0f : g;
        return static_cast<int16_t>(std::lround(clamped * 2000.0f));
    };

    const int16_t gx = to_gyro_lsb(gyro_bias_dps[0]);
    const int16_t gy = to_gyro_lsb(gyro_bias_dps[1]);
    const int16_t gz = to_gyro_lsb(gyro_bias_dps[2]);
    const int16_t ax = to_accel_lsb(accel_bias_g[0]);
    const int16_t ay = to_accel_lsb(accel_bias_g[1]);
    const int16_t az = to_accel_lsb(accel_bias_g[2]);

    /* Pack into 9 register bytes. Each 12-bit value splits across two regs. */
    const uint8_t regs[9] = {
        static_cast<uint8_t>(gx & 0xFF),                                              /* USER0 */
        static_cast<uint8_t>(((gy & 0x0F00) >> 4) | ((gx >> 8) & 0x0F)),              /* USER1 */
        static_cast<uint8_t>(gy & 0xFF),                                              /* USER2 */
        static_cast<uint8_t>(gz & 0xFF),                                              /* USER3 */
        static_cast<uint8_t>(((ax & 0x0F00) >> 4) | ((gz >> 8) & 0x0F)),              /* USER4 */
        static_cast<uint8_t>(ax & 0xFF),                                              /* USER5 */
        static_cast<uint8_t>(ay & 0xFF),                                              /* USER6 */
        static_cast<uint8_t>(((az & 0x0F00) >> 4) | ((ay >> 8) & 0x0F)),              /* USER7 */
        static_cast<uint8_t>(az & 0xFF),                                              /* USER8 */
    };

    for (uint8_t i = 0; i < 9; ++i)
    {
        if (!write_bank_reg(4, OFFSET_USER0 + i, regs[i]))
        {
            report_error();
            return false;
        }
    }

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Raw Data Read (for self-test)
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Iim42653::read_raw(int16_t accel[3], int16_t gyro[3])
{
    uint8_t buf[kBurstLen];

    if (!read_regs(TEMP_DATA1, buf, kBurstLen))
    {
        return false;
    }

    accel[0] = parse_be16(&buf[2]);
    accel[1] = parse_be16(&buf[4]);
    accel[2] = parse_be16(&buf[6]);

    gyro[0] = parse_be16(&buf[8]);
    gyro[1] = parse_be16(&buf[10]);
    gyro[2] = parse_be16(&buf[12]);

    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Self-Test
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Iim42653::self_test()
{
    if (!initialized_)
    {
        return false;
    }

    static constexpr int kNumSamples = 200;

    /* Save current config to restore later. */
    const Iim42653Config saved_config = config_;

    /* Configure for self-test ranges and stabilize. */
    if (!setup_self_test_mode())
    {
        return false;
    }

    /* Collect samples with self-test OFF. */
    int32_t accel_off[3] = {0, 0, 0};
    int32_t gyro_off[3]  = {0, 0, 0};

    if (!collect_st_samples(accel_off, gyro_off, kNumSamples))
    {
        return false;
    }

    /* Enable self-test. */
    if (!write_reg(SELF_TEST_CONFIG, 0x7F))
    {
        return false;
    }

    chThdSleepMilliseconds(200);

    /* Collect samples with self-test ON. */
    int32_t accel_on[3] = {0, 0, 0};
    int32_t gyro_on[3]  = {0, 0, 0};

    if (!collect_st_samples(accel_on, gyro_on, kNumSamples))
    {
        (void)write_reg(SELF_TEST_CONFIG, 0x00);
        return false;
    }

    /* Disable self-test. */
    if (!write_reg(SELF_TEST_CONFIG, 0x00))
    {
        return false;
    }

    chThdSleepMilliseconds(100);

    /* Compute self-test response. */
    float accel_response[3];
    float gyro_response[3];
    for (int ax = 0; ax < 3; ++ax)
    {
        accel_response[ax] =
            static_cast<float>(accel_on[ax] - accel_off[ax]) / static_cast<float>(kNumSamples);
        gyro_response[ax] =
            static_cast<float>(gyro_on[ax] - gyro_off[ax]) / static_cast<float>(kNumSamples);
    }

    /* Read factory references and validate. */
    float gyro_st_ref[3];
    float accel_st_ref[3];
    if (!read_st_factory_refs(gyro_st_ref, accel_st_ref))
    {
        return false;
    }

    const bool pass = validate_st_results(gyro_response, accel_response, gyro_st_ref, accel_st_ref);

    if (!pass)
    {
        error_report(ErrorCode::IMU_SELF_TEST_FAIL);
    }

    (void)configure(saved_config);
    return pass;
}

bool Iim42653::setup_self_test_mode()
{
    static constexpr int kSettleMs = 100;

    if (!write_reg(PWR_MGMT0, 0x00))
    {
        return false;
    }

    chThdSleepMicroseconds(300);

    /* Gyro: ±250 dps, 1 kHz. Accel: ±4 g, 1 kHz (per datasheet). */
    if (!write_reg(GYRO_CONFIG0,
                   static_cast<uint8_t>(GyroFsr::DPS_250) | static_cast<uint8_t>(GyroOdr::HZ_1000)))
    {
        return false;
    }

    if (!write_reg(ACCEL_CONFIG0,
                   static_cast<uint8_t>(AccelFsr::G_4) | static_cast<uint8_t>(AccelOdr::HZ_1000)))
    {
        return false;
    }

    if (!write_reg(PWR_MGMT0, 0x0F))
    {
        return false;
    }

    chThdSleepMilliseconds(kSettleMs);
    return true;
}

bool Iim42653::collect_st_samples(int32_t accel_sum[3], int32_t gyro_sum[3], int count)
{
    for (int i = 0; i < count; ++i)
    {
        int16_t a[3];
        int16_t g[3];
        if (!read_raw(a, g))
        {
            return false;
        }

        for (int ax = 0; ax < 3; ++ax)
        {
            accel_sum[ax] += a[ax];
            gyro_sum[ax] += g[ax];
        }

        chThdSleepMilliseconds(1);
    }

    return true;
}

bool Iim42653::read_st_factory_refs(float gyro_ref[3], float accel_ref[3])
{
    static constexpr uint8_t gyro_st_regs[3]  = {XG_ST_DATA, YG_ST_DATA, ZG_ST_DATA};
    static constexpr uint8_t accel_st_regs[3] = {XA_ST_DATA, YA_ST_DATA, ZA_ST_DATA};

    for (int ax = 0; ax < 3; ++ax)
    {
        auto gval = read_bank_reg(1, gyro_st_regs[ax]);
        if (!gval.has_value())
        {
            return false;
        }

        auto aval = read_bank_reg(2, accel_st_regs[ax]);
        if (!aval.has_value())
        {
            return false;
        }

        /* ST_OTP = 2620 * (1.01 ^ (code - 1)), or 0 if unprogrammed. */
        gyro_ref[ax] = (gval.value() != 0)
                            ? 2620.0f * std::pow(1.01f, static_cast<float>(gval.value()) - 1.0f)
                            : 0.0f;

        accel_ref[ax] = (aval.value() != 0)
                             ? 2620.0f * std::pow(1.01f, static_cast<float>(aval.value()) - 1.0f)
                             : 0.0f;
    }

    return true;
}

bool Iim42653::validate_st_results(const float gyro_response[3],
                                   const float accel_response[3],
                                   const float gyro_ref[3],
                                   const float accel_ref[3])
{
    static constexpr float kGyroSensitivity  = 131.0f;
    static constexpr float kAccelSensitivity = 8192.0f;
    static constexpr float kOtpTolerance     = 0.5f;
    static constexpr float kGyroMinDps       = 60.0f;
    static constexpr float kAccelMinMg       = 50.0f;

    bool pass = true;

    for (int ax = 0; ax < 3; ++ax)
    {
        /* Gyro check. */
        if (gyro_ref[ax] > 0.0f)
        {
            const float ratio = gyro_response[ax] / gyro_ref[ax];
            if (std::fabs(ratio - 1.0f) > kOtpTolerance)
            {
                pass = false;
            }
        }
        else
        {
            const float dps = std::fabs(gyro_response[ax]) / kGyroSensitivity;
            if (dps < kGyroMinDps)
            {
                pass = false;
            }
        }

        /* Accel check. */
        if (accel_ref[ax] > 0.0f)
        {
            const float ratio = accel_response[ax] / accel_ref[ax];
            if (std::fabs(ratio - 1.0f) > kOtpTolerance)
            {
                pass = false;
            }
        }
        else
        {
            const float mg = std::fabs(accel_response[ax]) / kAccelSensitivity * 1000.0f;
            if (mg < kAccelMinMg)
            {
                pass = false;
            }
        }
    }

    return pass;
}

}  // namespace acs
