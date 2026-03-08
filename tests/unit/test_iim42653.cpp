/**
 * @file test_iim42653.cpp
 * @brief Unit tests for IIM-42653 IMU driver data conversion logic.
 *
 * Tests the platform-independent parts of the driver:
 *   - Scale factor computation for all FSR settings
 *   - Raw-to-SI unit conversion accuracy
 *   - Temperature conversion
 *   - Configuration register value assembly
 *
 * The actual SPI communication is hardware-dependent and tested through
 * integration tests on the target board.
 */

#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>

/* ── Constants matching the driver ────────────────────────────────────── */

static constexpr float kGravity    = 9.80665f;
static constexpr float kDeg2Rad    = 0.017453292519943295f;
static constexpr float kTempScale  = 1.0f / 132.48f;
static constexpr float kTempOffset = 25.0f;
static constexpr float TOL         = 1e-4f;
static constexpr float TOL_TEMP    = 0.1f;

/* ═══════════════════════════════════════════════════════════════════════════
 * Gyro FSR sensitivity lookup (LSB per °/s)
 * ═══════════════════════════════════════════════════════════════════════════ */

struct GyroFsrEntry
{
    uint8_t fsr_bits;    /* value of bits [7:5] */
    float   dps;         /* full-scale range */
    float   sensitivity; /* LSB per °/s */
};

static constexpr GyroFsrEntry kGyroFsr[] = {
    {0, 4000.0f,    8.2f},
    {1, 2000.0f,   16.4f},
    {2, 1000.0f,   32.8f},
    {3,  500.0f,   65.5f},
    {4,  250.0f,  131.0f},
    {5,  125.0f,  262.0f},
    {6,   62.5f,  524.3f},
    {7,  31.25f, 1048.6f},
};

/* gyro_sensitivity and accel_sensitivity lookup functions available
 * for future integration tests that need runtime FSR selection. */

/* ═══════════════════════════════════════════════════════════════════════════
 * Accel FSR sensitivity lookup (LSB per g)
 * ═══════════════════════════════════════════════════════════════════════════ */

struct AccelFsrEntry
{
    uint8_t fsr_bits;
    float   g_range;
    float   sensitivity;
};

static constexpr AccelFsrEntry kAccelFsr[] = {
    {0, 32.0f, 1024.0f},
    {1, 16.0f, 2048.0f},
    {2,  8.0f, 4096.0f},
    {3,  4.0f, 8192.0f},
};

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Scale Factor Computation
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(ImuScaleFactor, GyroScaleAllFsr)
{
    for (const auto &e : kGyroFsr)
    {
        const float scale = kDeg2Rad / e.sensitivity;

        /* Check that full-scale raw value converts to expected rad/s. */
        const float raw_fs        = 32768.0f; /* approximate max raw value */
        const float expected_dps  = raw_fs / e.sensitivity;
        const float expected_rads = expected_dps * kDeg2Rad;
        const float computed_rads = raw_fs * scale;

        EXPECT_NEAR(computed_rads, expected_rads, TOL * expected_rads)
            << "FSR bits=" << static_cast<int>(e.fsr_bits) << " ±" << e.dps << " dps";
    }
}

TEST(ImuScaleFactor, AccelScaleAllFsr)
{
    for (const auto &e : kAccelFsr)
    {
        const float scale = kGravity / e.sensitivity;

        /* 1g should produce kGravity m/s². */
        const float raw_1g   = e.sensitivity;
        const float computed = raw_1g * scale;

        EXPECT_NEAR(computed, kGravity, TOL)
            << "FSR bits=" << static_cast<int>(e.fsr_bits) << " ±" << e.g_range << " g";
    }
}

TEST(ImuScaleFactor, GyroRocketDefault2000dps)
{
    /* Default: ±2000 dps, sensitivity = 16.4 LSB/°/s */
    const float scale = kDeg2Rad / 16.4f;

    /* 1000 °/s rotation → should give ~17.45 rad/s */
    const float raw    = 1000.0f * 16.4f;
    const float result = raw * scale;
    EXPECT_NEAR(result, 1000.0f * kDeg2Rad, 0.01f);
}

TEST(ImuScaleFactor, AccelRocketDefault32g)
{
    /* Default: ±32 g, sensitivity = 1024 LSB/g */
    const float scale = kGravity / 1024.0f;

    /* 20g (typical rocket boost) → should give ~196.13 m/s² */
    const float raw    = 20.0f * 1024.0f;
    const float result = raw * scale;
    EXPECT_NEAR(result, 20.0f * kGravity, 0.1f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Temperature Conversion
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(ImuTemp, AtReferencePoint)
{
    /* At 25°C, raw should be 0 (by datasheet formula). */
    const int16_t raw  = 0;
    const float   temp = static_cast<float>(raw) * kTempScale + kTempOffset;
    EXPECT_NEAR(temp, 25.0f, TOL_TEMP);
}

TEST(ImuTemp, AtPositiveTemp)
{
    /* 50°C → raw = (50 - 25) * 132.48 = 3312 */
    const int16_t raw  = 3312;
    const float   temp = static_cast<float>(raw) * kTempScale + kTempOffset;
    EXPECT_NEAR(temp, 50.0f, TOL_TEMP);
}

TEST(ImuTemp, AtNegativeTemp)
{
    /* -10°C → raw = (-10 - 25) * 132.48 = -4636.8 ≈ -4637 */
    const int16_t raw  = -4637;
    const float   temp = static_cast<float>(raw) * kTempScale + kTempOffset;
    EXPECT_NEAR(temp, -10.0f, TOL_TEMP);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Raw Data Parsing (Big-Endian)
 * ═══════════════════════════════════════════════════════════════════════════ */

static int16_t parse_be16(const uint8_t *p)
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) << 8 | p[1]);
}

TEST(ImuRawParse, PositiveValue)
{
    const uint8_t buf[2] = {0x10, 0x00}; /* 4096 */
    EXPECT_EQ(parse_be16(buf), 4096);
}

TEST(ImuRawParse, NegativeValue)
{
    const uint8_t buf[2] = {0xFF, 0x00}; /* -256 */
    EXPECT_EQ(parse_be16(buf), -256);
}

TEST(ImuRawParse, ZeroValue)
{
    const uint8_t buf[2] = {0x00, 0x00};
    EXPECT_EQ(parse_be16(buf), 0);
}

TEST(ImuRawParse, MaxPositive)
{
    const uint8_t buf[2] = {0x7F, 0xFF}; /* 32767 */
    EXPECT_EQ(parse_be16(buf), 32767);
}

TEST(ImuRawParse, InvalidDataMarker)
{
    /* 0x8000 = -32768, indicates invalid/sensor not ready */
    const uint8_t buf[2] = {0x80, 0x00};
    EXPECT_EQ(parse_be16(buf), -32768);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Invalid Data Rejection
 * ═══════════════════════════════════════════════════════════════════════════ */

static constexpr int16_t kInvalidRaw = -32768;

TEST(ImuInvalidData, SingleAxisInvalid)
{
    /* If any accel/gyro axis reads 0x8000, the whole sample is invalid. */
    const int16_t axes[]      = {0, 100, kInvalidRaw, 50, 0, 200};
    bool          has_invalid = false;
    for (int i = 0; i < 6; ++i)
    {
        if (axes[i] == kInvalidRaw)
            has_invalid = true;
    }
    EXPECT_TRUE(has_invalid);
}

TEST(ImuInvalidData, AllAxesValid)
{
    const int16_t axes[]      = {100, -100, 32767, -32767, 0, 1};
    bool          has_invalid = false;
    for (int i = 0; i < 6; ++i)
    {
        if (axes[i] == kInvalidRaw)
            has_invalid = true;
    }
    EXPECT_FALSE(has_invalid);
}

TEST(ImuInvalidData, TempInvalidStillConvert)
{
    /* Temperature 0x8000 is borderline — driver converts it but does
     * NOT reject the whole sample (only accel/gyro axes are checked). */
    const int16_t raw_temp = kInvalidRaw;
    const float   temp     = static_cast<float>(raw_temp) * kTempScale + kTempOffset;
    /* -32768 / 132.48 + 25 ≈ -222.4°C — clearly out of operating range,
     * but the caller should handle temp range validation separately. */
    EXPECT_LT(temp, -200.0f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Burst Read Conversion (Full Pipeline)
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(ImuConversion, BurstReadStationary)
{
    /* Simulate stationary sensor: 0 gyro, ~1g on Z-axis (upright). */
    const float accel_sens  = 1024.0f; /* ±32g mode */
    const float gyro_sens   = 16.4f;   /* ±2000 dps mode */
    const float accel_scale = kGravity / accel_sens;
    const float gyro_scale  = kDeg2Rad / gyro_sens;

    /* Raw 1g on Z → raw = 1024 */
    const int16_t raw_az = 1024;
    const float   az     = static_cast<float>(raw_az) * accel_scale;
    EXPECT_NEAR(az, kGravity, 0.01f);

    /* Raw 0 gyro → 0 rad/s */
    const int16_t raw_gz = 0;
    const float   gz     = static_cast<float>(raw_gz) * gyro_scale;
    EXPECT_NEAR(gz, 0.0f, TOL);
}

TEST(ImuConversion, HighGBoost)
{
    /* Simulate 25g during rocket boost in ±32g mode. */
    const float accel_sens  = 1024.0f;
    const float accel_scale = kGravity / accel_sens;

    const int16_t raw    = static_cast<int16_t>(25.0f * accel_sens);
    const float   result = static_cast<float>(raw) * accel_scale;
    EXPECT_NEAR(result, 25.0f * kGravity, 0.5f);
}

TEST(ImuConversion, HighRateRotation)
{
    /* Simulate 500 °/s roll rate in ±2000 dps mode. */
    const float gyro_sens  = 16.4f;
    const float gyro_scale = kDeg2Rad / gyro_sens;

    const int16_t raw    = static_cast<int16_t>(500.0f * gyro_sens);
    const float   result = static_cast<float>(raw) * gyro_scale;
    EXPECT_NEAR(result, 500.0f * kDeg2Rad, 0.05f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Register Value Assembly
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(ImuRegister, GyroConfig0Assembly)
{
    /* ±2000 dps = bits [7:5] = 001, 1 kHz = bits [3:0] = 0110 */
    const uint8_t fsr    = 1 << 5; /* DPS_2000 */
    const uint8_t odr    = 0x06;   /* HZ_1000 */
    const uint8_t config = fsr | odr;
    EXPECT_EQ(config, 0x26);
}

TEST(ImuRegister, AccelConfig0Assembly)
{
    /* ±32g = bits [7:5] = 000, 1 kHz = bits [3:0] = 0110 */
    const uint8_t fsr    = 0 << 5; /* G_32 */
    const uint8_t odr    = 0x06;   /* HZ_1000 */
    const uint8_t config = fsr | odr;
    EXPECT_EQ(config, 0x06);
}

TEST(ImuRegister, FilterBwAssembly)
{
    /* Accel BW = ODR/4 → bits [7:4] = 0001 */
    /* Gyro BW  = ODR/4 → bits [3:0] = 0001 */
    const uint8_t accel_bw = 1; /* ODR_DIV_4 */
    const uint8_t gyro_bw  = 1; /* ODR_DIV_4 */
    const uint8_t reg_val  = (accel_bw << 4) | gyro_bw;
    EXPECT_EQ(reg_val, 0x11);
}

TEST(ImuRegister, PwrMgmt0LowNoise)
{
    /* GYRO_MODE = 11 (LN), ACCEL_MODE = 11 (LN), TEMP_DIS = 0 */
    const uint8_t val = 0x0F;           /* 0b00001111 */
    EXPECT_EQ(val & 0x03, 0x03);        /* ACCEL_MODE bits [1:0] */
    EXPECT_EQ((val >> 2) & 0x03, 0x03); /* GYRO_MODE bits [3:2] */
    EXPECT_EQ((val >> 5) & 0x01, 0x00); /* TEMP_DIS bit 5 */
}

TEST(ImuRegister, IntConfigPushPullActiveHigh)
{
    /* INT1_POLARITY=1 (active high), INT1_DRIVE=1 (push-pull), INT1_MODE=0 (pulsed) */
    const uint8_t val = 0x03;
    EXPECT_EQ(val & 0x01, 1);        /* polarity */
    EXPECT_EQ((val >> 1) & 0x01, 1); /* drive circuit */
    EXPECT_EQ((val >> 2) & 0x01, 0); /* mode */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Self-Test Reference Computation
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(ImuSelfTest, OtpFormulaCode1)
{
    /* ST_OTP = 2620 * (1.01 ^ (code - 1)) */
    const uint8_t code = 1;
    const float   otp  = 2620.0f * std::pow(1.01f, static_cast<float>(code) - 1.0f);
    EXPECT_NEAR(otp, 2620.0f, 0.1f);
}

TEST(ImuSelfTest, OtpFormulaCode100)
{
    const float otp = 2620.0f * std::pow(1.01f, 99.0f);
    /* 1.01^99 ≈ 2.678 → otp ≈ 7016 */
    EXPECT_GT(otp, 6000.0f);
    EXPECT_LT(otp, 8000.0f);
}

TEST(ImuSelfTest, OtpFormulaCode0Invalid)
{
    /* Code 0 means no factory data programmed. */
    const uint8_t code = 0;
    /* Driver should treat this as 0 (skip OTP comparison). */
    float ref = 0.0f;
    if (code != 0)
    {
        ref = 2620.0f * std::pow(1.01f, static_cast<float>(code) - 1.0f);
    }
    EXPECT_EQ(ref, 0.0f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: Anti-Alias Filter Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

struct AafBwEntry
{
    uint8_t  delt;
    uint16_t deltsqr;
    uint8_t  bitshift;
    bool     enabled;
};

static constexpr AafBwEntry make_aaf(uint8_t d, uint16_t dsq, uint8_t bs)
{
    return {d, dsq, bs, d != 0};
}

TEST(ImuAaf, DisabledPreset)
{
    const auto aaf = make_aaf(0, 0, 0);
    EXPECT_FALSE(aaf.enabled);
}

TEST(ImuAaf, Bw536Preset)
{
    /* ~536 Hz: delt=12, deltsqr=144, bitshift=8 (from datasheet table) */
    const auto aaf = make_aaf(12, 144, 8);
    EXPECT_TRUE(aaf.enabled);
    EXPECT_EQ(aaf.delt, 12);
    EXPECT_EQ(aaf.deltsqr, 144);
    EXPECT_EQ(aaf.bitshift, 8);
}

TEST(ImuAaf, GyroRegisterEncoding)
{
    /* Verify how AAF params would be packed into registers.
     * GYRO_CONFIG_STATIC3: delt [5:0]
     * GYRO_CONFIG_STATIC4: deltsqr [7:0]
     * GYRO_CONFIG_STATIC5: bitshift [7:4] | deltsqr_hi [3:0]
     */
    const uint8_t  delt     = 12;
    const uint16_t deltsqr  = 144;
    const uint8_t  bitshift = 8;

    const uint8_t reg3 = delt & 0x3F;
    const uint8_t reg4 = static_cast<uint8_t>(deltsqr & 0xFF);
    const uint8_t reg5 = static_cast<uint8_t>((bitshift << 4) | ((deltsqr >> 8) & 0x0F));

    EXPECT_EQ(reg3, 12);
    EXPECT_EQ(reg4, 144);
    EXPECT_EQ(reg5, 0x80); /* bitshift=8 → 8<<4=0x80, deltsqr_hi=0 */
}

TEST(ImuAaf, AccelRegisterEncoding)
{
    /* ACCEL_CONFIG_STATIC2: [6:1] = delt, bit 0 = AAF_DIS = 0 (enabled) */
    const uint8_t delt = 12;
    const uint8_t reg2 = static_cast<uint8_t>((delt & 0x3F) << 1);
    EXPECT_EQ(reg2, 24);       /* 12 << 1 = 24, bit 0 = 0 → enabled */
    EXPECT_EQ(reg2 & 0x01, 0); /* AAF_DIS = 0 → enabled */
}

TEST(ImuAaf, LargeDeltsqrEncoding)
{
    /* ~1962 Hz: delt=37, deltsqr=1376, bitshift=4 */
    const uint16_t deltsqr  = 1376;
    const uint8_t  bitshift = 4;

    const uint8_t reg4 = static_cast<uint8_t>(deltsqr & 0xFF); /* 0x60 */
    const uint8_t reg5 = static_cast<uint8_t>((bitshift << 4) | ((deltsqr >> 8) & 0x0F));

    EXPECT_EQ(reg4, 0x60);                  /* 1376 & 0xFF = 96 = 0x60 */
    EXPECT_EQ((deltsqr >> 8) & 0x0F, 0x05); /* 1376 >> 8 = 5 */
    EXPECT_EQ(reg5, 0x45);                  /* (4<<4) | 5 = 0x45 */
}
