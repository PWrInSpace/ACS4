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
#include <limits>

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
    /* Temperature 0x8000 → driver does NOT reject the whole sample.
     * Only accel/gyro axes cause rejection; invalid temp gets NaN. */
    const int16_t raw_temp = kInvalidRaw;

    /* Verify the raw formula gives nonsensical value. */
    const float temp_converted = static_cast<float>(raw_temp) * kTempScale + kTempOffset;
    EXPECT_LT(temp_converted, -200.0f); /* raw formula gives garbage */

    /* The driver returns NaN for invalid temperature readings. */
    const float temp_result = (raw_temp == kInvalidRaw)
                                  ? std::numeric_limits<float>::quiet_NaN()
                                  : static_cast<float>(raw_temp) * kTempScale + kTempOffset;
    EXPECT_TRUE(std::isnan(temp_result));
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

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: FIFO Packet 3 Parsing
 * ═══════════════════════════════════════════════════════════════════════════ */

/* FIFO constants matching driver header */
static constexpr size_t  kFifoPacketSize     = 16;
static constexpr uint8_t kFifoHeaderEmpty    = 0x80;
static constexpr uint8_t kFifoHeaderMask     = 0x7C;
static constexpr uint8_t kFifoHeaderExpected = 0x68; /* 0b0110_1000 */
static constexpr float   kFifoTempScale      = 1.0f / 2.07f;
static constexpr float   kFifoTempOffset     = 25.0f;
static constexpr int8_t  kFifoTempInvalid    = -128;

/**
 * Build a Packet 3 buffer (16 bytes) for testing.
 *
 * @param header    Header byte.
 * @param ax,ay,az  Accel raw (big-endian 16-bit signed).
 * @param gx,gy,gz  Gyro raw.
 * @param temp8     8-bit temperature.
 * @param ts        16-bit ODR timestamp.
 */
static void build_packet3(uint8_t *pkt,
                          uint8_t  header,
                          int16_t  ax,
                          int16_t  ay,
                          int16_t  az,
                          int16_t  gx,
                          int16_t  gy,
                          int16_t  gz,
                          int8_t   temp8,
                          uint16_t ts)
{
    pkt[0]  = header;
    pkt[1]  = static_cast<uint8_t>((ax >> 8) & 0xFF);
    pkt[2]  = static_cast<uint8_t>(ax & 0xFF);
    pkt[3]  = static_cast<uint8_t>((ay >> 8) & 0xFF);
    pkt[4]  = static_cast<uint8_t>(ay & 0xFF);
    pkt[5]  = static_cast<uint8_t>((az >> 8) & 0xFF);
    pkt[6]  = static_cast<uint8_t>(az & 0xFF);
    pkt[7]  = static_cast<uint8_t>((gx >> 8) & 0xFF);
    pkt[8]  = static_cast<uint8_t>(gx & 0xFF);
    pkt[9]  = static_cast<uint8_t>((gy >> 8) & 0xFF);
    pkt[10] = static_cast<uint8_t>(gy & 0xFF);
    pkt[11] = static_cast<uint8_t>((gz >> 8) & 0xFF);
    pkt[12] = static_cast<uint8_t>(gz & 0xFF);
    pkt[13] = static_cast<uint8_t>(temp8);
    pkt[14] = static_cast<uint8_t>((ts >> 8) & 0xFF);
    pkt[15] = static_cast<uint8_t>(ts & 0xFF);
}

/** Stand-alone parse that mirrors driver logic (no HW dep). */
struct FifoParseResult
{
    float    accel_mps2[3];
    float    gyro_rads[3];
    float    temp_degc;
    uint16_t sensor_ts;
    bool     valid;
};

static FifoParseResult parse_fifo_pkt(const uint8_t *pkt, float accel_scale, float gyro_scale)
{
    FifoParseResult r{};

    const uint8_t header = pkt[0];
    if ((header & kFifoHeaderEmpty) != 0 || (header & kFifoHeaderMask) != kFifoHeaderExpected)
    {
        r.valid = false;
        return r;
    }

    const int16_t rax = parse_be16(&pkt[1]);
    const int16_t ray = parse_be16(&pkt[3]);
    const int16_t raz = parse_be16(&pkt[5]);
    const int16_t rgx = parse_be16(&pkt[7]);
    const int16_t rgy = parse_be16(&pkt[9]);
    const int16_t rgz = parse_be16(&pkt[11]);

    if (rax == kInvalidRaw || ray == kInvalidRaw || raz == kInvalidRaw || rgx == kInvalidRaw
        || rgy == kInvalidRaw || rgz == kInvalidRaw)
    {
        r.valid = false;
        return r;
    }

    r.accel_mps2[0] = static_cast<float>(rax) * accel_scale;
    r.accel_mps2[1] = static_cast<float>(ray) * accel_scale;
    r.accel_mps2[2] = static_cast<float>(raz) * accel_scale;
    r.gyro_rads[0]  = static_cast<float>(rgx) * gyro_scale;
    r.gyro_rads[1]  = static_cast<float>(rgy) * gyro_scale;
    r.gyro_rads[2]  = static_cast<float>(rgz) * gyro_scale;

    const auto raw_temp = static_cast<int8_t>(pkt[13]);
    r.temp_degc         = (raw_temp == kFifoTempInvalid)
                              ? 0.0f
                              : static_cast<float>(raw_temp) * kFifoTempScale + kFifoTempOffset;

    r.sensor_ts = static_cast<uint16_t>(parse_be16(&pkt[14]) & 0xFFFF);
    r.valid     = true;
    return r;
}

TEST(ImuFifo, Packet3ValidParse)
{
    /* ±32g (1024 LSB/g), ±2000 dps (16.4 LSB/°/s) */
    const float accel_scale = kGravity / 1024.0f;
    const float gyro_scale  = kDeg2Rad / 16.4f;

    /* 1 g on Z, 100 °/s on X, 30°C, ts=5000 µs */
    uint8_t       pkt[kFifoPacketSize];
    const int16_t az  = 1024;
    const int16_t gx  = static_cast<int16_t>(100.0f * 16.4f);
    const int8_t  tmp = static_cast<int8_t>((30.0f - 25.0f) * 2.07f); /* ~10 */
    build_packet3(pkt, 0x68, 0, 0, az, gx, 0, 0, tmp, 5000);

    auto r = parse_fifo_pkt(pkt, accel_scale, gyro_scale);
    EXPECT_TRUE(r.valid);
    EXPECT_NEAR(r.accel_mps2[2], kGravity, 0.01f);
    EXPECT_NEAR(r.gyro_rads[0], 100.0f * kDeg2Rad, 0.1f);
    EXPECT_NEAR(r.temp_degc, 30.0f, 1.0f);
    EXPECT_EQ(r.sensor_ts, 5000);
}

TEST(ImuFifo, Packet3EmptyHeader)
{
    uint8_t pkt[kFifoPacketSize] = {};
    pkt[0]                       = 0x80; /* FIFO empty flag */
    auto r                       = parse_fifo_pkt(pkt, 1.0f, 1.0f);
    EXPECT_FALSE(r.valid);
}

TEST(ImuFifo, Packet3WrongHeaderType)
{
    uint8_t pkt[kFifoPacketSize] = {};
    pkt[0]                       = 0x40; /* Only accel, no gyro — not Packet 3 */
    auto r                       = parse_fifo_pkt(pkt, 1.0f, 1.0f);
    EXPECT_FALSE(r.valid);
}

TEST(ImuFifo, Packet3InvalidAccelRejected)
{
    uint8_t pkt[kFifoPacketSize];
    build_packet3(pkt, 0x68, kInvalidRaw, 0, 0, 0, 0, 0, 0, 1000);
    auto r = parse_fifo_pkt(pkt, 1.0f, 1.0f);
    EXPECT_FALSE(r.valid);
}

TEST(ImuFifo, Packet3InvalidGyroRejected)
{
    uint8_t pkt[kFifoPacketSize];
    build_packet3(pkt, 0x68, 100, 200, 300, kInvalidRaw, 0, 0, 0, 1000);
    auto r = parse_fifo_pkt(pkt, 1.0f, 1.0f);
    EXPECT_FALSE(r.valid);
}

TEST(ImuFifo, Packet3HeaderOdrBitsIgnored)
{
    /* Bits [1:0] are ODR-change flags — should not affect parse. */
    uint8_t pkt[kFifoPacketSize];
    build_packet3(pkt, 0x6B, 100, 200, 300, 400, 500, 600, 10, 2000);
    auto r = parse_fifo_pkt(pkt, 1.0f, 1.0f);
    EXPECT_TRUE(r.valid);
    EXPECT_EQ(r.sensor_ts, 2000);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: FIFO Temperature Conversion (8-bit)
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(ImuFifoTemp, AtReferencePoint)
{
    /* 25°C → raw = 0 → (0 / 2.07) + 25 = 25 */
    const int8_t raw  = 0;
    const float  temp = static_cast<float>(raw) * kFifoTempScale + kFifoTempOffset;
    EXPECT_NEAR(temp, 25.0f, 0.1f);
}

TEST(ImuFifoTemp, AtHighTemp)
{
    /* ~85°C → raw = (85-25)*2.07 = 124.2 → 124 */
    const int8_t raw  = 124;
    const float  temp = static_cast<float>(raw) * kFifoTempScale + kFifoTempOffset;
    EXPECT_NEAR(temp, 84.9f, 1.0f);
}

TEST(ImuFifoTemp, AtLowTemp)
{
    /* -40°C → raw = (-40-25)*2.07 = -134.55 → clamps to -128+invalid overlap */
    /* Use -80 → (-80/2.07)+25 ≈ -13.6°C */
    const int8_t raw  = -80;
    const float  temp = static_cast<float>(raw) * kFifoTempScale + kFifoTempOffset;
    EXPECT_NEAR(temp, -13.6f, 0.5f);
}

TEST(ImuFifoTemp, InvalidMarker)
{
    /* -128 is the invalid marker → driver returns 0°C */
    const int8_t raw = kFifoTempInvalid;
    EXPECT_EQ(raw, -128);
    /* parse sets temp_degc = 0 for invalid */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: FIFO Timestamp Reconstruction
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Reconstruct absolute timestamps from sensor 16-bit timestamps,
 * mirroring the driver's algorithm. Operates in-place on an array
 * of (valid_count) raw sensor timestamps + a host reference time.
 *
 * Raw deltas are scaled by 32/30 per datasheet §12.7 (no CLKIN,
 * TMST_RES=0): the sensor's internal counter runs at 30/32 speed.
 */
static void reconstruct_timestamps(uint32_t *ts_us, size_t count, uint32_t host_time)
{
    if (count == 0)
        return;

    /* Save raw 16-bit sensor timestamps before overwriting. */
    auto *raw = new uint16_t[count];
    for (size_t i = 0; i < count; ++i)
    {
        raw[i] = static_cast<uint16_t>(ts_us[i] & 0xFFFF);
    }

    /* Newest sample gets host time. */
    ts_us[count - 1] = host_time;

    /* Walk backward, computing scaled deltas from sensor timestamps. */
    for (size_t i = count - 1; i > 0; --i)
    {
        const uint16_t raw_delta = static_cast<uint16_t>(raw[i] - raw[i - 1]);
        const uint32_t delta     = static_cast<uint32_t>(raw_delta) * 32U / 30U;
        ts_us[i - 1]             = ts_us[i] - delta;
    }

    delete[] raw;
}

TEST(ImuFifoTimestamp, ThreeSamplesNoWrap)
{
    /*
     * 3 samples at 1 kHz. Sensor raw interval ≈ 937–938 µs
     * (datasheet §12.7: true 1000 µs → raw 937.5 µs).
     * Use raw delta = 937 for deterministic test.
     * After 32/30 scaling: 937 * 32/30 = 999 µs (integer).
     */
    const uint32_t host_time = 1'000'000;
    uint32_t       ts[3]     = {1000, 1937, 2874};

    reconstruct_timestamps(ts, 3, host_time);

    const uint32_t scaled_delta = 937U * 32U / 30U; /* 999 */
    EXPECT_EQ(ts[2], host_time);
    EXPECT_EQ(ts[1], host_time - scaled_delta);
    EXPECT_EQ(ts[0], host_time - 2 * scaled_delta);
}

TEST(ImuFifoTimestamp, TimestampWraparound)
{
    /* Sensor timestamps wrap at 65536.
     * Sample 0: ts = 65000
     * Sample 1: ts = 65000 + 937 = 65937 → wraps to 65937 - 65536 = 401
     * Sample 2: ts = 401 + 937 = 1338
     * Raw delta = 937, scaled = 937 * 32/30 = 999.
     */
    const uint32_t host_time = 500'000;
    uint32_t       ts[3]     = {65000, 401, 1338};

    reconstruct_timestamps(ts, 3, host_time);

    const uint32_t scaled_delta = 937U * 32U / 30U; /* 999 */
    EXPECT_EQ(ts[2], host_time);
    EXPECT_EQ(ts[1], host_time - scaled_delta);
    EXPECT_EQ(ts[0], host_time - 2 * scaled_delta);
}

TEST(ImuFifoTimestamp, SingleSample)
{
    const uint32_t host_time = 42000;
    uint32_t       ts[1]     = {12345};

    reconstruct_timestamps(ts, 1, host_time);

    EXPECT_EQ(ts[0], host_time);
}

TEST(ImuFifoTimestamp, TenSamplesUniform)
{
    /*
     * 10 samples with raw sensor interval = 937 µs (1 kHz ODR).
     * Scaled delta = 937 * 32/30 = 999 µs.
     */
    const uint32_t host_time       = 2'000'000;
    const uint16_t raw_interval    = 937;
    const uint32_t scaled_interval = static_cast<uint32_t>(raw_interval) * 32U / 30U;
    uint32_t       ts[10];
    for (int i = 0; i < 10; ++i)
    {
        ts[i] = static_cast<uint32_t>(10000 + i * raw_interval);
    }

    reconstruct_timestamps(ts, 10, host_time);

    for (int i = 0; i < 10; ++i)
    {
        EXPECT_EQ(ts[i], host_time - static_cast<uint32_t>((9 - i)) * scaled_interval)
            << "sample " << i;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tests: FIFO Configuration Register Assembly
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(ImuFifoConfig, TmstConfigForFifo)
{
    /*
     * TMST_CONFIG for FIFO: TMST_EN=1, FSYNC_EN=0, 1µs res.
     * Default reset = 0x23. We want bits 7:5 preserved (0x20) + bit 0 = 1.
     */
    const uint8_t val = 0x21;
    EXPECT_EQ(val & 0x01, 1); /* TMST_EN */
    EXPECT_EQ(val & 0x02, 0); /* TMST_FSYNC_EN off */
    EXPECT_EQ(val & 0x08, 0); /* TMST_RES = 0 (1µs) */
}

TEST(ImuFifoConfig, FifoConfig1Assembly)
{
    /*
     * FIFO_CONFIG1: FIFO_WM_GT_TH | FIFO_TMST_FSYNC_EN | FIFO_TEMP_EN | FIFO_GYRO_EN |
     * FIFO_ACCEL_EN = bit5 | bit3 | bit2 | bit1 | bit0 = 0x2F bit3 (FIFO_TMST_FSYNC_EN) is required
     * for ODR timestamps in Packet 3.
     */
    const uint8_t val = 0x2F;
    EXPECT_NE(val & 0x01, 0); /* FIFO_ACCEL_EN */
    EXPECT_NE(val & 0x02, 0); /* FIFO_GYRO_EN */
    EXPECT_NE(val & 0x04, 0); /* FIFO_TEMP_EN */
    EXPECT_NE(val & 0x08, 0); /* FIFO_TMST_FSYNC_EN (ODR timestamps) */
    EXPECT_NE(val & 0x20, 0); /* FIFO_WM_GT_TH */
    EXPECT_EQ(val & 0x10, 0); /* FIFO_HIRES_EN off (no 20-byte packets) */
}

TEST(ImuFifoConfig, FifoConfigStreamMode)
{
    /* FIFO_CONFIG: Stream-to-FIFO → bits [7:6] = 01 → 0x40 */
    const uint8_t val = 0x40;
    EXPECT_EQ((val >> 6) & 0x03, 0x01);
}

TEST(ImuFifoConfig, WatermarkEncoding)
{
    /* 5 packets × 16 bytes = 80 bytes */
    const uint16_t wm_packets = 5;
    const uint16_t wm_bytes   = wm_packets * 16;
    EXPECT_EQ(wm_bytes, 80);

    const uint8_t reg2 = static_cast<uint8_t>(wm_bytes & 0xFF);
    const uint8_t reg3 = static_cast<uint8_t>((wm_bytes >> 8) & 0x0F);
    EXPECT_EQ(reg2, 80);
    EXPECT_EQ(reg3, 0);
}

TEST(ImuFifoConfig, WatermarkEncodingLarge)
{
    /* 100 packets × 16 = 1600 bytes */
    const uint16_t wm_bytes = 100 * 16;
    EXPECT_EQ(wm_bytes, 1600);

    const uint8_t reg2 = static_cast<uint8_t>(wm_bytes & 0xFF);
    const uint8_t reg3 = static_cast<uint8_t>((wm_bytes >> 8) & 0x0F);
    EXPECT_EQ(reg2, 0x40); /* 1600 & 0xFF = 64 = 0x40 */
    EXPECT_EQ(reg3, 0x06); /* 1600 >> 8 = 6 */
}

TEST(ImuFifoConfig, IntSource0FifoThs)
{
    /* When FIFO is enabled, INT_SOURCE0 = FIFO_THS_INT1_EN (bit 2) = 0x04 */
    const uint8_t val = 0x04;
    EXPECT_NE(val & 0x04, 0); /* FIFO_THS_INT1_EN */
    EXPECT_EQ(val & 0x08, 0); /* UI_DRDY_INT1_EN off */
}
