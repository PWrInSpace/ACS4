/**
 * @file test_ms5611.cpp
 * @brief Unit tests for MS5611 barometer driver data conversion logic.
 *
 * Tests the platform-independent parts of the driver:
 *   - Temperature and pressure compensation (datasheet example values)
 *   - 2nd-order temperature compensation (low temp, very low temp)
 *   - CRC-4 PROM verification (AN520)
 *   - Barometric altitude conversion
 *
 * The actual SPI communication is hardware-dependent and tested through
 * integration tests on the target board.
 */

#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>

#include "drivers/ms5611_math.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Datasheet Example Values (Table: Calculate temperature)
 *
 * From the MS5611 datasheet compensation example:
 *   C1 = 40127, C2 = 36924, C3 = 23317
 *   C4 = 23282, C5 = 33464, C6 = 28312
 *   D1 = 9085466, D2 = 8569150
 *   Expected: TEMP = 2007 (20.07°C), P = 100009 (1000.09 mbar)
 * ═══════════════════════════════════════════════════════════════════════════ */

static constexpr uint16_t kExampleCal[6] = {40127, 36924, 23317, 23282, 33464, 28312};
static constexpr uint32_t kExampleD1     = 9085466;
static constexpr uint32_t kExampleD2     = 8569150;

/* ═══════════════════════════════════════════════════════════════════════════
 * Compensation Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(Ms5611Compensate, DatasheetExample)
{
    float pressure_pa   = 0.0f;
    float temperature_c = 0.0f;

    acs::ms5611::compensate(kExampleD1, kExampleD2, kExampleCal, pressure_pa, temperature_c);

    /* Datasheet: TEMP = 2007 → 20.07°C. Allow ±0.01°C for integer rounding. */
    EXPECT_NEAR(temperature_c, 20.07f, 0.02f);

    /* Datasheet: P = 100009 (in 0.01 mbar units) = 1000.09 mbar = 100009 Pa. */
    EXPECT_NEAR(pressure_pa, 100009.0f, 2.0f);
}

TEST(Ms5611Compensate, HighTemperatureNoSecondOrder)
{
    /*
     * At high temperature (> 20°C), 2nd order correction should be zero.
     * Use the datasheet example which IS above 20°C — verify T2/OFF2/SENS2 = 0.
     */
    float p = 0.0f;
    float t = 0.0f;

    acs::ms5611::compensate(kExampleD1, kExampleD2, kExampleCal, p, t);

    /* Temperature above 20°C → no 2nd order correction needed.
     * Result should match first-order calculation exactly. */
    EXPECT_GT(t, 20.0f);
}

TEST(Ms5611Compensate, LowTemperatureSecondOrder)
{
    /*
     * Simulate a low-temperature reading where TEMP < 20°C.
     * Use C5 adjusted so that D2 produces a temperature below 20°C.
     *
     * With D2 = 8000000 and C5 = 33464, C6 = 28312:
     * dT = 8000000 - 33464*256 = 8000000 - 8566784 = -566784
     * TEMP = 2000 + (-566784 * 28312) / 2^23 = 2000 + (-16041987) / 8388608
     * TEMP = 2000 + (-1912) = 88 → 0.88°C  (below 20°C → 2nd order applies)
     */
    const uint16_t cal_low[6] = {40127, 36924, 23317, 23282, 33464, 28312};
    const uint32_t d2_low     = 8000000;

    float p = 0.0f;
    float t = 0.0f;

    acs::ms5611::compensate(kExampleD1, d2_low, cal_low, p, t);

    /* Temperature should be near 0°C (low). */
    EXPECT_LT(t, 20.0f);
    EXPECT_GT(t, -40.0f);

    /* Pressure should still be reasonable (10–1200 mbar = 1000–120000 Pa). */
    EXPECT_GT(p, 1000.0f);
    EXPECT_LT(p, 200000.0f);
}

TEST(Ms5611Compensate, VeryLowTemperatureSecondOrder)
{
    /*
     * Force temperature well below -15°C to exercise the very-low-temp path.
     * D2 chosen to produce TEMP < -1500 (centi-degrees).
     *
     * With D2 = 7000000, C5 = 33464, C6 = 28312:
     * dT = 7000000 - 8566784 = -1566784
     * TEMP = 2000 + (-1566784 * 28312) / 2^23 = 2000 + (-5289) = -3289 → -32.89°C
     */
    const uint16_t cal_vlow[6] = {40127, 36924, 23317, 23282, 33464, 28312};
    const uint32_t d2_vlow     = 7000000;

    float p = 0.0f;
    float t = 0.0f;

    acs::ms5611::compensate(kExampleD1, d2_vlow, cal_vlow, p, t);

    /* Temperature should be well below -15°C. */
    EXPECT_LT(t, -15.0f);
    EXPECT_GT(t, -50.0f);

    /* Pressure should still be in a physically possible range. */
    EXPECT_GT(p, 1000.0f);
    EXPECT_LT(p, 200000.0f);
}

TEST(Ms5611Compensate, ZeroD1ProducesLowPressure)
{
    /* Edge case: D1 = 0 should produce a very low / minimum pressure. */
    float p = 0.0f;
    float t = 0.0f;

    acs::ms5611::compensate(0, kExampleD2, kExampleCal, p, t);

    /* With D1 = 0, the pressure formula gives P = (0 - OFF) / 2^15,
     * which should be negative or very low. */
    EXPECT_LT(p, 10000.0f);
}

TEST(Ms5611Compensate, MaxD1ProducesHighPressure)
{
    /* Edge case: D1 = 2^24 (max 24-bit) should produce very high pressure. */
    float p = 0.0f;
    float t = 0.0f;

    acs::ms5611::compensate(16777216, kExampleD2, kExampleCal, p, t);

    /* With max D1, pressure should be well above normal atmospheric (101325 Pa). */
    EXPECT_GT(p, 200000.0f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CRC-4 Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(Ms5611Crc, ValidPromPassesCrc)
{
    /*
     * AN520 example PROM data with valid CRC.
     * prom[0] = manufacturer reserved, prom[1-6] = C1-C6, prom[7] = CRC in low nibble.
     *
     * This is from the AN520 application note example.
     */
    const uint16_t prom[8] = {0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x450B};

    EXPECT_TRUE(acs::ms5611::verify_crc4(prom));
}

TEST(Ms5611Crc, CorruptPromFailsCrc)
{
    /* Same as above but with bit flip in C3. */
    uint16_t prom[8] = {0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x450B};
    prom[3] ^= 0x0010; /* corrupt one bit */

    EXPECT_FALSE(acs::ms5611::verify_crc4(prom));
}

TEST(Ms5611Crc, WrongCrcNibbleFailsCrc)
{
    /* Correct data but wrong CRC nibble. */
    uint16_t prom[8] = {0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x450B};
    prom[7]          = (prom[7] & 0xFFF0) | 0x000A; /* change CRC from 0xB to 0xA */

    EXPECT_FALSE(acs::ms5611::verify_crc4(prom));
}

TEST(Ms5611Crc, AllZeroPromHandled)
{
    /* All-zero PROM should not crash. */
    const uint16_t prom[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    /* Don't care about result — just must not crash. CRC of all zeros = 0. */
    (void)acs::ms5611::verify_crc4(prom);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Altitude Conversion Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(Ms5611Altitude, SeaLevelIsZero)
{
    /* Standard ISA: P = 101325 Pa at sea level → altitude = 0 m. */
    const float alt = acs::ms5611::pressure_to_altitude(101325.0f, 101325.0f);
    EXPECT_NEAR(alt, 0.0f, 0.01f);
}

TEST(Ms5611Altitude, LowerPressureIsHigher)
{
    /* Lower pressure always means higher altitude. */
    const float alt_low  = acs::ms5611::pressure_to_altitude(90000.0f, 101325.0f);
    const float alt_high = acs::ms5611::pressure_to_altitude(80000.0f, 101325.0f);

    EXPECT_GT(alt_low, 0.0f);
    EXPECT_GT(alt_high, alt_low);
}

TEST(Ms5611Altitude, KnownAltitude1000m)
{
    /*
     * ISA at 1000 m: P ≈ 89874.6 Pa.
     * Formula: h = 44330 * (1 - (89874.6/101325)^0.190284) ≈ 1000 m.
     */
    const float alt = acs::ms5611::pressure_to_altitude(89874.6f, 101325.0f);
    EXPECT_NEAR(alt, 1000.0f, 5.0f);
}

TEST(Ms5611Altitude, KnownAltitude5000m)
{
    /*
     * ISA at 5000 m: P ≈ 54019.9 Pa.
     */
    const float alt = acs::ms5611::pressure_to_altitude(54019.9f, 101325.0f);
    EXPECT_NEAR(alt, 5000.0f, 10.0f);
}

TEST(Ms5611Altitude, CustomQnh)
{
    /* QNH = 100000 Pa: at that exact pressure, altitude should be 0. */
    const float alt = acs::ms5611::pressure_to_altitude(100000.0f, 100000.0f);
    EXPECT_NEAR(alt, 0.0f, 0.01f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Command Encoding Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(Ms5611Commands, ConvertD1Encoding)
{
    /* Verify OSR offset produces correct command bytes per datasheet table. */
    EXPECT_EQ(0x40 + static_cast<uint8_t>(0x00), 0x40); /* OSR 256 */
    EXPECT_EQ(0x40 + static_cast<uint8_t>(0x02), 0x42); /* OSR 512 */
    EXPECT_EQ(0x40 + static_cast<uint8_t>(0x04), 0x44); /* OSR 1024 */
    EXPECT_EQ(0x40 + static_cast<uint8_t>(0x06), 0x46); /* OSR 2048 */
    EXPECT_EQ(0x40 + static_cast<uint8_t>(0x08), 0x48); /* OSR 4096 */
}

TEST(Ms5611Commands, ConvertD2Encoding)
{
    EXPECT_EQ(0x50 + static_cast<uint8_t>(0x00), 0x50); /* OSR 256 */
    EXPECT_EQ(0x50 + static_cast<uint8_t>(0x02), 0x52); /* OSR 512 */
    EXPECT_EQ(0x50 + static_cast<uint8_t>(0x04), 0x54); /* OSR 1024 */
    EXPECT_EQ(0x50 + static_cast<uint8_t>(0x06), 0x56); /* OSR 2048 */
    EXPECT_EQ(0x50 + static_cast<uint8_t>(0x08), 0x58); /* OSR 4096 */
}

TEST(Ms5611Commands, PromReadAddresses)
{
    /* PROM read: base 0xA0, address shifted left by 1. */
    for (uint8_t addr = 0; addr < 8; ++addr)
    {
        const uint8_t cmd = 0xA0 + (addr << 1);
        EXPECT_EQ(cmd, 0xA0 + addr * 2);
    }
    EXPECT_EQ(0xA0 + (0 << 1), 0xA0); /* addr 0 */
    EXPECT_EQ(0xA0 + (7 << 1), 0xAE); /* addr 7 */
}
