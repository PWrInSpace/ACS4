/*
 * ACS4 Flight Computer — MS5611 Platform-Independent Math (Implementation)
 *
 * See ms5611_math.h for API documentation.
 */

#include "drivers/ms5611_math.h"

#include <cmath>

namespace acs::ms5611
{

/* ═══════════════════════════════════════════════════════════════════════════
 * CRC-4 Verification (AN520)
 * ═══════════════════════════════════════════════════════════════════════════ */

bool verify_crc4(const uint16_t prom[8])
{
    /* Extract stored CRC from prom[7] bits [3:0]. */
    const uint16_t crc_stored = prom[7] & 0x000F;

    /* Work on a copy — the algorithm modifies prom[0] and prom[7]. */
    uint16_t work[8];
    for (int i = 0; i < 8; ++i)
    {
        work[i] = prom[i];
    }

    work[7] &= 0xFF00; /* CRC byte is replaced by 0 */

    uint16_t remainder = 0;
    for (int cnt = 0; cnt < 16; ++cnt)
    {
        if (cnt % 2 == 1)
        {
            remainder ^= work[cnt >> 1] & 0x00FF;
        }
        else
        {
            remainder ^= work[cnt >> 1] >> 8;
        }

        for (int bit = 8; bit > 0; --bit)
        {
            if ((remainder & 0x8000) != 0)
            {
                remainder = (remainder << 1) ^ 0x3000;
            }
            else
            {
                remainder <<= 1;
            }
        }
    }

    remainder = (remainder >> 12) & 0x000F;

    return remainder == crc_stored;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Compensation (datasheet formulas + 2nd order)
 * ═══════════════════════════════════════════════════════════════════════════ */

void compensate(uint32_t       d1,
                uint32_t       d2,
                const uint16_t c[6],
                float         &pressure_pa,
                float         &temperature_c)
{
    /*
     * Variable naming follows the datasheet exactly.
     * All arithmetic uses int64_t to avoid overflow in intermediate results.
     * The datasheet specifies up to 58-bit intermediates.
     */

    /* Temperature difference from reference. */
    const auto dT = static_cast<int64_t>(d2) - (static_cast<int64_t>(c[4]) << 8);

    /* Actual temperature (centidegrees: 2000 = 20.00°C). */
    int64_t TEMP = 2000 + ((dT * static_cast<int64_t>(c[5])) >> 23);

    /* Offset at actual temperature. */
    int64_t OFF = (static_cast<int64_t>(c[1]) << 16) + ((static_cast<int64_t>(c[3]) * dT) >> 7);

    /* Sensitivity at actual temperature. */
    int64_t SENS = (static_cast<int64_t>(c[0]) << 15) + ((static_cast<int64_t>(c[2]) * dT) >> 8);

    /* ── 2nd order temperature compensation ────────────────────────────── */
    int64_t T2    = 0;
    int64_t OFF2  = 0;
    int64_t SENS2 = 0;

    if (TEMP < 2000)
    {
        /* Low temperature (< 20°C). */
        const int64_t temp_diff = TEMP - 2000;
        T2                      = (dT * dT) >> 31;
        OFF2                    = (5 * temp_diff * temp_diff) >> 1;
        SENS2                   = (5 * temp_diff * temp_diff) >> 2;

        if (TEMP < -1500)
        {
            /* Very low temperature (< −15°C). */
            const int64_t temp_diff2 = TEMP + 1500;
            OFF2 += 7 * temp_diff2 * temp_diff2;
            SENS2 += (11 * temp_diff2 * temp_diff2) >> 1;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    /* Temperature compensated pressure (centipascals: 100009 = 1000.09 mbar). */
    const int64_t P = ((static_cast<int64_t>(d1) * SENS >> 21) - OFF) >> 15;

    /* Convert to SI: °C and Pa.
     * TEMP is in centidegrees (2007 = 20.07°C).
     * P is in units of 0.01 mbar; since 0.01 mbar = 1 Pa, P is directly in Pa. */
    temperature_c = static_cast<float>(TEMP) * 0.01f;
    pressure_pa   = static_cast<float>(P);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Altitude Conversion
 * ═══════════════════════════════════════════════════════════════════════════ */

float pressure_to_altitude(float pressure_pa, float qnh_pa)
{
    /*
     * Hypsometric formula (ISA):
     *   h = 44330 * (1 - (P / P0) ^ 0.190284)
     */
    return 44330.0f * (1.0f - std::pow(pressure_pa / qnh_pa, 0.190284f));
}

}  // namespace acs::ms5611
