/*
 * ACS4 Flight Computer — MS5611 Niezalezna platformowo matma 
 *
 */

#include "drivers/ms5611_math.h"

#include <algorithm>
#include <cmath>

namespace acs::ms5611
{

/* ===========================
 * CRC-4 Verification (AN520)
 * =========================== */

bool verify_crc4(const uint16_t prom[8])
{
    /* Wyekstraktuj storagowany CRC z prom[7] bits [3:0]. */
    const uint16_t crc_stored = prom[7] & 0x000F;

    /* Pracujemy na kopii — algorytm modyfikuje prom[0] i prom[7]. */
    uint16_t work[8];
    std::copy(prom, prom + 8, work);

    work[7] &= 0xFF00; 

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

/* ==============================================
 * Kompensacja (wzory z datasheeta + drugi rzad)
 * ============================================== */

void compensate(uint32_t       d1,
                uint32_t       d2,
                const uint16_t c[6],
                float         &pressure_pa,
                float         &temperature_c)
{
    /*
     * Nazwy zmiennych zgodne z datasheetem.
     * Cala arytmetyka uzywa int64_t by uniknac overflow w posrednich wynikach.
     * Datasheet przewiduje wartosci posrednie do ~58 bitow.
     */

    /* Roznica temperatury wzgledem temperatury referencyjnej. */
    const auto dT = static_cast<int64_t>(d2) - (static_cast<int64_t>(c[4]) << 8);

    /* Rzeczywista temperatura (w setnych części stopnia: 2000 = 20.00°C). */
    int64_t TEMP = 2000 + ((dT * static_cast<int64_t>(c[5])) >> 23);

    /* Offset przy aktualnej temperaturze. */
    int64_t OFF = (static_cast<int64_t>(c[1]) << 16) + ((static_cast<int64_t>(c[3]) * dT) >> 7);

    /* Czułość (sensitivity) przy aktualnej temperaturze. */
    int64_t SENS = (static_cast<int64_t>(c[0]) << 15) + ((static_cast<int64_t>(c[2]) * dT) >> 8);

    /* Kompensacja temperaturowa drugiego rzedu*/
    int64_t T2    = 0;
    int64_t OFF2  = 0;
    int64_t SENS2 = 0;

    if (TEMP < 2000)
    {
        /* Niska temperatura (< 20 deg_C). */
        const int64_t temp_diff = TEMP - 2000;
        T2                      = (dT * dT) >> 31;
        OFF2                    = (5 * temp_diff * temp_diff) >> 1;
        SENS2                   = (5 * temp_diff * temp_diff) >> 2;

        if (TEMP < -1500)
        {
            /* Bardzo niska temperatura (< −15 deg_C). */
            const int64_t temp_diff2 = TEMP + 1500;
            OFF2 += 7 * temp_diff2 * temp_diff2;
            SENS2 += (11 * temp_diff2 * temp_diff2) >> 1;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    /* Cisnienie skompensowane temperaturowo (centipascals: 100009 = 1000.09 mbar). */
    const int64_t P = ((static_cast<int64_t>(d1) * SENS >> 21) - OFF) >> 15;

    /* Konwersja do SI, Paskale i stopnie Celsjusza.
     * TEMP is in centidegrees (2007 = 20.07 deg_C).
     * P is in units of 0.01 mbar; since 0.01 mbar = 1 Pa, P is directly in Pa. */
    temperature_c = static_cast<float>(TEMP) * 0.01f;
    pressure_pa   = static_cast<float>(P);
}

/* ===================
 * Liczenie wysokosci
 * =================== */

float pressure_to_altitude(float pressure_pa, float qnh_pa)
{
    /*
     * Hypsometric formula (ISA):
     *   h = 44330 * (1 - (P / P0) ^ 0.190284)
     */
    return 44330.0f * (1.0f - std::pow(pressure_pa / qnh_pa, 0.190284f));
}

}  // namespace acs::ms5611
