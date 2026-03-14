/*
 * ACS4 Flight Computer — MS5611 Platform-Independent Math
 *
 * Pure computation functions for MS5611 barometer:
 *   - Temperature/pressure compensation (datasheet + 2nd order)
 *   - CRC-4 PROM verification (AN520)
 *   - Barometric altitude (hypsometric formula)
 *
 * No RTOS or hardware dependencies — safe just for unit tests on host.
 */

#pragma once

#include <cstdint>

namespace acs::ms5611
{

/**
 * @brief Compute compensated temperature and pressure from raw ADC values.
 *
 * Implements full MS5611 compensation with 2nd-order correction
 * per datasheet. All intermediate calculations use int64_t to
 * preserve precision and avoid overflow.
 *
 * @param d1        Raw pressure ADC value (24-bit unsigned).
 * @param d2        Raw temperature ADC value (24-bit unsigned).
 * @param c         PROM calibration coefficients C1–C6 (array index 0–5).
 * @param[out] pressure_pa   Compensated pressure in Pa.
 * @param[out] temperature_c Compensated temperature in °C.
 */
void compensate(uint32_t       d1,
                uint32_t       d2,
                const uint16_t c[6],
                float         &pressure_pa,
                float         &temperature_c);

/**
 * @brief Verify CRC-4 of PROM data per AN520.
 *
 * @param prom  8-element array of 16-bit PROM words (addresses 0–7).
 * @return true if CRC matches.
 */
bool verify_crc4(const uint16_t prom[8]);

/**
 * @brief Convert pressure to barometric altitude (hypsometric formula).
 *
 * @param pressure_pa  Measured pressure in Pa.
 * @param qnh_pa       Reference sea-level pressure in Pa.
 * @return Altitude in meters above the QNH reference.
 */
float pressure_to_altitude(float pressure_pa, float qnh_pa);

}  // namespace acs::ms5611
