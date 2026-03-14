/*
 * ACS4 Flight Computer — MS5611 Barometric Pressure Sensor Driver (Implementation)
 *
 * MS5611-01BA03 piezo-resistive barometer with 24-bit ADC.
 * See ms5611.h for API documentation.
 *
 * State machine design: update() never blocks. Init is the only
 * blocking call (reset + PROM read, called once at boot).
 *
 * Compensation formulas implement the full datasheet algorithm
 * including 2nd-order temperature correction for low temps.
 *
 * All bus access goes through the SpiBus abstraction (DMA, mutex-protected).
 */

#include "drivers/ms5611.h"

#include <cmath>

#include "system/error_handler.h"
#include "utils/timestamp.h"

extern "C" {
#include "ch.h"
}

namespace acs
{

namespace
{

constexpr uint8_t CMD_RESET      = 0x1E;
constexpr uint8_t CMD_ADC_READ   = 0x00;
constexpr uint8_t CMD_CONVERT_D1 = 0x40; /* base command, add OSR offset */
constexpr uint8_t CMD_CONVERT_D2 = 0x50; /* base command, add OSR offset */
constexpr uint8_t CMD_PROM_READ  = 0xA0; /* base addr, + (index << 1) */

}  // namespace

// NOLINTBEGIN(cppcoreguidelines-avoid-do-while)

/** Write-or-fail: calls report_error() and returns false on SPI failure. */
#define BARO_TRY(expr)      \
    do                      \
    {                       \
        if (!(expr))        \
        {                   \
            report_error(); \
            return false;   \
        }                   \
    }                       \
    while (0)

/* ====================
 * Handlowanie errorow
 * ==================== */

void Ms5611::report_error()
{
    ++error_count_;
    error_report(ErrorCode::BARO_COMM_FAIL);
}

/* ============
 * SPI Helpers
 * ============= */

bool Ms5611::send_command(uint8_t cmd)
{
    /*
     * MS5611 SPI commands are a single byte on SDI.
     * Unlike register-based sensors, there is no read/write address bit —
     * the entire byte is the command.
     */
    return spi_->send(cs_line_, &cmd, 1, *spi_cfg_);
}

bool Ms5611::read_adc(uint32_t &result)
{
    /*
     * ADC Read: send 0x00, then clock out 3 bytes (MSB first).
     * Total transfer: 1 cmd byte + 3 data bytes = 4 bytes.
     */
    uint8_t tx[4] = {CMD_ADC_READ, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {};

    if (!spi_->transfer(cs_line_, tx, rx, 4, *spi_cfg_))
    {
        return false;
    }

    result = (static_cast<uint32_t>(rx[1]) << 16) | (static_cast<uint32_t>(rx[2]) << 8)
             | static_cast<uint32_t>(rx[3]);

    return true;
}

bool Ms5611::read_prom(uint16_t prom[8])
{
    /*
     * PROM Read: 8 addresses × 16-bit words.
     * Command = 0xA0 + (addr << 1), response is 2 bytes MSB first.
     */
    for (uint8_t addr = 0; addr < 8; ++addr)
    {
        uint8_t tx[3] = {static_cast<uint8_t>(CMD_PROM_READ + (addr << 1)), 0x00, 0x00};
        uint8_t rx[3] = {};

        if (!spi_->transfer(cs_line_, tx, rx, 3, *spi_cfg_))
        {
            return false;
        }

        prom[addr] = (static_cast<uint16_t>(rx[1]) << 8) | rx[2];
    }

    return true;
}

/* =======================================
 * Maskymalny czas konwersji Lookup table
 * ======================================= */

uint32_t Ms5611::conversion_time_us(Ms5611Osr osr)
{
    /*
     * Maksymalne czasy konwersji z datasheeta (zaokraglone z marginesem bezpieczenstwa):
     *   OSR 256:   600 µs
     *   OSR 512:  1170 µs
     *   OSR 1024: 2280 µs
     *   OSR 2048: 4540 µs
     *   OSR 4096: 9040 µs
     */
    switch (osr)
    {
        case Ms5611Osr::OSR_256:
            return 600;
        case Ms5611Osr::OSR_512:
            return 1170;
        case Ms5611Osr::OSR_1024:
            return 2280;
        case Ms5611Osr::OSR_2048:
            return 4540;
        case Ms5611Osr::OSR_4096:
        default:
            return 9040;
    }
}

/* =======================
 * Thread-Safe Accessors
 * ======================= */

bool Ms5611::has_new_data() const
{
    return new_data_;
}

BaroSample Ms5611::sample()
{
    chSysLock();
    new_data_    = false;
    BaroSample s = last_sample_;
    chSysUnlock();
    return s;
}

/* ==================================================================
 * Inicjalizacja (blokulacja, ale uruchamiana tylko raz przy boocie)
 * ================================================================== */

bool Ms5611::init(SpiBus &spi, ioline_t cs_line, const SPIConfig &spi_cfg, const Ms5611Config &cfg)
{
    spi_     = &spi;
    cs_line_ = cs_line;
    spi_cfg_ = &spi_cfg;
    osr_          = cfg.osr;
    qnh_pa_       = cfg.qnh_pa;
    conv_time_us_ = conversion_time_us(osr_);

    /*
     * Krok 1: Reset.
     * Laduje fabryczne dane kalibracyjne z PROM do wewnetrzynych rejestrow
     * Trzeba czekac co najmniej 2.8 ms po resecie
     */
    BARO_TRY(send_command(CMD_RESET));
    chThdSleepMilliseconds(3);

    /*
     * Krok 2: Odczytaj PROM (8 × 16-bit words).
     */
    uint16_t prom[8] = {};
    BARO_TRY(read_prom(prom));

    /*
     * Step 3: Zweryfikuj CRC-4.
     */
    if (!ms5611::verify_crc4(prom))
    {
        report_error();
        return false;
    }

    /* Wyekstraktuj wspolczynniki kalibracyjne C1–C6 (PROM addresses 1–6). */
    for (int i = 0; i < 6; ++i)
    {
        cal_[i] = prom[i + 1];
    }

    state_       = State::CONVERT_D1;
    initialized_ = true;
    return true;
}

/* ========================
 * Compensation + Publish
 * ======================== */

void Ms5611::compute_and_publish()
{
    float pressure_pa   = 0.0f;
    float temperature_c = 0.0f;
    ms5611::compensate(raw_d1_, raw_d2_, cal_, pressure_pa, temperature_c);

    BaroSample s{};
    s.pressure_pa   = pressure_pa;
    s.temperature_c = temperature_c;
    s.altitude_m    = ms5611::pressure_to_altitude(pressure_pa, qnh_pa_);
    s.timestamp_us  = timestamp_us();

    chSysLock();
    last_sample_ = s;
    new_data_    = true;
    chSysUnlock();
}

/* ============================
 * Nieblokujaca maszyna stanow
 * ============================ */

void Ms5611::update()
{
    if (!initialized_)
    {
        return;
    }

    switch (state_)
    {
        case State::CONVERT_D1:
        {
            const uint8_t cmd = CMD_CONVERT_D1 + static_cast<uint8_t>(osr_);
            if (!send_command(cmd))
            {
                report_error();
                return;
            }
            conv_start_us_ = timestamp_us();
            state_         = State::WAIT_D1;
            break;
        }

        case State::WAIT_D1:
        {
            const uint32_t elapsed = timestamp_us() - conv_start_us_;
            if (elapsed >= conv_time_us_)
            {
                state_ = State::READ_D1;
            }
            break;
        }

        case State::READ_D1:
        {
            if (!read_adc(raw_d1_))
            {
                report_error();
                state_ = State::CONVERT_D1; /* retry od poczatku */
                return;
            }

            /* Zero ADC wynik oznacza ze konwersja nie byla gotowa lub sie nie rozpoczela. */
            if (raw_d1_ == 0)
            {
                state_ = State::CONVERT_D1;
                return;
            }

            state_ = State::CONVERT_D2;
            break;
        }

        case State::CONVERT_D2:
        {
            const uint8_t cmd = CMD_CONVERT_D2 + static_cast<uint8_t>(osr_);
            if (!send_command(cmd))
            {
                report_error();
                return;
            }
            conv_start_us_ = timestamp_us();
            state_         = State::WAIT_D2;
            break;
        }

        case State::WAIT_D2:
        {
            const uint32_t elapsed = timestamp_us() - conv_start_us_;
            if (elapsed >= conv_time_us_)
            {
                state_ = State::READ_D2;
            }
            break;
        }

        case State::READ_D2:
        {
            if (!read_adc(raw_d2_))
            {
                report_error();
                state_ = State::CONVERT_D1;
                return;
            }

            if (raw_d2_ == 0)
            {
                state_ = State::CONVERT_D1;
                return;
            }

            compute_and_publish();

            /* Zloopbackuj do poczatku nowego cyklu */
            state_ = State::CONVERT_D1;
            break;
        }

        case State::IDLE:
        default:
            break;
    }
}

}  // namespace acs

#undef BARO_TRY

// NOLINTEND(cppcoreguidelines-avoid-do-while)
