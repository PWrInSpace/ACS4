/*
 * ACS4 Flight Computer — IIM-42653 IMU Driver
 *
 * TDK InvenSense IIM-42653 6-axis IMU.
 *
 * Sekwencja inicjalizacji jest zgodna z uwzglednieniem constraintsow
 * czasowych czyli:
 *   - 1 ms po soft resecie
 *   - 200 mikrosekund po wlaczeniu czujnikow rejestrze PWR_MGMT0
 *   - 45 ms minimum czasu startu zyroskopu (specyfikacja rejestru PWR_MGMT0)
 *   - INT_ASYNC_RESET musi zostac wyczyszczony w rejestrze INT_CONFIG1
 *
 * Cala komunikacja z magistrala idzie przez abstrakcje SpiBus
 */

#include "drivers/iim42653.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "system/error_handler.h"
#include "utils/timestamp.h"

extern "C" {
#include "ch.h"
}

namespace acs
{

using namespace iim42653_reg;

// NOLINTBEGIN(cppcoreguidelines-avoid-do-while)

/** Write-or-fail: wywoluje report_error() i zwraca false w przypadku bledu SPI. */
#define IMU_TRY(expr)       \
    do                      \
    {                       \
        if (!(expr))        \
        {                   \
            report_error(); \
            return false;   \
        }                   \
    }                       \
    while (0)

/* ==================
 * Parsowanie bajtow
 * ================== */

static constexpr int16_t parse_be16(const uint8_t *p)
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) << 8 | p[1]);
}

/** Raw value 0x8000 (−32768) oznacza ze dane z czujnika nie sa jeszcze gotowe. */
static constexpr int16_t kInvalidRaw = -32768;

bool Iim42653::has_invalid(const int16_t data[3])
{
    return data[0] == kInvalidRaw || data[1] == kInvalidRaw || data[2] == kInvalidRaw;
}

void Iim42653::convert_inertial(const int16_t accel[3],
                                const int16_t gyro[3],
                                ImuSample    &sample) const
{
    for (int i = 0; i < 3; ++i)
    {
        sample.accel_mps2[i] = static_cast<float>(accel[i]) * accel_scale_;
        sample.gyro_rads[i]  = static_cast<float>(gyro[i]) * gyro_scale_;
    }
}

/* ========================
 * Helpery do rejestrow
 * ======================== */

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

    /* Zawsze wracaj do bank 0. */
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

    /* Zawsze wracaj do bank 0. */
    (void)select_bank(0);

    return val;
}

bool Iim42653::modify_reg(uint8_t reg, uint8_t clear_mask, uint8_t set_mask)
{
    auto val = read_reg(reg);
    if (!val.has_value())
    {
        return false;
    }

    return write_reg(reg, (val.value() & ~clear_mask) | set_mask);
}

bool Iim42653::modify_bank_reg(uint8_t bank, uint8_t reg, uint8_t clear_mask, uint8_t set_mask)
{
    if (!select_bank(bank))
    {
        return false;
    }

    const bool ok = modify_reg(reg, clear_mask, set_mask);

    /* Zawsze wracaj do bank 0. */
    (void)select_bank(0);

    return ok;
}

/* ====================
 * Handlowanie errorow
 * ==================== */

void Iim42653::report_error()
{
    ++error_count_;
    error_report(ErrorCode::IMU_COMM_FAIL);
}

/* ========================
 * Wspolczynniki skalowania
 * ======================== */

float Iim42653::gyro_sensitivity(GyroFsr fsr)
{
    static constexpr float kTable[] = {8.2f, 16.4f, 32.8f, 65.5f, 131.0f, 262.0f, 524.3f, 1048.6f};
    const auto             idx      = static_cast<uint8_t>(fsr) >> 5;
    chDbgAssert(idx < 8, "invalid gyro FSR");
    return (idx < 8) ? kTable[idx] : kTable[1];
}

float Iim42653::accel_sensitivity(AccelFsr fsr)
{
    static constexpr float kTable[] = {1024.0f, 2048.0f, 4096.0f, 8192.0f};
    const auto             idx      = static_cast<uint8_t>(fsr) >> 5;
    chDbgAssert(idx < 4, "invalid accel FSR");
    return (idx < 4) ? kTable[idx] : kTable[0];
}

void Iim42653::compute_scale_factors()
{
    /*
     * Gyro:  raw / sensitivity_lsb_per_dps * (pi/180) = rad/s
     * Accel: raw / sensitivity_lsb_per_g * 9.80665 = m/s^2
     */
    gyro_scale_  = kDeg2Rad / gyro_sensitivity(config_.gyro_fsr);
    accel_scale_ = kGravity / accel_sensitivity(config_.accel_fsr);
}

/* =======================
 * Inicjalizacja czujnika
 * ======================= */

bool Iim42653::init(SpiBus &spi, ioline_t cs_line, const SPIConfig &spi_cfg)
{
    spi_          = &spi;
    cs_line_      = cs_line;
    spi_cfg_      = &spi_cfg;
    current_bank_ = 0;

    /*
     * Krok 1: Soft reset.
     * Zapisac SOFT_RESET_CONFIG = 1 do DEVICE_CONFIG (0x11).
     * Poczekac 1 ms na zakonczenie resetu.
     */
    IMU_TRY(write_reg(DEVICE_CONFIG, SOFT_RESET_EN));
    chThdSleepMilliseconds(2);

    /*
     * Krok 2: Zweryfikowac WHO_AM_I.
     * Zwrocona wartosc powinna byc: 0x56 dla IIM-42653.
     */
    auto who = read_reg(WHO_AM_I);
    if (!who.has_value() || who.value() != WHO_AM_I_VALUE)
    {
        report_error();
        return false;
    }

    /*
     * Krok 3-7: Skonfiguruj interfejsy, zegar, SPI, I3C.
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
    /* Wyczysc INT_ASYNC_RESET (bit 4) w INT_CONFIG1. */
    IMU_TRY(modify_reg(INT_CONFIG1, INT_ASYNC_RESET_BIT, 0x00));

    /* Konfiguracja interfejsow: big-endian, licznik FIFO w bajtach, SPI 4-wire,
     * wylacz I2C/I3C slave interfejs (UI_SIFS_CFG = 0b11). */
    IMU_TRY(write_reg(INTF_CONFIG0, INTF0_BIGENDIAN_SPI_ONLY));

    /* Zrodlo zegara — PLL auto-select (CLKSEL = 01). */
    IMU_TRY(modify_reg(INTF_CONFIG1, CLKSEL_MASK, CLKSEL_PLL_AUTO));

    /* Konfiguracja SPI slew rate. */
    IMU_TRY(write_reg(DRIVE_CONFIG, SPI_SLEW_DEFAULT));

    /* Trzeba wylaczyc I3C by pracowalo jedynie SPI.
     * Wartosc resetu to 0x5F — trzeba zachowac zarezerwowany 6 bit i wyczysic I3C bits [4:0]. */
    IMU_TRY(modify_bank_reg(1, INTF_CONFIG6, I3C_EN_MASK, 0x00));

    return true;
}

/* ===============
 * Konfiguracja
 * =============== */

bool Iim42653::configure(const Iim42653Config &cfg)
{
    if (!initialized_)
    {
        return false;
    }

    config_ = cfg;
    compute_scale_factors();

    /* Musimy sprawdzic czy czujniki sa wylaczone przed configiem. */
    IMU_TRY(write_reg(PWR_MGMT0, PWR_OFF));
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

    /* Wlaczamy czujniki: gyro i accel w trybie low noise, temperatura wlaczona. */
    IMU_TRY(write_reg(PWR_MGMT0, PWR_GYRO_ACCEL_LN));

    /* Czekamy na startup gyro (w datasheecie 45 ms minimum) + maly zapas czasu. */
    chThdSleepMilliseconds(50);

    return true;
}

bool Iim42653::configure_odr_fsr(const Iim42653Config &cfg)
{
    /* GYRO_CONFIG0: [7:5] = FSR, [3:0] = ODR. */
    const uint8_t gyro_config =
        static_cast<uint8_t>(cfg.gyro_fsr) | static_cast<uint8_t>(cfg.gyro_odr);
    IMU_TRY(write_reg(GYRO_CONFIG0, gyro_config));

    /* ACCEL_CONFIG0: [7:5] = FSR, [3:0] = ODR. */
    const uint8_t accel_config =
        static_cast<uint8_t>(cfg.accel_fsr) | static_cast<uint8_t>(cfg.accel_odr);
    IMU_TRY(write_reg(ACCEL_CONFIG0, accel_config));

    return true;
}

bool Iim42653::configure_ui_filters(const Iim42653Config &cfg)
{
    /* GYRO_CONFIG1: [7:5] = TEMP_FILT_BW, [3:2] = GYRO_UI_FILT_ORD, preserve [4] (reserved) and
     * [1:0]. */
    IMU_TRY(modify_reg(GYRO_CONFIG1,
                       GYRO_CFG1_FILTER_MASK,
                       (static_cast<uint8_t>(cfg.temp_filter_bw) << 5)
                           | (static_cast<uint8_t>(cfg.gyro_filter_order) << 2)));

    /* ACCEL_CONFIG1: [4:3] = ACCEL_UI_FILT_ORD. */
    IMU_TRY(modify_reg(ACCEL_CONFIG1,
                       ACCEL_CFG1_FILT_ORD_MASK,
                       static_cast<uint8_t>(cfg.accel_filter_order) << 3));

    /* GYRO_ACCEL_CONFIG0: [7:4] = accel BW, [3:0] = gyro BW. */
    const uint8_t filter_bw =
        (static_cast<uint8_t>(cfg.accel_filter_bw) << 4) | static_cast<uint8_t>(cfg.gyro_filter_bw);
    IMU_TRY(write_reg(GYRO_ACCEL_CONFIG0, filter_bw));

    return true;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool Iim42653::configure_aaf(const Iim42653Config &cfg)
{
    /* Gyro AAF - Bank 1 (do wszystkich rejestrow gyro AAF musimy zrobic pojedynczego bank switcha).
     */
    {
        const BankScope bank(*this, 1);
        if (!bank)
        {
            report_error();
            return false;
        }

        if (cfg.gyro_aaf.is_enabled())
        {
            IMU_TRY(write_reg(GYRO_CONFIG_STATIC3, cfg.gyro_aaf.delt & 0x3F));
            IMU_TRY(
                write_reg(GYRO_CONFIG_STATIC4, static_cast<uint8_t>(cfg.gyro_aaf.deltsqr & 0xFF)));
            IMU_TRY(write_reg(GYRO_CONFIG_STATIC5,
                              static_cast<uint8_t>((cfg.gyro_aaf.bitshift << 4)
                                                   | ((cfg.gyro_aaf.deltsqr >> 8) & 0x0F))));

            /* Wlacz gyro AAF (wyczysc GYRO_AAF_DIS bit 1). */
            IMU_TRY(modify_reg(GYRO_CONFIG_STATIC2, GYRO_AAF_DIS, 0x00));
        }
        else
        {
            /* Jawnie wylacz gyro AAF (ustaw GYRO_AAF_DIS bit 1). */
            IMU_TRY(modify_reg(GYRO_CONFIG_STATIC2, 0x00, GYRO_AAF_DIS));
        }
    }

    /* Accel AAF - Bank 2 (Jedno przelaczeniu banku dla wszystkich rejestrow accel AAF). */
    {
        const BankScope bank(*this, 2);
        if (!bank)
        {
            report_error();
            return false;
        }

        if (cfg.accel_aaf.is_enabled())
        {
            /* ACCEL_CONFIG_STATIC2: [6:1] = DELT, bit 0 = AAF_DIS = 0 (wlaczone). */
            IMU_TRY(write_reg(ACCEL_CONFIG_STATIC2,
                              static_cast<uint8_t>((cfg.accel_aaf.delt & 0x3F) << 1)));
            IMU_TRY(write_reg(ACCEL_CONFIG_STATIC3,
                              static_cast<uint8_t>(cfg.accel_aaf.deltsqr & 0xFF)));
            IMU_TRY(write_reg(ACCEL_CONFIG_STATIC4,
                              static_cast<uint8_t>((cfg.accel_aaf.bitshift << 4)
                                                   | ((cfg.accel_aaf.deltsqr >> 8) & 0x0F))));
        }
        else
        {
            /* Jawnie wylacz accel AAF (ustaw ACCEL_AAF_DIS bit 0). */
            IMU_TRY(modify_reg(ACCEL_CONFIG_STATIC2, 0x00, ACCEL_AAF_DIS));
        }
    }

    return true;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool Iim42653::configure_notch_filter(const Iim42653Config &cfg)
{
    if (!cfg.gyro_notch.is_enabled())
    {
        /* Upewnij sie ze gyro notch filter jest wylaczony (ustaw GYRO_NF_DIS = bit 0). */
        IMU_TRY(modify_bank_reg(1, GYRO_CONFIG_STATIC2, 0x00, GYRO_NF_DIS));
        return true;
    }

    /*
     * Obliczamy NF_COSWZ and NF_COSWZ_SEL zgodnie z datasheetem (patrz 5.2.1 w datasheet).
     * fdesired podane jest w Hz, a we wzorze sa kHz: cos(2pi·f_kHz/8).
     */
    const float f_khz = cfg.gyro_notch.freq_hz / 1000.0f;
    const float coswz = std::cos(2.0f * kPi * f_khz / 8.0f);

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

    /* Musimy to zakodowac jako 9-bitowa wartosc bez znaku do zapisania w rejestrach */
    const auto coswz_u9  = static_cast<uint16_t>(nf_coswz_raw & 0x01FF);
    const auto coswz_low = static_cast<uint8_t>(coswz_u9 & 0xFF);
    const auto coswz_hi  = static_cast<uint8_t>((coswz_u9 >> 8) & 0x01);

    /*
     * GYRO_CONFIG_STATIC9 (0x12):
     *   bit 0 = X NF_COSWZ[8]
     *   bit 1 = Y NF_COSWZ[8]
     *   bit 2 = Z NF_COSWZ[8]
     *   bit 3 = X NF_COSWZ_SEL
     *   bit 4 = Y NF_COSWZ_SEL
     *   bit 5 = Z NF_COSWZ_SEL
     */
    const uint8_t static9 = (coswz_hi << 0) | (coswz_hi << 1) | (coswz_hi << 2)
                            | (nf_coswz_sel << 3) | (nf_coswz_sel << 4) | (nf_coswz_sel << 5);

    /* Zapisz wszystkie rejestry filtra notch w jednej sesji banku 1 */
    {
        const BankScope bank(*this, 1);
        if (!bank)
        {
            report_error();
            return false;
        }

        /* Ta sama czestotliwosc dla wszystkich 3 osi */
        IMU_TRY(write_reg(GYRO_CONFIG_STATIC6, coswz_low));
        IMU_TRY(write_reg(GYRO_CONFIG_STATIC7, coswz_low));
        IMU_TRY(write_reg(GYRO_CONFIG_STATIC8, coswz_low));
        IMU_TRY(write_reg(GYRO_CONFIG_STATIC9, static9));

        /* GYRO_CONFIG_STATIC10 (0x13): [6:4] = NF_BW_SEL. */
        IMU_TRY(modify_reg(GYRO_CONFIG_STATIC10,
                           NF_BW_SEL_MASK,
                           static_cast<uint8_t>(cfg.gyro_notch.bw) << 4));

        /* Wlacz notch filter: clear GYRO_NF_DIS (bit 0) w GYRO_CONFIG_STATIC2. */
        IMU_TRY(modify_reg(GYRO_CONFIG_STATIC2, GYRO_NF_DIS, 0x00));
    }

    return true;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool Iim42653::configure_interrupts(const Iim42653Config &cfg)
{
    if (!cfg.enable_drdy_int1)
    {
        return true;
    }

    /* INT_CONFIG: active-high, push-pull, pulsed. */
    IMU_TRY(write_reg(INT_CONFIG, INT_ACTIVE_HIGH_PP_PULSE));

    if (cfg.enable_fifo)
    {
        /* FIFO mode: skieruj przerwanie FIFO_THS (watermark) na INT1. */
        IMU_TRY(write_reg(INT_CONFIG0, INT_CFG0_FIFO_MODE));
        IMU_TRY(write_reg(INT_SOURCE0, FIFO_THS_INT1_EN));
    }
    else
    {
        /* Register-read mode: przerwanie DRDY na INT1. */
        IMU_TRY(write_reg(INT_CONFIG0, INT_CFG0_DRDY_MODE));
        IMU_TRY(write_reg(INT_SOURCE0, UI_DRDY_INT1_EN));
    }

    return true;
}

/* ===================
 * FIFO Configuration
 * =================== */

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool Iim42653::configure_fifo(const Iim42653Config &cfg)
{
    if (!cfg.enable_fifo)
    {
        fifo_enabled_ = false;
        IMU_TRY(write_reg(FIFO_CONFIG, FIFO_BYPASS)); /* bypass mode */
        return true;
    }

    /* FIFO w trybie bypass podczas konfiguracji */
    IMU_TRY(write_reg(FIFO_CONFIG, FIFO_BYPASS));

    /*
     * TMST_CONFIG = 0x21: TMST_EN=1, FSYNC_EN=0, rozdzielczosc 1 mikrosekunda.
     * Nadpisuje wartosc po resecie 0x23 (czysci  bit TMST_FSYNC_EN, bit 1).
     */
    IMU_TRY(write_reg(TMST_CONFIG, TMST_EN_1US));

    /*
     * FIFO_CONFIG1 = 0x2F:
     *   bit 5 = FIFO_WM_GT_TH
     *   bit 3 = FIFO_TMST_FSYNC_EN  (wymagane do ODR timestamps)
     *   bit 2 = FIFO_TEMP_EN
     *   bit 1 = FIFO_GYRO_EN
     *   bit 0 = FIFO_ACCEL_EN
     * Generuje Packet 3 (16 B): Header + Accel + Gyro + Temp8 + Timestamp16.
     */
    IMU_TRY(write_reg(FIFO_CONFIG1, FIFO1_PKT3_ALL));

    /* Watermark w bajtach. Musimy ograniczyc do maksymalnej wartosci sprzetowej (2080 B / 16 B =
     * 130 pakietow). Datasheet w 6.3 mowi: physical FIFO = 2048 B + read cache, driver max = 2080
     * B. FIFO_WM nie moze byc rowne 0 (patrz datasheet w 14.48/14.49). */
    uint16_t wm_packets = (cfg.fifo_watermark > 0) ? cfg.fifo_watermark : 1;
    if (wm_packets > kFifoMaxPackets)
    {
        wm_packets = static_cast<uint16_t>(kFifoMaxPackets);
    }
    const uint16_t wm_bytes = wm_packets * static_cast<uint16_t>(kFifoPacketSize);

    IMU_TRY(write_reg(FIFO_CONFIG2, static_cast<uint8_t>(wm_bytes & 0xFF)));
    IMU_TRY(write_reg(FIFO_CONFIG3, static_cast<uint8_t>((wm_bytes >> 8) & 0x0F)));

    /* sflushuj FIFO (SIGNAL_PATH_RESET bit 1). */
    IMU_TRY(modify_reg(SIGNAL_PATH_RESET, 0x00, FIFO_FLUSH));

    chThdSleepMicroseconds(100);

    /* Stream-to-FIFO mode: FIFO_CONFIG bits [7:6] = 01. */
    IMU_TRY(write_reg(FIFO_CONFIG, FIFO_STREAM));

    fifo_enabled_ = true;
    return true;
}

/* ===============
 * Odczyt danych
 * =============== */

bool Iim42653::read(ImuSample &sample)
{
    if (!initialized_)
    {
        return false;
    }

    /*
     * Pobieranie timestampa na poczatku by jak najlepiej okreslic moment pomiaru.
     */
    sample.timestamp_us = timestamp_us();

    /*
     * Burstowo odczytujemy 14 bajtow: TEMP_DATA1 (0x1D) do GYRO_DATA_Z0 (0x2A).
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

    /* Parsujemy big-endian 16-bitowe signed inty */
    const int16_t raw_temp     = parse_be16(&buf[0]);
    const int16_t raw_accel[3] = {parse_be16(&buf[2]), parse_be16(&buf[4]), parse_be16(&buf[6])};
    const int16_t raw_gyro[3]  = {parse_be16(&buf[8]), parse_be16(&buf[10]), parse_be16(&buf[12])};

    /* Odrzucamy probke jezeli ktorakolwiek os gyro i akcelerometru zwraca nieprawidlowa wartosc
     * (0x8000). */
    if (has_invalid(raw_accel) || has_invalid(raw_gyro))
    {
        return false;
    }

    /* Konwertujemy inertial channels do jednostek SI. */
    convert_inertial(raw_accel, raw_gyro, sample);

    /* Temperatura: T(deg_C) = raw / 132.48 + 25. NaN jezeli dane z czujnika nie sa gotowe. */
    sample.temp_degc = (raw_temp == kInvalidRaw)
                           ? std::numeric_limits<float>::quiet_NaN()
                           : static_cast<float>(raw_temp) * kTempScale + kTempOffset;

    return true;
}

/* ============================
 * Sprawdzanie gotowosci danych
 * ============================ */

bool Iim42653::data_ready()
{
    auto status = read_reg(INT_STATUS);
    if (!status.has_value())
    {
        report_error();
        return false;
    }

    /* DATA_RDY_INT to bit 3 INT_STATUS (0x2D). */
    return (status.value() & DATA_RDY_INT) != 0;
}

/* ================
 * Odczyt FIFO
 * ================ */

int16_t Iim42653::fifo_byte_count()
{
    /*
     * Burst-read FIFO_COUNTH (0x2E) i FIFO_COUNTL (0x2F).
     * Czytanie FIFO_COUNTL zatrzaskuje oba rejestry dla spojnej wartosci.
     * INTF_CONFIG0 bit 5 = 1 -> kolejnosc bajtow big-endian.
     */
    uint8_t buf[2];
    if (!read_regs(FIFO_COUNTH, buf, 2))
    {
        return -1;
    }

    return parse_be16(buf);
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

    /* sprawdz FIFO-flag czy pusta. */
    if ((header & kFifoHeaderEmpty) != 0)
    {
        return false;
    }

    /* Validate header: expect accel + gyro + ODR timestamp (Packet 3). */
    if ((header & kFifoHeaderMask) != kFifoHeaderExpected)
    {
        return false;
    }

    const int16_t raw_accel[3] = {parse_be16(&pkt[1]), parse_be16(&pkt[3]), parse_be16(&pkt[5])};
    const int16_t raw_gyro[3]  = {parse_be16(&pkt[7]), parse_be16(&pkt[9]), parse_be16(&pkt[11])};

    /* Odrzucamy probke jezeli ktorakolwiek os gyro i akcelerometru zwraca nieprawidlowa wartosc
     * (0x8000). */
    if (has_invalid(raw_accel) || has_invalid(raw_gyro))
    {
        return false;
    }

    /* skonwertuj do SI units (taki sam wzor jak dla register-read path). */
    convert_inertial(raw_accel, raw_gyro, sample);

    /* 8-bit FIFO temperature: T(deg_C) = raw / 2.07 + 25, range -40 to 85deg_C. */
    const auto raw_temp = static_cast<int8_t>(pkt[13]);
    if (raw_temp == kFifoTempInvalid)
    {
        sample.temp_degc = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
        sample.temp_degc = static_cast<float>(raw_temp) * kFifoTempScale + kFifoTempOffset;
    }

    /* timestamp czujnika zapisany tymczasowo w timestamp_us (raw 16-bit mikrosekund).
     * Caller (read_fifo) rekonstruuje absolutne znaczniki czasu hosta. */
    sample.timestamp_us = static_cast<uint32_t>(parse_be16(&pkt[14]) & 0xFFFF);

    return true;
}

// NOLINTNEXTLINE(readability-function-size)
size_t Iim42653::read_fifo(ImuSample *samples, size_t max_count)
{
    if (!initialized_ || !fifo_enabled_ || samples == nullptr || max_count == 0)
    {
        return 0;
    }

    /*
     * Pobierz czas hosta — zostanie przypisany do najnowszej próbki.
     */
    const uint32_t host_time = timestamp_us();

    /* Sprawdź przepełnienie FIFO (bit 1 rejestru INT_STATUS) */
    auto int_status = read_reg(INT_STATUS);
    if (int_status.has_value() && (int_status.value() & FIFO_OVERFLOW_INT) != 0)
    {
        error_report(ErrorCode::IMU_FIFO_OVERFLOW);
        (void)flush_fifo();
        return 0;
    }

    /* Odczytaj liczbę bajtów w FIFO i oblicz liczbę pełnych pakietów. */
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
     * Odczytaj wszystkie pakiety FIFO w jednej transakcji SPI.
     * Pozwala to uniknąć kosztu przełączania CS i konfiguracji DMA dla każdego pakietu.
     */
    const size_t total_bytes = n_packets * kFifoPacketSize;

    if (!read_regs(FIFO_DATA, fifo_bulk_buf_, total_bytes))
    {
        report_error();
        return 0;
    }

    /* Sparsuj pakiety. FIFO zwraca dane od najstarszego do najnowszego. */
    size_t valid = 0;
    for (size_t i = 0; i < n_packets; ++i)
    {
        if (parse_fifo_packet(&fifo_bulk_buf_[i * kFifoPacketSize], samples[valid]))
        {
            ++valid;
        }
    }

    if (valid == 0)
    {
        return 0;
    }

    /*
     * Rekonstruujemy absolutne znaczniki czasu hosta na podstawie roznic timestampow po stronie
     * czujnika.
     *
     * Kazdy sample.timestamp_us zawiera obecnie raw 16-bit czujnik
     * timestamp (mikrosekundy, przepelnienie przy 65536). Najnowsza probka (ostatnia poprawna)
     * otrzymuje czas hosta z licznika DWT. Wczesniejsze probki sa przesuwane wstecz o roznice
     * timestampow miedzy kolejnymi probkami czujnika.
     *
     * Odejmowanie bez znaku dla 16 bitow poprawnie obsluguje pojedyncze przepelnienie,
     * o ile odstep miedzy kolejnymi probkami jest mniejszy niz 65.5 ms (ODR >= ok. 16 Hz).
     *
     * Surowe roznice musza zostac przeskalowane przez 32/30 (patrz datasheet 12.7)
     * bez zewnetrzenego CLKIN oraz przy TMST_RES=0 wewnetrzny licznik 1 mikrosekundy w czujniku
     * dziala z predkoscia 30/32 rzeczywistego czasu.
     *
     * Surowe timestampy czujnika musza zostac zapisane przed ich nadpisanie czasem hosta,
     * poniewaz rekonstrukja wykorzystuje jednoczesnie oba zrodla zegara
     *
     */
    for (size_t i = 0; i < valid; ++i)
    {
        raw_sensor_ts_[i] = static_cast<uint16_t>(samples[i].timestamp_us & 0xFFFF);
    }

    samples[valid - 1].timestamp_us = host_time;

    for (size_t i = valid - 1; i > 0; --i)
    {
        const auto     raw_delta = static_cast<uint16_t>(raw_sensor_ts_[i] - raw_sensor_ts_[i - 1]);
        const uint32_t delta     = (static_cast<uint32_t>(raw_delta) * 32U + 15U) / 30U;

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

    IMU_TRY(modify_reg(SIGNAL_PATH_RESET, 0x00, FIFO_FLUSH));
    return true;
}

/* ============================
 * Sprzetowa kalibracja offsetow
 * ============================ */

bool Iim42653::set_offsets(const std::array<float, 3> &gyro_bias_dps,
                          const std::array<float, 3> &accel_bias_g)
{
    if (!initialized_)
    {
        return false;
    }

    /* Sprawdz, czy czujniki sa WYLACZONE - offsety musza byc ustawione przed ich wlaczeniem
    przez rejestr PWR_MGMT0. */
    auto pwr = read_reg(PWR_MGMT0);
    if (!pwr.has_value())
    {
        report_error();
        return false;
    }
    if ((pwr.value() & PWR_SENSOR_MASK) != 0)
    {
        chDbgAssert(false, "set_offsets: sensors must be OFF");
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
        const float clamped = std::clamp(dps, -64.0f, 64.0f);
        const auto  raw     = static_cast<int16_t>(std::lround(clamped * 32.0f));
        /* 12-bit signed range: −2048 to +2047. Clamp to avoid sign flip at +64 dps. */
        return std::clamp(raw, static_cast<int16_t>(-2048), static_cast<int16_t>(2047));
    };

    auto to_accel_lsb = [](float g) -> int16_t {
        const float clamped = std::clamp(g, -1.0f, 1.0f);
        return static_cast<int16_t>(std::lround(clamped * 2000.0f));
    };

    const int16_t gx = to_gyro_lsb(gyro_bias_dps[0]);
    const int16_t gy = to_gyro_lsb(gyro_bias_dps[1]);
    const int16_t gz = to_gyro_lsb(gyro_bias_dps[2]);
    const int16_t ax = to_accel_lsb(accel_bias_g[0]);
    const int16_t ay = to_accel_lsb(accel_bias_g[1]);
    const int16_t az = to_accel_lsb(accel_bias_g[2]);

    /* Rzutuj na unsigned przed manipulacja bitowa, aby uniknac implementacyjnie zaleznego
     * zachowania przy przesunieciu w prawo liczb ze znakiem. */
    const auto ugx = static_cast<uint16_t>(gx);
    const auto ugy = static_cast<uint16_t>(gy);
    const auto ugz = static_cast<uint16_t>(gz);
    const auto uax = static_cast<uint16_t>(ax);
    const auto uay = static_cast<uint16_t>(ay);
    const auto uaz = static_cast<uint16_t>(az);

    /* Spakuj do 9 bajtow rejestrow. Kazda wartosc 12-bitowa jest dzielona miedzy dwa rejestry. */
    const uint8_t regs[9] = {
        static_cast<uint8_t>(ugx & 0xFF),                                  /* USER0 */
        static_cast<uint8_t>(((ugy & 0x0F00) >> 4) | ((ugx >> 8) & 0x0F)), /* USER1 */
        static_cast<uint8_t>(ugy & 0xFF),                                  /* USER2 */
        static_cast<uint8_t>(ugz & 0xFF),                                  /* USER3 */
        static_cast<uint8_t>(((uax & 0x0F00) >> 4) | ((ugz >> 8) & 0x0F)), /* USER4 */
        static_cast<uint8_t>(uax & 0xFF),                                  /* USER5 */
        static_cast<uint8_t>(uay & 0xFF),                                  /* USER6 */
        static_cast<uint8_t>(((uaz & 0x0F00) >> 4) | ((uay >> 8) & 0x0F)), /* USER7 */
        static_cast<uint8_t>(uaz & 0xFF),                                  /* USER8 */
    };

    /* Zapisz wszystkie 9 rejestrow offsetow w jednej sesji banku 4. */
    {
        const BankScope bank(*this, 4);
        if (!bank)
        {
            report_error();
            return false;
        }

        for (uint8_t i = 0; i < 9; ++i)
        {
            IMU_TRY(write_reg(OFFSET_USER0 + i, regs[i]));
        }
    }

    return true;
}

/* ========================================
 * Odczytaj surowe wartosci (for self-test)
 * ========================================*/

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

/* ==========
 * Self-Test
 * ========== */

// NOLINTNEXTLINE(readability-function-size)
bool Iim42653::self_test()
{
    if (!initialized_)
    {
        return false;
    }

    static constexpr int kNumSamples = 200;

    /* Zachowaj obecna konfiguracja aby przywrocic ja pozniej. */
    const Iim42653Config saved_config = config_;

    /* Helper: zawsze przywroc oryginalna konfiguracje przed wyjsciem. */
    auto restore = [&]() { (void)configure(saved_config); };

    /* Skonfiguruj zakresy do self-testu i poczekaj na stabilizacje */
    if (!setup_self_test_mode())
    {
        restore();
        return false;
    }

    /* Zbierz probki przy wylaczonym self-tescie */
    int32_t accel_off[3] = {0, 0, 0};
    int32_t gyro_off[3]  = {0, 0, 0};

    if (!collect_st_samples(accel_off, gyro_off, kNumSamples))
    {
        restore();
        return false;
    }

    /* Wlacz self-test */
    if (!write_reg(SELF_TEST_CONFIG, ST_ENABLE_ALL))
    {
        restore();
        return false;
    }

    chThdSleepMilliseconds(200);

    /* Zbierz probki przy wlaczonym self-tescie. */
    int32_t accel_on[3] = {0, 0, 0};
    int32_t gyro_on[3]  = {0, 0, 0};

    if (!collect_st_samples(accel_on, gyro_on, kNumSamples))
    {
        (void)write_reg(SELF_TEST_CONFIG, ST_DISABLE);
        restore();
        return false;
    }

    /* Wylacz self-test */
    if (!write_reg(SELF_TEST_CONFIG, ST_DISABLE))
    {
        restore();
        return false;
    }

    chThdSleepMilliseconds(100);

    /* Oblicz odpowiedz self-testu. */
    float accel_response[3];
    float gyro_response[3];
    for (int ax = 0; ax < 3; ++ax)
    {
        accel_response[ax] =
            static_cast<float>(accel_on[ax] - accel_off[ax]) / static_cast<float>(kNumSamples);
        gyro_response[ax] =
            static_cast<float>(gyro_on[ax] - gyro_off[ax]) / static_cast<float>(kNumSamples);
    }

    /* Odczytaj referencje fabryczne i zweryfikuj wyniki */
    float gyro_st_ref[3];
    float accel_st_ref[3];
    if (!read_st_factory_refs(gyro_st_ref, accel_st_ref))
    {
        restore();
        return false;
    }

    const bool pass = validate_st_results(gyro_response, accel_response, gyro_st_ref, accel_st_ref);

    if (!pass)
    {
        error_report(ErrorCode::IMU_SELF_TEST_FAIL);
    }

    restore();
    return pass;
}

bool Iim42653::setup_self_test_mode()
{
    static constexpr int kSettleMs = 100;

    if (!write_reg(PWR_MGMT0, PWR_OFF))
    {
        return false;
    }

    chThdSleepMicroseconds(300);

    /* Gyro: ±250 dps, 1 kHz. Accel: ±4 g, 1 kHz (patrz datasheet). */
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

    if (!write_reg(PWR_MGMT0, PWR_GYRO_ACCEL_LN))
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
        /* Czekaj na swieza probke (DRDY), aby uniknac odczytu starych danych */
        static constexpr int kDrdyTimeoutIter = 20;
        bool                 ready            = false;
        for (int w = 0; w < kDrdyTimeoutIter; ++w)
        {
            if (data_ready())
            {
                ready = true;
                break;
            }
            chThdSleepMicroseconds(100);
        }
        if (!ready)
        {
            return false;
        }

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
    }

    return true;
}

bool Iim42653::read_st_factory_refs(float gyro_ref[3], float accel_ref[3])
{
    static constexpr uint8_t gyro_st_regs[3]  = {XG_ST_DATA, YG_ST_DATA, ZG_ST_DATA};
    static constexpr uint8_t accel_st_regs[3] = {XA_ST_DATA, YA_ST_DATA, ZA_ST_DATA};

    uint8_t gyro_codes[3];
    uint8_t accel_codes[3];

    /* Odczytaj referencje self-testu gyro - jedna sesja banku 1 */
    {
        const BankScope bank(*this, 1);
        if (!bank)
        {
            return false;
        }

        for (int ax = 0; ax < 3; ++ax)
        {
            auto val = read_reg(gyro_st_regs[ax]);
            if (!val.has_value())
            {
                return false;
            }
            gyro_codes[ax] = val.value();
        }
    }

    /* Odczytaj referencje self-testu akcelerometru - jedna sesja banku 2 */
    {
        const BankScope bank(*this, 2);
        if (!bank)
        {
            return false;
        }

        for (int ax = 0; ax < 3; ++ax)
        {
            auto val = read_reg(accel_st_regs[ax]);
            if (!val.has_value())
            {
                return false;
            }
            accel_codes[ax] = val.value();
        }
    }

    /* Compute OTP references. ST_OTP = 2620 * (1.01 ^ (code - 1)), 0 if unprogrammed. */
    for (int ax = 0; ax < 3; ++ax)
    {
        gyro_ref[ax] = (gyro_codes[ax] != 0)
                           ? 2620.0f * std::pow(1.01f, static_cast<float>(gyro_codes[ax]) - 1.0f)
                           : 0.0f;

        accel_ref[ax] = (accel_codes[ax] != 0)
                            ? 2620.0f * std::pow(1.01f, static_cast<float>(accel_codes[ax]) - 1.0f)
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

#undef IMU_TRY

// NOLINTEND(cppcoreguidelines-avoid-do-while)
