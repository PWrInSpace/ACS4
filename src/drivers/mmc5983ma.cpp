/*
 * ACS4 Flight Computer — MMC5983MA Magnetometer Driver
 *
 * MEMSIC MMC5983MA 3-axis AMR magnetic sensor.
 *
 * Sekwencja inicjalizacji:
 *   - Soft reset (SW_RST w CTRL1), 15 ms oczekiwania na ponowne
 *     zaladowanie OTP i przywrocenie rejestrow
 *   - Weryfikacja Product ID (0x30 pod adresem 0x2F)
 *   - Poczatkowa operacja SET dla ustalonej polaryzacji czujnika
 *
 * Tryb ciagly (continuous mode):
 *   - Bandwidth decymacji w CTRL1 (BW[1:0])
 *   - Auto_SR_en w CTRL0 — automatyczny SET/RESET przed kazdym pomiarem,
 *     eliminuje offset mostka Wheatstone'a niezaleznie od temperatury
 *   - Cmm_en + CM_Freq w CTRL2 — ciagly pomiar z zadana czestotliwoscia
 *   - En_prd_set + Prd_set w CTRL2 — periodyczny SET co N pomiarow
 *
 * Odczyt 18-bitowy:
 *   7 bajtow burstowo: Xout0(0x00)..Zout1(0x05) + XYZout2(0x06)
 *   raw_18bit = (Out0 << 10) | (Out1 << 2) | (XYZout2 >> shift)
 *   field_uT = (raw - 131072) * 100.0 / 16384.0
 *
 * Cala komunikacja z magistrala idzie przez abstrakcje SpiBus
 */

#include "drivers/mmc5983ma.h"

#include "system/error_handler.h"
#include "utils/timestamp.h"

extern "C" {
#include "ch.h"
}

namespace acs
{

using namespace mmc5983ma_reg;

// NOLINTBEGIN(cppcoreguidelines-avoid-do-while)

/** Write-or-fail: wywoluje report_error() i zwraca false w przypadku bledu SPI. */
#define MAG_TRY(expr)       \
    do                      \
    {                       \
        if (!(expr))        \
        {                   \
            report_error(); \
            return false;   \
        }                   \
    }                       \
    while (0)

/* ========================
 * Helpery do rejestrow
 * ======================== */

bool Mmc5983ma::write_reg(uint8_t reg, uint8_t value)
{
    return spi_->write_register(cs_line_, reg, value, *spi_cfg_);
}

std::optional<uint8_t> Mmc5983ma::read_reg(uint8_t reg)
{
    return spi_->read_register(cs_line_, reg, *spi_cfg_);
}

bool Mmc5983ma::read_regs(uint8_t start_reg, uint8_t *buf, size_t len)
{
    return spi_->read_registers(cs_line_, start_reg, buf, len, *spi_cfg_);
}

/* ====================
 * Handlowanie errorow
 * ==================== */

void Mmc5983ma::report_error()
{
    ++error_count_;
    error_report(ErrorCode::MAG_COMM_FAIL);
}

/* ====================================
 * CTRL0 — persistent mode bits helper
 * ==================================== */

uint8_t Mmc5983ma::ctrl0_base() const
{
    /*
     * CTRL0 jest write-only. Bity komend (SET, RESET, TM_M, TM_T)
     * sa self-clearing, ale bity trybu (Auto_SR_en, INT_meas_done_en)
     * musza byc jawnie ustawiane przy kazdym zapisie do CTRL0.
     */
    uint8_t val = 0;
    if (config_.auto_set_reset)
    {
        val |= AUTO_SR_EN;
    }
    if (config_.enable_int)
    {
        val |= INT_MEAS_DONE_EN;
    }
    return val;
}

/* =======================
 * Inicjalizacja czujnika
 * ======================= */

bool Mmc5983ma::init(SpiBus &spi, ioline_t cs_line, const SPIConfig &spi_cfg)
{
    spi_     = &spi;
    cs_line_ = cs_line;
    spi_cfg_ = &spi_cfg;

    /*
     * Krok 1: Software reset.
     * Zapisac SW_RST (bit 7) do Internal Control 1 (0x0A).
     * Reset powoduje ponowne odczytanie OTP i przywrocenie
     * wszystkich rejestrow do wartosci domyslnych.
     * Datasheet: power-on time = 10 ms. Dodajemy zapas.
     */
    MAG_TRY(write_reg(CTRL1, SW_RST));
    chThdSleepMilliseconds(15);

    /*
     * Krok 2: Zweryfikowac Product ID.
     * Oczekiwana wartosc: 0x30 pod adresem 0x2F.
     */
    auto id = read_reg(PRODUCT_ID);
    if (!id.has_value() || id.value() != PRODUCT_ID_VALUE)
    {
        report_error();
        return false;
    }

    /*
     * Krok 3: Poczatkowa operacja SET.
     * Ustala znana polaryzacje warstwy magnetorezystancyjnej
     * po resecie. Bit DO_SET jest self-clearing po impulsie 500 ns.
     */
    MAG_TRY(write_reg(CTRL0, DO_SET));
    chThdSleepMicroseconds(1);

    initialized_ = true;
    return true;
}

/* ===============
 * Konfiguracja
 * =============== */

bool Mmc5983ma::configure(const Mmc5983maConfig &cfg)
{
    if (!initialized_)
    {
        return false;
    }

    /*
     * Walidacja kompatybilnosci BW z CM_Freq (datasheet wymaga):
     *   CM_Freq=200Hz (110) → BW >= 01 (HZ_200)
     *   CM_Freq=1000Hz (111) → BW = 11 (HZ_800)
     */
    if (cfg.cm_freq == MagCmFreq::HZ_200 && cfg.bandwidth < MagBandwidth::HZ_200)
    {
        return false;
    }
    if (cfg.cm_freq == MagCmFreq::HZ_1000 && cfg.bandwidth != MagBandwidth::HZ_800)
    {
        return false;
    }

    config_ = cfg;

    /*
     * CTRL1 (0x0A): BW[1:0] — dlugosc filtra decymacyjnego.
     * Kontroluje czas pomiaru i maksymalna czestotliwosc wyjsciowa.
     * Pozostale bity (X/YZ inhibit = 0, SW_RST = 0) zostawiamy zerowe.
     */
    MAG_TRY(write_reg(CTRL1, static_cast<uint8_t>(cfg.bandwidth)));

    /*
     * CTRL0 (0x09): Ustawienie bitow trybu (Auto_SR_en, INT).
     * Auto_SR_en powoduje automatyczny SET/RESET przed kazdym
     * pomiarem, eliminujac offset mostka niezaleznie od temperatury.
     */
    MAG_TRY(write_reg(CTRL0, ctrl0_base()));

    /*
     * CTRL2 (0x0B): Continuous mode + periodic SET.
     *
     * Layout rejestru:
     *   [7]   En_prd_set   — wlacz periodyczny SET
     *   [6:4] Prd_set[2:0] — interwal periodycznego SET
     *   [3]   Cmm_en       — wlacz tryb ciagly
     *   [2:0] CM_Freq[2:0] — czestotliwosc pomiarow
     *
     * Periodyczny SET wymaga jednoczesnego ustawienia
     * Auto_SR_en (CTRL0) i Cmm_en (CTRL2).
     */
    auto ctrl2 = static_cast<uint8_t>(cfg.cm_freq);

    if (cfg.cm_freq != MagCmFreq::OFF)
    {
        ctrl2 |= CMM_EN;
    }

    if (cfg.periodic_set && cfg.auto_set_reset)
    {
        ctrl2 |= EN_PRD_SET;
        ctrl2 |= static_cast<uint8_t>(static_cast<uint8_t>(cfg.periodic_set_interval) << 4);
    }

    MAG_TRY(write_reg(CTRL2, ctrl2));

    return true;
}

/* ===============
 * Odczyt danych
 * =============== */

bool Mmc5983ma::read(MagSample &sample)
{
    if (!initialized_)
    {
        return false;
    }

    /*
     * Sprawdz Meas_M_Done przed odczytem. W trybie ciaglym flaga
     * jest ustawiana po zakonczeniu pomiaru i kasowana automatycznie
     * przy starcie nastepnego. Zwracamy false jesli dane nie sa gotowe,
     * zeby uniknac odczytu zduplikowanej/nieaktualnej probki.
     */
    if (!data_ready())
    {
        return false;
    }

    sample.timestamp_us = timestamp_us();

    /*
     * Burstowo odczytujemy 7 bajtow: Xout0 (0x00) do XYZout2 (0x06).
     *
     * Layout:
     *   [0] Xout0   — Xout[17:10]
     *   [1] Xout1   — Xout[9:2]
     *   [2] Yout0   — Yout[17:10]
     *   [3] Yout1   — Yout[9:2]
     *   [4] Zout0   — Zout[17:10]
     *   [5] Zout1   — Zout[9:2]
     *   [6] XYZout2 — X[1:0](b7:6) Y[1:0](b5:4) Z[1:0](b3:2)
     *
     * Skladanie 18-bitowej wartosci unsigned:
     *   raw = (Out0 << 10) | (Out1 << 2) | (XYZout2 bits)
     *
     * Format z offset binarnym — null field = 131072 (2^17).
     */
    uint8_t buf[kBurstLen];

    if (!read_regs(XOUT0, buf, kBurstLen))
    {
        report_error();
        return false;
    }

    const uint32_t raw_x = (static_cast<uint32_t>(buf[0]) << 10)
                           | (static_cast<uint32_t>(buf[1]) << 2)
                           | (static_cast<uint32_t>(buf[6]) >> 6);

    const uint32_t raw_y = (static_cast<uint32_t>(buf[2]) << 10)
                           | (static_cast<uint32_t>(buf[3]) << 2)
                           | ((static_cast<uint32_t>(buf[6]) >> 4) & 0x03);

    const uint32_t raw_z = (static_cast<uint32_t>(buf[4]) << 10)
                           | (static_cast<uint32_t>(buf[5]) << 2)
                           | ((static_cast<uint32_t>(buf[6]) >> 2) & 0x03);

    /*
     * Konwersja na uT:
     *   field_gauss = (raw - 131072) / 16384
     *   field_uT    = field_gauss * 100
     *
     * kScale = 100.0 / 16384.0 ≈ 0.006104
     */
    sample.field_ut[0] = (static_cast<float>(raw_x) - kNullField) * kScale;
    sample.field_ut[1] = (static_cast<float>(raw_y) - kNullField) * kScale;
    sample.field_ut[2] = (static_cast<float>(raw_z) - kNullField) * kScale;

    return true;
}

/* ============================
 * Odczyt temperatury czujnika
 * ============================ */

bool Mmc5983ma::read_temperature(float &temp_degc)
{
    if (!initialized_)
    {
        return false;
    }

    /*
     * Wyzwolenie pomiaru temperatury. Bit TM_T jest self-clearing.
     * Dolaczamy bity trybu (Auto_SR_en, INT) bo CTRL0 jest write-only.
     */
    MAG_TRY(write_reg(CTRL0, TM_T | ctrl0_base()));

    /* Czekaj na zakonczenie pomiaru (typowo < 8 ms przy BW=00). */
    static constexpr int kTempPollIter    = 20;
    static constexpr int kTempPollDelayUs = 500;

    bool done = false;
    for (int i = 0; i < kTempPollIter; ++i)
    {
        chThdSleepMicroseconds(kTempPollDelayUs);

        auto st = read_reg(STATUS);
        if (st.has_value() && (st.value() & MEAS_T_DONE) != 0)
        {
            done = true;
            break;
        }
    }

    if (!done)
    {
        report_error();
        return false;
    }

    auto raw = read_reg(TOUT);
    if (!raw.has_value())
    {
        report_error();
        return false;
    }

    /*
     * Konwersja temperatury (datasheet):
     *   T(deg_C) = raw * 0.8 - 75
     *   Zakres: -75 do +125 deg_C, rozdzielczosc ~0.8 deg_C/LSB
     *   Wartosc 0x00 odpowiada -75 deg_C
     */
    temp_degc = static_cast<float>(raw.value()) * kTempScale + kTempOffset;

    return true;
}

/* ==================================
 * SET/RESET degauss
 * ================================== */

bool Mmc5983ma::degauss()
{
    if (!initialized_)
    {
        return false;
    }

    /*
     * SET: impuls pradu przez cewke czujnika (500 ns).
     * Ustawia polaryzacje warstwy magnetorezystancyjnej.
     * Po SET: Output = +H + Offset
     */
    MAG_TRY(write_reg(CTRL0, DO_SET | ctrl0_base()));
    chThdSleepMicroseconds(1);

    /*
     * RESET: impuls pradu w przeciwnym kierunku (500 ns).
     * Po RESET: Output = -H + Offset
     *
     * Sekwencja SET+RESET eliminuje offset mostka:
     *   H = (Output_SET - Output_RESET) / 2
     *   Offset = (Output_SET + Output_RESET) / 2
     */
    MAG_TRY(write_reg(CTRL0, DO_RESET | ctrl0_base()));
    chThdSleepMicroseconds(1);

    return true;
}

/* ============================
 * Sprawdzanie gotowosci danych
 * ============================ */

bool Mmc5983ma::data_ready()
{
    auto status = read_reg(STATUS);
    if (!status.has_value())
    {
        report_error();
        return false;
    }

    return (status.value() & MEAS_M_DONE) != 0;
}

}  // namespace acs

#undef MAG_TRY

// NOLINTEND(cppcoreguidelines-avoid-do-while)
