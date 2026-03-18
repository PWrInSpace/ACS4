/*
 * ACS4 Flight Computer — MS5611 Barometric Pressure Sensor Driver
 *
 * MEAS Switzerland MS5611-01BA03, 24-bit delta-sigma ADC barometer.
 * Communicates over SPI (Mode 0 or 3, max 20 MHz).
 *
 * Features:
 *   - Non-blocking state machine (no chThdSleep inside — driven by external tick)
 *   - Factory PROM calibration read with CRC-4 verification (AN520)
 *   - Full 2nd-order temperature compensation per datasheet
 *   - Configurable oversampling ratio (256–4096)
 *   - Output in SI units: Pa, °C
 *   - Barometric altitude helper
 *
 * Hardware mapping (production PCB):
 *   SPI2: PB13 (SCK) / PB14 (MISO) / PB15 (MOSI)
 *   CS:   PD11 (LINE_BARO_CS)
 *
 * Typical usage:
 *   acs::Ms5611 baro;
 *   baro.init(spi_bus, LINE_BARO_CS, spi_config);
 *   // In NavThread (~100 Hz tick):
 *   baro.update();
 *   if (baro.has_new_data()) {
 *       acs::BaroSample s = baro.sample();
 *   }
 */

#pragma once

#include <cstdint>

#include "drivers/ms5611_math.h"
#include "hal/spi_bus.h"

namespace acs
{

/* ========================================================
 * BaroSample — Skompensowany output z barometru
 * ======================================================== */

struct BaroSample
{
    float    pressure_pa;   /* Skompensowane cisnienie, Pa */
    float    temperature_c; /* Skompensowana temeperatura, degC */
    float    altitude_m;    /* Wysokosc barometryczna (ISA lub QNH), m */
    uint32_t timestamp_us;  /* host µs (DWT) na koncu konwersji */
};

/* ==================
 * Konfiguracja
 * =================== */

/**
 * Oversampling ratio. Wyzsze OSR = lepsza rozdzielczosc, dluzsza konwersja.
 * Value encodes the 2-bit offset added to CONVERT_D1/D2 commands.
 */
enum class Ms5611Osr : uint8_t
{
    OSR_256  = 0x00, /* 0.54 ms typ, 0.065 mbar RMS */
    OSR_512  = 0x02, /* 1.06 ms typ, 0.042 mbar RMS */
    OSR_1024 = 0x04, /* 2.08 ms typ, 0.027 mbar RMS */
    OSR_2048 = 0x06, /* 4.13 ms typ, 0.018 mbar RMS */
    OSR_4096 = 0x08, /* 8.22 ms typ, 0.012 mbar RMS */
};

struct Ms5611Config
{
    Ms5611Osr osr;    /* oversampling ratio zarowno dla P i T */
    float     qnh_pa; /* QNH cisnienie odniesienia dla wysokosci, Pa */

    /**
     * @brief Defaultowa konfiguracja dla lotu rakiety:
     *   - OSR 4096 (najlepsza rozdzielczosc, 10 cm wysokosci)
     *   - QNH = 101325 Pa (ISA standard)
     */
    static constexpr Ms5611Config rocket_default()
    {
        return {
            .osr    = Ms5611Osr::OSR_4096,
            .qnh_pa = 101325.0f,
        };
    }
};

/* =====================
 * MS5611 Driver Class
 * ===================== */

class Ms5611
{
  public:
    Ms5611()                          = default;
    ~Ms5611()                         = default;
    Ms5611(const Ms5611 &)            = delete;
    Ms5611 &operator=(const Ms5611 &) = delete;
    Ms5611(Ms5611 &&)                 = delete;
    Ms5611 &operator=(Ms5611 &&)      = delete;

    /**
     * @brief Zanicjalizuj barometr (blocking).
     *
     * Wykonuje:
     *   1. Komenda resetu + 3 ms czekania
     *   2. PROM odczyt (8 × 16-bit words, coefficients C1–C6)
     *   3. CRC-4 validation of PROM contents
     *
     * @param spi       Reference to initialized SpiBus.
     * @param cs_line   PAL line for chip select.
     * @param spi_cfg   SPI configuration (CPOL/CPHA/prescaler).
     * @param cfg       Driver configuration (OSR, QNH).
     * @return true on success, false on comm failure or CRC mismatch.
     */
    [[nodiscard]] bool
    init(SpiBus &spi, ioline_t cs_line, const SPIConfig &spi_cfg, const Ms5611Config &cfg);

    /**
     * @brief Non-blocking state machine tick.
     *
     * Call at a regular rate (≥ 100 Hz). The state machine issues
     * conversion commands and reads results without blocking.
     * A full pressure+temperature cycle takes ~20 ms at OSR 4096.
     *
     * When a complete sample is ready, has_new_data() becomes true.
     */
    void update();

    /**
     * @brief Check if a new compensated sample is available.
     *
     * Cleared after calling sample(). Thread-safe.
     */
    [[nodiscard]] bool has_new_data() const;

    /**
     * @brief Get the latest compensated sample and clear the flag.
     *
     * Thread-safe: uses a critical section to avoid tearing of the
     * BaroSample struct when update() runs on a different thread.
     */
    [[nodiscard]] BaroSample sample();

    /**
     * @brief Update QNH reference pressure at runtime.
     *
     * @param qnh_pa  New QNH pressure in Pa.
     */
    void set_qnh(float qnh_pa)
    {
        qnh_pa_ = qnh_pa;
    }

    /**
     * @brief Check if the driver has been successfully initialized.
     */
    [[nodiscard]] bool is_initialized() const
    {
        return initialized_;
    }

    /**
     * @brief Get cumulative communication error count.
     */
    [[nodiscard]] uint32_t error_count() const
    {
        return error_count_;
    }

  private:
    /* State machine */

    enum class State : uint8_t
    {
        IDLE,       /* not yet initialized */
        CONVERT_D1, /* send pressure conversion command */
        WAIT_D1,    /* wait for pressure ADC */
        READ_D1,    /* read pressure result */
        CONVERT_D2, /* send temperature conversion command */
        WAIT_D2,    /* wait for temperature ADC */
        READ_D2,    /* read temperature result + compute */
    };

    /* Computation helpers */

    void compute_and_publish();

    /* SPI helpers  */

    [[nodiscard]] bool send_command(uint8_t cmd);
    [[nodiscard]] bool read_adc(uint32_t &result);
    [[nodiscard]] bool read_prom(uint16_t prom[8]);

    /* Error handling */

    void report_error();

    /* Conversion time lookup */

    static uint32_t conversion_time_us(Ms5611Osr osr);

    /* State */

    SpiBus          *spi_     = nullptr;
    ioline_t         cs_line_ = 0;
    const SPIConfig *spi_cfg_ = nullptr;

    State state_       = State::IDLE;
    bool  initialized_ = false;
    bool  new_data_    = false;
    float qnh_pa_      = 101325.0f;

    Ms5611Osr osr_          = Ms5611Osr::OSR_4096;
    uint32_t  conv_time_us_ = 0; /* cached conversion_time_us(osr_) */

    uint16_t cal_[6] = {}; /* C1–C6 calibration coefficients (PROM addresses 1–6) */

    uint32_t raw_d1_ = 0; /* raw pressure ADC */
    uint32_t raw_d2_ = 0; /* raw temperature ADC */

    uint32_t conv_start_us_ = 0; /* timestamp when conversion was started */
    uint32_t error_count_   = 0;

    BaroSample last_sample_ = {};
};

/**
 * @brief Get pointer to the global barometer driver instance.
 * @return Pointer to initialized Ms5611, or nullptr if unavailable.
 */
Ms5611 *baro_instance();

}  // namespace acs
