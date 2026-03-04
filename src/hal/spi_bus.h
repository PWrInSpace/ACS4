/*
 * ACS4 Flight Computer — SPI Bus Abstraction
 *
 * Wraps ChibiOS SPIDriver (v2 API) with:
 *   - DMA transfers (via HAL SPIv3 LLD, circular mode off)
 *   - Blocking operations (DMA with synchronization)
 *   - Mutex-protected bus access (spiAcquireBus/spiReleaseBus)
 *   - Multiple chip-select support (software CS via PAL lines)
 *   - Error reporting via acs::error_report()
 *
 * Hardware mapping:
 *   CUSTOM_H725 (production PCB):
 *     SPI2: PB13 (SCK) / PB14 (MISO) / PB15 (MOSI)
 *       CS pins: PD8  (IMU  — IIM-42653)
 *                PD11 (BARO — MS5611)
 *                PB12 (MAG  — MMC5983MA)
 *     SPI kernel clock: PLL1_Q = 50 MHz
 *
 *   NUCLEO_H723 (dev board):
 *     SPI1: PA5 (SCK) / PA6 (MISO) / PA7 (MOSI) — CN7 header
 *       CS pins: user-defined on breadboard
 *     SPI kernel clock: PLL1_Q = 48 MHz
 *
 * Configuration:
 *   halconf.h  → HAL_USE_SPI = TRUE
 *                SPI_USE_SYNCHRONIZATION = TRUE
 *                SPI_SELECT_MODE = SPI_SELECT_MODE_NONE
 *   mcuconf.h  → STM32_SPI_USE_SPI2 = TRUE (prod) or SPI1 = TRUE (dev)
 *
 * SPIConfig (SPI_SELECT_MODE_NONE, no ssport/sspad fields):
 *   {
 *       .circular = false,
 *       .data_cb  = nullptr,
 *       .error_cb = nullptr,
 *       .cfg1     = SPI_CFG1_MBR_DIV8,       // clock prescaler
 *       .cfg2     = SPI_CFG2_CPOL | SPI_CFG2_CPHA  // mode 3
 *   }
 *
 * Usage:
 *   acs::SpiBus spi;
 *   spi.init(&SPID2);
 *
 *   static const SPIConfig imu_cfg = { false, nullptr, nullptr,
 *       SPI_CFG1_MBR_DIV8, SPI_CFG2_CPOL | SPI_CFG2_CPHA };
 *
 *   auto who = spi.read_register(LINE_IMU_CS, 0x75, imu_cfg); //
 * std::optional<uint8_t> spi.write_register(LINE_IMU_CS, 0x11, 0x0F, imu_cfg);
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

extern "C" {
#include "hal.h"
}

namespace acs
{

/**
 * @brief SPI bus wrapper with DMA, multi-CS, mutex and timeout.
 *
 * The ChibiOS SPIConfig is passed per-transfer (not at init) because
 * different devices on the same bus may require different clock polarity,
 * phase, and prescaler settings.
 *
 * CS (chip select) is managed manually via PAL lines because we use
 * SPI_SELECT_MODE_NONE — the ChibiOS driver does not touch any CS pin.
 */
class SpiBus
{
  public:
    SpiBus()                          = default;
    ~SpiBus()                         = default;
    SpiBus(const SpiBus &)            = delete;
    SpiBus &operator=(const SpiBus &) = delete;
    SpiBus(SpiBus &&)                 = delete;
    SpiBus &operator=(SpiBus &&)      = delete;

    /**
     * @brief Initialize the SPI bus.
     *
     * @param driver  Pointer to ChibiOS SPIDriver (e.g. &SPID2).
     * @return true on success, false if driver is null.
     *
     * @note SPIConfig is NOT set here — it is applied per-transfer
     *       so that devices with different CPOL/CPHA/prescaler can
     *       share one physical bus.
     */
    [[nodiscard]] bool init(SPIDriver *driver);

    /**
     * @brief Full-duplex SPI exchange with a specific device.
     *
     * Sequence:
     *   1. spiAcquireBus  (mutex — blocks other threads)
     *   2. spiStart with the supplied config
     *   3. CS assert       (palClearLine)
     *   4. spiExchange     (DMA, blocking with timeout)
     *   5. CS deassert     (palSetLine)
     *   6. spiStop + spiReleaseBus
     *
     * @param cs_line  PAL line for chip select (active low).
     * @param tx       Transmit buffer (may be nullptr → sends 0xFF).
     * @param rx       Receive buffer (may be nullptr → discards RX).
     * @param len      Number of bytes to exchange.
     * @param config   SPI configuration for this device.
     * @return true on success, false on timeout / DMA error.
     */
    [[nodiscard]] bool
    transfer(ioline_t cs_line, const uint8_t *tx, uint8_t *rx, size_t len, const SPIConfig &config);

    /**
     * @brief TX-only transfer (discard received data).
     *
     * @param cs_line  PAL line for chip select (active low).
     * @param tx       Transmit buffer.
     * @param len      Number of bytes to send.
     * @param config   SPI configuration for this device.
     * @return true on success, false on timeout / DMA error.
     */
    [[nodiscard]] bool
    send(ioline_t cs_line, const uint8_t *tx, size_t len, const SPIConfig &config);

    /**
     * @brief RX-only transfer (send 0xFF bytes, capture response).
     *
     * @param cs_line  PAL line for chip select (active low).
     * @param rx       Receive buffer.
     * @param len      Number of bytes to receive.
     * @param config   SPI configuration for this device.
     * @return true on success, false on timeout / DMA error.
     */
    [[nodiscard]] bool receive(ioline_t cs_line, uint8_t *rx, size_t len, const SPIConfig &config);

    /**
     * @brief Read a single 8-bit register (SPI sensor convention).
     *
     * Sends (reg | 0x80) + one dummy byte, returns the received byte.
     *
     * @param cs_line  PAL line for chip select.
     * @param reg      Register address (bit 7 will be set for read).
     * @param config   SPI configuration.
     * @return Register value, or std::nullopt on failure.
     */
    [[nodiscard]] std::optional<uint8_t>
    read_register(ioline_t cs_line, uint8_t reg, const SPIConfig &config);

    /**
     * @brief Write a single 8-bit register.
     *
     * Sends reg (bit 7 clear) followed by value.
     *
     * @param cs_line  PAL line for chip select.
     * @param reg      Register address.
     * @param value    Value to write.
     * @param config   SPI configuration.
     * @return true on success.
     */
    [[nodiscard]] bool
    write_register(ioline_t cs_line, uint8_t reg, uint8_t value, const SPIConfig &config);

    /**
     * @brief Read a burst of registers starting at `reg`.
     *
     * Sends (reg | 0x80) then clocks out `len` bytes into `buf`.
     *
     * @param cs_line  PAL line for chip select.
     * @param reg      Starting register address (bit 7 set automatically).
     * @param buf      Destination buffer.
     * @param len      Number of data bytes to read (max 32).
     * @param config   SPI configuration.
     * @return true on success.
     */
    [[nodiscard]] bool read_registers(ioline_t         cs_line,
                                      uint8_t          reg,
                                      uint8_t         *buf,
                                      size_t           len,
                                      const SPIConfig &config);

    /**
     * @brief Get the number of failed transfers since init.
     */
    [[nodiscard]] uint32_t error_count() const
    {
        return error_count_;
    }

  private:
    /** @brief RAII guard for spiAcquireBus+spiStart / spiStop+spiReleaseBus. */
    class BusGuard
    {
      public:
        BusGuard(SPIDriver *d, const SPIConfig *cfg) : d_(d)
        {
            spiAcquireBus(d_);
            spiStart(d_, cfg);
        }

        ~BusGuard()
        {
            spiStop(d_);
            spiReleaseBus(d_);
        }

        BusGuard(const BusGuard &)            = delete;
        BusGuard &operator=(const BusGuard &) = delete;

      private:
        SPIDriver *d_;
    };

    /** @brief RAII guard for CS assert (active-low) / deassert. */
    class CsGuard
    {
      public:
        explicit CsGuard(ioline_t cs) : cs_(cs)
        {
            palClearLine(cs_);
        }

        ~CsGuard()
        {
            palSetLine(cs_);
        }

        CsGuard(const CsGuard &)            = delete;
        CsGuard &operator=(const CsGuard &) = delete;

      private:
        ioline_t cs_;
    };

    SPIDriver *driver_      = nullptr;
    uint32_t   error_count_ = 0;
    bool       initialized_ = false;

    /**
     * @brief Report a SPI transfer error.
     *
     * Increments counter and calls acs::error_report().
     *
     * @param timeout  true if the error was a timeout, false for DMA.
     */
    void handle_error(bool timeout);
};

}  // namespace acs
