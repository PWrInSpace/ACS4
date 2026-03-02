/*
 * ACS4 Flight Computer — I2C Bus Abstraction
 *
 * Wraps ChibiOS I2CDriver with:
 *   - DMA transfers (via HAL I2Cv3 LLD)
 *   - Configurable timeout (default 10ms)
 *   - Mutex-protected bus access (i2cAcquireBus/i2cReleaseBus)
 *   - Bus recovery: 9 SCL clock pulses when SDA is stuck low
 *   - Error reporting via acs::error_report()
 *
 * Hardware mapping (both boards):
 *   I2C1: PB6 (SCL) / PB7 (SDA)  — gold-pin header
 *   I2C2: PB10 (SCL) / PB11 (SDA) — gold-pin header
 *
 * Configuration:
 *   halconf.h  → HAL_USE_I2C = TRUE
 *   mcuconf.h  → STM32_I2C_USE_I2C1 = TRUE, STM32_I2C_USE_I2C2 = TRUE
 *
 * Usage:
 *   acs::I2cBus i2c;
 *   i2c.init(&I2CD1, acs::I2cBus::default_config());
 *   uint8_t reg = 0x75;
 *   uint8_t buf[6];
 *   i2c.write_read(0x30, &reg, 1, buf, sizeof(buf));
 */

#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {
#include "hal.h"
}

namespace acs
{

class I2cBus
{
  public:
    I2cBus()                          = default;
    ~I2cBus()                         = default;
    I2cBus(const I2cBus &)            = delete;
    I2cBus &operator=(const I2cBus &) = delete;
    I2cBus(I2cBus &&)                 = delete;
    I2cBus &operator=(I2cBus &&)      = delete;

    /**
     * @brief Default I2C configuration: 400 kHz Fast Mode.
     *
     * TIMINGR value for STM32H7 @ PCLK1 = 137.5 MHz:
     *   PRESC=0x3, SCLDEL=0x4, SDADEL=0x2, SCLH=0x0F, SCLL=0x13
     *   → ~400 kHz (datasheet RM0468 §50.4.10)
     */
    static constexpr I2CConfig default_config()
    {
        return I2CConfig{
            .timingr = 0x30420F13U, /* 400 kHz Fast Mode @ 137.5 MHz PCLK1 */
            .cr1     = 0U,
            .cr2     = 0U,
        };
    }

    /**
     * @brief Initialize the I2C bus.
     *
     * @param driver   Pointer to ChibiOS I2CDriver (e.g. &I2CD1, &I2CD2).
     * @param config   I2C timing configuration.
     * @param scl_line PAL line for SCL (needed for bus recovery).
     * @param sda_line PAL line for SDA (needed for bus recovery).
     * @return true on success, false if driver is null.
     */
    [[nodiscard]] bool init(I2CDriver       *driver,
                            const I2CConfig &config,
                            ioline_t         scl_line,
                            ioline_t         sda_line);

    /**
     * @brief Write then read from an I2C device (combined transaction).
     *
     * Acquires bus mutex, performs transfer with timeout, releases.
     * On failure, increments error counter and attempts bus recovery.
     *
     * @param addr    7-bit I2C slave address.
     * @param tx      Transmit buffer (register address, etc.).
     * @param tx_len  Number of bytes to transmit.
     * @param rx      Receive buffer.
     * @param rx_len  Number of bytes to receive.
     * @return true on success, false on timeout/NACK/bus error.
     */
    [[nodiscard]] bool write_read(uint8_t        addr,
                                  const uint8_t *tx,
                                  size_t         tx_len,
                                  uint8_t       *rx,
                                  size_t         rx_len);

    /**
     * @brief Write-only transaction.
     *
     * @param addr    7-bit I2C slave address.
     * @param tx      Transmit buffer.
     * @param tx_len  Number of bytes to transmit.
     * @return true on success.
     */
    [[nodiscard]] bool write(uint8_t addr, const uint8_t *tx, size_t tx_len);

    /**
     * @brief Read-only transaction (no register address sent first).
     *
     * @param addr    7-bit I2C slave address.
     * @param rx      Receive buffer.
     * @param rx_len  Number of bytes to read.
     * @return true on success.
     */
    [[nodiscard]] bool read(uint8_t addr, uint8_t *rx, size_t rx_len);

    /**
     * @brief Perform I2C bus recovery (9 SCL clock pulses).
     *
     * Used when SDA is stuck low (slave holding the bus).
     * Temporarily reconfigures SCL/SDA as GPIO, generates 9 clocks
     * on SCL, then restores AF mode and restarts the driver.
     *
     * @return true if SDA was released, false if still stuck.
     */
    [[nodiscard]] bool bus_recovery();

    /**
     * @brief Check if a device responds at the given address.
     *
     * Sends a zero-length write and checks for ACK.
     *
     * @param addr 7-bit I2C slave address.
     * @return true if device ACK'd.
     */
    [[nodiscard]] bool probe(uint8_t addr);

    /**
     * @brief Get the number of failed transactions since init.
     */
    [[nodiscard]] uint32_t error_count() const
    {
        return error_count_;
    }

    /**
     * @brief Get the number of bus recoveries performed since init.
     */
    [[nodiscard]] uint32_t recovery_count() const
    {
        return recovery_count_;
    }

  private:
    static constexpr sysinterval_t kTimeout = TIME_MS2I(10); /* 10 ms */

    /** @brief RAII guard for i2cAcquireBus / i2cReleaseBus. */
    class BusGuard
    {
      public:
        explicit BusGuard(I2CDriver *d) : d_(d)
        {
            i2cAcquireBus(d_);
        }

        ~BusGuard()
        {
            i2cReleaseBus(d_);
        }

        BusGuard(const BusGuard &)            = delete;
        BusGuard &operator=(const BusGuard &) = delete;

      private:
        I2CDriver *d_;
    };

    I2CDriver *driver_         = nullptr;
    I2CConfig  config_         = {};
    ioline_t   scl_line_       = 0;
    ioline_t   sda_line_       = 0;
    uint32_t   error_count_    = 0;
    uint32_t   recovery_count_ = 0;
    bool       initialized_    = false;

    /**
     * @brief Handle a failed I2C transaction.
     *
     * Reads I2C error flags, reports via error_handler, and
     * attempts bus recovery if appropriate.
     */
    void handle_error();
};

}  // namespace acs
