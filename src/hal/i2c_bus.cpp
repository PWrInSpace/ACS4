/*
 * ACS4 Flight Computer — I2C Bus Abstraction (Implementation)
 *
 * ChibiOS I2Cv3 driver wrapper with DMA, timeout, and bus recovery.
 * See i2c_bus.h for API documentation.
 */

#include "hal/i2c_bus.h"

#include "system/error_handler.h"

extern "C" {
#include "ch.h"
}

namespace acs
{

/* ── Init ─────────────────────────────────────────────────────────────── */

bool I2cBus::init(I2CDriver       *driver,
                  const I2CConfig &config,
                  ioline_t         scl_line,
                  ioline_t         sda_line)
{
    if (driver == nullptr)
    {
        return false;
    }

    driver_   = driver;
    config_   = config;
    scl_line_ = scl_line;
    sda_line_ = sda_line;

    i2cStart(driver_, &config_);

    error_count_    = 0;
    recovery_count_ = 0;
    initialized_    = true;

    return true;
}

/* ── Write + Read (combined transaction) ──────────────────────────────── */

bool I2cBus::write_read(uint8_t        addr,
                        const uint8_t *tx,
                        size_t         tx_len,
                        uint8_t       *rx,
                        size_t         rx_len)
{
    if (!initialized_)
    {
        return false;
    }

    const BusGuard lock(driver_);
    const msg_t    status = i2cMasterTransmitTimeout(driver_,
                                                  addr,
                                                  tx,
                                                  tx_len,
                                                  rx,
                                                  rx_len,
                                                  kTimeout);

    if (status != MSG_OK)
    {
        handle_error();
        return false;
    }
    return true;
}

/* ── Write only ───────────────────────────────────────────────────────── */

bool I2cBus::write(uint8_t addr, const uint8_t *tx, size_t tx_len)
{
    if (!initialized_)
    {
        return false;
    }

    const BusGuard lock(driver_);
    const msg_t    status = i2cMasterTransmitTimeout(driver_,
                                                  addr,
                                                  tx,
                                                  tx_len,
                                                  nullptr,
                                                  0,
                                                  kTimeout);

    if (status != MSG_OK)
    {
        handle_error();
        return false;
    }
    return true;
}

/* ── Read only ────────────────────────────────────────────────────────── */

bool I2cBus::read(uint8_t addr, uint8_t *rx, size_t rx_len)
{
    if (!initialized_)
    {
        return false;
    }

    const BusGuard lock(driver_);
    const msg_t    status =
        i2cMasterReceiveTimeout(driver_, addr, rx, rx_len, kTimeout);

    if (status != MSG_OK)
    {
        handle_error();
        return false;
    }
    return true;
}

/* ── Probe (detect device at address) ─────────────────────────────────── */

bool I2cBus::probe(uint8_t addr)
{
    if (!initialized_)
    {
        return false;
    }

    /* Send a zero-byte write; device will ACK if present. */
    const uint8_t  dummy = 0;
    const BusGuard lock(driver_);
    const msg_t    status = i2cMasterTransmitTimeout(driver_,
                                                  addr,
                                                  &dummy,
                                                  0,
                                                  nullptr,
                                                  0,
                                                  kTimeout);

    if (status != MSG_OK)
    {
        /* Not a real error — just no device at this address.
         * Clear the error flags without counting it as a bus error. */
        i2cGetErrors(driver_);
        return false;
    }
    return true;
}

/* ── Bus Recovery (9-clock pulse method) ──────────────────────────────── */

bool I2cBus::bus_recovery()
{
    if (!initialized_)
    {
        return false;
    }

    /*
     * I2C bus recovery procedure (per NXP AN10216-01):
     *
     * 1. Stop the I2C peripheral (release pins to GPIO control).
     * 2. Configure SCL as push-pull output, SDA as input.
     * 3. Generate 9 clock pulses on SCL. If SDA goes high during
     *    any clock, the slave has released the bus — send STOP.
     * 4. Restart the I2C peripheral.
     */

    /* Step 1: stop driver to reconfigure pins as GPIO. */
    i2cStop(driver_);

    /* Step 2: SCL as open-drain output (high), SDA as input. */
    palSetLineMode(scl_line_, PAL_MODE_OUTPUT_OPENDRAIN);
    palSetLineMode(sda_line_, PAL_MODE_INPUT);

    /* Ensure SCL starts high. */
    palSetLine(scl_line_);
    chThdSleepMicroseconds(5);

    /* Step 3: generate 9 clock pulses. */
    bool sda_released = false;
    for (int i = 0; i < 9; i++)
    {
        palClearLine(scl_line_);
        chThdSleepMicroseconds(5);
        palSetLine(scl_line_);
        chThdSleepMicroseconds(5);

        /* Check if SDA was released (returned high). */
        if (palReadLine(sda_line_) == PAL_HIGH)
        {
            sda_released = true;
            break;
        }
    }

    /* Generate a STOP condition: SDA low -> SCL high -> SDA high. */
    if (sda_released)
    {
        palSetLineMode(sda_line_, PAL_MODE_OUTPUT_OPENDRAIN);
        palClearLine(sda_line_);
        chThdSleepMicroseconds(5);
        palSetLine(scl_line_);
        chThdSleepMicroseconds(5);
        palSetLine(sda_line_);
        chThdSleepMicroseconds(5);
    }

    /* Step 4: restore I2C alternate function and restart driver.
     * The pin AF mode depends on the I2C instance:
     *   I2C1: PB6/PB7  AF4
     *   I2C2: PB10/PB11 AF4
     *   (both are AF4 on STM32H7)
     */
    palSetLineMode(scl_line_,
                   PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetLineMode(sda_line_,
                   PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    i2cStart(driver_, &config_);

    recovery_count_++;

    return sda_released;
}

/* ── Error handling ───────────────────────────────────────────────────── */

void I2cBus::handle_error()
{
    /* Read and clear I2C error flags. */
    const i2cflags_t errors = i2cGetErrors(driver_);

    error_count_++;

    /* Report specific I2C error code based on ChibiOS flags. */
    if ((errors & I2C_BUS_ERROR) != 0u)
    {
        error_report(ErrorCode::I2C_ERR_BUS);
    }
    else if ((errors & I2C_ARBITRATION_LOST) != 0u)
    {
        error_report(ErrorCode::I2C_ERR_ARBITRATION);
    }
    else if ((errors & I2C_ACK_FAILURE) != 0u)
    {
        error_report(ErrorCode::I2C_ERR_NACK);
    }
    else
    {
        error_report(ErrorCode::I2C_ERR_TIMEOUT);
    }

    /* Attempt bus recovery only on bus-level errors (not simple NACKs). */
    if ((errors & (I2C_BUS_ERROR | I2C_ARBITRATION_LOST)) != 0u)
    {
        (void)bus_recovery();
    }
}

}  // namespace acs
