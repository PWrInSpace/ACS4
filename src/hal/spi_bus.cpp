/*
 * ACS4 Flight Computer — SPI Bus Abstraction (Implementation)
 *
 * ChibiOS SPIv3 LLD (v2 high-level driver) wrapper with:
 *   - DMA transfers via blocking spiExchange / spiSend / spiReceive
 *   - Per-device SPIConfig (different CPOL/CPHA/prescaler per slave)
 *   - Manual CS management via PAL (SPI_SELECT_MODE_NONE)
 *   - Mutex protection (spiAcquireBus / spiReleaseBus)
 *
 * See spi_bus.h for API documentation.
 *
 * ChibiOS v2 SPI driver notes:
 *   - Blocking spiExchange() requires SPI_USE_SYNCHRONIZATION = TRUE
 *   - spiStart() is called per-transfer with the device-specific config
 *   - CS is managed manually via PAL because SPI_SELECT_MODE = NONE
 *   - STM32_SPI_FILLER_PATTERN defaults to 0xFFFFFFFF (sends 0xFF when
 *     no TX buffer is provided)
 */

#include "hal/spi_bus.h"

#include <cstring>

#include "system/error_handler.h"

extern "C" {
#include "ch.h"
}

namespace acs
{

/* ── Init ─────────────────────────────────────────────────────────────── */

bool SpiBus::init(SPIDriver *driver)
{
    if (driver == nullptr)
    {
        return false;
    }

    driver_      = driver;
    error_count_ = 0;
    initialized_ = true;

    return true;
}

/* ── Full-duplex exchange ─────────────────────────────────────────────── */

bool SpiBus::transfer(ioline_t         cs_line,
                      const uint8_t   *tx,
                      uint8_t         *rx,
                      size_t           len,
                      const SPIConfig &config)
{
    if (!initialized_ || len == 0)
    {
        return false;
    }

    const BusGuard bus(driver_, &config);
    const CsGuard  cs(cs_line);

    /*
     * spiExchange (v2 blocking API):
     *   - Performs simultaneous TX+RX via DMA.
     *   - If tx is nullptr, ChibiOS sends STM32_SPI_FILLER_PATTERN (0xFF).
     *   - If rx is nullptr, received bytes are discarded internally.
     *   - Returns MSG_OK on success, MSG_TIMEOUT / MSG_RESET on failure.
     */
    const msg_t status = spiExchange(driver_, len, tx, rx);

    if (status != MSG_OK)
    {
        handle_error(status == MSG_TIMEOUT);
        return false;
    }

    return true;
}

/* ── TX-only ──────────────────────────────────────────────────────────── */

bool SpiBus::send(ioline_t         cs_line,
                  const uint8_t   *tx,
                  size_t           len,
                  const SPIConfig &config)
{
    if (!initialized_ || tx == nullptr || len == 0)
    {
        return false;
    }

    const BusGuard bus(driver_, &config);
    const CsGuard  cs(cs_line);

    const msg_t status = spiSend(driver_, len, tx);

    if (status != MSG_OK)
    {
        handle_error(status == MSG_TIMEOUT);
        return false;
    }

    return true;
}

/* ── RX-only ──────────────────────────────────────────────────────────── */

bool SpiBus::receive(ioline_t         cs_line,
                     uint8_t         *rx,
                     size_t           len,
                     const SPIConfig &config)
{
    if (!initialized_ || rx == nullptr || len == 0)
    {
        return false;
    }

    const BusGuard bus(driver_, &config);
    const CsGuard  cs(cs_line);

    const msg_t status = spiReceive(driver_, len, rx);

    if (status != MSG_OK)
    {
        handle_error(status == MSG_TIMEOUT);
        return false;
    }

    return true;
}

/* ── Read single register ─────────────────────────────────────────────── */

std::optional<uint8_t>
SpiBus::read_register(ioline_t cs_line, uint8_t reg, const SPIConfig &config)
{
    /*
     * Standard SPI sensor register read:
     *   TX: [reg | 0x80] [0x00]
     *   RX: [  junk    ] [data]
     *
     * The first RX byte is junk (slave needs a clock cycle to fetch
     * the register). The second byte is the register value.
     */
    uint8_t tx_buf[2] = {static_cast<uint8_t>(reg | 0x80U), 0x00U};
    uint8_t rx_buf[2] = {0, 0};

    if (!transfer(cs_line, tx_buf, rx_buf, 2, config))
    {
        return std::nullopt;
    }

    return rx_buf[1];
}

/* ── Write single register ────────────────────────────────────────────── */

bool SpiBus::write_register(ioline_t         cs_line,
                            uint8_t          reg,
                            uint8_t          value,
                            const SPIConfig &config)
{
    /*
     * Standard SPI sensor register write:
     *   TX: [reg & 0x7F] [value]
     *   RX: (ignored)
     */
    uint8_t tx_buf[2] = {static_cast<uint8_t>(reg & 0x7FU), value};

    return send(cs_line, tx_buf, 2, config);
}

/* ── Burst register read ──────────────────────────────────────────────── */

bool SpiBus::read_registers(ioline_t         cs_line,
                            uint8_t          reg,
                            uint8_t         *buf,
                            size_t           len,
                            const SPIConfig &config)
{
    if (!initialized_ || buf == nullptr || len == 0)
    {
        return false;
    }

    /*
     * Burst register read:
     *   TX: [reg | 0x80] [0x00] [0x00] ... (len+1 bytes total)
     *   RX: [  junk    ] [d0 ] [d1  ] ... (len data bytes)
     *
     * We use a stack buffer for small reads (up to 32 data bytes,
     * which covers all sensor register blocks). For larger reads
     * the caller should use transfer() directly.
     */
    static constexpr size_t kMaxBurst = 32;

    if (len > kMaxBurst)
    {
        return false;
    }

    uint8_t tx_buf[kMaxBurst + 1] = {};
    uint8_t rx_buf[kMaxBurst + 1] = {};

    tx_buf[0] = static_cast<uint8_t>(reg | 0x80U);
    /* Remaining tx bytes are already zero-initialized. */

    if (!transfer(cs_line, tx_buf, rx_buf, len + 1, config))
    {
        return false;
    }

    /* Skip the first junk byte. */
    std::memcpy(buf, &rx_buf[1], len);

    return true;
}

/* ── Error handling ───────────────────────────────────────────────────── */

void SpiBus::handle_error(bool timeout)
{
    error_count_++;

    if (timeout)
    {
        error_report(ErrorCode::SPI_ERR_TIMEOUT);
    }
    else
    {
        error_report(ErrorCode::SPI_ERR_DMA);
    }
}

}  // namespace acs
