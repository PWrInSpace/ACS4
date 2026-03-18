/*
 * ACS4 Flight Computer — SDMMC + FatFs Interface
 *
 * Wraps ChibiOS SDC driver (SDMMC1, 4-bit) and FatFs filesystem.
 * Provides mount/unmount, card detect, and free-space queries.
 *
 * Hardware: SDMMC1 on PC8-PC12 (D0-D3, CLK), PD2 (CMD), PA15 (DETECT_SD).
 */

#pragma once

#include <cstdint>

extern "C" {
#include "hal.h"

#include "ff.h"
}

namespace acs
{

/**
 * @brief Initialize SDMMC1 peripheral and start the SDC driver.
 * @return true if the SDC driver started successfully.
 */
bool sdmmc_init();

/**
 * @brief Mount the FatFs filesystem (must call sdmmc_init first).
 * @return true if the card was connected and filesystem mounted.
 */
bool sdmmc_mount();

/**
 * @brief Unmount the FatFs filesystem and disconnect the card.
 */
void sdmmc_unmount();

/**
 * @brief Check if a card is physically inserted (detect pin).
 */
[[nodiscard]] bool sdmmc_card_inserted();

/**
 * @brief Check if the filesystem is currently mounted and usable.
 */
[[nodiscard]] bool sdmmc_is_mounted();

/**
 * @brief Query free space on the mounted card.
 * @param[out] total_mb  Total card capacity in MiB.
 * @param[out] free_mb   Free space in MiB.
 * @return true on success.
 */
bool sdmmc_free_space(uint32_t &total_mb, uint32_t &free_mb);

/**
 * @brief Get the internal FatFs filesystem object (for direct f_open etc).
 */
[[nodiscard]] FATFS *sdmmc_fatfs();

}  // namespace acs
