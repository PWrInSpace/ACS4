/*
 * ACS4 Flight Computer — SDMMC + FatFs Implementation
 *
 * Uses ChibiOS SDC driver on SDMMC1 (4-bit mode) with FatFs.
 */

#include "hal/sdmmc.h"

#include "system/error_handler.h"

extern "C" {
#include "ch.h"
}

namespace acs
{

static FATFS s_fs;
static bool  s_mounted = false;

bool sdmmc_init()
{
    sdcStart(&SDCD1, nullptr);
    return true;
}

bool sdmmc_mount()
{
    if (s_mounted)
    {
        return true;
    }

    if (!sdmmc_card_inserted())
    {
        return false;
    }

    if (sdcConnect(&SDCD1) != HAL_SUCCESS)
    {
        error_report(ErrorCode::SD_MOUNT_FAIL);
        return false;
    }

    FRESULT res = f_mount(&s_fs, "/", 1);
    if (res != FR_OK)
    {
        sdcDisconnect(&SDCD1);
        error_report(ErrorCode::SD_MOUNT_FAIL);
        return false;
    }

    s_mounted = true;
    return true;
}

void sdmmc_unmount()
{
    if (!s_mounted)
    {
        return;
    }

    f_mount(nullptr, "/", 0);
    sdcDisconnect(&SDCD1);
    s_mounted = false;
}

bool sdmmc_card_inserted()
{
    return blkIsInserted(&SDCD1);
}

bool sdmmc_is_mounted()
{
    return s_mounted;
}

bool sdmmc_free_space(uint32_t &total_mb, uint32_t &free_mb)
{
    if (!s_mounted)
    {
        return false;
    }

    FATFS  *fs   = nullptr;
    DWORD   fre  = 0;
    FRESULT res  = f_getfree("/", &fre, &fs);
    if (res != FR_OK)
    {
        return false;
    }

    uint32_t sector_size  = 512;
    uint32_t cluster_size = static_cast<uint32_t>(fs->csize) * sector_size;

    total_mb = static_cast<uint32_t>((static_cast<uint64_t>(fs->n_fatent - 2) * cluster_size) >> 20U);
    free_mb  = static_cast<uint32_t>((static_cast<uint64_t>(fre) * cluster_size) >> 20U);

    return true;
}

FATFS *sdmmc_fatfs()
{
    return s_mounted ? &s_fs : nullptr;
}

}  // namespace acs
