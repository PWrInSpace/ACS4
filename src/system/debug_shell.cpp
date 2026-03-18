/*
 * ACS4 Flight Computer — Debug Shell Implementation
 *
 * ChibiOS Shell on UART (Nucleo) or USB CDC (custom PCB).
 * Low-priority thread for interactive debugging.
 */

#include "system/debug_shell.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iterator>

extern "C" {
#include "ch.h"

#include "hal.h"

#include <chprintf.h>

#include "shell.h"
}

#include "drivers/iim42653.h"
#include "drivers/mmc5983ma.h"
#include "drivers/ms5611.h"
#include "sensors/sensor_hub.h"

#if defined(STM32H725xx)
    #include "hal/sdmmc.h"
    #include "logger/flight_logger.h"
    #include "logger/ram_log.h"
#endif
#include "system/error_handler.h"
#include "system/params.h"
#include "utils/profiler.h"

/* Wersja o info */

#ifndef ACS4_VERSION
    #define ACS4_VERSION "0.1.0"
#endif

#ifndef ACS4_GIT_HASH
    #define ACS4_GIT_HASH "unknown"
#endif

/* Komendy shellowe */

static void cmd_version(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    chprintf(chp, "ACS4 Flight Computer v%s\r\n", ACS4_VERSION);
    chprintf(chp, "Build: %s %s\r\n", __DATE__, __TIME__);
    chprintf(chp, "Git:   %s\r\n", ACS4_GIT_HASH);
    chprintf(chp, "ChibiOS/RT %s\r\n", CH_KERNEL_VERSION);
    chprintf(chp, "SYSCLK: %lu MHz\r\n", STM32_SYS_CK / 1000000UL);
}

static void cmd_uptime(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    const auto     ms = static_cast<uint32_t>(chTimeI2MS(chVTGetSystemTimeX()));
    const uint32_t s  = ms / 1000;
    const uint32_t m  = s / 60;
    const uint32_t h  = m / 60;
    chprintf(chp, "Uptime: %lu:%02lu:%02lu (%lu ms)\r\n", h, m % 60, s % 60, ms);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    static const char *state_names[] = {"READY",
                                        "CURRENT",
                                        "STARTED",
                                        "SUSPENDED",
                                        "QUEUED",
                                        "WTSEM",
                                        "WTMTX",
                                        "WTCOND",
                                        "SLEEPING",
                                        "WTEXIT",
                                        "WTOREVT",
                                        "WTANDEVT",
                                        "SNDMSGQ",
                                        "SNDMSG",
                                        "WTMSG",
                                        "FINAL"};

    chprintf(chp, "%-16s %4s %10s %s\r\n", "Name", "Prio", "FreeStack", "State");
    chprintf(chp, "----------------------------------------------\r\n");

    thread_t *tp = chRegFirstThread();
    while (tp != nullptr)
    {
        uint32_t stk_free = 0;
#if CH_DBG_FILL_THREADS == TRUE
        auto *begin = reinterpret_cast<uint8_t *>(tp->wabase);
        auto *end   = reinterpret_cast<uint8_t *>(tp);
        while (begin < end && *begin == CH_DBG_STACK_FILL_VALUE)
        {
            begin++;
        }
        stk_free = static_cast<uint32_t>(begin - reinterpret_cast<uint8_t *>(tp->wabase));
#endif

        const char *name = (tp->name != nullptr) ? tp->name : "<unnamed>";
        const auto  idx  = static_cast<unsigned>(tp->state);
        const char *st   = (idx < std::size(state_names)) ? state_names[idx] : "???";

        chprintf(chp,
                 "%-16s %4u %10lu %s\r\n",
                 name,
                 static_cast<unsigned>(tp->hdr.pqueue.prio),
                 stk_free,
                 st);

        tp = chRegNextThread(tp);
    }
}

static void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    chprintf(chp, "Rebooting...\r\n");
    chThdSleepMilliseconds(100);
    NVIC_SystemReset();
}

static void cmd_perf(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    acs::profiler_print(chp);
}

static void cmd_errors(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    acs::error_print(chp);
}

static void cmd_param(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 0)
    {
        chprintf(chp,
                 "Usage: param list | get <name> | set <name> <value> | "
                 "defaults\r\n");
        return;
    }

    if (strcmp(argv[0], "list") == 0)
    {
        acs::param_list(chp);
    }
    else if (strcmp(argv[0], "get") == 0 && argc >= 2)
    {
        float val = 0.0f;
        if (acs::param_get(argv[1], val))
        {
            chprintf(chp, "%s = %.6f\r\n", argv[1], static_cast<double>(val));
        }
        else
        {
            chprintf(chp, "Unknown param: %s\r\n", argv[1]);
        }
    }
    else if (strcmp(argv[0], "set") == 0 && argc >= 3)
    {
        char *endptr = nullptr;
        auto  val    = static_cast<float>(strtod(argv[2], &endptr));
        if (endptr == argv[2])
        {
            chprintf(chp, "Invalid number: %s\r\n", argv[2]);
            return;
        }
        if (acs::param_set(argv[1], val))
        {
            chprintf(chp, "%s = %.6f\r\n", argv[1], static_cast<double>(val));
        }
        else
        {
            chprintf(chp, "Failed (unknown or out of range): %s\r\n", argv[1]);
        }
    }
    else if (strcmp(argv[0], "defaults") == 0)
    {
        acs::param_reset_all();
        chprintf(chp, "All parameters reset to defaults.\r\n");
    }
    else
    {
        chprintf(chp,
                 "Usage: param list | get <name> | set <name> <value> | "
                 "defaults\r\n");
    }
}

static void cmd_sensor_all(BaseSequentialStream *chp)
{
    const acs::SensorSnapshot s = acs::sensor_hub().snapshot();

    chprintf(chp, "--- SensorSnapshot (imu_t=%lu us) ---\r\n", s.imu_timestamp_us);

    chprintf(chp, "IMU  [%s]:\r\n", s.imu_valid ? "OK" : "--");
    chprintf(chp,
             "  Accel [m/s2]: %+.3f  %+.3f  %+.3f\r\n",
             static_cast<double>(s.accel_mps2[0]),
             static_cast<double>(s.accel_mps2[1]),
             static_cast<double>(s.accel_mps2[2]));
    chprintf(chp,
             "  Gyro [rad/s]: %+.4f  %+.4f  %+.4f\r\n",
             static_cast<double>(s.gyro_rads[0]),
             static_cast<double>(s.gyro_rads[1]),
             static_cast<double>(s.gyro_rads[2]));
    chprintf(chp, "  Temp:         %.1f C\r\n", static_cast<double>(s.imu_temp_c));

    chprintf(chp, "BARO [%s] fresh=%s:\r\n", s.baro_valid ? "OK" : "--", s.baro_fresh ? "Y" : "N");
    chprintf(chp, "  Pressure:     %.1f Pa\r\n", static_cast<double>(s.pressure_pa));
    chprintf(chp, "  Altitude:     %.2f m\r\n", static_cast<double>(s.altitude_m));
    chprintf(chp, "  Temp:         %.1f C\r\n", static_cast<double>(s.baro_temp_c));

    const auto mx = static_cast<double>(s.mag_ut[0]);
    const auto my = static_cast<double>(s.mag_ut[1]);
    const auto mz = static_cast<double>(s.mag_ut[2]);
    chprintf(chp, "MAG  [%s] fresh=%s:\r\n", s.mag_valid ? "OK" : "--", s.mag_fresh ? "Y" : "N");
    chprintf(chp,
             "  Field [uT]:   %+.1f  %+.1f  %+.1f  |B|=%.1f\r\n",
             mx,
             my,
             mz,
             sqrt(mx * mx + my * my + mz * mz));
}

static void cmd_sensor_imu(BaseSequentialStream *chp)
{
    auto *imu = acs::imu_instance();
    if (imu == nullptr)
    {
        chprintf(chp, "IMU not available (no hardware or init failed)\r\n");
        return;
    }

    acs::ImuSample sample{};
    if (!imu->read(sample))
    {
        chprintf(chp, "IMU read failed (errors: %lu)\r\n", imu->error_count());
        return;
    }

    chprintf(chp,
             "Accel [m/s2]: X=%+.3f  Y=%+.3f  Z=%+.3f\r\n",
             static_cast<double>(sample.accel_mps2[0]),
             static_cast<double>(sample.accel_mps2[1]),
             static_cast<double>(sample.accel_mps2[2]));
    chprintf(chp,
             "Gyro [rad/s]: X=%+.4f  Y=%+.4f  Z=%+.4f\r\n",
             static_cast<double>(sample.gyro_rads[0]),
             static_cast<double>(sample.gyro_rads[1]),
             static_cast<double>(sample.gyro_rads[2]));
    chprintf(chp, "Temp:         %.1f C\r\n", static_cast<double>(sample.temp_degc));
    chprintf(chp, "Timestamp:    %lu us\r\n", sample.timestamp_us);
    chprintf(chp, "Errors:       %lu\r\n", imu->error_count());
}

static void cmd_sensor_baro(BaseSequentialStream *chp)
{
    auto *baro = acs::baro_instance();
    if (baro == nullptr)
    {
        chprintf(chp, "BARO not available (no hardware or init failed)\r\n");
        return;
    }

    if (!baro->has_new_data())
    {
        chprintf(chp, "BARO: no new data (errors: %lu)\r\n", baro->error_count());
        return;
    }

    const acs::BaroSample s = baro->sample();
    chprintf(chp, "Pressure:    %.2f Pa\r\n", static_cast<double>(s.pressure_pa));
    chprintf(chp, "Temperature: %.2f C\r\n", static_cast<double>(s.temperature_c));
    chprintf(chp, "Altitude:    %.2f m\r\n", static_cast<double>(s.altitude_m));
    chprintf(chp, "Timestamp:   %lu us\r\n", s.timestamp_us);
    chprintf(chp, "Errors:      %lu\r\n", baro->error_count());
}

static void cmd_sensor_mag(BaseSequentialStream *chp)
{
    auto *mag = acs::mag_instance();
    if (mag == nullptr)
    {
        chprintf(chp, "MAG not available (no hardware or init failed)\r\n");
        return;
    }

    acs::MagSample sample{};
    if (!mag->read(sample))
    {
        chprintf(chp, "MAG read failed (errors: %lu)\r\n", mag->error_count());
        return;
    }

    const auto mx = static_cast<double>(sample.field_ut[0]);
    const auto my = static_cast<double>(sample.field_ut[1]);
    const auto mz = static_cast<double>(sample.field_ut[2]);

    chprintf(chp, "Field [uT]: mx=%+.1f  my=%+.1f  mz=%+.1f\r\n", mx, my, mz);
    chprintf(chp, "Magnitude:  %.1f uT\r\n", sqrt(mx * mx + my * my + mz * mz));
    chprintf(chp, "Timestamp:  %lu us\r\n", sample.timestamp_us);
    chprintf(chp, "Errors:     %lu\r\n", mag->error_count());
}

static void cmd_sensor(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 0)
    {
        chprintf(chp, "Usage: sensor imu | baro | mag | all\r\n");
        return;
    }

    if (strcmp(argv[0], "all") == 0)
    {
        cmd_sensor_all(chp);
    }
    else if (strcmp(argv[0], "imu") == 0)
    {
        cmd_sensor_imu(chp);
    }
    else if (strcmp(argv[0], "baro") == 0)
    {
        cmd_sensor_baro(chp);
    }
    else if (strcmp(argv[0], "mag") == 0)
    {
        cmd_sensor_mag(chp);
    }
    else
    {
        chprintf(chp, "Unknown sensor: %s\r\n", argv[0]);
        chprintf(chp, "Available: imu, baro, mag, all\r\n");
    }
}

#if defined(STM32H725xx)

static void cmd_log(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 0)
    {
        chprintf(chp, "Usage: log start | stop | status\r\n");
        return;
    }

    if (strcmp(argv[0], "start") == 0)
    {
        if (!acs::sdmmc_is_mounted())
        {
            chprintf(chp, "SD card not mounted, attempting mount...\r\n");
            if (!acs::sdmmc_mount())
            {
                chprintf(chp, "SD mount FAILED\r\n");
                return;
            }
            chprintf(chp, "SD mounted OK\r\n");
            acs::logger_init();
        }
        if (acs::logger_start())
        {
            const acs::LoggerStats st = acs::logger_stats();
            chprintf(chp, "Logging started -> %s\r\n", st.filename);
        }
        else
        {
            chprintf(chp, "Failed to start logging\r\n");
        }
    }
    else if (strcmp(argv[0], "stop") == 0)
    {
        acs::logger_stop();
        chprintf(chp, "Logging stopped\r\n");
    }
    else if (strcmp(argv[0], "status") == 0)
    {
        acs::logger_print_status(chp);
        acs::ram_log_print_status(chp);
    }
    else
    {
        chprintf(chp, "Usage: log start | stop | status\r\n");
    }
}

static void cmd_sd(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 0)
    {
        chprintf(chp, "Usage: sd status | mount | unmount | ls\r\n");
        return;
    }

    if (strcmp(argv[0], "status") == 0)
    {
        chprintf(chp, "Card inserted: %s\r\n", acs::sdmmc_card_inserted() ? "yes" : "no");
        chprintf(chp, "Mounted:       %s\r\n", acs::sdmmc_is_mounted() ? "yes" : "no");

        if (acs::sdmmc_is_mounted())
        {
            uint32_t total = 0;
            uint32_t free_space = 0;
            if (acs::sdmmc_free_space(total, free_space))
            {
                chprintf(chp, "Capacity:      %lu MiB\r\n", total);
                chprintf(chp, "Free:          %lu MiB\r\n", free_space);
            }
        }
    }
    else if (strcmp(argv[0], "mount") == 0)
    {
        if (acs::sdmmc_mount())
        {
            chprintf(chp, "SD card mounted OK\r\n");
        }
        else
        {
            chprintf(chp, "SD mount FAILED\r\n");
        }
    }
    else if (strcmp(argv[0], "unmount") == 0)
    {
        acs::sdmmc_unmount();
        chprintf(chp, "SD card unmounted\r\n");
    }
    else if (strcmp(argv[0], "ls") == 0)
    {
        if (!acs::sdmmc_is_mounted())
        {
            chprintf(chp, "SD not mounted\r\n");
            return;
        }

        DIR dir;
        FILINFO fno;
        FRESULT res = f_opendir(&dir, "/");
        if (res != FR_OK)
        {
            chprintf(chp, "Failed to open root dir\r\n");
            return;
        }

        chprintf(chp, "%-20s %10s\r\n", "Name", "Size");
        chprintf(chp, "-------------------------------\r\n");

        while (true)
        {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == '\0')
            {
                break;
            }
            chprintf(chp, "%-20s %10lu\r\n", fno.fname, fno.fsize);
        }

        f_closedir(&dir);
    }
    else
    {
        chprintf(chp, "Usage: sd status | mount | unmount | ls\r\n");
    }
}

#endif /* STM32H725xx */

/* Tablica komend shellowych */

static const ShellCommand shell_commands[] = {
    {"version", cmd_version},
    { "uptime",  cmd_uptime},
    {"threads", cmd_threads},
    { "reboot",  cmd_reboot},
    {   "perf",    cmd_perf},
    { "errors",  cmd_errors},
    {  "param",   cmd_param},
    { "sensor",  cmd_sensor},
#if defined(STM32H725xx)
    {    "log",     cmd_log},
    {     "sd",      cmd_sd},
#endif
    {  nullptr,     nullptr}
};

/* Shell thread */

static char shell_history_buf[SHELL_MAX_LINE_LENGTH * 4];

static THD_WORKING_AREA(waShell, 2048);

namespace acs
{

/*  Internal: Zlaunchuj shell thread z podanym streamem  */

static void shell_launch(BaseSequentialStream *stream)
{
    static bool initialized = false;
    if (!initialized)
    {
        shellInit();
        initialized = true;
    }

    static ShellConfig shell_cfg = {
        .sc_channel  = stream,
        .sc_commands = shell_commands,
#if SHELL_USE_HISTORY == TRUE
        .sc_histbuf  = shell_history_buf,
        .sc_histsize = sizeof(shell_history_buf),
#endif
    };

    chThdCreateStatic(waShell, sizeof(waShell), NORMALPRIO - 10, shellThread, &shell_cfg);
}

void shell_start(SerialDriver *serial_driver, uint32_t baudrate)
{
    static SerialConfig serial_cfg = {};
    serial_cfg.speed               = baudrate;
    sdStart(serial_driver, &serial_cfg);

    shell_launch(reinterpret_cast<BaseSequentialStream *>(serial_driver));
}

void shell_start(BaseSequentialStream *stream)
{
    shell_launch(stream);
}

}  // namespace acs
