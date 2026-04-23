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

#include "actuators/actuator_hub.h"
#include "drivers/iim42653.h"
#include "drivers/mmc5983ma.h"
#include "drivers/ms5611.h"
#include "drivers/servo_t75.h"
#include "sensors/sensor_hub.h"
#include "system/error_handler.h"
#include "system/params.h"
#include "utils/profiler.h"
#include "utils/timestamp.h"

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

/* Servo bank command (4-aileron canard).
 *
 * Public-facing fin numbering is 1..4 (matches PCB silkscreen CH1..CH4_PWM).
 * Internally fins are 0..3 — convert at the shell boundary.
 */

static bool parse_fin_arg(BaseSequentialStream *chp, const char *arg, uint8_t &fin_idx)
{
    char *endptr = nullptr;
    long  val    = strtol(arg, &endptr, 10);
    if (endptr == arg || val < 1 || val > 4)
    {
        chprintf(chp, "Invalid fin: %s (expected 1..4)\r\n", arg);
        return false;
    }
    fin_idx = static_cast<uint8_t>(val - 1);
    return true;
}

static void cmd_servo_status(BaseSequentialStream *chp)
{
    auto *bank = acs::servo_bank_instance();
    if (bank == nullptr)
    {
        chprintf(chp, "SERVOS not available (no hardware or init failed)\r\n");
        return;
    }

    const acs::ActuatorSnapshot snap = acs::actuator_hub().snapshot();
    const bool                  armed = bank->is_armed();

    chprintf(chp,
             "Servo bank: %s (req=%s)\r\n",
             armed ? "ARMED" : "DISARMED",
             snap.armed_request ? "arm" : "disarm");
    chprintf(chp, "%-4s %-10s %-10s %-10s %-6s %s\r\n",
             "fin", "cmd_deg", "target_us", "current_us", "ch", "sweep");
    for (uint8_t i = 0; i < acs::kAileronCount; ++i)
    {
        chprintf(chp,
                 "%-4u %+10.2f %10u %10u %-6u %s\r\n",
                 static_cast<unsigned>(i + 1),
                 static_cast<double>(snap.aileron_cmd_deg[i]),
                 static_cast<unsigned>(bank->target_pulse_us(i)),
                 static_cast<unsigned>(snap.aileron_current_us[i]),
                 static_cast<unsigned>(acs::ServoBankT75::timer_channel_for_fin(i)),
                 snap.sweep[i].active ? "Y" : "N");
    }
}

static void cmd_servo_set(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc < 3)
    {
        chprintf(chp, "Usage: servo set <fin 1..4> <deg>\r\n");
        return;
    }

    uint8_t fin = 0;
    if (!parse_fin_arg(chp, argv[1], fin))
    {
        return;
    }

    char *endptr = nullptr;
    auto  deg    = static_cast<float>(strtod(argv[2], &endptr));
    if (endptr == argv[2])
    {
        chprintf(chp, "Invalid angle: %s\r\n", argv[2]);
        return;
    }

    acs::actuator_hub().set_aileron_deg(fin, deg, acs::timestamp_us());
    chprintf(chp, "fin %u: cmd %.2f deg\r\n", static_cast<unsigned>(fin + 1), static_cast<double>(deg));
}

static void cmd_servo_us(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc < 3)
    {
        chprintf(chp, "Usage: servo us <fin 1..4> <pulse_us>\r\n");
        return;
    }

    auto *bank = acs::servo_bank_instance();
    if (bank == nullptr)
    {
        chprintf(chp, "SERVOS not available\r\n");
        return;
    }

    uint8_t fin = 0;
    if (!parse_fin_arg(chp, argv[1], fin))
    {
        return;
    }

    char *endptr = nullptr;
    long  us     = strtol(argv[2], &endptr, 10);
    if (endptr == argv[2] || us < 0 || us > 0xFFFF)
    {
        chprintf(chp, "Invalid pulse: %s\r\n", argv[2]);
        return;
    }

    /* Bench override: stop any sweep on this fin, write directly to bank. */
    acs::actuator_hub().set_sweep(fin, false, 0.0f, 0.0f, 0, 0);
    bank->set_pulse_us(fin, static_cast<uint16_t>(us));
    chprintf(chp,
             "fin %u: target %u us (after hardware clamp)\r\n",
             static_cast<unsigned>(fin + 1),
             static_cast<unsigned>(bank->target_pulse_us(fin)));
}

static void cmd_servo_sweep(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc < 5)
    {
        chprintf(chp, "Usage: servo sweep <fin 1..4> <min_deg> <max_deg> <period_ms>\r\n");
        chprintf(chp, "       servo sweep <fin 1..4> off\r\n");
        return;
    }

    uint8_t fin = 0;
    if (!parse_fin_arg(chp, argv[1], fin))
    {
        return;
    }

    if (strcmp(argv[2], "off") == 0)
    {
        acs::actuator_hub().set_sweep(fin, false, 0.0f, 0.0f, 0, 0);
        chprintf(chp, "fin %u: sweep off\r\n", static_cast<unsigned>(fin + 1));
        return;
    }

    char *e1 = nullptr;
    char *e2 = nullptr;
    char *e3 = nullptr;
    auto  min_deg   = static_cast<float>(strtod(argv[2], &e1));
    auto  max_deg   = static_cast<float>(strtod(argv[3], &e2));
    long  period_ms = strtol(argv[4], &e3, 10);
    if (e1 == argv[2] || e2 == argv[3] || e3 == argv[4] || period_ms <= 0)
    {
        chprintf(chp, "Invalid sweep arguments\r\n");
        return;
    }

    const auto now_ms = static_cast<uint32_t>(chTimeI2MS(chVTGetSystemTimeX()));
    acs::actuator_hub().set_sweep(
        fin, true, min_deg, max_deg, static_cast<uint32_t>(period_ms), now_ms);
    chprintf(chp,
             "fin %u: sweep %.2f..%.2f deg, period %ld ms\r\n",
             static_cast<unsigned>(fin + 1),
             static_cast<double>(min_deg),
             static_cast<double>(max_deg),
             period_ms);
}

static void cmd_servo(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 0)
    {
        chprintf(chp,
                 "Usage: servo status | arm | disarm | set <fin> <deg> | "
                 "us <fin> <us> | sweep <fin> <min> <max> <period_ms>\r\n");
        return;
    }

    if (strcmp(argv[0], "status") == 0)
    {
        cmd_servo_status(chp);
    }
    else if (strcmp(argv[0], "arm") == 0)
    {
        acs::actuator_hub().set_armed_request(true);
        chprintf(chp, "Arm request sent.\r\n");
    }
    else if (strcmp(argv[0], "disarm") == 0)
    {
        acs::actuator_hub().set_armed_request(false);
        chprintf(chp, "Disarm request sent.\r\n");
    }
    else if (strcmp(argv[0], "set") == 0)
    {
        cmd_servo_set(chp, argc, argv);
    }
    else if (strcmp(argv[0], "us") == 0)
    {
        cmd_servo_us(chp, argc, argv);
    }
    else if (strcmp(argv[0], "sweep") == 0)
    {
        cmd_servo_sweep(chp, argc, argv);
    }
    else
    {
        chprintf(chp, "Unknown servo subcommand: %s\r\n", argv[0]);
    }
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
    {  "servo",   cmd_servo},
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
