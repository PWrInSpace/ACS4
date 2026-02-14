/*
 * ACS4 Flight Computer — Debug Shell Implementation
 *
 * ChibiOS Shell on UART (Nucleo) or USB CDC (custom PCB).
 * Low-priority thread for interactive debugging.
 */

#include "system/debug_shell.h"

#include <cstdlib>

extern "C" {
#include "ch.h"

#include "hal.h"

#include <chprintf.h>

#include "shell.h"
}

#include "system/error_handler.h"
#include "system/params.h"
#include "utils/profiler.h"

/* ── Version info ─────────────────────────────────────────────────────── */

#ifndef ACS4_VERSION
    #define ACS4_VERSION "0.1.0"
#endif

#ifndef ACS4_GIT_HASH
    #define ACS4_GIT_HASH "unknown"
#endif

/* ── Shell commands ───────────────────────────────────────────────────── */

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
    const auto ms = static_cast<uint32_t>(chTimeI2MS(chVTGetSystemTimeX()));
    const uint32_t s = ms / 1000;
    const uint32_t m = s / 60;
    const uint32_t h = m / 60;
    chprintf(chp,
             "Uptime: %lu:%02lu:%02lu (%lu ms)\r\n",
             h,
             m % 60,
             s % 60,
             ms);
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

    chprintf(chp,
             "%-16s %4s %6s %10s %s\r\n",
             "Name",
             "Prio",
             "Stack",
             "FreeStack",
             "State");
    chprintf(chp, "------------------------------------------------------\r\n");

    thread_t *tp = chRegFirstThread();
    while (tp != nullptr)
    {
        uint32_t stk_free = 0;
#if CH_DBG_FILL_THREADS == TRUE
        auto *begin = reinterpret_cast<uint8_t *>(tp->wabase);
        auto *end   = reinterpret_cast<uint8_t *>(tp + 1);
        while (begin < end && *begin == CH_DBG_STACK_FILL_VALUE)
        {
            begin++;
        }
        stk_free = static_cast<uint32_t>(
            begin - reinterpret_cast<uint8_t *>(tp->wabase));
#endif

        const char *name = (tp->name != nullptr) ? tp->name : "<unnamed>";
        const auto  idx  = static_cast<unsigned>(tp->state);
        const char *st   = (idx < sizeof(state_names) / sizeof(state_names[0]))
                               ? state_names[idx]
                               : "???";

        chprintf(chp,
                 "%-16s %4u %6s %10lu %s\r\n",
                 name,
                 static_cast<unsigned>(tp->hdr.pqueue.prio),
                 "---",
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
        auto val = static_cast<float>(strtod(argv[2], nullptr));
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

/* ── Shell command table ──────────────────────────────────────────────── */

static const ShellCommand shell_commands[] = {
    {"version", cmd_version},
    { "uptime",  cmd_uptime},
    {"threads", cmd_threads},
    { "reboot",  cmd_reboot},
    {   "perf",    cmd_perf},
    { "errors",  cmd_errors},
    {  "param",   cmd_param},
    {  nullptr,     nullptr}
};

/* ── Shell thread ─────────────────────────────────────────────────────── */

static char shell_history_buf[SHELL_MAX_LINE_LENGTH * 4];

static THD_WORKING_AREA(waShell, 2048);

namespace acs
{

/* ── Internal: launch shell thread with a given stream ─────────────────── */

static void shell_launch(BaseSequentialStream *stream)
{
    shellInit();

    static ShellConfig shell_cfg = {
        .sc_channel  = stream,
        .sc_commands = shell_commands,
#if SHELL_USE_HISTORY == TRUE
        .sc_histbuf  = shell_history_buf,
        .sc_histsize = sizeof(shell_history_buf),
#endif
    };

    chThdCreateStatic(waShell,
                      sizeof(waShell),
                      NORMALPRIO - 10,
                      shellThread,
                      &shell_cfg);
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
