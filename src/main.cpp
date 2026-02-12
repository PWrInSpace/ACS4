/*
 * ACS4 Flight Computer — Main Entry Point
 *
 * Boot sequence:
 *   1. HAL + RTOS init
 *   2. DWT timestamp init
 *   3. Watchdog init (software + IWDG)
 *   4. Debug shell on UART3 (921600 baud)
 *   5. Worker threads (blinker)
 *
 * Board LEDs (active HIGH, NUCLEO-H723ZG):
 *   LED1 (green)  — PB0
 *   LED3 (red)    — PB14
 *
 * USART3 (ST-Link VCP):
 *   TX — PD8, RX — PD9
 */

extern "C" {
#include "ch.h"

#include "hal.h"

#include <chprintf.h>
}

#include "system/debug_shell.h"
#include "system/error_handler.h"
#include "system/watchdog.h"
#include "utils/timestamp.h"

/*---------------------------------------------------------------------------*/
/* LED blinker thread                                                        */
/*---------------------------------------------------------------------------*/

static THD_WORKING_AREA(waBlinker, 256);

static THD_FUNCTION(Blinker, arg)
{
    (void)arg;
    chRegSetThreadName("blinker");

    while (true)
    {
        palSetLine(LINE_LED1);
        chThdSleepMilliseconds(100);
        palClearLine(LINE_LED1);

        palSetLine(LINE_LED3);
        chThdSleepMilliseconds(100);
        palClearLine(LINE_LED3);

        chThdSleepMilliseconds(800);
    }
}

/*---------------------------------------------------------------------------*/
/* Application entry point                                                   */
/*---------------------------------------------------------------------------*/

int main(void)
{
    /* HAL initialization, this also initializes the configured device
     * drivers and performs the board-specific initializations. */
    halInit();

    /* Kernel initialization, the main() function becomes a thread and
     * the RTOS is active. */
    chSysInit();

    /* Enable DWT cycle counter for microsecond timestamps. */
    acs::timestamp_init();

    /* Start software + hardware watchdog. */
    acs::watchdog_init();

    /* Start debug shell on UART3 (ST-Link VCP) at 921600 baud.
     * This also calls sdStart(&SD3, ...) internally. */
    acs::shell_start(&SD3, 921600);

    /* Welcome banner on shell serial port. */
    auto *serial = reinterpret_cast<BaseSequentialStream *>(&SD3);
    chprintf(serial, "\r\n");
    chprintf(serial, "========================================\r\n");
    chprintf(serial, "  ACS4 Flight Computer\r\n");
    chprintf(serial, "  ChibiOS/RT %s\r\n", CH_KERNEL_VERSION);
    chprintf(serial, "  System clock: %lu MHz\r\n", STM32_SYS_CK / 1000000UL);
    chprintf(serial, "========================================\r\n");
    chprintf(serial, "\r\n");

    /* Create worker threads. */
    chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, Blinker, nullptr);

    /* main() becomes the idle thread. */
    while (true)
    {
        chThdSleepMilliseconds(1000);
    }
}
