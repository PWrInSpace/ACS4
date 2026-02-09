/*
 * ACS4 Flight Computer — Proof of Life
 *
 * Blinks the 3 LEDs on the NUCLEO-H723ZG and prints to USART3
 * (ST-Link Virtual COM Port, directly accessible via USB).
 *
 * Board LEDs (active HIGH):
 *   LED1 (green)  — PB0
 *   LED2 (yellow) — PE1
 *   LED3 (red)    — PB14
 *
 * USART3 (ST-Link VCP):
 *   TX — PD8
 *   RX — PD9
 */

extern "C"
{
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
}

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
/* Heartbeat thread — prints to USART3 periodically                          */
/*---------------------------------------------------------------------------*/

static THD_WORKING_AREA(waHeartbeat, 512);

static THD_FUNCTION(Heartbeat, arg)
{
    (void)arg;
    chRegSetThreadName("heartbeat");

    auto *serial  = reinterpret_cast<BaseSequentialStream *>(&SD3);
    uint32_t              counter = 0;

    while (true)
    {
        systime_t now = chVTGetSystemTimeX();
        uint32_t  ms  = (uint32_t)chTimeI2MS(now);

        chprintf(serial, "[%8lu ms] ACS4 heartbeat #%lu\r\n", ms, counter++);
        chThdSleepMilliseconds(1000);
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

    /* Activate USART3 (ST-Link Virtual COM Port).
     * Default config: 115200-8-N-1. */
    sdStart(&SD3, NULL);

    /* Welcome banner. */
    auto *serial = reinterpret_cast<BaseSequentialStream *>(&SD3);
    chprintf(serial, "\r\n");
    chprintf(serial, "========================================\r\n");
    chprintf(serial, "  ACS4 Flight Computer\r\n");
    chprintf(serial, "  Target: NUCLEO-H723ZG (Cortex-M7)\r\n");
    chprintf(serial, "  ChibiOS/RT %s\r\n", CH_KERNEL_VERSION);
    chprintf(serial, "  System clock: %lu MHz\r\n", STM32_SYS_CK / 1000000UL);
    chprintf(serial, "========================================\r\n");
    chprintf(serial, "\r\n");

    /* Create worker threads. */
    chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, Blinker, NULL);
    chThdCreateStatic(waHeartbeat,
                      sizeof(waHeartbeat),
                      NORMALPRIO + 1,
                      Heartbeat,
                      NULL);

    /* main() becomes the idle thread. */
    while (true)
    {
        chThdSleepMilliseconds(1000);
    }
}
