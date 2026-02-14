/*
 * ACS4 Flight Computer — Main Entry Point
 *
 * Boot sequence:
 *   1. HAL + RTOS init
 *   2. DWT timestamp init
 *   3. Watchdog init (software + IWDG)
 *   4. Debug shell (USB CDC on custom PCB, UART3 on Nucleo)
 *   5. Worker threads (blinker)
 *
 * NUCLEO-H723ZG (dev):
 *   LED1 (green) — PB0,  LED3 (red) — PB14
 *   USART3 (ST-Link VCP): TX=PD8, RX=PD9
 *
 * ACS4 custom PCB (prod):
 *   LED_1..LED_4 — PA1..PA3, PA7
 *   USB CDC on PA11/PA12 (OTG_HS in FS mode)
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

#if defined(STM32H725xx)
    #include "system/usb_cdc.h"
#endif

/* ── Board-specific LED aliases ──────────────────────────────────────────── */
#if defined(STM32H725xx)
    #define LINE_STATUS LINE_LED_1
    #define LINE_ERROR  LINE_LED_3
#else
    #define LINE_STATUS LINE_LED1
    #define LINE_ERROR  LINE_LED3
#endif

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
        palSetLine(LINE_STATUS);
        chThdSleepMilliseconds(100);
        palClearLine(LINE_STATUS);

        palSetLine(LINE_ERROR);
        chThdSleepMilliseconds(100);
        palClearLine(LINE_ERROR);

        chThdSleepMilliseconds(800);
    }
}

/*---------------------------------------------------------------------------*/
/* Application entry point                                                   */
/*---------------------------------------------------------------------------*/

int main()
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

    /* Start debug shell.
     * Custom PCB: USB CDC on PA11/PA12 (OTG_HS in FS mode).
     * Nucleo:     USART3 at 921600 baud via ST-Link VCP. */
#if defined(STM32H725xx)
    acs::usb_cdc_init();
    auto *serial = acs::usb_cdc_stream();
    acs::shell_start(serial);
#else
    acs::shell_start(&SD3, 921600);
    auto *serial = reinterpret_cast<BaseSequentialStream *>(&SD3);
#endif
    chprintf(serial, "\r\n");
    chprintf(serial, "========================================\r\n");
    chprintf(serial, "  ACS4 Flight Computer\r\n");
    chprintf(serial, "  ChibiOS/RT %s\r\n", CH_KERNEL_VERSION);
    chprintf(serial, "  System clock: %lu MHz\r\n", STM32_SYS_CK / 1000000UL);
    chprintf(serial, "========================================\r\n");
    chprintf(serial, "\r\n");

    /* Create worker threads. */
    chThdCreateStatic(waBlinker,
                      sizeof(waBlinker),
                      NORMALPRIO,
                      Blinker,
                      nullptr);

    /* main() becomes the idle thread. */
    while (true)
    {
        chThdSleepMilliseconds(1000);
    }
}
