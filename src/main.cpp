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

#include "actuators/actuator_hub.h"
#include "actuators/actuator_threads.h"
#include "drivers/iim42653.h"
#include "drivers/mmc5983ma.h"
#include "drivers/ms5611.h"
#include "drivers/servo_t75.h"
#include "sensors/sensor_threads.h"
#include "system/debug_shell.h"
#include "system/error_handler.h"
#include "system/watchdog.h"
#include "utils/timestamp.h"

#if defined(STM32H725xx)
    #include "hal/spi_bus.h"
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

/* ── Sensor hardware (custom PCB only — Nucleo has no sensors) ───────────── */
#if defined(STM32H725xx)

static const SPIConfig imu_spi_cfg = {
    .circular = false,
    .slave    = false,
    .data_cb  = nullptr,
    .error_cb = nullptr,
    .cfg1     = SPI_CFG1_MBR_DIV8,             /* kernel_clk / 8 */
    .cfg2     = SPI_CFG2_CPOL | SPI_CFG2_CPHA, /* SPI Mode 3 */
};

static const SPIConfig baro_spi_cfg = {
    .circular = false,
    .slave    = false,
    .data_cb  = nullptr,
    .error_cb = nullptr,
    .cfg1     = SPI_CFG1_MBR_DIV8,             /* kernel_clk / 8 */
    .cfg2     = SPI_CFG2_CPOL | SPI_CFG2_CPHA, /* SPI Mode 3 */
};

static const SPIConfig mag_spi_cfg = {
    .circular = false,
    .slave    = false,
    .data_cb  = nullptr,
    .error_cb = nullptr,
    .cfg1     = SPI_CFG1_MBR_DIV8,             /* kernel_clk / 8 → 6.25 MHz */
    .cfg2     = SPI_CFG2_CPOL | SPI_CFG2_CPHA, /* SPI Mode 3 */
};

static acs::SpiBus       g_spi;
static acs::Iim42653     g_imu;
static acs::Ms5611       g_baro;
static acs::Mmc5983ma    g_mag;
static acs::ServoBankT75 g_servos;

acs::Iim42653 *acs::imu_instance()
{
    return g_imu.is_initialized() ? &g_imu : nullptr;
}

acs::Ms5611 *acs::baro_instance()
{
    return g_baro.is_initialized() ? &g_baro : nullptr;
}

acs::Mmc5983ma *acs::mag_instance()
{
    return g_mag.is_initialized() ? &g_mag : nullptr;
}

acs::ServoBankT75 *acs::servo_bank_instance()
{
    return g_servos.is_initialized() ? &g_servos : nullptr;
}

#else /* NUCLEO — no sensors */

acs::Iim42653 *acs::imu_instance()
{
    return nullptr;
}

acs::Ms5611 *acs::baro_instance()
{
    return nullptr;
}

acs::Mmc5983ma *acs::mag_instance()
{
    return nullptr;
}

acs::ServoBankT75 *acs::servo_bank_instance()
{
    return nullptr;
}

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

    /* Initialize sensor SPI bus and IMU (custom PCB only). */
#if defined(STM32H725xx)
    if (g_spi.init(&SPID2))
    {
        if (g_imu.init(g_spi, LINE_IMU_CS, imu_spi_cfg))
        {
            if (g_imu.configure(acs::Iim42653Config::rocket_default()))
            {
                chprintf(serial, "IMU: IIM-42653 OK\r\n");
            }
            else
            {
                chprintf(serial, "IMU: configure FAILED\r\n");
            }
        }
        else
        {
            chprintf(serial, "IMU: init FAILED\r\n");
        }

        if (g_baro.init(g_spi, LINE_BARO_CS, baro_spi_cfg, acs::Ms5611Config::rocket_default()))
        {
            chprintf(serial, "BARO: MS5611 OK\r\n");
        }
        else
        {
            chprintf(serial, "BARO: init FAILED\r\n");
        }

        if (g_mag.init(g_spi, LINE_MAG_CS, mag_spi_cfg))
        {
            if (g_mag.configure(acs::Mmc5983maConfig::rocket_default()))
            {
                chprintf(serial, "MAG: MMC5983MA OK\r\n");
            }
            else
            {
                chprintf(serial, "MAG: configure FAILED\r\n");
            }
        }
        else
        {
            chprintf(serial, "MAG: init FAILED\r\n");
        }
    }
    else
    {
        chprintf(serial, "SPI bus: init FAILED\r\n");
    }

    /* TIM4 PWM bank for 4 canard ailerons (PD12..PD15). Boots disarmed:
     * pins LOW until the flight FSM (or `servo arm` shell cmd) issues arm. */
    if (g_servos.init(&PWMD4, acs::ServoT75Config::rocket_default()))
    {
        chprintf(serial, "SERVOS: TIM4 OK (disarmed)\r\n");
    }
    else
    {
        chprintf(serial, "SERVOS: init FAILED\r\n");
    }
#endif

    /* Start sensor acquisition threads (custom PCB only — no-op on Nucleo). */
    acs::start_sensor_threads();

    /* Start actuator write-out thread (custom PCB only — no-op on Nucleo). */
    acs::start_actuator_threads();

    /* Create worker threads. */
    chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, Blinker, nullptr);

    /* main() becomes the idle thread. */
    while (true)
    {
        chThdSleepMilliseconds(1000);
    }
}
