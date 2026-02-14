/*
    ACS4 Flight Computer - HAL Configuration
    Target: STM32H725VGT6 (ACS4 custom PCB)

    Enabled HAL subsystems:
    - PAL       (GPIO for LEDs, CS lines, pyro, arming)
    - SERIAL    (UART4 for GPS)
    - SPI       (SPI2 for IMU/BARO/MAG sensors)
    - USB       (OTG1 FS for CDC debug shell)
    - SERIAL_USB (CDC ACM driver on USB)
*/

#ifndef HALCONF_H
#define HALCONF_H

#define _CHIBIOS_HAL_CONF_
#define _CHIBIOS_HAL_CONF_VER_9_0_

#include "mcuconf.h"

/*---------------------------------------------------------------------------*/
/* HAL subsystem toggles                                                     */
/*---------------------------------------------------------------------------*/

#define HAL_USE_PAL                         TRUE
#define HAL_USE_ADC                         TRUE
#define HAL_USE_CAN                         TRUE
#define HAL_USE_CRY                         FALSE
#define HAL_USE_DAC                         FALSE
#define HAL_USE_EFL                         FALSE
#define HAL_USE_GPT                         FALSE
#define HAL_USE_I2C                         TRUE
#define HAL_USE_I2S                         FALSE
#define HAL_USE_ICU                         FALSE
#define HAL_USE_MAC                         FALSE
#define HAL_USE_MMC_SPI                     FALSE
#define HAL_USE_PWM                         TRUE
#define HAL_USE_RTC                         FALSE
#define HAL_USE_SDC                         TRUE
#define HAL_USE_SERIAL                      TRUE
#define HAL_USE_SERIAL_USB                  TRUE
#define HAL_USE_SIO                         FALSE
#define HAL_USE_SPI                         TRUE
#define HAL_USE_TRNG                        FALSE
#define HAL_USE_UART                        FALSE
#define HAL_USE_USB                         TRUE
#define HAL_USE_WDG                         FALSE
#define HAL_USE_WSPI                        FALSE

/*---------------------------------------------------------------------------*/
/* PAL driver settings                                                       */
/*---------------------------------------------------------------------------*/

#define PAL_USE_CALLBACKS                   FALSE
#define PAL_USE_WAIT                        FALSE

/*---------------------------------------------------------------------------*/
/* SERIAL driver settings                                                    */
/*---------------------------------------------------------------------------*/

#define SERIAL_DEFAULT_BITRATE              115200
#define SERIAL_BUFFERS_SIZE                 256

#endif /* HALCONF_H */
