/*
    ACS4 Flight Computer - HAL Configuration
    Target: STM32H723ZG (NUCLEO-H723ZG dev board)

    Enable only the HAL subsystems we need for the initial bring-up:
    - PAL (GPIO for LEDs)
    - SERIAL (USART3 for debug via ST-Link Virtual COM Port)
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
#define HAL_USE_ADC                         FALSE
#define HAL_USE_CAN                         FALSE
#define HAL_USE_CRY                         FALSE
#define HAL_USE_DAC                         FALSE
#define HAL_USE_EFL                         FALSE
#define HAL_USE_GPT                         FALSE
#define HAL_USE_I2C                         FALSE
#define HAL_USE_I2S                         FALSE
#define HAL_USE_ICU                         FALSE
#define HAL_USE_MAC                         FALSE
#define HAL_USE_MMC_SPI                     FALSE
#define HAL_USE_PWM                         FALSE
#define HAL_USE_RTC                         FALSE
#define HAL_USE_SDC                         FALSE
#define HAL_USE_SERIAL                      TRUE
#define HAL_USE_SERIAL_USB                  FALSE
#define HAL_USE_SIO                         FALSE
#define HAL_USE_SPI                         FALSE
#define HAL_USE_TRNG                        FALSE
#define HAL_USE_UART                        FALSE
#define HAL_USE_USB                         FALSE
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
